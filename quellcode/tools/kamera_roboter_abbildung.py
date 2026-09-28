#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""KAMERA -> ROBOTER-Abbildung im Arbeitsbereich messen (ChArUco am Greifer) und fitten.

WARUM (2026-09-13 Nacht): drei Griffversuche aus der Schale gingen daneben, obwohl die Kamera
richtig misst (Gitter 10 mm = 10.0 px, Schale/Platte deckungsgleich). Der Roboter faehrt im
MODELLRAUM (URDF-FK), der im Arbeitsbereich ~10 % / 1-2 cm neben der Kamerawelt liegt (die
angelernten Nester liegen per FK neben den echten Loechern). Ein konstanter Versatz reichte
nicht (zweimal falsch geraten). Deshalb wird die Abbildung GEMESSEN:

    p_modell = A * p_kamera + b        (affin, 12 Parameter, kleinste Quadrate)

Messung: das ChArUco-Board (6x3, charuco_params_gripper.yaml) sitzt am Greifer, in der
Greifstellung (tcp +Y nach unten) zeigt das Muster nach OBEN zur Kamera. Der Arm faehrt
per MoveIt (kollisionsgeprueft: Schale, Trichter, Ablageplatte, Sockel, Board im Modell)
eine Liste von Stellungen ueber Schale und Ablageplatte in 30-100 mm Hoehe ab. Je
Stellung nach Stillstand: TF robot_base -> charuco_muster (MODELL, FK) und
robot_base -> charuco_gemessen (KAMERA, Detektor). Beide beschreiben dieselbe Musterecke.

SICHERHEIT (Benutzer): alle Zielpunkte werden VORHER geprueft (--nur-pruefen): IK in der
bekannten Haltung, /check_state_validity mit Board, innerhalb der Kaiser-Grundplatte
(x -100..400, y -325..75, Rand 30 mm), Abstand zum Trichterfuss >= 90 mm (unter z 140),
r 140-230. Gefahren wird erst mit --fahren, jede Stellung per MoveIt.

    python3 tools/kamera_roboter_abbildung.py --nur-pruefen
    python3 tools/kamera_roboter_abbildung.py --fahren [--json kamera_roboter_abbildung.json]

Voraussetzungen: Stack mit CHARUCO_MONTIERT=1 KAMERA_PROFIL=1280 (12-mm-Marker), Detektor:
    ros2 run mycobot_calibration charuco_detector --ros-args \\
      --params-file src/mycobot_calibration/config/charuco_params_gripper.yaml \\
      -p board_frame:=charuco_gemessen -p min_corners:=8
Ergebnis: kamera_roboter_abbildung.json {A, b, rest_mm, stellungen}. Anwendung:
hole_aus_schale.py --abbildung (Vorgabe: Datei, wenn vorhanden).
"""
import argparse, json, math, os, sys, time
import numpy as np
import rclpy
from rclpy.action import ActionClient
from rclpy.node import Node
from geometry_msgs.msg import Pose, PoseStamped, Quaternion
from moveit_msgs.action import MoveGroup
from moveit_msgs.msg import Constraints, JointConstraint, RobotState
from moveit_msgs.srv import GetPositionIK, GetStateValidity, GetPositionFK
from sensor_msgs.msg import JointState
from std_msgs.msg import Int32
from builtin_interfaces.msg import Duration
import tf2_ros

ARM = ['joint2_to_joint1', 'joint3_to_joint2', 'joint4_to_joint3',
       'joint5_to_joint4', 'joint6_to_joint5', 'joint6output_to_joint6']
BASE = "robot_base"
PLATTE = (-0.100, 0.400, -0.325, 0.075)          # Kaiser-Grundplatte in robot_base [m]
TRICHTER = (0.086, -0.159)                        # Fuss, Ø 120

ap = argparse.ArgumentParser()
ap.add_argument("--nur-pruefen", action="store_true")
ap.add_argument("--fahren", action="store_true")
ap.add_argument("--json", default=os.path.expanduser("~/ros2_ws/kamera_roboter_abbildung.json"))
ap.add_argument("--rand", type=float, default=30.0, help="[mm] Abstand zur Plattenkante")
ap.add_argument("--greifer", type=float, default=-0.30, help="Greifergelenk waehrend der Messung")
ap.add_argument("--referenz-punkt", default="nest_33")
ap.add_argument("--datei", default=os.path.expanduser("~/ros2_ws/teach_punkte.json"))
ap.add_argument("--min-ecken", type=int, default=10)
ap.add_argument("--proben", type=int, default=15, help="TF-Proben je Stellung")
ap.add_argument("--tempo", type=float, default=0.3, help="MoveIt velocity scaling")
ap.add_argument("--stellungen", default="", help="eigene Liste 'r,az,z;r,az,z;...' [mm, Grad, mm]")
ap.add_argument("--yaw-relativ", type=float, default=90.0, help="[Grad] (nur --neigung 0) Gierwinkel der Schliessachse relativ zum Azimut")
ap.add_argument("--neigung", type=float, default=45.0,
                help="[Grad] Musternormale gegen die Senkrechte, zur Kamera geneigt. 0 = Greifer senkrecht (Muster zeigt "
                     "dann SEITLICH - 2026-09-13: 10 Stellungen, 0 Ecken). Das Muster sitzt auf der FLACHEN Seite des "
                     "Greifers, Normale = -tcpZ; bei 45 Grad sah die Kamera es am 9.9. mit allen 10 Ecken.")
ap.add_argument("--kamera-xy", type=float, nargs=2, default=(0.170, -0.115), help="[m] Kamera ueber der Platte (fuer die Neigungsrichtung)")
a = ap.parse_args()

# Stellungen (TCP): Schalensektor, dazwischen, Ablageplatte; Hoehen 40-100 (geneigter Greifer)
STELLUNGEN = [(160, 340, 40), (200, 340, 40), (180, 355, 70), (160, 5, 40), (200, 5, 90), (180, 340, 100),
              (180, 350, 50), (170, 0, 60), (200, 350, 100), (190, 335, 60),
              (165, 255, 50), (215, 255, 50), (190, 263, 90), (170, 268, 40), (190, 258, 60), (175, 262, 70), (200, 265, 45)]
if a.stellungen:
    STELLUNGEN = [tuple(float(v) for v in s.split(",")) for s in a.stellungen.split(";") if s.strip()]

Q_REF = (-0.583486105248945, 0.3994295494594975, -0.3994295494594975, 0.583486105248945)   # tcp +Y nach unten


def qmul(p, q):
    ax, ay, az, aw = p; bx, by, bz, bw = q
    return (aw*bx + ax*bw + ay*bz - az*by, aw*by - ax*bz + ay*bw + az*bx,
            aw*bz + ax*by - ay*bx + az*bw, aw*bw - ax*bx - ay*by - az*bz)


def qz(deg):
    h = math.radians(deg) / 2.0
    return (0.0, 0.0, math.sin(h), math.cos(h))


def mat_quat(R):
    t = R[0, 0] + R[1, 1] + R[2, 2]
    if t > 0:
        s_ = math.sqrt(t + 1.0) * 2; w = 0.25 * s_; x = (R[2, 1] - R[1, 2]) / s_; y = (R[0, 2] - R[2, 0]) / s_; z = (R[1, 0] - R[0, 1]) / s_
    elif R[0, 0] > R[1, 1] and R[0, 0] > R[2, 2]:
        s_ = math.sqrt(1.0 + R[0, 0] - R[1, 1] - R[2, 2]) * 2; w = (R[2, 1] - R[1, 2]) / s_; x = 0.25 * s_; y = (R[0, 1] + R[1, 0]) / s_; z = (R[0, 2] + R[2, 0]) / s_
    elif R[1, 1] > R[2, 2]:
        s_ = math.sqrt(1.0 + R[1, 1] - R[0, 0] - R[2, 2]) * 2; w = (R[0, 2] - R[2, 0]) / s_; x = (R[0, 1] + R[1, 0]) / s_; y = 0.25 * s_; z = (R[1, 2] + R[2, 1]) / s_
    else:
        s_ = math.sqrt(1.0 + R[2, 2] - R[0, 0] - R[1, 1]) * 2; w = (R[1, 0] - R[0, 1]) / s_; x = (R[0, 2] + R[2, 0]) / s_; y = (R[1, 2] + R[2, 1]) / s_; z = 0.25 * s_
    n = math.sqrt(x * x + y * y + z * z + w * w)
    return Quaternion(x=x / n, y=y / n, z=z / n, w=w / n)


def orientierung(x, y, az):
    """tcp-Orientierung fuer eine Stellung. --neigung 0: senkrecht wie zeige_punkt (Q_REF, Yaw relativ
    zum Azimut). Sonst: Musternormale n = -tcpZ um --neigung aus der Senkrechten ZUR KAMERA geneigt,
    Annaeherung tcpY moeglichst nach unten (senkrecht zu Z), tcpX = Y x Z."""
    if a.neigung <= 0.0:
        q = qmul(qz(az + a.yaw_relativ), Q_REF); return Quaternion(x=q[0], y=q[1], z=q[2], w=q[3])
    up = np.array([0.0, 0.0, 1.0]); dh = np.array([a.kamera_xy[0] - x, a.kamera_xy[1] - y, 0.0])
    dh = dh / np.linalg.norm(dh) if np.linalg.norm(dh) > 1e-6 else np.array([0.0, 1.0, 0.0])
    nrm = math.cos(math.radians(a.neigung)) * up + math.sin(math.radians(a.neigung)) * dh
    Z = -nrm / np.linalg.norm(nrm)
    down = np.array([0.0, 0.0, -1.0]); Y = down - np.dot(down, Z) * Z; Y = Y / np.linalg.norm(Y)
    X = np.cross(Y, Z)
    return mat_quat(np.column_stack([X, Y, Z]))


rclpy.init(); node = Node("kamera_roboter_abbildung"); js = {"m": None}
node.create_subscription(JointState, "/joint_states", lambda m: js.__setitem__("m", m), 10)
ecken = {"n": 0}
node.create_subscription(Int32, "/charuco_detector/corners_detected", lambda m: ecken.__setitem__("n", m.data), 10)
mg = ActionClient(node, MoveGroup, "/move_action")
ikc = node.create_client(GetPositionIK, "/compute_ik")
valc = node.create_client(GetStateValidity, "/check_state_validity")
fkc = node.create_client(GetPositionFK, "/compute_fk")
puffer = tf2_ros.Buffer(); tf2_ros.TransformListener(puffer, node)


def spin(sek):
    t0 = time.time()
    while time.time() - t0 < sek: rclpy.spin_once(node, timeout_sec=0.05)


def ist_rad():
    js["m"] = None
    for _ in range(150):
        rclpy.spin_once(node, timeout_sec=0.1)
        if js["m"] is not None:
            d = dict(zip(js["m"].name, js["m"].position))
            if all(j in d for j in ARM): return [d[j] for j in ARM]
    sys.exit("!! keine /joint_states")


def kollisionsfrei(rad):
    rq = GetStateValidity.Request(); rq.group_name = "arm"
    rq.robot_state.joint_state.name = list(ARM) + ["gripper_controller"]
    rq.robot_state.joint_state.position = [float(v) for v in rad] + [a.greifer]
    f = valc.call_async(rq); rclpy.spin_until_future_complete(node, f, timeout_sec=20); r = f.result()
    return (r is not None and r.valid), ([] if r is None else sorted({f"{c.contact_body_1}x{c.contact_body_2}" for c in r.contacts}))


def ik_haltung(pose, ref):
    best, bd = None, 1e9
    for versuch in range(15):
        seed = ref if versuch % 2 == 0 else ist_rad()
        rq = GetPositionIK.Request(); r = rq.ik_request
        r.group_name = "arm"; r.ik_link_name = "tcp"; r.avoid_collisions = True; r.timeout = Duration(sec=1)
        r.robot_state.joint_state.name = list(ARM) + ["gripper_controller"]
        r.robot_state.joint_state.position = [float(v) for v in seed] + [a.greifer]
        ps = PoseStamped(); ps.header.frame_id = BASE; ps.pose = pose; r.pose_stamped = ps
        f = ikc.call_async(rq); rclpy.spin_until_future_complete(node, f, timeout_sec=10); res = f.result()
        if res is None or res.error_code.val != 1: continue
        nm = list(res.solution.joint_state.name); vl = list(res.solution.joint_state.position)
        sol = [vl[nm.index(j)] for j in ARM]
        if sol[2] >= 0 or abs(sol[4]) > math.radians(45 if a.neigung <= 0 else 75): continue
        d = max(abs(s_ - z_) for s_, z_ in zip(sol[1:5], ref[1:5]))
        if d < bd: best, bd = sol, d
    return best


def moveit_gelenkziel(rad, was):
    print(f"  -> MoveIt (kollisionsgeprueft): {was}")
    goal = MoveGroup.Goal(); req = goal.request
    req.group_name = "arm"; req.num_planning_attempts = 10; req.allowed_planning_time = 8.0
    req.max_velocity_scaling_factor = a.tempo; req.max_acceleration_scaling_factor = 0.1
    req.workspace_parameters.header.frame_id = BASE
    for k, sgn in (("min_corner", -1.0), ("max_corner", 1.0)):
        c = getattr(req.workspace_parameters, k); c.x = c.y = c.z = sgn
    cs = Constraints()
    for jn, jv in zip(ARM, rad):
        jc = JointConstraint(); jc.joint_name = jn; jc.position = float(jv)
        jc.tolerance_above = jc.tolerance_below = 0.02; jc.weight = 1.0; cs.joint_constraints.append(jc)
    req.goal_constraints.append(cs); goal.planning_options.plan_only = False
    f = mg.send_goal_async(goal); rclpy.spin_until_future_complete(node, f, timeout_sec=25); gh = f.result()
    if gh is None or not gh.accepted: print("  !! abgelehnt"); return False
    r = gh.get_result_async(); rclpy.spin_until_future_complete(node, r, timeout_sec=180)
    ok = r.result() is not None and r.result().result.error_code.val == 1
    if not ok: print(f"  !! Fahrt fehlgeschlagen (code {r.result().result.error_code.val if r.result() else '?'})")
    return ok


def stillstand(max_s=20.0):
    letzte, ruhig = None, None; t0 = time.time()
    while time.time() - t0 < max_s:
        spin(0.3); jetzt = ist_rad()
        if letzte is not None and max(abs(math.degrees(x - y)) for x, y in zip(jetzt, letzte)) < 0.15:
            ruhig = ruhig or time.time()
            if time.time() - ruhig > 2.0: return True
        else: ruhig = None
        letzte = jetzt
    return False


def tf_pos(ziel, quelle):
    try:
        tf = puffer.lookup_transform(ziel, quelle, rclpy.time.Time())
        t = tf.transform.translation; return np.array([t.x, t.y, t.z])
    except Exception: return None


def fit_affin(P_cam, P_mod):
    """p_mod = A p_cam + b, kleinste Quadrate. -> A (3x3), b (3), Rest je Punkt [m]."""
    X = np.hstack([P_cam, np.ones((len(P_cam), 1))])        # N x 4
    M, *_ = np.linalg.lstsq(X, P_mod, rcond=None)           # 4 x 3
    A = M[:3].T; b = M[3]
    rest = np.linalg.norm((P_cam @ A.T + b) - P_mod, axis=1)
    return A, b, rest


# ------------------------------------------------------------------ Vorbereitung
for c, n_ in ((ikc, "/compute_ik"), (valc, "/check_state_validity")):
    if not c.wait_for_service(20): sys.exit(f"!! {n_} fehlt")
if not mg.wait_for_server(20): sys.exit("!! /move_action fehlt")
ist_rad()
try: ref = [float(v) for v in json.load(open(a.datei))[a.referenz_punkt]["rad"]]
except Exception: ref = [0.0] * 6
spin(3.0)                                   # TF-Puffer fuellen (der Listener braucht spin)
if not puffer.can_transform(BASE, "charuco_muster", rclpy.time.Time()):
    print("!! TF robot_base->charuco_muster fehlt: Stack mit CHARUCO_MONTIERT=1 starten"); sys.exit(1)

x0, x1, y0, y1 = PLATTE; rand = a.rand / 1e3
plan = []
print("Stellungen pruefen (Neigung %.0f Grad, Greifer %.2f):" % (a.neigung, a.greifer))
for i, (r_mm, az, z_mm) in enumerate(STELLUNGEN, 1):
    x, y, z = r_mm / 1e3 * math.cos(math.radians(az)), r_mm / 1e3 * math.sin(math.radians(az)), z_mm / 1e3
    gruende = []
    if not (x0 + rand <= x <= x1 - rand and y0 + rand <= y <= y1 - rand): gruende.append("ausserhalb der Grundplatte")
    if not (0.140 <= r_mm / 1e3 <= 0.230): gruende.append("r ausserhalb 140-230")
    if 150.0 <= az % 360 <= 176.0: gruende.append("Totsektor")
    if z < 0.140 and math.hypot(x - TRICHTER[0], y - TRICHTER[1]) < 0.090: gruende.append("zu nah am Trichter")
    rad = None; kont = []
    if not gruende:
        pose = Pose(); pose.position.x, pose.position.y, pose.position.z = x, y, z
        pose.orientation = orientierung(x, y, az)
        rad = ik_haltung(pose, ref)
        if rad is None: gruende.append("keine IK in der Haltung")
        else:
            ok, kont = kollisionsfrei(rad)
            if not ok: gruende.append("KOLLISION " + " ".join(kont))
    status = "ok" if not gruende else "VERWORFEN: " + ", ".join(gruende)
    print("  %2d: r %3.0f az %3.0f z %3.0f -> (%+6.1f, %+6.1f, %3.0f) %s%s" % (
        i, r_mm, az, z_mm, x * 1e3, y * 1e3, z_mm, status,
        "" if rad is None else "   J " + " ".join("%+6.1f" % math.degrees(v) for v in rad)))
    if not gruende: plan.append(((x, y, z), rad, (r_mm, az, z_mm)))
print("%d von %d Stellungen zulaessig." % (len(plan), len(STELLUNGEN)))
if not a.fahren:
    print("(--fahren zum Messen)"); node.destroy_node(); rclpy.shutdown(); sys.exit(0)
if len(plan) < 6:
    print("!! zu wenige Stellungen fuer einen affinen Fit (>= 6)"); sys.exit(1)

# ------------------------------------------------------------------ Messen
mess = []
for i, ((x, y, z), rad, (r_mm, az, z_mm)) in enumerate(plan, 1):
    print("\n[%d/%d] r %.0f az %.0f z %.0f" % (i, len(plan), r_mm, az, z_mm))
    if not moveit_gelenkziel(rad, "Stellung %d" % i):
        print("  uebersprungen"); continue
    if not stillstand(): print("  !! kein Stillstand - uebersprungen"); continue
    spin(1.0)
    P_m, P_g, best = [], [], 0
    t0 = time.time()
    while time.time() - t0 < 6.0 and len(P_g) < a.proben:
        spin(0.2); best = max(best, ecken["n"])
        pm = tf_pos(BASE, "charuco_muster"); pg = tf_pos(BASE, "charuco_gemessen")
        if pm is not None and pg is not None and ecken["n"] >= a.min_ecken:
            P_m.append(pm); P_g.append(pg)
    if len(P_g) < 5:
        print("  !! nur %d Proben (Ecken max %d) - Board nicht (ganz) sichtbar, uebersprungen" % (len(P_g), best)); continue
    pm, pg = np.mean(P_m, 0), np.mean(P_g, 0); streu = np.std(P_g, 0) * 1e3
    tcp = tf_pos(BASE, "tcp")
    print("  Ecken %d, %d Proben, Streuung %.1f/%.1f/%.1f mm" % (best, len(P_g), *streu))
    print("  Modell-Muster (%.1f, %.1f, %.1f)  Kamera-Muster (%.1f, %.1f, %.1f)  Diff (%+.1f, %+.1f, %+.1f) mm"
          % (*(pm * 1e3), *(pg * 1e3), *((pm - pg) * 1e3)))
    mess.append({"stellung": [r_mm, az, z_mm], "q_grad": [round(math.degrees(v), 2) for v in ist_rad()],
                 "tcp_modell_mm": None if tcp is None else [round(v * 1e3, 1) for v in tcp],
                 "modell_mm": [round(v * 1e3, 2) for v in pm], "kamera_mm": [round(v * 1e3, 2) for v in pg],
                 "ecken": int(best), "proben": len(P_g), "streuung_mm": [round(v, 2) for v in streu]})

print("\n" + "=" * 70)
if len(mess) < 6:
    print("!! nur %d Messungen - kein affiner Fit (>= 6 noetig). Gespeichert wird trotzdem." % len(mess))
    json.dump({"messungen": mess, "zeit": time.strftime("%Y-%m-%d %H:%M")}, open(a.json, "w"), indent=1); sys.exit(1)
P_cam = np.array([m["kamera_mm"] for m in mess]) / 1e3; P_mod = np.array([m["modell_mm"] for m in mess]) / 1e3
A, b, rest = fit_affin(P_cam, P_mod)
d = P_mod - P_cam
print("Konstanter Versatz Modell-Kamera: Mittel (%+.1f, %+.1f, %+.1f) mm, Streuung (%.1f, %.1f, %.1f)"
      % (*(d.mean(0) * 1e3), *(d.std(0) * 1e3)))
print("AFFINER FIT p_modell = A p_kamera + b:")
print("  A =\n" + "\n".join("    " + " ".join("%+.4f" % v for v in row) for row in A))
print("  b = (%+.1f, %+.1f, %+.1f) mm" % tuple(b * 1e3))
print("  Massstab (Singulaerwerte von A): " + " ".join("%.3f" % s for s in np.linalg.svd(A, compute_uv=False)))
print("  Rest je Stellung [mm]: " + " ".join("%.1f" % (v * 1e3) for v in rest) + "   rms %.1f, max %.1f" % (np.sqrt(np.mean(rest ** 2)) * 1e3, rest.max() * 1e3))
# Leave-one-out als Vorhersagefehler
loo = []
for k in range(len(mess)):
    idx = [j for j in range(len(mess)) if j != k]
    Ak, bk, _ = fit_affin(P_cam[idx], P_mod[idx]); loo.append(np.linalg.norm(Ak @ P_cam[k] + bk - P_mod[k]))
print("  Leave-one-out-Vorhersagefehler [mm]: " + " ".join("%.1f" % (v * 1e3) for v in loo) + "   rms %.1f" % (np.sqrt(np.mean(np.array(loo) ** 2)) * 1e3))
json.dump({"A": A.tolist(), "b_m": b.tolist(), "rest_mm": [round(v * 1e3, 2) for v in rest],
           "loo_mm": [round(v * 1e3, 2) for v in loo], "versatz_konstant_mm": [round(v * 1e3, 1) for v in d.mean(0)],
           "messungen": mess, "zeit": time.strftime("%Y-%m-%d %H:%M"),
           "hinweis": "p_modell = A * p_kamera + b (Meter, robot_base). Gemessen mit ChArUco am Greifer, senkrechte Greifstellung, z 30-100 mm."},
          open(a.json, "w"), indent=1)
print("geschrieben: %s" % a.json)
node.destroy_node(); rclpy.shutdown()
