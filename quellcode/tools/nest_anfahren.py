#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""Ein Nest der Ablageplatte LEER anfahren — prueft, ob ein angelernter/gerechneter Punkt sitzt.

Gedacht fuer die 16 bilinear GERECHNETEN Zwischennester (2026-09-11): der Greifer faehrt
ohne Welle ueber das Nest, senkt auf den Nestpunkt + --dz und bleibt stehen, bis der Benutzer
geschaut hat, ob die Fingerspitzen mittig ueber dem Nest stehen. Danach heben.

Ablauf (Helfer 1:1 aus lege_welle.py):
  1. Greifer auf (--greifer-lassen: unveraendert)
  2. UEBER das Nest     IK-Ziel = Nest-TCP + --ueber (40 mm), MoveIt kollisionsgeprueft
  3. ABSENKEN ab Soll   gerade nach unten auf Nest-TCP + --dz (0 = Nestpunkt selbst:
                        Fingerspitze ~1 mm ueber dem Kopf einer sitzenden Welle, ~51 mm ueber Grund)
  4. nachstellen        Totband raus; danach Ist-TCP (Modell) gegen Nest-TCP
  5. warten             ENTER = heben, 'a' = stehen lassen und beenden
  6. heben, optional --null

    python3 tools/nest_anfahren.py --nest nest_33
    python3 tools/nest_anfahren.py --nest nest_33 --dz -10     # 10 mm tiefer
    python3 tools/nest_anfahren.py --nest nest_33 --warte 60   # ohne Terminal: 60 s stehen, dann heben
    python3 tools/nest_anfahren.py --nest nest_33 --stehen     # hinfahren, stehen lassen ...
    python3 tools/nest_anfahren.py --nest nest_33 --nur-heben  # ... und spaeter heben
    python3 tools/nest_anfahren.py --nest trichter_greif --nur-heben --ueber 60 --blind-mm 30 --greifer-lassen  # aus dem Trichter: 30 mm blind, dann geprueft
    python3 tools/nest_anfahren.py --nest nest_11 --nur-ueber --greifer-lassen --auto   # KAMERAPOSE: nur ueber nest_11 (+40) und stehen bleiben
Exit: 0 ok, 3 vom Benutzer stehen gelassen, 1 Fehler.
"""
import argparse, json, math, os, sys, time
import rclpy
from rclpy.action import ActionClient
from rclpy.node import Node
from geometry_msgs.msg import Pose, PoseStamped, Quaternion
from moveit_msgs.action import MoveGroup
from moveit_msgs.msg import Constraints, JointConstraint, RobotState
from moveit_msgs.srv import GetCartesianPath, GetPositionFK, GetPositionIK
from sensor_msgs.msg import JointState
from control_msgs.action import FollowJointTrajectory
from trajectory_msgs.msg import JointTrajectoryPoint
from builtin_interfaces.msg import Duration
sys.path.insert(0, os.path.dirname(os.path.abspath(__file__))); from bahnpruefung import pruefe_bahn  # Stetigkeit (Unfall 13.9.)
from gelenkspiel import nullstellung_rad  # reale Nullstellung (20.9.)
from gelenkspiel import j4_von_oben  # Spiel hinter Achse 4 (19.9.)

ARM = ['joint2_to_joint1', 'joint3_to_joint2', 'joint4_to_joint3',
       'joint5_to_joint4', 'joint6_to_joint5', 'joint6output_to_joint6']
BASE = 'robot_base'
ap = argparse.ArgumentParser()
ap.add_argument("--nest", required=True)
ap.add_argument("--datei", default=os.path.expanduser("~/ros2_ws/teach_punkte.json"))
ap.add_argument("--ueber", type=float, default=40.0, help="[mm] Anfahr-/Hebehoehe ueber dem Nestpunkt")
ap.add_argument("--dz", type=float, default=0.0, help="[mm] Haltehoehe relativ zum Nestpunkt")
ap.add_argument("--auf", type=float, default=0.15)
ap.add_argument("--greifer-lassen", action="store_true", help="Greifer nicht oeffnen")
ap.add_argument("--auto", action="store_true", help="nicht warten, gleich heben")
ap.add_argument("--warte", type=float, default=0.0, help="[s] statt ENTER so lange stehen bleiben (fuer Aufrufe ohne Terminal)")
ap.add_argument("--tempo", type=float, default=2.0)
ap.add_argument("--setzzeit", type=float, default=4.0)
ap.add_argument("--gain", type=float, default=0.6); ap.add_argument("--runden", type=int, default=4)
ap.add_argument("--toleranz", type=float, default=0.3); ap.add_argument("--max-korrektur", type=float, default=4.0)
ap.add_argument("--null", action="store_true", help="danach zur Nullstellung")
ap.add_argument("--ik-max-grad", type=float, default=25.0, help="max. Gelenkabstand der Ueber-IK-Loesung von der Neststellung")
ap.add_argument("--stehen", action="store_true", help="nach dem Nachstellen STEHEN BLEIBEN und beenden (Exit 3); heben spaeter mit --nur-heben")
ap.add_argument("--nur-heben", action="store_true", help="nur von der aktuellen Stellung gerade auf Nest + --ueber heben")
ap.add_argument("--nur-ueber", action="store_true", help="nur ueber das Nest (+--ueber) fahren und dort STEHEN bleiben (Kamerapose nest_11 +40, Exit 0)")
ap.add_argument("--ohne-j4-von-oben", action="store_true", help="Achse 4 nach dem Absenken NICHT von oben auf das Soll setzen (19.9.: Spiel hinter dem Getriebe)")
ap.add_argument("--blind-mm", type=float, default=0.0, help="[mm] erstes senkrechtes Stueck beim Heben OHNE Kollisionspruefung (Trichter: 30)")
a = ap.parse_args()
T = lambda sek: max(1.5, sek / a.tempo)

punkte = json.load(open(a.datei))
if a.nest not in punkte: sys.exit(f"!! Nest '{a.nest}' nicht in {a.datei}")
nest_rad = [float(v) for v in punkte[a.nest]["rad"]]

rclpy.init(); node = Node("nest_anfahren"); js = {"m": None}
node.create_subscription(JointState, "/joint_states", lambda m: js.__setitem__("m", m), 10)
arm = ActionClient(node, FollowJointTrajectory, "/arm_controller/follow_joint_trajectory")
grp = ActionClient(node, FollowJointTrajectory, "/gripper_controller/follow_joint_trajectory")
mg = ActionClient(node, MoveGroup, "/move_action")
cart = node.create_client(GetCartesianPath, "/compute_cartesian_path")
fkc = node.create_client(GetPositionFK, "/compute_fk")
ikc = node.create_client(GetPositionIK, "/compute_ik")
ERGEBNIS = {"nest": a.nest, "angefahren": False, "abweichung_mm": None}


def ende(code):
    print("  ERGEBNIS " + json.dumps(ERGEBNIS))
    node.destroy_node(); rclpy.shutdown(); sys.exit(code)


def frage(was):
    if a.auto: return
    try: antwort = input(f"  ? {was} — ENTER = weiter, a = abbrechen: ").strip().lower()
    except EOFError: antwort = ""
    if antwort.startswith("a"):
        print("  Abbruch durch Benutzer. Arm bleibt stehen, Greifer unveraendert."); ende(3)


def spin(sek):
    t0 = time.time()
    while time.time() - t0 < sek: rclpy.spin_once(node, timeout_sec=0.1)


def ist_rad():
    js["m"] = None
    for _ in range(30):
        rclpy.spin_once(node, timeout_sec=0.1)
        if js["m"] is not None:
            d = dict(zip(js["m"].name, js["m"].position))
            if all(j in d for j in ARM): return [d[j] for j in ARM]
    print("!! keine /joint_states"); ende(1)


def greifer_prozent():
    js["m"] = None
    for _ in range(30):
        rclpy.spin_once(node, timeout_sec=0.1)
        if js["m"] is not None and "gripper_controller" in js["m"].name:
            return (js["m"].position[js["m"].name.index("gripper_controller")] + 0.74) / 0.89 * 100.0  # Firmware-%: Bereich [-0.74, 0.15] (seit 2026-09-11 auch beim Lesen)
    return None


def fahre_gelenke(rad, dauer, was, setzzeit=None):
    print(f"  -> {was} ({dauer:.0f} s)")
    arm.wait_for_server(timeout_sec=15.0)
    g = FollowJointTrajectory.Goal(); g.trajectory.joint_names = ARM
    pt = JointTrajectoryPoint(); pt.positions = [float(v) for v in rad]
    pt.time_from_start.sec = int(dauer); pt.time_from_start.nanosec = int((dauer % 1) * 1e9)
    g.trajectory.points = [pt]
    f = arm.send_goal_async(g); rclpy.spin_until_future_complete(node, f, timeout_sec=20.0)
    gh = f.result()
    if gh is None or not gh.accepted: print(f"!! '{was}' abgelehnt"); ende(1)
    rf = gh.get_result_async(); rclpy.spin_until_future_complete(node, rf, timeout_sec=dauer + 30.0)
    spin(a.setzzeit if setzzeit is None else setzzeit)


def greifer(wert, dauer, was):
    print(f"  -> Greifer {was} ({dauer:.0f} s)")
    grp.wait_for_server(timeout_sec=15.0)
    g = FollowJointTrajectory.Goal(); g.trajectory.joint_names = ["gripper_controller"]
    pt = JointTrajectoryPoint(); pt.positions = [float(wert)]; pt.time_from_start.sec = int(dauer)
    g.trajectory.points = [pt]
    f = grp.send_goal_async(g); rclpy.spin_until_future_complete(node, f, timeout_sec=20.0)
    gh = f.result()
    if gh is not None and gh.accepted:
        rf = gh.get_result_async(); rclpy.spin_until_future_complete(node, rf, timeout_sec=dauer + 20.0)
    spin(1.0)


def nachstellen(ziel, was):
    fmt = lambda v: " ".join(f"{math.degrees(x):+7.2f}" for x in v)
    kommando = list(ziel); fehler = [i - z for i, z in zip(ist_rad(), ziel)]
    nick = lambda f: math.degrees(f[1] + f[2] + f[3]); tol = math.radians(a.toleranz)
    print(f"  nachstellen {was}: Fehler {fmt(fehler)} (Nick {nick(fehler):+.2f})")
    for runde in range(1, a.runden + 1):
        if all(abs(e) <= tol for e in fehler): print(f"     fertig nach {runde-1} Korrektur(en)"); break
        kommando = [k - a.gain * e for k, e in zip(kommando, fehler)]
        if any(abs(k - z) > math.radians(a.max_korrektur) for k, z in zip(kommando, ziel)):
            print("  !! Vorhalt zu gross — abgebrochen"); break
        fahre_gelenke(kommando, T(3), f"Korrektur {runde}", setzzeit=4.0)
        fehler = [i - z for i, z in zip(ist_rad(), ziel)]
        print(f"     Fehler {fmt(fehler)} (Nick {nick(fehler):+.2f})")


def fk_pose(rad):
    fkc.wait_for_service(10)
    rs = RobotState(); rs.joint_state.name = list(ARM); rs.joint_state.position = [float(v) for v in rad]
    rq = GetPositionFK.Request(); rq.header.frame_id = BASE; rq.fk_link_names = ["tcp"]; rq.robot_state = rs
    f = fkc.call_async(rq); rclpy.spin_until_future_complete(node, f, timeout_sec=10)
    if f.result() is None or not f.result().pose_stamped: print("!! FK fehlgeschlagen"); ende(1)
    return f.result().pose_stamped[0].pose


def ik_nahe(pose, seed, max_grad=25.0):
    """IK fuer pose, die zur seed-Stellung naechste Loesung (TRAC-IK streut)."""
    ikc.wait_for_service(10); best, bd = None, 1e9
    for _ in range(10):
        rq = GetPositionIK.Request(); r = rq.ik_request
        r.group_name = "arm"; r.ik_link_name = "tcp"; r.avoid_collisions = True; r.timeout = Duration(sec=1)
        r.robot_state.joint_state.name = list(ARM); r.robot_state.joint_state.position = [float(v) for v in seed]
        ps = PoseStamped(); ps.header.frame_id = BASE; ps.pose = pose; r.pose_stamped = ps
        f = ikc.call_async(rq); rclpy.spin_until_future_complete(node, f, timeout_sec=10); res = f.result()
        if res is None or res.error_code.val != 1: continue
        nm = list(res.solution.joint_state.name); vl = list(res.solution.joint_state.position)
        sol = [vl[nm.index(j)] for j in ARM]; d = max(abs(s_ - z_) for s_, z_ in zip(sol, seed))
        if d < bd: best, bd = sol, d
    if best is None or bd > math.radians(max_grad):
        print(f"!! keine IK-Loesung nahe der Neststellung ({'-' if best is None else f'{math.degrees(bd):.0f} Grad'})"); ende(1)
    return best


def moveit_gelenkziel(rad, was):
    print(f"  -> MoveIt (kollisionsgeprueft): {was}")
    mg.wait_for_server(20)
    goal = MoveGroup.Goal(); req = goal.request
    req.group_name = "arm"; req.num_planning_attempts = 10; req.allowed_planning_time = 5.0
    req.max_velocity_scaling_factor = min(1.0, 0.15 * a.tempo); req.max_acceleration_scaling_factor = 0.1
    req.workspace_parameters.header.frame_id = BASE
    for k, sgn in (("min_corner", -1.0), ("max_corner", 1.0)):
        c = getattr(req.workspace_parameters, k); c.x = c.y = c.z = sgn
    cs = Constraints()
    for jn, jv in zip(ARM, rad):
        jc = JointConstraint(); jc.joint_name = jn; jc.position = float(jv)
        jc.tolerance_above = jc.tolerance_below = 0.02; jc.weight = 1.0; cs.joint_constraints.append(jc)
    req.goal_constraints.append(cs); goal.planning_options.plan_only = False
    f = mg.send_goal_async(goal); rclpy.spin_until_future_complete(node, f, timeout_sec=25); gh = f.result()
    if gh is None or not gh.accepted: print("!! MoveIt-Ziel abgelehnt"); ende(1)
    r = gh.get_result_async(); rclpy.spin_until_future_complete(node, r, timeout_sec=120)
    if r.result() is None or r.result().result.error_code.val != 1: print("!! MoveIt-Fahrt fehlgeschlagen"); ende(1)
    spin(a.setzzeit)


def gerade_ab_soll(soll, ziel_pose, dauer, was, avoid=True):
    """Gerade Bahn ab SOLL-Gelenken auf ziel_pose; erster Bahnpunkt = Ist. Rueckgabe: neue Soll-Gelenke.
    avoid=False: OHNE Kollisionspruefung (nur fuer das erste, senkrechte Stueck aus dem Trichter:
    im Modell stehen die geschlossenen Finger 1 mm im Kragen, real ~10 mm darueber)."""
    cart.wait_for_service(10)
    rs = RobotState(); rs.joint_state.name = list(ARM); rs.joint_state.position = [float(v) for v in soll]
    cq = GetCartesianPath.Request(); cq.header.frame_id = BASE; cq.group_name = "arm"; cq.link_name = "tcp"
    cq.start_state = rs; cq.waypoints = [ziel_pose]; cq.max_step = 0.005; cq.jump_threshold = 0.0; cq.avoid_collisions = avoid
    f = cart.call_async(cq); rclpy.spin_until_future_complete(node, f, timeout_sec=120); res = f.result()
    if res is None: print(f"!! {was}: compute_cartesian_path Timeout"); ende(1)
    if res.fraction < 0.95: print(f"!! {was}: nur {res.fraction*100:.0f} % planbar"); ende(1)
    ok_b, txt_b = pruefe_bahn(res.solution.joint_trajectory, ARM, start=list(soll))
    if not ok_b: print(f"!! {was}: {txt_b} - NICHT gefahren"); ende(1)
    traj = res.solution.joint_trajectory
    neu = [traj.points[-1].positions[traj.joint_names.index(j)] for j in ARM]
    traj.points[0].positions = ist_rad()
    n = len(traj.points)
    for i, pt in enumerate(traj.points):
        t = dauer * (i + 1) / n; pt.time_from_start.sec = int(t); pt.time_from_start.nanosec = int((t - int(t)) * 1e9)
        pt.velocities = []; pt.accelerations = []
    print(f"  -> {was} (gerade, {dauer:.0f} s)")
    g = FollowJointTrajectory.Goal(); g.trajectory = traj
    fu = arm.send_goal_async(g); rclpy.spin_until_future_complete(node, fu, timeout_sec=20); gh = fu.result()
    if gh is None or not gh.accepted: print(f"!! {was} abgelehnt"); ende(1)
    rf = gh.get_result_async(); rclpy.spin_until_future_complete(node, rf, timeout_sec=dauer + 40)
    spin(a.setzzeit)
    return neu


def pose_dz(pose, dz_mm):
    p = Pose(); p.orientation = pose.orientation
    p.position.x = pose.position.x; p.position.y = pose.position.y; p.position.z = pose.position.z + dz_mm / 1000.0
    return p


# ------------------------------------------------------------------ Ablauf
ist_rad()
nest_pose = fk_pose(nest_rad)
if a.nur_heben:
    print(f"\n[6] heben (ab Ist)"); soll = ist_rad()
    if a.blind_mm > 0:
        p0 = fk_pose(soll)
        soll = gerade_ab_soll(soll, pose_dz(p0, a.blind_mm), T(3), f"heben {a.blind_mm:.0f} mm OHNE Kollisionspruefung (senkrecht)", avoid=False)
    soll = gerade_ab_soll(soll, pose_dz(nest_pose, a.ueber), T(5), "heben")
    if a.null: print("\n[7] Nullstellung"); moveit_gelenkziel(nullstellung_rad(), "Nullstellung (kollisionsgeprueft)")
    ERGEBNIS["angefahren"] = True; print("\n  fertig."); ende(0)
mm = lambda p: (p.position.x*1000, p.position.y*1000, p.position.z*1000)
print(f"\n  Nest '{a.nest}': TCP-Modell ({mm(nest_pose)[0]:+.1f}, {mm(nest_pose)[1]:+.1f}, {mm(nest_pose)[2]:.1f}) mm"
      f"  [{punkte[a.nest].get('notiz', '')}]")

if not a.greifer_lassen:
    print("\n[1] Greifer auf"); greifer(a.auf, 3, "auf")

print(f"\n[2] ueber das Nest ({a.ueber:.0f} mm)")
# "ueber"-Gelenke: ZUERST per gerader Bahn AUS der Neststellung nach oben (gleicher IK-Zweig
# wie angelernt - 2026-09-12 wich die IK-Loesung sonst um 20-30 Grad in J3/J4 ab, nachstellen
# brach ab), erst wenn das nicht geht per IK nahe am Seed.
def ueber_per_bahn(rad, dz_mm):
    cart.wait_for_service(10)
    rs = RobotState(); rs.joint_state.name = list(ARM); rs.joint_state.position = [float(v) for v in rad]
    cq = GetCartesianPath.Request(); cq.header.frame_id = BASE; cq.group_name = "arm"; cq.link_name = "tcp"
    cq.start_state = rs; cq.waypoints = [pose_dz(nest_pose, dz_mm)]; cq.max_step = 0.005; cq.jump_threshold = 0.0; cq.avoid_collisions = True
    f = cart.call_async(cq); rclpy.spin_until_future_complete(node, f, timeout_sec=120); res = f.result()
    if res is None or res.fraction < 0.99: return None
    if not pruefe_bahn(res.solution.joint_trajectory, ARM, start=list(rad))[0]: return None
    t = res.solution.joint_trajectory; return [t.points[-1].positions[t.joint_names.index(j)] for j in ARM]
ueber_rad = ueber_per_bahn(nest_rad, a.ueber)
if ueber_rad is None:
    print("  (gerade Bahn nach oben nicht planbar - IK nahe am Seed)")
    ueber_rad = ik_nahe(pose_dz(nest_pose, a.ueber), nest_rad, max_grad=a.ik_max_grad)
else:
    print(f"  ueber-Punkt per gerader Bahn aus der Neststellung (max Gelenkabstand {max(abs(u-n_) for u,n_ in zip(ueber_rad,nest_rad))*57.3:.1f} Grad)")
moveit_gelenkziel(ueber_rad, f"ueber {a.nest}")
soll = list(ueber_rad)
if a.nur_ueber:
    spin(1.0); ist = fk_pose(ist_rad()); ERGEBNIS["angefahren"] = True
    print(f"  --nur-ueber: Arm bleibt ueber {a.nest} +{a.ueber:.0f} mm, Ist-TCP ({mm(ist)[0]:+.1f}, {mm(ist)[1]:+.1f}, {mm(ist)[2]:.1f}) mm"); ende(0)

print(f"\n[3] absenken auf Nestpunkt {a.dz:+.0f} mm")
soll = gerade_ab_soll(soll, pose_dz(nest_pose, a.dz), T(6), "absenken")
if not a.ohne_j4_von_oben:
    # 19.9.: Spiel hinter Achse 4 legt sich auf die Seite der letzten Bewegung - wie beim Anlernen von OBEN (tools/gelenkspiel.py)
    j4_von_oben(lambda rad, dauer, was: fahre_gelenke(rad, dauer, was, setzzeit=2.0), soll)
print("\n[4] nachstellen"); nachstellen(soll, "Neststellung")
spin(2.0)
ist = fk_pose(ist_rad()); zi = pose_dz(nest_pose, a.dz)
d = [i - z for i, z in zip(mm(ist), mm(zi))]
ERGEBNIS["angefahren"] = True; ERGEBNIS["abweichung_mm"] = [round(v, 1) for v in d]
print(f"  Ist-TCP (Modell aus Encoder): ({mm(ist)[0]:+.1f}, {mm(ist)[1]:+.1f}, {mm(ist)[2]:.1f}) mm"
      f"  -> Abweichung zum Ziel dx {d[0]:+.1f} dy {d[1]:+.1f} dz {d[2]:+.1f} mm")

print("\n[5] Sichtpruefung: stehen die Fingerspitzen mittig ueber dem Nest?")
if a.stehen:
    print("  Arm bleibt stehen (--stehen). Heben: --nur-heben"); ende(3)
if a.warte > 0:
    print(f"  bleibe {a.warte:.0f} s stehen ..."); spin(a.warte)
elif not a.auto:
    try: antwort = input("  ? ENTER = heben, a = stehen lassen und beenden: ").strip().lower()
    except EOFError: antwort = ""
    if antwort.startswith("a"): print("  Arm bleibt stehen."); ende(3)

print("\n[6] heben")
soll = gerade_ab_soll(soll, pose_dz(nest_pose, a.ueber), T(5), "heben")
if a.null:
    print("\n[7] Nullstellung"); moveit_gelenkziel(nullstellung_rad(), "Nullstellung (kollisionsgeprueft)")
print("\n  fertig."); ende(0)
