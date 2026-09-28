#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""Messreihe GELENKSPIEL: wie schief steht der "senkrechte" Greifer wirklich, und wie gross ist
der Fehler, den der Encoder NICHT sieht - je Nest und je Anfahrrichtung.

Warum (2026-09-19, Benutzer): "Die Teach-Punkte stimmen wegen des Spiels nicht, und der Greifer
steht nicht senkrecht, obwohl das Modell 90 Grad sagt. Bevor wir neu anlernen, muessen wir wissen,
wie gross das Spiel ist."

Zwei Fehleranteile, zwei Messungen:
  1. ENCODER-SEITIG (Totband der Servos): Soll-Gelenkwinkel gegen /joint_states nach dem Setzen.
     Wird hier je Gelenk protokolliert (vor und nach dem Nachstellen). Nick = Summe J2+J3+J4.
  2. HINTER DEM GETRIEBE (Spiel im Abtrieb, das der Encoder prinzipiell nicht sieht,
     Elephant Robotics 2026-09-08): von AUSSEN mit einer Wasserwaage/Handy-Neigungs-App am
     Greifergehaeuse gemessen. Der Benutzer liest zwei Winkel ab und tippt sie ein:
        radial     Handy flach an die Gehaeuseseite, die zur ROBOTERBASIS zeigt
                   (+ = Fingerspitzen weiter von der Basis weg, - = zur Basis hin)
        tangential Handy flach an eine der beiden anderen Seiten
                   (+ = Fingerspitzen nach LINKS, von der Basis zum Greifer geschaut)
     Bei 135 mm Fingerlaenge (Flansch -> Spitze) verschiebt 1 Grad Neigung die Spitzen um 2.4 mm.
     Das ist der Anteil, der die Nester "daneben" treffen laesst, obwohl nachstellen 0.3 Grad meldet.

Anfahrrichtung (das Spiel ist richtungsabhaengig, 2026-09-10: bis 14 mm):
    oben   ueber dem Nest (+ueber) -> gerade absenken auf den Nestpunkt         (wie im Zyklus)
    unten  ueber dem Nest -> absenken auf Nestpunkt - unten_mm -> gerade heben   (gegen die Schwerkraft)
Danach in beiden Faellen: nachstellen (Totband raus, wie lege_welle), Setzzeit, Messung.

Aus den FK-Empfindlichkeiten (TCP-Verschiebung je +1 Grad J4 bzw. J5) rechnet das Skript am Ende
eine Empfehlung: `--nick-korrektur` (Grad auf J4) und `--roll-korrektur` (Grad auf J5), die den
gemessenen Mittelwert aufheben wuerden. Zur Probe die Reihe mit diesen Werten wiederholen - die
Ablesung sollte dann ~0 sein.

    python3 tools/gelenkspiel_messreihe.py --nester nest_11 nest_14 nest_33 nest_51 nest_54
    python3 tools/gelenkspiel_messreihe.py --nester nest_33 --richtung oben unten --wiederholungen 2
    python3 tools/gelenkspiel_messreihe.py --nester nest_33 --nick-korrektur -3.5      # Probe der Korrektur
    python3 tools/gelenkspiel_messreihe.py --nester nest_11 --ohne-neigung --auto       # nur Encoder, ohne Fragen

Ausgabe: messungen/gelenkspiel_<Datum>.json (alle Einzelmessungen, wird fortgeschrieben),
messungen/gelenkspiel_<Datum>.md (Tabelle fuer die Arbeit), Kamerabild je Messung im gleichnamigen Ordner.
Vor jeder Fahrt ENTER (ohne --auto). Der Arm bleibt am Ende ueber dem letzten Nest (+ueber); --null = Nullstellung (MoveIt).
Exit 0 ok, 1 Fehler, 3 Abbruch durch Benutzer.
"""
import argparse, json, math, os, sys, time
import numpy as np
import rclpy
from rclpy.action import ActionClient
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from geometry_msgs.msg import Pose, PoseStamped
from moveit_msgs.action import MoveGroup
from moveit_msgs.msg import Constraints, JointConstraint, RobotState
from moveit_msgs.srv import GetCartesianPath, GetPositionFK, GetPositionIK
from control_msgs.action import FollowJointTrajectory
from trajectory_msgs.msg import JointTrajectoryPoint
from sensor_msgs.msg import JointState, Image
from builtin_interfaces.msg import Duration

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__))); from bahnpruefung import pruefe_bahn  # Stetigkeit (Unfall 13.9.)
from gelenkspiel import nullstellung_rad  # reale Nullstellung (20.9.)
ARM = ["joint2_to_joint1", "joint3_to_joint2", "joint4_to_joint3",
       "joint5_to_joint4", "joint6_to_joint5", "joint6output_to_joint6"]
BASE = "robot_base"
FINGER_MM = 135.0        # Flansch -> Fingerspitze (zu), URDF tcp

ap = argparse.ArgumentParser()
ap.add_argument("--nester", nargs="+", required=True, help="Namen aus teach_punkte.json")
ap.add_argument("--datei", default=os.path.expanduser("~/ros2_ws/teach_punkte.json"))
ap.add_argument("--richtung", nargs="+", default=["oben"], choices=["oben", "unten"])
ap.add_argument("--wiederholungen", type=int, default=1)
ap.add_argument("--ueber", type=float, default=40.0, help="[mm] Anfahrhoehe ueber dem Nestpunkt")
ap.add_argument("--dz", type=float, default=0.0, help="[mm] Messhoehe relativ zum Nestpunkt")
ap.add_argument("--unten-mm", type=float, default=12.0, help="[mm] Ueberfahrt nach unten fuer die Richtung 'unten'")
ap.add_argument("--setzzeit", type=float, default=6.0, help="[s] Setzzeit vor jeder Messung (2026-09-10: 6 s)")
ap.add_argument("--tempo", type=float, default=2.0)
ap.add_argument("--ohne-nachstellen", action="store_true", help="reine Anfahrt messen (kein Totband-Ausgleich)")
ap.add_argument("--gain", type=float, default=0.6); ap.add_argument("--runden", type=int, default=4)
ap.add_argument("--toleranz", type=float, default=0.3); ap.add_argument("--max-korrektur", type=float, default=4.0)
ap.add_argument("--nick-korrektur", type=float, default=0.0, help="[Grad] Vorhalt auf J4 (Probe der Korrektur)")
ap.add_argument("--roll-korrektur", type=float, default=0.0, help="[Grad] Vorhalt auf J5 (Probe der Korrektur)")
ap.add_argument("--ohne-neigung", action="store_true", help="keine Handy-Ablesung abfragen")
ap.add_argument("--ohne-bild", action="store_true")
ap.add_argument("--auto", action="store_true", help="keine ENTER-Abfragen vor den Fahrten")
ap.add_argument("--null", action="store_true", help="am Ende Nullstellung (MoveIt)")
ap.add_argument("--ordner", default=os.path.expanduser("~/ros2_ws/messungen"))
ap.add_argument("--notiz", default="", help="freier Text fuer das Protokoll (z.B. 'Greifer zu, ohne Welle')")
a = ap.parse_args()
T = lambda sek: max(1.5, sek / a.tempo)
DATUM = time.strftime("%Y-%m-%d")
os.makedirs(os.path.join(a.ordner, f"gelenkspiel_{DATUM}"), exist_ok=True)
JSON_PFAD = os.path.join(a.ordner, f"gelenkspiel_{DATUM}.json")
MD_PFAD = os.path.join(a.ordner, f"gelenkspiel_{DATUM}.md")

punkte = json.load(open(a.datei))
for nm in a.nester:
    if nm not in punkte: sys.exit(f"!! '{nm}' nicht in {a.datei}")

rclpy.init(); node = Node("gelenkspiel_messreihe"); js = {"m": None, "img": None}
node.create_subscription(JointState, "/joint_states", lambda m: js.__setitem__("m", m), 10)
if not a.ohne_bild:
    node.create_subscription(Image, "/camera/color/image_raw", lambda m: js.__setitem__("img", m), qos_profile_sensor_data)
arm = ActionClient(node, FollowJointTrajectory, "/arm_controller/follow_joint_trajectory")
mg = ActionClient(node, MoveGroup, "/move_action")
cart = node.create_client(GetCartesianPath, "/compute_cartesian_path")
fkc = node.create_client(GetPositionFK, "/compute_fk")
ikc = node.create_client(GetPositionIK, "/compute_ik")
MESSUNGEN = []


def ende(code):
    node.destroy_node(); rclpy.shutdown(); sys.exit(code)


def frage(was):
    if a.auto: return
    try: antwort = input(f"  ? {was} — ENTER = weiter, a = abbrechen: ").strip().lower()
    except EOFError: antwort = ""
    if antwort.startswith("a"): print("  Abbruch durch Benutzer. Arm bleibt stehen."); schreibe_bericht(); ende(3)


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


def nachstellen(ziel, was):
    """Wie lege_welle/nest_anfahren: Ziel + Fehler kommandieren, bis der Encoder das Ziel meldet.
    Rueckgabe: (Fehler vor dem Nachstellen, Fehler danach) [rad]."""
    fmt = lambda v: " ".join(f"{math.degrees(x):+6.2f}" for x in v)
    kommando = list(ziel); fehler0 = [i - z for i, z in zip(ist_rad(), ziel)]; fehler = list(fehler0)
    nick = lambda f: math.degrees(f[1] + f[2] + f[3]); tol = math.radians(a.toleranz)
    print(f"  Encoder-Fehler {was}: {fmt(fehler)} (Nick {nick(fehler):+.2f})")
    if a.ohne_nachstellen: return fehler0, fehler
    for runde in range(1, a.runden + 1):
        if all(abs(e) <= tol for e in fehler): print(f"     fertig nach {runde-1} Korrektur(en)"); break
        kommando = [k - a.gain * e for k, e in zip(kommando, fehler)]
        if any(abs(k - z) > math.radians(a.max_korrektur) for k, z in zip(kommando, ziel)):
            print("  !! Vorhalt zu gross — abgebrochen"); break
        fahre_gelenke(kommando, T(3), f"Korrektur {runde}", setzzeit=4.0)
        fehler = [i - z for i, z in zip(ist_rad(), ziel)]
        print(f"     Fehler {fmt(fehler)} (Nick {nick(fehler):+.2f})")
    return fehler0, fehler


def fk_pose(rad):
    fkc.wait_for_service(10)
    rs = RobotState(); rs.joint_state.name = list(ARM); rs.joint_state.position = [float(v) for v in rad]
    rq = GetPositionFK.Request(); rq.header.frame_id = BASE; rq.fk_link_names = ["tcp"]; rq.robot_state = rs
    f = fkc.call_async(rq); rclpy.spin_until_future_complete(node, f, timeout_sec=10)
    if f.result() is None or not f.result().pose_stamped: print("!! FK fehlgeschlagen"); ende(1)
    return f.result().pose_stamped[0].pose


def ik_nahe(pose, seed, max_grad=25.0):
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
        print(f"!! keine IK-Loesung nahe der Neststellung"); ende(1)
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
    spin(2.0)


def gerade_ab_soll(soll, ziel_pose, dauer, was):
    """Gerade Bahn ab SOLL-Gelenken (erster Punkt = Ist), Stetigkeitsprobe, Rueckgabe: neue Soll-Gelenke."""
    cart.wait_for_service(10)
    rs = RobotState(); rs.joint_state.name = list(ARM); rs.joint_state.position = [float(v) for v in soll]
    cq = GetCartesianPath.Request(); cq.header.frame_id = BASE; cq.group_name = "arm"; cq.link_name = "tcp"
    cq.start_state = rs; cq.waypoints = [ziel_pose]; cq.max_step = 0.005; cq.jump_threshold = 0.0; cq.avoid_collisions = True
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
    spin(2.0)
    return neu


def pose_dz(pose, dz_mm):
    p = Pose(); p.orientation = pose.orientation
    p.position.x = pose.position.x; p.position.y = pose.position.y; p.position.z = pose.position.z + dz_mm / 1000.0
    return p


mm = lambda p: np.array((p.position.x * 1000, p.position.y * 1000, p.position.z * 1000))


def richtungen(rad):
    """Einheitsvektoren radial (Basis -> TCP, waagerecht) und tangential (links davon) am Punkt."""
    p = mm(fk_pose(rad)); r = np.array((p[0], p[1], 0.0)); r /= max(np.linalg.norm(r), 1e-9)
    return r, np.array((-r[1], r[0], 0.0))


def empfindlichkeit(rad):
    """TCP-Verschiebung [mm] je +1 Grad J4 (radial) und J5 (tangential) - fuer die Vorzeichen der Korrektur."""
    p0 = mm(fk_pose(rad)); rd, tg = richtungen(rad); out = {}
    for name, idx, vec in (("J4", 3, rd), ("J5", 4, tg)):
        q = list(rad); q[idx] += math.radians(1.0)
        out[name] = float(np.dot(mm(fk_pose(q)) - p0, vec))
    return out


def ablesung(text):
    if a.ohne_neigung or a.auto: return None
    while True:
        try: s = input(f"  ? {text} [Grad], ENTER = keine Ablesung: ").strip().replace(",", ".")
        except EOFError: return None
        if s == "": return None
        try: return float(s)
        except ValueError: print("    bitte Zahl (z.B. -3.5)")


def bild_sichern(name):
    if a.ohne_bild: return ""
    for _ in range(30):
        rclpy.spin_once(node, timeout_sec=0.1)
        if js["img"] is not None: break
    m = js["img"]
    if m is None: print("  (kein Kamerabild)"); return ""
    try:
        import cv2
        buf = np.frombuffer(m.data, dtype=np.uint8).reshape(m.height, m.width, 3)
        bgr = np.ascontiguousarray(buf[:, :, ::-1] if m.encoding.lower() == "rgb8" else buf)
        pfad = os.path.join(a.ordner, f"gelenkspiel_{DATUM}", name + ".png"); cv2.imwrite(pfad, bgr); return pfad
    except Exception as e:
        print(f"  (Bild nicht gesichert: {e})"); return ""


def schreibe_bericht():
    alt = json.load(open(JSON_PFAD)) if os.path.exists(JSON_PFAD) else []
    alt.extend(MESSUNGEN); json.dump(alt, open(JSON_PFAD, "w"), indent=1, ensure_ascii=False)
    z = ["# Gelenkspiel-Messreihe " + DATUM, "",
         "Soll = angelernte Gelenkwinkel (teach_punkte.json), Ist = Encoder nach Setzzeit; Nick = Fehler J2+J3+J4.",
         "Neigung = Ablesung am Greifergehaeuse (Handy), radial + = Spitzen von der Basis weg, tangential + = nach links.",
         f"Spitzenversatz = Neigung x {FINGER_MM:.0f} mm Fingerlaenge. Korrekturen im Lauf: J4 {a.nick_korrektur:+.2f} Grad, J5 {a.roll_korrektur:+.2f} Grad.", "",
         "| Nest | Richtung | Wdh | Nick Encoder vor NS | Nick Encoder nach NS | max Gelenkfehler nach NS | Neigung radial | Neigung tangential | Spitzenversatz rad/tang [mm] | Bild |",
         "|---|---|---|---|---|---|---|---|---|---|"]
    for m in alt:
        f = lambda v: "-" if v is None else f"{v:+.2f}"
        vr = "-" if m["neigung_radial_grad"] is None else f"{math.tan(math.radians(m['neigung_radial_grad']))*FINGER_MM:+.1f}"
        vt = "-" if m["neigung_tangential_grad"] is None else f"{math.tan(math.radians(m['neigung_tangential_grad']))*FINGER_MM:+.1f}"
        z.append(f"| {m['nest']} | {m['richtung']} | {m['wiederholung']} | {m['nick_encoder_vor_grad']:+.2f} | {m['nick_encoder_nach_grad']:+.2f} | "
                 f"{m['max_gelenkfehler_nach_grad']:.2f} | {f(m['neigung_radial_grad'])} | {f(m['neigung_tangential_grad'])} | {vr} / {vt} | {os.path.basename(m['bild']) if m['bild'] else '-'} |")
    rad_w = [m["neigung_radial_grad"] for m in alt if m["neigung_radial_grad"] is not None]
    tan_w = [m["neigung_tangential_grad"] for m in alt if m["neigung_tangential_grad"] is not None]
    z += [""]
    if rad_w:
        z.append(f"Neigung radial: Mittel {np.mean(rad_w):+.2f} Grad, Streuung {np.std(rad_w):.2f}, n={len(rad_w)}"
                 f" -> Spitzen {math.tan(math.radians(np.mean(rad_w)))*FINGER_MM:+.1f} mm")
    if tan_w:
        z.append(f"Neigung tangential: Mittel {np.mean(tan_w):+.2f} Grad, Streuung {np.std(tan_w):.2f}, n={len(tan_w)}")
    for m in alt[-1:]:
        if m.get("empfehlung"): z.append(""); z.append("Empfehlung (letzter Lauf): " + m["empfehlung"])
    if a.notiz: z += ["", "Notiz: " + a.notiz]
    open(MD_PFAD, "w").write("\n".join(z) + "\n")
    print(f"\n  Protokoll: {MD_PFAD}\n             {JSON_PFAD}")


# ------------------------------------------------------------------ Ablauf
ist_rad()
print(f"\nGelenkspiel-Messreihe: {len(a.nester)} Nest(er) x {a.richtung} x {a.wiederholungen}; Setzzeit {a.setzzeit:.0f} s;"
      f" Korrektur J4 {a.nick_korrektur:+.2f} / J5 {a.roll_korrektur:+.2f} Grad; nachstellen {'AUS' if a.ohne_nachstellen else 'an'}")
print("  Greifer wird NICHT bewegt (zu lassen, ohne Welle).")
emp_alle = {}
for nest in a.nester:
    nest_rad = [float(v) for v in punkte[nest]["rad"]]
    nest_rad[3] += math.radians(a.nick_korrektur); nest_rad[4] += math.radians(a.roll_korrektur)
    # Ziel-TCP: LAGE ohne Korrektur (das Nest bleibt das Nest), ORIENTIERUNG mit Korrektur (sonst dreht die
    # gerade Bahn den Vorhalt beim Absenken wieder heraus und erst das Nachstellen bringt ihn zurueck)
    nest_pose = fk_pose(nest_rad); p_orig = fk_pose([float(v) for v in punkte[nest]["rad"]]).position
    nest_pose.position.x, nest_pose.position.y, nest_pose.position.z = p_orig.x, p_orig.y, p_orig.z
    emp = empfindlichkeit(nest_rad); emp_alle[nest] = emp
    p = mm(nest_pose)
    print(f"\n=== {nest}: TCP-Modell ({p[0]:+.1f}, {p[1]:+.1f}, {p[2]:.1f}) mm; +1 Grad J4 -> {emp['J4']:+.2f} mm radial, +1 Grad J5 -> {emp['J5']:+.2f} mm tangential")
    for richtung in a.richtung:
        for w in range(1, a.wiederholungen + 1):
            frage(f"{nest} von {richtung.upper()} (Wdh {w}) anfahren")
            # ueber-Punkt: per gerader Bahn aus der Neststellung nach oben (gleicher IK-Zweig), sonst IK nahe Seed
            cart.wait_for_service(10)
            rs = RobotState(); rs.joint_state.name = list(ARM); rs.joint_state.position = list(nest_rad)
            cq = GetCartesianPath.Request(); cq.header.frame_id = BASE; cq.group_name = "arm"; cq.link_name = "tcp"
            cq.start_state = rs; cq.waypoints = [pose_dz(nest_pose, a.ueber)]; cq.max_step = 0.005; cq.jump_threshold = 0.0; cq.avoid_collisions = True
            f = cart.call_async(cq); rclpy.spin_until_future_complete(node, f, timeout_sec=120); res = f.result()
            ueber_rad = None
            if res is not None and res.fraction >= 0.99 and pruefe_bahn(res.solution.joint_trajectory, ARM, start=list(nest_rad))[0]:
                t = res.solution.joint_trajectory; ueber_rad = [t.points[-1].positions[t.joint_names.index(j)] for j in ARM]
            if ueber_rad is None: ueber_rad = ik_nahe(pose_dz(nest_pose, a.ueber), nest_rad)
            moveit_gelenkziel(ueber_rad, f"ueber {nest} (+{a.ueber:.0f} mm)")
            soll = list(ueber_rad)
            if richtung == "oben":
                soll = gerade_ab_soll(soll, pose_dz(nest_pose, a.dz), T(6), "absenken auf den Nestpunkt")
            else:
                soll = gerade_ab_soll(soll, pose_dz(nest_pose, a.dz - a.unten_mm), T(6), f"absenken auf Nestpunkt -{a.unten_mm:.0f} mm")
                soll = gerade_ab_soll(soll, pose_dz(nest_pose, a.dz), T(3), "heben auf den Nestpunkt (von unten)")
            # Ziel-Gelenke fuer das Nachstellen: die angelernten (+ Korrektur), nicht das Bahnende
            ziel = list(nest_rad) if a.dz == 0 else soll
            print(f"  Setzzeit {a.setzzeit:.0f} s ..."); spin(a.setzzeit)
            f0, f1 = nachstellen(ziel, f"{nest}/{richtung}")
            if not a.ohne_nachstellen: print(f"  Setzzeit {a.setzzeit:.0f} s ..."); spin(a.setzzeit); f1 = [i - z for i, z in zip(ist_rad(), ziel)]
            ist = ist_rad(); tcp_ist = mm(fk_pose(ist)); d = tcp_ist - mm(pose_dz(nest_pose, a.dz))
            print(f"  Ist-TCP (Modell aus Encoder): ({tcp_ist[0]:+.1f}, {tcp_ist[1]:+.1f}, {tcp_ist[2]:.1f}) mm -> Abweichung dx {d[0]:+.1f} dy {d[1]:+.1f} dz {d[2]:+.1f} mm")
            name = f"{nest}_{richtung}_{w}_{time.strftime('%H%M%S')}"
            bild = bild_sichern(name)
            print("\n  MESSUNG von aussen: Handy/Wasserwaage flach an das Greifergehaeuse halten.")
            nr = ablesung("Neigung RADIAL (Seite zur Basis; + = Spitzen von der Basis weg)")
            nt = ablesung("Neigung TANGENTIAL (Seite quer; + = Spitzen nach links, von der Basis geschaut)")
            m = {"zeit": time.strftime("%Y-%m-%d %H:%M:%S"), "nest": nest, "richtung": richtung, "wiederholung": w,
                 "soll_grad": [round(math.degrees(v), 3) for v in ziel], "ist_grad": [round(math.degrees(v), 3) for v in ist],
                 "fehler_vor_ns_grad": [round(math.degrees(v), 3) for v in f0], "fehler_nach_ns_grad": [round(math.degrees(v), 3) for v in f1],
                 "nick_encoder_vor_grad": round(math.degrees(f0[1] + f0[2] + f0[3]), 3),
                 "nick_encoder_nach_grad": round(math.degrees(f1[1] + f1[2] + f1[3]), 3),
                 "max_gelenkfehler_nach_grad": round(max(abs(math.degrees(v)) for v in f1), 3),
                 "tcp_modell_mm": [round(float(v), 1) for v in tcp_ist], "abweichung_modell_mm": [round(float(v), 1) for v in d],
                 "neigung_radial_grad": nr, "neigung_tangential_grad": nt,
                 "nick_korrektur_grad": a.nick_korrektur, "roll_korrektur_grad": a.roll_korrektur,
                 "nachstellen": not a.ohne_nachstellen, "setzzeit_s": a.setzzeit, "bild": bild, "notiz": a.notiz,
                 "empfindlichkeit_mm_je_grad": emp}
            MESSUNGEN.append(m)
            if nr is not None: print(f"  -> Spitzenversatz radial {math.tan(math.radians(nr))*FINGER_MM:+.1f} mm")
            frage("heben")
            soll = gerade_ab_soll(ziel if a.dz == 0 else soll, pose_dz(nest_pose, a.ueber), T(5), "heben")

# Empfehlung: Vorzeichen aus der FK-Empfindlichkeit. Neigung radial + (Spitzen weg) heisst: die Spitzen stehen
# real weiter aussen als das Modell. Ein Vorhalt dJ4 verschiebt die Modell-Spitzen um emp[J4]*dJ4 mm radial;
# real sollen sie um -tan(nr)*FINGER zurueck -> dJ4 = -nr * sign(emp[J4])  (1 Grad Neigung ~ 1 Grad Gelenk).
rad_w = [m["neigung_radial_grad"] for m in MESSUNGEN if m["neigung_radial_grad"] is not None]
tan_w = [m["neigung_tangential_grad"] for m in MESSUNGEN if m["neigung_tangential_grad"] is not None]
if MESSUNGEN and (rad_w or tan_w):
    e = emp_alle[a.nester[-1]]; teile = []
    if rad_w:
        dj4 = a.nick_korrektur - float(np.mean(rad_w)) * (1.0 if e["J4"] > 0 else -1.0)
        teile.append(f"--nick-korrektur {dj4:+.2f}")
    if tan_w:
        dj5 = a.roll_korrektur - float(np.mean(tan_w)) * (1.0 if e["J5"] > 0 else -1.0)
        teile.append(f"--roll-korrektur {dj5:+.2f}")
    MESSUNGEN[-1]["empfehlung"] = " ".join(teile)
    print(f"\n  EMPFEHLUNG fuer den naechsten Lauf (hebt die mittlere Ablesung auf): {' '.join(teile)}")
schreibe_bericht()
if a.null: print("\nNullstellung"); moveit_gelenkziel(nullstellung_rad(), "Nullstellung (kollisionsgeprueft)")
print("\n  fertig."); ende(0)
