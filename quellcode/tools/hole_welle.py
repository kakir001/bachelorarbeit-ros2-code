#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""Welle aus dem Trichter holen — das Rezept des ERSTEN VOLLEN ZYKLUS (2026-09-12, 01:5x).

Ablauf (die Reihenfolge ist der ganze Punkt):

  1. Nullstellung                     Ausgangspunkt, immer derselbe (--ohne-nullstellung:
                                      von dort starten, wo der Arm steht)
  2. Greifer auf
  3. UEBER den Griffpunkt (+40 mm)    Ueber-Gelenke per GERADER BAHN aus der angelernten
                                      Griffstellung nach oben (gleicher IK-Zweig; per IK
                                      wich J3/J4 um 20-30 Grad ab, nachstellen brach ab),
                                      dann MoveIt kollisionsgeprueft dorthin.
  4. GERADE AB auf den Griffpunkt     Bahn ab Soll, erster Punkt = Ist. Dann NACHSTELLEN
                                      (Totband, tools/nachstellen.py) — ohne das steht der
                                      Greifer bis 4 Grad schief.
  5. Greifer GANZ zu (-0.74) + lesen  Nur -0.74 taugt als Sensor: leer = 0 %, Welle am Kopf
                                      (Reibgummi) = 15-22 %. Mit -0.62 meldet die Firmware
                                      bei leerem Greifer den Kommandowert (13 %).
                                        <  8 %  = nichts gefasst  -> auf, nachstellen, nochmal
                                        >= 30 % = Auslaufrand     -> auf, +2 mm ab Soll, nochmal
  6. SENKRECHT heraus                 Erste 30 mm OHNE Kollisionspruefung (das Modell stellt
                                      die geschlossenen Finger 1 mm in den Kragen, real ~10 mm
                                      darueber), dann geprueft auf Griffpunkt +60 mm. Rein
                                      vertikal, Orientierung fest: J6 im Trichter NIE drehen
                                      (Kabel wickelt sich, Waende). Seitlich verkantet die
                                      Welle im Auslauf und der Trichter verschiebt sich.
  6b. Haltepruefung                   Greifer lesen: < 8 % = Welle verloren -> ganzer Griff
                                      nochmal (--wiederholungen).
  7. Nullstellung, Greifer bleibt ZU  --nur-hin: stehen bleiben auf +60 (fuer lege_welle.py,
                                      das von dort direkt ueber das Nest faehrt).

Warum immer aus der Nullstellung: das Gelenkspiel ist richtungsabhaengig —
dieselben Winkel aus verschiedenen Richtungen angefahren ergaben bis zu 14 mm
Unterschied. Immer derselbe Weg heisst immer dasselbe Ergebnis.

    python3 tools/hole_welle.py                         # MANUELL: ENTER vor Schliessen und Heben
    python3 tools/hole_welle.py --auto --nur-hin        # VOLLAUTOMATIK, bleibt auf +60 stehen
    python3 tools/hole_welle.py --nur-anfahren          # bis nachstellen, Greifer offen (messen)
Exit: 0 Welle im Greifer, 1 Fehler (Arm steht, nichts seitlich fahren!), 2 keine Welle
      bekommen, 3 Abbruch durch Benutzer. Letzte Zeile: ERGEBNIS {json}.
"""
import argparse, json, math, os, sys, time
import rclpy
from rclpy.action import ActionClient
from rclpy.node import Node
from geometry_msgs.msg import Pose, PoseStamped
from moveit_msgs.action import MoveGroup
from moveit_msgs.msg import Constraints, JointConstraint, RobotState
from moveit_msgs.srv import GetCartesianPath, GetPositionFK, GetPositionIK, GetStateValidity
from sensor_msgs.msg import JointState
from control_msgs.action import FollowJointTrajectory
from trajectory_msgs.msg import JointTrajectoryPoint
from builtin_interfaces.msg import Duration
sys.path.insert(0, os.path.dirname(os.path.abspath(__file__))); from bahnpruefung import pruefe_bahn  # Stetigkeit (Unfall 13.9.)
from gelenkspiel import nullstellung_rad  # reale Nullstellung (20.9.)

ARM = ['joint2_to_joint1', 'joint3_to_joint2', 'joint4_to_joint3',
       'joint5_to_joint4', 'joint6_to_joint5', 'joint6output_to_joint6']
BASE = 'robot_base'

ap = argparse.ArgumentParser()
ap.add_argument("--punkt", default="trichter_greif")
ap.add_argument("--datei", default=os.path.expanduser("~/ros2_ws/teach_punkte.json"))
ap.add_argument("--ueber", type=float, default=60.0,
                help="[mm] Anfahrhoehe UEBER dem Griffpunkt. 12.9. Tag: 40 lag im Modell in der hohen Wand "
                     "(offene Finger, +5..+45 mm), 60 frei")
ap.add_argument("--heraus", type=float, default=60.0,
                help="[mm] Hoehe ueber dem Griffpunkt nach dem Herausziehen (2026-09-12: 60). "
                     "Erst ab hier darf gedreht/seitlich gefahren werden.")
ap.add_argument("--langsam-mm", type=float, default=30.0, help="[mm] erstes Stueck des Herausziehens langsam + Pause (14.9.)")
ap.add_argument("--langsam-s", type=float, default=5.0, help="[s] Dauer des langsamen Stuecks")
ap.add_argument("--blind-mm", type=float, default=45.0,
                help="[mm] erstes senkrechtes Stueck beim Heben OHNE Kollisionspruefung. Nachts 30; seit dem "
                     "Fingermodell (Keil + Reibgummi) stehen die geschlossenen Finger im Modell bis +40 mm im "
                     "Kragen (12.9. Tag gemessen) -> 45")
ap.add_argument("--blind-ab-mm", type=float, default=50.0,
                help="[mm] letztes senkrechtes Stueck der ABFAHRT auf den Griffpunkt OHNE Kollisionspruefung. "
                     "punkt_pruefen.py 12.9.: mit offenem Greifer streift finger_links im Modell +5..+45 mm "
                     "ueber dem Griff die hohe Wand um 1-3 mm (Modell-TCP dort ~10 mm zu tief). 0 = alles geprueft.")
ap.add_argument("--zu", type=float, default=-0.74,
                help="Greifer-Gelenkwert zu. NUR -0.74 taugt als Sensor (s. oben); der Wert "
                     "'greifer_zu' in der Punktdatei wird nicht mehr gelesen.")
ap.add_argument("--zu-dauer", type=float, default=6.0, help="[s] Schliesszeit")
ap.add_argument("--auf", type=float, default=0.15, help="Greifer-Gelenkwert auf")
ap.add_argument("--leer-prozent", type=float, default=8.0,
                help="Greifer ganz zu: darunter gilt als LEER (gemessen 2026-09-11/12: leer 0 %%)")
ap.add_argument("--rand-prozent", type=float, default=30.0,
                help="Greifer ganz zu: ab hier gilt als 'Auslaufrand gefasst' (Welle am Kopf mit "
                     "Reibgummi: 15-22 %%; der alte Wert 21 lag mitten in der Welle)")
ap.add_argument("--nachgriffe", type=int, default=2,
                help="wie oft bei 'Auslaufrand' auf, --nachgriff-dz hoeher ab Soll, nochmal zu (0 = aus)")
ap.add_argument("--nachgriff-dz", type=float, default=2.0, help="[mm] hoeher je Nachgriff")
ap.add_argument("--wiederholungen", type=int, default=2,
                help="wie oft bei LEER (am Griffpunkt oder nach dem Heben) der Griff wiederholt wird")
ap.add_argument("--auto", action="store_true",
                help="VOLLAUTOMATIK: keine Rueckfragen. Ohne --auto (MANUELL) wird vor dem Schliessen "
                     "und vor dem Heben ENTER verlangt; 'a' + ENTER bricht ab.")
ap.add_argument("--nur-hin", action="store_true",
                help="nach dem Heben auf +heraus stehen bleiben (Uebergabe an lege_welle.py)")
ap.add_argument("--nur-greifen", action="store_true",
                help="nur Schritt 5: Arm steht schon (nachgestellt) am Griffpunkt - Greifer zu, lesen, stehen bleiben")
ap.add_argument("--nur-heben", action="store_true",
                help="nur Schritt 6/6b/7: Welle ist gefasst, Arm steht am Griffpunkt - heben, pruefen, --nur-hin/Nullstellung")
ap.add_argument("--nur-anfahren", action="store_true",
                help="Schritte 1-4 (bis nachstellen), dann stehen bleiben - Greifer offen, "
                     "nichts geschlossen, nichts gehoben. Zum Messen der Greiferneigung.")
ap.add_argument("--ohne-nullstellung", action="store_true",
                help="Schritt 1 auslassen: von der aktuellen Stellung aus mit MoveIt ueber den Trichter")
ap.add_argument("--ohne-nachstellen", action="store_true")
ap.add_argument("--toleranz", type=float, default=0.3, help="Nachstellen: [Grad] je Gelenk")
ap.add_argument("--runden", type=int, default=4, help="Nachstellen: max. Korrekturen")
ap.add_argument("--gain", type=float, default=0.6,
                help="Nachstellen: Anteil des Fehlers je Runde. 1.0 schwang am 2026-09-11")
ap.add_argument("--max-korrektur", type=float, default=4.0, help="Nachstellen: [Grad] max. Vorhalt")
ap.add_argument("--ik-max-grad", type=float, default=35.0,
                help="Rueckfall-IK fuer den Ueber-Punkt: max. Gelenkabstand zur Griffstellung")
ap.add_argument("--setzzeit", type=float, default=4.0, help="[s] Wartezeit nach jeder Fahrt")
ap.add_argument("--tempo", type=float, default=2.0,
                help="Fahrten um diesen Faktor schneller als die Ausgangswerte (Nullstellung 14 s, "
                     "gerade ab 6 s, Heben 3+5 s, IK-Scaling 0.15). Setzzeiten bleiben.")
a = ap.parse_args()
T = lambda sek: max(1.5, sek / a.tempo)

if not os.path.exists(a.datei):
    sys.exit(f"!! {a.datei} fehlt — erst anlernen")
punkte = json.load(open(a.datei))
if a.punkt not in punkte:
    sys.exit(f"!! Punkt '{a.punkt}' nicht in {a.datei}. Vorhanden: {', '.join(sorted(punkte))}")
griff_rad = [float(v) for v in punkte[a.punkt]["rad"]]
print(f"  Punkt '{a.punkt}': " + ", ".join(f"{math.degrees(v):+7.2f}" for v in griff_rad)
      + f"   Greifer zu: {a.zu:+.2f}   [{punkte[a.punkt].get('notiz', '')[:60]}]")

rclpy.init(); node = Node("hole_welle"); js = {"m": None}
node.create_subscription(JointState, "/joint_states", lambda m: js.__setitem__("m", m), 10)
arm = ActionClient(node, FollowJointTrajectory, "/arm_controller/follow_joint_trajectory")
grp = ActionClient(node, FollowJointTrajectory, "/gripper_controller/follow_joint_trajectory")
mg = ActionClient(node, MoveGroup, "/move_action")
cart = node.create_client(GetCartesianPath, "/compute_cartesian_path")
fkc = node.create_client(GetPositionFK, "/compute_fk")
ikc = node.create_client(GetPositionIK, "/compute_ik")
valc = node.create_client(GetStateValidity, "/check_state_validity")
ERGEBNIS = {"punkt": a.punkt, "welle": False, "griffe": 0, "nachgriffe": 0, "prozent": None, "steht_auf": None}


def ende(code):
    print("  ERGEBNIS " + json.dumps(ERGEBNIS))
    node.destroy_node(); rclpy.shutdown(); sys.exit(code)


def frage(was):
    """MANUELL: ENTER = weiter, a = Abbruch (Arm bleibt stehen, Greifer unveraendert)."""
    if a.auto: return
    if not sys.stdin.isatty():
        print(f"  ? {was} — (kein Terminal: weiter)"); return
    try: antwort = input(f"  ? {was} — ENTER = weiter, a = abbrechen: ").strip().lower()
    except EOFError: antwort = ""
    if antwort.startswith("a"):
        print("  Abbruch durch Benutzer. Arm bleibt stehen, Greifer unveraendert."); ende(3)


def spin(sek):
    t0 = time.time()
    while time.time() - t0 < sek: rclpy.spin_once(node, timeout_sec=0.1)


def ist_rad():
    """Encoder-Stellung der sechs Armgelenke [rad] aus einer FRISCHEN /joint_states."""
    js["m"] = None
    for _ in range(150):
        rclpy.spin_once(node, timeout_sec=0.1)
        if js["m"] is not None:
            d = dict(zip(js["m"].name, js["m"].position))
            if all(j in d for j in ARM): return [d[j] for j in ARM]
    print("!! keine /joint_states"); ende(1)


def greifer_prozent():
    """Firmware-Greiferwert [%] aus einer frischen /joint_states; Bereich [-0.74, 0.15]."""
    js["m"] = None
    for _ in range(30):
        rclpy.spin_once(node, timeout_sec=0.1)
        if js["m"] is not None and "gripper_controller" in js["m"].name:
            return (js["m"].position[js["m"].name.index("gripper_controller")] + 0.74) / 0.89 * 100.0
    return None


def fahre_gelenke(rad, dauer, was, setzzeit=None):
    print(f"  -> {was} ({dauer:.0f} s)")
    if not arm.wait_for_server(timeout_sec=15.0): print("!! arm_controller fehlt"); ende(1)
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
    if not grp.wait_for_server(timeout_sec=15.0): print("  !! gripper_controller fehlt — uebersprungen"); return
    g = FollowJointTrajectory.Goal(); g.trajectory.joint_names = ["gripper_controller"]
    pt = JointTrajectoryPoint(); pt.positions = [float(wert)]; pt.time_from_start.sec = int(dauer)
    g.trajectory.points = [pt]
    f = grp.send_goal_async(g); rclpy.spin_until_future_complete(node, f, timeout_sec=20.0)
    gh = f.result()
    if gh is not None and gh.accepted:
        rf = gh.get_result_async(); rclpy.spin_until_future_complete(node, rf, timeout_sec=dauer + 20.0)
    spin(1.0)


VORHALT = [0.0] * 6      # Kommando - Encoder nach dem letzten Nachstellen (14.9., s. heben)


def nachstellen(ziel, was):
    """Ziel + Fehler kommandieren, bis der Encoder das Ziel meldet (tools/nachstellen.py).
    Gedaempft (--gain 0.6): mit vollem Fehler schwang es am 2026-09-11."""
    global VORHALT
    if a.ohne_nachstellen: return
    fmt = lambda v: " ".join(f"{math.degrees(x):+7.2f}" for x in v)
    kommando = list(ziel); fehler = [i - z for i, z in zip(ist_rad(), ziel)]
    nick = lambda f: math.degrees(f[1] + f[2] + f[3]); tol = math.radians(a.toleranz)
    print(f"  nachstellen {was}: Fehler {fmt(fehler)} (Nick J2+J3+J4 {nick(fehler):+.2f} Grad)")
    for runde in range(1, a.runden + 1):
        if all(abs(e) <= tol for e in fehler): print(f"     fertig nach {runde-1} Korrektur(en)"); return
        kommando = [k - a.gain * e for k, e in zip(kommando, fehler)]
        vorhalt = [k - z for k, z in zip(kommando, ziel)]
        if any(abs(v) > math.radians(a.max_korrektur) for v in vorhalt):
            print(f"  !! Vorhalt {fmt(vorhalt)} ueber {a.max_korrektur} Grad — abgebrochen, da stimmt etwas anderes nicht"); return
        print(f"     Runde {runde}: Vorhalt {fmt(vorhalt)}")
        fahre_gelenke(kommando, T(3), f"Korrektur {runde}", setzzeit=4.0)
        fehler = [i - z for i, z in zip(ist_rad(), ziel)]
        print(f"     Fehler {fmt(fehler)} (Nick {nick(fehler):+.2f})")
        VORHALT = [k - i for k, i in zip(kommando, ist_rad())]
    if not all(abs(e) <= tol for e in fehler): print(f"  !! nach {a.runden} Runden noch ausserhalb {a.toleranz} Grad")


def fk_pose(rad):
    if not fkc.wait_for_service(10): print("!! /compute_fk fehlt"); ende(1)
    rs = RobotState(); rs.joint_state.name = list(ARM); rs.joint_state.position = [float(v) for v in rad]
    rq = GetPositionFK.Request(); rq.header.frame_id = BASE; rq.fk_link_names = ["tcp"]; rq.robot_state = rs
    f = fkc.call_async(rq); rclpy.spin_until_future_complete(node, f, timeout_sec=10)
    if f.result() is None or not f.result().pose_stamped: print("!! FK fehlgeschlagen"); ende(1)
    return f.result().pose_stamped[0].pose


def pose_dz(pose, dz_mm):
    p = Pose(); p.orientation = pose.orientation
    p.position.x = pose.position.x; p.position.y = pose.position.y; p.position.z = pose.position.z + dz_mm / 1000.0
    return p


mm = lambda p: (p.position.x * 1000, p.position.y * 1000, p.position.z * 1000)


def bahn_planen(start_rad, ziel_pose, avoid=True, min_fraction=0.95):
    """compute_cartesian_path ab start_rad auf ziel_pose. Rueckgabe: Trajektorie oder None."""
    if not cart.wait_for_service(10): print("!! /compute_cartesian_path fehlt"); ende(1)
    rs = RobotState(); rs.joint_state.name = list(ARM); rs.joint_state.position = [float(v) for v in start_rad]
    cq = GetCartesianPath.Request(); cq.header.frame_id = BASE; cq.group_name = "arm"; cq.link_name = "tcp"
    cq.start_state = rs; cq.waypoints = [ziel_pose]; cq.max_step = 0.005; cq.jump_threshold = 0.0; cq.avoid_collisions = avoid
    # Kollisionspruefung je Bahnpunkt dauert auf dem Nano (Trichter: 183 Koerper) -> 120 s Timeout
    f = cart.call_async(cq); rclpy.spin_until_future_complete(node, f, timeout_sec=120); res = f.result()
    if res is None: print("  !! compute_cartesian_path: Timeout"); return None
    if res.fraction < min_fraction: print(f"  !! nur {res.fraction*100:.0f} % planbar"); return None
    ok_b, txt_b = pruefe_bahn(res.solution.joint_trajectory, ARM, start=list(start_rad))
    if not ok_b: print("  !! " + txt_b + " - NICHT gefahren"); return None
    return res.solution.joint_trajectory


def gerade(soll, ziel_pose, dauer, was, avoid=True, ab_ist=False):
    """Gerade Bahn auf ziel_pose, erster Bahnpunkt = Ist. Geplant ab SOLL (Ziel haengt so nicht
    am Durchhaengen) oder ab IST (ab_ist=True: rein vertikale Stuecke von dort, wo der Arm
    wirklich steht). avoid=False: OHNE Kollisionspruefung (nur das erste Stueck aus dem Trichter).
    Rueckgabe: neue Soll-Gelenke oder None (NICHT gefahren)."""
    start = ist_rad() if ab_ist else soll
    traj = bahn_planen(start, ziel_pose, avoid=avoid)
    if traj is None: print(f"  !! {was}: NICHT gefahren"); return None
    neu = [traj.points[-1].positions[traj.joint_names.index(j)] for j in ARM]
    traj.points[0].positions = ist_rad()
    n = len(traj.points)
    for i, pt in enumerate(traj.points):
        t = dauer * (i + 1) / n; pt.time_from_start.sec = int(t); pt.time_from_start.nanosec = int((t - int(t)) * 1e9)
        pt.velocities = []; pt.accelerations = []
    print(f"  -> {was} (gerade{'' if avoid else ', OHNE Kollisionspruefung'}, {dauer:.0f} s)")
    if not arm.wait_for_server(timeout_sec=15.0): print("!! arm_controller fehlt"); ende(1)
    g = FollowJointTrajectory.Goal(); g.trajectory = traj
    fu = arm.send_goal_async(g); rclpy.spin_until_future_complete(node, fu, timeout_sec=20); gh = fu.result()
    if gh is None or not gh.accepted: print(f"!! {was} abgelehnt"); ende(1)
    rf = gh.get_result_async(); rclpy.spin_until_future_complete(node, rf, timeout_sec=dauer + 40)
    spin(a.setzzeit)
    return neu


def ik_nahe(pose, seed, max_grad):
    """IK fuer pose, die zur seed-Stellung naechste Loesung (TRAC-IK streut)."""
    if not ikc.wait_for_service(10): print("!! /compute_ik fehlt"); ende(1)
    best, bd = None, 1e9
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
        print(f"!! keine IK-Loesung nahe der Griffstellung ({'-' if best is None else f'{math.degrees(bd):.0f} Grad'})"); ende(1)
    return best


def moveit_gelenkziel(rad, was):
    print(f"  -> MoveIt (kollisionsgeprueft): {was}")
    if not mg.wait_for_server(20): print("!! /move_action fehlt"); ende(1)
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


def kollisionsfrei(rad, greifer):
    if not valc.wait_for_service(10): print("!! /check_state_validity fehlt"); ende(1)
    rq = GetStateValidity.Request(); rq.group_name = "arm"
    rq.robot_state.joint_state.name = list(ARM) + ["gripper_controller"]
    rq.robot_state.joint_state.position = [float(v) for v in rad] + [float(greifer)]
    f = valc.call_async(rq); rclpy.spin_until_future_complete(node, f, timeout_sec=20); r = f.result()
    return (r is not None and r.valid), ([] if r is None else sorted({f"{c.contact_body_1}x{c.contact_body_2}" for c in r.contacts}))


def ueber_gelenke(dz_mm):
    """Ueber-Punkt: ZUERST per gerader Bahn AUS der Griffstellung nach oben (gleicher IK-Zweig
    wie angelernt) - OHNE Kollisionspruefung unterwegs (im Trichter streift das Modell +5..+45 mm,
    s. --blind-ab-mm), dafuer wird der Ueber-Punkt selbst geprueft; erst wenn das nicht geht per IK
    nahe am Seed."""
    traj = bahn_planen(griff_rad, pose_dz(griff_pose, dz_mm), avoid=False, min_fraction=0.99)
    if traj is not None:
        rad = [traj.points[-1].positions[traj.joint_names.index(j)] for j in ARM]
        ok, kont = kollisionsfrei(rad, a.auf)
        print(f"  Ueber-Punkt per gerader Bahn aus der Griffstellung (max Gelenkabstand "
              f"{math.degrees(max(abs(u - g) for u, g in zip(rad, griff_rad))):.1f} Grad): "
              + ("kollisionsfrei" if ok else "KOLLISION " + " ".join(kont)))
        if ok: return rad
        print(f"  !! Ueber-Punkt +{dz_mm:.0f} mm steht im Modell in Kollision - hoeher waehlen (--ueber)"); ende(1)
    print("  (gerade Bahn nach oben nicht planbar - IK nahe am Seed)")
    return ik_nahe(pose_dz(griff_pose, dz_mm), griff_rad, a.ik_max_grad)


def lese(was):
    spin(2.0); proz = greifer_prozent()
    ERGEBNIS["prozent"] = None if proz is None else round(proz, 1)
    print(f"     Greifer meldet {'?' if proz is None else f'{proz:.0f} %'} ({was})")
    return proz


def auf_den_griffpunkt(soll_von):
    """Von soll_von (ueber dem Trichter) gerade ab auf den Griffpunkt + nachstellen. Greifer OFFEN
    - offen stehen die Finger im Modell frei, die Bahn ist kollisionsgeprueft."""
    global soll
    blind = min(a.blind_ab_mm, a.ueber)
    if blind > 0:
        if a.ueber > blind:
            neu = gerade(soll_von, pose_dz(griff_pose, blind), T(3), f"gerade ab auf Griffpunkt +{blind:.0f} mm (geprueft)")
            if neu is None:
                print("  !! Abfahrt nicht planbar. Arm steht ueber dem Trichter."); ende(1)
            soll_von = neu
        frage(f"Letzte {blind:.0f} mm OHNE Kollisionspruefung senkrecht auf den Griffpunkt (langsam). Finger frei?")
        neu = gerade(soll_von, griff_pose, T(6), f"gerade ab auf den Griffpunkt, letzte {blind:.0f} mm", avoid=False)
    else:
        neu = gerade(soll_von, griff_pose, T(6), "gerade ab auf den Griffpunkt")
    if neu is None:
        print("  !! Griffpunkt nicht erreichbar. Arm steht ueber dem Trichter."); ende(1)
    soll = neu
    nachstellen(soll, "auf den Griffpunkt")
    ist = fk_pose(ist_rad())
    print(f"     Ist-TCP (Modell aus Encoder): ({mm(ist)[0]:+.1f}, {mm(ist)[1]:+.1f}, {mm(ist)[2]:.1f}) mm")


def greifen():
    """Schritt 5: ganz zu, lesen, ggf. Nachgriff. Rueckgabe: True = Welle am Kopf."""
    global soll
    for versuch in range(a.nachgriffe + 1):
        greifer(a.zu, a.zu_dauer, "ganz zu (-0.74, greift die Welle)")
        ERGEBNIS["griffe"] += 1
        proz = lese("am Griffpunkt")
        if proz is None:
            print("     Greiferwert nicht lesbar — weiter ohne Pruefung"); return True
        if proz < a.leer_prozent:
            print(f"     -> LEER (< {a.leer_prozent:.0f} %): nichts gefasst"); return False
        if proz < a.rand_prozent:
            print("     -> Welle am Kopf gefasst"); return True
        if versuch >= a.nachgriffe:
            print(f"  !! immer noch >= {a.rand_prozent:.0f} % (Auslaufrand?) — kein Nachgriff mehr, es wird trotzdem gehoben")
            return True
        print(f"     -> Auslaufrand gefasst (>= {a.rand_prozent:.0f} %): auf, {a.nachgriff_dz:+.1f} mm hoeher ab Soll, nochmal zu")
        ERGEBNIS["nachgriffe"] += 1
        greifer(a.auf, 3, "auf")
        neu = gerade(soll, pose_dz(fk_pose(soll), a.nachgriff_dz), T(3), f"Nachgriff {versuch+1}", ab_ist=True)
        if neu is None:
            print("  !! Verschiebung nicht moeglich — es wird auf der alten Hoehe gegriffen"); continue
        soll = neu
        nachstellen(soll, f"nach Nachgriff {versuch+1}")
    return True


def heben():
    """Schritt 6: erst --blind-mm rein vertikal ab IST ohne Kollisionspruefung, dann geprueft auf
    Griffpunkt + --heraus. Orientierung bleibt (Bahn), J6 dreht nicht."""
    global soll
    if a.blind_mm > 0:
        # 14.9. (Benutzer, Video): jedes NEUE Bahnstueck ab Encoder liess den Arm erst um den Servo-Vorhalt (Totband,
        # 1-4 Grad) zur Basis hin "setzen" - die Welle schlug dabei an der kleinen robotseitigen Wand an und verbog sich.
        # Darum: EINE senkrechte Bahn ab Encoder-Stellung, der Vorhalt aus dem Nachstellen wird auf ALLE Punkte
        # addiert (Kommando bleibt konsistent, der Arm faehrt parallel zu sich selbst), erste --langsam-mm langsam,
        # dann normal - ohne Zwischenstopp und ohne Neuplanen.
        ist0 = ist_rad(); p0 = fk_pose(ist0)
        traj = bahn_planen(ist0, pose_dz(p0, a.blind_mm), avoid=False)
        if traj is None:
            print("  !! senkrechtes Herausziehen nicht planbar — Arm steht noch im Trichter.")
            print("     NICHT seitlich fahren, sonst verkantet die Welle und der Trichter kippt."); ende(1)
        n = len(traj.points); erst = min(a.langsam_mm, a.blind_mm); n1 = max(1, int(round(n * erst / a.blind_mm)))
        vh = list(VORHALT) if max(abs(v) for v in VORHALT) < math.radians(a.max_korrektur + 0.5) else [0.0] * 6
        offs = [dict(zip(ARM, vh)).get(j, 0.0) for j in traj.joint_names]
        for i, pt in enumerate(traj.points):
            pt.positions = [float(v) + o for v, o in zip(pt.positions, offs)]
            t = a.langsam_s * (i + 1) / n1 if i < n1 else a.langsam_s + T(3) * (i + 1 - n1) / max(1, n - n1)
            pt.time_from_start.sec = int(t); pt.time_from_start.nanosec = int((t - int(t)) * 1e9); pt.velocities = []; pt.accelerations = []
        print(f"  -> heben {a.blind_mm:.0f} mm senkrecht in EINER Bahn: erste {erst:.0f} mm in {a.langsam_s:.0f} s, Rest {T(3):.0f} s; "
              f"Vorhalt {' '.join(f'{math.degrees(v):+.1f}' for v in vh)} Grad auf alle Punkte (OHNE Kollisionspruefung)")
        if not arm.wait_for_server(timeout_sec=15.0): print("!! arm_controller fehlt"); ende(1)
        g = FollowJointTrajectory.Goal(); g.trajectory = traj
        fu = arm.send_goal_async(g); rclpy.spin_until_future_complete(node, fu, timeout_sec=20); gh = fu.result()
        if gh is None or not gh.accepted: print("!! Herausziehen abgelehnt"); ende(1)
        rf = gh.get_result_async(); rclpy.spin_until_future_complete(node, rf, timeout_sec=a.langsam_s + T(3) + 40)
        spin(a.setzzeit)
        soll = [traj.points[-1].positions[traj.joint_names.index(j)] - dict(zip(ARM, vh)).get(j, 0.0) for j in ARM]
    neu = gerade(soll, pose_dz(griff_pose, a.heraus), T(5), f"heben auf Griffpunkt +{a.heraus:.0f} mm")
    if neu is None:
        print("  !! geprueftes Heben nicht planbar — Arm steht teilweise im Trichter. NICHT seitlich fahren."); ende(1)
    soll = neu


# ------------------------------------------------------------------ Ablauf
ist_rad()
griff_pose = fk_pose(griff_rad)
print(f"  Griffpunkt TCP-Modell ({mm(griff_pose)[0]:+.1f}, {mm(griff_pose)[1]:+.1f}, {mm(griff_pose)[2]:.1f}) mm")

if a.nur_greifen:
    soll = list(griff_rad)
    print("\n[5] --nur-greifen: Greifer ganz zu + lesen (Arm steht am Griffpunkt)")
    ok = greifen(); ende(0 if ok else 2)
if a.nur_heben:
    soll = list(griff_rad)
    print("\n[6] --nur-heben: SENKRECHT heraus")
    heben()
    print("\n[6b] Haltepruefung (Greifer ist ganz zu)")
    proz = lese("nach dem Heben")
    if proz is not None and proz < a.leer_prozent:
        print("     -> LEER: Welle beim Heben verloren"); ende(2)
    ERGEBNIS["welle"] = True; print("     -> WELLE DA")
    if a.nur_hin: print(f"\n[7] --nur-hin: Arm bleibt auf Griffpunkt +{a.heraus:.0f} mm stehen, Greifer ZU")
    else: print("\n[7] zurueck zur Nullstellung — Greifer bleibt ZU"); moveit_gelenkziel(nullstellung_rad(), "Nullstellung (kollisionsgeprueft)")
    ende(0)

if a.ohne_nullstellung:
    print("\n[1] Nullstellung: ausgelassen (--ohne-nullstellung)")
else:
    print("\n[1] Nullstellung"); moveit_gelenkziel(nullstellung_rad(), "Nullstellung (kollisionsgeprueft)")

print("\n[2] Greifer auf"); greifer(a.auf, 3, "auf")

print(f"\n[3] ueber den Griffpunkt (+{a.ueber:.0f} mm)")
ueber_rad = ueber_gelenke(a.ueber)
moveit_gelenkziel(ueber_rad, f"ueber '{a.punkt}'")
soll = list(ueber_rad)

print("\n[4] gerade ab auf den Griffpunkt + nachstellen")
auf_den_griffpunkt(soll)

if a.nur_anfahren:
    print("\n  --nur-anfahren: Arm bleibt in der Greifstellung, Greifer offen. Jetzt messen."); ende(0)

for durchlauf in range(1, a.wiederholungen + 2):
    print(f"\n[5] Greifer ganz zu + lesen" + (f"  (Wiederholung {durchlauf-1})" if durchlauf > 1 else ""))
    frage("Finger stehen QUER zur Wand, frei vom Auslaufrand? Dann schliessen")
    gefasst = greifen()
    if gefasst:
        print("\n[6] SENKRECHT heraus")
        frage("Heben?")
        heben()
        print("\n[6b] Haltepruefung (Greifer ist ganz zu)")
        proz = lese("nach dem Heben")
        if proz is None or proz >= a.leer_prozent:
            ERGEBNIS["welle"] = True; print("     -> WELLE DA"); break
        print("     -> LEER: Welle beim Heben verloren")
    if durchlauf > a.wiederholungen:
        print(f"\n  !! Welle nach {durchlauf} Griff(en) nicht bekommen — Abbruch, Arm bleibt stehen."); ende(2)
    print(f"\n  -> Wiederholung {durchlauf}: Greifer auf, zurueck auf den Griffpunkt")
    frage("Wiederholen?")
    greifer(a.auf, 3, "auf")
    if gefasst:                                   # wir stehen auf +heraus: gerade ab (offen, geprueft)
        auf_den_griffpunkt(soll)
    else:                                         # wir stehen am Griffpunkt (evtl. +2 mm Nachgriff)
        soll = list(griff_rad); fahre_gelenke(soll, T(3), "zurueck auf den Griffpunkt")
        nachstellen(soll, "auf den Griffpunkt (Wiederholung)")

if a.nur_hin:
    ERGEBNIS["steht_auf"] = f"{a.punkt} +{a.heraus:.0f} mm"
    print(f"\n[7] --nur-hin: Arm bleibt auf Griffpunkt +{a.heraus:.0f} mm stehen, Greifer ZU")
else:
    print("\n[7] zurueck zur Nullstellung — Greifer bleibt ZU")
    moveit_gelenkziel(nullstellung_rad(), "Nullstellung (kollisionsgeprueft)"); ERGEBNIS["steht_auf"] = "Nullstellung"

ist = fk_pose(ist_rad())
print(f"\n  fertig. TCP: x {mm(ist)[0]:+.1f}  y {mm(ist)[1]:+.1f}  z {mm(ist)[2]:+.1f} mm")
print("  Der Greifer wurde NICHT geoeffnet — die Welle haengt noch darin.")
ende(0)
