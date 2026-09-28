#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""Welle aus der BEREITSTELLUNGSSCHALE holen (2026-09-12 Nacht Skelett; 2026-09-13 01:5x ERSTER GRIFF).

Gegenstueck zu hole_welle.py (Trichter, angelernter Punkt): hier kommt das Ziel aus der KAMERA
(Detektor /welle/ziel) oder von Hand (--x --y --yaw), die Welle LIEGT (Flansch Ø 13.5 mm),
der Griff ist senkrecht von oben, Finger schliessen QUER zur Wellenachse.

Ablauf (dieselbe Ordnung wie im Trichter, weil das Gelenkspiel richtungsabhaengig ist):

  0. Ziel        --ziel-datei welle_ziel.json (welle_finden_schale.py: griff_mm + yaw_grad + Farbe)
                 ODER /welle/ziel (PoseStamped, tcp +Y nach unten, Yaw = Wellenachse) ODER --x --y
                 [--z] --yaw. z: NICHT aus der Tiefe (bei 13 mm duennen Teilen liest sie den
                 Boden), sondern Schalen-Innenboden (TF `schale`) + --griff-mm (6.5 = Griff 13.9.).
                 KAMERA -> MODELL (2026-09-13): der Roboter faehrt im Modellraum, der ~8 % radial
                 neben der Kamerawelt liegt (16 Nester, kamera_modell_platte.json). Massband 13.9.
                 Abend: die KAMERA stimmt (nest_53 real 210 mm, Kamera 209.6, FK 226.8) - das URDF
                 rechnet den TCP aus den Encoderwinkeln 15-17 mm zu weit aussen (Getriebespiel/
                 Durchhang hinter den Encodern). Die Abbildung korrigiert also den ROBOTER. --abbildung
                 aehnlichkeit (Default seit 14.9.: s 1.06, Drehung -1.5 Grad, t) - radial (x1.0825)
                 stimmte nur in -y-Richtung (alle 16 Nester liegen dort) und setzte tangential
                 liegende Wellen ~10 mm daneben (14.9. Nacht, 3 Griffe: +10/+10/0 mm entlang der
                 Achse; aehnlichkeit sagt +7/+8/-3.5 voraus, affin ueberzieht). radial/affin bleiben
                 waehlbar; keine = Ziel ist schon
                 Modellraum. Gilt fuer ALLE Quellen, auch --x/--y (die kommen in der Praxis aus der
                 Kamera). Dazu optional der empirische Rest-Versatz schale_versatz.json (dx dy dz)
                 und --dx/--dy/--dz.
                 Proben: r_robot 150-220 mm (Bodengriff, gemessen 12.9.), Greifer-Fussabdruck
                 in der Schale (wie im Detektor), Ziel im Kamerabild bestaetigen (MANUELL).
  1. Nullstellung (--ohne-nullstellung: von hier)
  2. Greifer auf --auf (-0.305 = 25 mm; nicht ganz auf: schmalerer Fussabdruck an der Wand)
  3. UEBER das Ziel (+--ueber 60 mm): IK senkrecht, Loesung in der HALTUNG der angelernten
     Punkte (Ellbogen wie nest_33: J3 < 0, |J5| < 45), kollisionsgeprueft, MoveIt dorthin
  4. GERADE AB auf den Griffpunkt (kollisionsgeprueft - Schale und Ablageplatte sind im
     Modell, die Welle nicht), letzte --blind-ab-mm (0) ungeprueft, dann NACHSTELLEN
  5. Greifer GANZ zu (-0.74) + lesen: < --leer-prozent (8) = nichts -> auf, nochmal
     (--wiederholungen); Erwartung am Flansch: ? % (noch nicht gemessen -> ERGEBNIS)
  6. SENKRECHT heben auf Ziel + --heraus (60), Haltepruefung
  7. --nur-hin: stehen bleiben (Uebergabe an die Trichter-Ablage) sonst Nullstellung, ZU

    python3 tools/hole_aus_schale.py --ziel-datei ~/ros2_ws/welle_ziel.json   # MANUELL, aus welle_finden_schale
    python3 tools/hole_aus_schale.py --x 155.3 --y=-54.3 --yaw=-153  # Kamerawerte von Hand (Abbildung radial)
    python3 tools/hole_aus_schale.py --ziel-topic /welle/ziel        # Detektor
    python3 tools/hole_aus_schale.py ... --nur-anfahren               # bis Nachstellen (Versatz messen!)
    python3 tools/hole_aus_schale.py ... --auto --nur-hin             # VOLLAUTOMATIK
Stufen wie hole_welle.py: --nur-anfahren / --nur-greifen / --nur-heben. Exit 0 Welle da,
1 Fehler (Arm steht), 2 keine Welle, 3 Abbruch, 4 Ziel abgelehnt (Proben). ERGEBNIS {json}.

Rest-Versatz messen (B): mit --nur-anfahren ueber einer bekannten Welle stehen, Ist-TCP gegen
das abgebildete Ziel messen -> schale_versatz.json schreiben (tools/versetze_soll.py hilft).
"""
import argparse, json, math, os, sys, time
import numpy as np
import rclpy
from rclpy.action import ActionClient
from rclpy.node import Node
from geometry_msgs.msg import Pose, PoseStamped, Quaternion
from moveit_msgs.action import MoveGroup
from moveit_msgs.msg import Constraints, JointConstraint, RobotState
from moveit_msgs.srv import GetCartesianPath, GetPositionFK, GetPositionIK, GetStateValidity
from sensor_msgs.msg import JointState
from control_msgs.action import FollowJointTrajectory
from trajectory_msgs.msg import JointTrajectoryPoint
from builtin_interfaces.msg import Duration
import tf2_ros
sys.path.insert(0, os.path.dirname(os.path.abspath(__file__))); from bahnpruefung import pruefe_bahn  # Stetigkeit (Unfall 13.9.)
from gelenkspiel import nullstellung_rad  # reale Nullstellung (20.9.)

ARM = ['joint2_to_joint1', 'joint3_to_joint2', 'joint4_to_joint3',
       'joint5_to_joint4', 'joint6_to_joint5', 'joint6output_to_joint6']
BASE = 'robot_base'
# Schale (wie schale_finden_kontur.py / Detektor)
R_INNEN, R_AUSSEN, WAND, WINKEL = 0.1869, 0.3019, 0.002, math.radians(45.3)

ap = argparse.ArgumentParser()
z_ = ap.add_argument_group("Ziel")
z_.add_argument("--ziel-topic", default="", help="PoseStamped-Topic des Detektors (z.B. /welle/ziel); leer = --x/--y")
z_.add_argument("--ziel-wartezeit", type=float, default=15.0, help="[s] auf das Topic warten")
z_.add_argument("--ziel-datei", default="", help="welle_ziel.json von welle_finden_schale.py (griff_mm, yaw_grad, koerper, streifen)")
z_.add_argument("--x", type=float, default=None, help="[mm] Griffpunkt in robot_base (Hand-Ziel)")
z_.add_argument("--y", type=float, default=None, help="[mm]")
z_.add_argument("--z", type=float, default=None, help="[mm] TCP-Hoehe; Vorgabe Schalen-Innenboden + --griff-mm")
z_.add_argument("--yaw", type=float, default=None, help="[Grad] Richtung der WELLENACHSE in robot_base (Finger schliessen quer dazu)")
z_.add_argument("--griff-mm", type=float, default=2.0,
                help="[mm] Modell-TCP (= Spitze ZU, 135 mm) ueber dem Innenboden. 6.5 = Griffe 13./14.9. - dabei standen die Spitzen "
                     "beim Zupacken (Oeffnung 13.5 -> Laenge ~132) 9 mm ueber dem Boden = ueber der Wellenachse (6.75): nur die oberen "
                     "4 mm der Welle zwischen den Fingerspitzen -> schwache Griffe. 2.0: Spitzen beim Zupacken ~5 mm ueber dem Boden, "
                     "Achse 2 mm ueber den Spitzen (Finger umfassen den Flansch). Offen (25 mm, Laenge 125) stehen die Spitzen 10 mm hoeher.")
z_.add_argument("--abbildung", default="aehnlichkeit", choices=["radial", "aehnlichkeit", "affin", "keine"],
                help="Kamera -> Modell aus --abbildung-datei: radial = Ziel um 'skalierung' um die Roboterachse strecken (13.9. Griff), "
                     "aehnlichkeit = s/theta/t, affin = A/b, keine = Ziel ist schon Modellraum")
z_.add_argument("--abbildung-datei", default=os.path.expanduser("~/ros2_ws/kamera_modell_platte.json"))
z_.add_argument("--versatz-datei", default=os.path.expanduser("~/ros2_ws/schale_versatz.json"),
                help="{dx, dy, dz} [mm], Kamera -> Fahrt, empirisch (offen B). Fehlt = 0 + Warnung")
z_.add_argument("--dx", type=float, default=0.0, help="[mm] zusaetzlich zum Versatz")
z_.add_argument("--dy", type=float, default=0.0); z_.add_argument("--dz", type=float, default=0.0)
z_.add_argument("--r-min", type=float, default=150.0)
z_.add_argument("--r-max", type=float, default=250.0, help="14.9.: 220 -> 250 wie welle_finden_schale (dort IK-Probe); ob der Griff geht, entscheidet die IK")
z_.add_argument("--ohne-schalenprobe", action="store_true", help="Fussabdruck-/Radiusproben nur melden, nicht abbrechen")
ap.add_argument("--ueber", type=float, default=60.0, help="[mm] Anfahrhoehe ueber dem Griffpunkt")
ap.add_argument("--heraus", type=float, default=60.0, help="[mm] Hebehoehe ueber dem Griffpunkt")
ap.add_argument("--blind-ab-mm", type=float, default=10.0,
                help="[mm] letztes Stueck der Abfahrt ohne Kollisionspruefung. 14.9.: 0 -> 10, weil das Modell die Finger bei 17 mm "
                     "Oeffnung ~5 mm je Seite zu weit auseinander zeichnet (Upstream-Kinematik) und die Abfahrt bei wandnahen Wellen "
                     "bei 92 % mit finger x schale (Wandoberkante) abbrach, obwohl real ~8 mm Luft sind. Die Wandnaehe prueft "
                     "welle_finden_schale (wand_quer >= Oeffnung/2 + 9 + 2 - 6.75)")
ap.add_argument("--blind-mm", type=float, default=0.0, help="[mm] erstes Stueck des Hebens ohne Kollisionspruefung")
ap.add_argument("--zu", type=float, default=-0.74, help="Greifer zu (NUR -0.74 taugt als Sensor)")
ap.add_argument("--zu-dauer", type=float, default=6.0)
ap.add_argument("--auf", type=float, default=None,
                help="Greifer auf (Gelenkwert). Vorgabe: aus --auf-mm bzw. 'oeffnung_mm' der Zieldatei ueber die Messkurve 9.9. "
                     "(-0.562 = 6, -0.384 = 21, -0.305 = 25, -0.206 = 30, 0.15 = 40 mm). Griffe 13.9. liefen mit -0.15 = 40 mm")
ap.add_argument("--auf-mm", type=float, default=None, help="[mm] Fingeroeffnung (Spitzen); Vorgabe = oeffnung_mm der Zieldatei, sonst 25")
ap.add_argument("--leer-prozent", type=float, default=3.0,
                help="[%] Greifer-Rueckmeldung, unter der NICHTS gehalten wird. 14.9.: 8 -> 3, weil eine schief (am Kopf/Schaft "
                     "O 7) gehaltene Welle nur 7 % meldete und als 'verloren' galt, obwohl sie in den Fingern hing (leer = 0 %)")
ap.add_argument("--unsicher-prozent", type=float, default=8.0, help="[%] darunter: 'Welle da, aber unsicher/schief' (Hinweis, kein Abbruch)")
ap.add_argument("--wiederholungen", type=int, default=2)
ap.add_argument("--fuss-lang", type=float, default=None, help="[m] Greifer-Fussabdruck quer zur Achse bei --auf; Vorgabe = Oeffnung + 4 mm (Griff 13.9.: 0.044 bei 40 mm)")
ap.add_argument("--fuss-kurz", type=float, default=0.020)
ap.add_argument("--wand-abstand", type=float, default=0.004)
ap.add_argument("--referenz-punkt", default="nest_33", help="Haltung, der die IK-Loesung nahe sein soll (teach_punkte.json)")
ap.add_argument("--datei", default=os.path.expanduser("~/ros2_ws/teach_punkte.json"))
ap.add_argument("--auto", action="store_true", help="VOLLAUTOMATIK: keine Rueckfragen")
ap.add_argument("--nur-pruefen", action="store_true",
                help="NICHTS fahren: Ziel + Proben, IK-Haltung fuer den Ueber-Punkt, Kollision, Bahn ab (plan only)")
ap.add_argument("--nur-hin", action="store_true"); ap.add_argument("--nur-anfahren", action="store_true")
ap.add_argument("--nur-greifen", action="store_true"); ap.add_argument("--nur-heben", action="store_true")
ap.add_argument("--ohne-nullstellung", action="store_true"); ap.add_argument("--ohne-nachstellen", action="store_true")
ap.add_argument("--toleranz", type=float, default=0.3); ap.add_argument("--runden", type=int, default=4)
ap.add_argument("--gain", type=float, default=0.6); ap.add_argument("--max-korrektur", type=float, default=4.0)
ap.add_argument("--setzzeit", type=float, default=4.0); ap.add_argument("--tempo", type=float, default=2.0)
a = ap.parse_args()
T = lambda sek: max(1.5, sek / a.tempo)

# Fingeroeffnung (Spitzen) -> Gelenkwert, Messkurve 2026-09-14 MIT Silikonpads (Kumpass): 0 % = 9 mm (Pads liegen an),
# 31 % = 17, 49 % = 25, 100 % = 40 mm; Gelenk = -0.74 + 0.89 * %/100. Linear dazwischen. (Kurve 9.9. ohne Pads: 0/6/21/30/35/40.)
OEFFNUNG_KURVE = [(9.0, -0.740), (17.0, -0.464), (25.0, -0.304), (40.0, 0.150)]
# Flansch -> Fingerspitze je Oeffnung (14.9.): die Spitzen wandern beim Schliessen NACH VORN (Viergelenk).
# Modell-tcp = 135 mm = ZU. Beim Zupacken auf die Welle (Ø 13.5) stoppt das Schliessen bei ~13.5 mm -> ~132 mm.
SPITZENLAENGE = [(9.0, 135.0), (17.0, 130.0), (25.0, 125.0), (40.0, 115.0)]
def spitze_mm(oeffnung_mm):
    o = min(40.0, max(9.0, oeffnung_mm))
    for (m0, l0), (m1, l1) in zip(SPITZENLAENGE, SPITZENLAENGE[1:]):
        if o <= m1: return l0 + (l1 - l0) * (o - m0) / (m1 - m0)
    return 115.0
def auf_aus_mm(mm_):
    mm_ = min(40.0, max(9.0, mm_))
    for (m0, j0), (m1, j1) in zip(OEFFNUNG_KURVE, OEFFNUNG_KURVE[1:]):
        if mm_ <= m1: return j0 + (j1 - j0) * (mm_ - m0) / (m1 - m0)
    return 0.150
if a.auf_mm is None and a.ziel_datei:
    try: a.auf_mm = float(json.load(open(a.ziel_datei)).get("oeffnung_mm") or 25.0)
    except Exception: a.auf_mm = 25.0
if a.auf_mm is None: a.auf_mm = 25.0
if a.auf is None: a.auf = round(auf_aus_mm(a.auf_mm), 3)
if a.fuss_lang is None: a.fuss_lang = (a.auf_mm + 4.0) / 1e3
print(f"  Greifer auf: {a.auf_mm:.0f} mm -> Gelenk {a.auf:+.3f}, Fussabdruck {a.fuss_lang*1e3:.0f} x {a.fuss_kurz*1e3:.0f} mm; "
      f"Spitzen offen {135 - spitze_mm(a.auf_mm):.0f} mm ueber dem Modell-TCP, beim Zupacken (13.5 mm) {135 - spitze_mm(13.5):.0f} mm "
      f"-> Spitzen {a.griff_mm + 135 - spitze_mm(13.5):.1f} mm ueber dem Boden (Wellenachse 6.75)")

rclpy.init(); node = Node("hole_aus_schale"); js = {"m": None}
node.create_subscription(JointState, "/joint_states", lambda m: js.__setitem__("m", m), 10)
arm = ActionClient(node, FollowJointTrajectory, "/arm_controller/follow_joint_trajectory")
grp = ActionClient(node, FollowJointTrajectory, "/gripper_controller/follow_joint_trajectory")
mg = ActionClient(node, MoveGroup, "/move_action")
cart = node.create_client(GetCartesianPath, "/compute_cartesian_path")
fkc = node.create_client(GetPositionFK, "/compute_fk")
ikc = node.create_client(GetPositionIK, "/compute_ik")
valc = node.create_client(GetStateValidity, "/check_state_validity")
puffer = tf2_ros.Buffer(); tf2_ros.TransformListener(puffer, node)
ERGEBNIS = {"ziel_mm": None, "ziel_kamera_mm": None, "abbildung": None, "yaw_grad": None, "versatz_mm": None,
            "koerper": None, "streifen": None, "welle": False, "griffe": 0, "prozent": None, "steht_auf": None, "proben": []}


# ------------------------------------------------------------------ Grundbausteine (wie hole_welle.py)
def ende(code):
    print("  ERGEBNIS " + json.dumps(ERGEBNIS))
    node.destroy_node(); rclpy.shutdown(); sys.exit(code)


def frage(was):
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
    js["m"] = None
    for _ in range(150):
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


def nachstellen(ziel, was):
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
            print(f"  !! Vorhalt {fmt(vorhalt)} ueber {a.max_korrektur} Grad — abgebrochen"); return
        print(f"     Runde {runde}: Vorhalt {fmt(vorhalt)}")
        fahre_gelenke(kommando, T(3), f"Korrektur {runde}", setzzeit=4.0)
        fehler = [i - z for i, z in zip(ist_rad(), ziel)]
        print(f"     Fehler {fmt(fehler)} (Nick {nick(fehler):+.2f})")
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
    if not cart.wait_for_service(10): print("!! /compute_cartesian_path fehlt"); ende(1)
    rs = RobotState(); rs.joint_state.name = list(ARM); rs.joint_state.position = [float(v) for v in start_rad]
    cq = GetCartesianPath.Request(); cq.header.frame_id = BASE; cq.group_name = "arm"; cq.link_name = "tcp"
    cq.start_state = rs; cq.waypoints = [ziel_pose]; cq.max_step = 0.005; cq.jump_threshold = 0.0; cq.avoid_collisions = avoid
    f = cart.call_async(cq); rclpy.spin_until_future_complete(node, f, timeout_sec=120); res = f.result()
    if res is None: print("  !! compute_cartesian_path: Timeout"); return None
    if res.fraction < min_fraction: print(f"  !! nur {res.fraction*100:.0f} % planbar"); return None
    ok_b, txt_b = pruefe_bahn(res.solution.joint_trajectory, ARM, start=list(start_rad))
    if not ok_b: print("  !! " + txt_b + " - NICHT gefahren"); return None
    return res.solution.joint_trajectory


def gerade(soll, ziel_pose, dauer, was, avoid=True, ab_ist=False):
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


def kollisionsfrei(rad, greifer_wert):
    if not valc.wait_for_service(10): print("!! /check_state_validity fehlt"); ende(1)
    rq = GetStateValidity.Request(); rq.group_name = "arm"
    rq.robot_state.joint_state.name = list(ARM) + ["gripper_controller"]
    rq.robot_state.joint_state.position = [float(v) for v in rad] + [float(greifer_wert)]
    f = valc.call_async(rq); rclpy.spin_until_future_complete(node, f, timeout_sec=20); r = f.result()
    return (r is not None and r.valid), ([] if r is None else sorted({f"{c.contact_body_1}x{c.contact_body_2}" for c in r.contacts}))


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


# ------------------------------------------------------------------ Orientierung / Schale
def quat_senkrecht(yaw_achse):
    """tcp +Y = (0,0,-1), tcp +Z = Wellenachse (cos, sin, 0), tcp +X = Y x Z (Schliessachse quer)."""
    eZ = np.array([math.cos(yaw_achse), math.sin(yaw_achse), 0.0]); eY = np.array([0.0, 0.0, -1.0])
    eX = np.cross(eY, eZ); R = np.column_stack([eX, eY, eZ])
    t = R[0, 0] + R[1, 1] + R[2, 2]
    if t > 0:
        s = math.sqrt(t + 1.0) * 2; w = 0.25 * s; x = (R[2, 1] - R[1, 2]) / s; y = (R[0, 2] - R[2, 0]) / s; z = (R[1, 0] - R[0, 1]) / s
    elif R[0, 0] > R[1, 1] and R[0, 0] > R[2, 2]:
        s = math.sqrt(1.0 + R[0, 0] - R[1, 1] - R[2, 2]) * 2; w = (R[2, 1] - R[1, 2]) / s; x = 0.25 * s; y = (R[0, 1] + R[1, 0]) / s; z = (R[0, 2] + R[2, 0]) / s
    elif R[1, 1] > R[2, 2]:
        s = math.sqrt(1.0 + R[1, 1] - R[0, 0] - R[2, 2]) * 2; w = (R[0, 2] - R[2, 0]) / s; x = (R[0, 1] + R[1, 0]) / s; y = 0.25 * s; z = (R[1, 2] + R[2, 1]) / s
    else:
        s = math.sqrt(1.0 + R[2, 2] - R[0, 0] - R[1, 1]) * 2; w = (R[1, 0] - R[0, 1]) / s; x = (R[0, 2] + R[2, 0]) / s; y = (R[1, 2] + R[2, 1]) / s; z = 0.25 * s
    n = math.sqrt(x * x + y * y + z * z + w * w)
    return Quaternion(x=x / n, y=y / n, z=z / n, w=w / n)


def yaw_aus_quat(q):
    """Wellenachse (tcp +Z) aus dem Detektor-Quaternion -> Winkel in der xy-Ebene."""
    x, y, z, w = q.x, q.y, q.z, q.w
    zx = 2 * (x * z + w * y); zy = 2 * (y * z - w * x)      # dritte Spalte der Rotationsmatrix
    return math.atan2(zy, zx)


def schale_lage():
    ende_t = time.time() + 10
    while time.time() < ende_t and not puffer.can_transform(BASE, "schale", rclpy.time.Time()):
        rclpy.spin_once(node, timeout_sec=0.2)
    if not puffer.can_transform(BASE, "schale", rclpy.time.Time()): return None
    tf = puffer.lookup_transform(BASE, "schale", rclpy.time.Time()); r, t = tf.transform.rotation, tf.transform.translation
    return (t.x, t.y, t.z, math.atan2(2 * (r.w * r.z + r.x * r.y), 1 - 2 * (r.y * r.y + r.z * r.z)))


def in_schale(lage, x, y):
    cx, cy, _, yaw = lage; dx, dy = x - cx, y - cy; r = math.hypot(dx, dy); rand = WAND + a.wand_abstand
    if not (R_INNEN + rand <= r <= R_AUSSEN - rand): return False
    t = (math.atan2(dy, dx) - yaw + math.pi) % (2 * math.pi) - math.pi
    return r * math.sin(max(0.0, WINKEL / 2 - abs(t))) >= rand


def fussabdruck_ok(lage, x, y, yaw_achse):
    ax, ay = math.cos(yaw_achse), math.sin(yaw_achse); qx, qy = -ay, ax
    L, K = a.fuss_lang / 2, a.fuss_kurz / 2
    return all(in_schale(lage, x + sl * L * qx + sk * K * ax, y + sl * L * qy + sk * K * ay)
               for sl, sk in ((1, 1), (1, -1), (-1, 1), (-1, -1), (1, 0), (-1, 0)))


# ------------------------------------------------------------------ Kamera -> Modell
def abbilden(x, y, yaw):
    """Kameraziel (robot_base, Kamerawelt) -> Modellziel, das der Roboter anfahren muss.
    Quelle: kamera_modell_platte.json (Kamera-Position gegen Modell-FK an 16 angelernten Nestern).
    radial: p_modell = s * p_kamera (um die Roboterachse), yaw unveraendert - der Griff vom 13.9.
    aehnlichkeit: s * R(theta) * p + t, yaw + theta.  affin: A * p + b, yaw ueber A * Achse."""
    if a.abbildung == "keine": ERGEBNIS["abbildung"] = "keine"; return x, y, yaw
    try: ab = json.load(open(a.abbildung_datei))
    except Exception as e:
        print(f"  !! Abbildung {a.abbildung_datei} nicht lesbar ({e}) - Ziel bleibt Kamerawelt (Modell ~8 %% daneben!)")
        ERGEBNIS["abbildung"] = "FEHLT"; return x, y, yaw
    if a.abbildung == "radial":
        s_ = float(ab["skalierung"]); xn, yn, yn_ = x * s_, y * s_, yaw; txt = f"radial x{s_:.4f}"
    elif a.abbildung == "aehnlichkeit":
        ae = ab["aehnlichkeit"]; s_ = float(ae["s"]); th = math.radians(float(ae["theta_grad"])); t_ = ae["t_mm"]
        xn = s_ * (math.cos(th) * x - math.sin(th) * y) + t_[0] / 1e3
        yn = s_ * (math.sin(th) * x + math.cos(th) * y) + t_[1] / 1e3; yn_ = yaw + th
        txt = f"aehnlichkeit s {s_:.4f} theta {math.degrees(th):+.2f} t ({t_[0]:+.1f}, {t_[1]:+.1f})"
    else:
        A = ab["affin"]["A"]; b = ab["affin"]["b_mm"]
        xn = A[0][0] * x + A[0][1] * y + b[0] / 1e3; yn = A[1][0] * x + A[1][1] * y + b[1] / 1e3
        ax, ay = math.cos(yaw), math.sin(yaw); yn_ = math.atan2(A[1][0] * ax + A[1][1] * ay, A[0][0] * ax + A[0][1] * ay)
        txt = "affin A/b"
    ERGEBNIS["abbildung"] = a.abbildung
    print(f"  Abbildung Kamera->Modell ({txt}): ({x*1e3:+.1f}, {y*1e3:+.1f}) -> ({xn*1e3:+.1f}, {yn*1e3:+.1f}) mm, "
          f"Achse {math.degrees(yaw):.1f} -> {math.degrees(yn_):.1f} Grad")
    return xn, yn, yn_


# ------------------------------------------------------------------ Ziel
def ziel_holen():
    """-> (x, y, z [m], yaw_achse [rad], quelle)"""
    lage = schale_lage()
    if lage is None:
        print("  !! kein TF robot_base->schale (Stack ohne SCHALE_MONTIERT=1?) - Hoehe nur mit --z, keine Schalenproben")
    if a.ziel_datei:
        try: zd = json.load(open(a.ziel_datei))
        except Exception as e: print(f"  !! --ziel-datei {a.ziel_datei}: {e}"); ende(4)
        g = zd.get("griff_mm"); yaw_g = zd.get("yaw_grad")
        if not g or yaw_g is None: print("  !! Zieldatei ohne griff_mm/yaw_grad"); ende(4)
        x, y, yaw, quelle = g[0] / 1e3, g[1] / 1e3, math.radians(yaw_g), os.path.basename(a.ziel_datei)
        ERGEBNIS["koerper"], ERGEBNIS["streifen"] = zd.get("koerper"), zd.get("streifen")
        print(f"  Zieldatei {quelle} ({zd.get('zeit', '?')}): Griff ({g[0]:+.1f}, {g[1]:+.1f}) mm, Achse {yaw_g:.1f} Grad, "
              f"Farbe {zd.get('koerper', '?')}/{zd.get('streifen', '?')}" + (f", {zd['anzahl']} Welle(n) im Bild" if zd.get("anzahl") else ""))
        if zd.get("gruende"): print("  !! Zieldatei meldet: " + "; ".join(zd["gruende"]))
        if zd.get("arm_im_bild"): print("  !! Zieldatei: Arm war ueber der Schale - Messung unzuverlaessig")
    elif a.ziel_topic:
        box = {"m": None}
        sub = node.create_subscription(PoseStamped, a.ziel_topic, lambda m: box.__setitem__("m", m), 1)
        print(f"  warte auf {a.ziel_topic} ({a.ziel_wartezeit:.0f} s) ...")
        t0 = time.time()
        while time.time() - t0 < a.ziel_wartezeit and box["m"] is None: rclpy.spin_once(node, timeout_sec=0.2)
        node.destroy_subscription(sub)
        if box["m"] is None: print("  !! kein Ziel vom Detektor"); ende(4)
        m = box["m"]
        if m.header.frame_id and m.header.frame_id != BASE:
            print(f"  !! Ziel-Frame '{m.header.frame_id}' statt {BASE}"); ende(4)
        x, y = m.pose.position.x, m.pose.position.y; yaw = yaw_aus_quat(m.pose.orientation)
        z_det = m.pose.position.z; quelle = a.ziel_topic
        print(f"  Detektor: ({x*1e3:+.1f}, {y*1e3:+.1f}, z {z_det*1e3:.1f}) mm, Achse {math.degrees(yaw):.1f} Grad "
              "(z wird NICHT benutzt)")
    else:
        if a.x is None or a.y is None or a.yaw is None: print("!! --x --y --yaw (oder --ziel-datei / --ziel-topic) angeben"); ende(4)
        x, y, yaw, quelle = a.x / 1e3, a.y / 1e3, math.radians(a.yaw), "Hand"
    x_cam, y_cam = x, y                      # Schalenproben in KAMERA-Koordinaten (Schale-TF ist Kamerawelt)
    ERGEBNIS["ziel_kamera_mm"] = [round(x * 1e3, 1), round(y * 1e3, 1)]
    # Kamera -> Modell (kamera_modell_platte.json, 16 Nester; 13.9.: radial 1.0825 brachte den Griff)
    x, y, yaw = abbilden(x, y, yaw)
    # Rest-Versatz (empirisch, schale_versatz.json) + Handkorrektur
    v = {"dx": 0.0, "dy": 0.0, "dz": 0.0}
    if os.path.exists(a.versatz_datei):
        v.update({k: float(json.load(open(a.versatz_datei)).get(k, 0.0)) for k in v})
        print(f"  Rest-Versatz aus {a.versatz_datei}: dx {v['dx']:+.1f} dy {v['dy']:+.1f} dz {v['dz']:+.1f} mm")
    v["dx"] += a.dx; v["dy"] += a.dy; v["dz"] += a.dz
    ERGEBNIS["versatz_mm"] = [round(v["dx"], 1), round(v["dy"], 1), round(v["dz"], 1)]
    x += v["dx"] / 1e3; y += v["dy"] / 1e3
    if a.z is not None: z = a.z / 1e3
    elif lage is not None: z = lage[2] + a.griff_mm / 1e3
    else: print("!! ohne Schalen-TF muss --z angegeben werden"); ende(4)
    z += v["dz"] / 1e3
    # Proben
    r = math.hypot(x, y); az = math.degrees(math.atan2(y, x)) % 360
    proben = []
    if not (a.r_min <= r * 1e3 <= a.r_max): proben.append(f"r_robot {r*1e3:.0f} mm ausserhalb {a.r_min:.0f}-{a.r_max:.0f} (Bodengriff)")
    if 150.0 <= az <= 176.0: proben.append(f"Azimut {az:.0f} im Totsektor")
    if lage is not None:
        if not in_schale(lage, x_cam, y_cam): proben.append("Griffpunkt liegt NICHT in der Schale")
        elif not fussabdruck_ok(lage, x_cam, y_cam, yaw): proben.append(f"Greifer-Fussabdruck {a.fuss_lang*1e3:.0f}x{a.fuss_kurz*1e3:.0f} ragt in die Schalenwand")
    ERGEBNIS["ziel_mm"] = [round(x * 1e3, 1), round(y * 1e3, 1), round(z * 1e3, 1)]; ERGEBNIS["yaw_grad"] = round(math.degrees(yaw), 1)
    ERGEBNIS["proben"] = proben
    print(f"  Ziel ({quelle}): ({x*1e3:+.1f}, {y*1e3:+.1f}, {z*1e3:.1f}) mm, r {r*1e3:.0f}, Azimut {az:.0f}, Wellenachse {math.degrees(yaw):.1f} Grad")
    for p_ in proben: print("  !! " + p_)
    if proben and not a.ohne_schalenprobe:
        print("  Ziel abgelehnt (--ohne-schalenprobe uebergeht)."); ende(4)
    return x, y, z, yaw


# ------------------------------------------------------------------ IK in bekannter Haltung
def ik_haltung(pose, greifer_wert):
    """Senkrechte IK-Loesung in der Haltung der angelernten Punkte: J3 < 0 (Ellbogen wie ueberall),
    |J5| < 45 Grad, kollisionsfrei, und davon die naechste zur Referenzhaltung (--referenz-punkt).
    TRAC-IK streut - deshalb 15 Versuche mit der Referenz und mit der Ist-Stellung als Seed."""
    if not ikc.wait_for_service(10): print("!! /compute_ik fehlt"); ende(1)
    ref = [0.0] * 6
    try:
        ref = [float(v) for v in json.load(open(a.datei))[a.referenz_punkt]["rad"]]
    except Exception: print(f"  (Referenzpunkt {a.referenz_punkt} nicht lesbar - Referenz = Nullstellung)")
    best, bd, gruende = None, 1e9, {}
    for versuch in range(15):
        seed = ref if versuch % 2 == 0 else ist_rad()
        rq = GetPositionIK.Request(); r = rq.ik_request
        r.group_name = "arm"; r.ik_link_name = "tcp"; r.avoid_collisions = True; r.timeout = Duration(sec=1)
        r.robot_state.joint_state.name = list(ARM) + ["gripper_controller"]
        r.robot_state.joint_state.position = [float(v) for v in seed] + [float(greifer_wert)]
        ps = PoseStamped(); ps.header.frame_id = BASE; ps.pose = pose; r.pose_stamped = ps
        f = ikc.call_async(rq); rclpy.spin_until_future_complete(node, f, timeout_sec=10); res = f.result()
        if res is None or res.error_code.val != 1: gruende["keine IK"] = gruende.get("keine IK", 0) + 1; continue
        nm = list(res.solution.joint_state.name); vl = list(res.solution.joint_state.position)
        sol = [vl[nm.index(j)] for j in ARM]
        if sol[2] >= 0: gruende["Ellbogen falsch (J3>=0)"] = gruende.get("Ellbogen falsch (J3>=0)", 0) + 1; continue
        if abs(sol[4]) > math.radians(45): gruende["J5 > 45"] = gruende.get("J5 > 45", 0) + 1; continue
        d = max(abs(s_ - z_) for s_, z_ in zip(sol[1:5], ref[1:5]))      # J1/J6 haengen vom Azimut/Yaw ab
        if d < bd: best, bd = sol, d
    if best is None:
        print("  !! keine brauchbare IK-Loesung: " + ", ".join(f"{k} {v}x" for k, v in gruende.items())); ende(1)
    print(f"  IK-Haltung: " + " ".join(f"{math.degrees(v):+7.1f}" for v in best) + f"  (J2-J5 max {math.degrees(bd):.0f} Grad von {a.referenz_punkt})")
    return best


def lese(was):
    spin(2.0); proz = greifer_prozent()
    ERGEBNIS["prozent"] = None if proz is None else round(proz, 1)
    print(f"     Greifer meldet {'?' if proz is None else f'{proz:.0f} %'} ({was})")
    return proz


# ------------------------------------------------------------------ Ablauf
ist_rad()
x, y, z, yaw = ziel_holen()
griff_pose = Pose(); griff_pose.position.x, griff_pose.position.y, griff_pose.position.z = x, y, z
griff_pose.orientation = quat_senkrecht(yaw)
soll = None

if a.nur_pruefen:
    print(f"\n[P] --nur-pruefen: Ueber-Punkt +{a.ueber:.0f} mm, Griffpunkt, Bahn - ohne Fahrt")
    ueber_pose = pose_dz(griff_pose, a.ueber)
    ueber_rad = ik_haltung(ueber_pose, a.auf)
    ok, kont = kollisionsfrei(ueber_rad, a.auf)
    print("  Ueber-Punkt: " + ("kollisionsfrei" if ok else "KOLLISION " + " ".join(kont)))
    traj = bahn_planen(ueber_rad, griff_pose, avoid=True)
    if traj is None: print("  Bahn ab auf den Griffpunkt: NICHT planbar (Greifer offen, Schale/Platte im Modell)")
    else:
        g_rad = [traj.points[-1].positions[traj.joint_names.index(j)] for j in ARM]
        okg, kg = kollisionsfrei(g_rad, a.auf); okz, kz = kollisionsfrei(g_rad, a.zu)
        print(f"  Bahn ab: planbar ({len(traj.points)} Punkte); Griffstellung " + " ".join(f"{math.degrees(v):+7.1f}" for v in g_rad))
        print("  Griffstellung Greifer offen: " + ("kollisionsfrei" if okg else "KOLLISION " + " ".join(kg))
              + " | ganz zu: " + ("kollisionsfrei" if okz else "KOLLISION " + " ".join(kz)))
        traj2 = bahn_planen(g_rad, pose_dz(griff_pose, a.heraus), avoid=True)
        print("  Heben +%.0f: %s" % (a.heraus, "planbar" if traj2 is not None else "NICHT planbar"))
    ende(0)

if a.nur_greifen or a.nur_heben:
    soll = ist_rad()
    if a.nur_greifen:
        print("\n[5] --nur-greifen: Greifer ganz zu + lesen (Arm steht am Griffpunkt)")
        greifer(a.zu, a.zu_dauer, "ganz zu"); ERGEBNIS["griffe"] += 1
        proz = lese("am Griffpunkt"); ende(2 if (proz is not None and proz < a.leer_prozent) else 0)
    print("\n[6] --nur-heben: senkrecht heben")
else:
    if a.ohne_nullstellung: print("\n[1] Nullstellung: ausgelassen")
    else: print("\n[1] Nullstellung"); moveit_gelenkziel(nullstellung_rad(), "Nullstellung (kollisionsgeprueft)")
    print(f"\n[2] Greifer auf ({a.auf:+.3f})"); greifer(a.auf, 3, "auf")

    print(f"\n[3] ueber das Ziel (+{a.ueber:.0f} mm)")
    ueber_pose = pose_dz(griff_pose, a.ueber)
    ueber_rad = ik_haltung(ueber_pose, a.auf)
    ok, kont = kollisionsfrei(ueber_rad, a.auf)
    if not ok: print("  !! Ueber-Punkt in Kollision: " + " ".join(kont)); ende(1)
    frage(f"Ueber-Punkt ({mm(ueber_pose)[0]:+.0f}, {mm(ueber_pose)[1]:+.0f}, {mm(ueber_pose)[2]:.0f}) mm anfahren (MoveIt)?")
    moveit_gelenkziel(ueber_rad, "ueber die Welle"); soll = list(ueber_rad)

    print("\n[4] gerade ab auf den Griffpunkt + nachstellen")
    blind = min(a.blind_ab_mm, a.ueber)
    if blind > 0:
        neu = gerade(soll, pose_dz(griff_pose, blind), T(4), f"gerade ab auf Griffpunkt +{blind:.0f} mm (geprueft)")
        if neu is None: print("  !! Abfahrt nicht planbar. Arm steht ueber der Schale."); ende(1)
        soll = neu
        frage(f"Letzte {blind:.0f} mm OHNE Kollisionspruefung. Finger frei?")
        neu = gerade(soll, griff_pose, T(6), f"gerade ab, letzte {blind:.0f} mm", avoid=False)
    else:
        frage("Gerade ab auf den Griffpunkt (kollisionsgeprueft, Welle selbst ist NICHT im Modell)?")
        neu = gerade(soll, griff_pose, T(6), "gerade ab auf den Griffpunkt")
    if neu is None: print("  !! Griffpunkt nicht erreichbar. Arm steht ueber der Schale."); ende(1)
    soll = neu; nachstellen(soll, "auf den Griffpunkt")
    # 14.9.: NOT-AUS aktiv -> Bruecke blockiert, Controller meldet trotzdem "ok"; der Arm stand noch in der Kamerapose,
    # der Greifer schloss ins Leere (99 %) und die Kette lief weiter. Darum: Encoder gegen Soll pruefen.
    _abw = max(abs(i - z) for i, z in zip(ist_rad(), soll))
    if _abw > math.radians(15):
        print(f"  !! Arm ist NICHT am Griffpunkt (groesster Gelenkfehler {math.degrees(_abw):.0f} Grad) - NOT-AUS aktiv? Bruecke blockiert? Abbruch ohne Greifen.")
        ERGEBNIS["grund"] = "Arm nicht am Ziel (Gelenkfehler %.0f Grad) - NOT-AUS aktiv / Bruecke blockiert?" % math.degrees(_abw); ende(1)
    ist = fk_pose(ist_rad())
    print(f"     Ist-TCP (Modell aus Encoder): ({mm(ist)[0]:+.1f}, {mm(ist)[1]:+.1f}, {mm(ist)[2]:.1f}) mm  "
          f"(Ziel {x*1e3:+.1f}, {y*1e3:+.1f}, {z*1e3:.1f})")
    if a.nur_anfahren:
        print("\n  --nur-anfahren: Arm steht am Griffpunkt, Greifer offen. Jetzt Versatz messen "
              "(Finger mittig ueber der Welle? -> versetze_soll.py, Differenz in schale_versatz.json)."); ende(0)

    for durchlauf in range(1, a.wiederholungen + 2):
        print(f"\n[5] Greifer ganz zu + lesen" + (f"  (Wiederholung {durchlauf-1})" if durchlauf > 1 else ""))
        frage("Finger QUER zur Welle, mittig? Dann schliessen")
        greifer(a.zu, a.zu_dauer, "ganz zu (-0.74)"); ERGEBNIS["griffe"] += 1
        proz = lese("am Griffpunkt")
        if proz is None or proz >= a.leer_prozent: break
        print(f"     -> LEER (< {a.leer_prozent:.0f} %)")
        if durchlauf > a.wiederholungen: print("\n  !! keine Welle bekommen - Abbruch, Arm bleibt stehen."); ende(2)
        frage("Wiederholen (auf, nachstellen, zu)?")
        greifer(a.auf, 3, "auf"); nachstellen(soll, "vor der Wiederholung")

print("\n[6] SENKRECHT heben")
frage("Heben?")
if a.blind_mm > 0:
    neu = gerade(ist_rad(), pose_dz(fk_pose(ist_rad()), a.blind_mm), T(3), f"heben {a.blind_mm:.0f} mm ungeprueft", avoid=False, ab_ist=True)
    if neu is None: print("  !! Heben nicht planbar"); ende(1)
    soll = neu
neu = gerade(soll, pose_dz(griff_pose, a.heraus), T(5), f"heben auf Griffpunkt +{a.heraus:.0f} mm")
if neu is None: print("  !! geprueftes Heben nicht planbar - Arm steht in der Schale, NICHT seitlich fahren."); ende(1)
soll = neu
print("\n[6b] Haltepruefung")
proz = lese("nach dem Heben")
if proz is not None and proz < a.leer_prozent:
    print(f"     -> LEER ({proz:.0f} % < {a.leer_prozent:.0f}): Welle beim Heben verloren")
    # 14.9.: Greifer OEFFNEN, bevor wir aufgeben - haengt doch etwas drin, faellt es hier ueber der Schale zurueck
    # (Benutzer: nie mit einer Welle in den Fingern von vorne anfangen). Der Arm steht auf Ziel +--heraus.
    ERGEBNIS["grund"] = f"Greifer meldet {proz:.0f} % nach dem Heben (leer < {a.leer_prozent:.0f}); Greifer ueber der Schale geoeffnet"
    greifer(a.auf, 3, "auf (Welle faellt ggf. in die Schale zurueck)"); ende(2)
if proz is not None and proz < a.unsicher_prozent:
    print(f"     (!) nur {proz:.0f} % - Welle wahrscheinlich schief / am Schaft gehalten (Flansch gibt 15-22 %)")
    ERGEBNIS["hinweis"] = f"Greifer meldet nur {proz:.0f} % - Welle schief/am Schaft gehalten?"
ERGEBNIS["welle"] = True; print("     -> WELLE DA")
if a.nur_hin:
    ERGEBNIS["steht_auf"] = f"Ziel +{a.heraus:.0f} mm"; print(f"\n[7] --nur-hin: Arm bleibt auf Ziel +{a.heraus:.0f} mm, Greifer ZU")
else:
    print("\n[7] Nullstellung, Greifer bleibt ZU"); moveit_gelenkziel(nullstellung_rad(), "Nullstellung (kollisionsgeprueft)"); ERGEBNIS["steht_auf"] = "Nullstellung"
ende(0)
