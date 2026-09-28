#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""Welle in ein Nest der Ablageplatte stellen — das Rezept des ERSTEN VOLLEN ZYKLUS (2026-09-12).

Voraussetzung: Welle haengt im Greifer (hole_welle.py --nur-hin: Arm steht 60 mm ueber dem
Trichter-Griffpunkt, Greifer ganz zu). Nester sind angelernte Gelenkstellungen
(teach_punkte.json, nest_JI: J = Zeile 1..5 nah -> fern, I = Spalte 1..4). Angelernt wurde mit
der Fingerspitze ~1 mm ueber dem Kopf einer SITZENDEN Welle.

Ablauf:
  0. Haltepruefung          Greifer ganz zu (-0.74) und lesen: < 8 % = leer -> Exit 2.
                            (-0.62 taugt nicht: die Firmware meldet dann den Kommandowert 13 %.)
  1. UEBER das Nest         Ueber-Gelenke per GERADER BAHN aus der Neststellung nach oben
                            (+40 mm, gleicher IK-Zweig; per IK wich J3/J4 um 20-30 Grad ab
                            -> nachstellen brach ab, nest_33 scheiterte so zweimal), dann
                            MoveIt kollisionsgeprueft dorthin. Drehen nur hier, weit ueber allem.
  2. ABSENKEN ab Soll       gerade nach unten auf Nestpunkt + --ablage-dz (Default 0: so stand
                            die Welle in nest_11 - die Welle ist am Kopf gefasst, ihr Fuss steht
                            damit knapp ueber dem Nest; die fruehere Annahme "-20 mm" war falsch)
                            + --dx/--dy (Ablageversatz: nest_11 brauchte dx -6, die Welle haengt
                            nicht mittig unter dem TCP; nest_11_ablage enthaelt das schon).
  3. nachstellen            Totband raus, Greifer senkrecht (Modell)
  4. loslassen              Greifer auf, Welle faellt die letzten mm ins Nest
  5. heben                  gerade hoch auf +ueber
  6. Tastprobe (--tastprobe) Greifer offen auf Nestpunkt + --tast-dz (-6: Finger 5 mm unter
                            Kopfoberkante), GANZ zu (-0.74), lesen: >= 8 % = Welle STEHT im Nest;
                            0 % = liegt/fehlt. Dann auf, heben.

    python3 tools/lege_welle.py --nest nest_11_ablage             # MANUELL: fragt vor dem Loslassen
    python3 tools/lege_welle.py --nest nest_33 --dx -6 --auto --tastprobe
Exit: 0 gelegt (und Tastprobe ok), 1 Fehler (Arm steht), 2 Greifer war leer, 3 Abbruch durch
      Benutzer, 4 Tastprobe: Welle steht nicht. Letzte Zeile: ERGEBNIS {json}.
"""
import argparse, json, math, os, sys, time
import rclpy
from rclpy.action import ActionClient
from rclpy.node import Node
from geometry_msgs.msg import Pose, PoseStamped
from moveit_msgs.action import MoveGroup
from moveit_msgs.msg import Constraints, JointConstraint, RobotState
from moveit_msgs.srv import GetCartesianPath, GetPositionFK, GetPositionIK
from sensor_msgs.msg import JointState
from control_msgs.action import FollowJointTrajectory
from trajectory_msgs.msg import JointTrajectoryPoint
from builtin_interfaces.msg import Duration
sys.path.insert(0, os.path.dirname(os.path.abspath(__file__))); from bahnpruefung import pruefe_bahn  # Stetigkeit (Unfall 13.9.)
from gelenkspiel import j4_von_oben  # Spiel hinter Achse 4 (19.9.)

ARM = ['joint2_to_joint1', 'joint3_to_joint2', 'joint4_to_joint3',
       'joint5_to_joint4', 'joint6_to_joint5', 'joint6output_to_joint6']
BASE = 'robot_base'
ap = argparse.ArgumentParser()
ap.add_argument("--nest", required=True)
ap.add_argument("--datei", default=os.path.expanduser("~/ros2_ws/teach_punkte.json"))
ap.add_argument("--ueber", type=float, default=40.0, help="[mm] Anfahr-/Hebehoehe ueber dem Nestpunkt")
ap.add_argument("--ablage-dz", type=float, default=5.0, help="[mm] Loslasshoehe relativ zum Nestpunkt (23.9.: +5 = Fuss UEBER der Bohrung, die Welle faellt selbst ein; vorher 0)")
ap.add_argument("--korrektur-ab", type=float, default=2.0, help="[mm] 23.9.: liegt der Arm nach der einen Absenkbahn weiter daneben, EINE Nachstell-Runde am Loslasspunkt (+ablage-dz, Fuss ueber dem Nestrand); 0 = immer, 99 = nie")
ap.add_argument("--nachstellen-unten", action="store_true", help="ALTES Verhalten: J4 von oben + nachstellen erst AM Nestpunkt (23.9.: stiess den Wellenfuss an den Nestrand)")
ap.add_argument("--dx", type=float, default=0.0, help="[mm] Ablageversatz x zum Nestpunkt (nest_11: -6; nest_11_ablage hat ihn schon)")
ap.add_argument("--dy", type=float, default=0.0, help="[mm] Ablageversatz y zum Nestpunkt")
ap.add_argument("--tastprobe", action="store_true")
ap.add_argument("--max-abweichung", type=float, default=4.0, help="[mm] groesste zulaessige Abweichung der Ablagestellung; darueber Exit 6 ohne Loslassen (14.9.)")
ap.add_argument("--tast-dz", type=float, default=-6.0, help="[mm] Tasthoehe relativ zum Nestpunkt")
ap.add_argument("--halte-prozent", type=float, default=3.0, help="Greifer ganz zu: ab hier gilt 'Welle da' (14.9.: 8 -> 3; am Kopf gehaltene Welle meldete 5 %% und galt als leer)")
ap.add_argument("--zu", type=float, default=-0.74, help="Greifer zu - NUR -0.74 taugt als Sensor")
ap.add_argument("--auf", type=float, default=0.15)
ap.add_argument("--ohne-haltepruefung", action="store_true")
ap.add_argument("--auto", action="store_true", help="VOLLAUTOMATIK: keine Rueckfragen")
ap.add_argument("--tempo", type=float, default=2.0)
ap.add_argument("--setzzeit", type=float, default=4.0)
ap.add_argument("--gain", type=float, default=0.6); ap.add_argument("--runden", type=int, default=4)
ap.add_argument("--toleranz", type=float, default=0.3); ap.add_argument("--max-korrektur", type=float, default=4.0)
ap.add_argument("--ik-max-grad", type=float, default=35.0, help="Rueckfall-IK: max. Gelenkabstand der Ueber-Loesung zur Neststellung")
ap.add_argument("--null", action="store_true", help="danach zur Nullstellung")
ap.add_argument("--farbe", nargs="+", default=None, metavar="FARBE",
                help="Koerper[ Streifen] der abgelegten Welle -> wird in platte_belegung.json eingetragen")
ap.add_argument("--belegung", default=os.path.expanduser("~/ros2_ws/platte_belegung.json"),
                help="Belegungsdatei (nest_fuer_farbe.py liest sie); '' = nicht schreiben")
ap.add_argument("--stehen", action="store_true", help="nach dem Nachstellen ueber dem Nest STEHEN BLEIBEN (Exit 3); weiter mit --ab-loslassen")
ap.add_argument("--ohne-j4-von-oben", action="store_true", help="Achse 4 nach dem Absenken NICHT von oben auf das Soll setzen (19.9.: Spiel hinter dem Getriebe)")
ap.add_argument("--ab-loslassen", action="store_true", help="Arm steht schon (nachgestellt) auf dem Ablagepunkt: nur loslassen, heben, Tastprobe")
a = ap.parse_args()
T = lambda sek: max(1.5, sek / a.tempo)

punkte = json.load(open(a.datei))
if a.nest not in punkte: sys.exit(f"!! Nest '{a.nest}' nicht in {a.datei}")
nest_rad = [float(v) for v in punkte[a.nest]["rad"]]

rclpy.init(); node = Node("lege_welle"); js = {"m": None}
node.create_subscription(JointState, "/joint_states", lambda m: js.__setitem__("m", m), 10)
arm = ActionClient(node, FollowJointTrajectory, "/arm_controller/follow_joint_trajectory")
grp = ActionClient(node, FollowJointTrajectory, "/gripper_controller/follow_joint_trajectory")
mg = ActionClient(node, MoveGroup, "/move_action")
cart = node.create_client(GetCartesianPath, "/compute_cartesian_path")
fkc = node.create_client(GetPositionFK, "/compute_fk")
ikc = node.create_client(GetPositionIK, "/compute_ik")
ERGEBNIS = {"nest": a.nest, "gelegt": False, "tastprobe": None, "prozent_vorher": None, "abweichung_mm": None}


def belegung_eintragen():
    """Nach erfolgreicher Ablage: Nest in platte_belegung.json als belegt eintragen (mit Farbe,
    wenn --farbe gegeben). Tastprobe 'liegt' -> trotzdem eintragen, aber als 'liegt' markiert:
    das Nest ist dann nicht frei, auch wenn die Welle nicht steht."""
    if not a.belegung or not ERGEBNIS["gelegt"]: return
    try:
        bel = json.load(open(a.belegung)) if os.path.exists(a.belegung) else {}
    except Exception: bel = {}
    t = ERGEBNIS.get("tastprobe")
    eintrag = {"koerper": a.farbe[0] if a.farbe else None,
               "streifen": a.farbe[1] if a.farbe and len(a.farbe) > 1 else None,
               "steht": None if t is None else bool(t["steht"]),
               "zeit": time.strftime("%Y-%m-%d %H:%M")}
    bel[a.nest[:-7] if a.nest.endswith("_ablage") else a.nest] = eintrag     # nest_11_ablage -> nest_11 (14.9.)
    bel["_hinweis"] = "Belegung der Ablageplatte; Wert = Eintrag (belegt) oder 'frei'. Geschrieben von lege_welle.py, gelesen von nest_fuer_farbe.py."
    json.dump(bel, open(a.belegung, "w"), indent=1, ensure_ascii=False)
    print(f"  Belegung: {a.nest[:-7] if a.nest.endswith('_ablage') else a.nest} -> {eintrag} ({a.belegung})")


def ende(code):
    belegung_eintragen()
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
            return (js["m"].position[js["m"].name.index("gripper_controller")] + 0.74) / 0.89 * 100.0  # Firmware-%: Bereich [-0.74, 0.15]
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


def lese(was):
    spin(2.0); proz = greifer_prozent()
    print(f"  Greifer meldet {'?' if proz is None else f'{proz:.0f} %'} ({was})")
    return proz


VORHALT = [0.0] * 6   # Kommando - Ziel nach dem letzten Nachstellen (23.9.: wird auf die Absenkbahn gelegt)


def nachstellen(ziel, was):
    global VORHALT
    fmt = lambda v: " ".join(f"{math.degrees(x):+7.2f}" for x in v)
    kommando = list(ziel); fehler = [i - z for i, z in zip(ist_rad(), ziel)]
    nick = lambda f: math.degrees(f[1] + f[2] + f[3]); tol = math.radians(a.toleranz)
    print(f"  nachstellen {was}: Fehler {fmt(fehler)} (Nick {nick(fehler):+.2f})")
    best = (max(abs(e) for e in fehler), list(kommando), list(fehler))   # 23.9.: beste Runde merken (Totband J2/J3 ~0.8 Grad: die Runden springen ueber das Ziel)
    for runde in range(1, a.runden + 1):
        if all(abs(e) <= tol for e in fehler):
            print(f"     fertig nach {runde-1} Korrektur(en)"); VORHALT = [k - z for k, z in zip(kommando, ziel)]; return
        kommando = [k - a.gain * e for k, e in zip(kommando, fehler)]
        if any(abs(k - z) > math.radians(a.max_korrektur) for k, z in zip(kommando, ziel)):
            print("  !! Vorhalt zu gross — abgebrochen"); VORHALT = [0.0] * 6; return
        fahre_gelenke(kommando, T(3), f"Korrektur {runde}", setzzeit=4.0)
        fehler = [i - z for i, z in zip(ist_rad(), ziel)]
        print(f"     Fehler {fmt(fehler)} (Nick {nick(fehler):+.2f})")
        if max(abs(e) for e in fehler) < best[0]: best = (max(abs(e) for e in fehler), list(kommando), list(fehler))
    if not all(abs(e) <= tol for e in fehler):
        print(f"  !! nach {a.runden} Runden noch ausserhalb {a.toleranz} Grad")
        if best[0] < max(abs(e) for e in fehler) - math.radians(0.05):
            kommando = best[1]
            fahre_gelenke(kommando, T(3), f"zurueck auf die beste Runde (max {math.degrees(best[0]):.2f} Grad)", setzzeit=4.0)
            fehler = [i - z for i, z in zip(ist_rad(), ziel)]
            print(f"     Fehler {fmt(fehler)} (Nick {nick(fehler):+.2f})")
    VORHALT = [k - z for k, z in zip(kommando, ziel)]


def fk_pose(rad):
    if not fkc.wait_for_service(10): print("!! /compute_fk fehlt"); ende(1)
    rs = RobotState(); rs.joint_state.name = list(ARM); rs.joint_state.position = [float(v) for v in rad]
    rq = GetPositionFK.Request(); rq.header.frame_id = BASE; rq.fk_link_names = ["tcp"]; rq.robot_state = rs
    f = fkc.call_async(rq); rclpy.spin_until_future_complete(node, f, timeout_sec=10)
    if f.result() is None or not f.result().pose_stamped: print("!! FK fehlgeschlagen"); ende(1)
    return f.result().pose_stamped[0].pose


def pose_d(pose, dx_mm=0.0, dy_mm=0.0, dz_mm=0.0):
    p = Pose(); p.orientation = pose.orientation
    p.position.x = pose.position.x + dx_mm / 1000.0; p.position.y = pose.position.y + dy_mm / 1000.0
    p.position.z = pose.position.z + dz_mm / 1000.0
    return p


mm = lambda p: (p.position.x * 1000, p.position.y * 1000, p.position.z * 1000)


def bahn_planen(start_rad, ziel_pose, min_fraction=0.95):
    if not cart.wait_for_service(10): print("!! /compute_cartesian_path fehlt"); ende(1)
    rs = RobotState(); rs.joint_state.name = list(ARM); rs.joint_state.position = [float(v) for v in start_rad]
    cq = GetCartesianPath.Request(); cq.header.frame_id = BASE; cq.group_name = "arm"; cq.link_name = "tcp"
    cq.start_state = rs; cq.waypoints = [ziel_pose]; cq.max_step = 0.005; cq.jump_threshold = 0.0; cq.avoid_collisions = True
    f = cart.call_async(cq); rclpy.spin_until_future_complete(node, f, timeout_sec=120); res = f.result()
    if res is None: print("  !! compute_cartesian_path: Timeout"); return None
    if res.fraction < min_fraction: print(f"  !! nur {res.fraction*100:.0f} % planbar"); return None
    ok_b, txt_b = pruefe_bahn(res.solution.joint_trajectory, ARM, start=list(start_rad))
    if not ok_b: print("  !! " + txt_b + " - NICHT gefahren"); return None
    return res.solution.joint_trajectory


def gerade_ab_soll(soll, ziel_pose, dauer, was, vorhalt=None):
    """Gerade Bahn ab SOLL-Gelenken auf ziel_pose; erster Bahnpunkt = Ist. Rueckgabe: neue Soll-Gelenke.
    vorhalt: [rad] je Gelenk, wird auf ALLE Bahnpunkte (ausser dem Start) addiert - der Servo-Vorhalt aus dem
    Nachstellen bleibt so waehrend der ganzen Fahrt erhalten (Spiel setzt sich nicht um; wie hole_welle 14.9.)."""
    traj = bahn_planen(soll, ziel_pose)
    if traj is None: print(f"!! {was}: NICHT gefahren, Arm steht"); ende(1)
    neu = [traj.points[-1].positions[traj.joint_names.index(j)] for j in ARM]
    if vorhalt is not None and any(abs(v) > 1e-6 for v in vorhalt):
        idx = [traj.joint_names.index(j) for j in ARM]
        for pt in traj.points[1:]:
            pos = list(pt.positions)
            for k, v in zip(idx, vorhalt): pos[k] += v
            pt.positions = pos
        print(f"     Vorhalt {' '.join(f'{math.degrees(v):+.1f}' for v in vorhalt)} Grad auf alle Bahnpunkte")
    traj.points[0].positions = ist_rad()
    n = len(traj.points)
    for i, pt in enumerate(traj.points):
        t = dauer * (i + 1) / n; pt.time_from_start.sec = int(t); pt.time_from_start.nanosec = int((t - int(t)) * 1e9)
        pt.velocities = []; pt.accelerations = []
    print(f"  -> {was} (gerade, {dauer:.0f} s)")
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
        print(f"!! keine IK-Loesung nahe der Neststellung ({'-' if best is None else f'{math.degrees(bd):.0f} Grad'})"); ende(1)
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


def ueber_gelenke(ziel_pose, dz_mm):
    """Ueber-Punkt: ZUERST per gerader Bahn AUS der Neststellung nach oben (gleicher IK-Zweig wie
    angelernt), erst wenn das nicht geht per IK nahe am Seed (2026-09-12, s. Docstring)."""
    traj = bahn_planen(nest_rad, pose_d(ziel_pose, dz_mm=dz_mm), min_fraction=0.99)
    if traj is not None:
        rad = [traj.points[-1].positions[traj.joint_names.index(j)] for j in ARM]
        print(f"  Ueber-Punkt per gerader Bahn aus der Neststellung (max Gelenkabstand "
              f"{math.degrees(max(abs(u - n_) for u, n_ in zip(rad, nest_rad))):.1f} Grad)")
        return rad
    print("  (gerade Bahn nach oben nicht planbar - IK nahe am Seed)")
    return ik_nahe(pose_d(ziel_pose, dz_mm=dz_mm), nest_rad, a.ik_max_grad)


# ------------------------------------------------------------------ Ablauf
ist_rad()
print("\n[0] Haltepruefung: Greifer ganz zu und lesen")
if a.ohne_haltepruefung or a.ab_loslassen:
    print("  uebersprungen")
else:
    greifer(a.zu, 3, "ganz zu (Pruefung)")
    proz = lese("vor der Ablage"); ERGEBNIS["prozent_vorher"] = None if proz is None else round(proz, 1)
    if proz is not None and proz < a.halte_prozent:
        print(f"  -> LEER (< {a.halte_prozent:.0f} %), nichts zu legen"); ende(2)
    print("  -> Welle da")

nest_pose = fk_pose(nest_rad)
ablage_pose = pose_d(nest_pose, a.dx, a.dy)         # Ablagepunkt in der Ebene versetzt, z = Nestpunkt
print(f"  Nest '{a.nest}': TCP-Modell ({mm(nest_pose)[0]:+.1f}, {mm(nest_pose)[1]:+.1f}, {mm(nest_pose)[2]:.1f}) mm"
      + (f"  Ablageversatz dx {a.dx:+.1f} dy {a.dy:+.1f}" if a.dx or a.dy else "")
      + f"  [{punkte[a.nest].get('notiz', '')[:60]}]")

if a.ab_loslassen:
    print("\n  --ab-loslassen: Arm steht auf dem Ablagepunkt (Soll = Bahnziel wird neu gerechnet)")
    traj = bahn_planen(nest_rad, pose_d(ablage_pose, dz_mm=a.ablage_dz), min_fraction=0.99)
    soll = [traj.points[-1].positions[traj.joint_names.index(j)] for j in ARM] if traj else ist_rad()
    # 14.9.: wurde J6 zwischendurch von Hand gedreht (GUI: "J6 180"), gilt die IST-Stellung als Soll - sonst wuerde das
    # Heben mit einem 180-Grad-Sprung im Handgelenk beginnen.
    _ist = ist_rad()
    if abs(soll[5] - _ist[5]) > 1.0: print("  (J6 weicht %.0f Grad vom geplanten Soll ab - Ist-Stellung gilt)" % math.degrees(soll[5] - _ist[5])); soll = _ist
else:
    print(f"\n[1] ueber das Nest (+{a.ueber:.0f} mm)")
    ueber_rad = ueber_gelenke(ablage_pose, a.ueber)
    moveit_gelenkziel(ueber_rad, f"ueber {a.nest}")
    soll = list(ueber_rad)

    if a.nachstellen_unten:
        print(f"\n[2] absenken auf Nestpunkt {a.ablage_dz:+.0f} mm (ALT: Korrektur unten)")
        soll = gerade_ab_soll(soll, pose_d(ablage_pose, dz_mm=a.ablage_dz), T(6), "absenken")
        if not a.ohne_j4_von_oben:
            # 19.9.: Spiel hinter Achse 4 legt sich auf die Seite der letzten Bewegung (bis 7 Grad Neigung) -
            # wie beim Anlernen von OBEN auf das Soll (tools/gelenkspiel.py)
            j4_von_oben(lambda rad, dauer, was: fahre_gelenke(rad, dauer, was, setzzeit=2.0), soll)
        print("\n[3] nachstellen"); nachstellen(soll, "Ablagestellung")
    else:
        # 23.9. (Benutzer): Absenken + J4 von oben + Nachstellen AM NEST liess den Wellenfuss in Stufen an den
        # Nestrand stossen -> Welle schief im Greifer, ging nicht ins Nest. Neu: das Spiel wird OBEN gesetzt
        # (+ueber: J2/J4 von oben, nachstellen), dann EINE gerade senkrechte Bahn mit dem Vorhalt auf allen
        # Punkten bis ueber das Nest (ablage-dz +5: Fuss ueber der Bohrung). Unten KEINE Korrektur mehr -
        # loslassen, die Welle faellt von selbst ein.
        print(f"\n[2] Spiel OBEN setzen (+{a.ueber:.0f} mm)")
        if not a.ohne_j4_von_oben:
            j4_von_oben(lambda rad, dauer, was: fahre_gelenke(rad, dauer, was, setzzeit=2.0), soll)
        nachstellen(soll, "Ueber-Punkt")
        print(f"\n[3] absenken in EINER Bahn auf Nestpunkt {a.ablage_dz:+.0f} mm (keine Korrektur unten)")
        soll = gerade_ab_soll(soll, pose_d(ablage_pose, dz_mm=a.ablage_dz), T(6), "absenken (eine Bahn)", vorhalt=VORHALT)
    spin(2.0)
    _e = [i - z for i, z in zip(ist_rad(), soll)]
    print("  Encoder - Soll [Grad]: " + " ".join(f"{math.degrees(v):+.2f}" for v in _e) + f"  (Nick {math.degrees(_e[1]+_e[2]+_e[3]):+.2f})")
    ist = fk_pose(ist_rad()); zi = pose_d(ablage_pose, dz_mm=a.ablage_dz)
    d = [i - z for i, z in zip(mm(ist), mm(zi))]; ERGEBNIS["abweichung_mm"] = [round(v, 1) for v in d]
    print(f"  Ist-TCP (Modell aus Encoder): ({mm(ist)[0]:+.1f}, {mm(ist)[1]:+.1f}, {mm(ist)[2]:.1f}) mm"
          f"  -> Abweichung zum Ziel dx {d[0]:+.1f} dy {d[1]:+.1f} dz {d[2]:+.1f} mm")
    if not a.nachstellen_unten and max(abs(v) for v in d) > a.korrektur_ab:
        # 23.9. (4 Absenkungen): nach der einen Bahn bleiben J2/J3 reproduzierbar +0.7..1.1 Grad zurueck (Arm zu hoch,
        # radial innen, 3.5-6.7 mm); bei 5.8 mm losgelassen -> Welle daneben. Am Loslasspunkt (+5 mm) steht der Fuss
        # ueber dem Nestrand, eine Korrektur stoesst dort nirgends an - anders als frueher bei dz 0.
        print(f"  Abweichung > {a.korrektur_ab:.0f} mm -> EINE Korrektur am Loslasspunkt (+{a.ablage_dz:.0f} mm, Fuss ueber dem Nestrand)")
        nachstellen(soll, "Loslasspunkt"); spin(2.0)
        _e = [i - z for i, z in zip(ist_rad(), soll)]
        print("  Encoder - Soll [Grad]: " + " ".join(f"{math.degrees(v):+.2f}" for v in _e) + f"  (Nick {math.degrees(_e[1]+_e[2]+_e[3]):+.2f})")
        ist = fk_pose(ist_rad()); d = [i - z for i, z in zip(mm(ist), mm(zi))]; ERGEBNIS["abweichung_mm"] = [round(v, 1) for v in d]
        print(f"  Ist-TCP nach Korrektur: ({mm(ist)[0]:+.1f}, {mm(ist)[1]:+.1f}, {mm(ist)[2]:.1f}) mm"
              f"  -> Abweichung dx {d[0]:+.1f} dy {d[1]:+.1f} dz {d[2]:+.1f} mm")
    # 14.9. (nest_51: Ist 8/9/-11 mm neben dem Ziel, Nachstellen brach ab, Welle wurde trotzdem losgelassen -> daneben):
    # bei zu grosser Abweichung NICHT loslassen, sondern heben und Exit 6 - die GUI fragt dann.
    if max(abs(v) for v in d) > a.max_abweichung and not a.ab_loslassen:
        print(f"  !! Abweichung > {a.max_abweichung:.0f} mm - NICHT losgelassen. Hebe auf +{a.ueber:.0f} mm zurueck.")
        ERGEBNIS["grund"] = "Ablagestellung %.0f/%.0f/%.0f mm neben dem Ziel (max %.0f) - nicht losgelassen" % (d[0], d[1], d[2], a.max_abweichung)
        gerade_ab_soll(ist_rad(), pose_d(ablage_pose, dz_mm=a.ueber), T(5), "heben (Welle noch im Greifer)"); ende(6)
    if a.stehen:
        print("  --stehen: Arm bleibt ueber dem Nest, Greifer ZU. Weiter: --ab-loslassen"); ende(3)

print("\n[4] loslassen")
frage("Welle steht mittig ueber dem Nest, Fuss knapp darueber? Dann loslassen (sie faellt selbst ein)")
greifer(a.auf, 3, "auf (Welle faellt ins Nest)")
spin(1.5)

print("\n[5] heben")
soll = gerade_ab_soll(soll, pose_d(ablage_pose, dz_mm=a.ueber), T(5), "heben")
ERGEBNIS["gelegt"] = True

if a.tastprobe:
    print(f"\n[6] Tastprobe: Greifer offen auf Nestpunkt {a.tast_dz:+.0f} mm, ganz zu, lesen")
    # Tastprobe OHNE Ablageversatz: die abgestellte Welle sitzt im Nest (Kopf = Nestpunkt), der
    # Versatz galt nur fuer die haengende Welle. 12.9.: mit Versatz meldete die Probe 0 % obwohl
    # die Welle stand.
    soll = gerade_ab_soll(soll, pose_d(nest_pose, dz_mm=a.tast_dz), T(5), "absenken zur Tastprobe (ohne Versatz)")
    nachstellen(soll, "Tastprobe")
    greifer(a.zu, 4, "ganz zu (tasten)")
    proz = lese("Tastprobe")
    steht = proz is not None and proz >= a.halte_prozent
    ERGEBNIS["tastprobe"] = {"prozent": None if proz is None else round(proz, 1), "steht": steht}
    print("  -> " + ("WELLE STEHT im Nest" if steht else "KEINE Welle gefasst — liegt oder fehlt"))
    greifer(a.auf, 3, "auf")
    soll = gerade_ab_soll(soll, pose_d(nest_pose, dz_mm=a.ueber), T(5), "heben")
    if not steht: ende(4)

if a.null:
    print("\n[7] Nullstellung"); moveit_gelenkziel([0.0] * 6, "Nullstellung (kollisionsgeprueft)")
print("\n  fertig."); ende(0)
