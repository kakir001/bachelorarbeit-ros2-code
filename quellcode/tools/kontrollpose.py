#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""KONTROLLPOSE: mit der Welle im Greifer unter die Kamera fahren, Greifer WAAGERECHT, damit die
Kamera die beiden herausragenden Wellenenden sieht (Kopf Ø 10 dick / Schaft Ø 7 duenn).

Warum (2026-09-14 Nacht): senkrecht ueber der Schale verdeckt das Greifergehaeuse die Welle voellig
(kopfseite_pruefen.py mass die Welle AUF der Schale statt die im Greifer). Benutzer: "Greifer waagerecht
unter die Kamera, dort pruefen, ob der Kopf auf der richtigen Seite ist; wenn nicht, J6 um 180 Grad;
wenn nicht sichtbar, fragen."

Werkzeugachsen (hole_aus_schale.quat_senkrecht): tcp +Y = Richtung Gehaeuse -> Fingerspitzen (beim Griff
nach unten), tcp +Z = Wellenachse Kopf -> Spitze, tcp +X = Schliessachse. Kontrollpose:
  +Y waagerecht (--richtung: Azimut in robot_base, Vorgabe 0 = +x, vom Roboter weg),
  +Z waagerecht quer dazu (die Welle liegt waagerecht, beide Enden ragen seitlich heraus),
  +X senkrecht (ein Finger oben, einer unten -> von oben sieht die Kamera an den Fingern vorbei
  auf die Enden). --x/--y/--z: TCP-Ort [mm], Vorgabe (150, -100, 170) - frei ueber der Schale.

Ablauf: IK (kollisionsgeprueft, mehrere Kandidaten: Richtung 0/-90/180/90 Grad, X nach oben/unten)
-> MoveIt plan_only -> Vorschau (Punkte, max. Gelenkschritt, TCP-Weg) -> --ausfuehren faehrt.
Danach: python3 tools/kopfseite_pruefen.py --bild k.png

    python3 tools/kontrollpose.py                 # nur planen und zeigen
    python3 tools/kontrollpose.py --ausfuehren
    python3 tools/kontrollpose.py --j6-180 --ausfuehren   # in der Kontrollpose Greifer um 180 Grad drehen (Welle verkehrt)
Exit 0 ok, 1 keine IK / kein Plan / Ausfuehrung fehlgeschlagen.
"""
import argparse, json, math, sys, time
import numpy as np
import rclpy
from rclpy.node import Node
from rclpy.action import ActionClient
from geometry_msgs.msg import Pose, PoseStamped, Quaternion
from moveit_msgs.action import MoveGroup, ExecuteTrajectory
from moveit_msgs.msg import Constraints, JointConstraint, RobotState
from moveit_msgs.srv import GetPositionIK, GetPositionFK
from sensor_msgs.msg import JointState
from builtin_interfaces.msg import Duration
from control_msgs.action import FollowJointTrajectory
from trajectory_msgs.msg import JointTrajectoryPoint

ARM = ['joint2_to_joint1', 'joint3_to_joint2', 'joint4_to_joint3', 'joint5_to_joint4', 'joint6_to_joint5', 'joint6output_to_joint6']
ap = argparse.ArgumentParser()
ap.add_argument("--x", type=float, default=150.0); ap.add_argument("--y", type=float, default=-100.0); ap.add_argument("--z", type=float, default=170.0)
ap.add_argument("--richtung", type=float, default=None, help="[Grad] Azimut von tcp +Y (Fingerspitzen); leer = 0/-90/180/90 probieren")
ap.add_argument("--x-oben", type=int, default=None, help="1: tcp +X nach oben, 0: nach unten; leer = beide probieren")
ap.add_argument("--ausfuehren", action="store_true")
ap.add_argument("--j6-180", action="store_true", help="nur J6 um 180 Grad drehen (in der Kontrollpose, Welle verkehrt herum)")
ap.add_argument("--j6-drehen", type=float, default=None, help="[Grad] nur J6 um diesen Winkel drehen (z.B. 90: Welle senkrecht stellen, Ende von oben sichtbar)")
ap.add_argument("--tempo", type=float, default=1.5)
ap.add_argument("--j5-min", type=float, default=60.0,
                help="[Grad] nur IK-Loesungen mit J5 >= diesem Wert (14.9. 14:0x: die Loesung J5 -152 fuhr den Greifer real in Gelenk 4; "
                     "die bewaehrte Haltung hat J5 ~ +115). 0 = keine Einschraenkung")
a = ap.parse_args()

rclpy.init(); node = Node("kontrollpose"); js = {}
node.create_subscription(JointState, "/joint_states", lambda m: js.update(dict(zip(m.name, m.position))), 10)
ikc = node.create_client(GetPositionIK, "/compute_ik"); fkc = node.create_client(GetPositionFK, "/compute_fk")
mg = ActionClient(node, MoveGroup, "/move_action"); ex = ActionClient(node, ExecuteTrajectory, "/execute_trajectory")
for c in (ikc, fkc): c.wait_for_service(10)
mg.wait_for_server(10); ex.wait_for_server(10)


def ist_rad():
    t = time.time()
    while time.time() - t < 5 and not all(j in js for j in ARM): rclpy.spin_once(node, timeout_sec=0.1)
    t = time.time()
    while time.time() - t < 0.5: rclpy.spin_once(node, timeout_sec=0.05)     # frische Werte (nach einer Fahrt)
    return [js[j] for j in ARM]


def quat_aus_achsen(eX, eY, eZ):
    R = np.column_stack([eX, eY, eZ]); t = R[0, 0] + R[1, 1] + R[2, 2]
    if t > 0:
        s = math.sqrt(t + 1.0) * 2; w = 0.25 * s; x = (R[2, 1] - R[1, 2]) / s; y = (R[0, 2] - R[2, 0]) / s; z = (R[1, 0] - R[0, 1]) / s
    elif R[0, 0] > R[1, 1] and R[0, 0] > R[2, 2]:
        s = math.sqrt(1.0 + R[0, 0] - R[1, 1] - R[2, 2]) * 2; w = (R[2, 1] - R[1, 2]) / s; x = 0.25 * s; y = (R[0, 1] + R[1, 0]) / s; z = (R[0, 2] + R[2, 0]) / s
    elif R[1, 1] > R[2, 2]:
        s = math.sqrt(1.0 + R[1, 1] - R[0, 0] - R[2, 2]) * 2; w = (R[0, 2] - R[2, 0]) / s; x = (R[0, 1] + R[1, 0]) / s; y = 0.25 * s; z = (R[1, 2] + R[2, 1]) / s
    else:
        s = math.sqrt(1.0 + R[2, 2] - R[0, 0] - R[1, 1]) * 2; w = (R[1, 0] - R[0, 1]) / s; x = (R[0, 2] + R[2, 0]) / s; y = (R[1, 2] + R[2, 1]) / s; z = 0.25 * s
    n = math.sqrt(x * x + y * y + z * z + w * w); return Quaternion(x=x / n, y=y / n, z=z / n, w=w / n)


def ik(pose, seed):
    for _ in range(12):
        rq = GetPositionIK.Request(); r = rq.ik_request
        r.group_name = "arm"; r.ik_link_name = "tcp"; r.avoid_collisions = True; r.timeout = Duration(sec=1)
        r.robot_state.joint_state.name = list(ARM) + ["gripper_controller"]
        r.robot_state.joint_state.position = [float(v) for v in seed] + [float(js.get("gripper_controller", -0.74))]
        ps = PoseStamped(); ps.header.frame_id = "robot_base"; ps.pose = pose; r.pose_stamped = ps
        f = ikc.call_async(rq); rclpy.spin_until_future_complete(node, f, timeout_sec=10); res = f.result()
        if res is not None and res.error_code.val == 1:
            nm = list(res.solution.joint_state.name); vl = list(res.solution.joint_state.position)
            sol = [vl[nm.index(j)] for j in ARM]
            if a.j5_min and math.degrees(sol[4]) < a.j5_min: continue      # andere Handgelenkloesung - verworfen
            return sol
    return None


def fk(rad):
    rs = RobotState(); rs.joint_state.name = ARM; rs.joint_state.position = list(rad)
    rq = GetPositionFK.Request(); rq.header.frame_id = "robot_base"; rq.fk_link_names = ["tcp"]; rq.robot_state = rs
    f = fkc.call_async(rq); rclpy.spin_until_future_complete(node, f, timeout_sec=10); p = f.result().pose_stamped[0].pose.position
    return p.x * 1e3, p.y * 1e3, p.z * 1e3


def plan(ziel_rad):
    goal = MoveGroup.Goal(); req = goal.request
    req.group_name = "arm"; req.num_planning_attempts = 10; req.allowed_planning_time = 10.0
    req.max_velocity_scaling_factor = min(1.0, 0.08 * a.tempo); req.max_acceleration_scaling_factor = 0.08
    c = Constraints()
    for j, v in zip(ARM, ziel_rad):
        jc = JointConstraint(); jc.joint_name = j; jc.position = float(v); jc.tolerance_above = jc.tolerance_below = 0.01; jc.weight = 1.0; c.joint_constraints.append(jc)
    req.goal_constraints = [c]; goal.planning_options.plan_only = True
    fu = mg.send_goal_async(goal); rclpy.spin_until_future_complete(node, fu, timeout_sec=20); gh = fu.result()
    rf = gh.get_result_async(); rclpy.spin_until_future_complete(node, rf, timeout_sec=60); res = rf.result().result
    if res.error_code.val != 1: return None
    return res.planned_trajectory


def vorschau(tr):
    jt = tr.joint_trajectory; idx = [jt.joint_names.index(j) for j in ARM]
    pts = [[p.positions[i] for i in idx] for p in jt.points]
    schritt = max(abs(b - a_) for p, q in zip(pts, pts[1:]) for a_, b in zip(p, q)) if len(pts) > 1 else 0
    tcps = [fk(p) for p in pts[:: max(1, len(pts) // 10)]] + [fk(pts[-1])]
    zmin = min(t[2] for t in tcps)
    print("  Plan: %d Punkte, max. Gelenkschritt %.1f Grad, TCP z min %.0f mm, Ende (%.0f, %.0f, %.0f)" % (len(pts), math.degrees(schritt), zmin, *tcps[-1]))
    for k_, p in enumerate(pts):
        if k_ % max(1, len(pts) // 6) == 0 or k_ == len(pts) - 1: print("   %2d: " % k_ + " ".join("%+7.1f" % math.degrees(v) for v in p))
    return math.degrees(schritt) <= 15 and zmin >= 50


def ausfuehren(tr):
    eg = ExecuteTrajectory.Goal(); eg.trajectory = tr
    fu = ex.send_goal_async(eg); rclpy.spin_until_future_complete(node, fu, timeout_sec=20); gh = fu.result()
    rf = gh.get_result_async(); rclpy.spin_until_future_complete(node, rf, timeout_sec=240)
    return rf.result().result.error_code.val == 1


start = ist_rad()
if a.j6_180 or a.j6_drehen is not None:
    ziel = list(start)
    if a.j6_180: ziel[5] = ziel[5] + math.pi if ziel[5] < 0 else ziel[5] - math.pi
    else:
        ziel[5] = ziel[5] + math.radians(a.j6_drehen)
        if abs(ziel[5]) > math.radians(170): ziel[5] = ziel[5] - math.copysign(2 * math.pi, ziel[5])
    print("J6 drehen: %.1f -> %.1f Grad (nur Handgelenk, in der Kontrollpose)" % (math.degrees(start[5]), math.degrees(ziel[5])))
    tr = plan(ziel)
    if tr is None: print("!! kein Plan (Kollision?)"); sys.exit(1)
    ok = vorschau(tr)
    if not a.ausfuehren: print("  (nur geplant)"); sys.exit(0)
    if not ok: print("!! Vorschau verletzt Grenzen - nicht gefahren"); sys.exit(1)
    print("  Ausfuehrung:", "ok" if ausfuehren(tr) else "FEHLER"); sys.exit(0)

richtungen = [a.richtung] if a.richtung is not None else [0.0, -90.0, 180.0, 90.0]
obens = [a.x_oben] if a.x_oben is not None else [1, 0]
loesung = None
for rz in richtungen:
    for oben in obens:
        eY = np.array([math.cos(math.radians(rz)), math.sin(math.radians(rz)), 0.0])
        eX = np.array([0.0, 0.0, 1.0 if oben else -1.0])
        eZ = np.cross(eX, eY)                                     # rechtshaendig: X x Y = Z
        pose = Pose(); pose.position.x, pose.position.y, pose.position.z = a.x / 1e3, a.y / 1e3, a.z / 1e3
        pose.orientation = quat_aus_achsen(eX, eY, eZ)
        # Seed: erst die bewaehrte Haltung vom 14.9. (J5 +115), dann die Ist-Stellung
        BEWAEHRT = [math.radians(v) for v in (-65.7, 13.0, -115.6, -77.4, 114.7, 41.8)]
        sol = ik(pose, BEWAEHRT) or ik(pose, start)
        print("  Richtung %+4.0f, X %s: %s" % (rz, "oben" if oben else "unten", "keine IK" if sol is None else "IK " + " ".join("%+6.1f" % math.degrees(v) for v in sol)))
        if sol is not None and loesung is None: loesung = (rz, oben, sol)
if loesung is None: print("!! keine kollisionsfreie IK fuer die Kontrollpose - --x/--y/--z aendern"); sys.exit(1)
rz, oben, sol = loesung
print("Kontrollpose: TCP (%.0f, %.0f, %.0f) mm, Finger zeigen nach Azimut %.0f Grad, Schliessachse %s, Welle waagerecht" % (a.x, a.y, a.z, rz, "senkrecht (X oben)" if oben else "senkrecht (X unten)"))
tr = plan(sol)
if tr is None: print("!! kein MoveIt-Plan"); sys.exit(1)
ok = vorschau(tr)
json.dump({"rad": [float(v) for v in sol], "grad": [round(math.degrees(v), 2) for v in sol], "tcp_mm": [a.x, a.y, a.z], "richtung": rz, "x_oben": oben},
          open("/tmp/kontrollpose.json", "w"))
if not a.ausfuehren: print("  (nur geplant; --ausfuehren faehrt)"); sys.exit(0)
if not ok: print("!! Vorschau verletzt Grenzen (Schritt > 15 Grad oder TCP < 50 mm) - nicht gefahren"); sys.exit(1)
print("  Ausfuehrung:", "ok" if ausfuehren(tr) else "FEHLER")
time.sleep(1.0); print("  Ist-TCP (%.0f, %.0f, %.0f) mm" % fk(ist_rad()))
