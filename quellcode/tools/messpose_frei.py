#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""Greifstellung im FREIEN Raum nachbilden - zum Messen der Greiferneigung.

Im Trichter ist kein Platz fuer eine Wasserwaage (2026-09-11). Durchhaengen und
Getriebespiel haengen aber an der Schwerkraftlast der Achsen 2-5, nicht an
Achse 1. Also: dieselben Winkel J2..J6 wie im Teach-Punkt, nur J1 gedreht -
gleiche Neigung, freier Zugang. Anfahrt kollisionsgeprueft ueber MoveIt (Gelenkziel),
danach nachstellen wie in hole_welle.py, dann stehen bleiben.

Voraussetzung: Arm in der Nullstellung (vorher hebe_und_parke.py --null).

    python3 tools/messpose_frei.py --punkt trichter_greif --j1 20
"""
import argparse, json, math, os, sys, time
import rclpy
from rclpy.action import ActionClient
from rclpy.node import Node
from control_msgs.action import FollowJointTrajectory
from moveit_msgs.action import MoveGroup
from moveit_msgs.msg import Constraints, JointConstraint
from moveit_msgs.srv import GetStateValidity
from moveit_msgs.msg import RobotState
from sensor_msgs.msg import JointState
from trajectory_msgs.msg import JointTrajectoryPoint
import tf2_ros

ARM = ["joint2_to_joint1", "joint3_to_joint2", "joint4_to_joint3",
       "joint5_to_joint4", "joint6_to_joint5", "joint6output_to_joint6"]
ap = argparse.ArgumentParser()
ap.add_argument("--punkt", default="trichter_greif")
ap.add_argument("--datei", default=os.path.expanduser("~/ros2_ws/teach_punkte.json"))
ap.add_argument("--j1", type=float, default=20.0, help="[Grad] Achse 1 in der Messstellung")
ap.add_argument("--gain", type=float, default=0.6)
ap.add_argument("--runden", type=int, default=4)
ap.add_argument("--toleranz", type=float, default=0.3)
ap.add_argument("--setzzeit", type=float, default=5.0)
ap.add_argument("--ohne-nachstellen", action="store_true")
a = ap.parse_args()

pkt = json.load(open(a.datei))[a.punkt]
ziel = list(pkt["rad"]); ziel[0] = math.radians(a.j1)
print("  Messstellung [Grad]: " + " ".join(f"{math.degrees(v):+7.2f}" for v in ziel))

rclpy.init(); node = Node("messpose_frei"); js = {"m": None}
node.create_subscription(JointState, "/joint_states", lambda m: js.__setitem__("m", m), 10)
arm = ActionClient(node, FollowJointTrajectory, "/arm_controller/follow_joint_trajectory")
mg = ActionClient(node, MoveGroup, "/move_action")
sv = node.create_client(GetStateValidity, "/check_state_validity")
buf = tf2_ros.Buffer(); tf2_ros.TransformListener(buf, node)

def spin(s):
    t = time.time()
    while time.time() - t < s: rclpy.spin_once(node, timeout_sec=0.1)

def ist_rad():
    js["m"] = None
    for _ in range(30):
        rclpy.spin_once(node, timeout_sec=0.1)
        if js["m"] is not None:
            d = dict(zip(js["m"].name, js["m"].position))
            if all(j in d for j in ARM): return [d[j] for j in ARM]
    sys.exit("!! keine /joint_states")

def fahre_gelenke(rad, dauer):
    g = FollowJointTrajectory.Goal(); g.trajectory.joint_names = ARM
    p = JointTrajectoryPoint(); p.positions = [float(v) for v in rad]; p.time_from_start.sec = int(dauer)
    g.trajectory.points = [p]
    f = arm.send_goal_async(g); rclpy.spin_until_future_complete(node, f, timeout_sec=20)
    gh = f.result()
    if gh is None or not gh.accepted: sys.exit("!! Fahrt abgelehnt")
    r = gh.get_result_async(); rclpy.spin_until_future_complete(node, r, timeout_sec=dauer + 30)

# 0) Ausgangslage pruefen: Nullstellung?
ist = ist_rad()
if max(abs(math.degrees(v)) for v in ist) > 5.0:
    sys.exit("!! Arm nicht in der Nullstellung (" + " ".join(f"{math.degrees(v):+.1f}" for v in ist)
             + ") - erst hebe_und_parke.py --null")

# 1) Zielstellung gueltig?
sv.wait_for_service(10)
rq = GetStateValidity.Request(); rs = RobotState(); rs.joint_state.name = list(ARM) + ["gripper_controller"]
rs.joint_state.position = ziel + [0.15]; rq.robot_state = rs; rq.group_name = "arm"
f = sv.call_async(rq); rclpy.spin_until_future_complete(node, f, timeout_sec=10); r = f.result()
if r is None or not r.valid:
    sys.exit("!! Messstellung ungueltig: " + str([(c.contact_body_1, c.contact_body_2) for c in r.contacts] if r else "keine Antwort"))
print("  Messstellung kollisionsfrei.")

# 2) mit MoveIt (kollisionsgeprueft) auf das Gelenkziel
mg.wait_for_server(20)
goal = MoveGroup.Goal(); req = goal.request
req.group_name = "arm"; req.num_planning_attempts = 10; req.allowed_planning_time = 5.0
req.max_velocity_scaling_factor = 0.15; req.max_acceleration_scaling_factor = 0.1
req.workspace_parameters.header.frame_id = "robot_base"
for k in ("min_corner", "max_corner"):
    c = getattr(req.workspace_parameters, k); s = -1.0 if k == "min_corner" else 1.0
    c.x = s; c.y = s; c.z = s
cs = Constraints()
for jn, jv in zip(ARM, ziel):
    jc = JointConstraint(); jc.joint_name = jn; jc.position = float(jv)
    jc.tolerance_above = jc.tolerance_below = 0.02; jc.weight = 1.0; cs.joint_constraints.append(jc)
req.goal_constraints.append(cs); goal.planning_options.plan_only = False
print("  -> MoveIt: Nullstellung -> Messstellung")
f = mg.send_goal_async(goal); rclpy.spin_until_future_complete(node, f, timeout_sec=25); gh = f.result()
if gh is None or not gh.accepted: sys.exit("!! MoveIt-Ziel abgelehnt")
r = gh.get_result_async(); rclpy.spin_until_future_complete(node, r, timeout_sec=120)
if r.result() is None or r.result().result.error_code.val != 1: sys.exit("!! MoveIt-Fahrt fehlgeschlagen")
spin(a.setzzeit)

# 3) exakt auf die Gelenke (direkt), dann nachstellen
fahre_gelenke(ziel, 4); spin(a.setzzeit)
fmt = lambda v: " ".join(f"{math.degrees(x):+7.2f}" for x in v)
kommando = list(ziel); tol = math.radians(a.toleranz)
fehler = [i - z for i, z in zip(ist_rad(), ziel)]
print(f"  Fehler  {fmt(fehler)}   Nick {math.degrees(fehler[1]+fehler[2]+fehler[3]):+.2f}")
if not a.ohne_nachstellen:
    for runde in range(1, a.runden + 1):
        if all(abs(e) <= tol for e in fehler): print(f"  fertig nach {runde-1} Korrektur(en)"); break
        kommando = [k - a.gain * e for k, e in zip(kommando, fehler)]
        print(f"  Runde {runde}: Vorhalt {fmt([k - z for k, z in zip(kommando, ziel)])}")
        fahre_gelenke(kommando, 3); spin(4.0)
        fehler = [i - z for i, z in zip(ist_rad(), ziel)]
        print(f"  Fehler  {fmt(fehler)}   Nick {math.degrees(fehler[1]+fehler[2]+fehler[3]):+.2f}")

# 4) Modell-Neigung aus dem Encoder
tf = None
for _ in range(40):
    rclpy.spin_once(node, timeout_sec=0.1)
    try: tf = buf.lookup_transform("robot_base", "gripper_base", rclpy.time.Time()); break
    except Exception: pass
if tf:
    q = tf.transform.rotation; x, y, z, w = q.x, q.y, q.z, q.w
    xa = (1-2*(y*y+z*z), 2*(x*y+z*w), 2*(x*z-y*w))      # gripper_base x = Fingeroeffnung
    za = (2*(x*z+y*w), 2*(y*z-x*w), 1-2*(x*x+y*y))      # gripper_base z = quer dazu
    print(f"\n  Modell (Encoder): Neigung in Fingerrichtung {math.degrees(math.asin(xa[2])):+.2f} Grad, "
          f"quer dazu {math.degrees(math.asin(za[2])):+.2f} Grad  (+ = Achse zeigt nach oben)")
print("  Arm steht. Jetzt messen (Wasserwaage an Fingeraussenflaeche und an Gehaeuseseite).")
node.destroy_node(); rclpy.shutdown()
