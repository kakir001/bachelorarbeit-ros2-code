#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""Erst SENKRECHT anheben, dann zur Nullstellung — ohne den Greifer zu drehen.

Warum nicht direkt zur Nullstellung: der Weg dorthin dreht den Greifer, und die
Teile an seiner Rueckseite streifen dann alles, was danebensteht (der Trichter
wurde so schon einmal verschoben). Trichter und Ablageplatte stehen NICHT im
Kollisionsmodell — MoveIt kann sie also gar nicht sehen und weicht ihnen nicht
aus. Deshalb wird die erste Bewegung rein senkrecht gefahren: gleiche x/y,
gleiche Orientierung, nur z hoch. Was daneben steht, kann dabei nicht getroffen
werden.

    python3 tools/hebe_und_parke.py --hoehe 160        # nur anheben
    python3 tools/hebe_und_parke.py --hoehe 160 --null # anheben, dann Nullstellung
"""
import argparse
import math
import sys
import time

import rclpy
from rclpy.action import ActionClient
from rclpy.node import Node
from geometry_msgs.msg import Pose
from moveit_msgs.srv import GetCartesianPath
from moveit_msgs.msg import RobotState
from sensor_msgs.msg import JointState
from control_msgs.action import FollowJointTrajectory
from trajectory_msgs.msg import JointTrajectoryPoint
import tf2_ros
import os, sys
sys.path.insert(0, os.path.dirname(os.path.abspath(__file__))); from bahnpruefung import pruefe_bahn  # Stetigkeit (Unfall 13.9.)

ARM_JOINTS = ['joint2_to_joint1', 'joint3_to_joint2', 'joint4_to_joint3',
              'joint5_to_joint4', 'joint6_to_joint5', 'joint6output_to_joint6']

ap = argparse.ArgumentParser()
ap.add_argument("--hoehe", type=float, default=160.0, help="Ziel-z [mm]")
ap.add_argument("--null", action="store_true", help="danach zur Nullstellung")
ap.add_argument("--dauer", type=float, default=8.0, help="Fahrzeit anheben [s]")
ap.add_argument("--null-dauer", type=float, default=10.0)
a = ap.parse_args()

rclpy.init()
node = Node("hebe_und_parke")

js = {"msg": None}
node.create_subscription(JointState, "/joint_states",
                         lambda m: js.__setitem__("msg", m), 10)
buf = tf2_ros.Buffer()
tf2_ros.TransformListener(buf, node)

t0 = time.time()
while time.time() - t0 < 15.0 and js["msg"] is None:
    rclpy.spin_once(node, timeout_sec=0.2)
if js["msg"] is None:
    sys.exit("!! keine /joint_states")

t0 = time.time()
tf = None
while time.time() - t0 < 15.0:
    rclpy.spin_once(node, timeout_sec=0.2)
    try:
        tf = buf.lookup_transform("robot_base", "tcp", rclpy.time.Time())
        break
    except Exception:
        pass
if tf is None:
    sys.exit("!! keine TF robot_base->tcp")

p = tf.transform.translation
q = tf.transform.rotation
print(f"  TCP jetzt:  x {p.x*1000:+.1f}  y {p.y*1000:+.1f}  z {p.z*1000:+.1f} mm")
print(f"  Ziel:       x {p.x*1000:+.1f}  y {p.y*1000:+.1f}  z {a.hoehe:+.1f} mm  (rein senkrecht)")

if abs(a.hoehe / 1000.0 - p.z) < 0.002:
    print("  schon auf dieser Hoehe — nichts zu tun")
else:
    cli = node.create_client(GetCartesianPath, "/compute_cartesian_path")
    if not cli.wait_for_service(timeout_sec=15.0):
        sys.exit("!! /compute_cartesian_path nicht da")

    rq = GetCartesianPath.Request()
    rq.header.frame_id = "robot_base"
    rq.group_name = "arm"
    rq.link_name = "tcp"
    rq.max_step = 0.005
    rq.jump_threshold = 0.0
    rq.avoid_collisions = True
    st = RobotState()
    st.joint_state = js["msg"]
    rq.start_state = st
    ziel = Pose()
    ziel.position.x = p.x
    ziel.position.y = p.y
    ziel.position.z = a.hoehe / 1000.0
    ziel.orientation = q          # Orientierung bleibt, der Greifer dreht sich nicht
    rq.waypoints = [ziel]

    fut = cli.call_async(rq)
    rclpy.spin_until_future_complete(node, fut, timeout_sec=30.0)
    res = fut.result()
    if res is None:
        sys.exit("!! keine Antwort vom Cartesian-Dienst")
    print(f"  senkrechter Weg geplant: {res.fraction*100:.0f} % der Strecke, "
          f"{len(res.solution.joint_trajectory.points)} Punkte")
    ok_b, txt_b = pruefe_bahn(res.solution.joint_trajectory, ARM)
    if not ok_b: sys.exit("!! " + txt_b + " - NICHT gefahren")
    if res.fraction < 0.90:
        sys.exit(f"!! nur {res.fraction*100:.0f} % planbar — NICHT gefahren. "
                 "Arm von Hand freimachen.")

    traj = res.solution.joint_trajectory
    # Der Planer rechnet vom MODELL aus. Weil der Arm durchhaengt, steht er in
    # Wirklichkeit woanders — die Bahn beginnt also neben seiner echten Lage.
    # Der Benutzer sah das als "zieht am Anfang leicht nach vorn"; solange die
    # Welle im Auslauf steckt, ist das gefaehrlich.
    #
    # NUR den ersten Bahnpunkt zu ersetzen macht es SCHLIMMER: dann springt die
    # Bahn im zweiten Schritt auf die Modell-Lage zurueck. Gemessen 2026-09-10:
    # 5 mm hoch befohlen -> tatsaechlich 10 mm zur Seite und 4 mm nach unten.
    #
    # Richtig ist, die GANZE Bahn um dieselbe Differenz zu verschieben: die
    # relative Bewegung bleibt erhalten, und sie beginnt dort, wo der Arm
    # wirklich steht.
    if traj.points:
        d = dict(zip(js["msg"].name, js["msg"].position))
        if all(j in d for j in traj.joint_names):
            versatz = [d[j] - p0 for j, p0 in
                       zip(traj.joint_names, traj.points[0].positions)]
            gross = max(abs(v) for v in versatz)
            if gross > 0.001:
                print(f"  (ganze Bahn um bis zu {math.degrees(gross):.2f} Grad "
                      f"verschoben — Durchhaengen)")
            for pt in traj.points:
                pt.positions = [w + v for w, v in zip(pt.positions, versatz)]

    n = len(traj.points)
    for i, pt in enumerate(traj.points):        # gleichmaessig langsam machen
        t = a.dauer * (i + 1) / n
        pt.time_from_start.sec = int(t)
        pt.time_from_start.nanosec = int((t - int(t)) * 1e9)
        pt.velocities = []
        pt.accelerations = []

    ac = ActionClient(node, FollowJointTrajectory,
                      "/arm_controller/follow_joint_trajectory")
    if not ac.wait_for_server(timeout_sec=15.0):
        sys.exit("!! arm_controller Action nicht da")
    g = FollowJointTrajectory.Goal()
    g.trajectory = traj
    print("  fahre senkrecht (langsam) ...")
    f = ac.send_goal_async(g)
    rclpy.spin_until_future_complete(node, f, timeout_sec=20.0)
    gh = f.result()
    if gh is None or not gh.accepted:
        sys.exit("!! Anheben abgelehnt")
    rf = gh.get_result_async()
    rclpy.spin_until_future_complete(node, rf, timeout_sec=a.dauer + 30.0)
    print("  angehoben.")

if a.null:
    # 2026-09-12 Nacht: bis hierher war das eine REINE Gelenkfahrt (FollowJointTrajectory,
    # keine Kollisionspruefung) - auf dem Weg von der Schale in die Nullstellung schwenkte der
    # Greifer tief und stiess an die Schale. Jetzt MoveIt-Gelenkziel: kollisionsgeprueft gegen
    # Schale, Trichter, Ablageplatte, Sockel (alles im Modell), langsam.
    print("\n  jetzt zur Nullstellung (MoveIt, kollisionsgeprueft) ...")
    from moveit_msgs.action import MoveGroup
    from moveit_msgs.msg import Constraints, JointConstraint
    mg = ActionClient(node, MoveGroup, "/move_action")
    if not mg.wait_for_server(timeout_sec=20.0):
        sys.exit("!! /move_action nicht da")
    goal = MoveGroup.Goal(); req = goal.request
    req.group_name = "arm"; req.num_planning_attempts = 10; req.allowed_planning_time = 5.0
    req.max_velocity_scaling_factor = 0.3; req.max_acceleration_scaling_factor = 0.1
    req.workspace_parameters.header.frame_id = "robot_base"
    for k_, sgn in (("min_corner", -1.0), ("max_corner", 1.0)):
        c = getattr(req.workspace_parameters, k_); c.x = c.y = c.z = sgn
    import os; sys.path.insert(0, os.path.dirname(os.path.abspath(__file__))); from gelenkspiel import nullstellung_rad
    null_rad = nullstellung_rad()
    cs = Constraints()
    for jn in ARM_JOINTS:
        jc = JointConstraint(); jc.joint_name = jn; jc.position = float(null_rad[ARM.index(jn)])   # reale Nullstellung (20.9.)
        jc.tolerance_above = jc.tolerance_below = 0.02; jc.weight = 1.0; cs.joint_constraints.append(jc)
    req.goal_constraints.append(cs); goal.planning_options.plan_only = False
    f = mg.send_goal_async(goal); rclpy.spin_until_future_complete(node, f, timeout_sec=25.0)
    gh = f.result()
    if gh is None or not gh.accepted:
        sys.exit("!! Nullfahrt (MoveIt) abgelehnt")
    rf = gh.get_result_async(); rclpy.spin_until_future_complete(node, rf, timeout_sec=120.0)
    if rf.result() is None or rf.result().result.error_code.val != 1:
        sys.exit("!! Nullfahrt (MoveIt) fehlgeschlagen - Arm steht, wo er ist")
    print("  Nullstellung erreicht.")

node.destroy_node()
rclpy.shutdown()
