#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""Kollisionsgeprueft in die Nullstellung (MoveIt). Ziel = REALE Nullstellung aus teach_punkte.json (20.9.: J2 +3, J3 +1, J4 -0.5 -
Encoder-Nullpunkte verschoben, s. gelenk_nullpunkte.json); --ziel-grad 0 0 0 0 0 0 = Encoder-Null. Greifer unveraendert.
    python3 tools/nullstellung_moveit.py [--tempo 2] [--ziel-grad j1 j2 j3 j4 j5 j6]
"""
import argparse, math, sys, time, rclpy
from rclpy.action import ActionClient
from rclpy.node import Node
from moveit_msgs.action import MoveGroup
from moveit_msgs.msg import Constraints, JointConstraint
ARM = ['joint2_to_joint1','joint3_to_joint2','joint4_to_joint3','joint5_to_joint4','joint6_to_joint5','joint6output_to_joint6']
ap = argparse.ArgumentParser(); ap.add_argument("--tempo", type=float, default=2.0); ap.add_argument("--ziel-grad", type=float, nargs=6, default=None, help="Vorgabe: reale Nullstellung aus teach_punkte.json (20.9.), sonst 0")
a = ap.parse_args()
import os; sys.path.insert(0, os.path.dirname(os.path.abspath(__file__))); from gelenkspiel import nullstellung_rad
if a.ziel_grad is None: a.ziel_grad = [math.degrees(v) for v in nullstellung_rad()]
rclpy.init(); n = Node("nullstellung_moveit"); mg = ActionClient(n, MoveGroup, "/move_action")
if not mg.wait_for_server(20): sys.exit("!! /move_action fehlt")
goal = MoveGroup.Goal(); req = goal.request
req.group_name = "arm"; req.num_planning_attempts = 10; req.allowed_planning_time = 5.0
req.max_velocity_scaling_factor = min(1.0, 0.15 * a.tempo); req.max_acceleration_scaling_factor = 0.1
req.workspace_parameters.header.frame_id = "robot_base"
for k, sgn in (("min_corner", -1.0), ("max_corner", 1.0)):
    c = getattr(req.workspace_parameters, k); c.x = c.y = c.z = sgn
cs = Constraints()
for jn, jv in zip(ARM, a.ziel_grad):
    jc = JointConstraint(); jc.joint_name = jn; jc.position = math.radians(jv); jc.tolerance_above = jc.tolerance_below = 0.02; jc.weight = 1.0; cs.joint_constraints.append(jc)
req.goal_constraints.append(cs); goal.planning_options.plan_only = False
print("  -> MoveIt (kollisionsgeprueft): Ziel " + " ".join(f"{v:+.1f}" for v in a.ziel_grad))
f = mg.send_goal_async(goal); rclpy.spin_until_future_complete(n, f, timeout_sec=25); gh = f.result()
if gh is None or not gh.accepted: sys.exit("!! MoveIt-Ziel abgelehnt")
r = gh.get_result_async(); rclpy.spin_until_future_complete(n, r, timeout_sec=120)
ok = r.result() is not None and r.result().result.error_code.val == 1
print("  angekommen." if ok else f"!! fehlgeschlagen (code {r.result().result.error_code.val if r.result() else '?'})")
n.destroy_node(); rclpy.shutdown(); sys.exit(0 if ok else 1)
