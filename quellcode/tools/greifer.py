#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""Nur den Greifer fahren (gripper_controller, ROS 2). Fuer die GUI-Knoepfe neben NOT-AUS (Benutzer 14.9.: nach
einem Abbruch haengt manchmal noch eine Welle in den Fingern - von Hand loslassen, ohne den Zyklus neu zu starten).
    python3 tools/greifer.py --auf [25]   # Spitzenoeffnung in mm (Messkurve 14.9.: 9..40)
    python3 tools/greifer.py --zu         # -0.74
"""
import argparse, sys, rclpy
from rclpy.node import Node
from rclpy.action import ActionClient
from control_msgs.action import FollowJointTrajectory
from trajectory_msgs.msg import JointTrajectoryPoint
KURVE = [(9.0, -0.740), (17.0, -0.464), (25.0, -0.304), (40.0, 0.150)]
def gelenk(mm):
    mm = min(40.0, max(9.0, mm))
    for (m0, j0), (m1, j1) in zip(KURVE, KURVE[1:]):
        if mm <= m1: return j0 + (j1 - j0) * (mm - m0) / (m1 - m0)
    return 0.150
ap = argparse.ArgumentParser(); ap.add_argument("--auf", type=float, nargs="?", const=25.0, default=None); ap.add_argument("--zu", action="store_true")
a = ap.parse_args()
wert = -0.74 if a.zu else gelenk(a.auf if a.auf is not None else 25.0)
rclpy.init(); n = Node("greifer_knopf"); c = ActionClient(n, FollowJointTrajectory, "/gripper_controller/follow_joint_trajectory")
if not c.wait_for_server(10): print("!! gripper_controller fehlt"); sys.exit(1)
g = FollowJointTrajectory.Goal(); g.trajectory.joint_names = ["gripper_controller"]
p = JointTrajectoryPoint(); p.positions = [float(wert)]; p.time_from_start.sec = 3; g.trajectory.points = [p]
f = c.send_goal_async(g); rclpy.spin_until_future_complete(n, f, timeout_sec=10); gh = f.result()
if gh is None or not gh.accepted: print("!! abgelehnt"); sys.exit(1)
r = gh.get_result_async(); rclpy.spin_until_future_complete(n, r, timeout_sec=15)
print("Greifer %s (Gelenk %+.3f)" % ("ZU" if a.zu else "AUF %.0f mm" % (a.auf if a.auf is not None else 25.0), wert))
n.destroy_node(); rclpy.shutdown()
