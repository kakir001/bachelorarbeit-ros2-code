#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""Achse 4 (und seit 20.9. auch Achse 2) immer von OBEN auf ihr Soll bringen (Spiel hinter dem Getriebe).

Befund am Roboter (Handy-Wasserwaage am Greifer, nest_11): dasselbe Encoder-Soll von J4 ergibt real bis zu
7 Grad verschiedene Greiferneigung, je nachdem, ob J4 zuletzt nach oben oder nach unten gefahren ist
(J4 +2 kommandiert -> real -7 Grad). Das Spiel im Abtrieb von Achse 4 legt sich auf die Seite der letzten
Bewegungsrichtung. Deshalb: J4 zuerst um --hub Grad UEBER das Soll, dann direkt auf das Soll herunter,
dann nachstellen (Totband). Alle anderen Gelenke bleiben auf ihrem Soll.

    python3 tools/j4_von_oben.py --soll -85.29 23.12 -140.81 33.48 1.76 130.64
    python3 tools/j4_von_oben.py --soll ... --hub 8
"""
import argparse, math, os, subprocess, sys, time
import rclpy
from rclpy.node import Node
from rclpy.action import ActionClient
from control_msgs.action import FollowJointTrajectory
from trajectory_msgs.msg import JointTrajectoryPoint
ARM = ["joint2_to_joint1", "joint3_to_joint2", "joint4_to_joint3",
       "joint5_to_joint4", "joint6_to_joint5", "joint6output_to_joint6"]
ap = argparse.ArgumentParser()
ap.add_argument("--soll", type=float, nargs=6, required=True, help="Soll-Gelenkwinkel [Grad]")
ap.add_argument("--hub", type=float, default=8.0, help="[Grad] wie weit die Gelenke zuerst ueber das Soll fahren")
ap.add_argument("--gelenke", type=int, nargs="+", default=[2, 4], help="welche Achsen von oben (Vorgabe 2 und 4; 19.9. nur 4)")
ap.add_argument("--dauer", type=float, default=4.0)
a = ap.parse_args()
rclpy.init(); n = Node("j4_von_oben")
ac = ActionClient(n, FollowJointTrajectory, "/arm_controller/follow_joint_trajectory"); ac.wait_for_server(10)
idx = [g - 1 for g in a.gelenke]
for schritt, hub in (("hoch", a.hub), ("Soll", 0.0)):
    q = list(a.soll)
    for i in idx: q[i] += hub
    g = FollowJointTrajectory.Goal(); g.trajectory.joint_names = ARM
    p = JointTrajectoryPoint(); p.positions = [math.radians(v) for v in q]
    p.time_from_start.sec = int(a.dauer); g.trajectory.points = [p]
    f = ac.send_goal_async(g); rclpy.spin_until_future_complete(n, f, timeout_sec=10); gh = f.result()
    if gh is None or not gh.accepted: sys.exit("!! Fahrt abgelehnt")
    r = gh.get_result_async(); rclpy.spin_until_future_complete(n, r, timeout_sec=a.dauer + 20)
    print(f"  {schritt}: " + " ".join(f"J{i+1} {q[i]:+.2f}" for i in idx))
    t = time.time()
    while time.time() - t < 2: rclpy.spin_once(n, timeout_sec=0.1)
n.destroy_node(); rclpy.shutdown()
sys.exit(subprocess.call([sys.executable, os.path.join(os.path.dirname(os.path.abspath(__file__)), "nachstellen.py"),
                          "--ziel", *[f"{v:.3f}" for v in a.soll], "--toleranz", "0.3", "--runden", "4", "--setzzeit", "4"]))
