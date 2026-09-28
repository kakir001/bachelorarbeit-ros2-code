#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""Verschiebt den TCP um einen kleinen Betrag — geradlinig, ohne den Greifer zu drehen.

Zum Nachjustieren beim Anlernen: der Arm steht fast richtig, es fehlen ein paar
Millimeter in eine Richtung. Gefahren wird als kartesische Gerade mit
unveraenderter Orientierung, damit nichts an der Rueckseite des Greifers
irgendwo anstoesst.

    python3 tools/versetze_tcp.py --radial -10      # 10 mm zur Basis hin
    python3 tools/versetze_tcp.py --radial +10      # 10 mm von der Basis weg
    python3 tools/versetze_tcp.py --dz 5            # 5 mm hoch
    python3 tools/versetze_tcp.py --dx -8 --dy 3    # in robot_base-Achsen

--radial ist meist gemeint, wenn jemand "vor/zurueck" sagt: entlang der Linie
Roboterbasis -> TCP. Minus = naeher an die Basis.
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
import tf2_ros
import os, sys
sys.path.insert(0, os.path.dirname(os.path.abspath(__file__))); from bahnpruefung import pruefe_bahn  # Stetigkeit (Unfall 13.9.)

ap = argparse.ArgumentParser()
ap.add_argument("--radial", type=float, default=0.0, help="mm entlang Basis->TCP (- = zur Basis)")
ap.add_argument("--dx", type=float, default=0.0)
ap.add_argument("--dy", type=float, default=0.0)
ap.add_argument("--dz", type=float, default=0.0)
ap.add_argument("--dauer", type=float, default=6.0)
a = ap.parse_args()

if a.radial == 0.0 and a.dx == 0.0 and a.dy == 0.0 and a.dz == 0.0:
    sys.exit("nichts zu tun — --radial / --dx / --dy / --dz angeben")

rclpy.init()
node = Node("versetze_tcp")
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

tf = None
t0 = time.time()
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
dx, dy, dz = a.dx, a.dy, a.dz
if a.radial != 0.0:
    r = math.hypot(p.x, p.y)
    if r < 1e-6:
        sys.exit("!! TCP steht auf der Hochachse — --radial nicht sinnvoll")
    dx += a.radial * (p.x / r)
    dy += a.radial * (p.y / r)

print(f"  TCP jetzt:  x {p.x*1000:+.1f}  y {p.y*1000:+.1f}  z {p.z*1000:+.1f} mm"
      f"   (r {math.hypot(p.x,p.y)*1000:.1f})")
print(f"  Versatz:    dx {dx:+.1f}  dy {dy:+.1f}  dz {dz:+.1f} mm")
zx, zy, zz = p.x + dx/1000.0, p.y + dy/1000.0, p.z + dz/1000.0
print(f"  Ziel:       x {zx*1000:+.1f}  y {zy*1000:+.1f}  z {zz*1000:+.1f} mm"
      f"   (r {math.hypot(zx,zy)*1000:.1f})")

cli = node.create_client(GetCartesianPath, "/compute_cartesian_path")
if not cli.wait_for_service(timeout_sec=15.0):
    sys.exit("!! /compute_cartesian_path nicht da")

rq = GetCartesianPath.Request()
rq.header.frame_id = "robot_base"
rq.group_name = "arm"
rq.link_name = "tcp"
rq.max_step = 0.002
rq.jump_threshold = 0.0
rq.avoid_collisions = True
st = RobotState()
st.joint_state = js["msg"]
rq.start_state = st
ziel = Pose()
ziel.position.x = zx
ziel.position.y = zy
ziel.position.z = zz
ziel.orientation = q
rq.waypoints = [ziel]

fut = cli.call_async(rq)
rclpy.spin_until_future_complete(node, fut, timeout_sec=30.0)
res = fut.result()
if res is None:
    sys.exit("!! keine Antwort")
print(f"  gerader Weg geplant: {res.fraction*100:.0f} %")
if res.fraction < 0.90:
    sys.exit(f"!! nur {res.fraction*100:.0f} % planbar — NICHT gefahren")
ok_b, txt_b = pruefe_bahn(res.solution.joint_trajectory, ARM)
print("  Bahnprobe: " + txt_b)
if not ok_b: sys.exit("!! " + txt_b + " - NICHT gefahren")

traj = res.solution.joint_trajectory
n = len(traj.points)
for i, pt in enumerate(traj.points):
    t = a.dauer * (i + 1) / n
    pt.time_from_start.sec = int(t)
    pt.time_from_start.nanosec = int((t - int(t)) * 1e9)
    pt.velocities = []
    pt.accelerations = []

ac = ActionClient(node, FollowJointTrajectory, "/arm_controller/follow_joint_trajectory")
if not ac.wait_for_server(timeout_sec=15.0):
    sys.exit("!! arm_controller nicht da")
g = FollowJointTrajectory.Goal()
g.trajectory = traj
print("  fahre (langsam) ...")
f = ac.send_goal_async(g)
rclpy.spin_until_future_complete(node, f, timeout_sec=20.0)
gh = f.result()
if gh is None or not gh.accepted:
    sys.exit("!! abgelehnt")
rf = gh.get_result_async()
rclpy.spin_until_future_complete(node, rf, timeout_sec=a.dauer + 30.0)
print("  fertig.")

node.destroy_node()
rclpy.shutdown()
