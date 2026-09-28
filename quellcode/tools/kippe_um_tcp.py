#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""Den Greifer um die FINGERSPITZE kippen - die Spitze bleibt, wo sie ist.

Wozu: nach dem Nachstellen (tools/nachstellen.py) meldet der Encoder den Greifer
senkrecht, der Benutzer sieht ihn aber schief - das ist das Spiel HINTER dem
Getriebe, das kein Encoder sieht. Solange es nicht von aussen vermessen ist,
wird es nach Augenmass korrigiert: "2 Grad zur Basis kippen". Damit die Finger
dabei nicht von der Welle rutschen, wird um den TCP gedreht, nicht um ein Gelenk
(2 Grad um Achse 4 wuerden die Spitze rund 6 mm versetzen).

Weg: compute_cartesian_path von der Ist-Stellung auf dieselbe TCP-Position mit
gekippter Orientierung - ein Wegpunkt, Orientierung wird interpoliert.

    python3 tools/kippe_um_tcp.py --radial 2     # Greifer-OBERSEITE 2 Grad zur Basis hin
    python3 tools/kippe_um_tcp.py --radial -2    # Oberseite 2 Grad von der Basis weg
    python3 tools/kippe_um_tcp.py --seitlich 1   # Oberseite 1 Grad nach links (gegen den Uhrzeigersinn um die Basis)
"""
import argparse
import math
import sys
import time

import rclpy
from rclpy.node import Node
from rclpy.action import ActionClient
from control_msgs.action import FollowJointTrajectory
from geometry_msgs.msg import Pose
from moveit_msgs.msg import RobotState
from moveit_msgs.srv import GetCartesianPath, GetPositionFK
from sensor_msgs.msg import JointState
from tf2_ros import Buffer, TransformListener
import os, sys
sys.path.insert(0, os.path.dirname(os.path.abspath(__file__))); from bahnpruefung import pruefe_bahn  # Stetigkeit (Unfall 13.9.)

ARM = ["joint2_to_joint1", "joint3_to_joint2", "joint4_to_joint3",
       "joint5_to_joint4", "joint6_to_joint5", "joint6output_to_joint6"]

ap = argparse.ArgumentParser()
ap.add_argument("--radial", type=float, default=0.0,
                help="Grad; + = Oberseite des Greifers zur Basis hin, Spitze von der Basis weg")
ap.add_argument("--seitlich", type=float, default=0.0,
                help="Grad; + = Oberseite nach links (Blick von der Basis zum TCP)")
ap.add_argument("--dauer", type=float, default=5.0)
a = ap.parse_args()
if a.radial == 0 and a.seitlich == 0:
    sys.exit("nichts zu tun")


def q_mul(p, q):
    x1, y1, z1, w1 = p; x2, y2, z2, w2 = q
    return (w1*x2 + x1*w2 + y1*z2 - z1*y2,
            w1*y2 - x1*z2 + y1*w2 + z1*x2,
            w1*z2 + x1*y2 - y1*x2 + z1*w2,
            w1*w2 - x1*x2 - y1*y2 - z1*z2)


def q_axis(ax, grad):
    s = math.sin(math.radians(grad) / 2)
    n = math.sqrt(sum(v*v for v in ax))
    return (ax[0]/n*s, ax[1]/n*s, ax[2]/n*s, math.cos(math.radians(grad) / 2))


class N(Node):
    def __init__(self):
        super().__init__("kippe_um_tcp")
        self.js = {}
        self.create_subscription(JointState, "/joint_states", self._cb, 10)
        self.fk = self.create_client(GetPositionFK, "/compute_fk")
        self.cart = self.create_client(GetCartesianPath, "/compute_cartesian_path")
        self.ac = ActionClient(self, FollowJointTrajectory, "/arm_controller/follow_joint_trajectory")
        for c, nm in ((self.fk, "compute_fk"), (self.cart, "compute_cartesian_path")):
            if not c.wait_for_service(timeout_sec=10):
                sys.exit(f"!! /{nm} fehlt")
        if not self.ac.wait_for_server(timeout_sec=10):
            sys.exit("!! arm_controller fehlt")

    def _cb(self, m):
        self.js.update(dict(zip(m.name, m.position)))

    def call(self, cli, rq):
        f = cli.call_async(rq)
        rclpy.spin_until_future_complete(self, f, timeout_sec=10)
        return f.result()

    def ist_state(self):
        self.js = {}
        t = time.time()
        while time.time() - t < 3 and not all(j in self.js for j in ARM):
            rclpy.spin_once(self, timeout_sec=0.1)
        rs = RobotState()
        rs.joint_state.name = list(ARM)
        rs.joint_state.position = [self.js[j] for j in ARM]
        return rs

    def tcp(self, rs):
        rq = GetPositionFK.Request()
        rq.header.frame_id = "robot_base"
        rq.fk_link_names = ["tcp"]
        rq.robot_state = rs
        return self.call(self.fk, rq).pose_stamped[0].pose


rclpy.init()
n = N()
rs = n.ist_state()
p = n.tcp(rs)
pos = p.position
print(f"  TCP jetzt: x {pos.x*1000:+.1f}  y {pos.y*1000:+.1f}  z {pos.z*1000:+.1f} mm")

# Achsen in robot_base: radial = Basis -> TCP (waagerecht), tangential = links davon
r = math.hypot(pos.x, pos.y)
radial = (pos.x / r, pos.y / r, 0.0)
tangential = (-radial[1], radial[0], 0.0)
# Oberseite zur Basis kippen = Drehung um die TANGENTIALE Achse; Vorzeichen so, dass
# +radial die Oberseite (oben, +z) zur Basis (-radial) bewegt: Rechte-Hand um tangential
# dreht +z nach +radial -> also -grad.
q = (p.orientation.x, p.orientation.y, p.orientation.z, p.orientation.w)
if a.radial:
    q = q_mul(q_axis(tangential, -a.radial), q)
if a.seitlich:
    # Oberseite nach links = +z nach +tangential: Drehung um radial mit -grad
    q = q_mul(q_axis(radial, -a.seitlich), q)

ziel = Pose()
ziel.position.x, ziel.position.y, ziel.position.z = pos.x, pos.y, pos.z
ziel.orientation.x, ziel.orientation.y, ziel.orientation.z, ziel.orientation.w = q
print(f"  Kippen: radial {a.radial:+.1f} Grad, seitlich {a.seitlich:+.1f} Grad - Spitze bleibt")

rq = GetCartesianPath.Request()
rq.header.frame_id = "robot_base"
rq.group_name = "arm"
rq.link_name = "tcp"
rq.start_state = rs
rq.waypoints = [ziel]
rq.max_step = 0.002
rq.jump_threshold = 0.0
rq.avoid_collisions = True
res = n.call(n.cart, rq)
if res is None or res.fraction < 0.99:
    sys.exit(f"!! nur {0 if res is None else res.fraction*100:.0f} % planbar - nicht gefahren")
ok_b, txt_b = pruefe_bahn(res.solution.joint_trajectory, ARM)
print("  Bahnprobe: " + txt_b)
if not ok_b: sys.exit("!! " + txt_b + " - NICHT gefahren")
traj = res.solution.joint_trajectory
print(f"  Weg geplant: {len(traj.points)} Punkte")
# Erster Punkt = Ist (Durchhaengen nicht zurueckziehen), Zeit gleichmaessig verteilen
traj.points[0].positions = list(rs.joint_state.position)
m = len(traj.points)
for i, pt in enumerate(traj.points):
    t = a.dauer * (i + 1) / m
    pt.time_from_start.sec = int(t)
    pt.time_from_start.nanosec = int((t % 1) * 1e9)
    pt.velocities = []; pt.accelerations = []
g = FollowJointTrajectory.Goal()
g.trajectory = traj
f = n.ac.send_goal_async(g)
rclpy.spin_until_future_complete(n, f)
r_ = f.result().get_result_async()
rclpy.spin_until_future_complete(n, r_, timeout_sec=a.dauer + 15)
print("  fertig." if r_.result().result.error_code == 0 else f"!! error_code {r_.result().result.error_code}")
n.destroy_node()
rclpy.shutdown()
