#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""Kleine gerade Verschiebung, gerechnet auf SOLL (gespeicherte Gelenkwinkel), nicht auf Ist.

versetze_tcp.py rechnet vom gemessenen TCP aus - das Durchhaengen wandert dann mit
jedem Schritt mit (2026-09-11 nachts: drei Schritte, 9 mm tiefer und 11 mm zur
Basis, ohne dass es gewollt war). Hier ist der Ausgangspunkt die SOLL-Stellung
(sechs Gelenkwinkel, z.B. aus teach_punkte.json), das Ziel = Soll-TCP + Versatz,
und nach der Fahrt wird auf die Ziel-Gelenke nachgestellt (tools/nachstellen.py).
Der erste Bahnpunkt wird durch die Ist-Stellung ersetzt, damit der Arm nicht
zuerst auf das Modell zurueckgezogen wird.

    python3 tools/versetze_soll.py --punkt trichter_greif3 --dz 2
    python3 tools/versetze_soll.py --grad -40.68 -7.96 -115.22 30.6 -0.29 130.58 --dz 2 --radial -1
Gibt am Ende die neuen Soll-Gelenkwinkel aus (fuer den naechsten Schritt).
"""
import argparse, json, math, os, subprocess, sys, time
import rclpy
from rclpy.node import Node
from rclpy.action import ActionClient
from control_msgs.action import FollowJointTrajectory
from geometry_msgs.msg import Pose
from moveit_msgs.msg import RobotState
from moveit_msgs.srv import GetCartesianPath, GetPositionFK
from sensor_msgs.msg import JointState
sys.path.insert(0, os.path.dirname(os.path.abspath(__file__))); from bahnpruefung import pruefe_bahn  # Stetigkeit (Unfall 13.9.)

ARM = ["joint2_to_joint1", "joint3_to_joint2", "joint4_to_joint3",
       "joint5_to_joint4", "joint6_to_joint5", "joint6output_to_joint6"]
ap = argparse.ArgumentParser()
ap.add_argument("--punkt"); ap.add_argument("--grad", type=float, nargs=6)
ap.add_argument("--datei", default=os.path.expanduser("~/ros2_ws/teach_punkte.json"))
ap.add_argument("--dx", type=float, default=0.0); ap.add_argument("--dy", type=float, default=0.0)
ap.add_argument("--dz", type=float, default=0.0); ap.add_argument("--radial", type=float, default=0.0)
ap.add_argument("--dauer", type=float, default=5.0)
ap.add_argument("--ohne-nachstellen", action="store_true")
ap.add_argument("--ohne-kollision", action="store_true", help="Bahn OHNE Kollisionspruefung (nur wenn das Modell nachweislich falsch liegt, z.B. Finger im Trichterkragen)")
a = ap.parse_args()
if a.grad: soll = list(a.grad)
elif a.punkt: soll = json.load(open(a.datei))[a.punkt]["grad"]
else: sys.exit("--punkt oder --grad")

class N(Node):
    def __init__(s):
        super().__init__("versetze_soll"); s.js = {}
        s.create_subscription(JointState, "/joint_states", lambda m: s.js.update(dict(zip(m.name, m.position))), 10)
        s.fk = s.create_client(GetPositionFK, "/compute_fk"); s.cart = s.create_client(GetCartesianPath, "/compute_cartesian_path")
        s.ac = ActionClient(s, FollowJointTrajectory, "/arm_controller/follow_joint_trajectory")
        s.fk.wait_for_service(10); s.cart.wait_for_service(10); s.ac.wait_for_server(10)
    def call(s, c, rq, timeout=10):
        f = c.call_async(rq); rclpy.spin_until_future_complete(s, f, timeout_sec=timeout); return f.result()
    def ist(s):
        s.js = {}; t = time.time()
        while time.time() - t < 3 and not all(j in s.js for j in ARM): rclpy.spin_once(s, timeout_sec=0.1)
        return [s.js[j] for j in ARM]

rclpy.init(); n = N()
rs = RobotState(); rs.joint_state.name = list(ARM); rs.joint_state.position = [math.radians(v) for v in soll]
rq = GetPositionFK.Request(); rq.header.frame_id = "robot_base"; rq.fk_link_names = ["tcp"]; rq.robot_state = rs
p = n.call(n.fk, rq).pose_stamped[0].pose
r = math.hypot(p.position.x, p.position.y)
dx = a.dx + a.radial * p.position.x / r; dy = a.dy + a.radial * p.position.y / r; dz = a.dz
print(f"  Soll-TCP: x {p.position.x*1000:+.1f} y {p.position.y*1000:+.1f} z {p.position.z*1000:+.1f} mm  ->  Versatz dx {dx:+.1f} dy {dy:+.1f} dz {dz:+.1f}")
ziel = Pose(); ziel.orientation = p.orientation
ziel.position.x = p.position.x + dx/1000; ziel.position.y = p.position.y + dy/1000; ziel.position.z = p.position.z + dz/1000
# 2026-09-12: Bahn ab IST planen, nicht ab Soll. Vorher: Bahn ab Soll, erster Punkt = Ist ->
# stand der Arm (Totband) 5-10 mm ueber/unter Soll, tauchte er erst auf die Soll-Hoehe ab,
# fuhr dann seitlich und wurde vom Nachstellen wieder hochgeholt (Benutzer: "sackt ab,
# kommt wieder hoch, mehrmals, kommt oft nicht an"). Das ZIEL bleibt aus dem Soll gerechnet,
# damit sich nichts aufsummiert; nur der Startpunkt der geraden Linie ist die Wirklichkeit.
ist_jetzt = n.ist()
rs_ist = RobotState(); rs_ist.joint_state.name = list(ARM); rs_ist.joint_state.position = [float(v) for v in ist_jetzt]
cq = GetCartesianPath.Request(); cq.header.frame_id = "robot_base"; cq.group_name = "arm"; cq.link_name = "tcp"
cq.start_state = rs_ist; cq.waypoints = [ziel]; cq.max_step = 0.001; cq.jump_threshold = 0.0; cq.avoid_collisions = not a.ohne_kollision
# Kollisionspruefung je Bahnpunkt dauert auf dem Nano (Trichter: 183 Koerper) - 20 mm in 1-mm-Schritten > 10 s
res = n.call(n.cart, cq, timeout=120)
if res is None: sys.exit("!! compute_cartesian_path: Timeout (120 s)")
if res.fraction < 0.99: sys.exit(f"!! nur {res.fraction*100:.0f} % planbar")
ok_b, txt_b = pruefe_bahn(res.solution.joint_trajectory, ARM, start=list(ist_jetzt))   # Ist [rad], nicht soll [Grad]
print("  Bahnprobe: " + txt_b)
if not ok_b: sys.exit("!! " + txt_b + " - NICHT gefahren")
traj = res.solution.joint_trajectory
ziel_grad = [round(math.degrees(v), 2) for v in traj.points[-1].positions]
traj.points[0].positions = ist_jetzt   # Bahn beginnt ohnehin im Ist
m = len(traj.points)
for i, pt in enumerate(traj.points):
    t = a.dauer * (i + 1) / m; pt.time_from_start.sec = int(t); pt.time_from_start.nanosec = int((t % 1) * 1e9)
    pt.velocities = []; pt.accelerations = []
g = FollowJointTrajectory.Goal(); g.trajectory = traj
f = n.ac.send_goal_async(g); rclpy.spin_until_future_complete(n, f)
rr = f.result().get_result_async(); rclpy.spin_until_future_complete(n, rr, timeout_sec=a.dauer + 15)
print("  gefahren." if rr.result().result.error_code == 0 else "!! Fahrt fehlgeschlagen")
print("  NEUE SOLL-GELENKE [Grad]:", " ".join(f"{v:.2f}" for v in ziel_grad))
n.destroy_node(); rclpy.shutdown()
if not a.ohne_nachstellen:
    time.sleep(3.0)   # 23.9.: Arm haengt nach dem Bahnende 1-2 s hinterher - sonst liest nachstellen 20 Grad "Fehler" und bricht ab
    subprocess.run([sys.executable, os.path.expanduser("~/ros2_ws/tools/nachstellen.py"), "--ziel", *[str(v) for v in ziel_grad], "--setzzeit", "4", "--dauer", "3"])
