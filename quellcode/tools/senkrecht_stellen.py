#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""Greifer an einer Stelle SENKRECHT stellen: IK (senkrecht, fester Gierwinkel) fuer die
gewuenschte TCP-Lage, Gelenke direkt anfahren, dann gedaempft nachstellen (Totband).

Wozu: nach einer kartesischen Fahrt bleibt der Greifer 3-4 Grad schief (2026-09-11 an
nest_11: quer +3.15 Grad), weil die Servos im Totband stehen bleiben. Hier wird auf die
IK-Gelenke nachgestellt, bis der Encoder sie meldet - dann ist das MODELL senkrecht.
Was der Encoder nicht sieht (Spiel hinter dem Getriebe), bleibt.

    python3 tools/senkrecht_stellen.py --x -35.6 --y -154.0 --z 56 --yaw 180
    python3 tools/senkrecht_stellen.py --hier --z 56        # x/y = aktuelle TCP-Lage
"""
import argparse, math, sys, time
import rclpy
from rclpy.node import Node
from rclpy.action import ActionClient
from control_msgs.action import FollowJointTrajectory
from geometry_msgs.msg import PoseStamped, Quaternion
from moveit_msgs.srv import GetPositionIK
from moveit_msgs.msg import RobotState
from sensor_msgs.msg import JointState
from trajectory_msgs.msg import JointTrajectoryPoint
from builtin_interfaces.msg import Duration
import tf2_ros

ARM = ["joint2_to_joint1", "joint3_to_joint2", "joint4_to_joint3",
       "joint5_to_joint4", "joint6_to_joint5", "joint6output_to_joint6"]
ap = argparse.ArgumentParser()
ap.add_argument("--x", type=float); ap.add_argument("--y", type=float)
ap.add_argument("--z", type=float, required=True)
ap.add_argument("--yaw", type=float, default=180.0,
                help="Gierwinkel RELATIV zum Azimut des Punktes, wie in zeige_punkt.py (180 = Standardgriff)")
ap.add_argument("--hier", action="store_true", help="x/y aus der aktuellen TCP-Lage")
ap.add_argument("--gain", type=float, default=0.6); ap.add_argument("--runden", type=int, default=4)
ap.add_argument("--toleranz", type=float, default=0.3); ap.add_argument("--setzzeit", type=float, default=4.0)
ap.add_argument("--max-korrektur", type=float, default=4.0)
ap.add_argument("--nick-vorhalt", type=float, default=0.0,
                help="[Grad] fester Vorhalt auf Achse 4 gegen das Spiel, das der Encoder NICHT sieht "
                     "(2026-09-11 nest_54: Encoder-Nick -0.19 Grad, Greifer sichtbar schief). Vorzeichen "
                     "am Aufbau pruefen: hilft es nicht, umdrehen.")
a = ap.parse_args()

rclpy.init(); node = Node("senkrecht_stellen"); js = {}
node.create_subscription(JointState, "/joint_states", lambda m: js.update(dict(zip(m.name, m.position))), 10)
buf = tf2_ros.Buffer(); tf2_ros.TransformListener(buf, node)
arm = ActionClient(node, FollowJointTrajectory, "/arm_controller/follow_joint_trajectory")
ikc = node.create_client(GetPositionIK, "/compute_ik")

def spin(s):
    t = time.time()
    while time.time() - t < s: rclpy.spin_once(node, timeout_sec=0.1)

def ist():
    js.clear()
    for _ in range(30):
        rclpy.spin_once(node, timeout_sec=0.1)
        if all(j in js for j in ARM): return [js[j] for j in ARM]
    sys.exit("!! keine /joint_states")

def tcp():
    for _ in range(40):
        rclpy.spin_once(node, timeout_sec=0.1)
        try:
            t = buf.lookup_transform("robot_base", "tcp", rclpy.time.Time()).transform.translation
            return t.x * 1000, t.y * 1000, t.z * 1000
        except Exception: pass
    sys.exit("!! keine TF")

def fahre(rad, dauer):
    g = FollowJointTrajectory.Goal(); g.trajectory.joint_names = ARM
    p = JointTrajectoryPoint(); p.positions = [float(v) for v in rad]
    p.time_from_start = Duration(sec=int(dauer), nanosec=int((dauer % 1) * 1e9)); g.trajectory.points = [p]
    f = arm.send_goal_async(g); rclpy.spin_until_future_complete(node, f, timeout_sec=15)
    gh = f.result()
    if gh is None or not gh.accepted: sys.exit("!! Fahrt abgelehnt")
    r = gh.get_result_async(); rclpy.spin_until_future_complete(node, r, timeout_sec=dauer + 20)

seed = ist()
x, y = (tcp()[:2] if a.hier else (a.x, a.y))
if x is None or y is None: sys.exit("--x/--y oder --hier")
Q = (-0.583486105248945, 0.3994295494594975, -0.3994295494594975, 0.583486105248945)
def qmul(p, q):
    ax, ay, az, aw = p; bx, by, bz, bw = q
    return (aw*bx+ax*bw+ay*bz-az*by, aw*by-ax*bz+ay*bw+az*bx, aw*bz+ax*by-ay*bx+az*bw, aw*bw-ax*bx-ay*by-az*bz)
azimut = math.degrees(math.atan2(y, x))
h = math.radians(azimut + a.yaw) / 2; q = qmul((0, 0, math.sin(h), math.cos(h)), Q)   # wie zeige_punkt.py
ikc.wait_for_service(15)
rq = GetPositionIK.Request(); r = rq.ik_request
r.group_name = "arm"; r.ik_link_name = "tcp"; r.avoid_collisions = True; r.timeout = Duration(sec=2)
r.robot_state.joint_state.name = list(ARM); r.robot_state.joint_state.position = seed   # Seed = Ist -> naechste Loesung
ps = PoseStamped(); ps.header.frame_id = "robot_base"
ps.pose.position.x, ps.pose.position.y, ps.pose.position.z = x / 1000, y / 1000, a.z / 1000
ps.pose.orientation = Quaternion(x=q[0], y=q[1], z=q[2], w=q[3]); r.pose_stamped = ps
# TRAC-IK liefert je Aufruf eine andere Loesung; mehrfach loesen, die zur Ist-Stellung naechste nehmen
ziel, best = None, 1e9
for _ in range(12):
    f = ikc.call_async(rq); rclpy.spin_until_future_complete(node, f, timeout_sec=10); res = f.result()
    if res is None or res.error_code.val != 1: continue
    nm = list(res.solution.joint_state.name); vl = list(res.solution.joint_state.position)
    kand = [vl[nm.index(j)] for j in ARM]
    d = max(abs(z_ - s_) for z_, s_ in zip(kand, seed))
    if d < best: ziel, best = kand, d
if ziel is None: sys.exit("!! IK fehlgeschlagen")
print(f"  IK: naechste Loesung {math.degrees(best):.1f} Grad von der Ist-Stellung")
if best > math.radians(25):
    sys.exit("!! IK-Loesung liegt > 25 Grad von der Ist-Stellung weg (andere Armkonfiguration) - nicht gefahren")
fmt = lambda v: " ".join(f"{math.degrees(v_):+7.2f}" for v_ in v)
if a.nick_vorhalt:
    ziel[3] += math.radians(a.nick_vorhalt)
    print(f"  Nick-Vorhalt {a.nick_vorhalt:+.1f} Grad auf Achse 4 (Modell steht dann absichtlich schief)")
print(f"  Ziel ({x:+.1f}, {y:+.1f}, {a.z:.1f}) yaw {a.yaw:.0f}  ->  Gelenke {fmt(ziel)}")
arm.wait_for_server(10); fahre(ziel, 4); spin(a.setzzeit)
kommando = list(ziel); tol = math.radians(a.toleranz)
fehler = [i - z_ for i, z_ in zip(ist(), ziel)]
print(f"  Fehler {fmt(fehler)}   Nick {math.degrees(fehler[1]+fehler[2]+fehler[3]):+.2f}")
for runde in range(1, a.runden + 1):
    if all(abs(e) <= tol for e in fehler): print(f"  fertig nach {runde-1} Korrektur(en)"); break
    kommando = [k - a.gain * e for k, e in zip(kommando, fehler)]
    vorhalt = [k - z_ for k, z_ in zip(kommando, ziel)]
    if any(abs(v) > math.radians(a.max_korrektur) for v in vorhalt): print("  !! Vorhalt zu gross - Abbruch"); break
    print(f"  Runde {runde}: Vorhalt {fmt(vorhalt)}")
    fahre(kommando, 3); spin(a.setzzeit)
    fehler = [i - z_ for i, z_ in zip(ist(), ziel)]
    print(f"  Fehler {fmt(fehler)}   Nick {math.degrees(fehler[1]+fehler[2]+fehler[3]):+.2f}")
t = tcp(); print(f"  TCP (Encoder): x {t[0]:+.1f}  y {t[1]:+.1f}  z {t[2]:+.1f} mm")
node.destroy_node(); rclpy.shutdown()
