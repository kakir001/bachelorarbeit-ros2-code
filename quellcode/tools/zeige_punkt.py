#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""Faehrt den TCP senkrecht ueber einen Punkt und LAESST IHN DORT STEHEN.

Wozu: bevor Trichter oder Ablageplatte festgeklebt werden, soll der Roboter
selbst zeigen, wo sie hinkommen. Der Arm zeigt die Stelle, der Benutzer schaut
nach, ob dort ueberhaupt Platz ist. Geraten wird nichts.

    python3 tools/zeige_punkt.py --azimut 270 --r 160 --z 115
    python3 tools/zeige_punkt.py --x 0 --y -160 --z 115      (gleichwertig)

Der Punkt ist die Stelle der FINGERSPITZE (tcp), nicht die des Trichterfusses.
Beim Trichter ist das die Stelle, an der spaeter die WELLE steht.

Sicherheit:
  * geplant wird mit MoveIt (Kollisionen beruecksichtigt), nicht blind gefahren
  * langsam: velocity 0.15, acceleration 0.10
  * ohne --sofort wird der Plan erst gezeigt und muss bestaetigt werden
"""
import argparse
import math
import sys
import time

import rclpy
from rclpy.action import ActionClient
from rclpy.node import Node
from geometry_msgs.msg import PoseStamped, Quaternion
from moveit_msgs.action import MoveGroup
from moveit_msgs.msg import Constraints, JointConstraint
from moveit_msgs.srv import GetPositionIK
from builtin_interfaces.msg import Duration

ARM_JOINTS = ['joint2_to_joint1', 'joint3_to_joint2', 'joint4_to_joint3',
              'joint5_to_joint4', 'joint6_to_joint5', 'joint6output_to_joint6']
BASE = 'robot_base'
# Senkrechter Griff: tcp +Y zeigt nach (0,0,-1) in robot_base. Aus
# tools/arbeitsraum_grenze.py uebernommen, damit beide dasselbe meinen.
Q_REF = (-0.583486105248945, 0.3994295494594975, -0.3994295494594975, 0.583486105248945)


def qmul(a, b):
    ax, ay, az, aw = a
    bx, by, bz, bw = b
    return (aw*bx + ax*bw + ay*bz - az*by,
            aw*by - ax*bz + ay*bw + az*bx,
            aw*bz + ax*by - ay*bx + az*bw,
            aw*bw - ax*bx - ay*by - az*bz)


def qz(deg):
    h = math.radians(deg) / 2.0
    return (0.0, 0.0, math.sin(h), math.cos(h))


ap = argparse.ArgumentParser()
ap.add_argument("--azimut", type=float, help="Grad, 0 = +x")
ap.add_argument("--r", type=float, help="Radius [mm]")
ap.add_argument("--x", type=float, help="[mm] statt azimut/r")
ap.add_argument("--y", type=float, help="[mm]")
ap.add_argument("--z", type=float, required=True, help="Hoehe ueber robot_base [mm]")
ap.add_argument("--yaw", type=float, default=None,
                help="Gierwinkel festnageln [Grad], statt 0/90/180/270 durchzuprobieren")
ap.add_argument("--sofort", action="store_true", help="ohne Rueckfrage ausfuehren")
ap.add_argument("--vel", type=float, default=0.15)
ap.add_argument("--acc", type=float, default=0.10)
a = ap.parse_args()

if a.x is not None and a.y is not None:
    x_mm, y_mm = a.x, a.y
    azimut = math.degrees(math.atan2(y_mm, x_mm)) % 360.0
    r_mm = math.hypot(x_mm, y_mm)
elif a.azimut is not None and a.r is not None:
    azimut, r_mm = a.azimut, a.r
    x_mm = r_mm * math.cos(math.radians(azimut))
    y_mm = r_mm * math.sin(math.radians(azimut))
else:
    sys.exit("entweder --azimut und --r, oder --x und --y angeben")

print(f"\n  Ziel:  x {x_mm:+.1f}  y {y_mm:+.1f}  z {a.z:+.1f} mm")
print(f"         (Azimut {azimut:.1f} Grad, r {r_mm:.1f} mm)")
if 150.0 <= azimut <= 176.0:
    print("  !! ACHTUNG: liegt im TOTEN SEKTOR (joint1-Grenze) — wird kaum gehen")

rclpy.init()
node = Node("zeige_punkt")

ik = node.create_client(GetPositionIK, "/compute_ik")
if not ik.wait_for_service(timeout_sec=20.0):
    sys.exit("!! /compute_ik nicht da — laeuft der Stack?")

gier = (a.yaw,) if a.yaw is not None else (0.0, 90.0, 180.0, 270.0)
loesung = None
for dy in gier:
    q = qmul(qz(azimut + dy), Q_REF)
    rq = GetPositionIK.Request()
    rq.ik_request.group_name = "arm"
    rq.ik_request.ik_link_name = "tcp"
    rq.ik_request.avoid_collisions = True
    rq.ik_request.timeout = Duration(sec=2)
    ps = PoseStamped()
    ps.header.frame_id = BASE
    ps.pose.position.x = x_mm / 1000.0
    ps.pose.position.y = y_mm / 1000.0
    ps.pose.position.z = a.z / 1000.0
    ps.pose.orientation = Quaternion(x=q[0], y=q[1], z=q[2], w=q[3])
    rq.ik_request.pose_stamped = ps
    fut = ik.call_async(rq)
    rclpy.spin_until_future_complete(node, fut, timeout_sec=10.0)
    res = fut.result()
    if res and res.error_code.val == 1:
        namen = list(res.solution.joint_state.name)
        werte = list(res.solution.joint_state.position)
        loesung = [werte[namen.index(j)] for j in ARM_JOINTS]
        print(f"  IK ok (Gierwinkel +{dy:.0f} Grad)")
        break

if loesung is None:
    sys.exit("!! keine kollisionsfreie IK-Loesung — Punkt nicht erreichbar")

print("  Gelenke [Grad]: " + ", ".join(f"{math.degrees(v):+7.2f}" for v in loesung))

if not a.sofort:
    if input("\n  Dorthin fahren? [j/N]: ").strip().lower() != "j":
        sys.exit("abgebrochen — der Arm hat sich nicht bewegt")

mg = ActionClient(node, MoveGroup, "/move_action")
if not mg.wait_for_server(timeout_sec=20.0):
    sys.exit("!! /move_action nicht da")

goal = MoveGroup.Goal()
req = goal.request
req.group_name = "arm"
req.num_planning_attempts = 10
req.allowed_planning_time = 5.0
req.max_velocity_scaling_factor = a.vel
req.max_acceleration_scaling_factor = a.acc
req.workspace_parameters.header.frame_id = BASE
req.workspace_parameters.min_corner.x = -1.0
req.workspace_parameters.min_corner.y = -1.0
req.workspace_parameters.min_corner.z = -1.0
req.workspace_parameters.max_corner.x = 1.0
req.workspace_parameters.max_corner.y = 1.0
req.workspace_parameters.max_corner.z = 1.0

c = Constraints()
for jn, jv in zip(ARM_JOINTS, loesung):
    jc = JointConstraint()
    jc.joint_name = jn
    jc.position = float(jv)
    jc.tolerance_above = 0.02
    jc.tolerance_below = 0.02
    jc.weight = 1.0
    c.joint_constraints.append(jc)
req.goal_constraints.append(c)

goal.planning_options.plan_only = False
goal.planning_options.look_around = False
goal.planning_options.replan = False

print("  MoveIt plant und faehrt (langsam) ...")
fut = mg.send_goal_async(goal)
rclpy.spin_until_future_complete(node, fut, timeout_sec=20.0)
gh = fut.result()
if gh is None or not gh.accepted:
    sys.exit("!! MoveGroup hat das Ziel abgelehnt")

rf = gh.get_result_async()
rclpy.spin_until_future_complete(node, rf, timeout_sec=90.0)
r = rf.result()
code = r.result.error_code.val if r else None
if code == 1:
    print("\n  ANGEKOMMEN. Der Arm steht jetzt ueber der Stelle.")
    print("  Die Fingerspitze zeigt den Punkt — dort kommt spaeter die WELLE hin,")
    print("  nicht die Mitte des Trichterfusses.")
else:
    print(f"\n  !! Fahrt nicht erfolgreich (error_code {code})")

node.destroy_node()
rclpy.shutdown()
