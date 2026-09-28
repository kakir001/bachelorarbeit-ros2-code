#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""TCP an einen Ort fahren und dabei die AKTUELLE Werkzeug-Orientierung behalten (MoveIt, kollisions-
geprueft, plan_only-Vorschau). Gedacht fuer die Trichter-Ablage nach der Kontrollpose (14.9.): die Welle
haengt am Kopf senkrecht zwischen den Fingern, Greifer waagerecht - so ueber den Trichter und loslassen.

    python3 tools/ablage_mit_orientierung.py --x 120 --y -197 --z 190            # nur planen
    python3 tools/ablage_mit_orientierung.py --x 120 --y -197 --z 190 --ausfuehren
Exit 0 ok, 1 keine IK/kein Plan.
"""
import argparse, math, sys, time
import rclpy
from rclpy.node import Node
from rclpy.action import ActionClient
from geometry_msgs.msg import Pose, PoseStamped
from moveit_msgs.action import MoveGroup, ExecuteTrajectory
from moveit_msgs.msg import Constraints, JointConstraint, RobotState
from moveit_msgs.srv import GetPositionIK, GetPositionFK
from sensor_msgs.msg import JointState
from builtin_interfaces.msg import Duration
import tf2_ros
ARM = ['joint2_to_joint1', 'joint3_to_joint2', 'joint4_to_joint3', 'joint5_to_joint4', 'joint6_to_joint5', 'joint6output_to_joint6']
ap = argparse.ArgumentParser()
ap.add_argument("--x", type=float, required=True); ap.add_argument("--y", type=float, required=True); ap.add_argument("--z", type=float, required=True)
ap.add_argument("--ausfuehren", action="store_true"); ap.add_argument("--tempo", type=float, default=1.5)
ap.add_argument("--z-min", type=float, default=100.0, help="[mm] TCP darf auf dem Weg nicht tiefer")
a = ap.parse_args()
rclpy.init(); node = Node("ablage_mit_orientierung"); js = {}
node.create_subscription(JointState, "/joint_states", lambda m: js.update(dict(zip(m.name, m.position))), 10)
puffer = tf2_ros.Buffer(); tf2_ros.TransformListener(puffer, node)
ikc = node.create_client(GetPositionIK, "/compute_ik"); fkc = node.create_client(GetPositionFK, "/compute_fk")
mg = ActionClient(node, MoveGroup, "/move_action"); ex = ActionClient(node, ExecuteTrajectory, "/execute_trajectory")
ikc.wait_for_service(10); fkc.wait_for_service(10); mg.wait_for_server(10); ex.wait_for_server(10)
t = time.time()
while time.time() - t < 5 and not (all(j in js for j in ARM) and puffer.can_transform("robot_base", "tcp", rclpy.time.Time())): rclpy.spin_once(node, timeout_sec=0.1)
start = [js[j] for j in ARM]
tf = puffer.lookup_transform("robot_base", "tcp", rclpy.time.Time()); q = tf.transform.rotation
print("Ist-TCP (%.0f, %.0f, %.0f) mm, Orientierung wird beibehalten" % (tf.transform.translation.x * 1e3, tf.transform.translation.y * 1e3, tf.transform.translation.z * 1e3))
pose = Pose(); pose.position.x, pose.position.y, pose.position.z = a.x / 1e3, a.y / 1e3, a.z / 1e3; pose.orientation = q
sol = None
for _ in range(10):
    rq = GetPositionIK.Request(); r = rq.ik_request
    r.group_name = "arm"; r.ik_link_name = "tcp"; r.avoid_collisions = True; r.timeout = Duration(sec=1)
    r.robot_state.joint_state.name = list(ARM) + ["gripper_controller"]; r.robot_state.joint_state.position = [float(v) for v in start] + [float(js.get("gripper_controller", -0.74))]
    ps = PoseStamped(); ps.header.frame_id = "robot_base"; ps.pose = pose; r.pose_stamped = ps
    f = ikc.call_async(rq); rclpy.spin_until_future_complete(node, f, timeout_sec=10); res = f.result()
    if res is not None and res.error_code.val == 1:
        nm = list(res.solution.joint_state.name); vl = list(res.solution.joint_state.position); sol = [vl[nm.index(j)] for j in ARM]
        if max(abs(s_ - v) for s_, v in zip(sol, start)) < math.radians(100): break
if sol is None: print("!! keine kollisionsfreie IK"); sys.exit(1)
print("IK: " + " ".join("%+6.1f" % math.degrees(v) for v in sol))
def fk(rad):
    rs = RobotState(); rs.joint_state.name = ARM; rs.joint_state.position = list(rad)
    rq = GetPositionFK.Request(); rq.header.frame_id = "robot_base"; rq.fk_link_names = ["tcp"]; rq.robot_state = rs
    f = fkc.call_async(rq); rclpy.spin_until_future_complete(node, f, timeout_sec=10); p = f.result().pose_stamped[0].pose.position
    return p.x * 1e3, p.y * 1e3, p.z * 1e3
goal = MoveGroup.Goal(); req = goal.request
req.group_name = "arm"; req.num_planning_attempts = 10; req.allowed_planning_time = 10.0
req.max_velocity_scaling_factor = min(1.0, 0.08 * a.tempo); req.max_acceleration_scaling_factor = 0.08
c = Constraints()
for j, v in zip(ARM, sol):
    jc = JointConstraint(); jc.joint_name = j; jc.position = float(v); jc.tolerance_above = jc.tolerance_below = 0.01; jc.weight = 1.0; c.joint_constraints.append(jc)
req.goal_constraints = [c]; goal.planning_options.plan_only = True
fu = mg.send_goal_async(goal); rclpy.spin_until_future_complete(node, fu, timeout_sec=20); gh = fu.result()
rf = gh.get_result_async(); rclpy.spin_until_future_complete(node, rf, timeout_sec=60); res = rf.result().result
if res.error_code.val != 1: print("!! kein Plan"); sys.exit(1)
tr = res.planned_trajectory; jt = tr.joint_trajectory; idx = [jt.joint_names.index(j) for j in ARM]
pts = [[p.positions[i] for i in idx] for p in jt.points]
schritt = max(abs(b - a_) for p, q_ in zip(pts, pts[1:]) for a_, b in zip(p, q_)) if len(pts) > 1 else 0
tcps = [fk(p) for p in pts[:: max(1, len(pts) // 10)]] + [fk(pts[-1])]; zmin = min(t_[2] for t_ in tcps)
print("Plan: %d Punkte, max. Gelenkschritt %.1f Grad, TCP z min %.0f mm, Ende (%.0f, %.0f, %.0f)" % (len(pts), math.degrees(schritt), zmin, *tcps[-1]))
for k_, p in enumerate(pts):
    if k_ % max(1, len(pts) // 6) == 0 or k_ == len(pts) - 1: print("  %2d: " % k_ + " ".join("%+7.1f" % math.degrees(v) for v in p))
if not a.ausfuehren: print("(nur geplant)"); sys.exit(0)
if math.degrees(schritt) > 15 or zmin < a.z_min: print("!! Vorschau verletzt Grenzen - nicht gefahren"); sys.exit(1)
eg = ExecuteTrajectory.Goal(); eg.trajectory = tr
fu = ex.send_goal_async(eg); rclpy.spin_until_future_complete(node, fu, timeout_sec=20); gh = fu.result()
rf = gh.get_result_async(); rclpy.spin_until_future_complete(node, rf, timeout_sec=240)
print("Ausfuehrung:", "ok" if rf.result().result.error_code.val == 1 else "FEHLER")
