#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""Angelernten Punkt OHNE Fahrt gegen das Kollisionsmodell pruefen (Vorschau in RViz).

Fuer jeden Punkt:
  1. FK: Modell-TCP der gespeicherten Gelenke
  2. /check_state_validity: steht der Arm dort kollisionsfrei (Greifer offen UND zu)?
  3. Ueber-Punkt (+--ueber mm) per gerader Bahn aus der Stellung (wie hole/lege_welle) - Anteil planbar
  4. MoveIt plan_only von der IST-Stellung zum Ueber-Punkt -> Bahn erscheint in RViz
     (Display "Planned Path"), NICHTS wird gefahren
  5. Gelenkabstand Ist -> Punkt (J6-Differenz = Kabel!)

    python3 tools/punkt_pruefen.py trichter_greif trichter_ablager
    python3 tools/punkt_pruefen.py trichter_ablager --ueber 60 --blind-mm 30
"""
import argparse, json, math, os, sys, time
import rclpy
from rclpy.action import ActionClient
from rclpy.node import Node
from geometry_msgs.msg import Pose, PoseStamped
from moveit_msgs.action import MoveGroup
from moveit_msgs.msg import Constraints, JointConstraint, RobotState
from moveit_msgs.srv import GetCartesianPath, GetPositionFK, GetStateValidity
from sensor_msgs.msg import JointState

ARM = ['joint2_to_joint1', 'joint3_to_joint2', 'joint4_to_joint3',
       'joint5_to_joint4', 'joint6_to_joint5', 'joint6output_to_joint6']
BASE = 'robot_base'
ap = argparse.ArgumentParser()
ap.add_argument("punkte", nargs="+")
ap.add_argument("--datei", default=os.path.expanduser("~/ros2_ws/teach_punkte.json"))
ap.add_argument("--ueber", type=float, default=40.0)
ap.add_argument("--blind-mm", type=float, default=0.0, help="[mm] erstes Stueck der Bahn nach oben ohne Kollisionspruefung")
ap.add_argument("--ohne-plan", action="store_true", help="Schritt 4 (MoveIt plan_only) auslassen")
a = ap.parse_args()
punkte = json.load(open(a.datei))

rclpy.init(); node = Node("punkt_pruefen"); js = {"m": None}
node.create_subscription(JointState, "/joint_states", lambda m: js.__setitem__("m", m), 10)
fkc = node.create_client(GetPositionFK, "/compute_fk")
val = node.create_client(GetStateValidity, "/check_state_validity")
cart = node.create_client(GetCartesianPath, "/compute_cartesian_path")
mg = ActionClient(node, MoveGroup, "/move_action")
for c in (fkc, val, cart):
    if not c.wait_for_service(20): sys.exit(f"!! {c.srv_name} fehlt")


def call(c, rq, t=60):
    f = c.call_async(rq); rclpy.spin_until_future_complete(node, f, timeout_sec=t); return f.result()


def ist():
    js["m"] = None
    for _ in range(100):
        rclpy.spin_once(node, timeout_sec=0.1)
        if js["m"] is not None:
            d = dict(zip(js["m"].name, js["m"].position))
            if all(j in d for j in ARM): return d
    sys.exit("!! keine /joint_states")


def rs_von(rad, greifer=None):
    rs = RobotState(); rs.joint_state.name = list(ARM); rs.joint_state.position = [float(v) for v in rad]
    if greifer is not None: rs.joint_state.name.append("gripper_controller"); rs.joint_state.position.append(float(greifer))
    return rs


def fk(rad):
    rq = GetPositionFK.Request(); rq.header.frame_id = BASE; rq.fk_link_names = ["tcp"]; rq.robot_state = rs_von(rad)
    return call(fkc, rq, 10).pose_stamped[0].pose


def gueltig(rad, greifer):
    rq = GetStateValidity.Request(); rq.robot_state = rs_von(rad, greifer); rq.group_name = "arm"
    r = call(val, rq, 20)
    kont = sorted({f"{c.contact_body_1}x{c.contact_body_2}" for c in r.contacts})
    return r.valid, kont


def bahn(start_rad, ziel, avoid=True):
    cq = GetCartesianPath.Request(); cq.header.frame_id = BASE; cq.group_name = "arm"; cq.link_name = "tcp"
    cq.start_state = rs_von(start_rad); cq.waypoints = [ziel]; cq.max_step = 0.005; cq.jump_threshold = 0.0; cq.avoid_collisions = avoid
    r = call(cart, cq, 120)
    if r is None: return None, None
    t = r.solution.joint_trajectory
    return r.fraction, ([t.points[-1].positions[t.joint_names.index(j)] for j in ARM] if t.points else None)


def dz(p, mm):
    q = Pose(); q.orientation = p.orientation; q.position.x = p.position.x; q.position.y = p.position.y; q.position.z = p.position.z + mm / 1000
    return q


d_ist = ist(); ist_rad = [d_ist[j] for j in ARM]; greifer_ist = d_ist.get("gripper_controller")
p_ist = fk(ist_rad)
print(f"IST: " + " ".join(f"{math.degrees(v):+7.2f}" for v in ist_rad) + f"  Greifer {greifer_ist:+.2f}"
      f"  TCP ({p_ist.position.x*1000:+.1f}, {p_ist.position.y*1000:+.1f}, {p_ist.position.z*1000:.1f}) mm")
ok_ist, k = gueltig(ist_rad, greifer_ist)
print(f"     Ist-Stellung kollisionsfrei: {'JA' if ok_ist else 'NEIN ' + ' '.join(k)}")

for name in a.punkte:
    if name not in punkte: print(f"\n!! '{name}' nicht in der Datei"); continue
    rad = [float(v) for v in punkte[name]["rad"]]
    p = fk(rad)
    print(f"\n=== {name}: " + " ".join(f"{math.degrees(v):+7.2f}" for v in rad)
          + f"  [{punkte[name].get('notiz', punkte[name].get('quelle', ''))[:70]}]")
    print(f"  TCP-Modell ({p.position.x*1000:+.1f}, {p.position.y*1000:+.1f}, {p.position.z*1000:.1f}) mm"
          f"   r {math.hypot(p.position.x, p.position.y)*1000:.1f}")
    diff = [math.degrees(r_ - i_) for r_, i_ in zip(rad, ist_rad)]
    print(f"  Gelenkweg ab Ist: " + " ".join(f"{v:+6.1f}" for v in diff) + f"   J6-Differenz {diff[5]:+.1f} Grad")
    for g, was in ((0.15, "offen"), (-0.74, "ganz zu")):
        ok, k = gueltig(rad, g)
        print(f"  Stellung mit Greifer {was:8s}: " + ("kollisionsfrei" if ok else "KOLLISION " + " ".join(k)))
    if a.blind_mm > 0:
        fr, zw = bahn(rad, dz(p, a.blind_mm), avoid=False)
        print(f"  Bahn +{a.blind_mm:.0f} mm OHNE Pruefung: {fr*100 if fr is not None else 0:.0f} %")
        fr2, ueber = bahn(zw, dz(p, a.ueber)) if zw else (None, None)
        print(f"  Bahn weiter auf +{a.ueber:.0f} mm geprueft: {fr2*100 if fr2 is not None else 0:.0f} %")
    else:
        fr2, ueber = bahn(rad, dz(p, a.ueber))
        print(f"  gerade Bahn Stellung -> +{a.ueber:.0f} mm (geprueft): {fr2*100 if fr2 is not None else 0:.0f} %")
    if ueber is not None and max(abs(u - r_) for u, r_ in zip(ueber, rad)) < 1e-6:
        print("  (Bahn liefert keinen Ueber-Punkt - 0 % - kein MoveIt-Plan)"); ueber = None
    if ueber is not None:
        ok, k = gueltig(ueber, 0.15)
        print(f"  Ueber-Punkt (+{a.ueber:.0f}): " + " ".join(f"{math.degrees(v):+7.2f}" for v in ueber)
              + "  " + ("kollisionsfrei" if ok else "KOLLISION " + " ".join(k)))
        if not a.ohne_plan and mg.wait_for_server(10):
            goal = MoveGroup.Goal(); req = goal.request
            req.group_name = "arm"; req.num_planning_attempts = 10; req.allowed_planning_time = 5.0
            req.max_velocity_scaling_factor = 0.3; req.max_acceleration_scaling_factor = 0.1
            req.workspace_parameters.header.frame_id = BASE
            for key, sgn in (("min_corner", -1.0), ("max_corner", 1.0)):
                c = getattr(req.workspace_parameters, key); c.x = c.y = c.z = sgn
            cs = Constraints()
            for jn, jv in zip(ARM, ueber):
                jc = JointConstraint(); jc.joint_name = jn; jc.position = float(jv); jc.tolerance_above = jc.tolerance_below = 0.02; jc.weight = 1.0
                cs.joint_constraints.append(jc)
            req.goal_constraints.append(cs); goal.planning_options.plan_only = True
            f = mg.send_goal_async(goal); rclpy.spin_until_future_complete(node, f, timeout_sec=20); gh = f.result()
            r = None
            if gh and gh.accepted:
                rf = gh.get_result_async(); rclpy.spin_until_future_complete(node, rf, timeout_sec=60); r = rf.result()
            if r and r.result.error_code.val == 1:
                tr = r.result.planned_trajectory.joint_trajectory
                print(f"  MoveIt plan_only Ist -> Ueber-Punkt: OK, {len(tr.points)} Punkte, {tr.points[-1].time_from_start.sec} s (in RViz 'Planned Path' ansehen). NICHT gefahren.")
            else:
                print(f"  MoveIt plan_only Ist -> Ueber-Punkt: FEHLGESCHLAGEN (code {r.result.error_code.val if r else '?'})")
node.destroy_node(); rclpy.shutdown()
