#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""KAMERAPOSE: Nullstellung, aber Gelenk 5 um 90 Grad gedreht, so dass der Greifer WAAGERECHT nach -y zeigt
und NICHT unter der Kamera haengt (Benutzer 2026-09-10/14: in der Nullstellung ragen die Finger ins Bild,
YOLO hielt sie fuer eine Welle; der Arm verschattet den IR-Projektor).

Fahrt NUR ueber MoveIt (kollisionsgeprueft, Gelenkziel). Greifer unveraendert (--greifer-zu: vorher zu).

    python3 tools/kamerapose.py --nur-pruefen        # nichts fahren: FK beider Vorzeichen, Kollision, Fingerrichtung
    python3 tools/kamerapose.py --plan-only          # Bahn planen, in RViz zeigen, nicht fahren
    python3 tools/kamerapose.py                      # MANUELL: Rueckfrage, dann fahren
    python3 tools/kamerapose.py --auto               # ohne Rueckfrage (GUI / ablauf.py)
    python3 tools/kamerapose.py --speichern          # Gelenkziel als 'kamerapose' in teach_punkte.json ablegen

Das Vorzeichen von J5 wird aus der FK bestimmt (--j5-grad erzwingt es): genommen wird das, bei dem die
tcp-y-Achse (= Blickrichtung des Greifers, beim senkrechten Griff (0,0,-1)) am staerksten nach -y zeigt.
Sim 14.9.: J5 -90 -> TCP (+7, -245, 430) mm, Blick (0, -1, 0); J5 +90 zeigt nach +y.
Exit 0 = angekommen / geprueft, 1 = Fehler, 3 = Abbruch.
"""
import os, argparse, json, math, os, sys, time
import rclpy
from rclpy.action import ActionClient
from rclpy.node import Node
from geometry_msgs.msg import PoseStamped
from moveit_msgs.action import MoveGroup
from moveit_msgs.msg import Constraints, JointConstraint, RobotState
from moveit_msgs.srv import GetPositionFK, GetStateValidity
from sensor_msgs.msg import JointState

ARM = ['joint2_to_joint1', 'joint3_to_joint2', 'joint4_to_joint3', 'joint5_to_joint4', 'joint6_to_joint5', 'joint6output_to_joint6']
ap = argparse.ArgumentParser()
ap.add_argument("--j5-grad", type=float, default=None, help="J5 erzwingen (+90 / -90); sonst aus der FK (Finger nach -y)")
ap.add_argument("--datei", default=os.path.expanduser("~/ros2_ws/teach_punkte.json"))
ap.add_argument("--name", default="kamerapose")
ap.add_argument("--nur-pruefen", action="store_true"); ap.add_argument("--plan-only", action="store_true")
ap.add_argument("--speichern", action="store_true", help="Gelenkziel in teach_punkte.json ablegen (auch ohne Fahrt)")
ap.add_argument("--auto", action="store_true"); ap.add_argument("--tempo", type=float, default=2.0)
ap.add_argument("--greifer-zu", action="store_true", help="vorher Greifer schliessen (-0.74)")
a = ap.parse_args()

rclpy.init(); n = Node("kamerapose"); js = {"m": None}
n.create_subscription(JointState, "/joint_states", lambda m: js.__setitem__("m", m), 10)
fkc = n.create_client(GetPositionFK, "/compute_fk"); valc = n.create_client(GetStateValidity, "/check_state_validity")
mg = ActionClient(n, MoveGroup, "/move_action")


def ende(code):
    n.destroy_node(); rclpy.shutdown(); sys.exit(code)


def ist_rad():
    t0 = time.time()
    while js["m"] is None and time.time() - t0 < 10: rclpy.spin_once(n, timeout_sec=0.1)
    if js["m"] is None: print("!! keine /joint_states"); ende(1)
    m = js["m"]; return [m.position[m.name.index(j)] for j in ARM]


def greifer_wert():
    m = js["m"]; return m.position[m.name.index("gripper_controller")] if m and "gripper_controller" in m.name else -0.74


def fk(rad):
    if not fkc.wait_for_service(10): print("!! /compute_fk fehlt"); ende(1)
    rq = GetPositionFK.Request(); rq.header.frame_id = "robot_base"; rq.fk_link_names = ["tcp"]
    rq.robot_state.joint_state.name = list(ARM); rq.robot_state.joint_state.position = [float(v) for v in rad]
    f = fkc.call_async(rq); rclpy.spin_until_future_complete(n, f, timeout_sec=10); r = f.result()
    if r is None or r.error_code.val != 1: print("!! FK fehlgeschlagen"); ende(1)
    return r.pose_stamped[0].pose


def achsen(q):
    """Spalten der Rotationsmatrix: x-, y-, z-Achse des tcp in robot_base."""
    x, y, z, w = q.x, q.y, q.z, q.w
    return ((1 - 2 * (y * y + z * z), 2 * (x * y + z * w), 2 * (x * z - y * w)),
            (2 * (x * y - z * w), 1 - 2 * (x * x + z * z), 2 * (y * z + x * w)),
            (2 * (x * z + y * w), 2 * (y * z - x * w), 1 - 2 * (x * x + y * y)))


def gueltig(rad, greifer):
    if not valc.wait_for_service(10): print("!! /check_state_validity fehlt"); ende(1)
    rq = GetStateValidity.Request(); rq.group_name = "arm"
    rq.robot_state.joint_state.name = list(ARM) + ["gripper_controller"]
    rq.robot_state.joint_state.position = [float(v) for v in rad] + [float(greifer)]
    f = valc.call_async(rq); rclpy.spin_until_future_complete(n, f, timeout_sec=10); r = f.result()
    if r is None: return False, ["keine Antwort"]
    return r.valid, sorted({f"{c.contact_body_1}x{c.contact_body_2}" for c in r.contacts})


ist_rad(); g = greifer_wert()
kand = []
sys.path.insert(0, os.path.dirname(os.path.abspath(__file__))); from gelenkspiel import nullstellung_rad
for j5 in ((a.j5_grad,) if a.j5_grad is not None else (-90.0, 90.0)):
    rad = list(nullstellung_rad()); rad[4] = math.radians(j5)          # reale Nullstellung (20.9.) + J5
    p = fk(rad); ax = achsen(p.orientation); bl = ax[1]           # tcp y = Blickrichtung des Greifers
    ok, kont = gueltig(rad, g)
    print(f"  J5 {j5:+.0f}: TCP ({p.position.x*1e3:+.0f}, {p.position.y*1e3:+.0f}, {p.position.z*1e3:.0f}) mm, "
          f"Greifer zeigt nach ({bl[0]:+.2f}, {bl[1]:+.2f}, {bl[2]:+.2f})  " + ("kollisionsfrei" if ok else "KOLLISION " + " ".join(kont)))
    kand.append((bl[1], ok, j5, rad, p))
kand.sort(key=lambda k: (not k[1], k[0]))                       # gueltig zuerst, dann staerkstes -y
fy, ok, j5, rad, p = kand[0]
if not ok: print("!! keine kollisionsfreie Kamerapose"); ende(1)
if fy > -0.5: print(f"!! Greifer zeigt nicht nach -y (y-Anteil {fy:+.2f}) - --j5-grad pruefen"); ende(1)
grad = [math.degrees(v) for v in rad]
print("  KAMERAPOSE: " + " ".join(f"{v:+.0f}" for v in grad) + f"  (J5 {j5:+.0f}, Greifer nach -y, TCP z {p.position.z*1e3:.0f} mm)")

if a.speichern:
    d = json.load(open(a.datei)) if os.path.exists(a.datei) else {}
    d[a.name] = {"grad": [round(v, 3) for v in grad], "rad": [round(v, 4) for v in rad],
                 "tcp_modell_mm": [round(p.position.x * 1e3, 1), round(p.position.y * 1e3, 1), round(p.position.z * 1e3, 1)],
                 "quelle": "kamerapose.py (Nullstellung + J5, Finger nach -y)", "zeit": time.strftime("%Y-%m-%d %H:%M:%S")}
    json.dump(d, open(a.datei, "w"), indent=1); print(f"  -> '{a.name}' in {a.datei}")
if a.nur_pruefen: ende(0)

if a.greifer_zu:
    from control_msgs.action import FollowJointTrajectory
    from trajectory_msgs.msg import JointTrajectoryPoint
    grp = ActionClient(n, FollowJointTrajectory, "/gripper_controller/follow_joint_trajectory")
    if grp.wait_for_server(10):
        goal = FollowJointTrajectory.Goal(); goal.trajectory.joint_names = ["gripper_controller"]
        pt = JointTrajectoryPoint(); pt.positions = [-0.74]; pt.time_from_start.sec = 3; goal.trajectory.points = [pt]
        f = grp.send_goal_async(goal); rclpy.spin_until_future_complete(n, f, timeout_sec=10)
        gh = f.result(); r = gh.get_result_async() if gh and gh.accepted else None
        if r is not None: rclpy.spin_until_future_complete(n, r, timeout_sec=15)
        print("  Greifer zu.")

if not a.auto and not a.plan_only:
    if input("  Kamerapose anfahren (MoveIt, kollisionsgeprueft)? [j/N] ").strip().lower() not in ("j", "ja", "y", "yes"):
        print("  Abbruch."); ende(3)
if not mg.wait_for_server(20): print("!! /move_action fehlt"); ende(1)
goal = MoveGroup.Goal(); req = goal.request
req.group_name = "arm"; req.num_planning_attempts = 10; req.allowed_planning_time = 5.0
req.max_velocity_scaling_factor = min(1.0, 0.15 * a.tempo); req.max_acceleration_scaling_factor = 0.1
req.workspace_parameters.header.frame_id = "robot_base"
for k, sgn in (("min_corner", -1.0), ("max_corner", 1.0)):
    c = getattr(req.workspace_parameters, k); c.x = c.y = c.z = sgn
cs = Constraints()
for jn, jv in zip(ARM, rad):
    jc = JointConstraint(); jc.joint_name = jn; jc.position = float(jv); jc.tolerance_above = jc.tolerance_below = 0.02; jc.weight = 1.0
    cs.joint_constraints.append(jc)
req.goal_constraints.append(cs); goal.planning_options.plan_only = bool(a.plan_only)
print("  -> MoveIt " + ("(nur planen, RViz)" if a.plan_only else "(fahren, kollisionsgeprueft)"))
f = mg.send_goal_async(goal); rclpy.spin_until_future_complete(n, f, timeout_sec=25); gh = f.result()
if gh is None or not gh.accepted: print("!! MoveIt-Ziel abgelehnt"); ende(1)
r = gh.get_result_async(); rclpy.spin_until_future_complete(n, r, timeout_sec=120)
ok = r.result() is not None and r.result().result.error_code.val == 1
print(("  Bahn geplant (RViz)." if a.plan_only else "  Kamerapose erreicht.") if ok
      else f"!! fehlgeschlagen (code {r.result().result.error_code.val if r.result() else '?'})")
ende(0 if ok else 1)
