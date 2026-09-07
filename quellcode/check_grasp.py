#!/usr/bin/env python3
"""
check_grasp.py — diagnostiziere ein fehlgeschlagenes Abstiegsziel.
Führt auf Hover- / Oberflächen- / Grasp-Höhen eine IK + Kollisionsprüfung durch;
listet auf, in welcher Höhe der Abstieg an welchem Link-Paar hängen bleibt.

Verwendung: source /opt/ros/galactic/setup.bash && source install/setup.bash
            python3 check_grasp.py
"""
import sys
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import PoseStamped
from moveit_msgs.srv import GetPositionIK, GetStateValidity

FRAME = "robot_base"
GROUP = "arm"
EEF = "tcp"

# Letztes fehlgeschlagenes Latch: x=179 y=-32 z=3mm, offset -0.025 → grasp z=-0.022
X, Y = 0.179, -0.032
QUAT = (-0.735, -0.031, 0.029, 0.677)
HEIGHTS = [
    ("hover    (+78mm)", 0.078),
    ("Oberflaeche (0mm)", 0.000),
    ("grasp    (-22mm)", -0.022),
]


def pose_at(z):
    ps = PoseStamped()
    ps.header.frame_id = FRAME
    ps.pose.position.x = X
    ps.pose.position.y = Y
    ps.pose.position.z = z
    ps.pose.orientation.x, ps.pose.orientation.y, ps.pose.orientation.z, ps.pose.orientation.w = QUAT
    return ps


def main():
    rclpy.init()
    node = Node("check_grasp")
    ik = node.create_client(GetPositionIK, "/compute_ik")
    sv = node.create_client(GetStateValidity, "/check_state_validity")
    for c, n in ((ik, "/compute_ik"), (sv, "/check_state_validity")):
        if not c.wait_for_service(timeout_sec=10.0):
            print(f"!! {n} nicht vorhanden — laeuft move_group?"); rclpy.shutdown(); return

    def solve_ik(z, avoid):
        req = GetPositionIK.Request()
        req.ik_request.group_name = GROUP
        req.ik_request.ik_link_name = EEF
        req.ik_request.pose_stamped = pose_at(z)
        req.ik_request.avoid_collisions = avoid
        req.ik_request.timeout.sec = 2
        f = ik.call_async(req)
        rclpy.spin_until_future_complete(node, f, timeout_sec=8.0)
        return f.result()

    def validity(robot_state):
        req = GetStateValidity.Request()
        req.robot_state = robot_state
        req.group_name = GROUP
        f = sv.call_async(req)
        rclpy.spin_until_future_complete(node, f, timeout_sec=8.0)
        return f.result()

    print(f"\n=== Ziel x={X} y={Y} (r={ (X*X+Y*Y)**0.5*1000:.0f}mm), quat={QUAT} ===")
    for name, z in HEIGHTS:
        # 1) Gibt es eine kollisionsVERMEIDENDE IK? (die Trajektorie verlangt das)
        r_avoid = solve_ik(z, True)
        ok_avoid = r_avoid and r_avoid.error_code.val == 1
        # 2) Löse eine IK, die die Kollision IGNORIERT, dann schaue WELCHES Paar kollidiert
        contacts = []
        valid = None
        r_free = solve_ik(z, False)
        joints = None
        if r_free and r_free.error_code.val == 1:
            v = validity(r_free.solution)
            if v is not None:
                valid = v.valid
                contacts = [f"{c.contact_body_1} × {c.contact_body_2}" for c in v.contacts]
            js = r_free.solution.joint_state
            joints = {n: p for n, p in zip(js.name, js.position)}
        tag = "✅ ERREICHBAR (kollisionsfreie IK vorhanden)" if ok_avoid else "🔴 KEINE kollisionsfreie IK"
        print(f"\n[{name}]  {tag}")
        if joints:
            order = ["joint2_to_joint1", "joint3_to_joint2", "joint4_to_joint3",
                     "joint5_to_joint4", "joint6_to_joint5", "joint6output_to_joint6"]
            vals = [joints.get(k, joints.get(k.replace("_to_", "_"), 0.0)) for k in order]
            # auch mit rohen Namen versuchen
            if all(v == 0.0 for v in vals):
                vals = list(joints.values())[:6]
            print("    Gelenke(rad): " + " ".join(f"{x:+.2f}" for x in vals))
        if valid is not None:
            print(f"    geometrischer IK-Status: {'GUELTIG' if valid else 'KOLLISION'}")
            if contacts:
                for cpair in sorted(set(contacts)):
                    print(f"      ↳ {cpair}")
        elif not r_free or r_free.error_code.val != 1:
            print("    (in dieser Hoehe konnte gar keine IK geloest werden — ausser Reichweite/Limit)")

    node.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
