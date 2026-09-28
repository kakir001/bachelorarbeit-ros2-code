#!/usr/bin/env python3
# =====================================================================
#  visit_boxes.py — die 5 angelernten Boxen DER REIHE NACH abfahren (Prüfrunde).
#
#  Ablauf: Roboter nach 0 → zur XY jeder Box fahren, z FEST (Standard 8cm über Plattform).
#  Die Boxen liegen an der Reichweite-GRENZE des Roboters → top-down (senkrechte) IK
#  löst nicht. Daher wird die Greifer-Ausrichtung wie in goto_clicked_point per
#  YAW + TILT-Scan gefunden (zuerst gerade nach unten, sonst schrittweise neigen).
#  Es wird mit der ersten lösbaren Ausrichtung gefahren.
#  /compute_ik (move_group offen) + /arm_controller (gleicher Weg wie cartesian_jog).
#
#  Verwendung:  python3 visit_boxes.py            # z=0.08
#               python3 visit_boxes.py --z 0.06   # andere Höhe
# =====================================================================
import argparse, json, math, os, time
import rclpy
from rclpy.node import Node
from rclpy.action import ActionClient
from control_msgs.action import FollowJointTrajectory
from trajectory_msgs.msg import JointTrajectoryPoint
from sensor_msgs.msg import JointState
from builtin_interfaces.msg import Duration
from moveit_msgs.srv import GetPositionIK
from geometry_msgs.msg import PoseStamped

ARM_JOINTS = ['joint2_to_joint1', 'joint3_to_joint2', 'joint4_to_joint3',
              'joint5_to_joint4', 'joint6_to_joint5', 'joint6output_to_joint6']
WS = os.path.expanduser("~/ros2_ws")
JSON_PATH = os.path.join(WS, ".box_teach.json")
ORDER = ["rot", "gelb", "weiss", "schwarz", "gruen"]


def quat_rpy(roll, pitch, yaw):
    """exakt wie tf2 setRPY (goto_clicked_point gripperOrientation)."""
    hr, hp, hy = roll / 2, pitch / 2, yaw / 2
    cr, sr = math.cos(hr), math.sin(hr)
    cp, sp = math.cos(hp), math.sin(hp)
    cy, sy = math.cos(hy), math.sin(hy)
    return (sr * cp * cy - cr * sp * sy,
            cr * sp * cy + sr * cp * sy,
            cr * cp * sy - sr * sp * cy,
            cr * cp * cy + sr * sp * sy)


class Visitor(Node):
    def __init__(self):
        super().__init__("visit_boxes")
        self.cli = ActionClient(self, FollowJointTrajectory,
                                "/arm_controller/follow_joint_trajectory")
        self.ik = self.create_client(GetPositionIK, "/compute_ik")
        self.cur = [0.0] * 6
        self.js_ok = False
        self.create_subscription(JointState, "/joint_states", self._js, 10)

    def _js(self, msg):
        for i, n in enumerate(ARM_JOINTS):
            if n in msg.name:
                self.cur[i] = msg.position[msg.name.index(n)]
        self.js_ok = True

    def wait_js(self, ticks=40):
        for _ in range(ticks):
            rclpy.spin_once(self, timeout_sec=0.1)
            if self.js_ok:
                return True
        return False

    def solve(self, x, y, z, seed):
        """Yaw + Tilt-Scan; die erste lösbare 6-Gelenk-Lösung zurückgeben (sonst None)."""
        if not self.ik.wait_for_service(timeout_sec=2.0):
            return None
        yaw_out = math.atan2(y, x)
        yaws = [yaw_out, yaw_out + math.pi/2, yaw_out - math.pi/2,
                yaw_out + math.pi, yaw_out + math.pi/4, yaw_out - math.pi/4]
        tilts = [0.0, math.radians(20), math.radians(35), math.radians(45), math.radians(55)]
        for tilt in tilts:
            for yaw in yaws:
                q = quat_rpy(-math.pi/2 + tilt, 0.0, yaw)
                req = GetPositionIK.Request()
                req.ik_request.group_name = 'arm'
                req.ik_request.ik_link_name = 'tcp'
                req.ik_request.avoid_collisions = False
                req.ik_request.timeout.sec = 1
                p = PoseStamped(); p.header.frame_id = 'robot_base'
                p.pose.position.x = x; p.pose.position.y = y; p.pose.position.z = z
                p.pose.orientation.x, p.pose.orientation.y, p.pose.orientation.z, p.pose.orientation.w = q
                req.ik_request.pose_stamped = p
                js = JointState(); js.name = list(ARM_JOINTS); js.position = list(seed)
                req.ik_request.robot_state.joint_state = js
                fut = self.ik.call_async(req)
                t0 = time.monotonic()
                while not fut.done() and (time.monotonic() - t0) < 2.0:
                    rclpy.spin_once(self, timeout_sec=0.02)
                if fut.done() and fut.result() and fut.result().error_code.val == 1:
                    nm = dict(zip(fut.result().solution.joint_state.name,
                                  fut.result().solution.joint_state.position))
                    return ([nm.get(j, 0.0) for j in ARM_JOINTS],
                            math.degrees(tilt), math.degrees(yaw - yaw_out))
        return None

    def send(self, positions, secs):
        if not self.cli.wait_for_server(timeout_sec=3.0):
            return False
        g = FollowJointTrajectory.Goal()
        g.trajectory.joint_names = list(ARM_JOINTS)
        pt = JointTrajectoryPoint()
        pt.positions = list(map(float, positions))
        pt.time_from_start = Duration(sec=int(secs), nanosec=int((secs % 1) * 1e9))
        g.trajectory.points = [pt]
        f = self.cli.send_goal_async(g)
        rclpy.spin_until_future_complete(self, f)
        gh = f.result()
        if gh is None or not gh.accepted:
            return False
        rclpy.spin_until_future_complete(self, gh.get_result_async())
        return True

    def settle(self, secs):
        for _ in range(int(secs / 0.1)):
            rclpy.spin_once(self, timeout_sec=0.1)


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--z", type=float, default=0.08)
    args = ap.parse_args()
    boxes = json.load(open(JSON_PATH))

    rclpy.init()
    n = Visitor()
    if not n.wait_js():
        print("⚠ Kein /joint_states — laeuft der Stack?"); rclpy.shutdown(); return

    print(f"\n=== BOX-ABFAHRRUNDE (z={args.z*100:.0f}cm, yaw+tilt-Scan) ===")
    print("→ zum Punkt 0..."); n.send([0.0]*6, 5); n.settle(2.0); print("  ✓ bei 0.")

    for c in ORDER:
        b = boxes.get(c)
        if b is None:
            print(f"→ {c}: kein Eintrag"); continue
        res = n.solve(b[0], b[1], args.z, list(n.cur))
        if res is None:
            print(f"→ {c} (r={math.hypot(b[0],b[1])*1000:.0f}mm z={args.z*1000:.0f}mm): "
                  f"kein yaw/tilt geloest — auf dieser Hoehe NICHT ERREICHBAR."); continue
        sol, tilt, dyaw = res
        print(f"→ {c} (x={b[0]*1000:.0f} y={b[1]*1000:.0f} z={args.z*1000:.0f}mm) "
              f"tilt={tilt:.0f}° dyaw={dyaw:+.0f}° wird angefahren...")
        if n.send(sol, 6):
            n.settle(2.5); print(f"  ✓ ueber {c}.")
        else:
            print(f"  ⚠ {c}: Senden fehlgeschlagen.")

    print("→ Rueckkehr nach 0..."); n.send([0.0]*6, 5); n.settle(2.0)
    print("✅ FERTIG — Roboter am Punkt 0.")
    n.destroy_node(); rclpy.shutdown()


if __name__ == "__main__":
    main()
