#!/usr/bin/env python3
"""
measure_zero.py — Gelenk-Null-Wiederholbarkeit: Kalibrierungsdrift oder Servo-
Toleranz? Bei jedem Versuch fährt der Arm in eine kleine Pose und wird zurück auf 0
gesendet, der echte Encoder wird aus /joint_states gelesen. N Versuche → meldet die
Offset-Konsistenz jedes Gelenks.
Läuft über laufendes move_group/arm_controller (Port muss nicht frei sein).
"""
import math
import statistics as st
import rclpy
from rclpy.node import Node
from rclpy.action import ActionClient
from control_msgs.action import FollowJointTrajectory
from trajectory_msgs.msg import JointTrajectoryPoint
from sensor_msgs.msg import JointState
from builtin_interfaces.msg import Duration

ARM = ["joint2_to_joint1", "joint3_to_joint2", "joint4_to_joint3",
       "joint5_to_joint4", "joint6_to_joint5", "joint6output_to_joint6"]
SHORT = ["j1", "j2", "j3", "j4", "j5", "j6"]
WIGGLE = [0.20, -0.20, 0.20, -0.20, 0.20, -0.20]
ZERO = [0.0] * 6
TRIALS = 5


class M(Node):
    def __init__(self):
        super().__init__("measure_zero")
        self.cli = ActionClient(self, FollowJointTrajectory,
                                "/arm_controller/follow_joint_trajectory")
        self.latest = {}
        self.create_subscription(JointState, "/joint_states", self._cb, 10)

    def _cb(self, msg):
        for n, p in zip(msg.name, msg.position):
            self.latest[n] = p

    def goto(self, pos, secs):
        g = FollowJointTrajectory.Goal()
        g.trajectory.joint_names = ARM
        pt = JointTrajectoryPoint()
        pt.positions = list(map(float, pos))
        pt.time_from_start = Duration(sec=secs)
        g.trajectory.points = [pt]
        self.cli.wait_for_server()
        f = self.cli.send_goal_async(g)
        rclpy.spin_until_future_complete(self, f)
        gh = f.result()
        if gh is None or not gh.accepted:
            self.get_logger().warn("Ziel abgelehnt"); return
        rf = gh.get_result_async()
        rclpy.spin_until_future_complete(self, rf)

    def settle_read(self, ticks=50, stable_eps=0.15):
        # Langer Spin (~5s) + verifiziere, dass die letzte Messung STABIL ist (das Servo
        # ist offene Schleife und kann sich nach Ende der Action noch bewegen). Prüfe,
        # ob die letzten 10 Messungen innerhalb von eps liegen.
        hist = []
        for _ in range(ticks):
            rclpy.spin_once(self, timeout_sec=0.1)
            hist.append([math.degrees(self.latest.get(n, float("nan"))) for n in ARM])
        last = hist[-1]
        window = hist[-10:]
        moved = max(max(abs(w[i] - last[i]) for w in window) for i in range(6))
        stable = moved < stable_eps
        return last, stable


def main():
    rclpy.init()
    node = M()
    if not node.cli.wait_for_server(timeout_sec=10.0):
        print("!! arm_controller Action nicht vorhanden — laeuft move_group/controller?")
        rclpy.shutdown(); return

    rows = []
    for t in range(1, TRIALS + 1):
        node.get_logger().info(f"Versuch {t}: wiggle → 0 ...")
        node.goto(WIGGLE, 3)
        wig, _ = node.settle_read(ticks=30)   # liveness: sollte ~+11.5° sehen
        node.goto(ZERO, 3)
        deg, stable = node.settle_read(ticks=50)
        rows.append(deg)
        flag = "" if stable else "  ⚠ NICHT STABIL (uebersprungen)"
        if not stable:
            rows.pop()  # nicht gesetzte Messung aus der Zusammenfassung entfernen
        print(f"  Versuch {t}: wiggle≈{wig[0]:+.1f}° | Null: " +
              " ".join(f"{SHORT[i]}={deg[i]:+.2f}" for i in range(6)) + flag)

    print("\n=== ZUSAMMENFASSUNG (Grad) ===")
    print(f"{'Gelenk':4} {'Mittel':>9} {'std(Streuung)':>13} {'min..max':>14}  Kommentar")
    for i in range(6):
        col = [r[i] for r in rows]
        mean = st.mean(col)
        sd = st.pstdev(col)
        rng = max(col) - min(col)
        # konsistente Drift: |Mittel| groß ABER Streuung klein → Kalibrierung
        # zufällige Toleranz: Streuung groß verglichen mit |Mittel| → Servo-Toleranz
        if abs(mean) > 0.5 and sd < 0.3:
            note = "🔧 KONSISTENTER Offset (Kalibrierungsdrift wahrscheinlich)"
        elif rng > 0.6:
            note = "🎲 gestreut (Servo-Toleranz/Backlash)"
        else:
            note = "✓ nahe Null + stabil"
        print(f"{SHORT[i]:4} {mean:+9.2f} {sd:13.2f} {min(col):+6.2f}..{max(col):+.2f}  {note}")

    node.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
