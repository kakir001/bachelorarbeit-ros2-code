#!/usr/bin/env python3
# myCobot 280 JN — bidirectional serial bridge for hand-eye calibration.
#
# Ein einziger Prozess hält den Port /dev/ttyTHS1:
#   * liest Encoder, publiziert /joint_states -> robot_state_publisher aktualisiert TF
#   * hört auf das Topic /joint_commands, sendet mit send_radians an den Roboter
#
# Die Slider-GUI (joint_state_publisher_gui) publiziert auf /joint_commands
# (Remap in der Launch-Datei). Die Servos bleiben IMMER GESPERRT — wir geben sie
# nicht frei (Drift-Problem). Der Benutzer stellt mit den Slidern die Pose ein,
# hält die Servos gesperrt, easy_handeye2 nimmt das Sample.
#
# send_coords wird NIEMALS aufgerufen. send_radians wird nur für das vom Slider
# kommende Gelenkziel verwendet (Phase 5 Sonderausnahme — im DEVLOG vermerkt).

import threading
import time

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, DurabilityPolicy, ReliabilityPolicy
from sensor_msgs.msg import JointState
from std_msgs.msg import Bool

from pymycobot import MyCobot280


# Gleiche Index-Reihenfolge wie pymycobot.get_radians().
URDF_ARM_JOINTS = [
    'joint2_to_joint1',
    'joint3_to_joint2',
    'joint4_to_joint3',
    'joint5_to_joint4',
    'joint6_to_joint5',
    'joint6output_to_joint6',
]

# Greifer-Gelenknamen — werden während der Kalibrierung auf 0 (geschlossen) gehalten.
# Die mimic-Gelenke folgen dem Hauptgelenk per URDF mimic, aber in Galactic
# erwartet robot_state_publisher alle Gelenke in /joint_states — daher
# publizieren wir alle explizit.
URDF_GRIPPER_JOINTS = [
    'gripper_controller',
    'gripper_base_to_gripper_left2',
    'gripper_left3_to_gripper_left1',
    'gripper_base_to_gripper_right3',
    'gripper_base_to_gripper_right2',
    'gripper_right3_to_gripper_right1',
]

# Bei Problemen mit der Encoder-Leserichtung auf -1 setzen (nach Sanity-Check in RViz).
ENCODER_SIGNS = [1, 1, 1, 1, 1, 1]
# send_radians Richtung hat dasselbe Problem — falls beim korrekten Slider-Senden ein Flip nötig ist, -1.
COMMAND_SIGNS = [1, 1, 1, 1, 1, 1]


class MyCobotBridge(Node):
    def __init__(self) -> None:
        super().__init__('mycobot_bridge')

        self.declare_parameter('port', '/dev/ttyTHS1')
        self.declare_parameter('baudrate', 1000000)
        self.declare_parameter('rate_hz', 10.0)
        self.declare_parameter('command_speed', 30)  # 0-100 pymycobot speed
        self.declare_parameter('min_command_interval_s', 0.20)  # 5 Hz max
        self.declare_parameter('freehand', False)

        port = self.get_parameter('port').value
        baud = int(self.get_parameter('baudrate').value)
        rate = float(self.get_parameter('rate_hz').value)
        self._speed = int(self.get_parameter('command_speed').value)
        self._min_cmd_dt = float(self.get_parameter('min_command_interval_s').value)
        self._freehand = bool(self.get_parameter('freehand').value)

        self.get_logger().info(f'Opening serial: {port} @ {baud}')
        self._mc = MyCobot280(port, baud)
        time.sleep(0.5)

        if self._freehand:
            self._mc.release_all_servos()
            time.sleep(0.3)
            self.get_logger().info('FREEHAND MODE — Servos frei, es wird nur der Encoder gelesen')

        radians = self._mc.get_radians()
        if not isinstance(radians, list) or len(radians) != 6:
            raise RuntimeError(f'Unexpected get_radians(): {radians!r}')
        self.get_logger().info(
            'Connected. Initial joints (rad): '
            + ', '.join(f'{r:+.3f}' for r in radians)
        )

        self._lock = threading.Lock()
        self._last_cmd_t = 0.0
        self._pending_cmd = None
        self._last_sent_cmd = None
        self._cmd_eps = 0.002

        self._pub = self.create_publisher(JointState, '/joint_states', 10)
        self.create_timer(1.0 / rate, self._on_read_tick)
        self.create_timer(5.0, self._log_stats)

        if not self._freehand:
            self.create_subscription(
                JointState, '/joint_commands', self._on_command, 10
            )
            self.create_timer(self._min_cmd_dt, self._on_command_tick)

        self._reads = 0
        self._fails = 0
        self._cmds = 0

        # --- NOT-AUS (Emergency Stop) ---
        # /estop (Bool): true=STOP (hält jede Bewegung an + blockiert Kommandos),
        # false=FREIGABE. Transient-local, damit ein spät gestarteter Bridge sofort
        # den aktuellen Zustand kennt. stop() hält an OHNE Servos freizugeben
        # (release_all_servos würde den Arm fallen lassen).
        self._estop = False
        _eq = QoSProfile(depth=1)
        _eq.durability = DurabilityPolicy.TRANSIENT_LOCAL
        _eq.reliability = ReliabilityPolicy.RELIABLE
        self.create_subscription(Bool, '/estop', self._on_estop, _eq)
        self.get_logger().info('NOT-AUS bereit: /estop (true=STOP, false=FREIGABE)')

    def _on_estop(self, msg: Bool) -> None:
        if msg.data:
            if not self._estop:
                self.get_logger().warn(
                    '*** NOT-AUS AKTIV *** — jede Bewegung gestoppt, Kommandos blockiert'
                )
            self._estop = True
            self._pending_cmd = None
            with self._lock:
                for _ in range(3):
                    try:
                        self._mc.stop()
                    except Exception as e:
                        self.get_logger().warn(f'stop() failed: {e}')
                    time.sleep(0.02)
        else:
            if self._estop:
                self.get_logger().info(
                    'NOT-AUS FREIGABE — Kommandos wieder aktiv (neue Pose senden)'
                )
            self._estop = False
            self._last_sent_cmd = None  # erzwingt Senden des nächsten Kommandos

    def _on_read_tick(self) -> None:
        with self._lock:
            try:
                radians = self._mc.get_radians()
            except Exception as e:
                self._fails += 1
                self.get_logger().warn(f'read failed: {e}', throttle_duration_sec=2.0)
                return

        if radians is None or isinstance(radians, int) or len(radians) != 6:
            self._fails += 1
            return

        msg = JointState()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.name = list(URDF_ARM_JOINTS) + list(URDF_GRIPPER_JOINTS)
        msg.position = (
            [float(r) * ENCODER_SIGNS[i] for i, r in enumerate(radians)]
            + [0.0] * len(URDF_GRIPPER_JOINTS)
        )
        self._pub.publish(msg)
        self._reads += 1

    def _on_command(self, msg: JointState) -> None:
        if self._estop:
            return  # NOT-AUS aktiv: Kommandos ignorieren
        # Im vom Slider-GUI gesendeten JointState können sowohl Arm- als auch Greifer-Gelenke sein;
        # nur die 6 Arm-Gelenke extrahieren.
        name_to_pos = dict(zip(msg.name, msg.position))
        cmd = []
        for j in URDF_ARM_JOINTS:
            if j not in name_to_pos:
                return  # fehlendes Gelenk -> nicht senden
            cmd.append(name_to_pos[j])
        # Letztes Ziel speichern; der Timer fasst es zusammen und sendet (verhindert Slider-Spam).
        self._pending_cmd = [cmd[i] * COMMAND_SIGNS[i] for i in range(6)]

    def _on_command_tick(self) -> None:
        if self._estop:
            self._pending_cmd = None  # NOT-AUS aktiv: nichts senden
            return
        if self._pending_cmd is None:
            return
        now = time.monotonic()
        if (now - self._last_cmd_t) < self._min_cmd_dt:
            return
        cmd = self._pending_cmd
        self._pending_cmd = None

        # Wenn die Abweichung vom zuvor gesendeten Kommando kleiner als eps ist, überspringen —
        # damit bei konstant gehaltenem Slider kein dauerndes send_radians-Spam entsteht.
        if self._last_sent_cmd is not None:
            delta = max(abs(cmd[i] - self._last_sent_cmd[i]) for i in range(6))
            if delta < self._cmd_eps:
                return

        with self._lock:
            try:
                self._mc.send_radians(cmd, self._speed)
            except Exception as e:
                self.get_logger().warn(f'send_radians failed: {e}', throttle_duration_sec=2.0)
                return
        self._last_cmd_t = now
        self._last_sent_cmd = list(cmd)
        self._cmds += 1
        self.get_logger().info(
            'cmd -> ' + ', '.join(f'{c:+.3f}' for c in cmd)
        )

    def _log_stats(self) -> None:
        self.get_logger().info(
            f'reads={self._reads}  fails={self._fails}  cmds_sent={self._cmds}'
        )


def main() -> None:
    rclpy.init()
    node = MyCobotBridge()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
