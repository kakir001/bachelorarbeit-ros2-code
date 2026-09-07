#!/usr/bin/env python3
# NOT-AUS-Relay — übersetzt das ROS-Topic /estop in das "estop"-Wire-Kommando
# des mycobot_bridge-Prozesses (Unix-Kommando-Socket).
#
# Hintergrund: mycobot_bridge.py ist BEWUSST KEIN ROS-Node (pybind11 2.2.7 auf
# Galactic segfaultet bei geteiltem GIL mit Hintergrund-Threads, siehe Docstring
# dort). Damit der Not-Aus-Taster (estop_button, publiziert /estop) trotzdem den
# echten Roboter stoppt, abonniert DIESER kleine Node das Topic und leitet den
# Zustand als einzeiliges Kommando an den Kommando-Socket des Bridge weiter.
#
# Eigenschaften:
#   * /estop ist transient_local (gelatcht) — beim Start übernimmt der Relay
#     sofort den letzten Zustand (z.B. wenn der Taster früher gedrückt wurde).
#   * Heartbeat: solange NOT-AUS aktiv ist, wird "estop 1" alle 2 s erneut
#     gesendet — ein neu gestarteter Bridge-Prozess (Stack-Neustart) fällt so
#     nicht in den freigegebenen Zustand zurück.
#   * Läuft strikt einfädig (rclpy.spin im Hauptthread, kein Executor-Thread)
#     — vermeidet die Galactic-pybind11-Threading-Falle.
#
# Start (normalerweise automatisch über demo.launch.py bei echter Hardware):
#   ros2 run mycobot_hardware estop_relay.py

import os
import socket

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, DurabilityPolicy, ReliabilityPolicy
from std_msgs.msg import Bool

# Kommando-Socket des Bridge — gleiche Konvention wie mycobot_bridge.py
# (--socket Basis-Pfad + ".cmd"). Per Umgebungsvariable überschreibbar.
CMD_SOCK = os.environ.get('MYCOBOT_BRIDGE_SOCKET', '/tmp/mycobot_bridge.sock') + '.cmd'


class EstopRelay(Node):
    def __init__(self):
        super().__init__('estop_relay')
        # QoS MUSS zum estop_button passen: transient_local + reliable, depth 1.
        q = QoSProfile(depth=1)
        q.durability = DurabilityPolicy.TRANSIENT_LOCAL
        q.reliability = ReliabilityPolicy.RELIABLE
        self._active = False
        self._fail_logged = False   # Socket-Fehler nur einmal loggen (kein Log-Spam)
        self.create_subscription(Bool, '/estop', self._on_estop, q)
        # Ack an den Taster: das Kommando wurde WIRKLICH auf den Bridge-Socket
        # geschrieben (Vorfall 2026-07-16: Taster hatte keine Rueckmeldung, ob der
        # Stopp irgendwo ankommt). estop_button abonniert /estop_ack und zeigt es an.
        self._pub_ack = self.create_publisher(Bool, '/estop_ack', q)
        self.create_timer(2.0, self._tick)
        self.get_logger().info(f'NOT-AUS-Relay bereit: /estop -> {CMD_SOCK}')

    def _send(self, active):
        """Zustand als Wire-Kommando an den Bridge-Socket schicken (einmalige Verbindung)."""
        try:
            s = socket.socket(socket.AF_UNIX, socket.SOCK_STREAM)
            s.settimeout(1.0)
            s.connect(CMD_SOCK)
            s.sendall(b'estop 1\n' if active else b'estop 0\n')
            s.close()
            self._fail_logged = False
            return True
        except OSError as e:
            # Bridge (noch) nicht da — z.B. fake hardware oder Stack startet gerade.
            if not self._fail_logged:
                self.get_logger().warn(f'Bridge-Socket nicht erreichbar ({e}) — es wird weiter versucht')
                self._fail_logged = True
            return False

    def _on_estop(self, msg):
        self._active = bool(msg.data)
        ok = self._send(self._active)
        text = 'NOT-AUS -> STOP' if self._active else 'NOT-AUS -> FREIGABE'
        if ok:
            self.get_logger().warn(text)
            self._pub_ack.publish(Bool(data=self._active))   # Zustellung bestaetigen
        else:
            self.get_logger().error(text + ' — Bridge-Socket NICHT erreichbar!')

    def _tick(self):
        # Heartbeat nur im STOP-Zustand — Freigabe wird nur bei Zustandswechsel gesendet.
        if self._active:
            self._send(True)


def main():
    rclpy.init()
    node = EstopRelay()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
