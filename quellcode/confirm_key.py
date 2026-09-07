#!/usr/bin/env python3
# =====================================================================
#  confirm_key.py — 'g'-Bestätigung in der RViz-Vorschau-Phase (da cam_viewer geschlossen ist).
#
#  Jedes ENTER im Terminal (oder 'g'+ENTER) → veröffentlicht /pick/confirm (std_msgs/Empty).
#  pick_tilt wartet zweimal auf Bestätigung: (1) im HOVER die Trajektorie in RViz prüfen→bestätigen,
#  (2) nach dem Greifen "gehalten?"→bestätigen. Mit Ctrl+D / Ctrl+C beenden.
# =====================================================================
import sys
import rclpy
from rclpy.node import Node
from std_msgs.msg import Empty


def main():
    rclpy.init()
    n = Node("confirm_key")
    pub = n.create_publisher(Empty, "/pick/confirm", 1)
    print("[confirm_key] BEREIT. Pruefe die Trajektorie in RViz; zum BESTAETIGEN ENTER in diesem Terminal druecken.")
    print("[confirm_key] (pick_tilt wartet auf 2 Bestaetigungen: nach HOVER + nach dem Greifen.) Ctrl+C=beenden")
    try:
        for _ in sys.stdin:
            pub.publish(Empty())
            print("[confirm_key] ✓ /pick/confirm gesendet.")
    except KeyboardInterrupt:
        pass
    n.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
