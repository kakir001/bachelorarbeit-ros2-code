#!/usr/bin/env python3
# view_overlay.py — /welle/overlay in einem LEICHTGEWICHTIGEN Fenster live zeigen.
# Statt RViz/rqt (Nano-RAM sparen). Die gewählte Welle GRUEN + mit "ZIEL" markiert.
#
# Verwendung (separates Terminal):
#   source /opt/ros/galactic/setup.bash && source ~/ros2_ws/install/setup.bash
#   DISPLAY=:0 python3 ~/ros2_ws/view_overlay.py
#   # im Fenster 'q' oder ESC = Ende
#
# Wenn du headless willst (kein Fenster, nur alle N Sekunden ein PNG speichern):
#   SAVE_ONLY=1 SAVE_DIR=/tmp/welle python3 ~/ros2_ws/view_overlay.py
import os
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
import numpy as np
import cv2

SAVE_ONLY = os.environ.get("SAVE_ONLY", "0") == "1"
SAVE_DIR = os.environ.get("SAVE_DIR", "/tmp/welle")
WIN = "Welle Overlay (ZIEL=gruen)  -  q/ESC=Ende"


class Viewer(Node):
    def __init__(self):
        super().__init__("welle_overlay_betrachter")
        # /welle/overlay wird mit default (reliable) QoS veröffentlicht; zum Matchen default-Abonnent.
        self.sub = self.create_subscription(Image, "/welle/overlay", self.cb, 1)
        self.frame = None
        self.n = 0
        if SAVE_ONLY:
            os.makedirs(SAVE_DIR, exist_ok=True)
            self.get_logger().info(f"SAVE_ONLY — alle 30 Frames ein PNG → {SAVE_DIR}")
        else:
            cv2.namedWindow(WIN, cv2.WINDOW_NORMAL)
            cv2.resizeWindow(WIN, 848, 480)
            self.get_logger().info("warte auf /welle/overlay... (im Fenster mit q/ESC beenden)")
        self.timer = self.create_timer(0.03, self.tick)

    def cb(self, msg: Image):
        # rgb8/bgr8 → BGR ndarray (ohne cv_bridge, wenig Abhängigkeiten)
        buf = np.frombuffer(msg.data, dtype=np.uint8).reshape(msg.height, msg.width, -1)
        if msg.encoding == "rgb8":
            buf = cv2.cvtColor(buf, cv2.COLOR_RGB2BGR)
        self.frame = buf

    def tick(self):
        if self.frame is None:
            return
        if SAVE_ONLY:
            self.n += 1
            if self.n % 30 == 0:
                p = os.path.join(SAVE_DIR, "overlay_latest.png")
                cv2.imwrite(p, self.frame)
                self.get_logger().info(f"gespeichert: {p}")
            return
        cv2.imshow(WIN, self.frame)
        k = cv2.waitKey(1) & 0xFF
        if k in (ord("q"), 27):  # q oder ESC
            rclpy.shutdown()


def main():
    rclpy.init()
    node = Viewer()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        if not SAVE_ONLY:
            cv2.destroyAllWindows()
        if rclpy.ok():
            node.destroy_node()
            rclpy.shutdown()


if __name__ == "__main__":
    main()
