#!/usr/bin/env python3
# record_cam_publisher.py — die USB-2.0-Kamera neben der Plattform (/dev/video3)
# auf ein ROS-Image-Topic veröffentlichen. Zweck: Aufnahme + Beobachtung in RViz,
# während der Roboter die Welle hält.
#
# Kamera 90° seitlich montiert → Korrektur per ROTATE env (cw/ccw/180/none).
# Ohne cv_bridge (wie das wellen_detektor-Muster), in Galactic wenig Abhängigkeiten.
#
# Verwendung (separates Terminal):
#   source /opt/ros/galactic/setup.bash && source ~/ros2_ws/install/setup.bash
#   python3 ~/ros2_ws/record_cam_publisher.py
#   # Parameter per env: DEV, WIDTH, HEIGHT, FPS, ROTATE, TOPIC
import os
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from std_msgs.msg import Header
import numpy as np
import cv2

DEV    = int(os.environ.get("REC_DEV", "3"))         # /dev/video3
WIDTH  = int(os.environ.get("REC_WIDTH", "640"))
HEIGHT = int(os.environ.get("REC_HEIGHT", "480"))
FPS    = float(os.environ.get("REC_FPS", "15"))
ROTATE = os.environ.get("REC_ROTATE", "cw").lower()  # cw / ccw / 180 / none
TOPIC  = os.environ.get("REC_TOPIC", "/record_cam/image_raw")
FRAME  = os.environ.get("REC_FRAME", "record_cam")

_ROT = {
    "cw":  cv2.ROTATE_90_CLOCKWISE,
    "ccw": cv2.ROTATE_90_COUNTERCLOCKWISE,
    "180": cv2.ROTATE_180,
}


def bgr_to_image(img, frame_id):
    msg = Image()
    msg.header = Header()
    msg.header.frame_id = frame_id
    msg.height, msg.width = img.shape[:2]
    msg.encoding = "bgr8"
    msg.is_bigendian = 0
    msg.step = msg.width * 3
    msg.data = np.ascontiguousarray(img).tobytes()
    return msg


class RecordCam(Node):
    def __init__(self):
        super().__init__("record_cam_publisher")
        self.pub = self.create_publisher(Image, TOPIC, 1)
        self.cap = cv2.VideoCapture(DEV, cv2.CAP_V4L2)
        self.cap.set(cv2.CAP_PROP_FRAME_WIDTH, WIDTH)
        self.cap.set(cv2.CAP_PROP_FRAME_HEIGHT, HEIGHT)
        try:
            self.cap.set(cv2.CAP_PROP_AUTO_EXPOSURE, 3)  # UVC auto
        except Exception:
            pass
        if not self.cap.isOpened():
            self.get_logger().error(f"/dev/video{DEV} konnte nicht geoeffnet werden!")
        else:
            self.get_logger().info(
                f"/dev/video{DEV} geoeffnet → {TOPIC} ({WIDTH}x{HEIGHT}@{FPS} rot={ROTATE})")
        # ein paar Frames verwerfen, damit sich die Auto-Belichtung einpegelt
        for _ in range(10):
            self.cap.read()
        self.timer = self.create_timer(1.0 / max(FPS, 1.0), self.tick)
        self._warned = False

    def tick(self):
        ok, frame = self.cap.read()
        if not ok or frame is None:
            if not self._warned:
                self.get_logger().warn("kein Frame empfangen", throttle_duration_sec=5.0)
                self._warned = True
            return
        if ROTATE in _ROT:
            frame = cv2.rotate(frame, _ROT[ROTATE])
        self.pub.publish(bgr_to_image(frame, FRAME))

    def destroy_node(self):
        try:
            self.cap.release()
        except Exception:
            pass
        super().destroy_node()


def main():
    rclpy.init()
    node = RecordCam()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
