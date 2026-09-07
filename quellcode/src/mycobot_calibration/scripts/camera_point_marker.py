#!/usr/bin/env python3
"""
Kamera-Bild anklicken -> in RViz als 3D-Marker anzeigen.
Roboter bewegt sich nicht, zeigt nur die 3D-Position des angeklickten
Punkts im robot_base Frame als Kugel in RViz an.

Verwendung:
  Terminal 1: System läuft (start_mycobot.sh + Kamera)
  Terminal 2: python3 ~/ros2_ws/src/mycobot_calibration/scripts/camera_point_marker.py

Mouse:
  Links klicken  — Punkt wählen (gelbe Kugel in RViz sichtbar)
  Rechts klicken — löschen
Keys:
  Q  — beenden
"""

import threading
from typing import Optional, Tuple

import cv2
import numpy as np

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy

from sensor_msgs.msg import Image, CameraInfo
from geometry_msgs.msg import PointStamped
from visualization_msgs.msg import Marker

import tf2_ros
import tf2_geometry_msgs  # noqa: F401

CAMERA_FRAME = 'camera_color_optical_frame'
ROBOT_BASE_FRAME = 'robot_base'


class CameraPointMarker(Node):
    def __init__(self):
        super().__init__('camera_point_marker')

        self._K: Optional[np.ndarray] = None
        self._latest_bgr: Optional[np.ndarray] = None
        self._latest_depth: Optional[np.ndarray] = None
        self._lock = threading.Lock()

        self._click_px: Optional[Tuple[int, int]] = None
        self._target_3d: Optional[np.ndarray] = None
        self._status = 'Warte auf Kamera...'
        self._marker_id = 0

        self._tf_buffer = tf2_ros.Buffer()
        self._tf_listener = tf2_ros.TransformListener(self._tf_buffer, self)

        self._marker_pub = self.create_publisher(
            Marker, '/camera_click_marker', 10
        )

        qos_ci = QoSProfile(
            reliability=ReliabilityPolicy.RELIABLE,
            history=HistoryPolicy.KEEP_LAST, depth=1,
        )
        self.create_subscription(
            CameraInfo, '/camera/color/camera_info', self._on_info, qos_ci
        )
        self.create_subscription(
            Image, '/camera/color/image_raw', self._on_color, 10
        )
        self.create_subscription(
            Image, '/camera/aligned_depth_to_color/image_raw',
            self._on_depth, 10
        )

    def _on_info(self, msg: CameraInfo):
        self._K = np.array(msg.k, dtype=np.float64).reshape(3, 3)

    def _on_color(self, msg: Image):
        try:
            arr = np.frombuffer(msg.data, dtype=np.uint8)
            enc = msg.encoding.lower()
            if enc == 'rgb8':
                bgr = cv2.cvtColor(
                    arr.reshape((msg.height, msg.width, 3)), cv2.COLOR_RGB2BGR
                )
            else:
                bgr = arr.reshape((msg.height, msg.width, 3)).copy()
            with self._lock:
                self._latest_bgr = bgr
                if self._status == 'Warte auf Kamera...':
                    self._status = 'Bereit — klicken!'
        except Exception:
            pass

    def _on_depth(self, msg: Image):
        try:
            if msg.encoding == '16UC1':
                d = np.frombuffer(msg.data, dtype=np.uint16).reshape(
                    (msg.height, msg.width)
                )
            elif msg.encoding == '32FC1':
                d = np.frombuffer(msg.data, dtype=np.float32).reshape(
                    (msg.height, msg.width)
                )
            else:
                return
            with self._lock:
                self._latest_depth = d
        except Exception:
            pass

    def pixel_to_3d_base(self, u: int, v: int) -> Optional[np.ndarray]:
        if self._K is None:
            self._status = 'Camera Info noch nicht da'
            return None
        with self._lock:
            depth = self._latest_depth
        if depth is None:
            self._status = 'Keine Tiefen-Daten'
            return None

        h, w = depth.shape
        if not (0 <= v < h and 0 <= u < w):
            return None

        region = depth[max(0, v - 3):v + 4, max(0, u - 3):u + 4]
        valid = region[region > 0]
        if len(valid) == 0:
            self._status = f'Keine Tiefe ({u},{v})'
            return None

        z_raw = float(np.median(valid))
        z = z_raw / 1000.0 if z_raw > 100 else z_raw

        fx, fy = self._K[0, 0], self._K[1, 1]
        cx, cy = self._K[0, 2], self._K[1, 2]
        x_cam = (u - cx) * z / fx
        y_cam = (v - cy) * z / fy

        pt_cam = PointStamped()
        pt_cam.header.frame_id = CAMERA_FRAME
        pt_cam.header.stamp = self.get_clock().now().to_msg()
        pt_cam.point.x = float(x_cam)
        pt_cam.point.y = float(y_cam)
        pt_cam.point.z = float(z)

        try:
            pt_base = self._tf_buffer.transform(
                pt_cam, ROBOT_BASE_FRAME,
                timeout=rclpy.time.Duration(seconds=2.0)
            )
        except Exception as e:
            self._status = f'TF-Fehler: {e}'
            return None

        return np.array([
            pt_base.point.x, pt_base.point.y, pt_base.point.z
        ], dtype=np.float64)

    def on_click(self, u: int, v: int):
        p = self.pixel_to_3d_base(u, v)
        if p is None:
            self._click_px = (u, v)
            self._target_3d = None
            return

        self._click_px = (u, v)
        self._target_3d = p
        self._status = (
            f'robot_base: [{p[0]:.3f}, {p[1]:.3f}, {p[2]:.3f}] m'
        )
        self.get_logger().info(self._status)
        self._publish_marker(p)

    def _publish_marker(self, p: np.ndarray):
        m = Marker()
        m.header.frame_id = ROBOT_BASE_FRAME
        m.header.stamp = self.get_clock().now().to_msg()
        m.ns = 'camera_click'
        m.id = self._marker_id
        self._marker_id += 1
        m.type = Marker.SPHERE
        m.action = Marker.ADD
        m.pose.position.x = float(p[0])
        m.pose.position.y = float(p[1])
        m.pose.position.z = float(p[2])
        m.pose.orientation.w = 1.0
        m.scale.x = 0.015
        m.scale.y = 0.015
        m.scale.z = 0.015
        m.color.r = 1.0
        m.color.g = 1.0
        m.color.b = 0.0
        m.color.a = 0.9
        m.lifetime.sec = 300
        self._marker_pub.publish(m)

    def clear_markers(self):
        m = Marker()
        m.header.frame_id = ROBOT_BASE_FRAME
        m.header.stamp = self.get_clock().now().to_msg()
        m.ns = 'camera_click'
        m.action = Marker.DELETEALL
        self._marker_pub.publish(m)
        self._click_px = None
        self._target_3d = None
        self._marker_id = 0
        self._status = 'Geloescht — klicken!'

    def get_display_frame(self) -> Optional[np.ndarray]:
        with self._lock:
            if self._latest_bgr is None:
                return None
            frame = self._latest_bgr.copy()

        h, w = frame.shape[:2]

        if self._click_px is not None:
            u, v = self._click_px
            color = (0, 255, 255)
            cv2.circle(frame, (u, v), 8, color, 2)
            cv2.drawMarker(frame, (u, v), color, cv2.MARKER_CROSS, 16, 2)

            if self._target_3d is not None:
                p = self._target_3d
                text = f'[{p[0]:.3f}, {p[1]:.3f}, {p[2]:.3f}]'
                cv2.putText(frame, text, (u + 12, v - 12),
                            cv2.FONT_HERSHEY_SIMPLEX, 0.45,
                            color, 1, cv2.LINE_AA)

        if self._status:
            cv2.rectangle(frame, (0, 0), (w, 26), (40, 40, 40), -1)
            cv2.putText(frame, self._status, (6, 18),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.4,
                        (255, 255, 255), 1, cv2.LINE_AA)

        cv2.rectangle(frame, (0, h - 20), (w, h), (40, 40, 40), -1)
        cv2.putText(frame, 'Links: Punkt waehlen | Rechts: loeschen | Q: beenden',
                    (4, h - 5), cv2.FONT_HERSHEY_SIMPLEX, 0.3,
                    (180, 180, 180), 1, cv2.LINE_AA)

        return frame


def mouse_cb(event, x, y, flags, param):
    node = param
    if event == cv2.EVENT_LBUTTONDOWN:
        node.on_click(x, y)
    elif event == cv2.EVENT_RBUTTONDOWN:
        node.clear_markers()


def main():
    rclpy.init()
    node = CameraPointMarker()

    spin_thread = threading.Thread(
        target=lambda: rclpy.spin(node), daemon=True
    )
    spin_thread.start()

    cv2.namedWindow('Camera → RViz Marker', cv2.WINDOW_NORMAL)
    cv2.resizeWindow('Camera → RViz Marker', 640, 380)
    cv2.setMouseCallback('Camera → RViz Marker', mouse_cb, node)

    print('\n' + '=' * 50)
    print('  Camera Point Marker')
    print('=' * 50)
    print('  Links klicken : Punkt in Kamera waehlen -> in RViz sehen')
    print('  Rechts klicken: alle Marker loeschen')
    print('  Q             : beenden')
    print('=' * 50 + '\n')

    try:
        while rclpy.ok():
            frame = node.get_display_frame()
            if frame is not None:
                cv2.imshow('Camera → RViz Marker', frame)

            key = cv2.waitKey(50) & 0xFF
            if key == ord('q') or key == ord('Q'):
                break
    except KeyboardInterrupt:
        pass
    finally:
        cv2.destroyAllWindows()
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
