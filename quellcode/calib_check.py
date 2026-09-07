#!/usr/bin/env python3
"""
NUR-LESE WAHRNEHMUNGSTEST — Genauigkeit der Kamera->robot_base Zuordnung.

Kann parallel geöffnet werden WAEHREND click_place_moveit.py LAEUFT: rührt
pymycobot/seriellen Port NICHT an, BEWEGT den Roboter NICHT. Hört nur Kamera + TF.
Schreibt die 3D-Position des angeklickten Pixels in robot_base auf den Bildschirm
und loggt sie ins Terminal.

Zweck: die Hälfte "angeklickter Punkt -> base-Koordinate, die der Roboter SIEHT"
isolieren. Wenn du die echte physische Position mit dem Lineal misst und vergleichst,
siehst du den KAMERA-Fehler (extrinsisch/Tiefe) getrennt von der Roboterbewegung.

Verwendung:
  python3 calib_check.py
  Linksklick  = Punkt wählen (base-Koordinate wird geschrieben + bleibt fest)
  Rechtsklick = letzten Punkt löschen
  k           = alle Punkte speichern (calib_check_points.json)
  q           = beenden
"""
import math, json, threading
import cv2, numpy as np
import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy
from sensor_msgs.msg import Image, CameraInfo
from geometry_msgs.msg import PointStamped
import tf2_ros, tf2_geometry_msgs  # noqa

CAMERA_FRAME = 'camera_color_optical_frame'
ROBOT_BASE   = 'robot_base'
OUT_FILE     = 'calib_check_points.json'


def img_bgr(m):
    a = np.frombuffer(m.data, np.uint8)
    if m.encoding.lower() == 'rgb8':
        return cv2.cvtColor(a.reshape((m.height, m.width, 3)), cv2.COLOR_RGB2BGR)
    return a.reshape((m.height, m.width, 3)).copy()


def depth_arr(m):
    if m.encoding == '16UC1':
        return np.frombuffer(m.data, np.uint16).reshape((m.height, m.width))
    if m.encoding == '32FC1':
        return np.frombuffer(m.data, np.float32).reshape((m.height, m.width))
    raise ValueError(m.encoding)


class CalibCheck(Node):
    def __init__(self):
        super().__init__('calib_check')
        self._K = None
        self._bgr = None
        self._depth = None
        self._lock = threading.Lock()
        self._status = 'Warten auf Kamera...'
        self._pts = []          # [{'u','v','base':[x,y,z],'z_cam','r'}]
        self._tf = tf2_ros.Buffer()
        tf2_ros.TransformListener(self._tf, self)
        qos = QoSProfile(reliability=ReliabilityPolicy.RELIABLE,
                         history=HistoryPolicy.KEEP_LAST, depth=1)
        self.create_subscription(CameraInfo, '/camera/color/camera_info', self._oninfo, qos)
        self.create_subscription(Image, '/camera/color/image_raw', self._oncolor, 10)
        self.create_subscription(Image, '/camera/aligned_depth_to_color/image_raw', self._ondepth, 10)

    def _oninfo(self, m):
        self._K = np.array(m.k, np.float64).reshape(3, 3)

    def _oncolor(self, m):
        try:
            with self._lock:
                self._bgr = img_bgr(m)
                if self._status.startswith('Warten'):
                    self._status = 'Bereit — klicke auf einen Punkt'
        except Exception:
            pass

    def _ondepth(self, m):
        try:
            with self._lock:
                self._depth = depth_arr(m)
        except Exception:
            pass

    def pixel_to_base(self, u, v):
        """GLEICHE Mathematik wie click_place_moveit.pixel_to_base (7x7 Median-Tiefe)."""
        if self._K is None:
            return None, None
        with self._lock:
            depth = self._depth
        if depth is None:
            return None, None
        h, w = depth.shape
        if not (0 <= v < h and 0 <= u < w):
            return None, None
        reg = depth[max(0, v - 3):v + 4, max(0, u - 3):u + 4]
        val = reg[reg > 0]
        if len(val) == 0:
            return None, None
        z = float(np.median(val))
        z = z / 1000.0 if z > 100 else z       # mm -> m (16UC1)
        fx, fy = self._K[0, 0], self._K[1, 1]
        cx, cy = self._K[0, 2], self._K[1, 2]
        pc = PointStamped()
        pc.header.frame_id = CAMERA_FRAME
        pc.header.stamp = self.get_clock().now().to_msg()
        pc.point.x = (u - cx) * z / fx
        pc.point.y = (v - cy) * z / fy
        pc.point.z = z
        try:
            pb = self._tf.transform(pc, ROBOT_BASE, timeout=rclpy.time.Duration(seconds=2.0))
        except Exception as e:
            self.get_logger().warn(f'TF: {e}')
            return None, z
        return np.array([pb.point.x, pb.point.y, pb.point.z]), z

    def on_click(self, u, v):
        pb, zcam = self.pixel_to_base(u, v)
        if pb is None:
            self._status = f'Depth/TF nicht verfuegbar ({u},{v})  z_cam={zcam}'
            self.get_logger().warn(self._status)
            return
        r = math.hypot(pb[0], pb[1])
        self._pts.append({'u': u, 'v': v,
                          'base': [float(pb[0]), float(pb[1]), float(pb[2])],
                          'z_cam': float(zcam), 'r': float(r)})
        self._status = (f'#{len(self._pts)} base=[{pb[0]*1000:.0f},{pb[1]*1000:.0f},'
                        f'{pb[2]*1000:.0f}]mm  r={r*1000:.0f}mm  Tiefe={zcam*1000:.0f}mm')
        self.get_logger().info('>> ' + self._status)

    def clear_last(self):
        if self._pts:
            self._pts.pop()
            self._status = f'letzter Punkt geloescht ({len(self._pts)} verbleiben)'

    def save(self):
        try:
            with open(OUT_FILE, 'w') as f:
                json.dump({'points': self._pts}, f, indent=2)
            self._status = f'{len(self._pts)} Punkte gespeichert -> {OUT_FILE}'
            self.get_logger().info(self._status)
        except Exception as e:
            self._status = f'Speicher-FEHLER: {e}'

    def frame(self):
        with self._lock:
            if self._bgr is None:
                return None
            f = self._bgr.copy()
        h, w = f.shape[:2]
        for i, p in enumerate(self._pts):
            u, v = p['u'], p['v']
            col = (0, 255, 0)
            cv2.drawMarker(f, (u, v), col, cv2.MARKER_CROSS, 22, 2)
            cv2.circle(f, (u, v), 10, col, 2)
            b = p['base']
            cv2.putText(f, f"#{i+1} [{b[0]*1000:.0f},{b[1]*1000:.0f},{b[2]*1000:.0f}]mm",
                        (u + 12, v - 6), cv2.FONT_HERSHEY_SIMPLEX, 0.45, col, 1, cv2.LINE_AA)
        cv2.rectangle(f, (0, 0), (w, 26), (40, 40, 40), -1)
        cv2.putText(f, self._status, (8, 18), cv2.FONT_HERSHEY_SIMPLEX, 0.5,
                    (255, 255, 255), 1, cv2.LINE_AA)
        cv2.rectangle(f, (0, h - 22), (w, h), (40, 40, 40), -1)
        cv2.putText(f, 'NUR-LESE (Roboter BEWEGT SICH NICHT)  Links:Punkt  Rechts:letzten-loeschen  k:speichern  q:beenden',
                    (4, h - 6), cv2.FONT_HERSHEY_SIMPLEX, 0.42, (180, 180, 180), 1, cv2.LINE_AA)
        return f


def mouse(ev, x, y, fl, node):
    if ev == cv2.EVENT_LBUTTONDOWN:
        node.on_click(x, y)
    elif ev == cv2.EVENT_RBUTTONDOWN:
        node.clear_last()


def main():
    rclpy.init()
    node = CalibCheck()
    threading.Thread(target=lambda: rclpy.spin(node), daemon=True).start()
    cv2.namedWindow('Calib Check (nur-lese)', cv2.WINDOW_NORMAL)
    cv2.resizeWindow('Calib Check (nur-lese)', 1280, 720)
    cv2.setMouseCallback('Calib Check (nur-lese)', mouse, node)
    print('\n  CALIB CHECK — BEWEGT den Roboter NICHT. Klicke auf einen Punkt, base-Koordinate wird geschrieben.\n')
    try:
        while rclpy.ok():
            fr = node.frame()
            if fr is not None:
                cv2.imshow('Calib Check (nur-lese)', fr)
            k = cv2.waitKey(50) & 0xFF
            if k in (ord('q'), ord('Q')):
                break
            elif k in (ord('k'), ord('K')):
                node.save()
    except KeyboardInterrupt:
        pass
    finally:
        cv2.destroyAllWindows()
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
