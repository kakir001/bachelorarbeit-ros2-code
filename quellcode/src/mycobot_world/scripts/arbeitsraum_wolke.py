#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""Punktwolke auf den Arbeitsraum beschneiden: nur Punkte ueber der Kaiser-Grundplatte.

Die D435i sieht weit ueber den Repro-Stand hinaus (Tisch, Wand, Kabel). In RViz
lenkt das ab und kostet den Nano Renderzeit. Dieser Knoten nimmt die Wolke der
Kamera, rechnet sie nach robot_base um (kalibrierte TF, einmal geholt) und laesst
nur Punkte innerhalb eines Kastens ueber der Grundplatte durch:
    platform_top = 50 x 40 cm, Oberseite z = 0, Ursprung (0.15, -0.125) in robot_base
    -> x -0.10..0.40, y -0.325..0.075 (URDF), z -0.03..0.50, plus --rand.

Reine numpy-Arbeit auf dem Rohpuffer (kein sensor_msgs_py in Galactic): der
Punkt-Datensatz bleibt byteweise erhalten (x y z rgb ...), es werden nur Zeilen
maskiert - das ist auf dem Nano bei 424x240 @ 15 Hz billig.

Ein: /camera/depth/color/points (best effort)
Aus: /camera/arbeitsraum/points  (best effort, gleicher Frame wie die Eingabe)
Parameter: rand [m] (0.02) - Kasten in x/y so viel groesser; z_max [m] (0.50).
"""
import numpy as np
import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import PointCloud2
import tf2_ros
from rclpy.duration import Duration

BASE = 'robot_base'
# Grundplatte laut URDF (robot_base_to_platform: xyz 0.15 -0.125, Box 0.50 x 0.40)
X0, X1 = 0.15 - 0.25, 0.15 + 0.25
Y0, Y1 = -0.125 - 0.20, -0.125 + 0.20


class Wolke(Node):
    def __init__(self):
        super().__init__('arbeitsraum_wolke')
        self.declare_parameter('rand', 0.02)
        self.declare_parameter('z_max', 0.50)
        self.declare_parameter('z_min', -0.03)
        self.tf_buf = tf2_ros.Buffer(); self.tf_l = tf2_ros.TransformListener(self.tf_buf, self)
        self.R = None; self.t = None; self.frame = None
        self.pub = self.create_publisher(PointCloud2, '/camera/arbeitsraum/points', qos_profile_sensor_data)
        self.create_subscription(PointCloud2, '/camera/depth/color/points', self.cb, qos_profile_sensor_data)
        self.n_in = self.n_out = 0
        self.create_timer(10.0, self.bericht)

    def hole_tf(self, frame):
        """Kamera -> robot_base einmal holen (statisch, kalibriert); bei Fehler None."""
        try:
            tr = self.tf_buf.lookup_transform(BASE, frame, rclpy.time.Time(), timeout=Duration(seconds=0.5)).transform
        except Exception as e:  # noqa: BLE001
            self.get_logger().warn(f'TF {BASE} <- {frame} noch nicht da: {e}', throttle_duration_sec=5.0); return False
        q = tr.rotation; x, y, z, w = q.x, q.y, q.z, q.w
        self.R = np.array([[1 - 2 * (y * y + z * z), 2 * (x * y - z * w), 2 * (x * z + y * w)],
                           [2 * (x * y + z * w), 1 - 2 * (x * x + z * z), 2 * (y * z - x * w)],
                           [2 * (x * z - y * w), 2 * (y * z + x * w), 1 - 2 * (x * x + y * y)]], dtype=np.float32)
        self.t = np.array([tr.translation.x, tr.translation.y, tr.translation.z], dtype=np.float32)
        self.frame = frame
        self.get_logger().info(f'TF {BASE} <- {frame}: t={self.t.round(3).tolist()}')
        return True

    def cb(self, m: PointCloud2):
        if self.frame != m.header.frame_id and not self.hole_tf(m.header.frame_id): return
        ps = m.point_step
        # x y z liegen laut Feldern bei Offset 0/4/8 (realsense: x y z rgb, 16 oder 32 Byte)
        off = {f.name: f.offset for f in m.fields}
        if not (off.get('x') == 0 and off.get('y') == 4 and off.get('z') == 8):
            self.get_logger().error(f'unerwartetes Punktformat {off}', throttle_duration_sec=10.0); return
        raw = np.frombuffer(m.data, dtype=np.uint8)
        n = len(raw) // ps
        rows = raw[:n * ps].reshape(n, ps)
        xyz = rows[:, :12].copy().view(np.float32).reshape(n, 3)
        ok = np.isfinite(xyz).all(axis=1)
        p = xyz @ self.R.T + self.t
        r = float(self.get_parameter('rand').value)
        z0 = float(self.get_parameter('z_min').value); z1 = float(self.get_parameter('z_max').value)
        ok &= (p[:, 0] >= X0 - r) & (p[:, 0] <= X1 + r) & (p[:, 1] >= Y0 - r) & (p[:, 1] <= Y1 + r) & (p[:, 2] >= z0) & (p[:, 2] <= z1)
        out = PointCloud2(); out.header = m.header; out.fields = m.fields; out.is_bigendian = m.is_bigendian
        out.point_step = ps; out.is_dense = True
        sel = rows[ok]
        out.height = 1; out.width = int(sel.shape[0]); out.row_step = ps * out.width
        out.data = sel.tobytes()
        self.pub.publish(out); self.n_in += n; self.n_out += out.width

    def bericht(self):
        if self.n_in:
            self.get_logger().info(f'{self.n_out / self.n_in * 100:.0f} % der Punkte im Arbeitsraum', throttle_duration_sec=60.0)
            self.n_in = self.n_out = 0


def main():
    rclpy.init(); n = Wolke()
    try: rclpy.spin(n)
    except KeyboardInterrupt: pass
    n.destroy_node(); rclpy.shutdown()


if __name__ == '__main__':
    main()
