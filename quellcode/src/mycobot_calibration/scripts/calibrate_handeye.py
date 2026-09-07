#!/usr/bin/env python3
"""
Terminal-based hand-eye calibration sampler for myCobot 280 JN + D435i.

Board-Erkennung erfolgt durch charuco_detector Node (grünes Overlay in RViz).
Dieses Skript liest nur Posen aus TF, sammelt Samples und berechnet die Kalibrierung.

Usage:
  Terminal 1: ros2 launch mycobot_calibration calibrate_session.launch.py
  Terminal 2: python3 ~/ros2_ws/src/mycobot_calibration/scripts/calibrate_handeye.py

Keys (im Terminal):
  S  — Sample nehmen (wenn Board vollständig erkannt)
  C  — Kalibrierung berechnen (min 3 Samples, 10+ empfohlen)
  W  — Ergebnis in YAML speichern
  D  — letztes Sample löschen
  Q  — beenden

Eye-on-base: Kamera fest, ChArUco Board am Roboterende.
"""

import math
import os
import select
import sys
import termios
import threading
import time
import tty
from typing import List, Optional, Tuple

import cv2
import numpy as np
import yaml

import rclpy
from rclpy.node import Node
from std_msgs.msg import Int32
from geometry_msgs.msg import TransformStamped
import tf2_ros

ROBOT_BASE_FRAME = 'robot_base'
ROBOT_EE_FRAME = 'tcp'
CAMERA_FRAME = 'camera_color_optical_frame'
BOARD_FRAME = 'charuco_board'

# 6x3 Greifer-ChArUco-Board -> (6-1)*(3-1) = 10 innere Ecken.
# 2026-08-30: Konstanten stammten noch vom alten 5x5-Board (16/14) — damit wurde
# JEDES Sample abgelehnt, weil das 6x3-Board maximal 10 Ecken liefern kann.
TOTAL_CORNERS = 10
MIN_CORNERS_FOR_SAMPLE = 9

OUTPUT_YAML = os.path.expanduser(
    '~/.ros/easy_handeye/mycobot_handeye_eye_on_base.yaml'
)

HANDEYE_METHODS = {
    'tsai': cv2.CALIB_HAND_EYE_TSAI,
    'park': cv2.CALIB_HAND_EYE_PARK,
    'horaud': cv2.CALIB_HAND_EYE_HORAUD,
    'andreff': cv2.CALIB_HAND_EYE_ANDREFF,
    'daniilidis': cv2.CALIB_HAND_EYE_DANIILIDIS,
}


def tf_to_R_t(tf_stamped: TransformStamped) -> Tuple[np.ndarray, np.ndarray]:
    t = tf_stamped.transform.translation
    r = tf_stamped.transform.rotation
    tvec = np.array([t.x, t.y, t.z], dtype=np.float64)
    qx, qy, qz, qw = r.x, r.y, r.z, r.w
    R = np.array([
        [1 - 2*(qy*qy + qz*qz), 2*(qx*qy - qz*qw),     2*(qx*qz + qy*qw)],
        [2*(qx*qy + qz*qw),     1 - 2*(qx*qx + qz*qz), 2*(qy*qz - qx*qw)],
        [2*(qx*qz - qy*qw),     2*(qy*qz + qx*qw),     1 - 2*(qx*qx + qy*qy)],
    ], dtype=np.float64)
    return R, tvec


def R_t_to_quaternion(R: np.ndarray, t: np.ndarray) -> dict:
    trace = R[0, 0] + R[1, 1] + R[2, 2]
    if trace > 0:
        s = 0.5 / math.sqrt(trace + 1.0)
        qw = 0.25 / s
        qx = (R[2, 1] - R[1, 2]) * s
        qy = (R[0, 2] - R[2, 0]) * s
        qz = (R[1, 0] - R[0, 1]) * s
    elif R[0, 0] > R[1, 1] and R[0, 0] > R[2, 2]:
        s = 2.0 * math.sqrt(1.0 + R[0, 0] - R[1, 1] - R[2, 2])
        qw = (R[2, 1] - R[1, 2]) / s
        qx = 0.25 * s
        qy = (R[0, 1] + R[1, 0]) / s
        qz = (R[0, 2] + R[2, 0]) / s
    elif R[1, 1] > R[2, 2]:
        s = 2.0 * math.sqrt(1.0 + R[1, 1] - R[0, 0] - R[2, 2])
        qw = (R[0, 2] - R[2, 0]) / s
        qx = (R[0, 1] + R[1, 0]) / s
        qy = 0.25 * s
        qz = (R[1, 2] + R[2, 1]) / s
    else:
        s = 2.0 * math.sqrt(1.0 + R[2, 2] - R[0, 0] - R[1, 1])
        qw = (R[1, 0] - R[0, 1]) / s
        qx = (R[0, 2] + R[2, 0]) / s
        qy = (R[1, 2] + R[2, 1]) / s
        qz = 0.25 * s
    return {
        'x': float(t[0]), 'y': float(t[1]), 'z': float(t[2]),
        'qx': float(qx), 'qy': float(qy), 'qz': float(qz), 'qw': float(qw),
    }


class CalibrationSampler(Node):
    def __init__(self):
        super().__init__('handeye_sampler')

        self._tf_buffer = tf2_ros.Buffer()
        self._tf_listener = tf2_ros.TransformListener(self._tf_buffer, self)

        self._corners_count = 0
        self._corners_lock = threading.Lock()
        self.create_subscription(
            Int32, '/charuco_detector/corners_detected',
            self._on_corners, 10
        )

        self._R_g2b: List[np.ndarray] = []
        self._t_g2b: List[np.ndarray] = []
        self._R_t2c: List[np.ndarray] = []
        self._t_t2c: List[np.ndarray] = []

        self._result_R: Optional[np.ndarray] = None
        self._result_t: Optional[np.ndarray] = None

    def _on_corners(self, msg: Int32):
        with self._corners_lock:
            self._corners_count = msg.data

    @property
    def corners(self) -> int:
        with self._corners_lock:
            return self._corners_count

    def take_sample(self) -> Tuple[bool, str]:
        corners = self.corners
        if corners < MIN_CORNERS_FOR_SAMPLE:
            return False, (
                f'ABGELEHNT — Board nicht vollstaendig erkannt '
                f'({corners}/{TOTAL_CORNERS} Ecken). '
                f'Board so ausrichten, dass es der Kamera vollstaendig gezeigt wird.'
            )

        try:
            tf_board = self._tf_buffer.lookup_transform(
                CAMERA_FRAME, BOARD_FRAME,
                rclpy.time.Time(), rclpy.time.Duration(seconds=1.0)
            )
        except Exception as e:
            return False, f'ABGELEHNT — Board TF nicht erhalten: {e}'

        board_age = (
            self.get_clock().now() -
            rclpy.time.Time.from_msg(tf_board.header.stamp)
        ).nanoseconds / 1e9
        if board_age > 1.0:
            return False, (
                f'ABGELEHNT — Board TF veraltet ({board_age:.1f}s). '
                f'Board ist moeglicherweise gerade nicht in der Kamera sichtbar.'
            )

        try:
            tf_robot = self._tf_buffer.lookup_transform(
                ROBOT_EE_FRAME, ROBOT_BASE_FRAME,
                rclpy.time.Time(), rclpy.time.Duration(seconds=1.0)
            )
        except Exception as e:
            return False, f'ABGELEHNT — Roboter TF nicht erhalten: {e}'

        R_robot, t_robot = tf_to_R_t(tf_robot)
        R_board, t_board = tf_to_R_t(tf_board)

        self._R_g2b.append(R_robot)
        self._t_g2b.append(t_robot.reshape(3, 1))
        self._R_t2c.append(R_board)
        self._t_t2c.append(t_board.reshape(3, 1))

        n = len(self._R_g2b)
        return True, (
            f'GESPEICHERT — Sample #{n}  '
            f'[corners: {corners}/{TOTAL_CORNERS}]  '
            f'Robot: [{t_robot[0]:+.3f}, {t_robot[1]:+.3f}, {t_robot[2]:+.3f}]  '
            f'Board: [{t_board[0]:+.3f}, {t_board[1]:+.3f}, {t_board[2]:+.3f}]'
        )

    def delete_last(self) -> str:
        if not self._R_g2b:
            return 'Kein Sample zum Loeschen.'
        self._R_g2b.pop()
        self._t_g2b.pop()
        self._R_t2c.pop()
        self._t_t2c.pop()
        return f'Letztes Sample geloescht. Verbleibend: {len(self._R_g2b)}'

    def compute(self) -> Tuple[bool, str]:
        n = len(self._R_g2b)
        if n < 3:
            return False, f'Mindestens 3 Samples noetig, aktuell {n} vorhanden.'

        lines = [f'Kalibrierung wird berechnet ({n} Samples)...\n']
        lines.append(f'  {"Method":12s}  {"x":>8s}  {"y":>8s}  {"z":>8s}')
        lines.append(f'  {"-"*12}  {"-"*8}  {"-"*8}  {"-"*8}')

        best_method = None
        best_R = None
        best_t = None

        for name, flag in HANDEYE_METHODS.items():
            try:
                R, t = cv2.calibrateHandEye(
                    self._R_g2b, self._t_g2b,
                    self._R_t2c, self._t_t2c,
                    method=flag,
                )
                tv = t.flatten()
                lines.append(
                    f'  {name:12s}  {tv[0]:+8.4f}  {tv[1]:+8.4f}  {tv[2]:+8.4f}'
                )
                if best_method is None:
                    best_method = name
                    best_R = R
                    best_t = tv
            except Exception as e:
                lines.append(f'  {name:12s}  FEHLGESCHLAGEN — {e}')

        if best_R is not None:
            self._result_R = best_R
            self._result_t = best_t
            q = R_t_to_quaternion(best_R, best_t)
            lines.append(f'\nGewaehlte Methode: {best_method}')
            lines.append(
                f'  t=[{q["x"]:.4f}, {q["y"]:.4f}, {q["z"]:.4f}]  '
                f'q=[{q["qx"]:.4f}, {q["qy"]:.4f}, {q["qz"]:.4f}, {q["qw"]:.4f}]'
            )

        return best_R is not None, '\n'.join(lines)

    def save(self) -> Tuple[bool, str]:
        if self._result_R is None:
            return False, 'Zuerst mit C die Kalibrierung berechnen.'

        q = R_t_to_quaternion(self._result_R, self._result_t)
        data = {
            'parameters': {
                'eye_on_hand': False,
                'robot_base_frame': ROBOT_BASE_FRAME,
                'robot_effector_frame': ROBOT_EE_FRAME,
                'tracking_base_frame': CAMERA_FRAME,
                'tracking_marker_frame': BOARD_FRAME,
            },
            'transformation': {
                'x': q['x'], 'y': q['y'], 'z': q['z'],
                'qx': q['qx'], 'qy': q['qy'], 'qz': q['qz'], 'qw': q['qw'],
            },
        }

        os.makedirs(os.path.dirname(OUTPUT_YAML), exist_ok=True)
        if os.path.exists(OUTPUT_YAML):
            ts = time.strftime('%Y%m%d_%H%M%S')
            backup = OUTPUT_YAML.replace('.yaml', f'_backup_{ts}.yaml')
            os.rename(OUTPUT_YAML, backup)

        with open(OUTPUT_YAML, 'w') as f:
            yaml.dump(data, f, default_flow_style=False)

        return True, (
            f'GESPEICHERT -> {OUTPUT_YAML}\n'
            f'  t=[{q["x"]:.4f}, {q["y"]:.4f}, {q["z"]:.4f}]\n'
            f'  q=[{q["qx"]:.4f}, {q["qy"]:.4f}, {q["qz"]:.4f}, {q["qw"]:.4f}]'
        )

    @property
    def sample_count(self) -> int:
        return len(self._R_g2b)


def get_key():
    """Single keypress read (no Enter required)."""
    if select.select([sys.stdin], [], [], 0.1)[0]:
        return sys.stdin.read(1)
    return None


def main():
    rclpy.init()
    node = CalibrationSampler()

    spin_thread = threading.Thread(
        target=lambda: rclpy.spin(node), daemon=True
    )
    spin_thread.start()

    old_settings = termios.tcgetattr(sys.stdin)

    print('\n' + '=' * 50)
    print('  HAND-EYE KALIBRIERUNG  (eye-on-base)')
    print('=' * 50)
    print('  S — Sample nehmen (wenn Board vollstaendig sichtbar)')
    print('  C — Kalibrierung berechnen')
    print('  W — Ergebnis speichern')
    print('  D — letztes Sample loeschen')
    print('  Q — beenden')
    print('=' * 50)
    print()
    print('Mit der Slider-GUI den Roboter in verschiedene Posen bringen,')
    print('wenn das Board vollstaendig in der Kamera sichtbar ist, S druecken.')
    print()

    try:
        tty.setraw(sys.stdin.fileno())

        while rclpy.ok():
            key = get_key()
            if key is None:
                continue

            k = key.lower()

            if k == 's':
                ok, msg = node.take_sample()
                sym = '>>>' if ok else '!!!'
                print(f'\r{sym} {msg}\r\n', end='', flush=True)
                if ok:
                    print(
                        f'\r    Samples gesamt: {node.sample_count}\r\n',
                        end='', flush=True
                    )

            elif k == 'c':
                print(f'\r--- Wird berechnet... ---\r\n', end='', flush=True)
                ok, msg = node.compute()
                for line in msg.split('\n'):
                    print(f'\r{line}\r\n', end='', flush=True)

            elif k == 'w':
                ok, msg = node.save()
                sym = '>>>' if ok else '!!!'
                for line in msg.split('\n'):
                    print(f'\r{sym} {line}\r\n', end='', flush=True)

            elif k == 'd':
                msg = node.delete_last()
                print(f'\r--- {msg}\r\n', end='', flush=True)

            elif k == 'q':
                print(f'\r\nBeenden...\r\n', end='', flush=True)
                break

    except KeyboardInterrupt:
        pass
    finally:
        termios.tcsetattr(sys.stdin, termios.TCSADRAIN, old_settings)
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
