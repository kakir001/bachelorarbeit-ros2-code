#!/usr/bin/env python3
"""
3 Punkte ins Kamera-Bild klicken -> der hindurchgehende KREIS wird bestimmt ->
Roboter zeichnet den VIERTELKREIS-Bogen durch diese Punkte (~3cm über der
Plattform, Greifer senkrecht-nach-unten). Vorschau wird über das Bild gelegt.

Pipeline:
  3x Pixel -> aligned depth -> 3D Kamera -> TF robot_base (XY)
  -> Z konstant (Plattform+3cm) -> circumcircle fit -> Bogen-Waypoints
  -> eigene FK/IK (Greifer nach unten, an Hardware verifiziert) -> Limit/Reichweite/
     Hindernis/Höhe-Prüfung -> Bridge-Socket mit sequenziellem send_radians.

Verwendung:
  Terminal 1: ~/ros2_ws/start_mycobot.sh   (oder demo.launch use_camera:=true)
  Terminal 2: python3 .../click_arc.py

Mouse:  Links = Punkt hinzufügen (max 3)   Rechts = alle löschen
Keys :  E = Bogen zeichnen (Roboter bewegt sich)   H = home(0)   C = löschen   Q = beenden
"""

import math
import threading
import time
from typing import List, Optional, Tuple

import cv2
import numpy as np

import rclpy
from rclpy.node import Node
from rclpy.action import ActionClient
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy

from sensor_msgs.msg import Image, CameraInfo, JointState
from geometry_msgs.msg import PointStamped
from visualization_msgs.msg import Marker, MarkerArray
from control_msgs.action import FollowJointTrajectory
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint
from builtin_interfaces.msg import Duration

import tf2_ros
import tf2_geometry_msgs  # noqa: F401

CAMERA_FRAME = 'camera_color_optical_frame'
ROBOT_BASE_FRAME = 'robot_base'
ARM_ACTION = '/arm_controller/follow_joint_trajectory'

# Trajektorien-Geschwindigkeit (rad/s): freie Bewegung + langsamer Abstieg
VEL_FREE = 1.4
VEL_DESCEND = 0.4
MIN_SEG = 0.12    # minimale Dauer pro Segment (s) — verhindert künstliche Langsamkeit

ARM_JOINTS = [
    'joint2_to_joint1', 'joint3_to_joint2', 'joint4_to_joint3',
    'joint5_to_joint4', 'joint6_to_joint5', 'joint6output_to_joint6',
]

ZC = 0.045        # Bogen tcp Höhe (m) -> unterster Greifer ~3cm über Plattform
HOVER_Z = 0.12    # sichere Hover-Höhe für Abstieg/Aufstieg (m)
N_PTS = 5         # Anzahl zu sammelnder Punkte (für Verlaufs-Schätzung)
N_ARC = 18        # Anzahl Waypoints entlang des angeklickten Spans
EXTRA_FRAC = 0.30 # "Schätzung" (Extrapolation) Anteil nach dem letzten Punkt
N_EXTRA = 6       # Anzahl Waypoints im Schätz-Bereich
MIN_CLEAR = 0.030 # minimale Höhe über Plattform (m)

# ---- URDF mycobot_280_jn kinematische Kette (robot_base Frame) ----
_LIMU = np.array([2.9321, 2.4434, 2.6179, 2.6179, 2.7925, 3.14159])
_LIML = -np.array([2.9321, 2.4434, 2.6179, 2.6179, 2.7052, 3.14159])
_ORIG = [
    ((0, 0, 0.15756), (0, 0, 0)),
    ((0, 0, -0.001), (0, 1.5708, -1.5708)),
    ((-0.1104, 0, 0), (0, 0, 0)),
    ((-0.096, 0, 0.06462), (0, 0, -1.5708)),
    ((0, -0.07318, 0), (1.5708, -1.5708, 0)),
    ((0, 0.0456, 0), (-1.5708, 0, 0)),
]


def _rpy(r, p, y):
    cr, sr = np.cos(r), np.sin(r)
    cp, sp = np.cos(p), np.sin(p)
    cy, sy = np.cos(y), np.sin(y)
    return (np.array([[cy, -sy, 0], [sy, cy, 0], [0, 0, 1]])
            @ np.array([[cp, 0, sp], [0, 1, 0], [-sp, 0, cp]])
            @ np.array([[1, 0, 0], [0, cr, -sr], [0, sr, cr]]))


def _T(xyz, r, p, yw):
    M = np.eye(4)
    M[:3, :3] = _rpy(r, p, yw)
    M[:3, 3] = xyz
    return M


def _Tz(q):
    M = np.eye(4)
    c, s = np.cos(q), np.sin(q)
    M[:3, :3] = [[c, -s, 0], [s, c, 0], [0, 0, 1]]
    return M


_FGB = _T((0, 0, 0.034), 1.579, 0, 0.8406)     # joint6output->gripper_base
_FTCP = _T((0, 0.110, -0.01), 0, 0, 0)          # gripper_base->tcp


def fk(q):
    """robot_base FK -> (frame_z_Liste, gripper_base_R, tcp_xyz)."""
    M = np.eye(4)
    zs = []
    for i, (xyz, rr) in enumerate(_ORIG):
        M = M @ _T(xyz, *rr) @ _Tz(q[i])
        zs.append(M[2, 3])
    Mgb = M @ _FGB
    Mt = Mgb @ _FTCP
    zs += [Mgb[2, 3], Mt[2, 3]]
    return zs, Mgb[:3, :3], Mt[:3, 3]


def _err(q, tgt):
    _, Rgb, tcp = fk(q)
    ydir = Rgb @ np.array([0, 1.0, 0])         # gripper_base +Y -> tcp Richtung
    return np.concatenate([tcp - tgt, np.cross(ydir, [0, 0, -1.0])])


def ik_down(tgt, seed, max_iter=150):
    """IK für Greifer senkrecht-nach-unten + tcp=tgt. -> (q, residual)."""
    q = np.array(seed, float)
    for _ in range(max_iter):
        e = _err(q, tgt)
        if np.linalg.norm(e) < 1e-5:
            break
        J = np.zeros((6, 6))
        d = 1e-6
        for k in range(6):
            qq = q.copy()
            qq[k] += d
            J[:, k] = (_err(qq, tgt) - e) / d
        q = np.clip(q + np.linalg.solve(J.T @ J + 0.01 * np.eye(6), -J.T @ e),
                    _LIML, _LIMU)
    return q, float(np.linalg.norm(_err(q, tgt)))


def ik_multi(tgt, seeds, max_iter=150):
    best, bres = None, 9.9
    for s in seeds:
        q, r = ik_down(tgt, s, max_iter)
        if r < bres:
            best, bres = q, r
        if bres < 1e-4:        # gute Lösung gefunden, andere nicht testen
            break
    return best, bres


# Hindernis XY-Boxen (robot_base): column + clamp -> Abstand zur Waagerechten halten
_OBST = [
    (0.335, -0.125, 0.0125 + 0.02, 0.0175 + 0.02),   # column +Zugabe
    (0.3225, -0.125, 0.025 + 0.02, 0.055 + 0.02),    # clamp +Zugabe
]


def obstacle_hit(x, y):
    for ox, oy, hx, hy in _OBST:
        if abs(x - ox) < hx and abs(y - oy) < hy:
            return True
    return False


SEEDS = [
    [0, 0.4, -2.2, 0.3, 0, 0.0],
    [0.6, 0.5, -2.3, 0.2, 0, 0.0],
    [-0.6, 0.5, -2.3, 0.2, 0, 0.0],
    [0, 0.2, -1.8, 0.6, 0, 0.0],
]


def circumcircle(p1, p2, p3):
    """3 Punkte(XY) -> (cx, cy, R). Wenn kollinear -> None."""
    ax, ay = p1
    bx, by = p2
    cx, cy = p3
    d = 2.0 * (ax * (by - cy) + bx * (cy - ay) + cx * (ay - by))
    if abs(d) < 1e-9:
        return None
    a2, b2, c2 = ax * ax + ay * ay, bx * bx + by * by, cx * cx + cy * cy
    ux = (a2 * (by - cy) + b2 * (cy - ay) + c2 * (ay - by)) / d
    uy = (a2 * (cx - bx) + b2 * (ax - cx) + c2 * (bx - ax)) / d
    R = math.hypot(ax - ux, ay - uy)
    return ux, uy, R


def fit_circle(pts):
    """n>=3 Punkte(XY) -> beste Anpassung (least-squares) (cx, cy, R). Wenn schlecht -> None."""
    P = np.array(pts, float)
    x, y = P[:, 0], P[:, 1]
    A = np.column_stack([2 * x, 2 * y, np.ones(len(P))])
    b = x * x + y * y
    try:
        sol, *_ = np.linalg.lstsq(A, b, rcond=None)
    except Exception:
        return None
    cx, cy, c = sol
    rr = c + cx * cx + cy * cy
    if rr <= 1e-9:
        return None
    return float(cx), float(cy), float(math.sqrt(rr))


def unwrap_angles(center, pts):
    """Winkel der angeklickten Punkte um das Zentrum, als monotone (unwrap)
    Folge, die die Klick-Reihenfolge bewahrt."""
    cx, cy = center
    raw = [math.atan2(p[1] - cy, p[0] - cx) for p in pts]
    out = [raw[0]]
    for a in raw[1:]:
        d = (a - out[-1] + math.pi) % (2 * math.pi) - math.pi
        out.append(out[-1] + d)
    return out


def arc_thetas(center, p1, p2, p3, n):
    """n Winkel entlang des Bogens p1->p3, der durch p2 geht, erzeugen."""
    cx, cy = center
    t1 = math.atan2(p1[1] - cy, p1[0] - cx)
    t2 = math.atan2(p2[1] - cy, p2[0] - cx)
    t3 = math.atan2(p3[1] - cy, p3[0] - cx)
    tau = 2 * math.pi
    ccw_span = (t3 - t1) % tau
    ccw_mid = (t2 - t1) % tau
    if ccw_mid <= ccw_span:           # CCW-Richtung schließt p2 ein
        direction, span = 1.0, ccw_span
    else:
        direction, span = -1.0, tau - ccw_span
    return [t1 + direction * span * k / (n - 1) for k in range(n)]


def img_to_bgr(msg):
    arr = np.frombuffer(msg.data, dtype=np.uint8)
    if msg.encoding.lower() == 'rgb8':
        return cv2.cvtColor(arr.reshape((msg.height, msg.width, 3)),
                            cv2.COLOR_RGB2BGR)
    return arr.reshape((msg.height, msg.width, 3)).copy()


def depth_to_arr(msg):
    if msg.encoding == '16UC1':
        return np.frombuffer(msg.data, np.uint16).reshape((msg.height, msg.width))
    if msg.encoding == '32FC1':
        return np.frombuffer(msg.data, np.float32).reshape((msg.height, msg.width))
    raise ValueError(f'depth encoding: {msg.encoding}')


class ClickArc(Node):
    def __init__(self):
        super().__init__('click_arc')
        self._K = None
        self._bgr = None
        self._depth = None
        self._joints = None
        self._lock = threading.Lock()

        self._pts: List[np.ndarray] = []     # robot_base 3D (z=ZC)
        self._px: List[Tuple[int, int]] = []  # Pixel
        self._arc_base = None                # Nx3 Bogen-Punkte (base)
        self._arc_q = None                   # 6-Gelenke pro Waypoint
        self._n_through = 0                   # Anzahl Waypoints im angeklickten Span
        self._hover_first = None
        self._hover_last = None
        self._valid = False
        self._busy = False
        self._status = 'Warte auf Kamera...'

        self._tfbuf = tf2_ros.Buffer()
        self._tf = tf2_ros.TransformListener(self._tfbuf, self)
        self._mk = self.create_publisher(MarkerArray, '/click_arc/markers', 5)
        self._arm = ActionClient(self, FollowJointTrajectory, ARM_ACTION)

        qos = QoSProfile(reliability=ReliabilityPolicy.RELIABLE,
                         history=HistoryPolicy.KEEP_LAST, depth=1)
        self.create_subscription(CameraInfo, '/camera/color/camera_info',
                                 self._on_info, qos)
        self.create_subscription(Image, '/camera/color/image_raw',
                                 self._on_color, 10)
        self.create_subscription(Image,
                                 '/camera/aligned_depth_to_color/image_raw',
                                 self._on_depth, 10)
        self.create_subscription(JointState, '/joint_states',
                                 self._on_joints, 10)

    # ---- subs ----
    def _on_info(self, m):
        self._K = np.array(m.k, np.float64).reshape(3, 3)

    def _on_color(self, m):
        try:
            with self._lock:
                self._bgr = img_to_bgr(m)
                if self._status == 'Warte auf Kamera...':
                    self._status = 'Bereit — 3 Punkte anklicken'
        except Exception:
            pass

    def _on_depth(self, m):
        try:
            with self._lock:
                self._depth = depth_to_arr(m)
        except Exception:
            pass

    def _on_joints(self, m):
        self._joints = m

    def _cur_joints(self):
        if self._joints is None:
            return None
        d = dict(zip(self._joints.name, self._joints.position))
        if not all(j in d for j in ARM_JOINTS):
            return None
        return [d[j] for j in ARM_JOINTS]

    # ---- Pixel -> base ----
    def pixel_to_base(self, u, v):
        if self._K is None:
            return None
        with self._lock:
            depth = self._depth
        if depth is None:
            return None
        h, w = depth.shape
        if not (0 <= v < h and 0 <= u < w):
            return None
        reg = depth[max(0, v - 3):v + 4, max(0, u - 3):u + 4]
        valid = reg[reg > 0]
        if len(valid) == 0:
            return None
        zr = float(np.median(valid))
        z = zr / 1000.0 if zr > 100 else zr
        fx, fy = self._K[0, 0], self._K[1, 1]
        cx, cy = self._K[0, 2], self._K[1, 2]
        pc = PointStamped()
        pc.header.frame_id = CAMERA_FRAME
        pc.header.stamp = self.get_clock().now().to_msg()
        pc.point.x = (u - cx) * z / fx
        pc.point.y = (v - cy) * z / fy
        pc.point.z = z
        try:
            pb = self._tfbuf.transform(pc, ROBOT_BASE_FRAME,
                                       timeout=rclpy.time.Duration(seconds=2.0))
        except Exception as e:
            self.get_logger().warn(f'TF: {e}')
            return None
        return np.array([pb.point.x, pb.point.y, ZC])

    # ---- base -> Pixel (overlay) ----
    def _base_to_cam(self):
        if self._K is None:
            return None
        try:
            tf = self._tfbuf.lookup_transform(CAMERA_FRAME, ROBOT_BASE_FRAME,
                                              rclpy.time.Time())
        except Exception:
            return None
        q = tf.transform.rotation
        t = tf.transform.translation
        x, y, z, w = q.x, q.y, q.z, q.w
        R = np.array([
            [1 - 2 * (y * y + z * z), 2 * (x * y - z * w), 2 * (x * z + y * w)],
            [2 * (x * y + z * w), 1 - 2 * (x * x + z * z), 2 * (y * z - x * w)],
            [2 * (x * z - y * w), 2 * (y * z + x * w), 1 - 2 * (x * x + y * y)]])
        return R, np.array([t.x, t.y, t.z])

    def _proj(self, pb, R, tv):
        pc = R @ pb + tv
        if pc[2] <= 1e-3:
            return None
        fx, fy = self._K[0, 0], self._K[1, 1]
        cx, cy = self._K[0, 2], self._K[1, 2]
        return (int(round(fx * pc[0] / pc[2] + cx)),
                int(round(fy * pc[1] / pc[2] + cy)))

    # ---- click ----
    def on_click(self, u, v):
        if self._busy:
            return
        if len(self._pts) >= N_PTS:
            self._status = f'{N_PTS} Punkte voll — mit C loeschen'
            return
        pb = self.pixel_to_base(u, v)
        if pb is None:
            self._status = f'Keine Tiefe ({u},{v}) — woanders klicken'
            return
        self._pts.append(pb)
        self._px.append((u, v))
        self._status = (f'{len(self._pts)}/{N_PTS} Punkte '
                        f'[{pb[0]:.3f},{pb[1]:.3f}] r={math.hypot(pb[0],pb[1])*1000:.0f}mm'
                        + ('  (P: fertig)' if len(self._pts) >= 3 else ''))
        self.get_logger().info(self._status)
        if len(self._pts) == N_PTS:
            self.finalize()

    def finalize(self):
        """Mit >=3 Punkten Verlaufs-Schätzung berechnen."""
        if self._busy or len(self._pts) < 3:
            if len(self._pts) < 3:
                self._status = 'Mindestens 3 Punkte noetig'
            return
        self._busy = True
        threading.Thread(target=self._compute_arc, daemon=True).start()

    def clear(self):
        self._pts, self._px = [], []
        self._arc_base = self._arc_q = None
        self._n_through = 0
        self._valid = False
        self._status = f'Geloescht — {N_PTS} Punkte anklicken (P: frueh fertig)'
        self._mk.publish(MarkerArray())

    # ---- Verlaufs-Schätzung + prüfen (Hintergrund-Thread) ----
    def _wp_ok(self, wp, seed):
        """IK + Gültigkeit für einen Waypoint. -> (q, valid, Grund)."""
        if seed is None:
            q, res = ik_multi(wp, SEEDS)
        else:
            q, res = ik_down(wp, seed, max_iter=120)
            if res > 1.5e-3:
                q, res = ik_multi(wp, SEEDS + [seed])
        zs, _, _ = fk(q)
        minz = min(zs)
        inlim = bool(np.all(q >= _LIML - 1e-6) and np.all(q <= _LIMU + 1e-6))
        if res > 1.5e-3:
            return q, False, 'IK'
        if not inlim:
            return q, False, 'limit'
        if minz < MIN_CLEAR - 1e-3:
            return q, False, f'z={minz*100:.1f}cm'
        if obstacle_hit(wp[0], wp[1]):
            return q, False, 'Hindernis'
        return q, True, ''

    def _compute_arc(self):
        try:
            self._status = f'{len(self._pts)} Punkte — Verlauf wird geschaetzt (IK)...'
            p = [(pt[0], pt[1]) for pt in self._pts]
            cc = fit_circle(p)
            line_mode = cc is None or cc[2] > 1.5     # sehr großes R -> Gerade
            if not line_mode:
                cx, cy, R = cc
                ang = unwrap_angles((cx, cy), p)
                span = ang[-1] - ang[0]
                if abs(span) < 1e-3:
                    line_mode = True
            if line_mode:
                # linear: erster->letzter Punkt + Schätz-Verlängerung
                a = np.array(p[0]); b = np.array(p[-1])
                seg = b - a
                thru = [a + seg * (k / (N_ARC - 1)) for k in range(N_ARC)]
                extra = [b + seg * (EXTRA_FRAC * (k + 1) / N_EXTRA)
                         for k in range(N_EXTRA)]
                shape = f'GERADE L={np.linalg.norm(seg)*1000:.0f}mm'
            else:
                thru_a = [ang[0] + span * k / (N_ARC - 1) for k in range(N_ARC)]
                extra_a = [ang[-1] + span * EXTRA_FRAC * (k + 1) / N_EXTRA
                           for k in range(N_EXTRA)]
                thru = [np.array([cx + R * math.cos(t), cy + R * math.sin(t)])
                        for t in thru_a]
                extra = [np.array([cx + R * math.cos(t), cy + R * math.sin(t)])
                         for t in extra_a]
                shape = f'BOGEN R={R*1000:.0f}mm {math.degrees(abs(span)):.0f}°'
            n_through = len(thru)
            allxy = thru + extra
            base_all = np.array([[w[0], w[1], ZC] for w in allxy])

            qs, seed = [], None
            bad_through = None
            kept = 0
            for i, wp in enumerate(base_all):
                q, ok, why = self._wp_ok(wp, seed)
                if not ok:
                    if i < n_through:            # angeklickter Punkt nicht erreichbar
                        bad_through = (i, why)
                        break
                    else:                        # Schätz-Bereich: hier abschneiden
                        break
                seed = q
                qs.append(q)
                kept = i + 1
            if bad_through is not None or kept < n_through:
                self._valid = False
                if bad_through:
                    self._status = f'UNGUELTIG wp{bad_through[0]}:{bad_through[1]} — C loeschen'
                else:
                    self._status = 'UNGUELTIG: angeklickter Span nicht erreichbar — C loeschen'
                self._arc_base = base_all[:max(1, kept)]
                self._n_through = n_through
                self._publish_markers(self._arc_base, ok=False)
                self.get_logger().error(self._status)
                return
            # gültig: kept >= n_through (um den Schätz-Teil verlängert)
            base = base_all[:kept]
            hf, rf = ik_multi(np.array([base[0][0], base[0][1], HOVER_Z]),
                              SEEDS + [qs[0]])
            hl, rl = ik_multi(np.array([base[-1][0], base[-1][1], HOVER_Z]),
                              SEEDS + [qs[-1]])
            self._arc_base = base
            self._arc_q = qs
            self._n_through = n_through
            self._hover_first, self._hover_last = hf, hl
            self._valid = True
            self._publish_markers(base, ok=True)
            extra_kept = kept - n_through
            self._status = (f'BEREIT {shape}  Schaetzung:+{extra_kept}wp  '
                            f'z>={MIN_CLEAR*100:.0f}cm — E: zeichnen')
            self.get_logger().info(self._status)
        finally:
            self._busy = False

    def _publish_markers(self, base, ok):
        arr = MarkerArray()
        ls = Marker()
        ls.header.frame_id = ROBOT_BASE_FRAME
        ls.header.stamp = self.get_clock().now().to_msg()
        ls.ns = 'arc'
        ls.id = 0
        ls.type = Marker.LINE_STRIP
        ls.action = Marker.ADD
        ls.scale.x = 0.004
        ls.color.r = 0.0 if ok else 1.0
        ls.color.g = 1.0 if ok else 0.0
        ls.color.b = 0.0
        ls.color.a = 0.9
        ls.pose.orientation.w = 1.0
        from geometry_msgs.msg import Point
        for wp in base:
            ls.points.append(Point(x=float(wp[0]), y=float(wp[1]), z=float(wp[2])))
        arr.markers.append(ls)
        for i, pt in enumerate(self._pts):
            sp = Marker()
            sp.header.frame_id = ROBOT_BASE_FRAME
            sp.header.stamp = ls.header.stamp
            sp.ns = 'pts'
            sp.id = i + 1
            sp.type = Marker.SPHERE
            sp.action = Marker.ADD
            sp.pose.position.x = float(pt[0])
            sp.pose.position.y = float(pt[1])
            sp.pose.position.z = float(pt[2])
            sp.pose.orientation.w = 1.0
            sp.scale.x = sp.scale.y = sp.scale.z = 0.02
            sp.color.b = 1.0
            sp.color.a = 1.0
            arr.markers.append(sp)
        self._mk.publish(arr)

    # ---- execute (FollowJointTrajectory action) ----
    def execute(self):
        if not self._valid or self._arc_q is None:
            self._status = 'Erst gueltigen 3-Punkte-Bogen erstellen!'
            return
        if self._busy:
            return
        self._busy = True
        threading.Thread(target=self._exec_thread, daemon=True).start()

    @staticmethod
    def _dur(sec):
        sec = max(0.001, sec)
        return Duration(sec=int(sec), nanosec=int((sec % 1) * 1e9))

    def _build_traj(self, waypoints, vels):
        """waypoints: Liste von 6-Gelenke; vels: rad/s pro Segment.
        Erzeugt streng steigende time_from_start ab der aktuellen Pose."""
        cur = self._cur_joints() or [0.0] * 6
        traj = JointTrajectory()
        traj.joint_names = ARM_JOINTS
        t = 0.0
        prev = cur
        for wp, v in zip(waypoints, vels):
            dmax = max(abs(a - b) for a, b in zip(wp, prev))
            t += max(MIN_SEG, dmax / v)
            p = JointTrajectoryPoint()
            p.positions = [float(x) for x in wp]
            p.velocities = [0.0] * 6
            p.time_from_start = self._dur(t)
            traj.points.append(p)
            prev = wp
        return traj

    def _run_traj(self, traj, label):
        self._status = label
        self.get_logger().info(label)
        if not self._arm.wait_for_server(timeout_sec=4.0):
            self._status = 'arm_controller Action fehlt!'
            self.get_logger().error(self._status)
            return False
        goal = FollowJointTrajectory.Goal()
        goal.trajectory = traj
        fut = self._arm.send_goal_async(goal)
        # Node spinnt in separatem Thread; Futures manuell abwarten
        t0 = time.time()
        while not fut.done() and time.time() - t0 < 6.0:
            time.sleep(0.05)
        gh = fut.result()
        if gh is None or not gh.accepted:
            self._status = 'Trajektorie ABGELEHNT'
            self.get_logger().error(self._status)
            return False
        rf = gh.get_result_async()
        total = traj.points[-1].time_from_start.sec + 3.0
        t0 = time.time()
        while not rf.done() and time.time() - t0 < total + 5.0:
            time.sleep(0.1)
        return True

    def _exec_thread(self):
        try:
            # Eine Trajektorie: hover -> Abstieg -> Bogen(15) -> Aufstieg -> home
            wps = ([self._hover_first, self._arc_q[0]]
                   + list(self._arc_q) + [self._hover_last, [0.0] * 6])
            vels = ([VEL_FREE, VEL_DESCEND]
                    + [VEL_FREE] * len(self._arc_q) + [VEL_FREE, VEL_FREE])
            traj = self._build_traj(wps, vels)
            self._run_traj(traj, f'BOGEN wird gezeichnet ({len(self._arc_q)}wp) — Roboter bewegt sich...')
            self._status = 'FERTIG — Bogen gezeichnet, Roboter in Null. C: neu'
            self.get_logger().info(self._status)
        finally:
            self._busy = False

    def home(self):
        if self._busy:
            return
        self._busy = True
        try:
            traj = self._build_traj([[0.0] * 6], [VEL_FREE])
            self._run_traj(traj, 'HOME (Null)...')
        finally:
            self._busy = False

    # ---- overlay ----
    def frame(self):
        with self._lock:
            if self._bgr is None:
                return None
            f = self._bgr.copy()
        h, w = f.shape[:2]
        rt = self._base_to_cam()
        # Bogen-Polyline (back-project): angeklickter Span=grün, Schätzung=orange
        if self._arc_base is not None and rt is not None:
            R, tv = rt
            nt = self._n_through
            if not self._valid:
                pix = [p for p in (self._proj(w, R, tv) for w in self._arc_base)
                       if p is not None]
                if len(pix) >= 2:
                    cv2.polylines(f, [np.array(pix, np.int32)], False,
                                  (0, 0, 255), 2, cv2.LINE_AA)
            else:
                thru = [p for p in (self._proj(w, R, tv)
                        for w in self._arc_base[:nt]) if p is not None]
                pred = [p for p in (self._proj(w, R, tv)
                        for w in self._arc_base[max(0, nt - 1):]) if p is not None]
                if len(thru) >= 2:
                    cv2.polylines(f, [np.array(thru, np.int32)], False,
                                  (0, 220, 0), 2, cv2.LINE_AA)
                if len(pred) >= 2:
                    cv2.polylines(f, [np.array(pred, np.int32)], False,
                                  (0, 160, 255), 2, cv2.LINE_AA)
        # angeklickte Punkte
        for i, (u, v) in enumerate(self._px):
            cv2.drawMarker(f, (u, v), (255, 200, 0), cv2.MARKER_CROSS, 18, 2)
            cv2.circle(f, (u, v), 9, (255, 200, 0), 2)
            cv2.putText(f, str(i + 1), (u + 10, v - 8),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.5, (255, 200, 0), 2,
                        cv2.LINE_AA)
        cv2.rectangle(f, (0, 0), (w, 28), (40, 40, 40), -1)
        cv2.putText(f, self._status, (8, 20), cv2.FONT_HERSHEY_SIMPLEX, 0.45,
                    (255, 255, 255), 1, cv2.LINE_AA)
        cv2.rectangle(f, (0, h - 22), (w, h), (40, 40, 40), -1)
        cv2.putText(f, f'Links:Punkt({N_PTS})  P:fertig  E:zeichnen  H:home  C:loeschen  Q:beenden'
                    '   [gruen=angeklickt  orange=Schaetzung]',
                    (4, h - 6), cv2.FONT_HERSHEY_SIMPLEX, 0.4, (180, 180, 180),
                    1, cv2.LINE_AA)
        return f


def mouse_cb(event, x, y, flags, node):
    if event == cv2.EVENT_LBUTTONDOWN:
        node.on_click(x, y)
    elif event == cv2.EVENT_RBUTTONDOWN:
        node.clear()


def main():
    rclpy.init()
    node = ClickArc()
    threading.Thread(target=lambda: rclpy.spin(node), daemon=True).start()
    cv2.namedWindow('Click Arc', cv2.WINDOW_NORMAL)
    cv2.resizeWindow('Click Arc', 1280, 720)
    cv2.setMouseCallback('Click Arc', mouse_cb, node)
    print(f'\n  CLICK ARC — {N_PTS} Punkte anklicken; Verlauf wird geschaetzt; '
          'mit E zeichnen (P=frueh fertig)\n')
    try:
        while rclpy.ok():
            fr = node.frame()
            if fr is not None:
                cv2.imshow('Click Arc', fr)
            k = cv2.waitKey(50) & 0xFF
            if k in (ord('e'), ord('E')):
                node.execute()
            elif k in (ord('p'), ord('P')):
                node.finalize()
            elif k in (ord('h'), ord('H')):
                node.home()
            elif k in (ord('c'), ord('C')):
                node.clear()
            elif k in (ord('q'), ord('Q')):
                break
    except KeyboardInterrupt:
        pass
    finally:
        cv2.destroyAllWindows()
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
