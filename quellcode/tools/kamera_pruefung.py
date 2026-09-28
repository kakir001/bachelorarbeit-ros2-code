#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""Kamera-Pruefungen fuer den Zyklus (zyklus_gui.py). Ein Aufruf = ein Bild, keine Bewegung.

  --foto DATEI                 aktuelles Farbbild speichern
  --trichter-referenz          Bild des LEEREN Trichters sichern (~/ros2_ws/trichter_hintergrund.npz)
  --trichter [--farbe rot]     steht eine Welle im Trichter-Rohr? ROI um den Griffpunkt `trichter_greif`
                               (FK der angelernten Gelenke, = Kopf einer stehenden Welle), Differenz zum
                               Referenzbild + Anteil der erwarteten Koerperfarbe. Exit 0 = Welle da,
                               2 = leer, 5 = unklar. --bild: Ausschnitt.
  --kopf [--farbe rot]         Welle haengt am Kopf zwischen den Fingern? (Kontrollpose + J6 90 Grad:
                               das Ende zeigt zur Kamera) ROI um den projizierten TCP: Anteil der
                               Koerperfarbe. Exit 0 = ja, 2 = nein, 5 = unklar. --bild: Ausschnitt.
Letzte Zeile: ERGEBNIS {json}.
"""
import argparse, json, math, os, sys, time
import numpy as np
import cv2
import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy
from sensor_msgs.msg import Image, CameraInfo
from moveit_msgs.srv import GetPositionFK
from moveit_msgs.msg import RobotState
import tf2_ros

ARM = ['joint2_to_joint1', 'joint3_to_joint2', 'joint4_to_joint3', 'joint5_to_joint4', 'joint6_to_joint5', 'joint6output_to_joint6']
REF = os.path.expanduser("~/ros2_ws/trichter_hintergrund.npz")
ap = argparse.ArgumentParser()
ap.add_argument("--foto", default=""); ap.add_argument("--trichter-referenz", action="store_true")
ap.add_argument("--trichter", action="store_true"); ap.add_argument("--kopf", action="store_true")
ap.add_argument("--farbe", default="", help="erwartete Koerperfarbe (schwarz gelb weiss gruen rot)")
ap.add_argument("--bild", default=""); ap.add_argument("--roi-mm", type=float, default=25.0)
ap.add_argument("--frames", type=int, default=4); ap.add_argument("--punkt", default="trichter_greif")
ap.add_argument("--datei", default=os.path.expanduser("~/ros2_ws/teach_punkte.json"))
a = ap.parse_args()


class N(Node):
    def __init__(s):
        super().__init__("kamera_pruefung"); s.imgs = []; s.K = None; s.cam_frame = None
        qos = QoSProfile(depth=5, reliability=ReliabilityPolicy.BEST_EFFORT, history=HistoryPolicy.KEEP_LAST)
        s.create_subscription(Image, "/camera/color/image_raw", s._img, qos)
        s.create_subscription(CameraInfo, "/camera/color/camera_info", s._info, qos)
        s.puffer = tf2_ros.Buffer(); tf2_ros.TransformListener(s.puffer, s)
        s.fk = s.create_client(GetPositionFK, "/compute_fk")
    def _img(s, m):
        if len(s.imgs) >= a.frames: return
        img = np.frombuffer(m.data, np.uint8).reshape(m.height, m.width, -1)[:, :, :3]
        s.imgs.append(np.ascontiguousarray(img[:, :, ::-1] if m.encoding == "rgb8" else img)); s.cam_frame = m.header.frame_id
    def _info(s, m): s.K = (m.k[0], m.k[4], m.k[2], m.k[5])


def ende(code, erg):
    print("  ERGEBNIS " + json.dumps(erg)); rclpy.shutdown(); sys.exit(code)


def klassenmaske(bgr, farbe):
    hsv = cv2.cvtColor(bgr, cv2.COLOR_BGR2HSV); H, S, V = (hsv[:, :, i].astype(np.int32) for i in range(3))
    g = cv2.cvtColor(bgr, cv2.COLOR_BGR2GRAY).astype(np.int32)
    return {"rot": ((H < 12) | (H >= 160)) & (S > 90), "gelb": (H >= 15) & (H < 34) & (S > 90), "gruen": (H >= 34) & (H < 95) & (S > 60),
            "weiss": (S < 60) & (g > 185), "schwarz": V < 70}.get(farbe, np.zeros(g.shape, bool))


rclpy.init(); n = N(); t0 = time.time()
while time.time() - t0 < 15 and (len(n.imgs) < a.frames or n.K is None): rclpy.spin_once(n, timeout_sec=0.1)
if len(n.imgs) < a.frames or n.K is None: print("!! kein Kamerabild"); ende(1, {"grund": "kein Bild"})
bild = np.median(np.stack(n.imgs), axis=0).astype(np.uint8); fx, fy, cx, cy = n.K
if a.foto: cv2.imwrite(a.foto, bild); print("  Foto:", a.foto)
if a.trichter_referenz:
    np.savez_compressed(REF, farbe=bild, zeit=time.strftime("%Y-%m-%d %H:%M")); print("  Trichter-Referenz (leer) gesichert:", REF); ende(0, {"referenz": REF})


def projiziere(p_base):
    """robot_base-Punkt [m] -> Bildpunkt"""
    t0 = time.time()
    while time.time() - t0 < 10 and not n.puffer.can_transform(n.cam_frame, "robot_base", rclpy.time.Time()): rclpy.spin_once(n, timeout_sec=0.1)
    T = n.puffer.lookup_transform(n.cam_frame, "robot_base", rclpy.time.Time()); q = T.transform.rotation
    x, y, z, w = q.x, q.y, q.z, q.w
    R = np.array([[1 - 2 * (y * y + z * z), 2 * (x * y - z * w), 2 * (x * z + y * w)], [2 * (x * y + z * w), 1 - 2 * (x * x + z * z), 2 * (y * z - x * w)], [2 * (x * z - y * w), 2 * (y * z + x * w), 1 - 2 * (x * x + y * y)]])
    t = np.array([T.transform.translation.x, T.transform.translation.y, T.transform.translation.z])
    p = R @ np.asarray(p_base) + t
    return np.array([p[0] / p[2] * fx + cx, p[1] / p[2] * fy + cy]), p[2]


def ausschnitt(mitte, r_px, marke, text):
    u, v = int(mitte[0]), int(mitte[1]); r = int(r_px * 3)
    crop = bild[max(0, v - r):v + r, max(0, u - r):u + r].copy()
    if crop.size == 0: return
    crop = cv2.resize(crop, None, fx=3, fy=3, interpolation=cv2.INTER_CUBIC)
    c = (min(u, r) * 3, min(v, r) * 3); cv2.circle(crop, c, int(r_px * 3), marke, 2)
    cv2.putText(crop, text, (8, 24), cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0, 0, 0), 4); cv2.putText(crop, text, (8, 24), cv2.FONT_HERSHEY_SIMPLEX, 0.7, (255, 255, 255), 2)
    cv2.imwrite(a.bild, crop); print("  Ausschnitt:", a.bild)


if a.trichter:
    rad = [math.radians(v) for v in json.load(open(a.datei))[a.punkt]["grad"]]
    n.fk.wait_for_service(10)
    rs = RobotState(); rs.joint_state.name = ARM; rs.joint_state.position = rad
    rq = GetPositionFK.Request(); rq.header.frame_id = "robot_base"; rq.fk_link_names = ["tcp"]; rq.robot_state = rs
    f = n.fk.call_async(rq); rclpy.spin_until_future_complete(n, f, timeout_sec=10); p = f.result().pose_stamped[0].pose.position
    mitte, tiefe = projiziere([p.x, p.y, p.z]); r_px = a.roi_mm / 1e3 * fx / tiefe
    yy, xx = np.mgrid[0:bild.shape[0], 0:bild.shape[1]]; roi = (xx - mitte[0]) ** 2 + (yy - mitte[1]) ** 2 <= r_px ** 2
    erg = {"punkt_tcp_mm": [round(p.x * 1e3), round(p.y * 1e3), round(p.z * 1e3)], "roi_px": int(roi.sum())}
    farbe_anteil = float(klassenmaske(bild, a.farbe)[roi].mean()) if a.farbe else None
    diff_anteil = None
    if os.path.exists(REF):
        ref = np.load(REF, allow_pickle=True)["farbe"]
        if ref.shape == bild.shape:
            diff = np.abs(bild.astype(np.int16) - ref.astype(np.int16)).max(axis=2) > 35; diff_anteil = float(diff[roi].mean())
    erg.update({"farbe": a.farbe or None, "farbe_anteil": None if farbe_anteil is None else round(farbe_anteil, 3),
                "diff_anteil": None if diff_anteil is None else round(diff_anteil, 3)})
    txt = "Trichter: Farbe %s %.0f %%, Differenz %s" % (a.farbe or "-", 100 * (farbe_anteil or 0), "-" if diff_anteil is None else "%.0f %%" % (100 * diff_anteil))
    print("  " + txt)
    da = (farbe_anteil is not None and farbe_anteil > 0.12) or (diff_anteil is not None and diff_anteil > 0.25)
    leer = (farbe_anteil is None or farbe_anteil < 0.04) and (diff_anteil is None or diff_anteil < 0.08)
    if a.bild: ausschnitt(mitte, r_px, (0, 255, 0) if da else (0, 0, 255), "Trichter: " + ("WELLE DA" if da else "leer" if leer else "unklar"))
    if farbe_anteil is None and diff_anteil is None: print("  !! weder Farbe noch Referenz - unklar"); ende(5, erg)
    if da: print("  -> Welle im Trichter"); ende(0, dict(erg, welle=True))
    if leer: print("  -> Trichter leer"); ende(2, dict(erg, welle=False))
    print("  -> unklar"); ende(5, dict(erg, welle=None))

if a.kopf:
    t0 = time.time()
    while time.time() - t0 < 10 and not n.puffer.can_transform("robot_base", "tcp", rclpy.time.Time()): rclpy.spin_once(n, timeout_sec=0.1)
    T = n.puffer.lookup_transform("robot_base", "tcp", rclpy.time.Time()); tt = T.transform.translation
    mitte, tiefe = projiziere([tt.x, tt.y, tt.z]); r_px = 12.0 / 1e3 * fx / tiefe
    yy, xx = np.mgrid[0:bild.shape[0], 0:bild.shape[1]]; roi = (xx - mitte[0]) ** 2 + (yy - mitte[1]) ** 2 <= r_px ** 2
    anteil = float(klassenmaske(bild, a.farbe)[roi].mean()) if a.farbe else None
    erg = {"tcp_mm": [round(tt.x * 1e3), round(tt.y * 1e3), round(tt.z * 1e3)], "farbe": a.farbe or None, "farbe_anteil": None if anteil is None else round(anteil, 3)}
    txt = "Kopf zwischen den Fingern: %s-Anteil %s" % (a.farbe or "-", "-" if anteil is None else "%.0f %%" % (100 * anteil))
    print("  " + txt)
    if a.bild: ausschnitt(mitte, r_px, (0, 255, 0) if (anteil or 0) > 0.15 else (0, 0, 255), txt)
    if anteil is None: ende(5, erg)
    if anteil > 0.15: print("  -> JA (Wellenende sichtbar)"); ende(0, dict(erg, kopf=True))
    if anteil < 0.05: print("  -> NEIN (keine Welle am TCP sichtbar)"); ende(2, dict(erg, kopf=False))
    print("  -> unklar"); ende(5, dict(erg, kopf=None))
ende(0, {"foto": a.foto})
