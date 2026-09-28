#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""Sicherheitsstufe nach dem Griff (Benutzerwunsch 2026-09-14): sitzt der KOPF der Welle auf der
richtigen Greiferseite? Kamera-Kontrolle, sonst Rueckfrage.

Warum: der Trichter-Ablagepunkt wurde mit einer bestimmten Wellenrichtung angelernt; greift der
Arm die Welle verkehrt herum (Kopfende aus dem Bild falsch bestimmt, 13.9. Nacht: Welle fiel auf
den Trichterrand), zeigt der Kopf am Trichter in die falsche Richtung.

Konvention (hole_aus_schale.quat_senkrecht): tcp +Z = Wellenachse Kopf -> Spitze, d.h. der KOPF
liegt auf der -Z-Seite des Werkzeugs, die Spitze auf +Z. Der Griff sitzt am Flansch (20.5 mm vom
Kopfende): Kopf ragt 21.5 mm auf der -Z-Seite heraus, Schaft 22 mm auf der +Z-Seite.

Kamera-Kriterium: die beiden herausragenden Enden werden entlang der Achse abgetastet
(|s| = --von .. --bis mm vom TCP, ausserhalb der Finger). Der Kopf ist Ø 10, der Schaft Ø 7 ->
die BREITERE Seite ist der Kopf (bei 0.5 mm/px ein Unterschied von ~5 px). Zusaetzlich: Ringe der
Streifenfarbe liegen 9-22 mm vom Kopfende, also groesstenteils unter den Fingern, ggf. am Rand.
Ist die Welle nicht sichtbar (Greifergehaeuse verdeckt sie, Breiten nicht messbar) -> Exit 5 =
Benutzer fragen.

    python3 tools/kopfseite_pruefen.py                 # Text + Exit-Code
    python3 tools/kopfseite_pruefen.py --bild k.png    # mit Overlay
Exit: 0 Kopf auf -Z (RICHTIG), 2 Kopf auf +Z (VERKEHRT -> J6 um 180 Grad drehen), 5 nicht
entscheidbar (Benutzer fragen), 1 Fehler. Letzte Zeile: ERGEBNIS {json}.
"""
import argparse, json, math, sys, time
import numpy as np
import cv2
import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy
from sensor_msgs.msg import Image, CameraInfo
import tf2_ros

ap = argparse.ArgumentParser()
ap.add_argument("--von", type=float, default=11.0, help="[mm] Abtastung ab hier vom TCP (Fingerbreite/2 + 1)")
ap.add_argument("--bis", type=float, default=21.0, help="[mm] bis hier (Kopf ragt 21.5 heraus)")
ap.add_argument("--z-tcp", type=float, default=None, help="[mm] Hoehe der Wellenachse ueber dem TCP-Ursprung (Vorgabe 0)")
ap.add_argument("--min-diff-px", type=float, default=2.0, help="Breitenunterschied [px], ab dem entschieden wird")
ap.add_argument("--frames", type=int, default=5)
ap.add_argument("--bild", default="")
ap.add_argument("--frame", default="robot_base")
a = ap.parse_args()


class N(Node):
    def __init__(s):
        super().__init__("kopfseite_pruefen"); s.imgs = []; s.K = None; s.cam_frame = None
        qos = QoSProfile(depth=5, reliability=ReliabilityPolicy.BEST_EFFORT, history=HistoryPolicy.KEEP_LAST)
        s.create_subscription(Image, "/camera/color/image_raw", s._img, qos)
        s.create_subscription(CameraInfo, "/camera/color/camera_info", s._info, qos)
        s.puffer = tf2_ros.Buffer(); tf2_ros.TransformListener(s.puffer, s)
    def _img(s, m):
        if len(s.imgs) >= a.frames: return
        img = np.frombuffer(m.data, np.uint8).reshape(m.height, m.width, -1)[:, :, :3]
        s.imgs.append(np.ascontiguousarray(img[:, :, ::-1] if m.encoding == "rgb8" else img)); s.cam_frame = m.header.frame_id
    def _info(s, m): s.K = (m.k[0], m.k[4], m.k[2], m.k[5])


def qmat(q):
    x, y, z, w = q
    return np.array([[1 - 2 * (y * y + z * z), 2 * (x * y - z * w), 2 * (x * z + y * w)],
                     [2 * (x * y + z * w), 1 - 2 * (x * x + z * z), 2 * (y * z - x * w)],
                     [2 * (x * z - y * w), 2 * (y * z + x * w), 1 - 2 * (x * x + y * y)]])


def ende(code, erg):
    print("  ERGEBNIS " + json.dumps(erg)); rclpy.shutdown(); sys.exit(code)


rclpy.init(); n = N()
t0 = time.time()
while time.time() - t0 < 15 and (len(n.imgs) < a.frames or n.K is None): rclpy.spin_once(n, timeout_sec=0.1)
if len(n.imgs) < a.frames or n.K is None: print("!! kein Kamerabild / camera_info"); ende(1, {"kopfseite": None, "grund": "kein Bild"})
bild = np.median(np.stack(n.imgs), axis=0).astype(np.uint8)
t0 = time.time()
while time.time() - t0 < 10 and not (n.puffer.can_transform(n.cam_frame, "tcp", rclpy.time.Time()) and n.puffer.can_transform(a.frame, "tcp", rclpy.time.Time())):
    rclpy.spin_once(n, timeout_sec=0.1)
try:
    T = n.puffer.lookup_transform(n.cam_frame, "tcp", rclpy.time.Time())        # Kamera <- tcp
except Exception as e: print("!! TF Kamera<-tcp fehlt:", e); ende(1, {"kopfseite": None, "grund": "TF"})
R = qmat((T.transform.rotation.x, T.transform.rotation.y, T.transform.rotation.z, T.transform.rotation.w))
t = np.array([T.transform.translation.x, T.transform.translation.y, T.transform.translation.z])
fx, fy, cx, cy = n.K
ez_cam = R[:, 2]                                   # Wellenachse (tcp +Z) im Kamerabild
ex_cam = R[:, 0]                                   # Schliessachse quer (tcp +X)

def px(p_tcp):
    p = R @ np.asarray(p_tcp) + t
    if p[2] <= 0.05: return None
    return np.array([p[0] / p[2] * fx + cx, p[1] / p[2] * fy + cy])

# Abtastung: fuer s in [-bis..-von] und [von..bis] entlang +Z die Breite quer (entlang +X) messen.
# Breite = Zahl der Bildpunkte quer, die sich vom lokalen Hintergrund (Median 6-9 mm weiter aussen)
# deutlich unterscheiden. Bildkoordinaten ueber die Kamera-Projektion (Welle ~ 60 mm ueber der Schale).
grau = cv2.cvtColor(bild, cv2.COLOR_BGR2GRAY).astype(np.float32)
hsv = cv2.cvtColor(bild, cv2.COLOR_BGR2HSV).astype(np.float32)
ueber = np.zeros(bild.shape[:2], np.uint8)

def wert(u, v):
    u, v = int(round(u)), int(round(v))
    if not (0 <= u < bild.shape[1] and 0 <= v < bild.shape[0]): return None
    return grau[v, u], hsv[v, u]

def breite_bei(s_mm, seite):
    """Breite [px] und Farbe an Achsposition s (mm, Vorzeichen = Seite). None = nicht messbar."""
    mitte = px([0.0, 0.0, s_mm / 1e3])
    if mitte is None: return None
    q1 = px([0.006, 0.0, s_mm / 1e3]); q0 = px([-0.006, 0.0, s_mm / 1e3])
    if q1 is None or q0 is None: return None
    quer = (q1 - q0) / 12.0                         # Bildvektor je mm quer
    # Hintergrund: 9..13 mm quer beidseitig
    hg = [wert(*(mitte + quer * d)) for d in list(range(-13, -8)) + list(range(9, 14))]
    hg = [h for h in hg if h is not None]
    if len(hg) < 6: return None
    g_hg = np.median([h[0] for h in hg]); s_hg = np.median([h[1][1] for h in hg])
    drin = []
    for d10 in range(-90, 91):                      # 0.1-mm-Schritte quer, +-9 mm
        w_ = wert(*(mitte + quer * (d10 / 10.0)))
        if w_ is None: drin.append(False); continue
        drin.append(abs(w_[0] - g_hg) > 25 or abs(w_[1][1] - s_hg) > 45)
        if drin[-1]:
            u, v = (mitte + quer * (d10 / 10.0)).round().astype(int); ueber[v, u] = 255
    drin = np.array(drin)
    # laengster zusammenhaengender Block um die Mitte
    if not drin[80:101].any(): return 0.0
    i0 = 90
    while i0 > 0 and drin[i0 - 1]: i0 -= 1
    i1 = 90
    while i1 < 180 and drin[i1 + 1]: i1 += 1
    return (i1 - i0 + 1) / 10.0                     # mm

kopf_seite, spitze_seite = [], []
for s in np.arange(a.von, a.bis + 0.01, 1.0):
    bk = breite_bei(-s, "-Z"); bs = breite_bei(+s, "+Z")
    if bk is not None: kopf_seite.append(bk)
    if bs is not None: spitze_seite.append(bs)
erg = {"kopfseite": None, "breite_minusZ_mm": None, "breite_plusZ_mm": None, "n": [len(kopf_seite), len(spitze_seite)]}
if len(kopf_seite) < 4 or len(spitze_seite) < 4:
    print("!! Welle im Bild nicht messbar (Greifer verdeckt sie?) - Benutzer fragen"); ende(5, dict(erg, grund="nicht sichtbar"))
bm, bp = float(np.median(kopf_seite)), float(np.median(spitze_seite))
erg["breite_minusZ_mm"], erg["breite_plusZ_mm"] = round(bm, 1), round(bp, 1)
print("  Breite der herausragenden Enden: -Z-Seite %.1f mm, +Z-Seite %.1f mm (Kopf Ø 10, Schaft Ø 7)" % (bm, bp))
if a.bild:
    out = bild.copy(); out[ueber > 0] = (0, 255, 255)
    for s, col in ((-a.bis, (255, 0, 255)), (a.bis, (0, 255, 0))):
        p = px([0.0, 0.0, s / 1e3])
        if p is not None: cv2.circle(out, tuple(p.round().astype(int)), 5, col, 2)
    p0 = px([0.0, 0.0, 0.0])
    if p0 is not None:
        cv2.circle(out, tuple(p0.round().astype(int)), 4, (255, 255, 255), -1)
        x0, y0 = p0.round().astype(int); crop = out[max(0, y0 - 120):y0 + 120, max(0, x0 - 160):x0 + 160]
        crop = cv2.resize(crop, None, fx=3, fy=3, interpolation=cv2.INTER_CUBIC)
        cv2.putText(crop, "magenta = -Z (Kopf erwartet), gruen = +Z", (8, 24), cv2.FONT_HERSHEY_SIMPLEX, 0.6, (255, 255, 255), 2)
        cv2.imwrite(a.bild, crop); print("  Overlay:", a.bild)
if bm < 3.0 and bp < 3.0: print("!! beide Seiten leer - Welle nicht sichtbar oder nicht im Greifer"); ende(5, dict(erg, grund="leer"))
diff_px = (bm - bp) / (12.0 / np.linalg.norm(px([0.006, 0, 0]) - px([-0.006, 0, 0])))   # mm -> px
if abs(bm - bp) * 2 < a.min_diff_px:              # ~0.5 mm/px
    print("?? Breiten zu aehnlich (%.1f / %.1f mm) - Benutzer fragen" % (bm, bp)); ende(5, dict(erg, grund="unklar"))
if bm > bp:
    erg["kopfseite"] = "-Z"; print("  KOPF auf der -Z-Seite = RICHTIG (wie angelernt)"); ende(0, erg)
erg["kopfseite"] = "+Z"; print("  KOPF auf der +Z-Seite = VERKEHRT -> Greifer (J6) um 180 Grad drehen"); ende(2, erg)
