#!/usr/bin/env python3
"""Probe fuer JEDE Kamerakalibrierung: kommt die Grundplatte dort heraus, wo sie steht?

WARUM ES DIESE PROBE GIBT (2026-09-10)
--------------------------------------
Die Hand-Auge-Kalibrierung vom 2026-09-09 sah in sich stimmig aus - Park und Horaud
waren sich auf 0.1 mm einig - und war trotzdem falsch. Durch sie gerechnet lag die
Oberflaeche der Grundplatte 4.77 Grad gekippt und 21.6 mm zu hoch. Niemand hat es
bemerkt, weil nichts danach gefragt hat: eine Kalibrierung, die nur gegen sich selbst
geprueft wird, kann beliebig daneben liegen, ohne dass eine Meldung erscheint.

Diese Probe fragt danach. Sie braucht nichts als die laufende Kamera und die TF:

  * Die Platte ist der einzige grosse, ebene, ORTSFESTE Koerper im Bild, dessen Lage
    das Modell schon kennt: das URDF setzt ihre Oberflaeche auf z=0 in robot_base, und
    der Roboter ist auf genau diese Platte geschraubt. Er kann ihr gegenueber nicht
    verkippt stehen. Kommt die Platte gekippt heraus, ist die KALIBRIERUNG gekippt.
  * Damit die Probe nicht die Kamera selbst beschuldigt, wenn die Tiefendaten schlecht
    sind, wird die Ebene ZUERST im Kamera-Frame gefittet. Ist sie dort schon unruhig
    (rms gross), sagt die Probe das und faellt kein Urteil ueber die Kalibrierung.

Was sie NICHT kann: eine plattenbasierte Kalibrierung bestaetigen. Die macht die Platte
per Konstruktion eben - fuer sie ist diese Probe nur eine Rechenkontrolle. Ihr Wert
liegt beim Gegenteil: sie faengt eine Kalibrierung ab, die die Platte VERFEHLT.

Aufruf (Kamera muss laufen, aligned_depth an):
    python3 tools/pruefe_kalibrierung_platte.py
    python3 tools/pruefe_kalibrierung_platte.py --bereich -0.07 0.37 -0.30 0.05
"""
import argparse
import sys

import numpy as np
import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy
from sensor_msgs.msg import Image, CameraInfo
import tf2_ros

# Grenzen, ab denen die Probe durchfaellt. Bezug: das Modell selbst ist auf 3.28 mm /
# 0.42 Grad genau (measure_model_vs_charuco, 2026-09-09) - schaerfer zu fordern waere
# sinnlos. Der abgewiesene Fall lag bei 4.77 Grad / 21.6 mm, also weit darueber.
MAX_WINKEL_GRAD = 1.5
MAX_VERSATZ_MM = 8.0
MAX_RMS_MM = 5.0          # darueber sind die Tiefendaten selbst zu unruhig fuer ein Urteil
MIN_ANTEIL = 0.60         # so viel der Flaeche muss wirklich Platte sein

ap = argparse.ArgumentParser()
ap.add_argument("--depth", default="/camera/aligned_depth_to_color/image_raw")
ap.add_argument("--info", default="/camera/color/camera_info")
ap.add_argument("--frame", default="robot_base")
ap.add_argument("--bereich", type=float, nargs=4, default=(-0.07, 0.37, -0.30, 0.05),
                metavar=("XMIN", "XMAX", "YMIN", "YMAX"),
                help="Plattenflaeche in robot_base [m], innerhalb der Plattenkante")
ap.add_argument("--rmin", type=float, default=0.14,
                help="alles naeher am Roboter wird verworfen (eigener Fuss) [m]")
ap.add_argument("--mittelung", type=int, default=8)
a = ap.parse_args()


def qrot(q, p):
    u = np.asarray(q[:3], dtype=np.float64)
    w = float(q[3])
    return p + 2.0 * w * np.cross(u, p) + 2.0 * np.cross(u, np.cross(u, p))


def ebene_robust(P, band=0.005, versuche=300, seed=12345):
    """Groesste Ebene in der Punktwolke (RANSAC), danach saubere Ausgleichsrechnung.

    Rueckgabe: (Koeffizienten a,b,c fuer z = a*x + b*y + c, rms im Band, Anteil im Band).

    WARUM RANSAC UND NICHT ETWAS SELBSTGEBAUTES (zwei Fehlschlaege am 2026-09-10):
      * Gewoehnliche kleinste Quadrate ueber alles: was auf der Platte steht - Schale,
        Kabel, Fuss - liegt IMMER DARUEBER, nie darunter. Diese einseitige Stoerung
        kippt die Ebene zu sich hin, und zwar je nachdem, wo das Zeug gerade liegt.
        Zwei Messungen desselben Aufbaus ergaben -10.0 und -29.1 mm/m in y.
      * "Untere Huelle" (rundenweise alles ueber der Ebene wegwerfen): laeuft nach
        unten weg. Das Ergebnis war zwar auf 0.07 Grad wiederholbar, aber falsch -
        nur noch 50 Prozent der Punkte lagen in der gefundenen Ebene.
    RANSAC sucht dagegen die Ebene mit den MEISTEN Punkten. Die Platte ist die weitaus
    groesste zusammenhaengende Flaeche im Bild, also gewinnt sie - egal was darauf
    steht und wo. Der gemeldete ANTEIL zeigt, ob das gestimmt hat: liegt er hoch, war
    die gefundene Ebene wirklich die Platte.
    """
    rng = np.random.default_rng(seed)
    n = len(P)
    bester, beste_zahl = None, -1
    for _ in range(versuche):
        idx = rng.choice(n, 3, replace=False)
        p0, p1, p2 = P[idx]
        nv = np.cross(p1 - p0, p2 - p0)
        if abs(nv[2]) < 1e-6:
            continue
        koef = np.array([-nv[0] / nv[2], -nv[1] / nv[2],
                         (nv @ p0) / nv[2]])
        rest = P[:, 2] - (koef[0] * P[:, 0] + koef[1] * P[:, 1] + koef[2])
        zahl = int(np.count_nonzero(np.abs(rest) < band))
        if zahl > beste_zahl:
            beste_zahl, bester = zahl, koef
    A = np.c_[P[:, 0], P[:, 1], np.ones(n)]
    koef = bester
    for _ in range(3):                       # auf den Treffern sauber ausgleichen
        rest = P[:, 2] - A @ koef
        drin = np.abs(rest) < band
        if drin.sum() < 500:
            break
        koef, *_ = np.linalg.lstsq(A[drin], P[drin, 2], rcond=None)
    rest = P[:, 2] - A @ koef
    drin = np.abs(rest) < band
    return koef, float(np.std(rest[drin])), float(drin.mean())


class Knoten(Node):
    def __init__(self):
        super().__init__('pruefe_kalibrierung_platte')
        qos = QoSProfile(depth=5, reliability=ReliabilityPolicy.BEST_EFFORT,
                         history=HistoryPolicy.KEEP_LAST)
        self.bilder = []
        self.info = None
        self.depth_frame = None
        self.create_subscription(Image, a.depth, self._on_depth, qos)
        self.create_subscription(CameraInfo, a.info, self._on_info, qos)
        self.puffer = tf2_ros.Buffer()
        tf2_ros.TransformListener(self.puffer, self)

    def _on_depth(self, msg):
        bild = np.frombuffer(msg.data, np.uint16).reshape(msg.height, msg.width).astype(np.float32)
        bild[bild == 0] = np.nan
        self.bilder.append(bild * 0.001)
        self.depth_frame = msg.header.frame_id

    def _on_info(self, msg):
        self.info = msg


def main():
    rclpy.init()
    k = Knoten()
    print("warte auf Tiefenbild (%s) ..." % a.depth)
    for _ in range(900):
        rclpy.spin_once(k, timeout_sec=0.05)
        if (len(k.bilder) >= a.mittelung and k.info is not None and k.depth_frame
                and k.puffer.can_transform(a.frame, k.depth_frame, rclpy.time.Time())):
            break
    else:
        print("ABBRUCH: kein Tiefenbild / keine TF. Laeuft die Kamera? Laeuft der Stack?")
        return 2

    tiefe = np.nanmedian(np.stack(k.bilder[:a.mittelung]), axis=0)
    m = k.info.k
    fx, fy, cx, cy = m[0], m[4], m[2], m[5]
    h, w = tiefe.shape
    vs, us = np.mgrid[0:h, 0:w]
    gut = np.isfinite(tiefe) & (tiefe > 0.15) & (tiefe < 2.0)
    u, v, z = us[gut].astype(np.float64), vs[gut].astype(np.float64), tiefe[gut].astype(np.float64)
    C = np.stack([(u - cx) * z / fx, (v - cy) * z / fy, z], axis=1)

    tf = k.puffer.lookup_transform(a.frame, k.depth_frame, rclpy.time.Time())
    q = tf.transform.rotation
    t = tf.transform.translation
    P = qrot((q.x, q.y, q.z, q.w), C) + np.array([t.x, t.y, t.z])

    x0, x1, y0, y1 = a.bereich
    auf_platte = ((P[:, 0] > x0) & (P[:, 0] < x1) & (P[:, 1] > y0) & (P[:, 1] < y1)
                  & (np.abs(P[:, 2]) < 0.12) & (np.hypot(P[:, 0], P[:, 1]) > a.rmin))
    if auf_platte.sum() < 2000:
        print("ABBRUCH: nur %d Punkte auf der Platte. Steht etwas davor, oder stimmt "
              "--bereich nicht?" % int(auf_platte.sum()))
        return 2

    # 1) Erst im KAMERA-Frame: sind die Tiefendaten ueberhaupt eben?
    _, rms_cam, anteil_cam = ebene_robust(C[auf_platte])
    # 2) Dann in robot_base: liegt die Ebene dort, wo das Modell sie hat?
    koef, rms_base, anteil = ebene_robust(P[auf_platte])
    A_p = np.c_[P[auf_platte][:, 0], P[auf_platte][:, 1], np.ones(int(auf_platte.sum()))]
    Q = P[auf_platte][np.abs(P[auf_platte][:, 2] - A_p @ koef) < 0.010]
    a_, b_, c_ = koef
    normale = np.array([-a_, -b_, 1.0])
    normale /= np.linalg.norm(normale)
    winkel = float(np.degrees(np.arccos(np.clip(abs(normale[2]), -1.0, 1.0))))
    hoehe_mitte = float(np.median(Q[:, 2]) * 1e3)

    print()
    print("=" * 70)
    print("PLATTENPROBE")
    print("=" * 70)
    print("Punkte auf der Platte     : %d davon %d in der Ebene (%.0f%%)"
          % (int(auf_platte.sum()), len(Q), anteil * 100))
    print("Ebenheit im Kamera-Frame  : %.2f mm rms ueber %.0f%% der Flaeche  -> Tiefendaten %s"
          % (rms_cam * 1e3, anteil_cam * 100,
             "in Ordnung" if (rms_cam * 1e3 <= MAX_RMS_MM and anteil_cam >= MIN_ANTEIL)
             else "ZU UNRUHIG"))
    print("-" * 70)
    print("Neigung gegen z=0         : %.2f Grad      (erlaubt %.2f)" % (winkel, MAX_WINKEL_GRAD))
    print("   in x                   : %+.1f mm/m" % (a_ * 1e3))
    print("   in y                   : %+.1f mm/m" % (b_ * 1e3))
    print("Hoehe der Plattenflaeche  : %+.1f mm       (soll 0, erlaubt +/-%.1f)"
          % (hoehe_mitte, MAX_VERSATZ_MM))
    print("So weit liegt die Platte daneben:")
    for xx in (0.05, 0.10, 0.20, 0.30):
        print("   bei x=%.2f m, y=-0.10 m : %+6.1f mm" % (xx, (a_ * xx + b_ * -0.10 + c_) * 1e3))
    print("-" * 70)

    if rms_cam * 1e3 > MAX_RMS_MM or anteil_cam < MIN_ANTEIL:
        print("KEIN URTEIL: die Tiefendaten sind schon im Kamera-Frame zu unruhig "
              "(%.2f mm rms auf %.0f%% der Flaeche). Steht zu viel auf der Platte, oder "
              "ist die Kamera/Szene nicht in Ordnung?" % (rms_cam * 1e3, anteil_cam * 100))
        return 2

    schlecht = []
    if winkel > MAX_WINKEL_GRAD:
        schlecht.append("Neigung %.2f Grad (> %.2f)" % (winkel, MAX_WINKEL_GRAD))
    if abs(hoehe_mitte) > MAX_VERSATZ_MM:
        schlecht.append("Versatz %+.1f mm (> %.1f)" % (hoehe_mitte, MAX_VERSATZ_MM))

    if schlecht:
        print("DURCHGEFALLEN: " + " und ".join(schlecht))
        print("Die Platte ist ortsfest und der Roboter ist auf sie geschraubt - sie kann")
        print("nicht schief stehen. Der Fehler liegt in der Kalibrierung, nicht im Aufbau.")
        print("Diese Kalibrierung NICHT verwenden. Siehe")
        print("   calibration_backups/2026-09-10_handauge_zurueckgenommen/WARUM.txt")
        return 1

    print("BESTANDEN: die Platte kommt dort heraus, wo das Modell sie hat.")
    print("(Eine plattenbasierte Kalibrierung besteht diese Probe per Konstruktion -")
    print(" fuer sie ist das nur eine Rechenkontrolle, keine Bestaetigung.)")
    return 0


if __name__ == '__main__':
    try:
        code = main()
    finally:
        try:
            rclpy.shutdown()
        except Exception:
            pass
    sys.exit(code)
