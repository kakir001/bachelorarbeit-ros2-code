#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""Punkt im Kamerabild anklicken -> robot_base-Koordinate -> Marker in RViz -> hinfahren.

Wozu (2026-09-11, Benutzer): "Beschreiben, wohin der Arm soll, ist muehsam. Ich
klicke die Stelle im Kamerabild an, du rechnest die Koordinate, zeigst sie in
RViz, faehrst hin; dann Feinabgleich und speichern." Genau das. Die Kamera ist
gegen robot_base plattenbasiert kalibriert (2.5 mm, tools/pruefe_kalibrierung_platte.py),
das ist besser als jedes Lineal vom Robotersockel aus.

Rechnung wie im wellen_detektor_node: Pixel + aligned depth -> optischer Frame
(fx, fy, cx, cy aus camera_info) -> TF -> robot_base. Tiefe = Median 5x5 um den Klick.

Tasten im Fenster:
    Linksklick   Punkt setzen (Koordinate im Terminal + Marker /punkt_klicken/marker)
    f            zum Punkt fahren: senkrecht darueber auf --hover [mm], Gierwinkel --yaw
                 (tools/zeige_punkt.py, MoveIt, kollisionsgeprueft)
    s            Punkt in --datei unter dem naechsten Namen sichern (nur xy aus der Kamera)
    q            Ende

    python3 tools/punkt_klicken.py --hover 120

In RViz: Add -> Marker, Topic /punkt_klicken/marker (gruene Kugel an der Stelle).
"""
import argparse, json, math, os, subprocess, sys, time
import numpy as np
import cv2
import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data, QoSProfile, DurabilityPolicy
from sensor_msgs.msg import Image, CameraInfo
from visualization_msgs.msg import Marker
import tf2_ros

ap = argparse.ArgumentParser()
ap.add_argument("--hover", type=float, default=120.0, help="[mm] Anfahrhoehe ueber robot_base")
ap.add_argument("--yaw", type=float, default=180.0)
ap.add_argument("--datei", default=os.path.expanduser("~/ros2_ws/klick_punkte.json"))
ap.add_argument("--base", default="robot_base")
ap.add_argument("--ebene-z", type=float, default=None,
                help="[mm] bekannte Hoehe der angeklickten Stelle ueber robot_base: der Sehstrahl wird "
                     "mit dieser Ebene geschnitten, die Tiefe wird nicht gebraucht. Wellenkopf im "
                     "Trichter 42, Nest der Ablageplatte 7. Ohne Angabe: Tiefe = naechster Punkt im "
                     "7x7-Fenster (10. Perzentil) - der Median las am 2026-09-11 bei der Ø-7-Welle "
                     "den Hintergrund (z 12 statt 42).")
a = ap.parse_args()


def image_to_bgr(msg):
    buf = np.frombuffer(msg.data, dtype=np.uint8).reshape(msg.height, msg.width, 3)
    return np.ascontiguousarray(buf[:, :, ::-1] if msg.encoding.lower() == "rgb8" else buf)


def qrot(q, v):
    x, y, z, w = q
    t = 2.0 * np.cross((x, y, z), v)
    return np.asarray(v) + w * t + np.cross((x, y, z), t)


class N(Node):
    def __init__(s):
        super().__init__("punkt_klicken")
        s.img = None; s.depth = None; s.K = None; s.frame = None
        s.create_subscription(Image, "/camera/color/image_raw", s._img, qos_profile_sensor_data)
        s.create_subscription(Image, "/camera/aligned_depth_to_color/image_raw", s._dep, qos_profile_sensor_data)
        s.create_subscription(CameraInfo, "/camera/color/camera_info", s._info, qos_profile_sensor_data)
        s.pub = s.create_publisher(Marker, "/punkt_klicken/marker", QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL))
        s.buf = tf2_ros.Buffer(); tf2_ros.TransformListener(s.buf, s)
        s.punkt = None; s.klick_px = None; s.maus = None

    def _img(s, m): s.img = image_to_bgr(m); s.frame = m.header.frame_id
    def _dep(s, m): s.depth = np.frombuffer(m.data, dtype=np.uint16).reshape(m.height, m.width)
    def _info(s, m): s.K = (m.k[0], m.k[4], m.k[2], m.k[5])

    def klick(s, u, v):
        if s.depth is None or s.K is None: print("  !! noch kein Tiefenbild / camera_info"); return
        h, w = s.depth.shape
        u = min(max(u, 3), w - 4); v = min(max(v, 3), h - 4)
        fen = s.depth[v-3:v+4, u-3:u+4].astype(float); fen = fen[fen > 0]
        try:
            tf = s.buf.lookup_transform(a.base, s.frame, rclpy.time.Time())
        except Exception as e:
            print(f"  !! kein TF {s.frame}->{a.base}: {e}"); return
        t = np.array((tf.transform.translation.x, tf.transform.translation.y, tf.transform.translation.z))
        r = tf.transform.rotation; q = (r.x, r.y, r.z, r.w)
        fx, fy, cx, cy = s.K
        richtung = qrot(q, ((u - cx) / fx, (v - cy) / fy, 1.0))   # Sehstrahl in robot_base, je 1 m Tiefe
        ergebnis = {}
        if fen.size:
            z = float(np.percentile(fen, 10)) / 1000.0            # naechster Punkt = Oberkante
            ergebnis["Tiefe"] = (t + richtung * z, f"Tiefe {z*1000:.0f} mm")
        if a.ebene_z is not None and abs(richtung[2]) > 1e-6:
            sz = (a.ebene_z / 1000.0 - t[2]) / richtung[2]
            ergebnis["Ebene"] = (t + richtung * sz, f"Ebene z={a.ebene_z:.0f} mm")
        if not ergebnis: print("  !! keine Tiefe an dieser Stelle"); return
        for art, (p, info) in ergebnis.items():
            az = math.degrees(math.atan2(p[1], p[0])) % 360; rr = math.hypot(p[0], p[1]) * 1000
            print(f"  Punkt [{art}]: x {p[0]*1000:+.1f}  y {p[1]*1000:+.1f}  z {p[2]*1000:+.1f} mm"
                  f"   (Azimut {az:.1f} Grad, r {rr:.0f} mm, {info}, Pixel {u},{v})")
        p = ergebnis["Ebene"][0] if "Ebene" in ergebnis else ergebnis["Tiefe"][0]
        s.punkt = [float(x) * 1000.0 for x in p]; s.klick_px = (u, v)
        mk = Marker(); mk.header.frame_id = a.base; mk.header.stamp = s.get_clock().now().to_msg()
        mk.ns = "klick"; mk.id = 0; mk.type = Marker.SPHERE; mk.action = Marker.ADD
        mk.pose.position.x, mk.pose.position.y, mk.pose.position.z = p
        mk.pose.orientation.w = 1.0
        mk.scale.x = mk.scale.y = mk.scale.z = 0.008
        mk.color.g = 1.0; mk.color.a = 0.9
        s.pub.publish(mk)


rclpy.init(); n = N()
WIN = "Punkt klicken  [Linksklick=Punkt  f=fahren  s=sichern  q=Ende]"
cv2.namedWindow(WIN, cv2.WINDOW_NORMAL)
def _maus(ev, x, y, fl, prm):
    n.maus = (x, y)
    if ev == cv2.EVENT_LBUTTONDOWN: n.klick(x, y)
cv2.setMouseCallback(WIN, _maus)
LUPE = 4          # Vergroesserung der Lupe (Bild ist nur 424x240, am Schirm auf Vollbild gezogen)
LUPE_R = 30       # Halbseite des Ausschnitts [px]
print("  warte auf Kamerabild ...")
while rclpy.ok():
    rclpy.spin_once(n, timeout_sec=0.03)
    if n.img is None: continue
    im = n.img.copy()
    if n.maus:                                    # Lupe: Ausschnitt um den Mauszeiger, oben rechts
        mx, my = n.maus; H, W = im.shape[:2]
        x0, y0 = max(0, mx - LUPE_R), max(0, my - LUPE_R)
        x1, y1 = min(W, mx + LUPE_R), min(H, my + LUPE_R)
        aus = im[y0:y1, x0:x1]
        if aus.size:
            gross = cv2.resize(aus, None, fx=LUPE, fy=LUPE, interpolation=cv2.INTER_CUBIC)
            gh, gw = gross.shape[:2]
            cx_, cy_ = (mx - x0) * LUPE, (my - y0) * LUPE
            cv2.drawMarker(gross, (cx_, cy_), (0, 0, 255), cv2.MARKER_CROSS, 40, 1)
            cv2.rectangle(gross, (0, 0), (gw - 1, gh - 1), (0, 200, 255), 2)
            if gw <= W and gh + 40 <= H:
                im[40:40 + gh, W - gw:W] = gross
    if n.klick_px:
        cv2.drawMarker(im, n.klick_px, (0, 255, 0), cv2.MARKER_CROSS, 24, 2)
        p = n.punkt
        cv2.putText(im, f"x {p[0]:+.0f} y {p[1]:+.0f} z {p[2]:+.0f} mm", (10, 28),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0, 255, 0), 2)
    else:
        cv2.putText(im, "Stelle anklicken", (10, 28), cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0, 200, 255), 2)
    cv2.imshow(WIN, im)
    k = cv2.waitKey(1) & 0xFF
    if k == ord("q"): break
    if k == ord("f") and n.punkt:
        print(f"  -> fahre auf ({n.punkt[0]:+.1f}, {n.punkt[1]:+.1f}, {a.hover:.0f}) yaw {a.yaw:.0f}")
        subprocess.run([sys.executable, os.path.expanduser("~/ros2_ws/tools/zeige_punkt.py"),
                        "--x", f"{n.punkt[0]:.1f}", "--y", f"{n.punkt[1]:.1f}", "--z", f"{a.hover:.1f}",
                        "--yaw", str(a.yaw), "--vel", "0.3", "--acc", "0.15", "--sofort"])
    if k == ord("s") and n.punkt:
        d = json.load(open(a.datei)) if os.path.exists(a.datei) else {}
        name = f"klick_{len(d)+1}"
        d[name] = {"xyz_mm": [round(v, 1) for v in n.punkt], "pixel": list(n.klick_px),
                   "zeit": time.strftime("%Y-%m-%d %H:%M")}
        json.dump(d, open(a.datei, "w"), indent=1); print(f"  gesichert als {name} in {a.datei}")
cv2.destroyAllWindows(); n.destroy_node(); rclpy.shutdown()
