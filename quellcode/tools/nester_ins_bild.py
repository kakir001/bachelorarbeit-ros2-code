#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""Alle Nester (und beliebige Punkte) ins Kamerabild projizieren - Sichtpruefung der
Kamerakalibrierung: liegen die projizierten Nestpunkte auf den echten Nestern?
    python3 tools/nester_ins_bild.py --bild /tmp/nester.png [--kopf-z 50] [--punkt trichter_greif]
"""
import argparse, json, math, os, sys, time, cv2, numpy as np, rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import Image, CameraInfo
import tf2_ros
ap = argparse.ArgumentParser()
ap.add_argument("--datei", default=os.path.expanduser("~/ros2_ws/teach_punkte.json"))
ap.add_argument("--kopf-z", type=float, default=50.0, help="[mm] Hoehe, auf die die Nest-xy projiziert wird")
ap.add_argument("--punkt", nargs="*", default=[], help="weitere Punkte (tcp_modell_mm, eigene z)")
ap.add_argument("--bild", required=True); ap.add_argument("--skal", type=int, default=3)
a = ap.parse_args()
P = json.load(open(a.datei))
def qrot(q, v):
    x, y, z, w = q; vx, vy, vz = v
    t = (2*(y*vz - z*vy), 2*(z*vx - x*vz), 2*(x*vy - y*vx))
    return (vx + w*t[0] + (y*t[2] - z*t[1]), vy + w*t[1] + (z*t[0] - x*t[2]), vz + w*t[2] + (x*t[1] - y*t[0]))
rclpy.init(); n = Node("nester_ins_bild"); d = {}
n.create_subscription(Image, "/camera/color/image_raw", lambda m: d.__setitem__("img", m), qos_profile_sensor_data)
n.create_subscription(CameraInfo, "/camera/color/camera_info", lambda m: d.__setitem__("K", (m.k[0], m.k[4], m.k[2], m.k[5], m.header.frame_id)), qos_profile_sensor_data)
buf = tf2_ros.Buffer(); tf2_ros.TransformListener(buf, n)
t0 = time.time()
while time.time() - t0 < 15 and not ("img" in d and "K" in d): rclpy.spin_once(n, timeout_sec=0.1)
if "img" not in d: sys.exit("!! kein Kamerabild")
fx, fy, cx, cy, frame = d["K"]
tf = None
for _ in range(50):
    rclpy.spin_once(n, timeout_sec=0.1)
    try: tf = buf.lookup_transform(frame, "robot_base", rclpy.time.Time()); break
    except Exception: pass
if tf is None: sys.exit("!! keine TF robot_base -> " + frame)
tr = tf.transform.translation; r = tf.transform.rotation; q = (r.x, r.y, r.z, r.w)
m = d["img"]; img = np.frombuffer(m.data, np.uint8).reshape(m.height, m.width, -1)
img = cv2.cvtColor(img, cv2.COLOR_RGB2BGR) if m.encoding == "rgb8" else img.copy()
S = a.skal; big = cv2.resize(img, None, fx=S, fy=S, interpolation=cv2.INTER_CUBIC)
def proj(xyz_mm):
    p = qrot(q, [v / 1000.0 for v in xyz_mm]); po = (p[0] + tr.x, p[1] + tr.y, p[2] + tr.z)
    return fx * po[0] / po[2] + cx, fy * po[1] / po[2] + cy
punkte = [(k, v["tcp_modell_mm"][0], v["tcp_modell_mm"][1], a.kopf_z) for k, v in P.items() if k.startswith("nest_") and "tcp_modell_mm" in v and "_" not in k[5:]]
punkte += [(k, *P[k]["tcp_modell_mm"]) for k in a.punkt if k in P and "tcp_modell_mm" in P[k]]
for k, x, y, z in punkte:
    u, v = proj((x, y, z)); U, V = int(round(u * S)), int(round(v * S))
    cv2.circle(big, (U, V), 4, (255, 0, 0), 1); cv2.putText(big, k[5:] if k.startswith("nest_") else k, (U + 4, V - 3), cv2.FONT_HERSHEY_SIMPLEX, 0.35, (0, 255, 255), 1)
    print(f"{k:16s} ({x:+.1f},{y:+.1f},{z:.0f}) -> Pixel ({u:.1f},{v:.1f})")
cv2.imwrite(a.bild, big); print("Bild:", a.bild); rclpy.shutdown()
