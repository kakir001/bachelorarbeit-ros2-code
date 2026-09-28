#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""Kamera-Pruefung: steht in einem Nest der Ablageplatte eine Welle (aufrecht), liegt sie, fehlt sie?

Ohne YOLO (RAM auf dem Nano), rein geometrisch: Nest-TCP aus teach_punkte.json ->
Kopf-Ebene z = --kopf-z (50 mm, Kopf einer sitzenden Welle) -> Pixel (camera_info + TF,
Umkehrung von punkt_klicken.py). Um den Pixel wird die Farbe der inneren Scheibe (Kopf,
Ø 7 mm ~ 4 px) mit dem Ring aussen (Platte) verglichen; die abweichenden Pixel im Fenster
bilden einen Klecks:
    kompakt (Laenge <= --stehend-px)   -> STEHT   (nur der Kopf ist zu sehen)
    lang    (Laenge >= --liegend-px)   -> LIEGT   (44 mm Welle ~ 22 px)
    nichts                             -> FEHLT
Der Arm darf das Nest nicht verdecken (vorher weg fahren, z.B. zum Trichter).
Debugbild mit Fenster und Klecks: --bild PFAD.png

    python3 tools/pruefe_nest_kamera.py --nest nest_11
    python3 tools/pruefe_nest_kamera.py --nest nest_11 nest_14 --bild /tmp/nest.png
Exit 0 = alle stehen, 4 = mindestens eine liegt/fehlt, 1 = Fehler. Ausgabe je Nest eine Zeile
und am Ende ERGEBNIS {json}.
"""
import argparse, json, math, os, sys, time
import numpy as np, cv2
import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import Image, CameraInfo
from moveit_msgs.srv import GetPositionFK
from moveit_msgs.msg import RobotState
import tf2_ros

ARM = ['joint2_to_joint1', 'joint3_to_joint2', 'joint4_to_joint3',
       'joint5_to_joint4', 'joint6_to_joint5', 'joint6output_to_joint6']
ap = argparse.ArgumentParser()
ap.add_argument("--nest", nargs="+", required=True)
ap.add_argument("--datei", default=os.path.expanduser("~/ros2_ws/teach_punkte.json"))
ap.add_argument("--kopf-z", type=float, default=50.0, help="[mm] Kopfoberkante einer sitzenden Welle")
ap.add_argument("--frames", type=int, default=5)
ap.add_argument("--fenster", type=int, default=14, help="[px] Halbseite des Pruef-Fensters")
ap.add_argument("--schwelle", type=float, default=40.0, help="Farbabstand (BGR, L1) zum Plattenring")
ap.add_argument("--stehend-px", type=float, default=9.0); ap.add_argument("--liegend-px", type=float, default=14.0)
ap.add_argument("--bild", default=None)
a = ap.parse_args()


def qrot(q, v):
    x, y, z, w = q; t = 2.0 * np.cross((x, y, z), v)
    return np.asarray(v) + w * t + np.cross((x, y, z), t)


class N(Node):
    def __init__(s):
        super().__init__("pruefe_nest_kamera"); s.imgs = []; s.K = None; s.frame = None
        s.create_subscription(Image, "/camera/color/image_raw", s._img, qos_profile_sensor_data)
        s.create_subscription(CameraInfo, "/camera/color/camera_info", s._info, qos_profile_sensor_data)
        s.buf = tf2_ros.Buffer(); tf2_ros.TransformListener(s.buf, s)
        s.fk = s.create_client(GetPositionFK, "/compute_fk")
    def _img(s, m):
        if len(s.imgs) < a.frames:
            im = np.frombuffer(m.data, dtype=np.uint8).reshape(m.height, m.width, 3)
            s.imgs.append(np.ascontiguousarray(im[:, :, ::-1] if m.encoding.lower() == "rgb8" else im)); s.frame = m.header.frame_id
    def _info(s, m): s.K = (m.k[0], m.k[4], m.k[2], m.k[5])


rclpy.init(); n = N(); t0 = time.time()
while time.time() - t0 < 15 and (len(n.imgs) < a.frames or n.K is None): rclpy.spin_once(n, timeout_sec=0.1)
if len(n.imgs) < a.frames or n.K is None: sys.exit("!! kein Kamerabild / camera_info")
img = np.median(np.stack(n.imgs), axis=0).astype(np.uint8)
try:
    tf = n.buf.lookup_transform(n.frame, "robot_base", rclpy.time.Time())   # robot_base -> optisch
except Exception as e: sys.exit(f"!! kein TF: {e}")
t = np.array((tf.transform.translation.x, tf.transform.translation.y, tf.transform.translation.z))
r = tf.transform.rotation; q = (r.x, r.y, r.z, r.w); fx, fy, cx, cy = n.K
punkte = json.load(open(a.datei)); n.fk.wait_for_service(10)
debug = img.copy(); ergebnis = {}; alle_ok = True
for nest in a.nest:
    if nest not in punkte: print(f"!! {nest} nicht in {a.datei}"); alle_ok = False; continue
    rs = RobotState(); rs.joint_state.name = list(ARM); rs.joint_state.position = [float(v) for v in punkte[nest]["rad"]]
    rq = GetPositionFK.Request(); rq.header.frame_id = "robot_base"; rq.fk_link_names = ["tcp"]; rq.robot_state = rs
    f = n.fk.call_async(rq); rclpy.spin_until_future_complete(n, f, timeout_sec=5); p = f.result().pose_stamped[0].pose.position
    pb = np.array((p.x, p.y, a.kopf_z / 1000.0))                      # Nest xy, Kopfhoehe
    po = qrot(q, pb) + t                                               # im optischen Frame
    u, v = int(round(fx * po[0] / po[2] + cx)), int(round(fy * po[1] / po[2] + cy))
    H, W = img.shape[:2]; R = a.fenster
    if not (R <= u < W - R and R <= v < H - R): print(f"{nest}: Pixel ({u},{v}) ausserhalb des Bildes"); alle_ok = False; continue
    win = img[v-R:v+R+1, u-R:u+R+1].astype(float)
    yy, xx = np.mgrid[-R:R+1, -R:R+1]; rr = np.hypot(xx, yy)
    ring = win[(rr >= 8) & (rr <= R)]; ref = np.median(ring, axis=0)          # Plattenfarbe
    diff = np.abs(win - ref).sum(axis=2); maske = (diff > a.schwelle).astype(np.uint8)
    maske = cv2.morphologyEx(maske, cv2.MORPH_OPEN, np.ones((2, 2), np.uint8))
    cnts, _ = cv2.findContours(maske, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
    # Klecks, der der Mitte am naechsten liegt
    best, bl = None, 1e9
    for c in cnts:
        if cv2.contourArea(c) < 3: continue
        m = cv2.moments(c); cxm, cym = m["m10"] / m["m00"] - R, m["m01"] / m["m00"] - R
        d = math.hypot(cxm, cym)
        if d < bl: best, bl = c, d
    if best is None: zustand, laenge = "FEHLT", 0.0
    else:
        (_, _), (w_, h_), _ = cv2.minAreaRect(best); laenge = max(w_, h_)
        zustand = "STEHT" if laenge <= a.stehend_px else ("LIEGT" if laenge >= a.liegend_px else "UNKLAR")
    innen = win[rr <= 3.5]; kontrast = float(np.abs(innen - ref).sum(axis=1).mean())
    ergebnis[nest] = {"pixel": [u, v], "zustand": zustand, "klecks_px": round(float(laenge), 1), "kontrast": round(kontrast, 1),
                      "mitte_abstand_px": None if best is None else round(float(bl), 1)}
    print(f"{nest}: Pixel ({u},{v})  {zustand}  Klecks {laenge:.1f} px, Kontrast innen {kontrast:.0f}, Abstand zur Mitte {'-' if best is None else f'{bl:.1f}'} px")
    if zustand != "STEHT": alle_ok = False
    cv2.rectangle(debug, (u-R, v-R), (u+R, v+R), (0, 255, 0) if zustand == "STEHT" else (0, 0, 255), 1)
    cv2.drawMarker(debug, (u, v), (255, 0, 0), cv2.MARKER_CROSS, 8, 1)
    cv2.putText(debug, f"{nest[5:]}:{zustand[0]}", (u-R, v-R-2), cv2.FONT_HERSHEY_SIMPLEX, 0.35, (0, 255, 255), 1)
if a.bild:
    cv2.imwrite(a.bild, cv2.resize(debug, None, fx=3, fy=3, interpolation=cv2.INTER_NEAREST)); print(f"  Debugbild: {a.bild}")
print("  ERGEBNIS " + json.dumps(ergebnis))
n.destroy_node(); rclpy.shutdown(); sys.exit(0 if alle_ok else 4)
