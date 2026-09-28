#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""Erreichbarkeit der Bereitstellungsschale pruefen - OHNE den Arm zu bewegen.

Frage (Benutzer 2026-09-12): kann der Roboter ueberall in der Schale SENKRECHT greifen?
Wenn nicht, wird die Schale verschoben, bevor sie festgeklebt wird.

Verfahren: Raster im Schaleninneren (schale-Frame aus TF, Form aus schale.xacro: R 186.9..
301.9 mm, Sektor 45.3 Grad, Wand 2 mm, Rand-Abstand --rand). Je Rasterpunkt und je
Gierwinkel (Vorgabe 0/90/180/270 Grad um die senkrechte Greifachse) eine kollisionsgeprueft
IK ueber /compute_ik (MoveIt, aktuelle Planungsszene mit Schale, Trichter, Sockel) fuer
ZWEI Hoehen:
    Griff : TCP (Fingerspitze) auf Innenboden + --griff-mm  (liegende Welle: Flansch
            13.5 mm, Fingerspitzen ~3 mm ueber dem Boden)
    Ueber : Griff + --ueber-mm  (Anfahren/Heben senkrecht)
Der Greifer wird dabei als OFFEN angenommen (--greifer, Vorgabe -0.335 = Vorgriff 23.5 mm),
denn offene Finger sind das, was an die Wand stoesst. Ausgabe: Tabelle, ASCII-Karte,
und (wenn die Kamera laeuft) Kamerabild mit Punkten: gruen = Griff UND Ueber loesbar,
gelb = nur Ueber, rot = nichts. Dazu, welche Gierwinkel je Punkt gehen.

    python3 tools/schale_erreichbarkeit.py                   # Stack mit SCHALE_MONTIERT=1
    python3 tools/schale_erreichbarkeit.py --bild karte.png --ringe 4 --schritte 6

Faehrt NICHT. Ein "nein" heisst: mit dieser Schalenlage plant MoveIt dort keinen senkrechten
Griff - Ursachen koennen Gelenkgrenzen (Totsektor 150-176 Grad Azimut), der Sockelkasten oder
die Schalenwand sein; welche, sagt die Karte nicht, nur WO.
"""
import argparse, math, os, sys, time
import numpy as np
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import PoseStamped, Quaternion
from moveit_msgs.srv import GetPositionIK
from moveit_msgs.msg import RobotState
from builtin_interfaces.msg import Duration
import tf2_ros

ARM = ['joint2_to_joint1', 'joint3_to_joint2', 'joint4_to_joint3',
       'joint5_to_joint4', 'joint6_to_joint5', 'joint6output_to_joint6']
BASE = "robot_base"
# Senkrechter Griff (tcp +Y nach unten) - wie zeige_punkt.py / arbeitsraum_grenze.py
Q_REF = (-0.583486105248945, 0.3994295494594975, -0.3994295494594975, 0.583486105248945)
R_INNEN, R_AUSSEN, WAND, WINKEL = 0.1869, 0.3019, 0.002, math.radians(45.3)

ap = argparse.ArgumentParser()
ap.add_argument("--ringe", type=int, default=4, help="Radien zwischen den Waenden")
ap.add_argument("--schritte", type=int, default=6, help="Winkelschritte im Sektor")
ap.add_argument("--rand", type=float, default=12.0, help="[mm] Abstand von der Wandinnenseite")
ap.add_argument("--griff-mm", type=float, default=3.0, help="[mm] TCP ueber dem Innenboden")
ap.add_argument("--ueber-mm", type=float, default=40.0)
ap.add_argument("--gier", type=float, nargs="+", default=[0.0, 90.0, 180.0, 270.0],
                help="Gierwinkel [Grad] relativ zum Azimut des Punkts")
ap.add_argument("--greifer", type=float, default=-0.335, help="Greifergelenk fuer die Kollisionspruefung")
ap.add_argument("--timeout", type=float, default=1.0, help="[s] je IK-Anfrage")
ap.add_argument("--bild", default="", help="Karte ins Kamerabild zeichnen und hierhin sichern")
a = ap.parse_args()


def qmul(p, q):
    ax, ay, az, aw = p; bx, by, bz, bw = q
    return (aw*bx + ax*bw + ay*bz - az*by, aw*by - ax*bz + ay*bw + az*bx,
            aw*bz + ax*by - ay*bx + az*bw, aw*bw - ax*bx - ay*by - az*bz)


def qz(deg):
    h = math.radians(deg) / 2.0
    return (0.0, 0.0, math.sin(h), math.cos(h))


def qrot(q, p):
    x, y, z, w = q
    R = np.array([[1 - 2*(y*y + z*z), 2*(x*y - z*w), 2*(x*z + y*w)],
                  [2*(x*y + z*w), 1 - 2*(x*x + z*z), 2*(y*z - x*w)],
                  [2*(x*z - y*w), 2*(y*z + x*w), 1 - 2*(x*x + y*y)]])
    return np.asarray(p) @ R.T


rclpy.init()
node = Node("schale_erreichbarkeit")
puffer = tf2_ros.Buffer(); tf2_ros.TransformListener(puffer, node)
ik = node.create_client(GetPositionIK, "/compute_ik")
if not ik.wait_for_service(20.0):
    sys.exit("!! /compute_ik fehlt - laeuft der Stack?")
ende = time.time() + 15
while time.time() < ende and not puffer.can_transform(BASE, "schale", rclpy.time.Time()):
    rclpy.spin_once(node, timeout_sec=0.2)
if not puffer.can_transform(BASE, "schale", rclpy.time.Time()):
    sys.exit("!! TF robot_base -> schale fehlt. Stack mit SCHALE_MONTIERT=1 gestartet?")
tf = puffer.lookup_transform(BASE, "schale", rclpy.time.Time())
r_, t_ = tf.transform.rotation, tf.transform.translation
q_s = (r_.x, r_.y, r_.z, r_.w); t_s = np.array([t_.x, t_.y, t_.z])
yaw_s = math.atan2(2*(q_s[3]*q_s[2] + q_s[0]*q_s[1]), 1 - 2*(q_s[1]**2 + q_s[2]**2))
print("Schale: Mittelpunkt (%.1f, %.1f) mm, Innenboden z %.1f mm, Halbierende %.1f Grad"
      % (t_s[0]*1e3, t_s[1]*1e3, t_s[2]*1e3, math.degrees(yaw_s)))

# aktuelle Gelenke als Seed (IK-Loesungen nahe der jetzigen Stellung), Greifer offen
from sensor_msgs.msg import JointState
seed = {}
def _js(m): seed.update(zip(m.name, m.position))
node.create_subscription(JointState, "/joint_states", _js, 10)
ende = time.time() + 5
while time.time() < ende and not all(j in seed for j in ARM):
    rclpy.spin_once(node, timeout_sec=0.1)
rs = RobotState()
rs.joint_state.name = list(ARM) + ["gripper_controller"]
rs.joint_state.position = [float(seed.get(j, 0.0)) for j in ARM] + [a.greifer]


def ik_ok(x, y, z, gier_abs):
    rq = GetPositionIK.Request()
    rq.ik_request.group_name = "arm"; rq.ik_request.ik_link_name = "tcp"
    rq.ik_request.avoid_collisions = True
    rq.ik_request.timeout = Duration(sec=int(a.timeout), nanosec=int((a.timeout % 1) * 1e9))
    rq.ik_request.robot_state = rs
    ps = PoseStamped(); ps.header.frame_id = BASE
    ps.pose.position.x, ps.pose.position.y, ps.pose.position.z = float(x), float(y), float(z)
    q = qmul(qz(gier_abs), Q_REF)
    ps.pose.orientation = Quaternion(x=q[0], y=q[1], z=q[2], w=q[3])
    rq.ik_request.pose_stamped = ps
    fut = ik.call_async(rq)
    rclpy.spin_until_future_complete(node, fut, timeout_sec=a.timeout + 5)
    res = fut.result()
    return bool(res and res.error_code.val == 1)


# Raster im schale-Frame: Radien r, Winkel phi (relativ zur Halbierenden)
rand = a.rand / 1e3
radien = np.linspace(R_INNEN + WAND + rand, R_AUSSEN - WAND - rand, a.ringe)
# Winkelrand so, dass der Abstand zur Seitenwand am jeweiligen Radius >= rand ist
ergeb = []          # (i_ring, i_phi, x, y, ok_griff[list gier], ok_ueber[list gier])
z_griff = t_s[2] + a.griff_mm / 1e3
z_ueber = z_griff + a.ueber_mm / 1e3
n_ik = 0; t0 = time.time()
for i, r in enumerate(radien):
    dphi = math.asin(rand / r)
    phis = np.linspace(-WINKEL/2 + dphi, WINKEL/2 - dphi, a.schritte)
    for j, phi in enumerate(phis):
        p = qrot(q_s, [r*math.cos(phi), r*math.sin(phi), 0.0]) + t_s
        x, y = p[0], p[1]
        azimut = math.degrees(math.atan2(y, x))
        okg, oku = [], []
        for g in a.gier:
            oku.append(ik_ok(x, y, z_ueber, azimut + g)); n_ik += 1
            okg.append(oku[-1] and ik_ok(x, y, z_griff, azimut + g)); n_ik += 1
        ergeb.append((i, j, x, y, okg, oku))
        print("  r %.0f phi %+5.1f -> (%+6.1f, %+6.1f) r_robot %.0f az %.0f : Griff %s  Ueber %s"
              % (r*1e3, math.degrees(phi), x*1e3, y*1e3, math.hypot(x, y)*1e3, azimut % 360,
                 "".join("X" if v else "." for v in okg), "".join("X" if v else "." for v in oku)))
print("%d IK-Anfragen in %.0f s" % (n_ik, time.time() - t0))

# Zusammenfassung
n = len(ergeb)
g_any = sum(1 for e in ergeb if any(e[4])); u_any = sum(1 for e in ergeb if any(e[5]))
print("=" * 70)
print("Rasterpunkte %d: Griff+Ueber loesbar an %d (%.0f%%), nur Ueber an %d, nichts an %d"
      % (n, g_any, 100.0*g_any/n, u_any - g_any, n - u_any))
for k, g in enumerate(a.gier):
    print("  Gierwinkel %+4.0f: Griff an %d Punkten" % (g, sum(1 for e in ergeb if e[4][k])))
print("ASCII-Karte (Zeile = Radius innen->aussen, Spalte = Winkel von -%.0f bis +%.0f Grad):"
      % (math.degrees(WINKEL/2), math.degrees(WINKEL/2)))
for i in range(a.ringe):
    zeile = ""
    for j in range(a.schritte):
        e = [e for e in ergeb if e[0] == i and e[1] == j][0]
        zeile += " G" if any(e[4]) else (" u" if any(e[5]) else " -")
    print("  r %.0f mm:%s" % (radien[i]*1e3, zeile))
print("  G = Griff+Ueber, u = nur Ueber, - = nichts")

if a.bild:
    try:
        import cv2
        sys.argv = [sys.argv[0]]
        sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
        import schale_finden as sf, schale_finden_kontur as sk
        k = sf.Finder()
        ende = time.time() + 20
        while time.time() < ende and not (k.bereit() and k.farbbild is not None):
            rclpy.spin_once(k, timeout_sec=0.2)
        if k.farbbild is None:
            raise RuntimeError("kein Kamerabild")
        tfc = k.puffer.lookup_transform(BASE, k.depth_frame, rclpy.time.Time()) if k.puffer.can_transform(BASE, k.depth_frame, rclpy.time.Time()) else puffer.lookup_transform(BASE, k.depth_frame, rclpy.time.Time())
        rc, tc = tfc.transform.rotation, tfc.transform.translation
        Rq = sk.rot_inv((rc.x, rc.y, rc.z, rc.w)); tc = np.array([tc.x, tc.y, tc.z])
        fx, fy, cx, cy = k.K; sk_ = 3
        out = cv2.resize(k.farbbild, None, fx=sk_, fy=sk_, interpolation=cv2.INTER_CUBIC)
        for e in ergeb:
            Pc = (np.array([e[2], e[3], z_griff]) - tc) @ Rq.T
            u, v = int((Pc[0]/Pc[2]*fx + cx)*sk_), int((Pc[1]/Pc[2]*fy + cy)*sk_)
            col = (0, 200, 0) if any(e[4]) else ((0, 220, 255) if any(e[5]) else (0, 0, 255))
            cv2.circle(out, (u, v), 7, col, -1); cv2.circle(out, (u, v), 7, (0, 0, 0), 1)
            cv2.putText(out, "".join("X" if v_ else "." for v_ in e[4]), (u + 8, v + 4),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.35, (255, 255, 255), 1)
        cv2.putText(out, "gruen = Griff+Ueber, gelb = nur Ueber, rot = nichts; Ziffern = Gierwinkel %s"
                    % "/".join("%.0f" % g for g in a.gier), (8, 24), cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0, 0, 0), 3)
        cv2.putText(out, "gruen = Griff+Ueber, gelb = nur Ueber, rot = nichts; Ziffern = Gierwinkel %s"
                    % "/".join("%.0f" % g for g in a.gier), (8, 24), cv2.FONT_HERSHEY_SIMPLEX, 0.6, (255, 255, 255), 1)
        cv2.imwrite(a.bild, out); print("Karte: %s" % a.bild)
    except Exception as ex:                                   # noqa: BLE001
        print("Kein Kamerabild (%s) - nur Tabelle." % ex)
node.destroy_node(); rclpy.shutdown()
