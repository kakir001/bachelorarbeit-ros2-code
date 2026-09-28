#!/usr/bin/env python3
"""Misst mit der KAMERA, wie weit der Greifer nachgibt, wenn von Hand gezogen wird.

Warum die Kamera und nicht der Encoder
--------------------------------------
Die Encodermessung (tools/measure_joint_play.py) hat am 2026-09-08 gezeigt: beim
Ziehen von Hand meldet der Encoder hoechstens 0.36 Grad, und das geht vollstaendig
zurueck. Sichtbar bewegt sich der Arm aber deutlich mehr. Der Encoder sitzt am
Servo-EINGANG — Spiel HINTER dem Getriebe sieht er prinzipiell nicht, und genau
dieses Spiel erfaehrt das Modell (FK/RViz) nie. Deshalb wird hier von AUSSEN
gemessen: die Tiefenkamera schaut auf den Greifer, unabhaengig von jeder
Roboterelektronik.

Warum nicht in Nullstellung
---------------------------
In Nullstellung steht der TCP bei z ~ 0.52 und die Kamera bei z ~ 0.61 — keine
9 cm dazwischen, unter der Mindestreichweite der D435i und ausserhalb des
Sichtfelds. Gemessen wird deshalb in einer Arbeitspose ueber der Plattform, wo
die Kamera den Greifer sauber sieht (und wo Genauigkeit ohnehin zaehlt).

Ablauf
------
    1. IK fuer die Zielpose (Greifer senkrecht nach unten) ueber MoveIt
    2. der GESAMTE Gelenkweg dorthin wird in Schritten mit /check_state_validity
       geprueft — bei einer einzigen unzulaessigen Zwischenstellung faehrt nichts
    3. langsam anfahren
    4. drei Zeitfenster mit Anzeige: RUHE / ZIEHEN UND HALTEN / LOSGELASSEN.
       In jedem Fenster wird die Tiefenwolke gemittelt und der Greifer in einem
       festen Quader um den erwarteten TCP herum ausgewertet.
    5. Ergebnis: Verschiebung des Greifers in mm — einmal beim Ziehen (elastisch
       + Spiel) und einmal bleibend nach dem Loslassen (das ist das echte Spiel).

Aufruf
------
    python3 tools/measure_deflection_camera.py --dry-run      # nur rechnen/pruefen
    python3 tools/measure_deflection_camera.py                # fahren und messen
    python3 tools/measure_deflection_camera.py --target 0.15 -0.12 0.12
"""
import argparse
import json
import math
import sys
import time

import cv2
import numpy as np
import rclpy
from rclpy.action import ActionClient
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import Image, CameraInfo, JointState
from moveit_msgs.srv import GetStateValidity
from moveit_msgs.msg import RobotState
from control_msgs.action import FollowJointTrajectory
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint
import tf2_ros

sys.path.insert(0, "/home/er/ros2_ws/src/mycobot_calibration")
from mycobot_calibration.numerical_ik import NumericalIK, ARM_JOINTS  # noqa: E402

PHASEN = [
    (10, "BEREIT MACHEN", "Hand an den Greifer - noch NICHT ziehen", (120, 120, 120), None),
    (7,  "RUHE", "NICHT beruehren - Nullbezug wird gemessen", (60, 160, 60), 2.5),
    (11, "JETZT ZIEHEN UND HALTEN", "SANFT in EINE Richtung - halten bis Umschaltung",
     (0, 140, 230), 4.5),
    (10, "LOSLASSEN", "Greifer freigeben und nicht mehr beruehren", (160, 100, 60), 4.5),
]


def tafel(titel, unter, farbe, rest, i):
    img = np.zeros((420, 900, 3), np.uint8)
    img[:] = (28, 28, 30)
    cv2.rectangle(img, (0, 0), (900, 90), farbe, -1)
    cv2.putText(img, "AUSLENKUNG MIT KAMERA MESSEN  (%d/4)" % i, (24, 58),
                cv2.FONT_HERSHEY_SIMPLEX, 0.95, (255, 255, 255), 2, cv2.LINE_AA)
    cv2.putText(img, titel, (24, 190), cv2.FONT_HERSHEY_SIMPLEX, 1.6, farbe, 4, cv2.LINE_AA)
    cv2.putText(img, unter, (24, 245), cv2.FONT_HERSHEY_SIMPLEX, 0.75,
                (220, 220, 220), 1, cv2.LINE_AA)
    cv2.putText(img, "%d" % int(np.ceil(rest)), (700, 340),
                cv2.FONT_HERSHEY_SIMPLEX, 4.5, (255, 255, 255), 8, cv2.LINE_AA)
    return img


class Messung(Node):
    def __init__(self):
        super().__init__("auslenkung_kamera")
        self.js = None
        self.tiefe = None
        self.info = None
        self.create_subscription(JointState, "/joint_states", self._js, 20)
        self.create_subscription(Image, "/camera/depth/image_rect_raw",
                                 self._img, qos_profile_sensor_data)
        self.create_subscription(CameraInfo, "/camera/depth/camera_info",
                                 self._info, qos_profile_sensor_data)
        self.buf = tf2_ros.Buffer()
        self.lis = tf2_ros.TransformListener(self.buf, self)
        self.valid = self.create_client(GetStateValidity, "/check_state_validity")
        self.traj = ActionClient(self, FollowJointTrajectory,
                                 "/arm_controller/follow_joint_trajectory")

    def _js(self, m):
        self.js = m

    def _img(self, m):
        self.tiefe = m

    def _info(self, m):
        self.info = m

    def warte(self, pruef, sek=20.0, text=""):
        t0 = time.time()
        while rclpy.ok() and time.time() - t0 < sek:
            rclpy.spin_once(self, timeout_sec=0.1)
            if pruef():
                return True
        if text:
            print("FEHLER: %s" % text)
        return False

    def gelenke(self):
        d = dict(zip(self.js.name, self.js.position))
        return [d[j] for j in ARM_JOINTS]

    def tf_mat(self, ziel, quelle):
        t0 = time.time()
        while time.time() - t0 < 5.0:
            try:
                t = self.buf.lookup_transform(ziel, quelle, rclpy.time.Time())
                q = t.transform.rotation
                x, y, z, w = q.x, q.y, q.z, q.w
                R = np.array([
                    [1 - 2 * (y * y + z * z), 2 * (x * y - z * w), 2 * (x * z + y * w)],
                    [2 * (x * y + z * w), 1 - 2 * (x * x + z * z), 2 * (y * z - x * w)],
                    [2 * (x * z - y * w), 2 * (y * z + x * w), 1 - 2 * (x * x + y * y)]])
                tr = np.array([t.transform.translation.x, t.transform.translation.y,
                               t.transform.translation.z])
                return R, tr
            except Exception:
                rclpy.spin_once(self, timeout_sec=0.1)
        return None, None

    def zulaessig(self, q):
        """Eine Gelenkstellung von MoveIt pruefen lassen (Selbst- und Weltkollision)."""
        req = GetStateValidity.Request()
        rs = RobotState()
        js = JointState()
        js.name = list(ARM_JOINTS)
        js.position = [float(v) for v in q]
        rs.joint_state = js
        req.robot_state = rs
        req.group_name = "arm"
        fut = self.valid.call_async(req)
        rclpy.spin_until_future_complete(self, fut, timeout_sec=5.0)
        return bool(fut.result() and fut.result().valid)

    def wolke(self, frames=25):
        """Tiefenbilder mitteln und in robot_base umrechnen."""
        stapel = []
        t0 = time.time()
        letzte_stempel = None
        while len(stapel) < frames and time.time() - t0 < 15.0:
            rclpy.spin_once(self, timeout_sec=0.05)
            m = self.tiefe
            if m is None:
                continue
            stempel = (m.header.stamp.sec, m.header.stamp.nanosec)
            if stempel == letzte_stempel:
                continue
            letzte_stempel = stempel
            stapel.append(np.frombuffer(m.data, dtype=np.uint16).reshape(
                m.height, m.width).astype(np.float32))
        if len(stapel) < 5:
            return None
        a = np.stack(stapel)
        a[a == 0] = np.nan
        with np.errstate(invalid="ignore"):
            d = np.nanmedian(a, axis=0) * 0.001
        k = self.info.k
        fx, fy, cx, cy = k[0], k[4], k[2], k[5]
        h, w = d.shape
        vs, us = np.mgrid[0:h, 0:w]
        ok = np.isfinite(d) & (d > 0.05) & (d < 3.0)
        us, vs, z = us[ok], vs[ok], d[ok]
        P = np.column_stack([(us - cx) * z / fx, (vs - cy) * z / fy, z])
        R, t = self.tf_mat("robot_base", self.tiefe.header.frame_id)
        if R is None:
            return None
        return (R @ P.T).T + t


def greifer_punkte(B, mitte, halb=0.055, hoch=0.055, z_min=0.05):
    """Punkte in einem WUERFEL um das Bezugsframe herum.

    Frueher war der Quader nach unten offen (z_min bis mitte+hoch) — damit lag
    die halbe Szene drin und der Schwerpunkt beschrieb nicht mehr den Greifer.
    Jetzt ist er in allen drei Richtungen begrenzt; z_min haelt nur die
    Plattform heraus."""
    m = ((np.abs(B[:, 0] - mitte[0]) < halb) &
         (np.abs(B[:, 1] - mitte[1]) < halb) &
         (np.abs(B[:, 2] - mitte[2]) < hoch) & (B[:, 2] > z_min))
    return B[m]


def kennwerte(P):
    """Schwerpunkt + Schwerpunkt der untersten 25 % (das sind die Fingerspitzen)."""
    if P.shape[0] < 200:
        return None
    unten = P[P[:, 2] <= np.percentile(P[:, 2], 25)]
    return {"n": int(P.shape[0]), "schwerpunkt": P.mean(axis=0),
            "spitze": unten.mean(axis=0), "z_min": float(np.percentile(P[:, 2], 2))}


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--target", nargs=3, type=float, default=[0.15, -0.12, 0.12],
                    help="Ziel-TCP in robot_base (m)")
    ap.add_argument("--joints", nargs=6, type=float, default=None,
                    help="Zielstellung direkt in GRAD angeben (umgeht die IK — auf dem Nano "
                         "braucht MoveIts Multi-Seed-IK je Ziel mehrere Sekunden)")
    ap.add_argument("--dry-run", action="store_true", help="nur rechnen und pruefen")
    ap.add_argument("--skip-move", action="store_true",
                    help="nicht fahren, in der aktuellen Stellung messen")
    ap.add_argument("--frame", default="gripper_base",
                    help="Frame, um das der Messquader gelegt wird. Voreinstellung "
                         "gripper_base: von oben sieht die Kamera den Greifer-KOERPER, "
                         "die duennen Finger stehen ihr entgegen und liefern kaum Punkte. "
                         "Der Koerper sitzt starr am Arm, gibt die Auslenkung also genauso "
                         "wieder.")
    ap.add_argument("--halb", type=float, default=0.055, help="halbe Quaderbreite (m)")
    ap.add_argument("--richtung", default="", help="nur fuer den Bericht: in welche Richtung gezogen wurde (z.B. 'X vor/zurueck' oder 'Y links/rechts')")
    ap.add_argument("--sekunden", type=float, default=8.0, help="Fahrzeit zur Pose")
    ap.add_argument("--json", default="")
    a = ap.parse_args()

    rclpy.init()
    n = Messung()
    if not n.warte(lambda: n.js is not None, 20, "keine /joint_states — laeuft der Stack?"):
        return 1
    if not n.warte(lambda: n.tiefe is not None and n.info is not None, 25,
                   "keine Tiefenbilder — laeuft die Kamera?"):
        return 1
    if not n.valid.wait_for_service(timeout_sec=10.0):
        print("FEHLER: /check_state_validity nicht da — laeuft move_group?")
        return 1

    start = n.gelenke()
    print("Aktuelle Gelenke (Grad): %s" % [round(math.degrees(v), 1) for v in start])

    if a.joints is not None:
        q = [math.radians(v) for v in a.joints]
        ziel = None
        print("\nZielstellung direkt vorgegeben (Grad): %s" % list(a.joints))
    else:
        ik = NumericalIK(n)
        ziel = np.array(a.target, float)
        print("\nIK fuer TCP %s (Greifer senkrecht nach unten) ..." % np.round(ziel, 3))
        loesung = ik.solve_multi_seed(list(ziel), gripper_down=True, current_joints=start)
        if loesung is None:
            print("FEHLER: keine IK-Loesung. Anderes Ziel probieren (naeher an den Roboter).")
            return 2
        q = list(loesung)
        print("Loesung (Grad): %s" % [round(math.degrees(v), 1) for v in q])

    print("\nGesamten Weg pruefen (21 Zwischenstellungen, /check_state_validity) ...")
    schlecht = []
    for i in range(21):
        s = i / 20.0
        qi = [(1 - s) * start[k] + s * q[k] for k in range(6)]
        if not n.zulaessig(qi):
            schlecht.append(round(100 * s))
    if schlecht:
        print("ABBRUCH: unzulaessige Stellungen bei %s %% des Weges." % schlecht)
        print("Es wird NICHT gefahren. Anderes Ziel waehlen.")
        return 2
    print("alle 21 Stellungen zulaessig.")

    R, t = n.tf_mat("robot_base", a.frame)
    print("%s jetzt: %s" % (a.frame, np.round(t, 4)))

    if a.dry_run:
        print("\n--dry-run: es wird NICHT gefahren.")
        return 0

    if a.skip_move:
        print("\n--skip-move: es wird in der aktuellen Stellung gemessen.")
    else:
        print("\nFahre langsam zur Pose (%.0f s) ..." % a.sekunden)
        if not n.traj.wait_for_server(timeout_sec=10.0):
            print("FEHLER: arm_controller action nicht da.")
            return 1
        g = FollowJointTrajectory.Goal()
        jt = JointTrajectory()
        jt.joint_names = list(ARM_JOINTS)
        pt = JointTrajectoryPoint()
        pt.positions = [float(v) for v in q]
        pt.time_from_start.sec = int(a.sekunden)
        jt.points = [pt]
        g.trajectory = jt
        fut = n.traj.send_goal_async(g)
        rclpy.spin_until_future_complete(n, fut, timeout_sec=15.0)
        handle = fut.result()
        if handle is None or not handle.accepted:
            print("FEHLER: Ziel abgelehnt.")
            return 1
        erg = handle.get_result_async()
        rclpy.spin_until_future_complete(n, erg, timeout_sec=a.sekunden + 20)

    # WICHTIG: waehrend des Wartens MUSS gespint werden, sonst bleibt der
    # TF-Puffer auf dem Stand von VOR der Fahrt stehen und der Messquader wird
    # um die alte (falsche) TCP-Lage gelegt. Genau daran ist der erste Versuch
    # am 2026-09-08 gescheitert (Quader lag 19 cm zu hoch).
    t_ruhe = time.time()
    while time.time() - t_ruhe < 2.5:
        rclpy.spin_once(n, timeout_sec=0.05)
    R, tcp_ist = n.tf_mat("robot_base", a.frame)
    q_ist = n.gelenke()
    print("Stellung jetzt (Grad): %s" % [round(math.degrees(v), 1) for v in q_ist])
    print("%s jetzt: %s   Abstand zur Kamera: %.0f mm"
          % (a.frame, np.round(tcp_ist, 4),
             1000 * np.linalg.norm(tcp_ist - np.array([0.1686, -0.1261, 0.609]))))
    abweichung = max(abs(math.degrees(q_ist[k] - q[k])) for k in range(6))
    if abweichung > 3.0:
        print("ABBRUCH: Stellung weicht um %.1f Grad vom Ziel ab." % abweichung)
        return 2

    cv2.namedWindow("Auslenkung", cv2.WINDOW_NORMAL)
    cv2.resizeWindow("Auslenkung", 900, 420)
    cv2.moveWindow("Auslenkung", 40, 40)
    ergebnisse = []
    for i, (dauer, titel, unter, farbe, ab) in enumerate(PHASEN):
        t0 = time.time()
        gemessen = False
        while True:
            rest = dauer - (time.time() - t0)
            if rest <= 0:
                break
            u = unter
            if a.richtung and "ZIEHEN" in titel:
                u = "Richtung: %s  -  SANFT ziehen und halten" % a.richtung
            cv2.imshow("Auslenkung", tafel(titel, u, farbe, rest, i + 1))
            if cv2.waitKey(30) == 27:
                cv2.destroyAllWindows()
                print("Abgebrochen.")
                return 1
            if ab is not None and not gemessen and (time.time() - t0) >= ab:
                B = n.wolke()
                if B is None:
                    print("  [%s] keine Wolke" % titel)
                else:
                    P = greifer_punkte(B, tcp_ist, halb=a.halb)
                    kw = kennwerte(P)
                    if kw is None:
                        print("  [%s] zu wenige Greiferpunkte (%d)"
                              % (titel, 0 if P is None else P.shape[0]))
                    else:
                        kw["titel"] = titel
                        ergebnisse.append(kw)
                        print("  [%s] %d Punkte, Spitze z=%.4f m"
                              % (titel, kw["n"], kw["spitze"][2]))
                gemessen = True
            rclpy.spin_once(n, timeout_sec=0.01)
    cv2.destroyAllWindows()

    if len(ergebnisse) < 3:
        print("\nNicht alle drei Aufnahmen gelungen — keine Auswertung.")
        return 2

    r, gez, los = ergebnisse
    print("\n" + "=" * 68)
    print(" ERGEBNIS — mit der KAMERA gemessen (unabhaengig vom Encoder)")
    if a.richtung:
        print(" Zugrichtung: %s" % a.richtung)
    print("=" * 68)
    for name, feld in (("Schwerpunkt", "schwerpunkt"), ("Fingerspitzen", "spitze")):
        d1 = 1000 * (gez[feld] - r[feld])
        d2 = 1000 * (los[feld] - r[feld])
        print(" %-14s beim Ziehen : dx %+6.1f  dy %+6.1f  dz %+6.1f mm  (Betrag %5.1f mm)"
              % (name, d1[0], d1[1], d1[2], np.linalg.norm(d1)))
        print(" %-14s bleibend    : dx %+6.1f  dy %+6.1f  dz %+6.1f mm  (Betrag %5.1f mm)"
              % ("", d2[0], d2[1], d2[2], np.linalg.norm(d2)))
    print("\n Deutung: 'beim Ziehen' = elastisches Nachgeben PLUS Spiel.")
    print("          'bleibend'    = das, was nicht zurueckfedert, also echtes Spiel.")
    print(" Gegenprobe: der Encoder sah bei derselben Bewegung hoechstens 0.36 Grad,")
    print(" vollstaendig rueckfedernd. Was hier mehr steht, sieht das Modell NIE.")

    if a.json:
        with open(a.json, "w") as f:
            json.dump({k["titel"]: {"n": k["n"],
                                    "schwerpunkt": k["schwerpunkt"].tolist(),
                                    "spitze": k["spitze"].tolist()}
                       for k in ergebnisse}, f, indent=2)
        print("\n JSON: %s" % a.json)
    n.destroy_node()
    rclpy.shutdown()
    return 0


if __name__ == "__main__":
    sys.exit(main())
