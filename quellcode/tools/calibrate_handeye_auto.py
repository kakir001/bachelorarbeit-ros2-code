#!/usr/bin/env python3
"""Hand-Auge-Kalibrierung, die ihre Stellungen selbst anfaehrt.

Warum nicht von Hand
--------------------
Die bisherige Vorschrift war "16+ Stellungen freihaendig anfahren". Das ist
schlecht wiederholbar, und beim Anfassen verstellt sich der Arm. Hier werden die
Stellungen vorher berechnet (Board zeigt zur Kamera, kollisionsfrei, moeglichst
verschiedene DREHUNGEN - darauf beruht die Hand-Auge-Gleichung), einzeln
angefahren und mit dem Positionierer nachgefuehrt, bis sie ruhig stehen.

Aufbau
------
Eye-to-hand: die Kamera steht fest, das Board sitzt am Greifer. Deshalb wird
die Roboterlage INVERS abgetastet (Basis im tcp-Frame) - so liefert
cv2.calibrateHandEye die gesuchte Kamera-zu-Basis-Transformation.

Wichtig
-------
- Das Board-Frame ist `charuco_gemessen` (vom Detektor), NICHT `charuco_board`
  (der Modell-Link). Beide gleich zu benennen war ein Fehler, der stillschweigend
  das Modell statt der Kamera vermessen haette.
- Tsai-Lenz wird NICHT verwendet: es hat in diesem Aufbau schon einmal die
  Hoehe verdorben (2026-06-02). Gerechnet wird mit Park, gegengeprueft mit
  Horaud, Daniilidis und Andreff.
- Die Kamera muss auf 1280x720 stehen; bei 424x240 ist der Marker zu klein.

Stellungen
----------
Vorgabe ist `kalibrierposen_charuco.json` im Repository. Darin stehen die 16
Stellungen, in denen das Board am 2026-09-09 vollstaendig erkannt wurde - eine
erneute Suche eruebrigt sich, solange Board und Kamera an ihrem Platz bleiben.
Hat sich etwas daran geaendert, muss neu gesucht werden (Stellungen ueber
/compute_fk bewerten: bedruckte Seite ist -z des Frames `charuco_muster`, der
Einfallswinkel darf bis rund 63 Grad gehen, und der Punkt muss auch wirklich im
Bild liegen - das war beim ersten Versuch der uebersehene Teil).

Aufruf
------
    python3 tools/calibrate_handeye_auto.py --dry-run
    python3 tools/calibrate_handeye_auto.py
    python3 tools/calibrate_handeye_auto.py --posen /tmp/eigene_suche.json
"""
import argparse
import json
import math
import os
import sys
import time

import cv2
import numpy as np
import rclpy

sys.path.insert(0, "/home/er/ros2_ws/tools")
from measure_deflection_camera import Messung  # noqa: E402
sys.path.insert(0, "/home/er/ros2_ws/src/mycobot_calibration")
from mycobot_calibration.positionieren import Positionierer  # noqa: E402
from std_msgs.msg import Int32  # noqa: E402

BASIS = "robot_base"
TCP = "tcp"
KAMERA = "camera_color_optical_frame"
BOARD = os.environ.get("CHARUCO_BOARD_FRAME", "charuco_gemessen")
MIN_ECKEN = 9          # das 6x3-Board hat 10 innere Ecken

VERFAHREN = [("park", cv2.CALIB_HAND_EYE_PARK),
             ("horaud", cv2.CALIB_HAND_EYE_HORAUD),
             ("daniilidis", cv2.CALIB_HAND_EYE_DANIILIDIS),
             ("andreff", cv2.CALIB_HAND_EYE_ANDREFF)]


def main():
    ap = argparse.ArgumentParser()
    # Vorgabe ist die Sammlung im Repository: dort stehen die Stellungen, in denen
    # das Board am 2026-09-09 nachweislich vollstaendig erkannt wurde. Solange
    # Board und Kamera an ihrem Platz bleiben, muss nichts neu gesucht werden.
    ap.add_argument("--posen", default="/home/er/ros2_ws/kalibrierposen_charuco.json")
    ap.add_argument("--toleranz", type=float, default=1.0)
    ap.add_argument("--erwartete-hoehe", type=float, default=0.609,
                    help="unabhaengig gemessene Kamerahoehe [m], nur zur Plausibilitaet")
    ap.add_argument("--json", default="/tmp/handeye_auto.json")
    ap.add_argument("--dry-run", action="store_true")
    a = ap.parse_args()

    roh = json.load(open(a.posen))
    if isinstance(roh, dict):                  # Sammlung aus dem Repository
        posen = [e["q_grad"] for e in roh["stellungen"]]
        print("Stellungen aus %s (erzeugt %s)" % (os.path.basename(a.posen),
                                                  roh.get("erzeugt", "?")))
        for satz in roh.get("gilt_solange", []):
            print("   gilt, solange: %s" % satz)
    else:                                      # Ergebnis einer frischen Suche
        posen = [e["q_grad"] for e in roh]
    print("%d Stellungen geladen\n" % len(posen))
    if a.dry_run:
        for q in posen:
            print("   %s" % q)
        return 0

    rclpy.init()
    n = Messung()
    if not n.warte(lambda: n.js is not None, 20, "keine /joint_states"):
        return 1
    n.valid.wait_for_service(timeout_sec=10.0)
    n.traj.wait_for_server(timeout_sec=10.0)
    pos = Positionierer(n, traj_client=n.traj, validity_client=n.valid,
                        js_getter=lambda: n.gelenke() if n.js is not None else None)
    ecken = {"n": 0}
    n.create_subscription(Int32, "/charuco_detector/corners_detected",
                          lambda m: ecken.__setitem__("n", m.data), 10)

    R_b2g, t_b2g, R_t2c, t_t2c, proben = [], [], [], [], []
    for i, q_grad in enumerate(posen, 1):
        erg = pos.anfahren([math.radians(v) for v in q_grad],
                           toleranz_grad=a.toleranz, max_versuche=6, log=None)
        ecken["n"] = 0
        t0 = time.time(); best = 0
        while time.time() - t0 < 5:
            rclpy.spin_once(n, timeout_sec=0.05)
            best = max(best, ecken["n"])
        if best < MIN_ECKEN:
            print("  %2d/%d  nur %d Ecken - uebersprungen" % (i, len(posen), best))
            continue
        # Eye-to-hand: Basis im tcp-Frame abtasten
        Rr, tr = n.tf_mat(TCP, BASIS)
        Rc, tc = n.tf_mat(KAMERA, BOARD)
        if Rr is None or Rc is None:
            print("  %2d/%d  TF fehlt - uebersprungen" % (i, len(posen)))
            continue
        R_b2g.append(Rr); t_b2g.append(tr)
        R_t2c.append(Rc); t_t2c.append(tc)
        proben.append({"q_grad": q_grad, "ecken": best, "rest_grad": erg.rest_grad})
        print("  %2d/%d  Ecken %2d  Rest %.2f Grad  ok (%d Proben)"
              % (i, len(posen), best, erg.rest_grad, len(R_b2g)))

    if len(R_b2g) < 5:
        print("\nZu wenige Proben (%d) - die Kalibrierung waere nicht belastbar." % len(R_b2g))
        return 1

    print("\n" + "=" * 66)
    print("ERGEBNISSE  (%d Proben; Tsai-Lenz bewusst nicht dabei)" % len(R_b2g))
    print("=" * 66)
    print("%-12s %9s %9s %9s" % ("Verfahren", "x [m]", "y [m]", "z [m]"))
    ergebnisse = {}
    for name, flag in VERFAHREN:
        try:
            R, t = cv2.calibrateHandEye(R_b2g, t_b2g, R_t2c, t_t2c, method=flag)
            tv = np.asarray(t).flatten()
            ergebnisse[name] = (R, tv)
            print("%-12s %+9.4f %+9.4f %+9.4f" % (name, tv[0], tv[1], tv[2]))
        except Exception as e:                              # noqa: BLE001
            print("%-12s FEHLGESCHLAGEN - %s" % (name, e))

    if "park" not in ergebnisse:
        print("\nPark hat nicht gerechnet - Abbruch.")
        return 1
    R_park, t_park = ergebnisse["park"]

    print("\nAbweichung der uebrigen Verfahren von Park:")
    einig = True
    for name, (R, t) in ergebnisse.items():
        if name == "park":
            continue
        d = float(np.linalg.norm(t - t_park)) * 1000
        c = (np.trace(R.T @ R_park) - 1) / 2
        w = math.degrees(math.acos(max(-1.0, min(1.0, c))))
        marke = "ok" if d <= 10.0 else "ZU GROSS"
        if d > 10.0:
            einig = False
        print("   %-12s %6.1f mm / %5.2f Grad   %s" % (name, d, w, marke))

    print("\nPlausibilitaet der Hoehe: z = %.4f m, unabhaengig gemessen %.4f m, Differenz %.1f mm"
          % (t_park[2], a.erwartete_hoehe, abs(t_park[2] - a.erwartete_hoehe) * 1000))
    if not einig:
        print("\n⚠️ Die Verfahren sind sich NICHT einig (>10 mm). Das Ergebnis ist nicht "
              "belastbar - mehr oder besser verteilte Stellungen noetig.")

    with open(a.json, "w") as fh:
        json.dump({"proben": proben,
                   "park": {"R": R_park.tolist(), "t": t_park.tolist()},
                   "alle": {k: {"R": v[0].tolist(), "t": v[1].tolist()}
                            for k, v in ergebnisse.items()}}, fh, indent=2)
    print("\nJSON: %s" % a.json)
    print("\n(Es wurde noch NICHTS ins URDF oder in die .calib geschrieben.)")
    n.destroy_node(); rclpy.shutdown()
    return 0


if __name__ == "__main__":
    sys.exit(main())
