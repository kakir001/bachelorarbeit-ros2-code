#!/usr/bin/env python3
"""Misst den Modellfehler gegen das ChArUco-Board am Greifer.

Warum das besser ist als der Schwerpunkt
----------------------------------------
Die fruehere Messung (measure_model_accuracy.py) verglich den Schwerpunkt der
sichtbaren Tiefenpunkte. Der liegt systematisch neben dem Linkursprung, und der
Versatz aendert sich mit der Stellung - herausgekommen ist deshalb nur eine
OBERE SCHRANKE von 7.6 bis 27.0 mm, in der Messverzerrung und Modellfehler nicht
zu trennen waren.

Das ChArUco-Board hat bekannte Geometrie. Die Kamera liefert daraus eine
vollstaendige 6-DoF-Lage, ohne Schwerpunkt und ohne dessen Verzerrung.

Das Problem des unbekannten Befestigungsmasses - und wie es geloest wird
-----------------------------------------------------------------------
Wo genau die Platte am Greifer sitzt, ist nicht gemessen; im URDF stehen
geschaetzte Werte. Ein absoluter Vergleich wuerde also vor allem diese Schaetzung
messen. Deshalb wird die Differenz aufgeteilt:

    T_gemessen = T_modell * X  +  Fehler(Stellung)

X ist der feste Befestigungsversatz - unbekannt, aber in allen Stellungen
derselbe. Er wird aus den Messungen geschaetzt (Mittel ueber alle Stellungen)
und abgezogen. Was uebrig bleibt, haengt von der Stellung ab und ist der
eigentliche MODELLFEHLER - die gesuchte Zahl.

Nebenbei faellt X als gemessenes Befestigungsmass ab und kann direkt ins URDF.

Voraussetzungen
---------------
- Der charuco_detector laeuft mit charuco_params_gripper.yaml und
  board_frame:=charuco_gemessen (NICHT charuco_board - das ist der Modell-Link).
- Die Kamera liefert 1280x720. Bei 424x240 ist ein 12-mm-Marker rund 10 px gross;
  DICT_4X4 braucht 6x6 Zellen, das sind unter 2 px je Zelle und wird nicht erkannt.
- Jede Stellung wird nachgefuehrt, sonst misst man den Ankunftsfehler der Servos.

Aufruf
------
    python3 tools/measure_model_vs_charuco.py --dry-run
    python3 tools/measure_model_vs_charuco.py --stellungen 8 --json /tmp/mess.json
"""
import argparse
import json
import math
import sys
import time

import numpy as np
import rclpy

sys.path.insert(0, "/home/er/ros2_ws/tools")
from measure_deflection_camera import Messung  # noqa: E402
sys.path.insert(0, "/home/er/ros2_ws/src/mycobot_calibration")
from mycobot_calibration.positionieren import Positionierer  # noqa: E402
from std_msgs.msg import Int32  # noqa: E402


def mat(R, t):
    T = np.eye(4); T[:3, :3] = R; T[:3, 3] = t
    return T


def dreh_winkel(R):
    """Betrag der Drehung einer Rotationsmatrix in Grad."""
    c = (np.trace(R) - 1.0) / 2.0
    return float(np.degrees(np.arccos(np.clip(c, -1.0, 1.0))))


def mittel_transform(Ts):
    """Mittelt eine Liste homogener Transformationen (Rotation ueber SVD)."""
    t = np.mean([T[:3, 3] for T in Ts], axis=0)
    M = np.sum([T[:3, :3] for T in Ts], axis=0)
    U, _, Vt = np.linalg.svd(M)
    R = U @ Vt
    if np.linalg.det(R) < 0:
        U[:, -1] *= -1
        R = U @ Vt
    return mat(R, t)


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--stellungen", type=int, default=8)
    ap.add_argument("--streuung", type=float, default=10.0,
                    help="wie weit die Stellungen um die Ausgangslage streuen [Grad]")
    ap.add_argument("--toleranz", type=float, default=1.0)
    # Vorgabe 10 (also ALLE Ecken), nicht 6: am 2026-09-10 gemessen, wie stark eine
    # nur teilweise erkannte Platte die Lageschaetzung verdirbt - Stellungen mit 7
    # bis 8 Ecken kamen auf 8.12 mm Rest, die mit 10 Ecken auf 2.52 mm. Dieselbe
    # Kalibrierung, dieselbe Stunde: der Unterschied ist die Platte im Bild, nicht
    # das Modell. Wer weniger fordert, misst vor allem seine eigene Eckenausbeute.
    ap.add_argument("--min-ecken", type=int, default=10)
    ap.add_argument("--dry-run", action="store_true")
    ap.add_argument("--json", default="")
    a = ap.parse_args()

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

    basis = [math.degrees(v) for v in n.gelenke()]
    print("Ausgangsstellung (Grad): %s" % [round(v, 1) for v in basis])
    print("Es wird von HIER aus gestreut - das Board muss sichtbar bleiben.\n")

    # Kleine Auslenkungen um die Ausgangslage; das Board bleibt dabei im Bild.
    rng = np.random.default_rng(20260909)
    kandidaten = [list(basis)]
    while len(kandidaten) < a.stellungen * 3:
        d = rng.uniform(-a.streuung, a.streuung, 6)
        d[5] *= 0.5                      # Handgelenksrolle vorsichtiger
        kandidaten.append([basis[i] + d[i] for i in range(6)])

    plan = []
    for q_grad in kandidaten:
        if len(plan) >= a.stellungen:
            break
        q = [math.radians(v) for v in q_grad]
        if pos.zulaessig(q):
            plan.append(q_grad)
    print("%d zulaessige Stellungen\n" % len(plan))
    if a.dry_run:
        for q in plan:
            print("   %s" % [round(v, 1) for v in q])
        return 0

    messungen = []
    for i, q_grad in enumerate(plan, 1):
        erg = pos.anfahren([math.radians(v) for v in q_grad],
                           toleranz_grad=a.toleranz, max_versuche=6, log=None)

        # (1) Stellungen, die die Nachfuehrung NICHT eingefangen hat, wegwerfen.
        # Bis 2026-09-10 wurden sie mitgezaehlt: die Zeile darunter druckte
        # bedingungslos "ok". In jenem Lauf blieben zwei Stellungen 8.7 und 12.3 Grad
        # daneben - und genau die beiden lieferten die groessten "Modellfehler"
        # (72 und 31 mm). Was da gemessen wurde, war der Ankunftsfehler der Servos,
        # nicht das Modell.
        if erg.rest_grad > a.toleranz:
            print("  %2d/%d  Nachfuehrung blieb %.2f Grad daneben (erlaubt %.2f) - verworfen"
                  % (i, len(plan), erg.rest_grad, a.toleranz))
            continue

        # (2) Erst messen, wenn der Arm WIRKLICH steht. Die Kamera zeigt ein Bild von
        # vor rund 100 ms, die TF den neuesten Gelenkstand - faehrt der Arm noch, sind
        # beide Seiten verschiedene Stellungen, und die Differenz landet als
        # "Modellfehler" in der Rechnung.
        letzte = None; ruhig_seit = None; t0 = time.time()
        while time.time() - t0 < 20.0:
            for _ in range(10):
                rclpy.spin_once(n, timeout_sec=0.05)
            jetzt = n.gelenke() if hasattr(n, "gelenke") else None
            if jetzt is None:
                break
            if letzte is not None:
                d = max(abs(math.degrees(x - y)) for x, y in zip(jetzt, letzte))
                if d < 0.15:
                    if ruhig_seit is None:
                        ruhig_seit = time.time()
                    elif time.time() - ruhig_seit > 2.0:
                        break
                else:
                    ruhig_seit = None
            letzte = jetzt

        t0 = time.time(); best = 0
        while time.time() - t0 < 5:
            rclpy.spin_once(n, timeout_sec=0.05)
            best = max(best, ecken["n"])
        if best < a.min_ecken:
            print("  %2d/%d  nur %d Ecken - verworfen" % (i, len(plan), best))
            continue
        R_m, t_m = n.tf_mat("robot_base", "charuco_board")       # Modell
        R_g, t_g = n.tf_mat("robot_base", "charuco_gemessen")    # Kamera
        if R_m is None or R_g is None:
            print("  %2d/%d  TF fehlt - verworfen" % (i, len(plan)))
            continue
        messungen.append({"q_grad": [round(v, 2) for v in q_grad],
                          "rest_grad": erg.rest_grad, "ecken": best,
                          "modell": mat(R_m, t_m).tolist(),
                          "gemessen": mat(R_g, t_g).tolist()})
        print("  %2d/%d  Ecken %2d  Rest %.2f Grad  ok" % (i, len(plan), best, erg.rest_grad))

    if len(messungen) < 3:
        print("\nZu wenige gueltige Messungen (%d)." % len(messungen))
        return 1

    # --- Befestigungsversatz X schaetzen und abziehen ---
    Xs = [np.linalg.inv(np.array(m["modell"])) @ np.array(m["gemessen"]) for m in messungen]
    X = mittel_transform(Xs)
    print("\n" + "=" * 70)
    print("GESCHAETZTER BEFESTIGUNGSVERSATZ X (Modell-Board -> echtes Board)")
    print("=" * 70)
    print("  Verschiebung: %s mm" % np.round(X[:3, 3] * 1000, 1))
    print("  Drehung     : %.2f Grad" % dreh_winkel(X[:3, :3]))

    print("\n" + "=" * 70)
    print("RESTFEHLER NACH ABZUG VON X  =  MODELLFEHLER")
    print("=" * 70)
    print("%-4s %8s %10s %s" % ("Nr", "mm", "Grad", "Stellung"))
    d_mm, d_grad = [], []
    for k, m in enumerate(messungen, 1):
        vorher = np.array(m["modell"]) @ X
        rest = np.linalg.inv(vorher) @ np.array(m["gemessen"])
        mm = float(np.linalg.norm(rest[:3, 3]) * 1000)
        gr = dreh_winkel(rest[:3, :3])
        d_mm.append(mm); d_grad.append(gr)
        print("%-4d %8.2f %10.2f  %s" % (k, mm, gr, m["q_grad"]))
    print("-" * 70)
    print("Modellfehler: Mittel %.2f mm / %.2f Grad,  groesster %.2f mm / %.2f Grad,  n=%d"
          % (float(np.mean(d_mm)), float(np.mean(d_grad)),
             float(np.max(d_mm)), float(np.max(d_grad)), len(d_mm)))

    if a.json:
        with open(a.json, "w") as fh:
            json.dump({"X": X.tolist(), "messungen": messungen,
                       "rest_mm": d_mm, "rest_grad": d_grad}, fh, indent=2)
        print("\nJSON: %s" % a.json)
    n.destroy_node(); rclpy.shutdown()
    return 0


if __name__ == "__main__":
    sys.exit(main())
