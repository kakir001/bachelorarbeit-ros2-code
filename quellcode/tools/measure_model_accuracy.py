#!/usr/bin/env python3
"""Misst, wie genau das Modell den ECHTEN Roboter beschreibt.

Die Frage
---------
Die Meshes stammen unveraendert vom Hersteller, die Kinematik wurde nie
unabhaengig nachgemessen. Wie gross ist der Fehler des Modells wirklich?

Warum kein absoluter Vergleich
------------------------------
Naheliegend waere: "das Modell sagt, der Flansch ist bei X, die Kamera misst Y,
Fehler = |X-Y|". Das waere falsch. Gemessen wird der Schwerpunkt der SICHTBAREN
Punkte, und die Kamera sieht je nach Stellung andere Flaechen - der Schwerpunkt
liegt also systematisch neben dem Linkursprung, und dieser Versatz aendert sich
mit der Stellung. Ein absoluter Vergleich misst vor allem diesen Versatz, nicht
das Modell.

Was stattdessen gemessen wird
-----------------------------
Eine VERSCHIEBUNG bei GLEICHBLEIBENDER Ausrichtung. Der TCP wird um eine bekannte
Strecke versetzt (die IK haelt den Greifer dabei senkrecht nach unten), und es
wird verglichen:

    Delta_Modell  = Flanschlage laut TF        (Ziel minus Ausgangsstellung)
    Delta_Kamera  = gemessener Schwerpunkt     (Ziel minus Ausgangsstellung)

Weil die Ausrichtung gleich bleibt, sieht die Kamera dieselben Flaechen; der
Versatz faellt in der Differenz weitgehend heraus. Uebrig bleibt, worin sich
Modell und Wirklichkeit ueber diese Strecke unterscheiden.

Zwei Fehlerquellen sauber trennen
---------------------------------
Ohne Nachfuehrung wuerde hier der ANKUNFTSFEHLER der Servos gemessen (bis 15 Grad,
2026-09-08) statt des Modellfehlers. Deshalb wird jede Stellung mit dem
Positionierer nachgefuehrt, bis der Encoder das Ziel bestaetigt. Was dann noch
bleibt, ist Modell + alles, was der Encoder nicht sieht (Getriebespiel,
Nachgeben unter Last) - und genau das ist die interessante Groesse.

Aufruf
------
    python3 tools/measure_model_accuracy.py --dry-run
    python3 tools/measure_model_accuracy.py --strecke 0.03 --json /tmp/modell.json
"""
import argparse
import json
import math
import sys
import time

import numpy as np
import rclpy

sys.path.insert(0, "/home/er/ros2_ws/tools")
from measure_deflection_camera import Messung, greifer_punkte, kennwerte  # noqa: E402
sys.path.insert(0, "/home/er/ros2_ws/src/mycobot_calibration")
from mycobot_calibration.numerical_ik import ARM_JOINTS  # noqa: E402
from mycobot_calibration.positionieren import Positionierer  # noqa: E402
from moveit_msgs.srv import GetPositionIK  # noqa: E402
from sensor_msgs.msg import JointState  # noqa: E402
from geometry_msgs.msg import PoseStamped  # noqa: E402

# Ausgangsstellung in GRAD. Vielfach gefahren und geprueft; der Flansch steht
# dabei gut sichtbar vor der Kamera.
AUSGANG_GRAD = [-10.0, -20.0, 0.0, -70.0, 0.0, 0.0]


def ik_loesen(n, ik_client, position, orientierung, seed):
    """Gelenkloesung fuer eine Lage ueber MoveIt (TRAC-IK).

    Der eigene numerische Loeser (numerical_ik) braucht auf dem Jetson rund 30 s
    je Aufruf und fand fuer die hier gewuenschten Lagen keine Loesung. MoveIt
    antwortet in Sekundenbruchteilen und rechnet ausserdem mit genau dem Modell,
    mit dem auch geplant wird.

    Die ORIENTIERUNG wird unveraendert uebergeben - darauf beruht die ganze
    Messung: nur wenn der Greifer gleich ausgerichtet bleibt, sieht die Kamera
    dieselben Flaechen und der Schwerpunkt-Versatz faellt in der Differenz heraus.
    """
    req = GetPositionIK.Request()
    req.ik_request.group_name = "arm"
    req.ik_request.ik_link_name = "tcp"
    req.ik_request.avoid_collisions = True
    req.ik_request.timeout.sec = 2
    js = JointState()
    js.name = list(ARM_JOINTS)
    js.position = [float(v) for v in seed]
    req.ik_request.robot_state.joint_state = js
    ps = PoseStamped()
    ps.header.frame_id = "robot_base"
    ps.pose.position.x, ps.pose.position.y, ps.pose.position.z = [float(v) for v in position]
    (ps.pose.orientation.x, ps.pose.orientation.y,
     ps.pose.orientation.z, ps.pose.orientation.w) = [float(v) for v in orientierung]
    req.ik_request.pose_stamped = ps
    fut = ik_client.call_async(req)
    rclpy.spin_until_future_complete(n, fut, timeout_sec=10.0)
    r = fut.result()
    if r is None or r.error_code.val != 1:
        return None
    d = dict(zip(r.solution.joint_state.name, r.solution.joint_state.position))
    try:
        return [d[j] for j in ARM_JOINTS]
    except KeyError:
        return None


def messe(n, p, bezug, halb, frames):
    """Modellage laut TF und gemessener Schwerpunkt der sichtbaren Punkte."""
    _, modell = n.tf_mat("robot_base", bezug)
    if modell is None:
        return None, None, 0
    modell = np.array(modell)
    B = n.wolke(frames=frames)
    if B is None or B.shape[0] == 0:
        return modell, None, 0
    P = greifer_punkte(B, modell, halb=halb, hoch=halb)
    k = kennwerte(P)
    if k is None:
        return modell, None, int(P.shape[0])
    return modell, k["schwerpunkt"], k["n"]


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--strecke", type=float, default=0.030,
                    help="Laenge der Verschiebung je Achse [m]")
    ap.add_argument("--bezug", default="joint6_flange",
                    help="welches Frame vermessen wird (am Flansch sind genug Punkte)")
    ap.add_argument("--halb", type=float, default=0.055, help="halbe Kantenlaenge des Messwuerfels")
    ap.add_argument("--frames", type=int, default=25)
    ap.add_argument("--toleranz", type=float, default=0.5, help="Nachfuehrung bis auf x Grad")
    ap.add_argument("--dry-run", action="store_true")
    ap.add_argument("--json", default="")
    a = ap.parse_args()

    rclpy.init()
    n = Messung()
    if not n.warte(lambda: n.js is not None, 20, "keine /joint_states"):
        return 1
    if not n.warte(lambda: n.tiefe is not None and n.info is not None, 25, "keine Tiefenbilder"):
        return 1
    if not n.valid.wait_for_service(timeout_sec=10.0):
        print("FEHLER: /check_state_validity fehlt"); return 1
    if not n.traj.wait_for_server(timeout_sec=10.0):
        print("FEHLER: arm_controller fehlt"); return 1

    d = a.strecke
    ik_client = n.create_client(GetPositionIK, "/compute_ik")
    if not ik_client.wait_for_service(timeout_sec=10.0):
        print("FEHLER: /compute_ik fehlt"); return 1
    pos = Positionierer(n, traj_client=n.traj, validity_client=n.valid,
                        js_getter=lambda: n.gelenke() if n.js is not None else None)

    # Erst in die Ausgangsstellung, dann von DORT aus messen. Lage und
    # Ausrichtung des TCP werden dabei ausgelesen - die Ausrichtung bleibt fuer
    # alle weiteren Stellungen unveraendert.
    print("Ausgangsstellung anfahren ...")
    q_aus = [math.radians(v) for v in AUSGANG_GRAD]
    erg = pos.anfahren(q_aus, toleranz_grad=a.toleranz, log=print)
    print("  %s\n" % erg.text())
    R, BASIS = n.tf_mat("robot_base", "tcp")
    if R is None:
        print("FEHLER: keine TF robot_base->tcp"); return 1
    BASIS = np.array(BASIS)
    # Rotationsmatrix -> Quaternion
    sp = np.trace(R)
    if sp > 0:
        w = math.sqrt(1.0 + sp) / 2.0
        quat = [(R[2,1]-R[1,2])/(4*w), (R[0,2]-R[2,0])/(4*w), (R[1,0]-R[0,1])/(4*w), w]
    else:
        i = int(np.argmax(np.diag(R))); j, k = (i+1) % 3, (i+2) % 3
        t = math.sqrt(max(1e-12, 1.0 + R[i,i] - R[j,j] - R[k,k]))
        q = [0.0]*4; q[i] = t/2.0
        q[j] = (R[j,i]+R[i,j])/(2*t); q[k] = (R[k,i]+R[i,k])/(2*t)
        quat = q[:3] + [(R[k,j]-R[j,k])/(2*t)]

    stellungen = [("Ausgang", np.zeros(3))]
    for achse, i in (("x", 0), ("y", 1), ("z", 2)):
        for vz in (+1, -1):
            v = np.zeros(3); v[i] = vz * d
            stellungen.append(("%s%+.0fmm" % (achse, vz * d * 1000), v))
    stellungen.append(("Ausgang (Wiederholung)", np.zeros(3)))

    print("Ausgangslage TCP: %s m,  Verschiebung je Achse: %.0f mm\n"
          % (np.round(BASIS, 3), d * 1000))
    print("Ausrichtung bleibt fuer alle Stellungen: %s\n" % np.round(quat, 4))

    # --- IK fuer alle Stellungen vorab, damit nichts halb gefahren wird ---
    plan = []
    q_vor = n.gelenke()
    for name, v in stellungen:
        ziel = BASIS + v
        q = ik_loesen(n, ik_client, ziel, quat, q_vor)
        if q is None:
            print("  %-22s IK findet keine Loesung -> wird ausgelassen" % name)
            continue
        if not pos.weg_zulaessig(q_vor, list(q)):
            print("  %-22s Weg unzulaessig -> wird ausgelassen" % name)
            continue
        print("  %-22s Ziel %s m  ok" % (name, np.round(ziel, 3)))
        plan.append((name, ziel, list(q)))
        q_vor = list(q)
    if len(plan) < 3:
        print("\nZu wenige erreichbare Stellungen - Abbruch.")
        return 1
    if a.dry_run:
        print("\n--dry-run: es wird nicht gefahren.")
        return 0

    ergebnis = []
    for name, ziel, q in plan:
        print("\n=== %s ===" % name)
        erg = pos.anfahren(q, toleranz_grad=a.toleranz, weg_pruefen=True, log=print)
        print("  %s" % erg.text())
        modell, gemessen, anzahl = messe(n, pos, a.bezug, a.halb, a.frames)
        if gemessen is None:
            print("  Kamera: zu wenige Punkte (%d) - Stellung wird verworfen" % anzahl)
            continue
        print("  Modell   %s m" % np.round(modell, 4))
        print("  Kamera   %s m   (%d Punkte)" % (np.round(gemessen, 4), anzahl))
        ergebnis.append({"name": name, "ziel": ziel.tolist(), "rest_grad": erg.rest_grad,
                         "modell": modell.tolist(), "kamera": gemessen.tolist(), "n": anzahl})

    # --- Auswertung: Verschiebungen vergleichen ---
    basis = next((e for e in ergebnis if e["name"] == "Ausgang"), None)
    if basis is None:
        print("\nAusgangsmessung fehlt - keine Auswertung moeglich.")
        return 1
    m0, k0 = np.array(basis["modell"]), np.array(basis["kamera"])

    print("\n" + "=" * 72)
    print("VERSCHIEBUNG: Modell gegen Kamera   (alles in mm)")
    print("=" * 72)
    print("%-24s %-22s %-22s %s" % ("Stellung", "Modell dx dy dz", "Kamera dx dy dz", "Abweichung"))
    abw = []
    for e in ergebnis:
        if e is basis:
            continue
        dm = (np.array(e["modell"]) - m0) * 1000
        dk = (np.array(e["kamera"]) - k0) * 1000
        d3 = dk - dm
        betrag = float(np.linalg.norm(d3))
        if e["name"].startswith("Ausgang"):
            print("%-24s %-22s %-22s %6.2f  <- Rauschen des Verfahrens"
                  % (e["name"], np.round(dm, 1), np.round(dk, 1), betrag))
        else:
            abw.append(betrag)
            print("%-24s %-22s %-22s %6.2f" % (e["name"], np.round(dm, 1), np.round(dk, 1), betrag))
    if abw:
        print("-" * 72)
        print("Abweichung Modell<->Wirklichkeit ueber %.0f mm Verschiebung:" % (d * 1000))
        print("   Mittel %.2f mm,  groesster Wert %.2f mm,  n = %d"
              % (float(np.mean(abw)), float(np.max(abw)), len(abw)))

    if a.json:
        with open(a.json, "w") as fh:
            json.dump({"strecke_m": d, "bezug": a.bezug, "messungen": ergebnis}, fh, indent=2)
        print("\nJSON: %s" % a.json)

    n.destroy_node()
    rclpy.shutdown()
    return 0


if __name__ == "__main__":
    sys.exit(main())
