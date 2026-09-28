#!/usr/bin/env python3
"""Misst, wie viel Spiel beim ANFAHREN uebrig bleibt — je nach Anfahrrichtung.

Die Frage dahinter
------------------
Elephant Robotics hat am 2026-09-08 bestaetigt: das Spiel an Achse 2 ist eine
Eigenschaft der Servos und nicht zu beheben. Damit ist nicht mehr die Frage, ob
es weg geht, sondern WIE VIEL es beim Positionieren kostet — und ob es hilft,
eine Zielstellung immer aus derselben Richtung anzufahren.

Verfahren
---------
Dieselbe Zielstellung wird abwechselnd von OBEN (Achse 2 groesser) und von UNTEN
(Achse 2 kleiner) angefahren, je mehrfach. Nach jeder Fahrt wird der Flansch mit
der Tiefenkamera vermessen.

    Abstand der beiden Mittelwerte   = Spiel, wie es sich beim Anfahren auswirkt
    Streuung innerhalb einer Richtung = Wiederholgenauigkeit dieser Richtung

Warum der Vergleich sauber ist
------------------------------
Gemessen wird der Schwerpunkt der Punkte in einem Wuerfel um die vom Modell
vorhergesagte Flanschlage. Dieses Mass ist zwischen VERSCHIEDENEN Stellungen
verzerrt, weil die Kamera je nach Stellung andere Flaechen sieht. Hier wird aber
immer DIESELBE Zielstellung verglichen — die Verzerrung ist in beiden
Anfahrrichtungen dieselbe und faellt in der Differenz heraus. Uebrig bleibt das,
was sich zwischen den Anfahrrichtungen wirklich unterscheidet.

Was das NICHT misst: den absoluten Fehler des Modells (z.B. den versetzten
J2-Nullpunkt). Dafuer braucht es ein Ziel bekannter Geometrie am Greifer —
das ChArUco-Board — und nicht diesen Schwerpunkt.

Aufruf
------
    python3 tools/measure_approach_backlash.py --dry-run
    python3 tools/measure_approach_backlash.py --wiederholungen 4
"""
import argparse
import json
import math
import sys
import time

import numpy as np
import rclpy
from control_msgs.action import FollowJointTrajectory
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint

sys.path.insert(0, "/home/er/ros2_ws/tools")
from measure_deflection_camera import Messung, greifer_punkte, kennwerte  # noqa: E402
sys.path.insert(0, "/home/er/ros2_ws/src/mycobot_calibration")
from mycobot_calibration.numerical_ik import ARM_JOINTS  # noqa: E402

# Zielstellungen in GRAD. Erste ist die am 2026-09-08 gepruefte Messpose.
ZIELE = [
    ("Pose A (Flansch ~455 mm vor der Kamera)", [-10.0, -20.0, 0.0, -70.0, 0.0, 0.0]),
    ("Pose B (Arm weiter ausgestreckt, mehr Moment auf Achse 2)",
     [-10.0, -34.0, 12.0, -62.0, 0.0, 0.0]),
]


def fahre(n, q_grad, sekunden):
    g = FollowJointTrajectory.Goal()
    jt = JointTrajectory()
    jt.joint_names = list(ARM_JOINTS)
    pt = JointTrajectoryPoint()
    pt.positions = [math.radians(v) for v in q_grad]
    pt.time_from_start.sec = int(sekunden)
    pt.time_from_start.nanosec = int((sekunden - int(sekunden)) * 1e9)
    jt.points = [pt]
    g.trajectory = jt
    fut = n.traj.send_goal_async(g)
    rclpy.spin_until_future_complete(n, fut, timeout_sec=15.0)
    h = fut.result()
    if h is None or not h.accepted:
        return False
    erg = h.get_result_async()
    rclpy.spin_until_future_complete(n, erg, timeout_sec=sekunden + 20)
    return True


def ruhen(n, sekunden):
    """Warten und dabei SPINNEN — sonst bleibt der TF-Puffer stehen."""
    t0 = time.time()
    while time.time() - t0 < sekunden:
        rclpy.spin_once(n, timeout_sec=0.05)


def einschwingen(n, max_s=25.0, ruhe_s=2.0, schwelle_grad=0.06):
    """Warten, BIS die Gelenke wirklich stillstehen.

    Der Trajektorien-Controller meldet fertig, wenn seine Zeit abgelaufen ist —
    die Servos fahren danach oft noch weiter (send_radians ist offene Schleife mit
    fester Geschwindigkeit). Beim ersten Durchlauf am 2026-09-08 wurde deshalb
    gemessen, waehrend der Arm noch unterwegs war: bis zu 9.3 Grad neben dem Ziel.
    Hier wird gewartet, bis sich ueber ruhe_s Sekunden nichts mehr aendert."""
    letzte = None
    still_seit = None
    t0 = time.time()
    q = None
    while time.time() - t0 < max_s:
        rclpy.spin_once(n, timeout_sec=0.05)
        if n.js is None:
            continue
        q = [math.degrees(v) for v in n.gelenke()]
        if letzte is not None:
            if max(abs(q[k] - letzte[k]) for k in range(6)) < schwelle_grad:
                if still_seit is None:
                    still_seit = time.time()
                elif time.time() - still_seit >= ruhe_s:
                    return q, time.time() - t0, True
            else:
                still_seit = None
        letzte = q
    return q, time.time() - t0, False


def weg_zulaessig(n, q_von, q_nach, schritte=15):
    for i in range(schritte + 1):
        s = i / schritte
        qi = [(1 - s) * q_von[k] + s * q_nach[k] for k in range(6)]
        if not n.zulaessig([math.radians(v) for v in qi]):
            return False, round(100 * s)
    return True, None


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--wiederholungen", type=int, default=3)
    ap.add_argument("--vorhalt", type=float, default=8.0,
                    help="um wie viel Grad Achse 2 vor dem Ziel ausgelenkt wird")
    ap.add_argument("--sekunden", type=float, default=7.0)
    ap.add_argument("--ruhe", type=float, default=2.0,
                    help="wie lange die Gelenke stillstehen muessen, bevor gemessen wird")
    ap.add_argument("--halb", type=float, default=0.055)
    ap.add_argument("--frame", default="joint6_flange")
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
        print("FEHLER: /check_state_validity nicht da")
        return 1
    if not n.traj.wait_for_server(timeout_sec=10.0):
        print("FEHLER: arm_controller action nicht da")
        return 1

    start = [math.degrees(v) for v in n.gelenke()]
    print("Start (Grad): %s" % [round(v, 1) for v in start])

    # --- alle noetigen Wege vorher pruefen, es wird sonst gar nicht gefahren ---
    print("\nWege pruefen ...")
    plan = []
    vorherige = start
    for name, ziel in ZIELE:
        eintraege = []
        ok_ziel = True
        for vz, richtung in ((+1, "von OBEN"), (-1, "von UNTEN")):
            anfahrt = list(ziel)
            anfahrt[1] = ziel[1] + vz * a.vorhalt          # Achse 2 vorhalten
            for q_von, q_nach in ((vorherige, anfahrt), (anfahrt, ziel)):
                ok, wo = weg_zulaessig(n, q_von, q_nach)
                if not ok:
                    print("  %s / %s: unzulaessig bei %s %% des Weges" % (name, richtung, wo))
                    ok_ziel = False
                    break
            if not ok_ziel:
                break
            eintraege.append((richtung, anfahrt))
            vorherige = ziel
        if ok_ziel:
            plan.append((name, ziel, eintraege))
            print("  %s: beide Anfahrrichtungen zulaessig" % name)
    if not plan:
        print("\nKeine der Zielstellungen ist sicher erreichbar — es wird nichts gefahren.")
        return 2

    fahrten = sum(len(e) for _, _, e in plan) * a.wiederholungen
    print("\n%d Zielstellung(en), 2 Anfahrrichtungen, %d Wiederholungen = %d Fahrten"
          % (len(plan), a.wiederholungen, fahrten))
    print("geschaetzte Dauer: ~%.0f min" % (fahrten * (2 * a.sekunden + a.ruhe + 9) / 60.0))
    if a.dry_run:
        print("\n--dry-run: es wird NICHT gefahren.")
        return 0

    ergebnis = {}
    for name, ziel, eintraege in plan:
        print("\n" + "=" * 68)
        print(" %s   Ziel (Grad): %s" % (name, ziel))
        print("=" * 68)
        messungen = {}
        for richtung, anfahrt in eintraege:
            punkte = []
            bleibt = []
            for w in range(a.wiederholungen):
                if not fahre(n, anfahrt, a.sekunden):
                    print("  Fahrt abgelehnt."); return 1
                ruhen(n, 0.8)
                if not fahre(n, ziel, a.sekunden):
                    print("  Fahrt abgelehnt."); return 1
                q_ist, wartezeit, still = einschwingen(n, ruhe_s=a.ruhe)
                abw = max(abs(q_ist[k] - ziel[k]) for k in range(6))
                if not still:
                    print("  WARNUNG: Gelenke standen nach %.0f s noch nicht still" % wartezeit)
                R, mitte = n.tf_mat("robot_base", a.frame)
                B = n.wolke(frames=25)
                if B is None:
                    print("  keine Wolke — uebersprungen"); continue
                kw = kennwerte(greifer_punkte(B, mitte, halb=a.halb, hoch=a.halb))
                if kw is None:
                    print("  zu wenige Punkte — uebersprungen"); continue
                punkte.append(kw["schwerpunkt"])
                print("  %-10s %d/%d: Schwerpunkt %s mm | Gelenkabw. %.2f Grad | %d Pkt | "
                      "still nach %.1f s"
                      % (richtung, w + 1, a.wiederholungen,
                         np.round(1000 * kw["schwerpunkt"], 1), abw, kw["n"], wartezeit))
                bleibt.append(abw)
            if punkte:
                messungen[richtung] = np.array(punkte)
                print("  %-10s Gelenkabweichung vom Ziel nach dem Einschwingen: "
                      "%.2f Grad (Mittel), max %.2f" % (richtung, float(np.mean(bleibt)),
                                                        float(np.max(bleibt))))
        ergebnis[name] = messungen

        if len(messungen) == 2:
            o = messungen["von OBEN"]
            u = messungen["von UNTEN"]
            d = 1000 * (o.mean(axis=0) - u.mean(axis=0))
            print("\n  --- Auswertung %s ---" % name)
            print("  Wiederholgenauigkeit von OBEN : %.2f mm (Spanne %.2f mm)"
                  % (1000 * np.linalg.norm(o.std(axis=0)),
                     1000 * max(np.ptp(o, axis=0))))
            print("  Wiederholgenauigkeit von UNTEN: %.2f mm (Spanne %.2f mm)"
                  % (1000 * np.linalg.norm(u.std(axis=0)),
                     1000 * max(np.ptp(u, axis=0))))
            print("  UNTERSCHIED der Anfahrrichtungen: dx %+.2f  dy %+.2f  dz %+.2f mm"
                  % (d[0], d[1], d[2]))
            print("  -> Betrag %.2f mm  = das Spiel, wie es sich beim Anfahren auswirkt"
                  % np.linalg.norm(d))
            print("  -> so viel spart eine Prozedur, die IMMER aus derselben Richtung anfaehrt.")

    if a.json:
        with open(a.json, "w") as f:
            json.dump({k: {r: v.tolist() for r, v in m.items()} for k, m in ergebnis.items()},
                      f, indent=2)
        print("\nJSON: %s" % a.json)
    n.destroy_node()
    rclpy.shutdown()
    return 0


if __name__ == "__main__":
    sys.exit(main())
