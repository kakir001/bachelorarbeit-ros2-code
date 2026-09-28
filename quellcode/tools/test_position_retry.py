#!/usr/bin/env python3
"""Prueft, ob ERNEUTES SENDEN desselben Ziels den Ankunftsfehler verkleinert.

Die Frage dahinter
------------------
Am 2026-09-08 wurde gemessen: die Gelenke stehen nach dem Einschwingen bis zu
8.2 Grad neben dem kommandierten Ziel — und der Fehler haengt von der
Anfahrrichtung ab (Pose A: 1.8 Grad von oben, 6.7 Grad von unten). Das ist kein
Spiel (Spiel waere symmetrisch), sondern ein bleibender Regelfehler: der Servo,
der gegen die Schwerkraft faehrt, kommt zu kurz.

Die Bruecke (send_radians) sendet das Ziel EINMAL, in offener Schleife. Kommt der
Servo zu kurz, korrigiert niemand. Dieses Werkzeug prueft die billigste
denkbare Abhilfe: dasselbe Ziel einfach noch einmal schicken.

Verfahren
---------
Je Zielstellung wird bewusst VON UNTEN angefahren (dort war der Fehler am
groessten). Danach in einer Schleife:

    einschwingen  ->  Fehler messen  ->  dasselbe Ziel erneut senden

bis der Fehler unter der Toleranz liegt oder die Versuche aufgebraucht sind.

Zwei Betriebsarten
------------------
--korrektur 0   sendet WORTGLEICH dasselbe Ziel noch einmal.
--korrektur k   sendet ein KORRIGIERTES Ziel und rechnet den Fehler auf das
                Kommando auf:   kommando <- kommando - k * (ist - ziel)
                Das wirkt wie ein I-Anteil: das Kommando laeuft so weit ueber
                das Ziel hinaus, wie der Servo unter der Last zurueckbleibt.

Warum das noetig ist (gemessen 2026-09-08)
------------------------------------------
Bei --korrektur 0 blieb ein Versuch wirkungslos: Versuch 1 und 2 lieferten auf
zwei Nachkommastellen dieselben Winkel. Der Grund steht in
mycobot_hardware.cpp:373 — die Aenderungserkennung vergleicht das neue Kommando
mit dem ZULETZT GESENDETEN Kommando, nicht mit der GEMESSENEN Lage. Ein
unveraendertes Ziel faellt damit still unter den Tisch, auch wenn der Arm 9 Grad
daneben steht. Ein korrigiertes Ziel ist zwangslaeufig ein anderer Wert und geht
deshalb durch.

Was das zeigt
-------------
Faellt der Fehler mit jedem Versuch, ist eine aeussere Positionsschleife in der
Bruecke die richtige Antwort. Bleibt er stehen, ist der Servo am Drehmomentlimit
und es hilft nur weniger Last oder Schwerkraftausgleich.

Aufruf
------
    python3 tools/test_position_retry.py --dry-run
    python3 tools/test_position_retry.py --versuche 5 --toleranz 0.5
"""
import argparse
import json
import math
import sys
import time

import rclpy

sys.path.insert(0, "/home/er/ros2_ws/tools")
from measure_approach_backlash import einschwingen, fahre, weg_zulaessig  # noqa: E402
from measure_deflection_camera import Messung  # noqa: E402
sys.path.insert(0, "/home/er/ros2_ws/src/mycobot_calibration")
from mycobot_calibration.numerical_ik import ARM_JOINTS  # noqa: E402

# Zielstellungen in GRAD — dieselben wie beim Anfahr-Versuch, damit die Zahlen
# direkt vergleichbar sind.
ZIELE = [
    ("Pose A (Flansch ~455 mm vor der Kamera)", [-10.0, -20.0, 0.0, -70.0, 0.0, 0.0]),
    ("Pose B (Arm weiter ausgestreckt, mehr Moment auf Achse 2)",
     [-10.0, -34.0, 12.0, -62.0, 0.0, 0.0]),
]


def fehler(ist_grad, ziel_grad):
    return [ist_grad[k] - ziel_grad[k] for k in range(6)]


def zeile(nr, f):
    return "  Versuch %d:  max %5.2f deg   je Gelenk %s" % (
        nr, max(abs(v) for v in f), [round(v, 2) for v in f])


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--versuche", type=int, default=5,
                    help="wie oft dasselbe Ziel hoechstens erneut gesendet wird")
    ap.add_argument("--toleranz", type=float, default=0.5,
                    help="ab welchem Restfehler (Grad) abgebrochen wird")
    ap.add_argument("--korrektur", type=float, default=0.0,
                    help="Anteil des Fehlers, der auf das Kommando aufgerechnet wird "
                         "(0 = wortgleich dasselbe Ziel erneut senden)")
    ap.add_argument("--max-korrektur", type=float, default=15.0,
                    help="Sicherheitsdeckel: so weit darf das Kommando hoechstens "
                         "vom Ziel abweichen (Grad je Gelenk)")
    ap.add_argument("--vorhalt", type=float, default=8.0,
                    help="um wie viel Grad Achse 2 UNTER dem Ziel startet")
    ap.add_argument("--sekunden", type=float, default=7.0, help="Fahrzeit zur Anfahrt")
    ap.add_argument("--nachsekunden", type=float, default=3.0,
                    help="Fahrzeit fuer die Wiederholungen (der Weg ist winzig)")
    ap.add_argument("--ruhe", type=float, default=2.0)
    ap.add_argument("--dry-run", action="store_true")
    ap.add_argument("--json", default="")
    a = ap.parse_args()

    rclpy.init()
    n = Messung()
    if not n.warte(lambda: n.js is not None, 20, "keine /joint_states"):
        return 1
    if not n.valid.wait_for_service(timeout_sec=10.0):
        print("FEHLER: /check_state_validity nicht da")
        return 1
    if not n.traj.wait_for_server(timeout_sec=10.0):
        print("FEHLER: arm_controller action nicht da")
        return 1

    start = [math.degrees(v) for v in n.gelenke()]
    print("Betriebsart: %s" % ("wortgleiche Wiederholung" if a.korrektur == 0
                                else "korrigiertes Kommando, k = %.2f" % a.korrektur))
    print("Start (Grad): %s\n" % [round(v, 1) for v in start])

    # --- erst alle Wege pruefen, sonst wird gar nicht gefahren ----------------
    print("Wege pruefen ...")
    plan = []
    vorherige = start
    for name, ziel in ZIELE:
        anfahrt = list(ziel)
        anfahrt[1] = ziel[1] - a.vorhalt          # von UNTEN: Achse 2 kleiner
        ok = True
        for q_von, q_nach in ((vorherige, anfahrt), (anfahrt, ziel)):
            frei, wo = weg_zulaessig(n, q_von, q_nach)
            if not frei:
                print("  %s: unzulaessig bei %s %% des Weges — wird uebersprungen" % (name, wo))
                ok = False
                break
        if ok:
            print("  %s: frei" % name)
            plan.append((name, ziel, anfahrt))
            vorherige = ziel
    if not plan:
        print("Kein zulaessiger Weg — Abbruch.")
        return 1

    if a.dry_run:
        print("\n--dry-run: es wird nicht gefahren.")
        return 0

    ergebnis = []
    for name, ziel, anfahrt in plan:
        print("\n=== %s ===" % name)
        print("Ziel (Grad): %s" % [round(v, 1) for v in ziel])

        if not fahre(n, anfahrt, a.sekunden):
            print("  Anfahrt abgelehnt — weiter mit der naechsten Pose.")
            continue
        einschwingen(n, ruhe_s=a.ruhe)

        if not fahre(n, ziel, a.sekunden):
            print("  Fahrt zum Ziel abgelehnt.")
            continue

        verlauf = []
        kommando = list(ziel)
        for versuch in range(1, a.versuche + 1):
            ist, dauer, still = einschwingen(n, ruhe_s=a.ruhe)
            if not still:
                print("  Versuch %d: Gelenke kamen in %.0f s nicht zur Ruhe" % (versuch, dauer))
            f = fehler(ist, ziel)
            verlauf.append({"versuch": versuch, "ist": [round(v, 3) for v in ist],
                            "fehler": [round(v, 3) for v in f],
                            "max": round(max(abs(v) for v in f), 3),
                            "kommando": [round(v, 3) for v in kommando]})
            print(zeile(versuch, f))
            if max(abs(v) for v in f) <= a.toleranz:
                print("  -> unter Toleranz (%.2f deg), fertig nach %d Versuch(en)."
                      % (a.toleranz, versuch))
                break
            if versuch >= a.versuche:
                break

            if a.korrektur > 0.0:
                # Fehler auf das Kommando aufrechnen und am Deckel begrenzen.
                neu_kdo = []
                for k in range(6):
                    v = kommando[k] - a.korrektur * f[k]
                    ab = max(-a.max_korrektur, min(a.max_korrektur, v - ziel[k]))
                    neu_kdo.append(ziel[k] + ab)
                # Das korrigierte Kommando MUSS geprueft werden - es liegt
                # ausserhalb des Ziels und wurde vorher nie auf Kollision getestet.
                frei, wo = weg_zulaessig(n, ist, neu_kdo)
                if not frei:
                    print("  korrigiertes Kommando unzulaessig bei %s %% - Abbruch dieser Pose" % wo)
                    break
                kommando = neu_kdo
                print("      Kommando -> %s" % [round(v, 2) for v in kommando])

            fahre(n, kommando, a.nachsekunden)

        bester = min(v["max"] for v in verlauf)
        erst, letzt = verlauf[0]["max"], verlauf[-1]["max"]
        print("  bester Wert im Verlauf: %.2f deg" % bester)
        print("  Bilanz: %.2f -> %.2f deg  (%s)" % (
            erst, letzt,
            "verbessert um %.0f %%" % (100 * (erst - letzt) / erst) if erst > 0 else "-"))
        ergebnis.append({"pose": name, "ziel": ziel, "korrektur": a.korrektur,
                         "bester": bester, "verlauf": verlauf})

    if a.json:
        with open(a.json, "w") as fh:
            json.dump(ergebnis, fh, indent=2)
        print("\nJSON: %s" % a.json)

    n.destroy_node()
    rclpy.shutdown()
    return 0


if __name__ == "__main__":
    sys.exit(main())
