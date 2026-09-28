#!/usr/bin/env python3
# =====================================================================
#  confirm_key.py — 'g'-Bestätigung in der RViz-Vorschau-Phase (da cam_viewer geschlossen ist).
#
#  Jedes ENTER im Terminal (oder 'g'+ENTER) → veröffentlicht /pick/confirm (std_msgs/Empty).
#  pick_tilt wartet zweimal auf Bestätigung: (1) im HOVER die Trajektorie in RViz prüfen→bestätigen,
#  (2) nach dem Greifen "gehalten?"→bestätigen. Mit Ctrl+D / Ctrl+C beenden.
#
#  NACHFUEHRUNG vor dem Abstieg (NACHFUEHREN=1, Vorgabe an):
#  Beim ERSTEN ENTER wird vor der Bestätigung nachgefuehrt. Grund: die Servos
#  kommen unter Last nicht am kommandierten Winkel an — gemessen am 2026-09-08
#  bis zu 15.2 Grad, ohne dass MoveIt etwas davon meldet. Der Arm steht dann
#  neben dem Hover, und weil der Abstieg von der TATSAECHLICHEN Lage aus
#  kartesisch weitergeht, wandert der Fehler direkt in den Greifpunkt.
#  Genau hier ist der richtige Moment dafuer: der Hover ist gefahren, der
#  Controller ist in Ruhe, und pick_tilt wartet ohnehin.
#  Der zweite ENTER fuehrt NICHT nach — da haelt der Greifer bereits die Welle,
#  und zusaetzliche Bewegung wuerde sie nur stoeren.
#  Abschalten: NACHFUEHREN=0 ./run_pick_preview.sh
# =====================================================================
import os
import subprocess
import sys

import rclpy
from rclpy.node import Node
from std_msgs.msg import Empty

WS = os.path.expanduser("~/ros2_ws")
NACHFUEHREN = os.environ.get("NACHFUEHREN", "1") == "1"


def nachfuehren():
    """Sollstellung nachfahren, bis der Encoder sie bestaetigt.

    Als eigener Prozess, nicht im selben Knoten: die Schleife spinnt ihren
    Knoten selbst, und auf Galactic sind Nebenlaeufigkeiten in rclpy-Knoten
    besser zu meiden. Ein Fehlschlag ist NICHT toedlich — dann wird eben
    ohne Nachfuehrung bestaetigt, wie vorher auch.
    """
    print("[confirm_key] Nachfuehrung laeuft (kann ~30-50 s dauern)...", flush=True)
    try:
        r = subprocess.run(["ros2", "run", "mycobot_calibration", "nachfuehren"],
                           timeout=180)
        if r.returncode == 0:
            print("[confirm_key] ✓ Nachfuehrung fertig — Ziel erreicht.")
        elif r.returncode == 1:
            print("[confirm_key] ! Nachfuehrung hat die Toleranz nicht erreicht "
                  "— es wird trotzdem bestaetigt.")
        else:
            print("[confirm_key] ! Nachfuehrung konnte nicht laufen "
                  "— es wird ohne sie bestaetigt.")
    except subprocess.TimeoutExpired:
        print("[confirm_key] ! Nachfuehrung ueberschritt 180 s — abgebrochen, "
              "es wird ohne sie bestaetigt.")
    except Exception as e:                                   # noqa: BLE001
        print("[confirm_key] ! Nachfuehrung fehlgeschlagen (%s) "
              "— es wird ohne sie bestaetigt." % e)


def main():
    rclpy.init()
    n = Node("confirm_key")
    pub = n.create_publisher(Empty, "/pick/confirm", 1)
    print("[confirm_key] BEREIT. Pruefe die Trajektorie in RViz; zum BESTAETIGEN ENTER in diesem Terminal druecken.")
    print("[confirm_key] (pick_tilt wartet auf 2 Bestaetigungen: nach HOVER + nach dem Greifen.) Ctrl+C=beenden")
    if NACHFUEHREN:
        print("[confirm_key] Nachfuehrung vor dem Abstieg: AN (NACHFUEHREN=0 schaltet sie ab)")
    try:
        anzahl = 0
        for _ in sys.stdin:
            anzahl += 1
            if anzahl == 1 and NACHFUEHREN:
                nachfuehren()
            pub.publish(Empty())
            print("[confirm_key] ✓ /pick/confirm gesendet.")
    except KeyboardInterrupt:
        pass
    n.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
