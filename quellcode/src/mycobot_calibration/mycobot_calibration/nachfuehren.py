#!/usr/bin/env python3
"""Fuehrt die aktuelle Sollstellung nach, bis der Encoder sie bestaetigt.

Wozu
----
Die Servos kommen unter Last nicht am kommandierten Winkel an (gemessen
2026-09-08: bis 15.2 Grad). MoveIt merkt davon nichts — `execute()` meldet
Erfolg, sobald der Controller seine Zeit abgelaufen hat. Dieses Werkzeug wird
NACH einer Bewegung aufgerufen und schiebt den Arm auf den Sollwert nach.

Woher das Ziel kommt
--------------------
Aus `/arm_controller/state` (`desired.positions`) — dem Sollwert des
Trajektorien-Controllers. Achtung: nach einem Lauf steht dort das KORRIGIERTE
Kommando, nicht mehr das urspruengliche Ziel. Direkt nach einem MoveIt-execute
ist beides dasselbe; ein zweiter Lauf ohne Bewegung dazwischen jagt dagegen dem
schon korrigierten Wert nach. Wer das nicht will, gibt --winkel vor. Der Aufrufer muss also nichts uebergeben und braucht
die Zielstellung nicht zu kennen; das macht das Werkzeug aus C++, Python und
Shell gleichermassen benutzbar. Mit --winkel laesst sich ein Ziel auch direkt
vorgeben (Grad, ARM_JOINTS-Reihenfolge); dann wird es ANGEFAHREN und danach
nachgefuehrt, denn ein ausdrueckliches Ziel heisst "fahre dorthin".

Wann es NICHT laufen darf
-------------------------
Waehrend move_group gerade ausfuehrt. Es sendet eigene Ziele an denselben
arm_controller; das gehoert in eine Ruhepause (z.B. waehrend pick_tilt im Hover
auf /pick/confirm wartet), nicht mitten in eine laufende Bahn.

Aufruf
------
    ros2 run mycobot_calibration nachfuehren
    ros2 run mycobot_calibration nachfuehren --ros-args -- --toleranz 0.3
    ros2 run mycobot_calibration nachfuehren -- --winkel -10 -20 0 -70 0 0

Rueckgabe: 0 = Ziel erreicht, 1 = nicht erreicht, 2 = gar nicht gelaufen.
"""
import argparse
import math
import sys

import rclpy
from control_msgs.msg import JointTrajectoryControllerState
from rclpy.node import Node

from mycobot_calibration.numerical_ik import ARM_JOINTS
from mycobot_calibration.positionieren import (K_VORGABE, MAX_VERSUCHE,
                                               TOLERANZ_GRAD, Positionierer)


class SollLeser(Node):
    def __init__(self):
        super().__init__("nachfuehren")
        self.soll = None
        self.js = None
        self.create_subscription(JointTrajectoryControllerState,
                                 "/arm_controller/state", self._on_state, 10)

    def _on_state(self, m):
        # joint_names des Controllers muessen nicht in ARM_JOINTS-Reihenfolge kommen.
        d = dict(zip(m.joint_names, m.desired.positions))
        try:
            self.soll = [d[j] for j in ARM_JOINTS]
        except KeyError:
            self.soll = None


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--winkel", type=float, nargs=6, default=None,
                    help="Ziel in GRAD statt aus /arm_controller/state")
    ap.add_argument("--k", type=float, default=K_VORGABE)
    ap.add_argument("--toleranz", type=float, default=TOLERANZ_GRAD)
    ap.add_argument("--versuche", type=int, default=MAX_VERSUCHE)
    ap.add_argument("--still", action="store_true", help="nur das Ergebnis ausgeben")
    a = ap.parse_args()

    rclpy.init()
    n = SollLeser()
    p = Positionierer(n)
    if not p.bereit(15.0):
        print("[nachfuehren] FEHLER: /joint_states, arm_controller oder "
              "/check_state_validity nicht erreichbar")
        return 2

    if a.winkel is not None:
        ziel = [math.radians(v) for v in a.winkel]
    else:
        if not n.js and not p.gelenke_rad():
            print("[nachfuehren] FEHLER: keine Gelenkdaten")
            return 2
        # auf den Sollwert des Controllers warten
        for _ in range(100):
            rclpy.spin_once(n, timeout_sec=0.1)
            if n.soll is not None:
                break
        if n.soll is None:
            print("[nachfuehren] FEHLER: /arm_controller/state liefert keinen Sollwert")
            return 2
        ziel = list(n.soll)

    vorher = p.gelenke_rad()
    vorher_f = max(abs(math.degrees(vorher[i] - ziel[i])) for i in range(6))
    if not a.still:
        print("[nachfuehren] Ziel (Grad): %s"
              % [round(math.degrees(v), 2) for v in ziel])
        print("[nachfuehren] Fehler vorher: %.2f Grad" % vorher_f)

    # Ein ausdruecklich uebergebenes Ziel heisst "fahre dorthin" - also anfahren()
    # (Fahrt samt Wegpruefung, danach nachfuehren). Nur nachzufuehren waere hier
    # falsch: die Schleife korrigiert mit k=0.5 und kriecht dem Ziel entgegen,
    # statt es anzufahren. Beim ersten Versuch am 2026-09-09 dauerte es so acht
    # Durchgaenge, um von 68.7 auf 4.2 Grad zu kommen - die Fahrt selbst haette
    # es in einem Zug erledigt.
    # Kommt das Ziel dagegen aus /arm_controller/state, steht der Arm bereits
    # dort; dann ist nachfuehren() genau richtig.
    if a.winkel is not None:
        erg = p.anfahren(ziel, k=a.k, toleranz_grad=a.toleranz,
                         max_versuche=a.versuche,
                         log=None if a.still else print)
    else:
        erg = p.nachfuehren(ziel, k=a.k, toleranz_grad=a.toleranz,
                            max_versuche=a.versuche,
                            log=None if a.still else print)
    print("[nachfuehren] %s  (vorher %.2f Grad)" % (erg.text(), vorher_f))

    n.destroy_node()
    rclpy.shutdown()
    return 0 if erg.erfolg else 1


if __name__ == "__main__":
    sys.exit(main())
