#!/usr/bin/env python3
"""Misst das Spiel/Nachgeben je Gelenk, waehrend der Arm von Hand ausgelenkt wird.

Warum nicht per Screenshot
--------------------------
Der Roboter in RViz wird AUS den Encoderwerten (/joint_states) gezeichnet. Ein
Bildvergleich misst also genau das, was hier direkt in Grad ausgelesen wird — nur
ungenauer. Deshalb wird hier gemessen statt geschaut.

Ablauf (drei Aufnahmen, jeweils mit ENTER ausgeloest)
----------------------------------------------------
    1. RUHE      — Arm unberuehrt. Nullbezug.
    2. AUSGELENKT— Arm von Hand SANFT in eine Richtung gezogen und GEHALTEN.
    3. LOSGELASSEN — Arm wieder freigegeben, kurz warten.

Auswertung
----------
    * Auslenkung je Gelenk = Aufnahme 2 - Aufnahme 1  (Vorzeichen = Richtung)
    * Bleibende Abweichung = Aufnahme 3 - Aufnahme 1
      -> geht sie zurueck, war es elastisches Nachgeben (Servo-Regelkreis)
      -> bleibt sie stehen, ist es echtes mechanisches Spiel
    * dazu die Verschiebung des TCP in mm (ueber TF robot_base -> tcp), damit die
      Gradzahlen greifbar werden

WICHTIGE GRENZE: gemessen wird, was der ENCODER sieht. Der Encoder sitzt am
Servo-EINGANG. Spiel HINTER dem Getriebe (Abtrieb, Gelenkschale) sieht er
prinzipiell nicht — dann bleiben die Gradzahlen klein, obwohl der TCP real
wandert. Ein vollstaendiges Bild gibt erst eine Messung von aussen (Marker am
Flansch, mit der Kamera vermessen).

Bedienung: Stack muss laufen (Servos unter Drehmoment).
    python3 tools/measure_joint_play.py
    python3 tools/measure_joint_play.py --json /tmp/spiel.json
"""
import argparse
import json
import math
import sys
import time

import cv2
import numpy as np
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import JointState
import tf2_ros

ARM = ["joint2_to_joint1", "joint3_to_joint2", "joint4_to_joint3",
       "joint5_to_joint4", "joint6_to_joint5", "joint6output_to_joint6"]
KURZ = {"joint2_to_joint1": "Achse 1", "joint3_to_joint2": "Achse 2",
        "joint4_to_joint3": "Achse 3", "joint5_to_joint4": "Achse 4",
        "joint6_to_joint5": "Achse 5", "joint6output_to_joint6": "Achse 6"}


class Leser(Node):
    def __init__(self):
        super().__init__("spiel_messung")
        self.letzte = None
        self.create_subscription(JointState, "/joint_states", self._cb, 50)
        self.buf = tf2_ros.Buffer()
        self.lis = tf2_ros.TransformListener(self.buf, self)

    def _cb(self, m):
        self.letzte = m

    def mittel(self, sekunden=1.5):
        """Ueber mehrere Nachrichten mitteln — daempft das Encoderrauschen."""
        proben = {j: [] for j in ARM}
        t0 = time.time()
        while rclpy.ok() and time.time() - t0 < sekunden:
            rclpy.spin_once(self, timeout_sec=0.05)
            m = self.letzte
            if m is None:
                continue
            d = dict(zip(m.name, m.position))
            for j in ARM:
                if j in d:
                    proben[j].append(d[j])
        if not all(proben[j] for j in ARM):
            sys.exit("FEHLER: /joint_states unvollstaendig — laeuft der Stack?")
        return ({j: float(np.mean(proben[j])) for j in ARM},
                {j: float(np.std(proben[j])) for j in ARM},
                len(proben[ARM[0]]))

    def tcp(self):
        t0 = time.time()
        while time.time() - t0 < 3.0:
            try:
                t = self.buf.lookup_transform("robot_base", "tcp", rclpy.time.Time())
                return np.array([t.transform.translation.x,
                                 t.transform.translation.y,
                                 t.transform.translation.z])
            except Exception:
                rclpy.spin_once(self, timeout_sec=0.1)
        return None


def aufnehmen(n, titel, hinweis):
    print("\n" + "=" * 68)
    print(titel)
    print(hinweis)
    input(">>> ENTER druecken, wenn es soweit ist ... ")
    print("    messe (1.5 s) ...", flush=True)
    w, s, k = n.mittel()
    p = n.tcp()
    print("    fertig (%d Messwerte, Rauschen max %.3f Grad)"
          % (k, max(math.degrees(v) for v in s.values())))
    return {"winkel": w, "rausch": s, "tcp": p}


PHASEN = [
    # (Sekunden, Titel, Unterzeile, Farbe BGR, ab welcher Sekunde aufgezeichnet wird)
    (10, "BEREIT MACHEN", "Hand an den Arm - noch NICHT ziehen", (120, 120, 120), None),
    (6,  "RUHE", "Arm NICHT beruehren - Nullbezug wird gemessen", (60, 160, 60), 2.0),
    (10, "JETZT ZIEHEN UND HALTEN", "SANFT in EINE Richtung - halten bis Umschaltung",
     (0, 140, 230), 4.0),
    (9,  "LOSLASSEN", "Arm freigeben und nicht mehr beruehren", (160, 100, 60), 4.0),
]


def _tafel(titel, unter, farbe, rest, phase_i):
    """Grosse, gut sichtbare Anzeige — Text deutsch und ASCII-sicher."""
    img = np.zeros((420, 900, 3), np.uint8)
    img[:] = (28, 28, 30)
    cv2.rectangle(img, (0, 0), (900, 90), farbe, -1)
    cv2.putText(img, "GELENKSPIEL MESSEN  (%d/4)" % phase_i, (24, 58),
                cv2.FONT_HERSHEY_SIMPLEX, 1.1, (255, 255, 255), 2, cv2.LINE_AA)
    cv2.putText(img, titel, (24, 190), cv2.FONT_HERSHEY_SIMPLEX, 1.7, farbe, 4, cv2.LINE_AA)
    cv2.putText(img, unter, (24, 245), cv2.FONT_HERSHEY_SIMPLEX, 0.8,
                (220, 220, 220), 1, cv2.LINE_AA)
    cv2.putText(img, "%d" % int(np.ceil(rest)), (700, 340),
                cv2.FONT_HERSHEY_SIMPLEX, 4.5, (255, 255, 255), 8, cv2.LINE_AA)
    cv2.putText(img, "Sekunden", (690, 385), cv2.FONT_HERSHEY_SIMPLEX, 0.7,
                (180, 180, 180), 1, cv2.LINE_AA)
    return img


def getimt(n, a):
    """Feste Zeitfenster statt ENTER — der Benutzer folgt der Anzeige."""
    cv2.namedWindow("Gelenkspiel", cv2.WINDOW_NORMAL)
    cv2.resizeWindow("Gelenkspiel", 900, 420)
    cv2.moveWindow("Gelenkspiel", 40, 40)

    aufnahmen = []
    for i, (dauer, titel, unter, farbe, ab) in enumerate(PHASEN):
        proben = {j: [] for j in ARM}
        t0 = time.time()
        while True:
            verstrichen = time.time() - t0
            if verstrichen >= dauer:
                break
            rclpy.spin_once(n, timeout_sec=0.02)
            m = n.letzte
            if m is not None and ab is not None and verstrichen >= ab:
                d = dict(zip(m.name, m.position))
                for j in ARM:
                    if j in d:
                        proben[j].append(d[j])
            cv2.imshow("Gelenkspiel", _tafel(titel, unter, farbe, dauer - verstrichen, i + 1))
            if cv2.waitKey(30) == 27:      # ESC bricht ab
                cv2.destroyAllWindows()
                sys.exit("Abgebrochen.")
        if ab is not None:
            if not all(proben[j] for j in ARM):
                cv2.destroyAllWindows()
                sys.exit("FEHLER: /joint_states unvollstaendig — laeuft der Stack?")
            aufnahmen.append({
                "winkel": {j: float(np.mean(proben[j])) for j in ARM},
                "rausch": {j: float(np.std(proben[j])) for j in ARM},
                "tcp": n.tcp(),
                "n": len(proben[ARM[0]]),
                "titel": titel})
            print("  [%s] %d Messwerte, Rauschen max %.3f Grad"
                  % (titel, aufnahmen[-1]["n"],
                     max(math.degrees(v) for v in aufnahmen[-1]["rausch"].values())))

    cv2.imshow("Gelenkspiel", _tafel("FERTIG", "Auswertung im Terminal", (60, 160, 60), 0, 4))
    cv2.waitKey(1200)
    cv2.destroyAllWindows()
    return aufnahmen


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--json", default="")
    ap.add_argument("--timed", action="store_true",
                    help="feste Zeitfenster mit Anzeige statt ENTER-Aufforderungen")
    a = ap.parse_args()

    rclpy.init()
    n = Leser()
    print("=" * 68)
    print(" GELENKSPIEL MESSEN — Encoder lesen, waehrend von Hand ausgelenkt wird")
    print("=" * 68)
    print(" Sicherheit: SANFT ziehen. Nicht gegen die Servos reissen, keine")
    print(" Endanschlaege anfahren. Es geht um das Spiel, nicht um Kraft.")

    if a.timed:
        print(" Ablauf ueber ein Fenster auf dem Bildschirm — der Anzeige folgen.")
        print(" Vorbereiten 10 s, Ruhe 6 s, ZIEHEN UND HALTEN 10 s, Loslassen 9 s.")
        print(" ESC im Fenster bricht ab.\n")
        r, a2, a3 = getimt(n, a)
    else:
        r = aufnehmen(n, " AUFNAHME 1/3 — RUHE",
                      " Arm NICHT beruehren. Diese Stellung ist der Nullbezug.")
        a2 = aufnehmen(n, " AUFNAHME 2/3 — AUSGELENKT",
                       " Arm jetzt SANFT in EINE Richtung ziehen und dort HALTEN.\n"
                       " Waehrend der Messung weiter halten (1.5 s).")
        a3 = aufnehmen(n, " AUFNAHME 3/3 — LOSGELASSEN",
                       " Arm loslassen, 2-3 s warten, nicht mehr beruehren.")

    print("\n" + "=" * 68)
    print(" ERGEBNIS  (+ = positive Gelenkrichtung)")
    print("=" * 68)
    print(" %-9s %14s %14s %10s" % ("", "ausgelenkt", "bleibend", "Rueckkehr"))
    zeilen = {}
    for j in ARM:
        d2 = math.degrees(a2["winkel"][j] - r["winkel"][j])
        d3 = math.degrees(a3["winkel"][j] - r["winkel"][j])
        rueck = "-" if abs(d2) < 0.05 else ("%.0f%%" % (100 * (1 - abs(d3) / abs(d2))))
        print(" %-9s %+11.2f Grad %+11.2f Grad %10s" % (KURZ[j], d2, d3, rueck))
        zeilen[j] = {"ausgelenkt_grad": d2, "bleibend_grad": d3}

    if r["tcp"] is not None and a2["tcp"] is not None:
        d = 1000 * (a2["tcp"] - r["tcp"])
        print("\n TCP-Verschiebung beim Auslenken: dx %+.1f  dy %+.1f  dz %+.1f mm "
              "(Betrag %.1f mm)" % (d[0], d[1], d[2], np.linalg.norm(d)))
        if a3["tcp"] is not None:
            d3 = 1000 * (a3["tcp"] - r["tcp"])
            print(" TCP-Verschiebung bleibend      : dx %+.1f  dy %+.1f  dz %+.1f mm "
                  "(Betrag %.1f mm)" % (d3[0], d3[1], d3[2], np.linalg.norm(d3)))

    groesste = max(ARM, key=lambda j: abs(zeilen[j]["ausgelenkt_grad"]))
    print("\n Groesste Auslenkung: %s mit %+.2f Grad"
          % (KURZ[groesste], zeilen[groesste]["ausgelenkt_grad"]))
    bleib = max(ARM, key=lambda j: abs(zeilen[j]["bleibend_grad"]))
    print(" Groesste BLEIBENDE Abweichung: %s mit %+.2f Grad"
          % (KURZ[bleib], zeilen[bleib]["bleibend_grad"]))
    print("\n Deutung: was zurueckgeht = elastisches Nachgeben des Servo-Regelkreises.")
    print("          was stehen bleibt = echtes mechanisches Spiel.")
    print(" Der Encoder sieht nur Spiel VOR dem Getriebe. Fuer den Rest ist eine")
    print(" Messung von aussen noetig (Marker am Flansch + Kamera).")

    if a.json:
        with open(a.json, "w") as f:
            json.dump({"ruhe": {k: v for k, v in r["winkel"].items()},
                       "ausgelenkt": {k: v for k, v in a2["winkel"].items()},
                       "losgelassen": {k: v for k, v in a3["winkel"].items()},
                       "auswertung": zeilen}, f, indent=2)
        print("\n JSON: %s" % a.json)

    n.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
