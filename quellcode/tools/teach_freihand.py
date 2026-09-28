#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""Anlernen von Stellungen VON HAND — Servos frei, Arm wird gefuehrt.

Warum nicht joggen: das Spiel in den Achsen (J1 lose Innenschrauben, J2
bauartbedingtes Servospiel) macht das schrittweise Anfahren muehsam und ungenau.
Von Hand fuehren umgeht das komplett: die Stellung, die der Encoder dabei liest,
IST die Stellung, die der Arm spaeter wieder anfaehrt. Kein Modellfehler, keine
IK, kein TCP-Versatz — alles faellt weg, weil in GELENKWINKELN gespeichert wird.

    python3 tools/teach_freihand.py --datei teach_punkte.json

GEFAHR — bitte lesen:
  release_all_servos() nimmt JEDES Drehmoment weg. Der Arm faellt dann unter
  seinem eigenen Gewicht, mit dem Greifer als Hebel. Aus der Nullstellung
  (Arm senkrecht nach oben) ist der Fall am weitesten und am haertesten.
  Deshalb: dieses Werkzeug gibt die Servos erst frei, NACHDEM du bestaetigt
  hast, dass du den Arm mit der Hand haeltst.

  Der Stack darf NICHT laufen: er haelt /dev/ttyTHS1 belegt. Sim-Stack
  (USE_FAKE_HARDWARE=true) stoert nicht, der fasst den Port nicht an.

Ablauf:
  1. Arm von Hand festhalten, ENTER  -> Servos werden frei
  2. Arm an die Zielstelle fuehren, dort festhalten
  3. Name eingeben + ENTER          -> Stellung wird gespeichert
  4. weitere Punkte wie 2./3.
  5. 'q' + ENTER                    -> power_on (Arm haelt wieder), Ende

Gespeichert wird je Punkt: Gelenkwinkel in Grad UND Radiant, Zeitstempel.
"""
import argparse
import json
import os
import sys
import time

try:
    from pymycobot import MyCobot280
except ImportError:
    sys.exit("pymycobot fehlt — 'pip3 install pymycobot' oder falsches Python")

ap = argparse.ArgumentParser()
ap.add_argument("--port", default=os.environ.get("MYCOBOT_PORT", "/dev/ttyTHS1"))
ap.add_argument("--baud", type=int, default=1000000)
ap.add_argument("--datei", default="teach_punkte.json")
ap.add_argument("--mittel", type=int, default=5,
                help="so viele Encoder-Lesungen mitteln (gegen Rauschen)")
a = ap.parse_args()

print("=" * 68)
print("  ANLERNEN VON HAND — die Servos werden gleich FREIGEGEBEN")
print("=" * 68)
print()
print("  Der Arm faellt, sobald das Drehmoment weg ist. Der Greifer wirkt")
print("  dabei als Hebel. Bevor es weitergeht:")
print()
print("    * Halte den Arm MIT DER HAND fest (am Unterarm, nicht am Greifer)")
print("    * Steht der Arm senkrecht nach oben? Dann bring ihn VORHER")
print("      tiefer — von dort ist der Fall am weitesten.")
print("    * Steht etwas Zerbrechliches unter dem Arm? Trichter, Platte,")
print("      Schale wegnehmen oder Arm daneben halten.")
print()
antwort = input("  Haelst du den Arm fest? Dann ENTER. Abbruch mit 'n': ").strip().lower()
if antwort == "n":
    sys.exit("abgebrochen — nichts geaendert")

print(f"\n  Verbinde mit {a.port} @ {a.baud} ...")
mc = MyCobot280(a.port, a.baud)
time.sleep(0.5)

vorher = mc.get_angles()
if not isinstance(vorher, list) or len(vorher) != 6:
    sys.exit(f"Encoder antwortet nicht plausibel: {vorher!r} — Port belegt (Stack laeuft)?")
print("  Stellung jetzt:  " + ", ".join(f"{v:+7.2f}" for v in vorher))

mc.release_all_servos()
time.sleep(0.3)
print("\n  >>> SERVOS SIND FREI — der Arm haelt sich NICHT mehr selbst <<<\n")

punkte = {}
if os.path.exists(a.datei):
    try:
        punkte = json.load(open(a.datei))
        print(f"  {len(punkte)} vorhandene Punkte aus {a.datei} geladen: "
              + ", ".join(sorted(punkte)))
    except Exception as e:
        print(f"  {a.datei} nicht lesbar ({e}) — wird neu angelegt")


def lies_gemittelt(n):
    """n Lesungen mitteln; gibt (grad, radiant, streuung_grad) zurueck."""
    proben = []
    for _ in range(n):
        w = mc.get_angles()
        if isinstance(w, list) and len(w) == 6:
            proben.append(w)
        time.sleep(0.08)
    if not proben:
        return None, None, None
    grad = [sum(p[i] for p in proben) / len(proben) for i in range(6)]
    streu = [max(p[i] for p in proben) - min(p[i] for p in proben) for i in range(6)]
    rad = mc.get_radians()
    if not (isinstance(rad, list) and len(rad) == 6):
        rad = [g * 3.141592653589793 / 180.0 for g in grad]
    return grad, rad, max(streu)


def sichern():
    with open(a.datei, "w") as f:
        json.dump(punkte, f, indent=2, sort_keys=True)


try:
    while True:
        print("  Arm an die Stelle fuehren und dort FESTHALTEN.")
        name = input("  Name des Punktes (oder 'q' zum Beenden): ").strip()
        if name.lower() == "q":
            break
        if not name:
            print("  -- leerer Name, nochmal --\n")
            continue
        if name in punkte:
            if input(f"  '{name}' gibt es schon. Ueberschreiben? [j/N]: ").strip().lower() != "j":
                print()
                continue

        grad, rad, streu = lies_gemittelt(a.mittel)
        if grad is None:
            print("  !! Encoder antwortet nicht — nichts gespeichert\n")
            continue

        punkte[name] = {
            "grad": [round(v, 3) for v in grad],
            "rad": [round(v, 5) for v in rad],
            "streuung_grad": round(streu, 3),
            "zeit": time.strftime("%Y-%m-%d %H:%M:%S"),
        }
        sichern()
        print("  gespeichert: " + ", ".join(f"{v:+7.2f}" for v in grad))
        if streu > 0.5:
            print(f"  ACHTUNG: Streuung {streu:.2f} Grad — der Arm hat sich beim")
            print("           Lesen bewegt. Ruhiger halten und nochmal aufnehmen.")
        print(f"  -> {a.datei} ({len(punkte)} Punkte)\n")

finally:
    print("\n  Arm weiter FESTHALTEN — die Servos werden jetzt wieder eingeschaltet.")
    input("  ENTER wenn du ihn haelst: ")
    try:
        mc.power_on()
        time.sleep(0.5)
        nachher = mc.get_angles()
        print("  Servos an. Stellung: "
              + (", ".join(f"{v:+7.2f}" for v in nachher)
                 if isinstance(nachher, list) and len(nachher) == 6 else str(nachher)))
        print("  Vorsichtig loslassen und pruefen, ob der Arm haelt.")
    except Exception as e:
        print(f"  !! power_on fehlgeschlagen: {e}")
        print("  !! ARM WEITER FESTHALTEN und den Roboter stromlos machen.")
    print(f"\n  {len(punkte)} Punkte in {a.datei}")
