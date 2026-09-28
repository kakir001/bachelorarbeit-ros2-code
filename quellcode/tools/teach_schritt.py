#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""Anlernen in EINZELSCHRITTEN — jeder Aufruf macht genau eine Sache.

Gedacht fuer den Fall, dass jemand anders (Claude) die Befehle absetzt, waehrend
der Benutzer den Arm mit der Hand fuehrt. Der interaktive Bruder dieses
Werkzeugs ist tools/teach_freihand.py — der fragt selbst nach und ist richtig,
wenn man allein am Terminal sitzt.

    python3 tools/teach_schritt.py frei              # Servos loesen
    python3 tools/teach_schritt.py lies trichter_greif
    python3 tools/teach_schritt.py halt              # Servos wieder einschalten
    python3 tools/teach_schritt.py zeig              # nur lesen, nichts aendern

GEFAHR bei 'frei': jedes Drehmoment faellt weg, der Arm sackt unter seinem
eigenen Gewicht ab. Vorher festhalten.

Der Stack darf NICHT laufen — er belegt /dev/ttyTHS1.
"""
import argparse
import json
import math
import os
import sys
import time

try:
    from pymycobot import MyCobot280
except ImportError:
    sys.exit("pymycobot fehlt")

ap = argparse.ArgumentParser()
ap.add_argument("befehl", choices=["frei", "lies", "halt", "zeig"])
ap.add_argument("name", nargs="?", help="Punktname (nur bei 'lies')")
ap.add_argument("--port", default=os.environ.get("MYCOBOT_PORT", "/dev/ttyTHS1"))
ap.add_argument("--baud", type=int, default=1000000)
ap.add_argument("--datei", default=os.path.expanduser("~/ros2_ws/teach_punkte.json"))
ap.add_argument("--mittel", type=int, default=7)
a = ap.parse_args()

if a.befehl == "lies" and not a.name:
    sys.exit("'lies' braucht einen Namen")

mc = MyCobot280(a.port, a.baud)
time.sleep(0.5)

winkel = mc.get_angles()
if not isinstance(winkel, list) or len(winkel) != 6:
    sys.exit(f"Encoder antwortet nicht plausibel: {winkel!r}\n"
             f"-> laeuft der Stack noch? Er belegt {a.port}.")

print("  Stellung jetzt: " + ", ".join(f"{v:+7.2f}" for v in winkel))

if a.befehl == "zeig":
    sys.exit(0)

if a.befehl == "frei":
    mc.release_all_servos()
    time.sleep(0.3)
    print("\n  >>> SERVOS SIND FREI — der Arm haelt sich NICHT mehr selbst <<<")
    sys.exit(0)

if a.befehl == "halt":
    mc.power_on()
    time.sleep(0.6)
    nach = mc.get_angles()
    print("  Servos EIN. Stellung: "
          + (", ".join(f"{v:+7.2f}" for v in nach)
             if isinstance(nach, list) and len(nach) == 6 else str(nach)))
    print("  Vorsichtig loslassen und pruefen, ob der Arm haelt.")
    sys.exit(0)

# ---- lies ----
proben = []
for _ in range(a.mittel):
    w = mc.get_angles()
    if isinstance(w, list) and len(w) == 6:
        proben.append(w)
    time.sleep(0.08)
if not proben:
    sys.exit("!! Encoder antwortet nicht — nichts gespeichert")

grad = [sum(p[i] for p in proben) / len(proben) for i in range(6)]
streu = max(max(p[i] for p in proben) - min(p[i] for p in proben) for i in range(6))
rad = mc.get_radians()
if not (isinstance(rad, list) and len(rad) == 6):
    rad = [math.radians(g) for g in grad]

punkte = {}
if os.path.exists(a.datei):
    try:
        punkte = json.load(open(a.datei))
    except Exception:
        pass

punkte[a.name] = {
    "grad": [round(v, 3) for v in grad],
    "rad": [round(v, 5) for v in rad],
    "streuung_grad": round(streu, 3),
    "proben": len(proben),
    "zeit": time.strftime("%Y-%m-%d %H:%M:%S"),
}
with open(a.datei, "w") as f:
    json.dump(punkte, f, indent=2, sort_keys=True)

print(f"\n  '{a.name}' gespeichert:")
print("    " + ", ".join(f"{v:+7.2f}" for v in grad))
print(f"    Streuung ueber {len(proben)} Lesungen: {streu:.3f} Grad")
if streu > 0.5:
    print("    ACHTUNG: der Arm hat sich beim Lesen bewegt — ruhiger halten,")
    print("             Punkt noch einmal aufnehmen.")
print(f"  -> {a.datei}  ({len(punkte)} Punkte: {', '.join(sorted(punkte))})")
