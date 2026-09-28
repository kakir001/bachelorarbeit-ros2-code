#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""Gemeinsame Regeln fuer das Gelenkspiel (Spiel hinter dem Getriebe), gemessen am 2026-09-19.

Befund (Handy-Wasserwaage am Greifergehaeuse, nest_11):
  * Dasselbe Encoder-Soll ergibt real bis zu 7 Grad verschiedene Greiferneigung, je nachdem, aus welcher
    Richtung Achse 4 zuletzt gefahren ist (J4 +2 kommandiert -> real -7 Grad). Der Abtrieb von Achse 4
    legt sich auf die Seite der letzten Bewegungsrichtung; der Encoder (am Servo-Eingang) sieht das nicht.
  * Steht der Greifer real senkrecht, zeigt das Modell ~10 Grad radial / ~5 Grad tangential Neigung
    (teach_punkte.json: nest_11_ablage, modell_neigung_grad). Angelernte Punkte tragen diesen Versatz
    deshalb im Modell mit sich - das ist gewollt, sie sind am realen Roboter richtig.
  * 20.9. (Benutzer): in der Nullstellung laesst sich der Arm von Hand deutlich um Achse 2 bewegen - das
    Spiel von Achse 2 ist ohne Schwerkraftmoment frei und legt sich sonst auf die Seite der letzten Bewegung.
Regel fuer alle Fahrwerkzeuge: nach dem Absenken auf einen Teach-Punkt Achse 2 UND Achse 4 immer VON OBEN auf
ihr Soll bringen (erst +hub, dann herunter, gleiche Richtung wie die Abfahrt), dann nachstellen. So sitzt das
Spiel bei jeder Ablage auf derselben Seite wie beim Anlernen (tools/j4_von_oben.py macht dasselbe von Hand).

    from gelenkspiel import j4_von_oben
    j4_von_oben(fahre_gelenke, soll)          # fahre_gelenke(rad, dauer, was) wie in lege_welle/nest_anfahren
"""
import math

J2 = 1                  # Index von joint3_to_joint2 in der ARM-Liste
J4 = 3                  # Index von joint5_to_joint4
HUB_GRAD = 8.0          # so weit ueber das Soll, dann herunter (19.9.: 8 Grad reichten, Spiel J4 ~6 Grad)
GELENKE_VON_OBEN = (J2, J4)   # 20.9.: auch Achse 2 - in der Nullstellung (kein Schwerkraftmoment) laesst sich der Arm
                              # von Hand um das Spiel von Achse 2 bewegen; in Arbeitsstellungen legt es sich auf die
                              # Seite der letzten Bewegung. Beim Absenken im Zyklus wird J2 KLEINER -> "von oben" =
                              # Soll+hub, dann Soll, gleiche Richtung wie die Abfahrt.


def von_oben(fahre_gelenke, soll, gelenke=GELENKE_VON_OBEN, hub_grad=HUB_GRAD, dauer=3.0, was="von oben"):
    """Die genannten Gelenke (Vorgabe J2 + J4) erst um hub_grad UEBER das Soll, dann direkt auf das Soll;
    uebrige Gelenke bleiben auf Soll. fahre_gelenke(rad, dauer, was) -> faehrt und wartet (Setzzeit inklusive).
    Danach im Aufrufer nachstellen (Totband)."""
    hoch = list(soll)
    for g in gelenke: hoch[g] += math.radians(hub_grad)
    namen = "/".join(f"J{g+1}" for g in gelenke)
    fahre_gelenke(hoch, dauer, f"{was} ({namen}): +{hub_grad:.0f} Grad")
    fahre_gelenke(list(soll), dauer, f"{was} ({namen}): herunter auf Soll")


def j4_von_oben(fahre_gelenke, soll, hub_grad=HUB_GRAD, dauer=3.0, was="J4 von oben"):
    """Rueckwaertskompatibel: seit 20.9. J2 UND J4 (GELENKE_VON_OBEN)."""
    von_oben(fahre_gelenke, soll, GELENKE_VON_OBEN, hub_grad, dauer, was)


def nullstellung_rad(datei=None):
    """Die REALE Nullstellung (Arm steht senkrecht) aus teach_punkte.json["nullstellung"], sonst [0]*6.
    20.9.: Encoder-Nullpunkte J2 +2.3 / J3 +0.3 Grad verschoben (gelenk_nullpunkte.json) - die Encoder-Null
    [0]*6 laesst den Arm sichtbar nach vorn kippen. Alle Park-/Nullfahrten nehmen deshalb diesen Punkt."""
    import json, os
    datei = datei or os.path.expanduser("~/ros2_ws/teach_punkte.json")
    try: return [float(v) for v in json.load(open(datei))["nullstellung"]["rad"]]
    except Exception: return [0.0] * 6
