#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""Kollisionsmodell der ABLAGEPLATTE (5 x 4 Nester) aus den angelernten Nestern erzeugen.

Schreibt mycobot_world/urdf/ablageplatte_pose.xacro: Lage der Platte in robot_base UND die
20 Nestbosse als Versatz zur Plattenmitte. Die MASSE stehen in ablageplatte.xacro (am Bauteil
gemessen 2026-09-10, DEVLOG): Platte 94 x 78 x 1 mm, Nestbosse Ø 12 aussen / Ø 8 innen,
6 mm hoch, Raster 17 mm, Rand 7 / 7.5 mm.

WARUM AUS DEN ANGELERNTEN NESTERN und nicht aus der Kamera: die Nester (teach_punkte.json,
nest_JI) sind die Punkte, die der Arm WIRKLICH anfaehrt - im "Modellraum" des Roboters.
Der weicht von der Kamerawelt hier um ~10 % ab (Ecknester im Modell 56x60/73x74 mm statt
68x51). Ein Kollisionskoerper muss dort stehen, wo der Arm die Platte "sieht", sonst schuetzt
er die falsche Stelle. Deshalb: Bosse genau an den angelernten xy, Plattenkasten als
umschliessendes Rechteck der Bosse + Rand (+3 mm Sicherheit). Hoehe REAL (Platte 0..1 mm,
Bosse bis 7 mm ueber der Grundplatte): die Finger stehen beim Ablegen >= 30 mm darueber.

    python3 tools/ablageplatte_modell.py             # schreiben + Bericht
    python3 tools/ablageplatte_modell.py --trocken   # nur rechnen

Einschalten: ABLAGEPLATTE_MONTIERT=1 (Standard in start_mycobot.sh: 1, die Platte ist
festgeklebt). Nach dem Neu-Anlernen von Nestern neu erzeugen.
"""
import argparse, json, math, os, sys
from datetime import date
import numpy as np

WS = os.path.expanduser("~/ros2_ws")
ap = argparse.ArgumentParser()
ap.add_argument("--datei", default=os.path.join(WS, "teach_punkte.json"))
ap.add_argument("--ziel", default=os.path.join(WS, "src/mycobot_world/urdf/ablageplatte_pose.xacro"))
ap.add_argument("--rand", type=float, default=7.0, help="[mm] Plattenrand um die Bossmitten (7 / 7.5 gemessen)")
ap.add_argument("--sicherheit", type=float, default=3.0, help="[mm] Zuschlag am Plattenrand")
ap.add_argument("--trocken", action="store_true")
a = ap.parse_args()

d = json.load(open(a.datei))
def xy(v):
    return (v.get("tcp_mm") or v.get("tcp_modell_mm"))[:2]   # Ecknester gemessen, Zwischennester gerechnet
nester = {k: v for k, v in d.items() if k.startswith("nest_") and len(k) == 7 and (v.get("tcp_mm") or v.get("tcp_modell_mm"))}
if len(nester) < 4:
    sys.exit("!! zu wenige Nester mit tcp_mm in %s (%d)" % (a.datei, len(nester)))
namen = sorted(nester)
P = np.array([xy(nester[n]) for n in namen]) / 1e3
mitte = P.mean(0)

# Ausrichtung: Spaltenrichtung (I 1..4) = +x-artig, Zeilenrichtung (J 1..5) = -y-artig.
# Yaw aus allen Spalten-/Zeilennachbarn mitteln.
def richtung(paare):
    v = []
    for a_, b_ in paare:
        if a_ in nester and b_ in nester:
            v.append(np.array(xy(nester[b_])) - np.array(xy(nester[a_])))
    return np.mean(v, 0) if v else None
sp = richtung([("nest_%d%d" % (j, i), "nest_%d%d" % (j, i + 1)) for j in range(1, 6) for i in range(1, 4)])
ze = richtung([("nest_%d%d" % (j, i), "nest_%d%d" % (j + 1, i)) for j in range(1, 5) for i in range(1, 5)])
yaw_sp = math.atan2(sp[1], sp[0])
yaw_ze = math.atan2(ze[1], ze[0]) + math.pi / 2          # Zeilen laufen nach -y -> +90 Grad = x-Richtung
yaw = math.atan2(math.sin(yaw_sp) + math.sin(yaw_ze), math.cos(yaw_sp) + math.cos(yaw_ze))
R = np.array([[math.cos(yaw), math.sin(yaw)], [-math.sin(yaw), math.cos(yaw)]])   # welt -> platte
lokal = (P - mitte) @ R.T
# Plattenkasten: umschliessendes Rechteck der Bossmitten + Rand + Sicherheit, Mitte nachziehen
lo, hi = lokal.min(0), lokal.max(0)
laenge = (hi - lo) + 2 * (a.rand + a.sicherheit) / 1e3
versatz = (hi + lo) / 2.0
mitte_platte = mitte + R.T @ versatz
lokal = lokal - versatz

print("Nester: %d, Mitte (%.1f, %.1f) mm, Yaw %.2f Grad (Spalten %.2f / Zeilen %.2f)"
      % (len(namen), mitte_platte[0] * 1e3, mitte_platte[1] * 1e3, math.degrees(yaw), math.degrees(yaw_sp), math.degrees(yaw_ze)))
print("Bossfeld im Modell %.1f x %.1f mm (real 51 x 68), Plattenkasten %.1f x %.1f mm (real 78 x 94)"
      % ((hi - lo)[0] * 1e3, (hi - lo)[1] * 1e3, laenge[0] * 1e3, laenge[1] * 1e3))
if a.trocken:
    sys.exit(0)

import json as _json
_fp = _json.load(open(os.path.expanduser("~/ros2_ws/farbplan.json"))).get("nester", {}) if os.path.exists(os.path.expanduser("~/ros2_ws/farbplan.json")) else {}
zeilen = "\n".join('    <xacro:ablageplatte_boss name="%s" x="%.4f" y="%.4f" koerper="%s" streifen="%s"/>'
                    % (n, lokal[k, 0], lokal[k, 1], _fp.get(n, {}).get("koerper", "-"), _fp.get(n, {}).get("streifen", "-"))
                   for k, n in enumerate(namen))
xml = '''<?xml version="1.0"?>
<!--
  ================= ERZEUGTE DATEI - NICHT VON HAND AENDERN =================
  Lage der Ablageplatte (robot_base -> ablageplatte) und die Nestbosse, erzeugt von
  tools/ablageplatte_modell.py aus teach_punkte.json (%d Nester), %s.
  Masse in ablageplatte.xacro. Frame `ablageplatte`: Mitte der Platte, z = 0 =
  Plattenunterseite (liegt auf der Grundplatte), +X = Spaltenrichtung (I 1..4).
  Bossfeld im Modell %.1f x %.1f mm (real 51 x 68 - Modellraum, s. Werkzeugkopf).
  ==========================================================================
-->
<robot xmlns:xacro="http://www.ros.org/wiki/xacro">
  <xacro:property name="ablageplatte_x"   value="%.6f"/>
  <xacro:property name="ablageplatte_y"   value="%.6f"/>
  <xacro:property name="ablageplatte_z"   value="0.0"/>
  <xacro:property name="ablageplatte_yaw" value="%.6f"/>
  <xacro:property name="ablageplatte_laenge_x" value="%.4f"/>
  <xacro:property name="ablageplatte_laenge_y" value="%.4f"/>
  <xacro:macro name="ablageplatte_bosse">
%s
  </xacro:macro>
</robot>
''' % (len(namen), date.today().isoformat(), (hi - lo)[0] * 1e3, (hi - lo)[1] * 1e3,
       mitte_platte[0], mitte_platte[1], yaw, laenge[0], laenge[1], zeilen)
open(a.ziel, "w").write(xml)
print("geschrieben: %s" % a.ziel)
