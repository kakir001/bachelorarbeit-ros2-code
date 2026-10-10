#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""Offline-Demo 1: Wellen- und Kopfseitenerkennung mit klassischer Bildverarbeitung (ohne KI) auf aufgezeichneten Szenen.

Laeuft OHNE Roboter, OHNE Kamera und OHNE ROS 2 (Windows, Linux, macOS). Ausgefuehrt wird
der UNVERAENDERTE Code der Arbeit:

    quellcode/tools/test_welle_finden.py   ->  quellcode/tools/welle_finden_schale.py (welle_in_frame)

auf den am 13.09.2026 aufgezeichneten RGB-D-Szenen in quellcode/testdaten/ (Abschnitt 5.8.2 der
Arbeit: Szenen mit 11 bzw. 12 Wellen). Dieses Skript aendert nichts an diesem Code; es sorgt nur
dafuer, dass er ausserhalb des Jetson startet:

  1. ros_stubs/ wird vor den Suchpfad gestellt. welle_finden_schale.py und schale_finden.py
     importieren auf Modulebene rclpy, tf2_ros und Nachrichtentypen; die Bildverarbeitung selbst
     (welle_in_frame) benutzt davon nichts.
  2. Die im Code festen Vorgabepfade "~/ros2_ws/..." werden auf den Ordner quellcode/ dieses
     Repositorys umgelenkt (Kamera-Modell-Abbildung kamera_modell_platte.json, testdaten/).
  3. Die Mosaike werden nach offline_demo/ausgabe/ geschrieben, nicht in den Quellcode-Ordner.

Zusaetzlich entsteht je Szene ein Uebersichtsbild (ganzes Kamerabild, je Welle Kopfende rot,
Spitze blau, Achse magenta, laufende Nummer) und am Ende eine Tabelle "Szene / Anzahl Wellen".

    python wellenerkennung_offline.py                 # alle Szenen
    python wellenerkennung_offline.py --frames ../quellcode/testdaten/wellen_12_stueck_verteilt_2026-09-13.npz
"""
import argparse
import os
import runpy
import sys
from pathlib import Path

sys.dont_write_bytecode = True  # keine __pycache__-Ordner im Quellcode-Ordner anlegen

if os.name == "nt" and not sys.flags.utf8_mode:
    # Der Code der Arbeit oeffnet seine UTF-8-Dateien ohne encoding-Angabe (auf dem Jetson Vorgabe UTF-8).
    import subprocess
    sys.exit(subprocess.call([sys.executable, "-X", "utf8"] + sys.argv))

HIER = Path(__file__).resolve().parent
QUELLCODE = HIER.parent / "quellcode"
TOOLS = QUELLCODE / "tools"
AUSGABE = HIER / "ausgabe"

ap = argparse.ArgumentParser(description=__doc__.split("\n")[0])
ap.add_argument("--frames", nargs="*", default=None, help="npz-Szenen (Vorgabe: alle in quellcode/testdaten/)")
ap.add_argument("--aus", default=str(AUSGABE), help="Ausgabeordner (Vorgabe: offline_demo/ausgabe)")
ap.add_argument("--echtes-ros", action="store_true",
                help="ros_stubs NICHT verwenden (nur zum Vergleich auf einem Rechner mit ROS 2)")
arg = ap.parse_args()

for strom in (sys.stdout, sys.stderr):
    try:
        strom.reconfigure(encoding="utf-8", errors="replace")
    except Exception:
        pass

if not (TOOLS / "test_welle_finden.py").is_file():
    sys.exit("Ordner %s nicht gefunden - bitte das Repository vollstaendig herunterladen." % TOOLS)

# 1. ROS-Platzhalter
if not arg.echtes_ros:
    sys.path.insert(0, str(HIER / "ros_stubs"))
sys.path.insert(0, str(TOOLS))

# 2. "~/ros2_ws" -> quellcode/
_expanduser = os.path.expanduser


def _umgelenkt(pfad):
    p = os.fspath(pfad)
    if isinstance(p, str) and (p == "~/ros2_ws" or p.startswith("~/ros2_ws/")):
        return str(QUELLCODE) + p[len("~/ros2_ws"):].replace("/", os.sep)
    return _expanduser(pfad)


os.path.expanduser = _umgelenkt

import cv2  # noqa: E402
import numpy as np  # noqa: E402

sys.argv = [str(TOOLS / "welle_finden_schale.py")]
import welle_finden_schale as wfs  # noqa: E402  (setzt sys.argv zurueck; deshalb HIER zuerst importieren)

import rclpy  # noqa: E402
print("ROS: %s" % ("Platzhalter aus offline_demo/ros_stubs (kein ROS noetig)" if getattr(rclpy, "OFFLINE_PLATZHALTER", False)
                   else "echte ROS-2-Installation (%s)" % os.path.dirname(rclpy.__file__)))
print("Kamera-Modell-Abbildung: %s" % ("geladen" if wfs.ABB is not None else "NICHT geladen"))

# Jeden Aufruf von welle_in_frame mitschreiben (fuer Uebersichtsbild und Tabelle); die Funktion selbst bleibt unveraendert.
_original = wfs.welle_in_frame
mitschrift = []


def _welle_in_frame(farbe, *args, **kwargs):
    ergebnis = _original(farbe, *args, **kwargs)
    mitschrift.append((farbe, ergebnis))
    return ergebnis


wfs.welle_in_frame = _welle_in_frame

aus = Path(arg.aus).resolve()
aus.mkdir(parents=True, exist_ok=True)
testdaten = QUELLCODE / "testdaten"
frames = [str(Path(f).resolve()) for f in arg.frames] if arg.frames else sorted(
    str(f) for f in testdaten.glob("*.npz") if "hintergrund" not in f.name)
if not frames:
    sys.exit("Keine Szenen in %s gefunden." % testdaten)

tabelle = []
for frame in frames:
    mitschrift.clear()
    sys.argv = [str(TOOLS / "test_welle_finden.py"), "--frames", frame, "--aus", str(aus),
                "--hintergrund", str(testdaten / "schale_hintergrund_2026-09-13.npz")]
    try:
        runpy.run_path(str(TOOLS / "test_welle_finden.py"), run_name="__main__")
    except SystemExit as ende:
        if ende.code not in (0, None):
            raise
    name = Path(frame).stem
    wellen = []
    for farbe, ergebnis in mitschrift:
        if isinstance(ergebnis, dict):
            wellen = sorted([ergebnis] + list(ergebnis.get("weitere", [])), key=lambda w: w["i"])
            bild = farbe.copy()
    if wellen:
        for nr, w in enumerate(wellen, 1):
            kopf = tuple(int(round(v)) for v in w["kopf_px"])
            spitze = tuple(int(round(v)) for v in w["spitze_px"])
            cv2.line(bild, kopf, spitze, (255, 0, 255), 2)
            cv2.circle(bild, kopf, 7, (0, 0, 255), 2)
            cv2.circle(bild, spitze, 5, (255, 128, 0), 2)
            ort = (kopf[0] + 9, kopf[1] - 9)
            cv2.putText(bild, str(nr), ort, cv2.FONT_HERSHEY_SIMPLEX, 0.8, (0, 0, 0), 4)
            cv2.putText(bild, str(nr), ort, cv2.FONT_HERSHEY_SIMPLEX, 0.8, (255, 255, 255), 2)
        cv2.putText(bild, "%s: %d Wellen  (rot = Kopfende, blau = Spitze)" % (name, len(wellen)), (12, 30),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.8, (0, 255, 0), 2)
        ziel = aus / (name + "_uebersicht.png")
        cv2.imwrite(str(ziel), bild)
        print("  Uebersicht:", ziel)
    sicher = sum(1 for w in wellen if "sicher" in str(w.get("kopf_wie", "")) and "unsicher" not in str(w.get("kopf_wie", "")))
    tabelle.append((name, len(wellen), sicher))

print("\n" + "=" * 78)
print("ZUSAMMENFASSUNG (Offline-Demo 1, klassische Bildverarbeitung)")
print("%-44s %8s" % ("Szene", "Wellen"))
for name, n, _ in tabelle:
    print("%-44s %8d" % (name, n))
print("Bilder in: %s" % aus)
print("Ob Kopf (rot) und Spitze (blau) je Welle richtig liegen, zeigen die Bilder *_uebersicht.png und *_mosaik.png.")
