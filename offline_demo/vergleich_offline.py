#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""Offline-Demo 3 (nach der Abgabe, 11.10.2026): Verfahrensvergleich klassische Bildverarbeitung <-> KI (YOLO11-seg).

Beide Verfahren laufen auf denselben aufgezeichneten Szenen durch DIESELBE Geometrie (welle_in_frame: Stab,
Kopfseite, Ringfarben, Bewertung 0-100); nur die Segmentierung ist verschieden:

  klassisch : Differenz zum Referenzbild der leeren Schale, HSV-Farbklassen, Morphologie, Wasserscheide
  yolo      : Instanzmasken des trainierten YOLO11l-seg-Modells (Klasse = Koerperfarbe), Ausschnitt um die Schale

Ausgefuehrt wird der Code der Erweiterung in ../erweiterung_erkennung/tools/ (Verfahrenswahl), die uebrigen Module
(schale_finden, welle_farbe, ...) kommen unveraendert aus ../quellcode/tools/. Laeuft OHNE Roboter, Kamera und ROS 2
(Windows, Linux, macOS); YOLO rechnet auf der CPU, wenn keine CUDA-Grafikkarte da ist.

Ergebnis in offline_demo/ausgabe/:
  <szene>_vergleich.png      Mosaik links klassisch, rechts KI (Kopf links, rot = Kopfende, gruen = Griffpunkt)
  vergleich_beide.json       je Szene: Anzahl, greifbar, gewaehlte Welle, Zeiten, Uebereinstimmung

    python vergleich_offline.py
    python vergleich_offline.py --wiederholungen 3          # Zeiten als Median aus 3 Laeufen
"""
import argparse
import os
import runpy
import sys
from pathlib import Path

sys.dont_write_bytecode = True

if os.name == "nt" and not sys.flags.utf8_mode:
    import subprocess
    sys.exit(subprocess.call([sys.executable, "-X", "utf8"] + sys.argv))

HIER = Path(__file__).resolve().parent
QUELLCODE = HIER.parent / "quellcode"
TOOLS = QUELLCODE / "tools"
ERW = HIER.parent / "erweiterung_erkennung" / "tools"
AUSGABE = HIER / "ausgabe"
GEWICHTE = QUELLCODE / "src" / "wellenerkennung" / "weights" / "welle_gross.pt"

ap = argparse.ArgumentParser(description=__doc__.split("\n")[0])
ap.add_argument("--frames", nargs="*", default=None)
ap.add_argument("--aus", default=str(AUSGABE))
ap.add_argument("--wiederholungen", type=int, default=1)
ap.add_argument("--geraet", default=None, help="cpu oder 0 (Vorgabe: CUDA, falls vorhanden, sonst CPU)")
arg = ap.parse_args()

for strom in (sys.stdout, sys.stderr):
    try:
        strom.reconfigure(encoding="utf-8", errors="replace")
    except Exception:
        pass

for p, was in ((ERW / "test_welle_finden.py", "erweiterung_erkennung/tools"), (TOOLS / "schale_finden.py", "quellcode/tools")):
    if not p.is_file():
        sys.exit("Ordner %s nicht gefunden - bitte das Repository vollstaendig herunterladen." % was)
if not GEWICHTE.is_file():
    # wie yolo_offline.py: fehlen die Gewichte (z.B. ZIP ohne Git LFS), aus dem Release laden
    import urllib.request
    url = "https://github.com/kakir001/bachelorarbeit-ros2-code/releases/download/v1.0-abgabe/welle_gross.pt"
    print("Lade Modellgewichte von %s ..." % url)
    GEWICHTE.parent.mkdir(parents=True, exist_ok=True)
    urllib.request.urlretrieve(url, str(GEWICHTE))
os.environ["WELLE_GEWICHTE"] = str(GEWICHTE)

# Suchpfad: ROS-Platzhalter, dann die Erweiterung (welle_finden_schale, test_welle_finden, yolo_segmentierung), dann der Abgabestand
sys.path.insert(0, str(TOOLS))
sys.path.insert(0, str(ERW))
sys.path.insert(0, str(HIER / "ros_stubs"))

_expanduser = os.path.expanduser


def _umgelenkt(pfad):
    p = os.fspath(pfad)
    if isinstance(p, str) and (p == "~/ros2_ws" or p.startswith("~/ros2_ws/")):
        return str(QUELLCODE) + p[len("~/ros2_ws"):].replace("/", os.sep)
    return _expanduser(pfad)


os.path.expanduser = _umgelenkt

sys.argv = [str(ERW / "welle_finden_schale.py")]
import welle_finden_schale  # noqa: E402,F401  (setzt sys.argv zurueck)

testdaten = QUELLCODE / "testdaten"
frames = [str(Path(f).resolve()) for f in arg.frames] if arg.frames else sorted(
    str(f) for f in testdaten.glob("*.npz") if "hintergrund" not in f.name)
aus = Path(arg.aus).resolve(); aus.mkdir(parents=True, exist_ok=True)
sys.argv = [str(ERW / "test_welle_finden.py"), "--verfahren", "beide", "--aus", str(aus), "--wiederholungen", str(arg.wiederholungen),
            "--hintergrund", str(testdaten / "schale_hintergrund_2026-09-13.npz"), "--frames"] + frames
if arg.geraet:
    sys.argv += ["--geraet", arg.geraet]
try:
    runpy.run_path(str(ERW / "test_welle_finden.py"), run_name="__main__")
except SystemExit as ende:
    if ende.code not in (0, None):
        raise
print("Bilder und vergleich_beide.json in: %s" % aus)
