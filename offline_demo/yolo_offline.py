#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""Offline-Demo 2: KI-gestuetzte Instanzsegmentierung (YOLO11l-seg) auf aufgezeichneten Szenen.

Laeuft OHNE Roboter, OHNE Kamera und OHNE ROS 2 (Windows, Linux, macOS; auch nur mit CPU).
Verwendet werden die trainierten Gewichte der Arbeit (welle_gross.pt, Abschnitt 5.3.2) und die
UNVERAENDERTEN Auswertefunktionen aus

    quellcode/src/wellenerkennung/wellenerkennung/erkennung.py
        wellen_erkennen()        Inferenz, Instanzmasken              (Quelltext 5-2)
        get_mask_orientation()   Mittelpunkt + Laengsachse per PCA    (Abschnitt 5.4.1, 5.4.4)
        kopf_bewusster_griff()   breiteres Ende = Kopf, Griffpunkt

Eingabe sind die Farbbilder der am 13.09.2026 aufgezeichneten Szenen in quellcode/testdaten/*.npz.
Je Szene entsteht ein PNG in offline_demo/ausgabe/: Maske farbig, Laengsachse (magenta),
Mittelpunkt (gruen), Kopfende (rot, falls eindeutig), Klasse und Konfidenz.

    python yolo_offline.py                      # alle Szenen, Schwelle 0.70 wie im Detektorknoten
    python yolo_offline.py --conf 0.5
    python yolo_offline.py --gpu                # Vorgabe ist die CPU (ca. 1-10 s je Bild)
    python yolo_offline.py --gewichte C:\\Pfad\\welle_gross.pt

Die Gewichte (54 MB) liegen im Repository unter quellcode/src/wellenerkennung/weights/ und
zusaetzlich am Release "v1.0-abgabe". Fehlen sie, werden sie von dort nach
offline_demo/welle_gross.pt geladen.
"""
import argparse
import math
import os
import sys
import time
from pathlib import Path

sys.dont_write_bytecode = True  # keine __pycache__-Ordner im Quellcode-Ordner anlegen

if os.name == "nt" and not sys.flags.utf8_mode:
    import subprocess
    sys.exit(subprocess.call([sys.executable, "-X", "utf8"] + sys.argv))
if "--gpu" not in sys.argv:
    os.environ.setdefault("CUDA_VISIBLE_DEVICES", "")  # CPU: keine GPU-Initialisierung (auf dem Jetson Nano sonst Speichermangel)

HIER = Path(__file__).resolve().parent
QUELLCODE = HIER.parent / "quellcode"
AUSGABE = HIER / "ausgabe"
RELEASE_URL = "https://github.com/kakir001/bachelorarbeit-ros2-code/releases/download/v1.0-abgabe/welle_gross.pt"
GROESSE_SOLL = 55842202  # Bytes

ap = argparse.ArgumentParser(description=__doc__.split("\n")[0])
ap.add_argument("--frames", nargs="*", default=None, help="npz-Szenen (Vorgabe: alle in quellcode/testdaten/)")
ap.add_argument("--gewichte", default="", help="Pfad zu welle_gross.pt")
ap.add_argument("--conf", type=float, default=0.70, help="Konfidenzschwelle (Vorgabe 0.70 wie im Detektorknoten)")
ap.add_argument("--aus", default=str(AUSGABE))
ap.add_argument("--gpu", action="store_true", help="auf der GPU rechnen (Vorgabe: CPU - laeuft ueberall gleich)")
arg = ap.parse_args()

for strom in (sys.stdout, sys.stderr):
    try:
        strom.reconfigure(encoding="utf-8", errors="replace")
    except Exception:
        pass


def gewichte_finden():
    kandidaten = [Path(arg.gewichte)] if arg.gewichte else []
    kandidaten += [QUELLCODE / "src" / "wellenerkennung" / "weights" / "welle_gross.pt",
                   HIER / "welle_gross.pt",
                   HIER.parent / "welle_gross.pt"]
    for k in kandidaten:
        if k.is_file():
            return k
    if arg.gewichte:
        sys.exit("Gewichte nicht gefunden: %s" % arg.gewichte)
    ziel = HIER / "welle_gross.pt"
    print("welle_gross.pt nicht gefunden - lade aus dem Release v1.0-abgabe (54 MB) ...")
    print("  ", RELEASE_URL)
    try:
        import urllib.request
        teil = ziel.with_suffix(".teil")
        urllib.request.urlretrieve(RELEASE_URL, str(teil))
        teil.replace(ziel)
    except Exception as fehler:
        sys.exit("Herunterladen fehlgeschlagen (%s).\nBitte die Datei von Hand laden:\n  %s\nund ablegen als:\n  %s"
                 % (fehler, RELEASE_URL, ziel))
    return ziel


gewichte = gewichte_finden()
if gewichte.stat().st_size != GROESSE_SOLL:
    print("  Hinweis: %s hat %d Bytes, erwartet %d." % (gewichte, gewichte.stat().st_size, GROESSE_SOLL))

try:
    import cv2
    import numpy as np
    from ultralytics import YOLO
except ImportError as fehler:
    sys.exit("Paket fehlt (%s). Bitte zuerst:  pip install -r requirements-offline.txt" % fehler)

sys.path.insert(0, str(QUELLCODE / "src" / "wellenerkennung"))
from wellenerkennung.erkennung import CLASS_NAMES, kopf_bewusster_griff, wellen_erkennen  # noqa: E402

FARBE_BGR = {"gelb": (0, 220, 255), "weiss": (255, 255, 255), "schwarz": (90, 90, 90), "gruen": (0, 200, 0), "rot": (0, 0, 255)}

print("Gewichte:", gewichte)
modell = YOLO(str(gewichte))
print("Modellklassen (intern):", modell.names, "->", CLASS_NAMES)
if not arg.gpu:
    predict = modell.predict
    modell.predict = lambda *a, **k: predict(*a, **dict(k, device="cpu"))

testdaten = QUELLCODE / "testdaten"
frames = [Path(f) for f in arg.frames] if arg.frames else sorted(f for f in testdaten.glob("*.npz") if "hintergrund" not in f.name)
if not frames:
    sys.exit("Keine Szenen in %s gefunden." % testdaten)
aus = Path(arg.aus).resolve()
aus.mkdir(parents=True, exist_ok=True)

tabelle = []
for pfad in frames:
    bild = np.load(str(pfad), allow_pickle=True)["farbe"]  # BGR, 1280 x 720
    t0 = time.time()
    funde = wellen_erkennen(bild, modell, conf=arg.conf)
    dauer = time.time() - t0
    zeichnung = bild.copy()
    je_klasse = {}
    kopf_bekannt = 0
    for f in funde:
        maske = f["mask"] > 0
        farbe = FARBE_BGR.get(f["class_name"], (255, 0, 255))
        zeichnung[maske] = (0.55 * zeichnung[maske] + 0.45 * np.array(farbe)).astype(np.uint8)
        kontur = cv2.findContours(f["mask"], cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)[0]
        cv2.drawContours(zeichnung, kontur, -1, farbe, 1)
    for f in funde:
        maske = f["mask"] > 0
        cx, cy = f["center_px"]
        a = math.radians(f["angle_deg"])
        halb = 0.5 * f["major_len_px"]
        p0 = (int(round(cx - math.cos(a) * halb)), int(round(cy - math.sin(a) * halb)))
        p1 = (int(round(cx + math.cos(a) * halb)), int(round(cy + math.sin(a) * halb)))
        cv2.line(zeichnung, p0, p1, (255, 0, 255), 2)
        cv2.circle(zeichnung, (int(round(cx)), int(round(cy))), 4, (0, 255, 0), -1)
        griff = kopf_bewusster_griff(maske, cx, cy, f["angle_deg"])
        if griff["head_known"]:
            kopf_bekannt += 1
            cv2.circle(zeichnung, tuple(int(round(v)) for v in griff["head_px"]), 7, (0, 0, 255), 2)
        text = "%s %.2f" % (f["class_name"], f["conf"])
        ort = (int(f["bbox"][0]), max(14, int(f["bbox"][1]) - 5))
        cv2.putText(zeichnung, text, ort, cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 0, 0), 3)
        cv2.putText(zeichnung, text, ort, cv2.FONT_HERSHEY_SIMPLEX, 0.5, (255, 255, 255), 1)
        je_klasse[f["class_name"]] = je_klasse.get(f["class_name"], 0) + 1
    kopfzeile = "YOLO11l-seg, conf >= %.2f: %d Wellen  (magenta = PCA-Achse, gruen = Mittelpunkt, rot = Kopfende)" % (arg.conf, len(funde))
    cv2.putText(zeichnung, kopfzeile, (12, 30), cv2.FONT_HERSHEY_SIMPLEX, 0.65, (0, 255, 0), 2)
    ziel = aus / (pfad.stem + "_yolo.png")
    cv2.imwrite(str(ziel), zeichnung)
    print("\n%s: %d Wellen in %.1f s  %s" % (pfad.stem, len(funde), dauer, je_klasse))
    for nr, f in enumerate(funde, 1):
        print("  %2d  %-8s conf %.2f  Mitte (%4.0f, %4.0f) px  Achse %6.1f Grad  Laenge %3.0f px"
              % (nr, f["class_name"], f["conf"], f["center_px"][0], f["center_px"][1], f["angle_deg"], f["major_len_px"]))
    print("  Bild:", ziel)
    tabelle.append((pfad.stem, len(funde), kopf_bekannt, dauer))

print("\n" + "=" * 78)
print("ZUSAMMENFASSUNG (Offline-Demo 2, YOLO11l-seg, Schwelle %.2f)" % arg.conf)
print("%-44s %8s %14s %8s" % ("Szene", "Wellen", "Kopf eindeutig", "Zeit s"))
for name, n, k, d in tabelle:
    print("%-44s %8d %14d %8.1f" % (name, n, k, d))
print("Bilder in: %s" % aus)
