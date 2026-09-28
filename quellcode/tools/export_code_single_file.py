#!/usr/bin/env python3
"""Exportiert den GESAMTEN selbst geschriebenen Quellcode in EINE Textdatei.

Zweck: die Datei laesst sich direkt in einen Chat hochladen (ChatGPT o.ae.), damit
die Implementierung Datei fuer Datei gegen die Kapitel der Thesis geprueft werden
kann — ohne den Code irgendwo zu veroeffentlichen (Sperrvermerk).

Enthalten:  eigene ROS-2-Pakete (src/mycobot_*, src/wellenerkennung), Skripte im
            Wurzelverzeichnis, tools/, sowie Konfiguration: launch/, URDF/xacro,
            SRDF, YAML, RViz-Konfigurationen.
Nicht enthalten: Upstream-Pakete (mycobot_ros2, realsense-ros, easy_handeye2,
            trac_ik), build/install/log, Modellgewichte, Datensaetze, Bilder,
            *.bak*, __pycache__.

Aufruf:  python3 tools/export_code_single_file.py [ZIELDATEI]
"""
import os
import subprocess
import sys
from datetime import date

WS = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
OUT = sys.argv[1] if len(sys.argv) > 1 else os.path.join(
    os.path.expanduser("~/Schreibtisch"), "BA_CODE_%s.txt" % date.today().isoformat())

EIGENE_PAKETE = ("mycobot_calibration", "mycobot_demo", "mycobot_hardware",
                 "mycobot_moveit_config", "mycobot_world", "wellenerkennung")
ENDUNGEN = (".py", ".cpp", ".hpp", ".h", ".sh", ".yaml", ".yml", ".xml",
            ".xacro", ".urdf", ".srdf", ".rviz", ".json", ".cfg", ".txt", ".md")
AUS = ("build", "install", "log", "__pycache__", "weights", "datasets",
       "boards", "resource", ".git", "calibration_backups", "logs")


def passt(pfad):
    rel = os.path.relpath(pfad, WS)
    teile = rel.split(os.sep)
    if any(t in AUS for t in teile):
        return False
    if ".bak" in os.path.basename(pfad):
        return False
    if not pfad.endswith(ENDUNGEN):
        return False
    if teile[0] == "src" and len(teile) > 1 and teile[1] not in EIGENE_PAKETE:
        return False   # Upstream aussortieren
    if teile[0] == "patches":
        return False
    return True


def sammeln():
    treffer = []
    for dp, dn, fn in os.walk(WS):
        dn[:] = [d for d in dn if d not in AUS]
        for f in sorted(fn):
            p = os.path.join(dp, f)
            if passt(p):
                treffer.append(p)
    # DEVLOG ist ein Tagebuch, kein Quellcode -> separat, nicht hier
    return sorted(t for t in treffer if os.path.basename(t) != "DEVLOG.md")


def main():
    dateien = sammeln()
    try:
        head = subprocess.check_output(["git", "-C", WS, "rev-parse", "--short", "HEAD"]).decode().strip()
    except Exception:
        head = "unbekannt"

    zeilen_gesamt = 0
    with open(OUT, "w", encoding="utf-8") as out:
        out.write("=" * 78 + "\n")
        out.write("BACHELORARBEIT — GESAMTER SELBST GESCHRIEBENER QUELLCODE\n")
        out.write("Stand: %s   Git-Commit: %s\n" % (date.today().isoformat(), head))
        out.write("Workspace: ~/ros2_ws (ROS 2 Galactic, NVIDIA Jetson Nano)\n")
        out.write("Thema: Wellenerkennung (YOLO11-seg + RealSense D435if) und\n")
        out.write("       robotische Handhabung mit myCobot 280 JN\n")
        out.write("=" * 78 + "\n\n")
        out.write("NICHT enthalten: Upstream-Pakete (mycobot_ros2, realsense-ros,\n")
        out.write("easy_handeye2, trac_ik), Modellgewichte, Datensaetze, build/install/log.\n\n")
        out.write("INHALTSVERZEICHNIS (%d Dateien)\n" % len(dateien))
        out.write("-" * 78 + "\n")
        for i, p in enumerate(dateien, 1):
            n = sum(1 for _ in open(p, encoding="utf-8", errors="replace"))
            zeilen_gesamt += n
            out.write("%3d. %-62s %5d Z.\n" % (i, os.path.relpath(p, WS), n))
        out.write("-" * 78 + "\n")
        out.write("Summe: %d Dateien, %d Zeilen\n\n\n" % (len(dateien), zeilen_gesamt))

        for i, p in enumerate(dateien, 1):
            rel = os.path.relpath(p, WS)
            out.write("\n" + "=" * 78 + "\n")
            out.write("### DATEI %d/%d: %s\n" % (i, len(dateien), rel))
            out.write("=" * 78 + "\n")
            with open(p, encoding="utf-8", errors="replace") as f:
                out.write(f.read())
            if not out.tell() or True:
                out.write("\n")

    print("geschrieben: %s" % OUT)
    print("%d Dateien, %d Zeilen, %.2f MB"
          % (len(dateien), zeilen_gesamt, os.path.getsize(OUT) / 1e6))


if __name__ == "__main__":
    main()
