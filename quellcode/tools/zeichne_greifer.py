#!/usr/bin/env python3
"""Zeichnet das KOLLISIONSMODELL des Greifers massstaeblich, zum Vergleich mit dem Foto.

Warum nicht einfach ein RViz-Bild: dort verdeckt der Kamera-Arm den Greifer, die
Sichtmeshes ueberlagern die Kollisionskoerper, und man sieht nicht, was MoveIt
tatsaechlich prueft. Hier wird genau das gezeichnet, was in der Kollisionswelt steht.

Aufruf:  python3 tools/zeichne_greifer.py [--datei ausgabe.png]
"""
import argparse
import re
import subprocess
import sys

import matplotlib
matplotlib.use("Agg")
import matplotlib.pyplot as plt
import numpy as np
from matplotlib.patches import Rectangle

URDF = "/home/er/ros2_ws/src/mycobot_world/urdf/mycobot_world.urdf.xacro"


def lade_urdf():
    return subprocess.run(["xacro", URDF], capture_output=True, text=True, check=True).stdout


def sammle(urdf):
    """Alle Kollisionsboxen der Greiferteile samt Lage in gripper_base (mm)."""
    import xml.etree.ElementTree as ET
    baum = ET.fromstring(urdf)

    # Gelenkkette aufbauen (nur feste/mimic Gelenke im Greifer, Nullstellung)
    eltern, versatz = {}, {}
    for g in baum.findall("joint"):
        k = g.find("child").get("link")
        eltern[k] = g.find("parent").get("link")
        o = g.find("origin")
        versatz[k] = np.array([float(v) for v in o.get("xyz").split()]) if o is not None else np.zeros(3)

    def nach_base(link):
        p = np.zeros(3)
        while link != "gripper_base":
            if link not in eltern:
                return None
            p = p + versatz[link]
            link = eltern[link]
        return p

    teile = []
    for l in baum.findall("link"):
        name = l.get("name")
        if not any(s in name for s in ("gripper", "finger", "halter", "greifer", "tcp",
                                       "charuco", "mechanik")):
            continue
        basis = nach_base(name)
        if basis is None:
            continue
        if name == "tcp":
            teile.append((name, basis * 1000, None))
            continue
        for k in l.findall("collision"):
            geo = k.find("geometry/box")
            if geo is None:
                continue
            groesse = np.array([float(v) for v in geo.get("size").split()])
            o = k.find("origin")
            lokal = np.array([float(v) for v in o.get("xyz").split()]) if o is not None else np.zeros(3)
            teile.append((name, (basis + lokal) * 1000, groesse * 1000))
    return teile


FARBEN = {"finger": "#d62728", "halter": "#1f1f1f", "sockel": "#7f7f7f",
          "mechanik": "#ff7f0e", "charuco": "#1f77b4",
          "huelle": "#2ca02c", "gripper": "#8c8c8c"}


def farbe(name):
    for s, f in FARBEN.items():
        if s in name:
            return f
    return "#8c8c8c"


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--datei", default="/tmp/greifer_modell.png")
    a = ap.parse_args()

    teile = sammle(lade_urdf())
    fig, achsen = plt.subplots(1, 2, figsize=(13, 7))
    ansichten = [(0, 1, "x (Schliessrichtung)", "y (Richtung Finger)", "Draufsicht: x-y"),
                 (2, 1, "z", "y (Richtung Finger)", "Seitenansicht: z-y")]

    for ax, (i, j, xl, yl, titel) in zip(achsen, ansichten):
        for name, mitte, groesse in teile:
            if groesse is None:
                ax.plot(mitte[i], mitte[j], "b*", markersize=18, zorder=5)
                ax.annotate("tcp", (mitte[i], mitte[j]), textcoords="offset points",
                            xytext=(8, 4), color="blue", fontsize=10, weight="bold")
                continue
            durchsichtig = "huelle" in name
            ax.add_patch(Rectangle((mitte[i]-groesse[i]/2, mitte[j]-groesse[j]/2),
                                   groesse[i], groesse[j],
                                   fill=not durchsichtig, alpha=0.35 if not durchsichtig else 1.0,
                                   facecolor=farbe(name), edgecolor=farbe(name),
                                   linewidth=2 if durchsichtig else 1,
                                   linestyle="--" if durchsichtig else "-", zorder=2))
        ax.axhline(0, color="0.8", lw=0.8); ax.axvline(0, color="0.8", lw=0.8)
        ax.set_xlabel(xl + "  [mm]"); ax.set_ylabel(yl + "  [mm]")
        ax.set_title(titel); ax.set_aspect("equal"); ax.grid(alpha=0.25)
        ax.set_xlim(-80, 80); ax.set_ylim(-70, 160)

    fig.suptitle("Kollisionsmodell des Greifers in gripper_base   rot = Finger, "
                 "orange = Viergelenkkette, blau = ChArUco-Platte,\n"
                 "schwarz = Kamerahalter, grau = Sockel, gruen gestrichelt = Koerperhuelle",
                 fontsize=10)
    fig.tight_layout()
    fig.savefig(a.datei, dpi=110)
    print("geschrieben: %s" % a.datei)
    for name, mitte, groesse in sorted(teile, key=lambda t: t[1][1]):
        if groesse is None:
            print("  %-16s Punkt bei y=%6.1f" % (name, mitte[1]))
        else:
            print("  %-16s Mitte (%6.1f %6.1f %6.1f)  Groesse (%5.1f %5.1f %5.1f)"
                  % (name, *mitte, *groesse))
    return 0


if __name__ == "__main__":
    sys.exit(main())
