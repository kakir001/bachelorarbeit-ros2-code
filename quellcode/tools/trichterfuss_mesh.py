#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""Trichterfuss als STL: Halbscheibe (Radius r, Halbkreis nach +X) mit OVALEN Ecken an der geraden
Kante (Benutzer 2026-09-14: die Ecken sind real rund, nicht spitz). Ursprung = Mitte der geraden
Kante (die Kante laeuft in y), z 0..dicke. Ersetzt meshes/halbscheibe_r60.stl (spitze Ecken, 12.9.).

    python3 tools/trichterfuss_mesh.py --r 60 --ecke 20 --dicke 4 --aus src/mycobot_world/meshes/trichterfuss_r60_oval.stl
"""
import argparse, math, struct
ap = argparse.ArgumentParser()
ap.add_argument("--r", type=float, default=60.0, help="[mm] Halbkreisradius")
ap.add_argument("--ecke", type=float, default=20.0, help="[mm] Rundungsradius der beiden Ecken an der geraden Kante")
ap.add_argument("--dicke", type=float, default=4.0); ap.add_argument("--n", type=int, default=48)
ap.add_argument("--aus", default="src/mycobot_world/meshes/trichterfuss_r60_oval.stl")
a = ap.parse_args()
R, e, d = a.r / 1e3, a.ecke / 1e3, a.dicke / 1e3
# Umriss gegen den Uhrzeigersinn. Gerade Kante bei x = 0 (laeuft in y), Halbkreis um (0,0) mit Radius R nach +X.
# Eckrundung mit Radius e: ihr Mittelpunkt liegt bei x = e (tangential zur geraden Kante) UND im Abstand R-e vom
# Ursprung (innen tangential zum grossen Kreis) -> Mittelpunkt (e, +-cy) mit cy = sqrt((R-e)^2 - e^2).
# Die Rundung laeuft von der Kante (Winkel 180 Grad) bis zum Beruehrpunkt mit dem grossen Kreis (Richtung des
# Eckmittelpunkts, Winkel atan2(cy, e)); dazwischen der grosse Bogen.
cy = math.sqrt((R - e) ** 2 - e ** 2); psi = math.atan2(cy, e)
def bogen(cx, cy_, r, a0, a1, n):
    return [(cx + r * math.cos(a0 + (a1 - a0) * i / n), cy_ + r * math.sin(a0 + (a1 - a0) * i / n)) for i in range(n + 1)]
pts = []
pts += bogen(e, -cy, e, math.pi, 2 * math.pi - psi, 10)        # untere Ecke: 180 -> 360-psi (im Uhrzeigersinn ueber 270)
pts += bogen(0, 0, R, -psi, psi, a.n)                            # grosser Bogen
pts += bogen(e, cy, e, psi, math.pi, 10)                         # obere Ecke: psi -> 180
# Doppelte/entartete Punkte raus
u = []
for p in pts:
    if not u or math.hypot(p[0] - u[-1][0], p[1] - u[-1][1]) > 1e-6: u.append(p)
if math.hypot(u[0][0] - u[-1][0], u[0][1] - u[-1][1]) < 1e-6: u.pop()
tri = []
c = (sum(p[0] for p in u) / len(u), sum(p[1] for p in u) / len(u))
m = len(u)
for i in range(m):
    p, q = u[i], u[(i + 1) % m]
    tri.append(((c[0], c[1], d), (p[0], p[1], d), (q[0], q[1], d)))            # Deckel (Normale +z)
    tri.append(((c[0], c[1], 0.0), (q[0], q[1], 0.0), (p[0], p[1], 0.0)))      # Boden (Normale -z)
    tri.append(((p[0], p[1], 0.0), (q[0], q[1], 0.0), (q[0], q[1], d)))        # Mantel
    tri.append(((p[0], p[1], 0.0), (q[0], q[1], d), (p[0], p[1], d)))
def normal(t):
    (x0, y0, z0), (x1, y1, z1), (x2, y2, z2) = t
    ux, uy, uz = x1 - x0, y1 - y0, z1 - z0; vx, vy, vz = x2 - x0, y2 - y0, z2 - z0
    nx, ny, nz = uy * vz - uz * vy, uz * vx - ux * vz, ux * vy - uy * vx; l = math.sqrt(nx * nx + ny * ny + nz * nz) or 1.0
    return nx / l, ny / l, nz / l
with open(a.aus, "wb") as f:
    f.write(b"trichterfuss oval".ljust(80, b"\0")); f.write(struct.pack("<I", len(tri)))
    for t in tri:
        f.write(struct.pack("<3f", *normal(t)))
        for v in t: f.write(struct.pack("<3f", *v))
        f.write(struct.pack("<H", 0))
xs = [p[0] for p in u]; ys = [p[1] for p in u]
print("%s: %d Dreiecke, x %.1f..%.1f mm, y %.1f..%.1f mm, gerade Kante y +-%.1f, Ecken r %.0f" % (a.aus, len(tri), min(xs) * 1e3, max(xs) * 1e3, min(ys) * 1e3, max(ys) * 1e3, cy * 1e3, a.ecke))
