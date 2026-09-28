#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""Bereitstellungsschale als STL-Mesh mit RUNDEN Kanten (Benutzer 2026-09-14: "keine eckige Kante, jede
Ecke oval - unten, oben, aussen, innen"). Ersetzt die Sehnen-Kaesten aus schale.xacro fuer Visual UND
Kollision (die Kaesten bleiben als Alternative im xacro).

Geometrie wie bisher (Frame `schale`: Ursprung = Ringmittelpunkt, z 0 = Innenboden, +X = Winkelhalbierende):
  Wand = ein geschlossener Zug aus innerem Bogen (r_innen + wand/2), aeusserem Bogen (r_aussen - wand/2)
  und den beiden radialen Flanken (+-winkel/2); die vier Ecken in der Draufsicht mit Radius --ecke gerundet.
  Querschnitt der Wand: Breite wand, Hoehe 0..hoehe, oben UND unten halbrund (Radius wand/2).
  Das Profil wird entlang des Zugs "gezogen" (Sweep) -> geschlossene Roehre = Wand. Dazu der Boden als
  eigene duenne Scheibe (nur Visual im xacro; hier als zweites Mesh, --boden).

    python3 tools/schale_mesh.py                     # schale_wand.stl + schale_boden.stl nach meshes/
    python3 tools/schale_mesh.py --ecke 12 --winkel 45.3 --r-innen 186.9 --r-aussen 301.9 --wand 2 --hoehe 17
"""
import argparse, math, struct, os
ap = argparse.ArgumentParser()
ap.add_argument("--r-innen", type=float, default=186.9, help="[mm] Innenflaeche der inneren Wand")
ap.add_argument("--r-aussen", type=float, default=301.9, help="[mm] Aussenflaeche der aeusseren Wand")
ap.add_argument("--winkel", type=float, default=45.3, help="[Grad] Sektor")
ap.add_argument("--wand", type=float, default=2.0); ap.add_argument("--hoehe", type=float, default=17.0)
ap.add_argument("--ecke", type=float, default=10.0, help="[mm] Rundung der vier Ecken (Wandmitte) in der Draufsicht")
ap.add_argument("--boden-dicke", type=float, default=2.0)
ap.add_argument("--schritt", type=float, default=2.0, help="[mm] Punktabstand entlang des Zugs")
ap.add_argument("--profil-n", type=int, default=10, help="Punkte je Halbrundung")
ap.add_argument("--aus-dir", default=os.path.join(os.path.dirname(os.path.abspath(__file__)), "..", "src", "mycobot_world", "meshes"))
a = ap.parse_args()

ri, ro, th = (a.r_innen + a.wand / 2) / 1e3, (a.r_aussen - a.wand / 2) / 1e3, math.radians(a.winkel / 2)
w, h, e, st = a.wand / 1e3, a.hoehe / 1e3, a.ecke / 1e3, a.schritt / 1e3


def pol(r, phi): return (r * math.cos(phi), r * math.sin(phi))


def bogen(r, p0, p1):
    n = max(2, int(abs(p1 - p0) * r / st)); return [pol(r, p0 + (p1 - p0) * i / n) for i in range(n + 1)]


def kreis(c, r, a0, a1):
    n = max(3, int(abs(a1 - a0) * r / st)); return [(c[0] + r * math.cos(a0 + (a1 - a0) * i / n), c[1] + r * math.sin(a0 + (a1 - a0) * i / n)) for i in range(n + 1)]


# --- Zug in der Draufsicht (gegen den Uhrzeigersinn), Ecken gerundet -----------------------------------
# Eckmittelpunkte: aussen bei Radius ro-e, innen bei ri+e, jeweils um d = asin(e / R') von der Flanke weg,
# damit der Eckkreis die radiale Flanke beruehrt. Tangentenpunkte: auf dem Bogen bei Winkel +-(th - d),
# auf der Flanke beim Radius des Eckmittelpunkts.
d_o = math.asin(e / (ro - e)); d_i = math.asin(e / (ri + e))
c_oa = pol(ro - e, th - d_o); c_ob = pol(ro - e, -(th - d_o))      # aeussere Ecken (+phi / -phi)
c_ia = pol(ri + e, th - d_i); c_ib = pol(ri + e, -(th - d_i))      # innere Ecken
# Zug nur fuer die +phi-Haelfte bauen (aussen -> Ecke -> Flanke -> Ecke -> innen) und an y = 0 spiegeln.
fa, fb = pol(ro - e, th), pol(ri + e, th)
n = max(2, int(math.hypot(fa[0] - fb[0], fa[1] - fb[1]) / st))
half = []
half += bogen(ro, 0.0, th - d_o)
half += kreis(c_oa, e, th - d_o, th + math.pi / 2)
half += [(fa[0] + (fb[0] - fa[0]) * i / n, fa[1] + (fb[1] - fa[1]) * i / n) for i in range(1, n)]
half += kreis(c_ia, e, th + math.pi / 2, math.pi + (th - d_i))
half += bogen(ri, th - d_i, 0.0)
# volle Schleife: obere Haelfte (y >= 0) + gespiegelte untere Haelfte rueckwaerts
zug = half + [(p[0], -p[1]) for p in reversed(half[1:-1])]
u = []
for p in zug:
    if not u or math.hypot(p[0] - u[-1][0], p[1] - u[-1][1]) > 1e-7: u.append(p)
if math.hypot(u[0][0] - u[-1][0], u[0][1] - u[-1][1]) < 1e-7: u.pop()
zug = u; m = len(zug)

# --- Profil (Querschnitt): u = quer zur Wand (nach aussen der Schleife positiv), v = z ----------------------
r = w / 2
prof = []
prof += [(r * math.cos(t), h - r + r * math.sin(t)) for t in [i * math.pi / a.profil_n for i in range(0, a.profil_n + 1)]]         # oben halbrund, von +u nach -u
prof += [(-r * math.cos(t), r - r * math.sin(t)) for t in [i * math.pi / a.profil_n for i in range(1, a.profil_n)]]                 # unten halbrund
k = len(prof)

# --- Sweep: je Zugpunkt die Normale (nach aussen der Schleife = rechts der Laufrichtung bei ccw) --------------
def normale(i):
    p0, p1 = zug[i - 1], zug[(i + 1) % m]; tx, ty = p1[0] - p0[0], p1[1] - p0[1]; l = math.hypot(tx, ty) or 1.0
    return (ty / l, -tx / l)
# Umlaufsinn pruefen (ccw -> Flaeche > 0), sonst umdrehen
fl = sum(zug[i][0] * zug[(i + 1) % m][1] - zug[(i + 1) % m][0] * zug[i][1] for i in range(m))
if fl < 0: zug = zug[::-1]
ringe = []
for i in range(m):
    nx, ny = normale(i); c = zug[i]
    ringe.append([(c[0] + nx * pu, c[1] + ny * pu, pv) for pu, pv in prof])
tri = []
for i in range(m):
    A, B = ringe[i], ringe[(i + 1) % m]
    for j in range(k):
        j2 = (j + 1) % k
        tri.append((A[j], B[j], B[j2])); tri.append((A[j], B[j2], A[j2]))


def normal(t):
    (x0, y0, z0), (x1, y1, z1), (x2, y2, z2) = t
    ux, uy, uz = x1 - x0, y1 - y0, z1 - z0; vx, vy, vz = x2 - x0, y2 - y0, z2 - z0
    nx, ny, nz = uy * vz - uz * vy, uz * vx - ux * vz, ux * vy - uy * vx; l = math.sqrt(nx * nx + ny * ny + nz * nz) or 1.0
    return nx / l, ny / l, nz / l


def stl(pfad, tris, name):
    with open(pfad, "wb") as f:
        f.write(name.encode().ljust(80, b"\0")); f.write(struct.pack("<I", len(tris)))
        for t in tris:
            f.write(struct.pack("<3f", *normal(t)))
            for v in t: f.write(struct.pack("<3f", *v))
            f.write(struct.pack("<H", 0))


os.makedirs(a.aus_dir, exist_ok=True)
p_wand = os.path.join(a.aus_dir, "schale_wand.stl"); stl(p_wand, tri, "schale wand rund")
print("%s: %d Dreiecke, Zug %d Punkte, Profil %d Punkte, Ecken r %.0f mm" % (os.path.normpath(p_wand), len(tri), m, k, a.ecke))

# --- Boden: Ringsektor-Scheibe (Innenboden z 0 .. -boden_dicke), Ecken wie die Wand gerundet -----------------
bd = a.boden_dicke / 1e3
bz = [(p[0], p[1]) for p in zug]                                    # Wandmitte als Umriss reicht (liegt unter der Wand)
cz = (sum(p[0] for p in bz) / m, sum(p[1] for p in bz) / m)
tb = []
for i in range(m):
    p, q = bz[i], bz[(i + 1) % m]
    tb.append(((cz[0], cz[1], 0.0), (p[0], p[1], 0.0), (q[0], q[1], 0.0)))
    tb.append(((cz[0], cz[1], -bd), (q[0], q[1], -bd), (p[0], p[1], -bd)))
    tb.append(((p[0], p[1], -bd), (q[0], q[1], -bd), (q[0], q[1], 0.0))); tb.append(((p[0], p[1], -bd), (q[0], q[1], 0.0), (p[0], p[1], 0.0)))
p_boden = os.path.join(a.aus_dir, "schale_boden.stl"); stl(p_boden, tb, "schale boden")
print("%s: %d Dreiecke" % (os.path.normpath(p_boden), len(tb)))
