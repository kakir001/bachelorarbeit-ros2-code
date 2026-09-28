#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""EIN Gelenk in allen Teach-Punkten (teach_punkte.json) und Kalibrierposen (kalibrierposen_charuco.json)
um --grad verschieben. Noetig, wenn der Servo-Nullpunkt dieser Achse neu gesetzt wurde
(tools/servo_nullen.py): die physische Stellung bleibt, der Encoder liest um den alten Wert weniger.
Beispiel: Achse 5 stand real senkrecht bei Encoder +3.0 -> servo_nullen 5 -> hier --gelenk 5 --grad -3.0
Sicherung: <datei>_vor_<gelenk>_<datum>.json. --nur-anzeigen: nichts schreiben.
"""
import argparse, json, math, shutil, time
ap = argparse.ArgumentParser()
ap.add_argument("--gelenk", type=int, required=True, choices=range(1, 7))
ap.add_argument("--grad", type=float, required=True, help="Verschiebung [Grad], wird ADDIERT")
ap.add_argument("--teach", default="teach_punkte.json"); ap.add_argument("--charuco", default="kalibrierposen_charuco.json")
ap.add_argument("--nur-anzeigen", action="store_true")
a = ap.parse_args(); i = a.gelenk - 1; stamp = time.strftime("%Y-%m-%d")
def r2(x): return round(x, 2)

def punkte(d, pfad, n):
    """rekursiv: jedes dict mit 'grad' (6 Werte) verschieben; Archiv-Unterdicts auch."""
    for k, v in d.items():
        if isinstance(v, dict):
            if isinstance(v.get("grad"), list) and len(v["grad"]) == 6:
                alt = v["grad"][i]; v["grad"][i] = r2(alt + a.grad)
                if isinstance(v.get("rad"), list) and len(v["rad"]) == 6: v["rad"][i] = math.radians(v["grad"][i])
                n.append(f"{pfad}{k}: J{a.gelenk} {alt:+.2f} -> {v['grad'][i]:+.2f}")
            else:
                punkte(v, pfad + k + "/", n)

t = json.load(open(a.teach)); n = []; punkte(t, "", n)
t["_hinweis_versatz"] = t.get("_hinweis_versatz", "") + f" | {stamp}: J{a.gelenk} um {a.grad:+.2f} Grad verschoben (Servo-Nullpunkt neu, teach_gelenk_verschieben.py)"
c = json.load(open(a.charuco)); m = []
for key in ("stellungen", "verworfen"):
    for e in c.get(key, []):
        if isinstance(e.get("q_grad"), list) and len(e["q_grad"]) == 6:
            alt = e["q_grad"][i]; e["q_grad"][i] = r2(alt + a.grad); m.append(f"{key}: {alt:+.2f} -> {e['q_grad'][i]:+.2f}")
c["_hinweis_versatz"] = c.get("_hinweis_versatz", "") + f" | {stamp}: J{a.gelenk} um {a.grad:+.2f} Grad verschoben"
print("\n".join(n)); print(f"Teach-Punkte: {len(n)}  Kalibrierposen: {len(m)}")
if a.nur_anzeigen: print("(nur anzeigen - nichts geschrieben)"); raise SystemExit
for src in (a.teach, a.charuco):
    shutil.copy(src, src.replace(".json", f"_vor_J{a.gelenk}_{stamp}.json"))
json.dump(t, open(a.teach, "w"), indent=1, ensure_ascii=False); json.dump(c, open(a.charuco, "w"), indent=1, ensure_ascii=False)
print("geschrieben + Sicherungen *_vor_J%d_%s.json" % (a.gelenk, stamp))
