#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""Zielnest fuer eine Welle aus Koerper- und Streifenfarbe (farbplan.json, Benutzer 12.9.).

    python3 tools/nest_fuer_farbe.py schwarz rot          -> nest_11
    python3 tools/nest_fuer_farbe.py gelb                  -> WARNUNG, irgendein freies Nest der Zeile "gelb"
    python3 tools/nest_fuer_farbe.py gelb --belegt nest_21 nest_22
Belegung: --belegt <nester...> oder Datei ~/ros2_ws/platte_belegung.json {"nest_11": "...", ...}.
Ist die Belegung des gewaehlten Nests unbekannt (nicht in der Datei, kein --belegt), wird gefragt
(ohne Terminal: Exit 5). Ausgabe letzte Zeile: NEST <name> oder NEST - ; Exit 0 ok, 2 Farbe unbekannt,
4 kein freies Nest, 5 Belegung unbekannt / abgelehnt.
"""
import argparse, json, os, sys
ap = argparse.ArgumentParser()
ap.add_argument("koerper"); ap.add_argument("streifen", nargs="?", default=None)
ap.add_argument("--plan", default=os.path.expanduser("~/ros2_ws/farbplan.json"))
ap.add_argument("--belegung", default=os.path.expanduser("~/ros2_ws/platte_belegung.json"))
ap.add_argument("--belegt", nargs="*", default=None, help="belegte Nester (ersetzt die Datei)")
ap.add_argument("--auto", action="store_true", help="nicht fragen: unbekannte Belegung = Abbruch (Exit 5)")
a = ap.parse_args()
plan = json.load(open(a.plan)); nester = plan["nester"]
farben = set(plan["zeilen"].values())
norm = lambda f: {"siyah": "schwarz", "sari": "gelb", "beyaz": "weiss", "yesil": "gruen", "kirmizi": "rot",
                  "black": "schwarz", "yellow": "gelb", "white": "weiss", "green": "gruen", "red": "rot"}.get(f.lower(), f.lower()) if f else None
k, s = norm(a.koerper), norm(a.streifen)
if k not in farben: print(f"!! Koerperfarbe '{a.koerper}' unbekannt ({', '.join(sorted(farben))})"); print("NEST -"); sys.exit(2)
if a.belegt is not None: belegt = {n: "belegt" for n in a.belegt}; bekannt = True
elif os.path.exists(a.belegung):
    belegt = {k: v for k, v in json.load(open(a.belegung)).items() if not k.startswith("_")}; bekannt = True
else: belegt = {}; bekannt = False

def frei(n):
    if n in belegt: return belegt[n] in (None, "", "frei")      # dict-Eintrag (lege_welle) = belegt
    if bekannt: return True                       # Datei vorhanden, Nest nicht drin = frei
    if a.auto or not sys.stdin.isatty(): return None
    try: ant = input(f"  ? Ist {n} ({nester[n]['koerper']}/{nester[n]['streifen']}) FREI? j/n: ").strip().lower()
    except EOFError: return None
    return ant.startswith("j") or ant.startswith("y") or ant.startswith("e")

if s is not None:
    if s not in farben: print(f"!! Streifenfarbe '{a.streifen}' unbekannt"); print("NEST -"); sys.exit(2)
    treffer = [n for n, v in nester.items() if v["koerper"] == k and v["streifen"] == s]
    if not treffer: print(f"!! keine Welle {k}/{s} im Plan (gruen/gruen gibt es nicht)"); print("NEST -"); sys.exit(2)
    n = treffer[0]; f = frei(n)
    if f is None: print(f"!! Belegung von {n} unbekannt"); print("NEST -"); sys.exit(5)
    if not f: print(f"!! {n} ist BELEGT ({belegt.get(n)})"); print("NEST -"); sys.exit(4)
    print(f"Welle {k}/{s} -> {n}"); print(f"NEST {n}"); sys.exit(0)
print(f"!! WARNUNG: Streifenfarbe unbekannt - Welle kommt in ein freies Nest der Zeile '{k}' (spaeter umsortieren)")
for n, v in nester.items():
    if v["koerper"] != k: continue
    f = frei(n)
    if f is None: print(f"!! Belegung von {n} unbekannt"); print("NEST -"); sys.exit(5)
    if f: print(f"Welle {k}/? -> {n} (Streifen laut Plan: {v['streifen']})"); print(f"NEST {n}"); sys.exit(0)
print(f"!! kein freies Nest in Zeile '{k}'"); print("NEST -"); sys.exit(4)
