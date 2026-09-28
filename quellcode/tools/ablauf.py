#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""Ablauf: Welle(n) holen und in Nester der Ablageplatte stellen - die Kette aus den Einzelskripten,
mit ZWEI MODI, die an alle durchgereicht werden:

  MANUELL (Default)   vor jedem Zyklus, vor jedem Teilschritt und in den Skripten vor dem
                      Schliessen/Heben/Loslassen wird ENTER verlangt; 'a' + ENTER haelt an
                      (Arm bleibt stehen).
  VOLLAUTOMATIK       --auto: keine Rueckfragen. Nur mit funktionierendem Not-Aus und wenn der
                      Zyklus MANUELL an genau diesem Aufbau schon einmal durchgelaufen ist.

Zwei Startpunkte (--von):

  trichter (bisher)   Welle liegt schon im Trichter. Je Zyklus:
      hole_welle.py --nur-hin            -> Welle im Greifer, Arm 60 mm ueber dem Trichter
      lege_welle.py --nest <nest>        -> Welle steht, Arm +40 ueber dem Nest
      Nester kommen aus --nester (Reihenfolge) - die Farbe der Welle im Trichter sieht die
      Kamera nicht (Welle im Greifer verdeckt).

  schale (2026-09-13) Wellen liegen in der Bereitstellungsschale. Je Zyklus:
      [K] Kamerapose      nest_anfahren.py --nest nest_11 --nur-ueber   (Arm aus dem Bild;
                          in der Nullstellung schattet er den IR-Projektor ab)
      [S] Suchen          welle_finden_schale.py --nur-hell -> welle_ziel.json: Griffpunkt,
                          Achse, KOERPER- und STREIFENFARBE (tools/welle_farbe.py), weitere Wellen
      [N] Zielnest        nest_fuer_farbe.py <koerper> <streifen> (farbplan.json, Belegung);
                          --nester ueberstimmt das (Reihenfolge)
      [G] Griff           hole_aus_schale.py --ziel-datei --ohne-nullstellung --nur-hin
                          (Kamera->Modell Abbildung aehnlichkeit eingebaut)
      [T] Trichter        lege_welle.py --nest trichter_ablager --ueber 60 --ablage-dz 5
                          (Welle faellt in den Trichter; Punkt wurde 12.9. mit J6 -132 angelernt -
                          !! 13.9. Nacht Kollision des Greifergehaeuses, s. Memory; erst MANUELL)
      [H] Aus Trichter    hole_welle.py --punkt trichter_greif --nur-hin  (Griff am Kopf)
      [P] Platte          lege_welle.py --nest <nest> --farbe <koerper> <streifen>
      Keine Welle mehr in der Schale (welle_finden Exit 2) -> Kette endet mit Exit 2 = fertig.
      --zyklen N begrenzt (Default: bis die Schale leer ist bzw. --nester aufgebraucht).

Ergebnis je Skript aus dessen letzter Zeile `ERGEBNIS {json}`; alles wird nach
~/ros2_ws/ablauf_log.jsonl geschrieben. Bricht ein Schritt ab (Exit != 0), haelt die Kette an -
der Arm steht dann dort, wo das Skript ihn gelassen hat, NICHTS wird automatisch weggefahren.

    python3 tools/ablauf.py --von trichter --nester nest_11_ablage          # ein Zyklus, MANUELL
    python3 tools/ablauf.py --von trichter --nester nest_12 nest_13 --dx -6 --tastprobe
    python3 tools/ablauf.py --von schale                                    # Farbe -> Nest, bis Schale leer, MANUELL
    python3 tools/ablauf.py --von schale --zyklen 1 --auto                  # VOLLAUTOMATIK, eine Welle
    python3 tools/ablauf.py --von schale --bis trichter                     # nur Schale -> Trichter (Test)
    python3 tools/ablauf.py --von schale --probe                            # nur Befehle zeigen
Exit: 0 alle Zyklen ok, 2 keine Welle (Trichter/Schale leer), 3 Abbruch durch Benutzer,
      4 Tastprobe: Welle steht nicht / kein Nest fuer die Farbe, 1 Fehler.
"""
import argparse, datetime, json, os, subprocess, sys

HIER = os.path.dirname(os.path.abspath(__file__))
LOG = os.path.expanduser("~/ros2_ws/ablauf_log.jsonl")
ZIEL = os.path.expanduser("~/ros2_ws/welle_ziel.json")

ap = argparse.ArgumentParser()
ap.add_argument("--von", choices=["trichter", "schale"], default="trichter", help="Startpunkt der Kette")
ap.add_argument("--bis", choices=["trichter", "platte"], default="platte",
                help="schale: Kette nach der Trichter-Ablage beenden (Test des ersten Teils)")
ap.add_argument("--nester", nargs="*", default=None, help="Zielnester in Reihenfolge (trichter: Pflicht; schale: ueberstimmt die Farbwahl)")
ap.add_argument("--zyklen", type=int, default=0, help="schale: hoechstens so viele Zyklen (0 = bis die Schale leer ist)")
ap.add_argument("--punkt", default="trichter_greif", help="Griffpunkt im Trichter")
ap.add_argument("--ablagepunkt", default="trichter_ablager", help="Ablagepunkt am Trichter (schale)")
ap.add_argument("--ablage-ueber", type=float, default=60.0, help="[mm] Anfahr-/Hebehoehe ueber dem Trichter-Ablagepunkt")
ap.add_argument("--ablage-dz", type=float, default=0.0, help="[mm] Loslasshoehe ueber dem Trichter-Ablagepunkt (14.9.: Punkt neu = Loslasshoehe -> 0)")
ap.add_argument("--kamerapose", default="kamerapose", help="'kamerapose' = Nullstellung + J5 -90 (kamerapose.py, 14.9.); Nestname = +40 mm darueber; '' = nicht fahren")
ap.add_argument("--auto", action="store_true", help="VOLLAUTOMATIK - alle Skripte ohne Rueckfragen")
ap.add_argument("--dx", type=float, default=None, help="[mm] Ablageversatz x fuer lege_welle auf der Platte (nest_11 brauchte -6; *_ablage-Punkte haben ihn schon)")
ap.add_argument("--dy", type=float, default=None)
ap.add_argument("--tastprobe", action="store_true", help="lege_welle: nach dem Loslassen pruefen, ob die Welle steht")
ap.add_argument("--ohne-nullstellung", action="store_true", help="hole_welle ab dem 2. Zyklus (trichter) bzw. immer (schale) ohne Nullstellung starten")
ap.add_argument("--null", action="store_true", help="nach dem letzten Zyklus zur Nullstellung")
ap.add_argument("--tempo", type=float, default=None); ap.add_argument("--setzzeit", type=float, default=None)
ap.add_argument("--nur-hell", dest="nur_hell", action="store_true", default=True, help="welle_finden_schale --nur-hell (Default)")
ap.add_argument("--auch-dunkel", dest="nur_hell", action="store_false", help="welle_finden_schale OHNE --nur-hell (schwarze Wellen; Schatten stoert)")
ap.add_argument("--finden-args", default="", help="weitere Argumente fuer welle_finden_schale.py, in Anfuehrungszeichen")
ap.add_argument("--schale-args", default="", help="weitere Argumente fuer hole_aus_schale.py")
ap.add_argument("--trichter-args", default="", help="weitere Argumente fuer lege_welle.py an der Trichter-Ablage")
ap.add_argument("--hole-args", default="", help="weitere Argumente fuer hole_welle.py")
ap.add_argument("--lege-args", default="", help="weitere Argumente fuer lege_welle.py auf der Platte")
ap.add_argument("--probe", action="store_true", help="nur die Befehle zeigen, nichts fahren")
a = ap.parse_args()
MODUS = "VOLLAUTOMATIK" if a.auto else "MANUELL"
if a.von == "trichter" and not a.nester: ap.error("--von trichter braucht --nester")


def frage(was):
    """Nur MANUELL. ENTER = weiter, a = Kette beenden (Arm bleibt stehen)."""
    if a.auto: return
    try: antwort = input(f"  ? {was} — ENTER = weiter, a = beenden: ").strip().lower()
    except EOFError: antwort = ""
    if antwort.startswith("a"):
        print("  Kette beendet durch Benutzer. Arm bleibt stehen."); sys.exit(3)


def schritt(skript, args, zyklus, kennung=""):
    """Skript als Unterprozess (stdin durchgereicht: die ENTER-Fragen laufen im selben Terminal),
    Ausgabe mitlesen, ERGEBNIS-/NEST-Zeile und Exit-Code zurueckgeben."""
    cmd = [sys.executable, "-u", os.path.join(HIER, skript)] + args
    print(f"\n===== Zyklus {zyklus} [{MODUS}]{(' ' + kennung) if kennung else ''}: {skript} {' '.join(args)}")
    if a.probe: return 0, None
    ergebnis = None
    p = subprocess.Popen(cmd, stdout=subprocess.PIPE, stderr=subprocess.STDOUT, text=True, bufsize=1)
    for zeile in p.stdout:
        print("  | " + zeile.rstrip())
        z = zeile.strip()
        if z.startswith("ERGEBNIS "):
            try: ergebnis = json.loads(z[len("ERGEBNIS "):])
            except json.JSONDecodeError: pass
        elif z.startswith("NEST "): ergebnis = {"nest": z[5:].strip()}
    code = p.wait()
    with open(LOG, "a") as f:
        f.write(json.dumps({"zeit": datetime.datetime.now().isoformat(timespec="seconds"), "modus": MODUS, "von": a.von,
                            "zyklus": zyklus, "schritt": kennung, "skript": skript, "args": args, "exit": code, "ergebnis": ergebnis}) + "\n")
    print(f"===== {skript}: Exit {code}" + (f"  {json.dumps(ergebnis)}" if ergebnis else ""))
    return code, ergebnis


def halt(code, text):
    print("  " + text + (" Kette angehalten - Arm steht, NICHTS seitlich fahren." if code == 1 else " Kette angehalten."))
    sys.exit(code)


gemeinsam = (["--auto"] if a.auto else []) + \
            (["--tempo", str(a.tempo)] if a.tempo is not None else []) + \
            (["--setzzeit", str(a.setzzeit)] if a.setzzeit is not None else [])


def platte_legen(nest, zyklus, farbe, letzter):
    lege = ["--nest", nest] + gemeinsam + a.lege_args.split()
    if farbe: lege += ["--farbe"] + [f for f in farbe if f]
    if a.dx is not None: lege += ["--dx", str(a.dx)]
    if a.dy is not None: lege += ["--dy", str(a.dy)]
    if a.tastprobe: lege.append("--tastprobe")
    if a.null and letzter: lege.append("--null")
    code, erg = schritt("lege_welle.py", lege, zyklus, "[P] Platte")
    if code != 0:
        halt(code if code in (2, 3, 4) else 1,
             {2: "lege_welle: Greifer war leer - Welle unterwegs verloren.", 3: "Abbruch durch Benutzer in lege_welle.",
              4: f"Tastprobe: Welle steht NICHT in {nest}."}.get(code, "lege_welle fehlgeschlagen."))


def aus_trichter(zyklus, ohne_null):
    hole = ["--punkt", a.punkt, "--nur-hin"] + gemeinsam + a.hole_args.split()
    if ohne_null: hole.append("--ohne-nullstellung")
    code, erg = schritt("hole_welle.py", hole, zyklus, "[H] aus dem Trichter")
    if code != 0:
        halt(code if code in (2, 3) else 1,
             {2: "Keine Welle bekommen - Trichter leer?", 3: "Abbruch durch Benutzer in hole_welle."}.get(code, "hole_welle fehlgeschlagen."))


# ================================================================== trichter -> Platte (bisher)
if a.von == "trichter":
    print(f"Ablauf Trichter -> Platte, Modus {MODUS}, {len(a.nester)} Zyklus/Zyklen: {' '.join(a.nester)}")
    if a.auto and not a.probe: print("  VOLLAUTOMATIK: keine Rueckfragen mehr. Not-Aus bereit halten.")
    gelegt = []
    for i, nest in enumerate(a.nester, 1):
        frage(f"Zyklus {i}/{len(a.nester)}: Welle aus '{a.punkt}' nach '{nest}'. Trichter befuellt, Nest frei?")
        aus_trichter(i, a.ohne_nullstellung and i > 1)
        platte_legen(nest, i, None, i == len(a.nester))
        gelegt.append(nest); print(f"  Zyklus {i} fertig: Welle steht in {nest}.")
    print(f"\nFertig: {len(gelegt)} Welle(n) gelegt: {' '.join(gelegt)}" + ("" if a.probe else f"   (Log: {LOG})"))
    sys.exit(0)

# ================================================================== schale -> Trichter -> Platte (2026-09-13)
maximal = a.zyklen if a.zyklen > 0 else (len(a.nester) if a.nester else 999)
print(f"Ablauf Schale -> Trichter -> Platte, Modus {MODUS}, "
      + (f"hoechstens {maximal} Zyklus/Zyklen" if maximal < 999 else "bis die Schale leer ist")
      + (f", Nester vorgegeben: {' '.join(a.nester)}" if a.nester else ", Nest nach Farbe (farbplan.json)")
      + (f", Ende nach der Trichter-Ablage (--bis trichter)" if a.bis == "trichter" else ""))
if a.auto and not a.probe: print("  VOLLAUTOMATIK: keine Rueckfragen mehr. Not-Aus bereit halten.")
gelegt = []; i = 0
while i < maximal:
    i += 1
    frage(f"Zyklus {i}: Kamerapose, Welle in der Schale suchen, greifen, Trichter, Platte. Schale bestueckt, Trichter leer?")

    # [K] Kamerapose - der Arm darf nicht ueber der Schale / vor dem Projektor stehen
    if a.kamerapose == "kamerapose":       # 2026-09-14: Nullstellung + J5 -90 (Greifer nach -y), tools/kamerapose.py
        code, erg = schritt("kamerapose.py", ["--auto"] + ([f"--tempo", str(a.tempo)] if a.tempo is not None else []), i, "[K] Kamerapose")
        if code != 0: halt(1, "Kamerapose (J5 -90) nicht erreicht.")
    elif a.kamerapose:
        code, erg = schritt("nest_anfahren.py", ["--nest", a.kamerapose, "--nur-ueber", "--greifer-lassen", "--auto"]
                            + ([f"--tempo", str(a.tempo)] if a.tempo is not None else []), i, "[K] Kamerapose")
        if code != 0: halt(1, f"Kamerapose ueber {a.kamerapose} nicht erreicht.")

    # [S] Suchen + Farbe
    finden = ["--ziel", ZIEL] + (["--nur-hell"] if a.nur_hell else []) + a.finden_args.split()
    code, erg = schritt("welle_finden_schale.py", finden, i, "[S] Suchen")
    if code == 4:
        halt(4, "Wellen in der Schale, aber keine greifbar (Sperrzone/Nachbar/Form) - Welle(n) umlegen und neu starten.")
    if code == 2:
        print(f"\n  Keine Welle mehr in der Schale - {len(gelegt)} Welle(n) gelegt.")
        if a.null and not a.probe: schritt("nullstellung_moveit.py", [], i, "[0] Nullstellung")
        sys.exit(2 if not gelegt else 0)
    if code != 0: halt(1, "welle_finden_schale fehlgeschlagen.")
    ziel = {} if a.probe else json.load(open(ZIEL))
    koerper, streifen = ziel.get("koerper"), ziel.get("streifen")
    if streifen == "unbekannt": streifen = None
    if not a.probe:
        print(f"  Welle: Griff ({ziel['griff_mm'][0]:+.1f}, {ziel['griff_mm'][1]:+.1f}) mm, Achse {ziel['yaw_grad']:.0f} Grad, "
              f"Farbe {koerper}/{streifen or '?'}" + ("" if ziel.get("farbe_sicher") else " (UNSICHER)")
              + (f", {ziel.get('anzahl')} Welle(n) im Bild" if ziel.get("anzahl") else "")
              + f"; KONFIDENZ {ziel.get('konfidenz', 0):.0f}/100 (Wand quer {ziel.get('wand_quer_mm')} mm, Nachbar {ziel.get('nachbar_mm')} mm, Oeffnung {ziel.get('oeffnung_mm')} mm)")

    # [N] Zielnest
    if a.nester:
        nest = a.nester[i - 1]; farbe = [koerper, streifen] if koerper and koerper != "?" else None
        print(f"  Zielnest vorgegeben: {nest}")
    elif a.probe:
        nest, farbe = "<nest nach Farbe>", None
    else:
        if not koerper or koerper == "?": halt(4, "Koerperfarbe nicht erkannt - kein Zielnest (mit --nester vorgeben).")
        nf = [koerper] + ([streifen] if streifen else []) + (["--auto"] if a.auto else [])
        code, erg = schritt("nest_fuer_farbe.py", nf, i, "[N] Zielnest")
        if code != 0 or not erg or erg.get("nest") in (None, "-"):
            halt(4, {2: "Farbe nicht im Plan.", 4: "kein freies Nest fuer diese Farbe.", 5: "Belegung unbekannt."}.get(code, "nest_fuer_farbe fehlgeschlagen."))
        nest = erg["nest"]; farbe = [koerper, streifen]
        if not streifen: print(f"  !! Streifenfarbe unbekannt -> {nest} (Zeile {koerper}); spaeter pruefen/umsortieren")
    frage(f"Welle {koerper}/{streifen or '?'} -> {nest}. Greifen (hole_aus_schale, Kamera->Modell aehnlichkeit)?")

    # [G] Griff aus der Schale
    hs = ["--ziel-datei", ZIEL, "--ohne-nullstellung", "--nur-hin"] + gemeinsam + a.schale_args.split()
    code, erg = schritt("hole_aus_schale.py", hs, i, "[G] Griff Schale")
    if code != 0:
        halt(code if code in (2, 3, 4) else 1,
             {2: "Keine Welle bekommen (Greifer leer).", 3: "Abbruch durch Benutzer in hole_aus_schale.",
              4: "Ziel abgelehnt (Proben) - Welle umlegen."}.get(code, "hole_aus_schale fehlgeschlagen."))

    # [T] Trichter-Ablage
    frage(f"Welle im Greifer. Zum Trichter ({a.ablagepunkt}, ueber +{a.ablage_ueber:.0f}, loslassen bei +{a.ablage_dz:.0f})? Greifergehaeuse beobachten!")
    ta = ["--nest", a.ablagepunkt, "--ueber", str(a.ablage_ueber), "--ablage-dz", str(a.ablage_dz), "--belegung", ""] \
         + gemeinsam + a.trichter_args.split()
    code, erg = schritt("lege_welle.py", ta, i, "[T] Trichter")
    if code != 0:
        halt(code if code in (2, 3) else 1,
             {2: "lege_welle (Trichter): Greifer war leer - Welle verloren.", 3: "Abbruch durch Benutzer an der Trichter-Ablage."}.get(code, "Trichter-Ablage fehlgeschlagen."))
    if a.bis == "trichter":
        print(f"\n  --bis trichter: Welle {koerper}/{streifen or '?'} liegt im Trichter, Arm +{a.ablage_ueber:.0f} darueber. Ende.")
        gelegt.append(f"Trichter({koerper}/{streifen or '?'})"); break

    # [H] aus dem Trichter, [P] Platte
    frage(f"Welle im Trichter. Griff aus dem Trichter ({a.punkt}) und nach {nest}?")
    aus_trichter(i, a.ohne_nullstellung)
    platte_legen(nest, i, farbe, i == maximal)
    gelegt.append(f"{nest}({koerper}/{streifen or '?'})"); print(f"  Zyklus {i} fertig: Welle {koerper}/{streifen or '?'} steht in {nest}.")

print(f"\nFertig: {len(gelegt)} Welle(n): {' '.join(gelegt)}" + ("" if a.probe else f"   (Log: {LOG})"))
