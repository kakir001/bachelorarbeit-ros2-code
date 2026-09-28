#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""PROGRAMMABLAUF mit Fenster (Benutzerwunsch 2026-09-14): Schale -> Griff -> Kamerakontrolle -> Trichter
-> Kamerakontrolle -> Griff am Kopf -> Ablageplatte nach Farbe, bis die Schale leer ist. 19 Wellen
(5 Farben x 4 Streifen, gruen/gruen gibt es nicht).

Fenster: zu Beginn zwei Knoepfe AUTOMATIK / MANUELL (mit Bestaetigung vor jedem Schritt), dauernd ein
grosser NOT-AUS (kill des laufenden Skripts + /estop = true; FREIGABE hebt ihn auf), die Schrittliste mit
dem aktuellen Schritt, Farben/Nest der gewaehlten Welle, der Bildausschnitt (Kopf rot, Mitte gruen,
Griff gelb, Achse magenta) bzw. die letzte Kamerakontrolle, und das Protokoll.

Schritte je Zyklus:
  0 Kamerapose (Nullstellung + J5 -90, Greifer nach -y)   kamerapose.py --auto  (oder --kamerapose nest_11: +40 ueber dem Nest)
  1 Welle bestimmen (Modell + Farben)                     welle_finden_schale.py -> welle_ziel.json
  2 Zielnest aus Farbe                                    nest_fuer_farbe.py
  3 Welle greifen (Abbildung Kamera->Modell)              hole_aus_schale.py --nur-hin
      nicht bekommen -> zurueck zu 0 (max. 3x je Zyklus)
  4 Unter der Kamera pruefen: Welle am Kopf im Greifer?   kontrollpose.py (+ J6 90) + kamera_pruefung.py --kopf
      nein/unklar -> ANHALTEN (Benutzer)
  5 Am Trichter ablegen (angelernter Punkt)               lege_welle.py --nest trichter_ablager  (finger x trichter
                                                          nur waehrend dieses Schritts in der ACM frei, danach zurueck)
  6 Kamerapose + pruefen: steht die Welle im Trichter?    kamera_pruefung.py --trichter
      nein -> zurueck zu 0 (Welle gilt als verloren, Benutzer sammelt sie ein)
  7 Welle am Kopf aus dem Trichter greifen               hole_welle.py --punkt trichter_greif --nur-hin
  8 In das Nest der Ablageplatte stellen                  lege_welle.py --nest <nest> --farbe k s
Ende: Schale leer (welle_finden Exit 2) und Trichter leer, oder 19 Wellen gelegt.

Alle Skripte laufen als Unterprozesse mit --auto (die GUI ist die einzige Rueckfrage-Stelle); ROS laeuft
nur in den Unterprozessen (Galactic-pybind/Thread-Falle). Log: ~/ros2_ws/zyklus_log.jsonl.

    python3 tools/zyklus_gui.py
    python3 tools/zyklus_gui.py --ablage-dz 5 --ohne-kontrollpose   # Kamerakontrolle ueberspringen
"""
import argparse, datetime, json, os, queue, re, signal, subprocess, sys, threading, time
import tkinter as tk
from tkinter import ttk
from PIL import Image, ImageTk

HIER = os.path.dirname(os.path.abspath(__file__)); WS = os.path.expanduser("~/ros2_ws")
ZIEL = os.path.join(WS, "welle_ziel.json"); LOG = os.path.join(WS, "zyklus_log.jsonl")
PROTOKOLL = os.path.join(WS, "zyklus_protokoll.txt")   # Textprotokoll des Fensters (14.9.)
TMP = "/tmp/zyklus_gui"; os.makedirs(TMP, exist_ok=True)
ap = argparse.ArgumentParser()
ap.add_argument("--ab-schritt", type=int, default=0, help="Wiedereinstieg: 5 = Welle im Greifer (Kontrollpose) -> Trichter-Ablage; 7 = Welle steckt im Trichter -> holen + Nest; 8 = Welle im Greifer -> nur Nest; Farbe wird abgefragt")
ap.add_argument("--j6-kontrolle", type=float, default=-90.0, help="[Grad] J6-Drehung in der Kontrollpose (14.9.: -90 nach dem Umsetzen des Greifers; vorher +90)")
ap.add_argument("--kamerapose", default="kamerapose", help="'kamerapose' = Nullstellung + J5 -90 (kamerapose.py); sonst Nestname (+40 mm, nest_anfahren)"); ap.add_argument("--ablagepunkt", default="trichter_ablager")
ap.add_argument("--ablage-ueber", type=float, default=60.0); ap.add_argument("--ablage-dz", type=float, default=0.0)   # 14.9. 14:3x: Punkt = Loslasshoehe
ap.add_argument("--griffpunkt", default="trichter_greif"); ap.add_argument("--max-griffe", type=int, default=3)
ap.add_argument("--ohne-kontrollpose", action="store_true"); ap.add_argument("--tempo", type=float, default=None)
ap.add_argument("--nester", nargs="*", default=None, help="Nester in Reihenfolge statt Farbwahl")
ap.add_argument("--nur-schale-trichter", action="store_true", help="nach Schritt 6 aufhoeren (Test)")
A = ap.parse_args()

SCHRITTE = ["0 Kamerapose (Greifer aus dem Bild)", "1 Welle bestimmen aus der Schale (Modell + Farbe)", "2 Zielnest aus Koerper-/Streifenfarbe",
            "3 Welle greifen", "4 Unter der Kamera: Welle am Kopf im Greifer?", "5 Am Trichter ablegen",
            "6 Kamera: steht die Welle im Trichter?", "7 Welle am Kopf aus dem Trichter greifen", "8 In das Nest der Ablageplatte stellen"]


class Abbruch(Exception): pass


class Zyklus:
    def __init__(s, gui):
        s.gui = gui; s.modus = None; s.proc = None; s.stop = threading.Event(); s.weiter = threading.Event(); s.antwort = None
        s.gelegt = []; s.verloren = 0; s.farbe = (None, None); s.farbe_sicher = False; s.nest = None

    # ---------------------------------------------------------------- Hilfen
    def log(s, txt):
        s.gui.q.put(("log", txt))
        try:
            with open(PROTOKOLL, "a") as f: f.write(time.strftime("%H:%M:%S ") + txt + "\n")
        except Exception: pass
    def schritt_anzeigen(s, i): s.gui.q.put(("schritt", i))
    def bild(s, pfad, titel=""): s.gui.q.put(("bild", pfad, titel))

    def frage(s, text, optionen=("Weiter", "Abbrechen")):
        """MANUELL: im Fenster fragen; AUTOMATIK: sofort erste Option."""
        if s.modus == "AUTOMATIK": return optionen[0]
        s.antwort = None; s.weiter.clear(); s.gui.q.put(("frage", text, optionen))
        while not s.weiter.is_set():
            if s.stop.is_set(): raise Abbruch("NOT-AUS")
            time.sleep(0.1)
        if s.antwort == "Abbrechen": raise Abbruch("Benutzer")
        return s.antwort

    def farbe_waehlen(s, k, st, grund):
        """Koerper-/Streifenfarbe vom Benutzer bestaetigen oder eingeben (14.9.: kein Abbruch mehr, wenn die
        Kamera die Streifen nicht erkennt - der Benutzer sagt sie). Auch in AUTOMATIK, weil sonst kein Zielnest."""
        s.antwort = None; s.weiter.clear(); s.gui.q.put(("farbwahl", k or "?", st or "?", grund))
        while not s.weiter.is_set():
            if s.stop.is_set(): raise Abbruch("NOT-AUS")
            time.sleep(0.1)
        if s.antwort == "Abbrechen": raise Abbruch("Benutzer")
        k2, st2 = s.antwort
        s.log("Farbe vom Benutzer: %s/%s (Kamera: %s/%s)" % (k2, st2, k or "?", st or "?"))
        s.farbe = (k2, st2); s.farbe_sicher = True; s.gui.q.put(("farbe", k2, st2, True, None)); return k2, st2

    def halt(s, text, optionen=("Weiter", "Abbrechen")):
        """Immer fragen (auch AUTOMATIK) - Sicherheitsstopp."""
        s.antwort = None; s.weiter.clear(); s.gui.q.put(("frage", text, optionen))
        while not s.weiter.is_set():
            if s.stop.is_set(): raise Abbruch("NOT-AUS")
            time.sleep(0.1)
        if s.antwort == "Abbrechen": raise Abbruch("Benutzer")
        return s.antwort

    def run(s, skript, args, kennung):
        """Unterprozess (Python-Skript aus tools/), Ausgabe ins Protokoll, (exit, ERGEBNIS)."""
        if s.stop.is_set(): raise Abbruch("NOT-AUS")
        cmd = [sys.executable, "-u", os.path.join(HIER, skript)] + [str(x) for x in args]
        s.log("$ %s %s" % (skript, " ".join(str(x) for x in args)))
        env = dict(os.environ); env.setdefault("LC_ALL", "C"); env.setdefault("CYCLONEDDS_URI", "file://" + os.path.join(WS, "cyclonedds_loopback.xml"))
        s.proc = subprocess.Popen(cmd, stdout=subprocess.PIPE, stderr=subprocess.STDOUT, text=True, bufsize=1, stdin=subprocess.DEVNULL, env=env, preexec_fn=os.setsid)
        erg = None; s.ausgabe = []
        for zeile in s.proc.stdout:
            z = zeile.rstrip(); s.ausgabe.append(z)
            if "multicast" in z or "Exception ignored" in z or "InvalidHandle" in z or z.startswith("Traceback") or z.startswith("  File "): continue
            if z.strip().startswith("ERGEBNIS "):
                try: erg = json.loads(z.strip()[9:])
                except json.JSONDecodeError: pass
            elif z.strip().startswith("NEST "): erg = {"nest": z.strip()[5:].strip()}
            if z.strip(): s.log("  | " + z)
        code = s.proc.wait(); s.proc = None
        with open(LOG, "a") as f:
            f.write(json.dumps({"zeit": datetime.datetime.now().isoformat(timespec="seconds"), "modus": s.modus, "schritt": kennung, "skript": skript, "args": [str(x) for x in args], "exit": code, "ergebnis": erg}) + "\n")
        s.log("  -> Exit %d" % code)
        if s.stop.is_set(): raise Abbruch("NOT-AUS")
        return code, erg

    def not_aus(s):
        s.stop.set()
        if s.proc is not None:
            try: os.killpg(os.getpgid(s.proc.pid), signal.SIGINT)
            except Exception: pass
        subprocess.Popen(["bash", "-c", "source /opt/ros/galactic/setup.bash && source %s/install/setup.bash && "
                          "ros2 topic pub -1 --qos-durability transient_local --qos-reliability reliable /estop std_msgs/msg/Bool '{data: true}'" % WS],
                         stdout=subprocess.DEVNULL, stderr=subprocess.DEVNULL, env=dict(os.environ, LC_ALL="C", CYCLONEDDS_URI="file://" + os.path.join(WS, "cyclonedds_loopback.xml")))
        s.log("!!! NOT-AUS: laufendes Skript beendet, /estop = true")

    def freigabe(s):
        subprocess.Popen(["bash", "-c", "source /opt/ros/galactic/setup.bash && source %s/install/setup.bash && "
                          "ros2 topic pub -1 --qos-durability transient_local --qos-reliability reliable /estop std_msgs/msg/Bool '{data: false}'" % WS],
                         stdout=subprocess.DEVNULL, stderr=subprocess.DEVNULL, env=dict(os.environ, LC_ALL="C", CYCLONEDDS_URI="file://" + os.path.join(WS, "cyclonedds_loopback.xml")))
        s.log("/estop = false (Freigabe). Ablauf muss neu gestartet werden.")

    # ---------------------------------------------------------------- Schritte
    def tempo(s): return ["--tempo", str(A.tempo)] if A.tempo is not None else []

    def kamerapose(s):
        # 2026-09-14: 'kamerapose' = Nullstellung + J5 -90 (Greifer waagerecht nach -y, nicht unter der Kamera;
        # tools/kamerapose.py, MoveIt). Jeder andere Name = wie bisher ueber ein Nest (+40 mm).
        if A.kamerapose == "kamerapose":
            s.schritt_anzeigen(0); s.frage("Schritt 0: Kamerapose (Nullstellung, J5 -90, Greifer nach -y) anfahren?")
            code, _ = s.run("kamerapose.py", ["--auto"] + s.tempo(), "0 Kamerapose")
        else:
            s.schritt_anzeigen(0); s.frage("Schritt 0: Kamerapose (%s +40) anfahren?" % A.kamerapose)
            code, _ = s.run("nest_anfahren.py", ["--nest", A.kamerapose, "--nur-ueber", "--greifer-lassen", "--auto"] + s.tempo(), "0 Kamerapose")
        if code != 0: s.halt("Kamerapose nicht erreicht (Exit %d). Arm steht. Trotzdem weiter?" % code)

    def welle_bestimmen(s):
        s.schritt_anzeigen(1); s.frage("Schritt 1: Welle in der Schale bestimmen (Kamera)?")
        crop = os.path.join(TMP, "welle_%d.png" % int(time.time())); ueb = crop.replace(".png", "_schale.png")
        # Uebersicht: ganze Schale, NUR die gewaehlte Welle (Kopf + Mittellinie) und die 20-mm-Sperrzone an der Wand rot -
        # der Betrachter sieht, warum Wellen am Rand nicht genommen werden (Benutzer 14.9.)
        code, _ = s.run("welle_finden_schale.py", ["--ziel", ZIEL, "--ausschnitt", crop, "--uebersicht", ueb], "1 Welle bestimmen")
        if code == 2: return None
        if code == 4:
            s.halt("Wellen in der Schale, aber KEINE greifbar (Sperrzone / Nachbar / Form - s. Protokoll). "
                   "Welle(n) von Hand umlegen, dann Weiter = neu suchen."); return s.welle_bestimmen()
        if code != 0: s.halt("welle_finden_schale fehlgeschlagen (Exit %d). Weiter = nochmal." % code); return s.welle_bestimmen()
        z = json.load(open(ZIEL)); k, st = z.get("koerper"), z.get("streifen")
        if st == "unbekannt": st = None
        s.farbe = (k, st); s.farbe_sicher = bool(z.get("farbe_sicher"))
        s.gui.q.put(("farbe", k, st, z.get("farbe_sicher"), z.get("anzahl")))
        s.bild(ueb if os.path.exists(ueb) else crop, "Gewaehlte Welle: %s / %s  Konfidenz %.0f  (rot = 20 mm Sperrzone)" % (k, st or "?", z.get("konfidenz", 0)))
        s.log("Welle: Griff (%.1f, %.1f) mm, Achse %.0f Grad, Farbe %s/%s%s, %s Welle(n) im Bild"
              % (z["griff_mm"][0], z["griff_mm"][1], z["yaw_grad"], k, st or "?", "" if z.get("farbe_sicher") else " (UNSICHER)", z.get("anzahl")))
        s.log("  KONFIDENZ %.0f/100 (%s); Wand quer %s mm, Nachbar %s mm, Oeffnung %s mm"
              % (z.get("konfidenz", 0), " ".join("%s %.0f" % kv for kv in (z.get("konfidenz_teile") or {}).items()),
                 z.get("wand_quer_mm"), z.get("nachbar_mm"), z.get("oeffnung_mm")))
        return z

    def nest_bestimmen(s):
        s.schritt_anzeigen(2)
        if A.nester:
            s.nest = A.nester[len(s.gelegt)] if len(s.gelegt) < len(A.nester) else None
            if s.nest is None: s.halt("Nesterliste aufgebraucht."); raise Abbruch("Nester")
            s.gui.q.put(("nest", s.nest)); return s.nest
        k, st = s.farbe
        # MANUELL: IMMER fragen "Farbe richtig?" (Benutzer 14.9.: Kamera sagte schwarz/rot, Welle war schwarz/gruen) -
        # Auswahl vorbelegt, ein Klick = weiter; AUTOMATIK: nur wenn die Kamera unsicher ist oder etwas fehlt.
        if s.modus == "MANUELL" or not k or k == "?" or not st or st in ("?", "unbekannt") or not s.farbe_sicher:
            k, st = s.farbe_waehlen(k, st, "Schritt 2: Farbe laut Kamera: Koerper %s, Streifen %s%s - richtig? Sonst hier aendern."
                                    % (k or "?", st or "?", "" if s.farbe_sicher else " (UNSICHER)"))
        code, erg = s.run("nest_fuer_farbe.py", [k, st, "--auto"], "2 Zielnest")
        while code != 0 or not erg or erg.get("nest") in (None, "-"):
            grund = {2: "Farbe nicht im Plan", 4: "kein freies Nest fuer diese Farbe", 5: "Belegung unbekannt"}.get(code, "Exit %d" % code)
            if code == 4:
                # 23.9. (Benutzer): die Belegungsdatei sagte "nest_41 BELEGT", das Nest war aber laengst leer geraeumt -
                # das System weiss nicht, was auf der Platte steht. Also fragen statt abbrechen.
                m = re.search(r"!! (nest_\d+) ist BELEGT", "\n".join(getattr(s, "ausgabe", [])))
                if m:
                    n_bel = m.group(1)
                    ant = s.halt("Laut Belegung ist %s BESETZT (%s/%s). Ist das Nest in Wirklichkeit FREI?" % (n_bel, k, st),
                                 ("Ja - Nest ist frei, dort stellen", "Farbe/Nest aendern", "Abbrechen"))
                    if ant.startswith("Ja"):
                        s.belegung_freigeben(n_bel)
                        code, erg = s.run("nest_fuer_farbe.py", [k, st, "--auto"], "2 Zielnest"); continue
                else:
                    ant = s.halt("Kein freies Nest mehr fuer %s/%s (Zeile voll laut Belegung)." % (k, st),
                                 ("Farbe/Nest aendern", "Abbrechen"))
                k, st = s.farbe_waehlen(k, st, "Kein Zielnest fuer %s/%s (%s) - Farbe korrigieren" % (k, st, grund))
                code, erg = s.run("nest_fuer_farbe.py", [k, st, "--auto"], "2 Zielnest"); continue
            k, st = s.farbe_waehlen(k, st, "Kein Zielnest fuer %s/%s (%s) - Farbe korrigieren" % (k, st, grund))
            code, erg = s.run("nest_fuer_farbe.py", [k, st, "--auto"], "2 Zielnest")
        s.nest = erg["nest"]; s.gui.q.put(("nest", s.nest)); s.log("Zielnest: %s" % s.nest); return s.nest

    def belegung_freigeben(s, nest):
        """Nest in platte_belegung.json als frei eintragen (Benutzer hat es geraeumt)."""
        pfad = os.path.join(WS, "platte_belegung.json")
        try:
            b = json.load(open(pfad))
            alt = b.pop(nest, None)
            b["_freigegeben_" + datetime.datetime.now().strftime("%Y-%m-%d_%H%M")] = "%s vom Benutzer (GUI) als frei gemeldet, war: %s" % (nest, alt)
            json.dump(b, open(pfad, "w"), indent=1, ensure_ascii=False)
            s.log("Belegung: %s vom Benutzer als FREI gemeldet (war %s)" % (nest, alt))
        except Exception as e:
            s.log("!! Belegung nicht aenderbar: %s" % e)

    def greifen(s):
        s.schritt_anzeigen(3); s.frage("Schritt 3: Welle greifen (ueber -> ab -> zu -> heben)?")
        code, erg = s.run("hole_aus_schale.py", ["--ziel-datei", ZIEL, "--ohne-nullstellung", "--nur-hin", "--auto"] + s.tempo(), "3 Greifen")
        if code == 0:
            s.log("Welle im Greifer (%s %%)" % (erg or {}).get("prozent"))
            if (erg or {}).get("hinweis"): s.log("  (!) " + erg["hinweis"])
            return True
        if code in (2, 4):
            grund = (erg or {}).get("grund") or {2: "Greifer meldet leer (%s %%)" % (erg or {}).get("prozent"), 4: "Ziel von den Proben abgelehnt"}.get(code)
            s.log("NICHT BEKOMMEN (Exit %d): %s" % (code, grund))
            if code == 2:
                # 14.9. (Benutzer): dieselbe Welle noch einmal versuchen, ohne erst zur Kamerapose zu fahren
                ant = s.halt("Welle nicht bekommen: %s. Nochmal dieselbe Welle greifen?" % grund, ("Nochmal versuchen", "Neu suchen (Kamerapose)", "Abbrechen"))
                if ant.startswith("Nochmal"): return s.greifen()
                if ant.startswith("Abbrechen"): raise Abbruch("Benutzer beim Greifen")
            s.log("-> neuer Versuch ab Kamerapose"); return False
        s.halt("hole_aus_schale fehlgeschlagen (Exit %d). Arm steht. Weiter = neuer Versuch ab Kamerapose." % code); return False

    def kontrolle_kopf(s):
        s.schritt_anzeigen(4)
        if A.ohne_kontrollpose: s.log("Kontrollpose uebersprungen (--ohne-kontrollpose)"); return
        s.frage("Schritt 4: Kontrollpose unter der Kamera (Greifer waagerecht, J6 %s Grad) anfahren?" % A.j6_kontrolle)
        # 14.9. 14:0x: gripper_base x joint4 NICHT mehr freigeben - mit der Freigabe nahm die IK eine andere Handgelenk-
        # loesung (J5 -152 statt +115) und der Greifer fuhr real in Gelenk 4 (NOT-AUS). Stattdessen J5 > 0 erzwingen.
        code, _ = s.run("kontrollpose.py", ["--richtung", "0", "--x-oben", "1", "--j5-min", "60", "--ausfuehren"] + s.tempo(), "4 Kontrollpose")
        if code != 0: s.halt("Kontrollpose nicht erreichbar (Exit %d). Weiter = ohne Kamerakontrolle." % code); return
        # Erst JETZT (Arm steht in der bewaehrten Haltung) gripper_base x joint4 freigeben: das Modell meldet dort bei manchen
        # J6-Winkeln eine Selbstkollision, die es real nicht gibt (Vormittag, Arm stand dort). Ohne Freigabe: "kein Plan" fuer
        # die J6-Drehung und "Startzustand in Kollision" fuer die Fahrt zum Trichter. NIE fuer die Anfahrt selbst (14:0x Unfall).
        s.run("acm_greifer.py", ["--paar", "gripper_base", "joint4"], "4 ACM Handgelenk (nur Drehung/Abfahrt)")
        # 14.9. Mittag (Benutzer): Greifer war beim Anlernen um 180 Grad verdreht am Flansch montiert und ist jetzt richtig
        # herum -> die Drehung in der Kontrollpose ist -90 statt +90 (sonst haengt die Welle verkehrt). --j6-kontrolle setzt sie.
        s.run("kontrollpose.py", ["--j6-drehen", str(A.j6_kontrolle), "--ausfuehren"], "4 J6 %s" % A.j6_kontrolle)
        crop = os.path.join(TMP, "kopf_%d.png" % int(time.time()))
        code, erg = s.run("kamera_pruefung.py", ["--kopf", "--farbe", s.farbe[0] or "", "--bild", crop], "4 Kopfkontrolle")
        s.bild(crop, "Kopfkontrolle: %s" % {0: "Welle am Kopf sichtbar", 2: "KEINE Welle am TCP", 5: "unklar"}.get(code, "?"))
        # 14.9. (Benutzer): IMMER den Benutzer bestaetigen lassen - auch in AUTOMATIK -, dass die Welle GERADE haengt
        # und der KOPF OBEN (zwischen den Fingern) ist. Die Kamera liefert nur den Hinweis.
        kam = {0: "Kamera: Wellenende zwischen den Fingern sichtbar.", 2: "Kamera: KEINE Welle am TCP erkannt!", 5: "Kamera: unklar."}.get(code, "Kamera: Exit %d." % code)
        while True:
            ant = s.halt(kam + " Bitte hinsehen: haengt die Welle GERADE (senkrecht) und ist der KOPF OBEN zwischen den Fingern?",
                         ("Ja - gerade, Kopf oben", "Kopf unten - J6 180 drehen", "Schief / keine Welle - Abbrechen"))
            if ant.startswith("Ja"): s.log("Benutzer: Welle gerade, Kopf oben - weiter zum Trichter"); break
            if ant.startswith("Kopf unten"):
                s.run("kontrollpose.py", ["--j6-180", "--ausfuehren"], "4 J6 180"); kam = "Um 180 Grad gedreht."; continue
            raise Abbruch("Benutzer: Welle schief oder nicht im Greifer")

    def trichter_ablegen(s):
        s.schritt_anzeigen(5); s.frage("Schritt 5: Am Trichter ablegen (%s, ueber +%.0f, loslassen bei +%.0f)?" % (A.ablagepunkt, A.ablage_ueber, A.ablage_dz))
        while True:
            s.run("acm_greifer.py", ["--trichter-finger-frei"], "5 ACM frei")
            # Startzustand Kontrollpose: gripper_base x joint4 bleibt fuer die ABFAHRT erlaubt (Ziel und Weg sind ohne das Paar
            # frei); im finally wieder wie SRDF. Fuer die ANFAHRT einer IK-Pose wird das Paar nie freigegeben (14:0x Unfall).
            s.run("acm_greifer.py", ["--paar", "gripper_base", "joint4"], "5 ACM Handgelenk (Startzustand)")
            try:
                basis = ["--nest", A.ablagepunkt, "--ueber", A.ablage_ueber, "--ablage-dz", A.ablage_dz, "--belegung", "", "--auto"] + s.tempo()
                # 14.9. (Benutzer, Video): erst ueber dem Trichter STEHEN bleiben und bestaetigen lassen, dann loslassen.
                # Grund: der Ablagepunkt war mit verdreht montiertem Greifer angelernt (J6 um 180 falsch) - die Welle hing
                # verkehrt, stiess oben am Trichter an und fiel dahinter. Jetzt J6 im Teach-Punkt korrigiert, trotzdem fragen.
                code, erg = s.run("lege_welle.py", basis + ["--stehen"], "5 Trichter anfahren")
                if code == 3:
                    txt = "Ueber dem Trichter (+%.0f mm). Hinsehen: haengt die Welle SENKRECHT, Kopf oben, Spitze ueber dem Rohr? Loslassen?" % A.ablage_dz
                    while True:
                        ant = s.halt(txt, ("Ja - loslassen", "J6 180 drehen", "Abbrechen"))
                        if ant.startswith("Ja"): break
                        if ant.startswith("J6"): s.run("kontrollpose.py", ["--j6-180", "--ausfuehren"], "5 J6 180"); txt = "Gedreht. Jetzt richtig? Loslassen?"; continue
                        raise Abbruch("Benutzer am Trichter")
                    code, erg = s.run("lege_welle.py", basis + ["--ab-loslassen"], "5 Trichter loslassen")
            finally:
                s.run("acm_greifer.py", ["--zurueck"], "5 ACM zurueck")
            if code == 2: s.log("Greifer war leer - Welle unterwegs verloren"); s.verloren += 1; return False
            if code != 0:
                ant = s.halt("Trichter-Ablage fehlgeschlagen (Exit %d). Arm steht (Welle im Greifer). Nochmal versuchen?" % code,
                             ("Nochmal versuchen", "Zyklus neu ab Kamerapose", "Abbrechen"))
                if ant.startswith("Nochmal"): continue
                return False
            return True

    def trichter_pruefen(s):
        s.kamerapose(); s.schritt_anzeigen(6); s.frage("Schritt 6: Kamera - steht die Welle im Trichter?")
        crop = os.path.join(TMP, "trichter_%d.png" % int(time.time()))
        code, erg = s.run("kamera_pruefung.py", ["--trichter", "--farbe", s.farbe[0] or "", "--bild", crop, "--punkt", A.griffpunkt], "6 Trichterkontrolle")
        s.bild(crop, "Trichter: %s" % {0: "WELLE DA", 2: "leer", 5: "unklar"}.get(code, "?"))
        if code == 0: return True
        if code == 2: s.log("Trichter leer - Welle nicht angekommen (Benutzer bitte einsammeln)"); s.verloren += 1; return False
        ant = s.halt("Trichterkontrolle unklar. Steht die Welle im Trichter?", ("Ja", "Nein", "Abbrechen"))
        return ant == "Ja"

    def aus_trichter(s):
        s.schritt_anzeigen(7); s.frage("Schritt 7: Welle am Kopf aus dem Trichter greifen?")
        code, erg = s.run("hole_welle.py", ["--punkt", A.griffpunkt, "--nur-hin", "--auto"] + s.tempo(), "7 aus Trichter")
        if code == 0: return True
        if code == 2: s.log("Keine Welle bekommen"); s.verloren += 1; return False
        s.halt("hole_welle fehlgeschlagen (Exit %d). Arm steht. Weiter = Zyklus neu." % code); return False

    def platte(s):
        s.schritt_anzeigen(8)
        # 14.9.: vor dem Stellen die Farbe/das Nest noch einmal bestaetigen oder aendern (Benutzer: Welle war weiss/rot,
        # System hatte gelb/schwarz -> falsches Nest)
        while True:
            ant = s.halt("Schritt 8: Welle %s/%s in %s stellen?" % (s.farbe[0], s.farbe[1] or "?", s.nest), ("Ja - stellen", "Farbe/Nest aendern", "Abbrechen"))
            if ant.startswith("Ja"): break
            if ant.startswith("Farbe"):
                s.farbe_sicher = False
                if s.nest_bestimmen() is None: raise Abbruch("kein Nest"); 
                continue
            raise Abbruch("Benutzer vor der Platte")
        k, st = s.farbe
        # 14.9.: gibt es einen angelernten Ablagepunkt '<nest>_ablage' (mit Versatz, z.B. nest_11 dx -6), den nehmen -
        # die schwarze Welle sprang aus nest_11, weil der Nestpunkt selbst 6 mm daneben liegt.
        try: punkte = json.load(open(os.path.join(WS, "teach_punkte.json")))
        except Exception: punkte = {}
        ziel = s.nest + "_ablage" if (s.nest + "_ablage") in punkte else s.nest
        if ziel != s.nest: s.log("Ablagepunkt %s statt %s (angelernter Versatz)" % (ziel, s.nest))
        args = ["--nest", ziel, "--auto"] + s.tempo() + (["--farbe", k] + ([st] if st else []) if k else [])
        while True:
            code, erg = s.run("lege_welle.py", args, "8 Platte")
            if code != 6: break
            # 14.9.: Ablagestellung zu weit daneben (> 4 mm) - Welle noch im Greifer, Arm ueber dem Nest
            ant = s.halt("Ablage %s: %s. Nochmal absenken?" % (s.nest, (erg or {}).get("grund", "Abweichung zu gross")),
                         ("Nochmal versuchen", "Trotzdem loslassen", "Abbrechen"))
            if ant.startswith("Nochmal"): continue
            if ant.startswith("Trotzdem"): code, erg = s.run("lege_welle.py", args + ["--max-abweichung", "99"], "8 Platte (erzwungen)"); break
            raise Abbruch("Benutzer an der Platte")
        if code == 0: s.gelegt.append("%s(%s/%s)" % (s.nest, k, st or "?")); s.gui.q.put(("gelegt", len(s.gelegt))); s.log("Welle steht in %s" % s.nest); return True
        if code == 2:
            # 14.9.: Greifer meldete 5 % (Welle schief am Kopf) -> "leer" -> Kette sprang mit Welle im Greifer an den Anfang
            ant = s.halt("Greifer meldet %s %% - Welle NICHT mehr im Greifer? Hinsehen!" % (erg or {}).get("prozent_vorher"),
                         ("Welle ist drin - trotzdem stellen", "Wirklich leer - weiter", "Abbrechen"))
            if ant.startswith("Welle ist drin"):
                code, erg = s.run("lege_welle.py", args + ["--ohne-haltepruefung"], "8 Platte (ohne Haltepruefung)")
                if code == 0: s.gelegt.append("%s(%s/%s)" % (s.nest, k, st or "?")); s.gui.q.put(("gelegt", len(s.gelegt))); s.log("Welle steht in %s" % s.nest); return True
            elif ant.startswith("Abbrechen"): raise Abbruch("Benutzer an der Platte")
        s.halt("Ablage in %s: Exit %d (%s). Weiter = naechster Zyklus." % (s.nest, code, {2: "Greifer leer", 4: "Tastprobe: steht nicht"}.get(code, "Fehler"))); return False

    # ---------------------------------------------------------------- Hauptschleife
    def estop_aktiv(s):
        """/estop lesen (latched). True = NOT-AUS aktiv -> die Bruecke blockiert jede Fahrt."""
        try:
            env = dict(os.environ); env.setdefault("LC_ALL", "C"); env.setdefault("CYCLONEDDS_URI", "file://" + os.path.join(WS, "cyclonedds_loopback.xml"))
            r = subprocess.run(["bash", "-c", "source /opt/ros/galactic/setup.bash && source %s/install/setup.bash && timeout 6 ros2 topic echo /estop "
                                "--qos-durability transient_local --qos-reliability reliable" % WS], capture_output=True, text=True, env=env, timeout=10)
            return "data: true" in r.stdout
        except Exception: return False

    def lauf(s):
        try:
            # 14.9.: bei aktivem NOT-AUS lief die Kette "erfolgreich" durch, ohne dass der Arm sich bewegte
            if s.estop_aktiv():
                s.halt("NOT-AUS ist AKTIV (/estop = true): der Arm faehrt nicht. Erst 'Freigabe' druecken, dann Weiter.")
                if s.estop_aktiv(): raise Abbruch("NOT-AUS noch aktiv")
            wieder = A.ab_schritt
            while wieder >= 5:
                # Wiedereinstieg (14.9.): Welle haengt im Greifer, Arm steht (z.B. Kontrollpose). Kein Referenzbild moeglich
                # (Trichter nicht einsehbar) -> die Trichterkontrolle in Schritt 6 nutzt das alte trichter_hintergrund.npz.
                z = json.load(open(ZIEL)); s.farbe = (z.get("koerper"), None if z.get("streifen") in (None, "unbekannt") else z.get("streifen")); s.farbe_sicher = False
                s.gui.q.put(("farbe", s.farbe[0], s.farbe[1], True, None))
                s.log("WIEDEREINSTIEG ab Schritt %d: Welle %s/%s im Greifer (aus welle_ziel.json)" % (wieder, s.farbe[0], s.farbe[1] or "?"))
                if s.nest_bestimmen() is None: raise Abbruch("kein Nest")
                if wieder >= 8:
                    # Welle haengt (aus dem Trichter geholt) im Greifer: nur noch ins Nest stellen
                    s.log("WIEDEREINSTIEG: Welle im Greifer -> Schritt 8 (Nest)"); s.platte()
                elif wieder >= 7:
                    # Welle steckt (von Hand) im Trichter: nur noch holen und ins Nest stellen
                    s.log("WIEDEREINSTIEG: Welle steht im Trichter -> Schritt 7 (holen) + 8 (Nest)")
                    if s.aus_trichter(): s.platte()
                elif s.trichter_ablegen() and s.trichter_pruefen() and s.aus_trichter(): s.platte()
                # 23.9. (Benutzer): nach dem Wiedereinstieg selbst waehlen, wie es weitergeht - kein Neustart des Fensters
                ant = s.halt("Wiedereinstieg fertig. Wie weiter?", ("Nochmal ab 7 (Trichter)", "Nochmal ab 8 (Greifer)", "Voller Zyklus", "Beenden"))
                if ant.startswith("Nochmal ab 7"): wieder = 7
                elif ant.startswith("Nochmal ab 8"): wieder = 8
                elif ant.startswith("Voller"): wieder = 0
                else: raise Abbruch("Benutzer: Ende nach dem Wiedereinstieg")
            s.log("Modus %s. Trichter muss LEER sein - Referenzbild wird aufgenommen." % s.modus)
            s.kamerapose(); s.run("kamera_pruefung.py", ["--trichter-referenz"], "0 Trichter-Referenz")
            zyklus = 0
            while not s.stop.is_set():
                zyklus += 1; s.log("===== Zyklus %d (gelegt %d, verloren %d)" % (zyklus, len(s.gelegt), s.verloren))
                if zyklus > 1: s.kamerapose()
                z = s.welle_bestimmen()
                if z is None:
                    s.log("Keine greifbare Welle mehr in der Schale.")
                    if s.trichter_pruefen_leise(): s.log("... aber im Trichter steht noch eine - weiter mit Schritt 7.");
                    else: break
                    if s.aus_trichter() and s.platte(): continue
                    else: continue
                if s.nest_bestimmen() is None: continue
                ok = False
                for versuch in range(A.max_griffe):
                    if s.greifen(): ok = True; break
                    s.kamerapose()
                    if s.welle_bestimmen() is None: break
                if not ok: s.log("Welle nach %d Versuchen nicht bekommen - naechster Zyklus" % A.max_griffe); continue
                s.kontrolle_kopf()
                if not s.trichter_ablegen(): continue
                if not s.trichter_pruefen(): continue
                if A.nur_schale_trichter: s.log("--nur-schale-trichter: Ende nach der Trichterkontrolle."); break
                if not s.aus_trichter(): continue
                s.platte()
                if len(s.gelegt) >= 19: s.log("19 Wellen gelegt - fertig."); break
            s.log("FERTIG: %d Welle(n) gelegt: %s | verloren %d" % (len(s.gelegt), " ".join(s.gelegt), s.verloren))
        except Abbruch as e:
            s.log("ABBRUCH (%s). Arm steht, NICHTS faehrt." % e)
        except Exception as e:
            s.log("FEHLER: %r" % e)
        s.gui.q.put(("ende",))

    def trichter_pruefen_leise(s):
        code, _ = s.run("kamera_pruefung.py", ["--trichter", "--punkt", A.griffpunkt], "Trichterkontrolle (Schale leer)")
        return code == 0


SCHRITT_WAHL = {"1 - voller Zyklus (ab Schale)": 0,
                "5 - Welle im Greifer -> Trichter": 5,
                "7 - Welle im Trichter -> holen + Nest": 7,
                "8 - Welle im Greifer -> nur Nest": 8}


# ==================================================================== Fenster
class GUI:
    def __init__(s):
        s.root = tk.Tk(); s.root.title("Wellen-Zyklus: Schale -> Trichter -> Ablageplatte"); s.root.geometry("1180x760")
        s.q = queue.Queue(); s.z = Zyklus(s); s.foto = None
        oben = tk.Frame(s.root); oben.pack(fill="x", padx=8, pady=6)
        s.b_auto = tk.Button(oben, text="AUTOMATIK\n(keine Rueckfragen)", font=("Sans", 13, "bold"), bg="#3a7", fg="white", width=18, height=2, command=lambda: s.start("AUTOMATIK"))
        s.b_man = tk.Button(oben, text="MANUELL\n(vor jedem Schritt bestaetigen)", font=("Sans", 13, "bold"), bg="#37a", fg="white", width=26, height=2, command=lambda: s.start("MANUELL"))
        s.b_auto.pack(side="left", padx=4); s.b_man.pack(side="left", padx=4)
        # 23.9. (Benutzer): den Einstiegsschritt selbst waehlen statt das Fenster mit --ab-schritt neu zu starten
        vorgabe = next((k for k, v in SCHRITT_WAHL.items() if v == A.ab_schritt), list(SCHRITT_WAHL)[0])
        s.v_schritt = tk.StringVar(value=vorgabe)
        zeile2 = tk.Frame(s.root); zeile2.pack(fill="x", padx=8, pady=(0, 4))
        tk.Label(zeile2, text="Start ab Schritt:", font=("Sans", 11)).pack(side="left", padx=(0, 4))
        s.o_schritt = tk.OptionMenu(zeile2, s.v_schritt, *SCHRITT_WAHL.keys()); s.o_schritt.config(font=("Sans", 11), width=34); s.o_schritt.pack(side="left")
        tk.Label(zeile2, text="  -> dann MANUELL oder AUTOMATIK druecken", font=("Sans", 10), fg="#555").pack(side="left")
        s.b_notaus = tk.Button(oben, text="NOT-AUS", font=("Sans", 20, "bold"), bg="#c00", fg="white", width=12, height=2, command=s.not_aus)
        s.b_notaus.pack(side="right", padx=4)
        s.b_frei = tk.Button(oben, text="Freigabe", font=("Sans", 11), bg="#888", fg="white", command=s.z.freigabe); s.b_frei.pack(side="right", padx=4)
        # Greifer von Hand (14.9., Benutzer): nach einem Abbruch die Welle loslassen, ohne den Zyklus neu zu starten
        tk.Button(oben, text="Greifer ZU", font=("Sans", 11, "bold"), bg="#555", fg="white", command=lambda: s.greifer_knopf("--zu")).pack(side="right", padx=4)
        tk.Button(oben, text="Greifer AUF", font=("Sans", 11, "bold"), bg="#357", fg="white", command=lambda: s.greifer_knopf("--auf")).pack(side="right", padx=4)
        s.l_modus = tk.Label(oben, text="Modus: -", font=("Sans", 12, "bold")); s.l_modus.pack(side="left", padx=16)
        mitte = tk.Frame(s.root); mitte.pack(fill="both", expand=True, padx=8)
        links = tk.Frame(mitte); links.pack(side="left", fill="y")
        tk.Label(links, text="Programmablauf", font=("Sans", 12, "bold")).pack(anchor="w")
        s.schritte = []
        for t in SCHRITTE:
            l = tk.Label(links, text="   " + t, font=("Mono", 11), anchor="w", width=54); l.pack(anchor="w"); s.schritte.append(l)
        s.l_farbe = tk.Label(links, text="Welle: -", font=("Sans", 14, "bold"), fg="#333", anchor="w", justify="left"); s.l_farbe.pack(anchor="w", pady=(12, 0))
        s.l_nest = tk.Label(links, text="Zielnest: -   gelegt: 0", font=("Sans", 12), anchor="w"); s.l_nest.pack(anchor="w")
        s.l_frage = tk.Label(links, text="", font=("Sans", 11), fg="#a00", wraplength=420, justify="left"); s.l_frage.pack(anchor="w", pady=(12, 4))
        s.f_knoepfe = tk.Frame(links); s.f_knoepfe.pack(anchor="w")
        rechts = tk.Frame(mitte); rechts.pack(side="left", fill="both", expand=True, padx=(12, 0))
        s.l_bildtitel = tk.Label(rechts, text="Bild: -", font=("Sans", 11)); s.l_bildtitel.pack(anchor="w")
        s.canvas = tk.Label(rechts, bg="#222", width=560, height=420); s.canvas.pack(anchor="nw")
        s.text = tk.Text(rechts, height=12, font=("Mono", 9)); s.text.pack(fill="both", expand=True, pady=(6, 0))
        s.root.after(150, s.poll)

    def greifer_knopf(s, arg):
        """Greifer sofort auf/zu (eigener Unterprozess, unabhaengig vom laufenden Schritt)."""
        def lauf():
            env = dict(os.environ); env.setdefault("LC_ALL", "C"); env.setdefault("CYCLONEDDS_URI", "file://" + os.path.join(WS, "cyclonedds_loopback.xml"))
            r = subprocess.run([sys.executable, os.path.join(HIER, "greifer.py"), arg], capture_output=True, text=True, env=env)
            s.q.put(("log", "Greifer-Knopf %s: %s" % (arg, (r.stdout.strip().splitlines() or ["?"])[-1])))
        threading.Thread(target=lauf, daemon=True).start()

    def start(s, modus):
        A.ab_schritt = SCHRITT_WAHL.get(s.v_schritt.get(), 0)
        s.z.modus = modus; s.l_modus.config(text="Modus: " + modus); s.b_auto.config(state="disabled"); s.b_man.config(state="disabled")
        threading.Thread(target=s.z.lauf, daemon=True).start()

    def not_aus(s):
        s.z.not_aus(); s.l_frage.config(text="NOT-AUS ausgeloest. Freigabe: Knopf 'Freigabe', dann Fenster neu starten.")

    def zeige_frage(s, text, optionen):
        s.l_frage.config(text=text)
        for w in s.f_knoepfe.winfo_children(): w.destroy()
        for i, o in enumerate(optionen):   # 23.9.: bis 2 Knoepfe je Zeile (linke Spalte ~500 px), sonst ragten 4 lange Knoepfe aus dem Fenster
            tk.Button(s.f_knoepfe, text=o, font=("Sans", 12, "bold"), width=max(14, min(30, len(o) + 2)),
                      bg="#3a7" if o.startswith(("Weiter", "Ja")) else "#a33" if o in ("Abbrechen", "Beenden") else "#cb3",
                      fg="white", command=lambda o=o: s.antwort(o)).grid(row=i // 2, column=i % 2, padx=3, pady=3, sticky="w")

    def antwort(s, o):
        s.z.antwort = o; s.z.weiter.set(); s.l_frage.config(text="")
        for w in s.f_knoepfe.winfo_children(): w.destroy()

    FARBEN = ("schwarz", "gelb", "weiss", "gruen", "rot")
    def zeige_farbwahl(s, k, st, grund):
        """Zwei Auswahlfelder (Koerper, Streifen) + Uebernehmen/Abbrechen - statt Exit, wenn die Kamera unsicher ist."""
        s.l_frage.config(text=grund)
        for w in s.f_knoepfe.winfo_children(): w.destroy()
        vk = tk.StringVar(value=k if k in s.FARBEN else s.FARBEN[0]); vs = tk.StringVar(value=st if st in s.FARBEN else s.FARBEN[0])
        z1 = tk.Frame(s.f_knoepfe); z1.pack(anchor="w")
        tk.Label(z1, text="Koerper:", font=("Sans", 12)).pack(side="left"); tk.OptionMenu(z1, vk, *s.FARBEN).pack(side="left", padx=6)
        tk.Label(z1, text="Streifen:", font=("Sans", 12)).pack(side="left", padx=(12, 0)); tk.OptionMenu(z1, vs, *s.FARBEN).pack(side="left", padx=6)
        z2 = tk.Frame(s.f_knoepfe); z2.pack(anchor="w")
        tk.Button(z2, text="Farbe richtig - Weiter", font=("Sans", 12, "bold"), width=22, bg="#3a7", fg="white",
                  command=lambda: s.antwort((vk.get(), vs.get()))).pack(side="left", padx=3, pady=3)
        tk.Button(z2, text="Abbrechen", font=("Sans", 12, "bold"), width=14, bg="#a33", fg="white", command=lambda: s.antwort("Abbrechen")).pack(side="left", padx=3, pady=3)

    def poll(s):
        try:
            while True:
                ev = s.q.get_nowait()
                if ev[0] == "log":
                    s.text.insert("end", time.strftime("%H:%M:%S ") + ev[1] + "\n"); s.text.see("end")
                elif ev[0] == "schritt":
                    for i, l in enumerate(s.schritte): l.config(bg="#ff3" if i == ev[1] else s.root.cget("bg"), text=("-> " if i == ev[1] else "   ") + SCHRITTE[i])
                elif ev[0] == "farbe":
                    k, st, sicher, n = ev[1:]
                    s.l_farbe.config(text="Welle: Koerper %s / Streifen %s%s   (%s im Bild)" % (k, st or "?", "" if sicher else "  UNSICHER", n))
                elif ev[0] == "nest": s.l_nest.config(text="Zielnest: %s   gelegt: %d" % (ev[1], len(s.z.gelegt)))
                elif ev[0] == "gelegt": s.l_nest.config(text="Zielnest: %s   gelegt: %d" % (s.z.nest, ev[1]))
                elif ev[0] == "bild":
                    try:
                        im = Image.open(ev[1]); im.thumbnail((560, 420)); s.foto = ImageTk.PhotoImage(im); s.canvas.config(image=s.foto, width=im.width, height=im.height)
                        s.l_bildtitel.config(text="Bild: " + ev[2])
                    except Exception as e: s.text.insert("end", "Bild nicht ladbar: %s\n" % e)
                elif ev[0] == "frage": s.zeige_frage(ev[1], ev[2])
                elif ev[0] == "farbwahl": s.zeige_farbwahl(ev[1], ev[2], ev[3])
                elif ev[0] == "ende":
                    s.l_frage.config(text="Ablauf beendet. Fenster schliessen oder neu starten."); s.b_auto.config(state="normal"); s.b_man.config(state="normal")
        except queue.Empty: pass
        s.root.after(150, s.poll)


if __name__ == "__main__":
    GUI().root.mainloop()
