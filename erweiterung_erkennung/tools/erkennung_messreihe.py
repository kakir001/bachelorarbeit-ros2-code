#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""Messreihe am Versuchsaufbau: N Wellensuchen je Verfahren (klassisch / yolo), KEINE Roboterbewegung (2026-10-11).

Jede Suche ist ein eigener Aufruf von welle_finden_schale.py - genau wie Schritt 1 in zyklus_gui.py - und wird von
aussen gestoppt (t_suche = Prozessstart bis Ende, wie der Bediener sie erlebt). Aus welle_ziel.json kommen
t_segmentierung_s / t_inferenz_s, Anzahl, gewaehlte Welle und Konfidenz. Waehrend der Reihe werden der kleinste
freie Arbeitsspeicher (MemAvailable) und die hoechste Temperatur mitgeschrieben.
Fuer yolo muss der Erkennungsdienst laufen (tools/erkennung_dienst.py); --dienst-starten startet ihn und misst t_bereit.

    python3 tools/erkennung_messreihe.py --n 10 --verfahren klassisch yolo --dienst-starten
Ergebnis: ~/ros2_ws/erkennung_messreihe_<datum>.json + Tabelle.
"""
import argparse, glob, json, os, signal, statistics, subprocess, sys, threading, time

HIER = os.path.dirname(os.path.abspath(__file__)); WS = os.path.dirname(HIER)
ap = argparse.ArgumentParser()
ap.add_argument("--n", type=int, default=10)
ap.add_argument("--verfahren", nargs="+", default=["klassisch", "yolo"])
ap.add_argument("--dienst-starten", action="store_true")
ap.add_argument("--aus", default=os.path.join(WS, "erkennung_messreihe_%s.json" % time.strftime("%Y-%m-%d_%H%M")))
ap.add_argument("--abwechselnd", action="store_true",
                help="klassisch/yolo im Wechsel (1 k, 1 y, 2 k, ...) - gleiche Bedingungen (Swap, Waerme) fuer beide; der Dienst laeuft dabei durchgehend")
ap.add_argument("--zusatz", nargs="*", default=[], help="weitere Argumente fuer welle_finden_schale.py")
a = ap.parse_args()

mess = {"mem_min_mb": 10 ** 9, "temp_max": 0.0}; stop = threading.Event()


def waechter():
    while not stop.is_set():
        try:
            for z in open("/proc/meminfo"):
                if z.startswith("MemAvailable"): mess["mem_min_mb"] = min(mess["mem_min_mb"], int(z.split()[1]) // 1024)
            for p in glob.glob("/sys/devices/virtual/thermal/thermal_zone*/temp"):
                mess["temp_max"] = max(mess["temp_max"], int(open(p).read()) / 1000)
        except Exception: pass
        time.sleep(0.5)


def reihe(v, nur=None):
    werte = []
    for i in (range(a.n) if nur is None else [nur]):
        ziel = "/tmp/welle_ziel_messreihe.json"
        try: os.remove(ziel)
        except OSError: pass
        mess["mem_min_mb"] = 10 ** 9; mess["temp_max"] = 0.0
        t0 = time.time()
        p = subprocess.run([sys.executable, "-u", os.path.join(HIER, "welle_finden_schale.py"), "--verfahren", v, "--ziel", ziel] + a.zusatz,
                           capture_output=True, text=True)
        dt = time.time() - t0
        e = {"lauf": i + 1, "exit": p.returncode, "t_suche_s": round(dt, 2), "mem_min_mb": mess["mem_min_mb"], "temp_max": mess["temp_max"]}
        if p.returncode == 0 and os.path.exists(ziel):
            z = json.load(open(ziel))
            e.update({k: z.get(k) for k in ("t_segmentierung_s", "t_inferenz_s", "t_gesamt_s", "anzahl", "konfidenz", "koerper", "streifen", "mitte_mm", "yaw_grad")})
            e["greifbar"] = 1 + sum(1 for w in z.get("weitere", []) if not w.get("gruende"))
        else:
            e["meldung"] = (p.stdout.strip().splitlines() or [""])[-1][:200]
        werte.append(e)
        print("%-9s %2d: Exit %d, Suche %.1f s, Seg %s s, %s Wellen (greifbar %s), gewaehlt %s/%s Konf %s, RAM min %d MB, %.0f C"
              % (v, i + 1, p.returncode, dt, e.get("t_segmentierung_s"), e.get("anzahl"), e.get("greifbar"), e.get("koerper"), e.get("streifen"),
                 e.get("konfidenz"), e["mem_min_mb"], e["temp_max"])); sys.stdout.flush()
    return werte


threading.Thread(target=waechter, daemon=True).start()
erg = {"datum": time.strftime("%Y-%m-%d %H:%M"), "n": a.n, "zusatz": a.zusatz, "reihen": {}}
dienst = None
for v in a.verfahren:
    if v == "yolo" and a.dienst_starten:
        try: os.remove(os.path.join(WS, ".erkennung_status.json"))
        except OSError: pass
        t0 = time.time(); mess["mem_min_mb"] = 10 ** 9
        dienst = subprocess.Popen([sys.executable, "-u", os.path.join(HIER, "erkennung_dienst.py")], stdout=open("/tmp/erkennung_dienst_messreihe.log", "w"),
                                  stderr=subprocess.STDOUT, preexec_fn=os.setsid)
        while True:
            try: st = json.load(open(os.path.join(WS, ".erkennung_status.json")))
            except Exception: st = {}
            if st.get("zustand") in ("bereit", "fehler") or dienst.poll() is not None: break
            time.sleep(0.5)
        erg["dienst"] = dict(st, t_bereit_gemessen_s=round(time.time() - t0, 1), mem_min_beim_laden_mb=mess["mem_min_mb"])
        print("Dienst: %s nach %.1f s (Laden %s s, Aufwaermen %s s), RAM min %d MB" % (st.get("zustand"), time.time() - t0, st.get("t_laden_s"), st.get("t_aufwaermen_s"), mess["mem_min_mb"]))
    if not a.abwechselnd: erg["reihen"][v] = reihe(v)
if a.abwechselnd:
    erg["abwechselnd"] = True
    for i in range(a.n):
        for v in a.verfahren: erg["reihen"].setdefault(v, []).extend(reihe(v, i))
stop.set()
if dienst is not None:
    os.killpg(os.getpgid(dienst.pid), signal.SIGTERM)
    try: dienst.wait(5)
    except subprocess.TimeoutExpired: os.killpg(os.getpgid(dienst.pid), signal.SIGKILL)
print("\nZUSAMMENFASSUNG (n = %d je Verfahren)" % a.n)
for v, w in erg["reihen"].items():
    ok = [e for e in w if e["exit"] == 0]
    ts = [e["t_suche_s"] for e in w]; tg = [e["t_segmentierung_s"] for e in ok if e.get("t_segmentierung_s") is not None]
    zf = {"n_ok": len(ok), "t_suche_median": statistics.median(ts), "t_suche_min": min(ts), "t_suche_max": max(ts),
          "t_seg_median": statistics.median(tg) if tg else None, "anzahl_median": statistics.median([e["anzahl"] for e in ok]) if ok else None,
          "mem_min_mb": min(e["mem_min_mb"] for e in w), "temp_max": max(e["temp_max"] for e in w)}
    erg.setdefault("zusammenfassung", {})[v] = zf
    print("%-9s ok %d/%d | Suche Median %.1f s (%.1f-%.1f) | Segmentierung Median %s s | Wellen Median %s | RAM min %d MB | max %.0f C"
          % (v, len(ok), len(w), zf["t_suche_median"], zf["t_suche_min"], zf["t_suche_max"], None if zf["t_seg_median"] is None else round(zf["t_seg_median"], 2),
             zf["anzahl_median"], zf["mem_min_mb"], zf["temp_max"]))
json.dump(erg, open(a.aus, "w"), indent=1, ensure_ascii=False)
print("-> %s" % a.aus)
