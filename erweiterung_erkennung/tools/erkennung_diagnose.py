#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""Warum ist YOLO auf dem Jetson so langsam? Messung ohne Roboter und ohne ROS (2026-10-11).

Jede Variante laeuft in einem FRISCHEN Prozess (sonst waere der erste Aufruf schon warm) und misst mit
time.perf_counter() + torch.cuda.synchronize():
  import torch | CUDA-Kontext (torch.zeros(1).cuda()) | erstes conv2d (cuDNN laden) | YOLO(weights) |
  erste predict | 20 weitere predict (Mittel/Median/Min/Max) | Peak-RSS
Varianten (Aufgabe Phase A):
  a  wie im Versuch: imgsz 640, FP32, retina_masks=True, ganzes Bild
  b  half=True
  u  Ultralytics model.predict(device=0) unveraendert (zeigt den NaN-Fehler: 0 Detektionen, s. yolo_segmentierung.GpuModell)
  c  retina_masks=False (nur CPU-Weg; auf der GPU hat der eigene Weg nur Masken in Bildaufloesung)
  d  nur der Ausschnitt um die Schale (ROI) ins Netz
  e  zweiter Start direkt nach a (zeigt, ob ein Cache wirkt)
  f  wie a mit CUDA_CACHE_MAXSIZE=2 GiB
  g  Kombination b + c + d (Vorgabe des Erkennungsdienstes)
Ausgabe: ~/ros2_ws/erkennung_diagnose_<datum>.json + Tabelle auf der Konsole.

    python3 tools/erkennung_diagnose.py                    # alle Varianten (~20 min)
    python3 tools/erkennung_diagnose.py --varianten a g
"""
import argparse, json, os, resource, subprocess, sys, time

HIER = os.path.dirname(os.path.abspath(__file__)); WS = os.path.dirname(HIER)
ap = argparse.ArgumentParser()
ap.add_argument("--varianten", nargs="*", default=list("uabdefg"))
ap.add_argument("--frame", default=os.path.join(WS, "testdaten", "wellen_11_kamerapose_2026-09-13.npz"))
ap.add_argument("--aufloesung", default="848x480", help="Frame vor der Messung auf die Kameraaufloesung des Zyklus skalieren")
ap.add_argument("--n", type=int, default=20)
ap.add_argument("--worker", default="", help=argparse.SUPPRESS)
ap.add_argument("--aus", default=os.path.join(WS, "erkennung_diagnose_%s.json" % time.strftime("%Y-%m-%d")))
a = ap.parse_args()

VARIANTEN = {"u": dict(half=False, retina=True, roi=False, ultra=True), "a": dict(half=False, retina=True, roi=False), "b": dict(half=True, retina=True, roi=False),
             "c": dict(half=False, retina=False, roi=False), "d": dict(half=False, retina=True, roi=True),
             "e": dict(half=False, retina=True, roi=False), "f": dict(half=False, retina=True, roi=False, cache=True),
             "g": dict(half=True, retina=False, roi=True)}


def sh(cmd):
    try: return subprocess.run(cmd, shell=True, capture_output=True, text=True, timeout=20).stdout.strip()
    except Exception as e: return "(%s)" % e


def mem():
    d = {}
    for z in open("/proc/meminfo"):
        k, v = z.split(":"); d[k] = int(v.split()[0]) // 1024
    return {"frei_mb": d["MemFree"], "verfuegbar_mb": d["MemAvailable"], "swap_belegt_mb": d["SwapTotal"] - d["SwapFree"]}


def temp():
    out = {}
    import glob
    for z in sorted(glob.glob("/sys/devices/virtual/thermal/thermal_zone*")):
        try: out[open(z + "/type").read().strip()] = int(open(z + "/temp").read()) / 1000
        except Exception: pass
    return out


def worker(name):
    """Eine Variante, frischer Prozess. Druckt am Ende eine JSON-Zeile."""
    v = VARIANTEN[name]; r = {"variante": name, **{k: v.get(k, False) for k in ("half", "retina", "roi", "cache")}}
    T = time.perf_counter
    t = T(); import torch; r["t_import_torch"] = T() - t
    t = T(); x = torch.zeros(1).cuda(); torch.cuda.synchronize(); r["t_cuda_kontext"] = T() - t
    t = T(); import torch.nn.functional as F
    y = F.conv2d(torch.zeros(1, 3, 64, 64).cuda(), torch.zeros(8, 3, 3, 3).cuda()); torch.cuda.synchronize(); r["t_erstes_conv2d"] = T() - t
    r["cudnn_benchmark"] = bool(torch.backends.cudnn.benchmark)
    import numpy as np, cv2
    sys.path.insert(0, HIER); import yolo_segmentierung as ys
    d = np.load(a.frame, allow_pickle=True); farbe = d["farbe"]
    W, H = (int(s) for s in a.aufloesung.split("x"))
    roi = None
    if v["roi"]:
        sch = dict(d["schale"].item()); K = tuple(d["K"])
        sk = W / farbe.shape[1]; K = (K[0] * sk, K[1] * sk, K[2] * sk, K[3] * sk)
    if farbe.shape[1] != W: farbe = cv2.resize(farbe, (W, H), interpolation=cv2.INTER_AREA)
    if v["roi"]: roi = ys.schale_roi(farbe.shape, K, tuple(d["q"]), np.array(d["t"]), sch); r["roi"] = roi
    if v.get("ultra"):
        # unveraenderter Ultralytics-Weg (AutoBackend) - nur zur Dokumentation des Fehlers
        from ultralytics import YOLO
        t = T(); m = YOLO(ys.GEWICHTE, task="segment"); r["t_yolo_laden"] = T() - t
        t = T(); res = m.predict(farbe, conf=0.70, imgsz=640, retina_masks=True, device="0", verbose=False)[0]; torch.cuda.synchronize()
        r["t_erste_inferenz"] = T() - t; r["anzahl_erste"] = len(res.boxes)
        zs = []
        for _ in range(a.n):
            t = T(); res = m.predict(farbe, conf=0.70, imgsz=640, retina_masks=True, device="0", verbose=False)[0]; torch.cuda.synchronize(); zs.append(T() - t)
        zs.sort(); r.update({"t_inferenz_mittel": sum(zs) / len(zs), "t_inferenz_median": zs[len(zs) // 2], "t_inferenz_min": zs[0], "t_inferenz_max": zs[-1],
                             "anzahl": len(res.boxes), "peak_rss_mb": resource.getrusage(resource.RUSAGE_SELF).ru_maxrss / 1024})
        print("ERGEBNIS " + json.dumps(r)); sys.stdout.flush(); return
    t = T(); m = ys.lade_modell(); r["t_yolo_laden"] = T() - t
    kw = dict(roi=roi, half=v["half"], retina=v["retina"])
    t = T(); out, _ = ys.segmentieren(m, farbe, **kw); torch.cuda.synchronize(); r["t_erste_inferenz"] = T() - t; r["anzahl_erste"] = len(out)
    zeiten = []
    for _ in range(a.n):
        t = T(); out, _ = ys.segmentieren(m, farbe, **kw); torch.cuda.synchronize(); zeiten.append(T() - t)
    zs = sorted(zeiten)
    r.update({"t_inferenz_mittel": sum(zs) / len(zs), "t_inferenz_median": zs[len(zs) // 2], "t_inferenz_min": zs[0], "t_inferenz_max": zs[-1],
              "anzahl": len(out), "klassen": [o["klasse"] for o in out], "conf": [round(o["conf"], 3) for o in out],
              "peak_rss_mb": resource.getrusage(resource.RUSAGE_SELF).ru_maxrss / 1024})
    print("ERGEBNIS " + json.dumps(r)); sys.stdout.flush()


if a.worker:
    worker(a.worker); sys.exit(0)

cache = os.path.expanduser("~/.nv/ComputeCache")
system = {"datum": time.strftime("%Y-%m-%d %H:%M"), "nvpmodel": sh("nvpmodel -q 2>&1 | grep -v WARN"),
          "jetson_clocks": sh("jetson_clocks --show 2>&1 | head -12"), "swap": sh("swapon --show"),
          "speicher": mem(), "temperatur": temp(), "speicherfresser": sh("ps aux --sort=-rss | head -8 | cut -c1-150"),
          "rviz_laeuft": bool(sh("pgrep -x rviz2")), "desktop": sh("systemctl get-default"),
          "CUDA_CACHE_MAXSIZE": os.environ.get("CUDA_CACHE_MAXSIZE"), "CUDA_CACHE_PATH": os.environ.get("CUDA_CACHE_PATH"),
          "compute_cache_vorher": sh("du -sk %s 2>/dev/null | cut -f1" % cache)}
torch_info = sh("%s -c \"import torch;print(torch.__version__, torch.version.cuda, torch.backends.cudnn.version(), torch.cuda.get_arch_list())\"" % sys.executable)
system["torch"] = torch_info; system["sm_53_im_build"] = "sm_53" in torch_info
print(json.dumps(system, indent=1, ensure_ascii=False))
ergebnisse = []
for name in a.varianten:
    env = dict(os.environ)
    if VARIANTEN[name].get("cache"): env["CUDA_CACHE_MAXSIZE"] = "2147483648"
    vorher = mem(); t0 = time.perf_counter()
    p = subprocess.run([sys.executable, os.path.abspath(__file__), "--worker", name, "--frame", a.frame, "--aufloesung", a.aufloesung, "--n", str(a.n)],
                       capture_output=True, text=True, env=env)
    r = None
    for z in p.stdout.splitlines():
        if z.startswith("ERGEBNIS "): r = json.loads(z[9:])
    if r is None: r = {"variante": name, "fehler": (p.stderr or p.stdout)[-800:]}
    r["t_prozess_gesamt"] = time.perf_counter() - t0; r["speicher_vorher"] = vorher; r["temperatur_nachher"] = temp()
    ergebnisse.append(r)
    print("%s: Prozess %.1f s | torch %.1f | CUDA %.1f | conv2d %.1f | Laden %.1f | 1. Inferenz %.1f | Median %.3f s | %s Wellen | RSS %.0f MB"
          % (name, r["t_prozess_gesamt"], r.get("t_import_torch", -1), r.get("t_cuda_kontext", -1), r.get("t_erstes_conv2d", -1),
             r.get("t_yolo_laden", -1), r.get("t_erste_inferenz", -1), r.get("t_inferenz_median", -1), r.get("anzahl", "?"), r.get("peak_rss_mb", -1)))
    sys.stdout.flush()
system["compute_cache_nachher"] = sh("du -sk %s 2>/dev/null | cut -f1" % cache)
json.dump({"system": system, "varianten": ergebnisse}, open(a.aus, "w"), indent=1, ensure_ascii=False)
print("-> %s" % a.aus)
