#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""Erkennungsdienst: YOLO11-seg EINMAL laden und aufwaermen, dann geladen bleiben (2026-10-11).

Warum: im Versuch kostete die KI vor allem das wiederholte Laden + Aufwaermen (CUDA-Kontext, cuDNN, erste
Inferenz: zusammen Minuten auf dem Jetson Nano). Im zweiphasigen Ablauf (run_pick_preview.sh) wiederholte sich
das bei JEDER Welle. Der Dienst laedt einmal je Sitzung; ab der zweiten Anfrage zaehlt nur noch die Inferenz.

Eigenstaendiger Prozess, KEIN ROS (Galactic-pybind11-Falle im GUI-Prozess; Kamera holt welle_finden_schale.py).
Schnittstelle: Unix-Socket ~/ros2_ws/.erkennung.sock, eine Zeile JSON je Anfrage:
    Anfrage  {"bild_npz": "<pfad mit Schluessel farbe>", "roi": [x0, y0, x1, y1] | null}
             {"ping": true}                                         -> {"ok": true, "zustand": ...}
    Antwort  {"ok": true, "masken_npz": "<pfad>", "anzahl": n, "t_inferenz_s": ...}  (Masken s. yolo_segmentierung.speichern_npz)
Zustand fuer die Oberflaeche (liest nur diese Datei): ~/ros2_ws/.erkennung_status.json
    {"zustand": "laedt|waermt|bereit|fehler", "seit": <unix>, "t_bereit_s": ..., "pid": ..., "backend": ...}
Beenden: SIGTERM/SIGINT (zyklus_gui: SIGTERM, nach 5 s SIGKILL) - Socket und Status werden aufgeraeumt.

    python3 tools/erkennung_dienst.py                 # Vorgabe: backend auto
    python3 tools/erkennung_dienst.py --backend pytorch
"""
import argparse, json, os, signal, socket, sys, tempfile, threading, time

HIER = os.path.dirname(os.path.abspath(__file__)); WS = os.path.dirname(HIER)
ap = argparse.ArgumentParser()
ap.add_argument("--socket", default=os.path.join(WS, ".erkennung.sock"))
ap.add_argument("--status", default=os.path.join(WS, ".erkennung_status.json"))
ap.add_argument("--backend", choices=("auto", "pytorch", "engine", "onnxruntime"), default="auto",
                help="auto: TensorRT-Engine (falls tensorrt importierbar und Datei da) > ONNX Runtime > PyTorch")
ap.add_argument("--aufwaerm-aufloesung", default="848x480")
a = ap.parse_args()

# Compute-Cache gross genug (falls der Torch-Build PTX nachuebersetzen muss; auf diesem Jetson ist sm_53 im Build,
# dann ist es wirkungslos, schadet aber nicht). Muss VOR dem ersten CUDA-Aufruf gesetzt sein.
os.environ.setdefault("CUDA_CACHE_MAXSIZE", "2147483648")
sys.path.insert(0, HIER)
T0 = time.time(); ZUSTAND = {"zustand": "laedt", "seit": T0, "t_bereit_s": None, "pid": os.getpid(), "backend": None, "anfragen": 0}


def status(**kw):
    ZUSTAND.update(kw)
    tmp = a.status + ".tmp"
    with open(tmp, "w") as f: json.dump(ZUSTAND, f)
    os.replace(tmp, a.status)                       # atomar: die GUI liest nie eine halbe Datei


def aufraeumen(*_):
    for p in (a.socket,):
        try: os.remove(p)
        except OSError: pass
    try: status(zustand="beendet")
    except Exception: pass
    os._exit(0)


signal.signal(signal.SIGTERM, aufraeumen); signal.signal(signal.SIGINT, aufraeumen)


def backend_waehlen():
    """-> (Pfad der Gewichte, Name). Engine/ONNX nur, wenn die Laufzeit da ist - nichts installieren (Aufgabe 4)."""
    import yolo_segmentierung as ys
    w = os.path.dirname(ys.GEWICHTE)
    engine, onnx = os.path.join(w, "welle_gross_fp16.engine"), os.path.join(w, "welle_gross.onnx")
    wahl = a.backend
    if wahl in ("auto", "engine") and os.path.exists(engine):
        try:
            import tensorrt  # noqa: F401
            return engine, "engine"
        except ImportError:
            if wahl == "engine": raise
    if wahl in ("auto", "onnxruntime") and os.path.exists(onnx):
        try:
            import onnxruntime  # noqa: F401
            return onnx, "onnxruntime"
        except ImportError:
            if wahl == "onnxruntime": raise
    return ys.GEWICHTE, "pytorch"


def bearbeiten(verb, modell, ys, sperre):
    try:
        puffer = b""
        while not puffer.endswith(b"\n"):
            teil = verb.recv(65536)
            if not teil: return
            puffer += teil
        anf = json.loads(puffer.decode())
        if anf.get("ping"):
            antwort = {"ok": True, **ZUSTAND}
        elif ZUSTAND["zustand"] != "bereit":
            antwort = {"ok": False, "fehler": "nicht bereit (%s)" % ZUSTAND["zustand"]}
        else:
            import numpy as np
            farbe = np.load(anf["bild_npz"])["farbe"]
            with sperre:                                         # eine Inferenz zur Zeit (GPU teilt den Speicher)
                masken, dt = ys.segmentieren(modell, farbe, roi=anf.get("roi"), **ys.STANDARD)
            fd, pfad = tempfile.mkstemp(suffix=".npz", prefix="welle_masken_", dir=ys.AUSTAUSCH); os.close(fd)
            ys.speichern_npz(pfad, masken)
            ZUSTAND["anfragen"] += 1; status()
            antwort = {"ok": True, "masken_npz": pfad, "anzahl": len(masken), "t_inferenz_s": round(dt, 4),
                       "klassen": [m["klasse"] for m in masken], "conf": [round(m["conf"], 3) for m in masken]}
            print("Anfrage %d: %d Masken, %.2f s" % (ZUSTAND["anfragen"], len(masken), dt)); sys.stdout.flush()
    except Exception as e:
        antwort = {"ok": False, "fehler": repr(e)}
    try: verb.sendall((json.dumps(antwort) + "\n").encode())
    except Exception: pass
    verb.close()


def main():
    if os.path.exists(a.socket):
        # laeuft schon einer? Dann nicht doppelt laden (2 x 2 GB passen nicht in 4 GB)
        try:
            s_ = socket.socket(socket.AF_UNIX, socket.SOCK_STREAM); s_.settimeout(2); s_.connect(a.socket); s_.close()
            print("Erkennungsdienst laeuft bereits (%s) - Ende." % a.socket); return 3
        except OSError:
            os.remove(a.socket)                              # Rest eines abgestuerzten Dienstes
    status(zustand="laedt")
    try:
        import yolo_segmentierung as ys
        gewichte, backend = backend_waehlen(); status(backend=backend)
        print("Lade %s (%s) ..." % (gewichte, backend)); sys.stdout.flush()
        modell = ys.lade_modell(gewichte); t_laden = time.time() - T0
        status(zustand="waermt", t_laden_s=round(t_laden, 1))
        W, H = (int(v) for v in a.aufwaerm_aufloesung.split("x"))
        t_w = ys.aufwaermen(modell, (H, W, 3), **ys.STANDARD)
        status(zustand="bereit", t_bereit_s=round(time.time() - T0, 1), t_aufwaermen_s=round(t_w, 1))
        print("bereit nach %.1f s (Laden %.1f s, Aufwaermen %.1f s)" % (time.time() - T0, t_laden, t_w)); sys.stdout.flush()
    except Exception as e:
        status(zustand="fehler", fehler=repr(e)); print("FEHLER: %r" % e); return 1
    srv = socket.socket(socket.AF_UNIX, socket.SOCK_STREAM); srv.bind(a.socket); os.chmod(a.socket, 0o600); srv.listen(4)
    sperre = threading.Lock()
    while True:
        verb, _ = srv.accept()
        threading.Thread(target=bearbeiten, args=(verb, modell, ys, sperre), daemon=True).start()


if __name__ == "__main__":
    sys.exit(main())
