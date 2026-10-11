#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""YOLO11-seg-Segmentierung der Wellen als reine Funktion - ohne ROS, ohne Socket (2026-10-11).

Warum eine eigene Datei: dieselbe Segmentierung wird an drei Stellen gebraucht und soll ueberall
GLEICH rechnen, damit der Vergleich klassisch <-> KI fair ist:
  * tools/erkennung_dienst.py      haelt das Modell auf dem Jetson dauerhaft geladen (Zyklus)
  * tools/erkennung_diagnose.py    misst Laden/Aufwaermen/Inferenz je Variante
  * offline_demo (Windows/Linux)   rechnet ohne Dienst direkt im Prozess
Die Geometrie danach (Stab, Kopfseite, Farbe, Konfidenz) macht welle_finden_schale.welle_in_frame -
hier entstehen nur die Instanzmasken.

Modell: src/wellenerkennung/weights/welle_gross.pt (YOLO11l-seg, 5 Klassen = Koerperfarbe, Reihenfolge wie
erkennung.CLASS_NAMES). Schwelle conf 0.70 wie im Detektorknoten (Versuch), damit die Ergebnisse vergleichbar bleiben.
"""
import math, os, time
import numpy as np
import cv2

# Reihenfolge der Klassen im Modell (mit model.names geprueft, s. src/wellenerkennung/wellenerkennung/erkennung.py)
CLASS_NAMES = ["gelb", "weiss", "schwarz", "gruen", "rot"]
HIER = os.path.dirname(os.path.abspath(__file__))
GEWICHTE = os.path.normpath(os.path.join(HIER, "..", "src", "wellenerkennung", "weights", "welle_gross.pt"))
# Offline-Demo/Repository: Gewichte liegen woanders -> Umgebungsvariable WELLE_GEWICHTE (2026-10-11)
GEWICHTE = os.environ.get("WELLE_GEWICHTE", GEWICHTE)
# Laufzeit-Einstellung fuer Dienst, Direktaufruf und Offline-Demo - EINE Stelle. conf 0.70 / imgsz 640 wie der
# Detektorknoten im Versuch. Messung erkennung_diagnose 11.10. (ERKENNUNG_BENCHMARK.md): half=True brachte nichts
# (Median 0.67 statt 0.57 s), der Ausschnitt um die Schale (ROI, setzt welle_finden_schale immer) verdoppelt die
# gefundenen Wellen (10 statt 5 von 11) - die Wellen sind dann so gross wie in den Trainingsbildern (25-40 cm Abstand).
STANDARD = dict(conf=0.70, imgsz=640, half=False, retina=True, max_det=30)
# Schale (Kreisringausschnitt, wie welle_finden_schale.py)
R_INNEN, R_AUSSEN, WINKEL = 0.1869, 0.3019, math.radians(45.3)


def _qrot(q, p):
    x, y, z, w = q
    R = np.array([[1 - 2 * (y * y + z * z), 2 * (x * y - z * w), 2 * (x * z + y * w)],
                  [2 * (x * y + z * w), 1 - 2 * (x * x + z * z), 2 * (y * z - x * w)],
                  [2 * (x * z - y * w), 2 * (y * z + x * w), 1 - 2 * (x * x + y * y)]])
    return p @ R.T


def schale_roi(shape, K, q, t, schale, rand_px=24):
    """Rechteck [x0, y0, x1, y1] um die Schale im Bild (Bodenebene, wie die Schalenmaske in welle_in_frame).
    Warum: das Modell rechnet immer auf 640 px - schneidet man vorher die Schale aus, hat jede Welle mehr
    Bildpunkte und das Hochskalieren der Masken auf die volle Kameraaufloesung entfaellt (2026-10-11)."""
    h, w = shape[:2]; fx, fy, cx, cy = K; cs, yaw_s, z = schale["cs"], schale["yaw"], schale["z_boden"]
    pts = []
    for r_ in (R_INNEN, R_AUSSEN):
        for th in np.linspace(-WINKEL / 2, WINKEL / 2, 25):
            pts.append([cs[0] + r_ * math.cos(yaw_s + th), cs[1] + r_ * math.sin(yaw_s + th), z])
    P = np.array(pts) - np.asarray(t)
    x, y, zz, ww = q; Rinv = np.array([[1 - 2 * (y * y + zz * zz), 2 * (x * y - zz * ww), 2 * (x * zz + y * ww)],
                                       [2 * (x * y + zz * ww), 1 - 2 * (x * x + zz * zz), 2 * (y * zz - x * ww)],
                                       [2 * (x * zz - y * ww), 2 * (y * zz + x * ww), 1 - 2 * (x * x + y * y)]]).T
    Pc = P @ Rinv.T
    u = Pc[:, 0] / Pc[:, 2] * fx + cx; v = Pc[:, 1] / Pc[:, 2] * fy + cy
    x0, x1 = int(max(0, u.min() - rand_px)), int(min(w, u.max() + rand_px))
    y0, y1 = int(max(0, v.min() - rand_px)), int(min(h, v.max() + rand_px))
    if x1 - x0 < 32 or y1 - y0 < 32: return None
    return [x0, y0, x1, y1]


def geraet():
    """'0' (CUDA), wenn vorhanden, sonst 'cpu' - so laeuft dasselbe unter Windows ohne Grafikkarte."""
    try:
        import torch
        return "0" if torch.cuda.is_available() else "cpu"
    except Exception:
        return "cpu"


class GpuModell:
    """Eigener CUDA-Inferenzweg (2026-10-11). WARUM: auf diesem Jetson (torch 1.13 NVIDIA-Build, Ultralytics 8.4.33)
    liefert model.predict(device=0) 0 Detektionen - das von predict gebaute AutoBackend-Modell gibt auf der GPU nur NaN
    aus (206640 NaN im Rohausgang, mit und ohne cuDNN). Ursache: Ultralytics fusioniert Conv+BN auf der CPU, und diese
    Fusion liefert auf dem Jetson NaN-Gewichte. Auf die GPU gelegt und DORT fusioniert rechnet dasselbe Modell richtig
    (Klassenmaximum 0.91 wie unfusioniert), NMS auf der GPU ebenfalls (14 wie CPU).
    Darum hier: LetterBox -> fusioniertes Netz -> Ultralytics-NMS -> process_mask_native, alles wie im Segment-Predictor,
    nur ohne AutoBackend. Jede Ausgabe wird auf NaN geprueft (dann Fehler statt stiller 0 Wellen)."""

    def __init__(s, gewichte):
        import torch
        from ultralytics import YOLO
        y = YOLO(gewichte, task="segment")
        s.names = y.names; s.torch = torch
        # ZUERST auf die GPU, DANN fusionieren: Conv+BN-Fusion auf der Jetson-CPU ergab NaN-Gewichte (gemessen 11.10.:
        # fuse()->cuda() 206640 NaN bei JEDER Eingabe; cuda()->fuse() 0 NaN, Klassenmaximum 0.91 wie unfusioniert)
        s.net = y.model.float().eval().to("cuda").fuse(); s.half = False
        s.stride = int(max(getattr(s.net, "stride", [32])))

    def predict(s, bild, conf=0.70, imgsz=640, half=False, max_det=30, iou=0.7):
        torch = s.torch
        from ultralytics.data.augment import LetterBox
        from ultralytics.utils import ops
        try:
            from ultralytics.utils.nms import non_max_suppression
        except ImportError:                                                      # aeltere Ultralytics
            non_max_suppression = ops.non_max_suppression
        if half != s.half: s.net = s.net.half() if half else s.net.float(); s.half = half
        im = LetterBox((imgsz, imgsz), auto=True, stride=s.stride)(image=bild)
        x = torch.from_numpy(np.ascontiguousarray(im[:, :, ::-1].transpose(2, 0, 1)))[None].to("cuda")
        x = (x.half() if half else x.float()) / 255
        with torch.no_grad():
            aus = s.net(x)
        tens = []
        def sammeln(o):
            if isinstance(o, (list, tuple)):
                for e in o: sammeln(e)
            elif hasattr(o, "dim"): tens.append(o)
        sammeln(aus)
        nc = len(s.names)
        pred = next(t for t in tens if t.dim() == 3 and t.shape[1] > 4 + nc)
        proto = next(t for t in tens if t.dim() == 4)
        if torch.isnan(pred).any(): raise RuntimeError("GPU-Inferenz lieferte NaN")
        det = non_max_suppression(pred.float(), conf_thres=conf, iou_thres=iou, max_det=max_det, nc=nc)[0]
        if not len(det): return [], [], []
        det[:, :4] = ops.scale_boxes(x.shape[2:], det[:, :4], bild.shape)
        masken = ops.process_mask_native(proto[0].float(), det[:, 6:], det[:, :4], bild.shape[:2])
        return masken.cpu().numpy() > 0.5, det[:, 5].cpu().numpy().astype(int), det[:, 4].cpu().numpy()


def lade_modell(gewichte=GEWICHTE, device=None):
    """YOLO-Modell laden. Mit CUDA: eigener Weg (GpuModell, s. dort warum); sonst Ultralytics (CPU, auch Windows)."""
    dev = device if device is not None else geraet()
    # Eigener GPU-Weg nur auf dem Jetson (dort ist predict(device=0) fehlerhaft, s. GpuModell); auf anderen Rechnern
    # (z.B. Windows mit NVIDIA-Karte) der normale Ultralytics-Weg - unabhaengig von Versionsunterschieden der Ultralytics-API
    if dev != "cpu" and gewichte.endswith(".pt") and os.path.exists("/etc/nv_tegra_release"):
        return GpuModell(gewichte)
    from ultralytics import YOLO
    y = YOLO(gewichte, task="segment")
    if gewichte.endswith(".pt"): nan_probe(y)
    return y


def nan_probe(y):
    """CPU-Weg: einmal rechnen und auf NaN pruefen (2026-10-11). Auf dem Jetson lieferte PyTorch auf der CPU bei
    VERSTECKTER GPU (CUDA_VISIBLE_DEVICES=) im ersten Faltungsblock nur NaN -> predict meldete still 0 Wellen.
    Lieber eine klare Meldung als ein falsches 'Schale leer'."""
    import torch
    with torch.no_grad():
        o = y.model.float().eval()(torch.rand(1, 3, 64, 64))
    while isinstance(o, (list, tuple)): o = o[0]
    if torch.isnan(o).any():
        raise RuntimeError("PyTorch rechnet auf dieser CPU NaN (YOLO wuerde 0 Wellen melden). Bekannt auf dem Jetson Nano, "
                           "wenn die GPU versteckt ist (CUDA_VISIBLE_DEVICES=): GPU sichtbar lassen bzw. GPU-Weg verwenden.")


def aufwaermen(modell, shape=(480, 848, 3), **kw):
    """Ein Leerbild in Kameraaufloesung - danach sind CUDA-Kontext, cuDNN und Speicher eingerichtet."""
    t0 = time.perf_counter(); segmentieren(modell, np.zeros(shape, np.uint8), **kw); return time.perf_counter() - t0


def segmentieren(modell, bgr, roi=None, conf=0.70, imgsz=640, half=False, retina=True, max_det=30, device=None):
    """BGR-Bild -> Liste {"maske": bool HxW (volles Bild), "klasse": str, "klasse_id": int, "conf": float}, nach conf
    absteigend, und die Zeit der Inferenz [s].

    retina=True  : Masken von Ultralytics auf die (ROI-)Aufloesung hochgerechnet (wie der Detektorknoten im Versuch)
    retina=False : Masken auf 160x160 bleiben klein; hier wird nur das UMRISSPOLYGON (masks.xy, schon in
                   Bildkoordinaten) gefuellt - spart das Hochskalieren aller Masken auf der CPU.
    roi          : [x0, y0, x1, y1] - nur dieser Ausschnitt geht ins Netz; Masken werden zurueck ins volle Bild gesetzt.
    """
    h, w = bgr.shape[:2]
    x0, y0, x1, y1 = roi if roi is not None else (0, 0, w, h)
    bild = np.ascontiguousarray(bgr[y0:y1, x0:x1])
    t0 = time.perf_counter()
    out = []
    if isinstance(modell, GpuModell):
        # CUDA-Weg: Masken immer in Bildaufloesung (wie retina); retina=False hat hier keine eigene Variante
        mk, ids, confs = modell.predict(bild, conf=conf, imgsz=imgsz, half=half, max_det=max_det)
        for i in range(len(ids)):
            voll = np.zeros((h, w), bool); voll[y0:y1, x0:x1] = mk[i]
            if voll.sum() < 10: continue
            out.append({"maske": voll, "klasse_id": int(ids[i]), "conf": float(confs[i]),
                        "klasse": CLASS_NAMES[ids[i]] if 0 <= ids[i] < len(CLASS_NAMES) else "cls_%d" % ids[i]})
        out.sort(key=lambda d: -d["conf"])
        return out, time.perf_counter() - t0
    res = modell.predict(bild, conf=conf, imgsz=imgsz, half=half, retina_masks=retina, max_det=max_det,
                         verbose=False, device=device if device is not None else geraet())[0]
    if res.masks is not None and len(res.boxes):
        ids = res.boxes.cls.cpu().numpy().astype(int); confs = res.boxes.conf.cpu().numpy()
        if retina:
            daten = res.masks.data.cpu().numpy()
        for i in range(len(ids)):
            voll = np.zeros((h, w), bool)
            if retina:
                m = daten[i]
                if m.shape != bild.shape[:2]: m = cv2.resize(m, (bild.shape[1], bild.shape[0]), interpolation=cv2.INTER_LINEAR)
                voll[y0:y1, x0:x1] = m > 0.5
            else:
                poly = res.masks.xy[i]
                if poly is None or len(poly) < 3: continue
                m = np.zeros(bild.shape[:2], np.uint8)
                cv2.fillPoly(m, [np.round(poly).astype(np.int32)], 1)
                voll[y0:y1, x0:x1] = m > 0
            if voll.sum() < 10: continue
            out.append({"maske": voll, "klasse_id": int(ids[i]),
                        "klasse": CLASS_NAMES[ids[i]] if 0 <= ids[i] < len(CLASS_NAMES) else "cls_%d" % ids[i],
                        "conf": float(confs[i])})
    dt = time.perf_counter() - t0
    out.sort(key=lambda d: -d["conf"])
    return out, dt


# Austauschordner Dienst <-> Aufrufer: /dev/shm (RAM) statt SD-Karte, falls vorhanden (2026-10-11: mit
# savez_compressed auf /tmp kostete die Uebergabe ~3 s je Suche bei 0.74 s Inferenz)
AUSTAUSCH = "/dev/shm" if os.path.isdir("/dev/shm") else None


def speichern_npz(pfad, masken):
    """Masken sichern (Schnittstelle Dienst -> welle_finden_schale), unkomprimiert - Zeit vor Platz."""
    if masken:
        np.savez(pfad, masken=np.stack([m["maske"] for m in masken]).astype(bool),
                            klasse=np.array([m["klasse"] for m in masken]), klasse_id=np.array([m["klasse_id"] for m in masken]),
                            conf=np.array([m["conf"] for m in masken], np.float32))
    else:
        np.savez(pfad, masken=np.zeros((0, 1, 1), bool), klasse=np.array([], dtype="<U8"),
                            klasse_id=np.zeros(0, int), conf=np.zeros(0, np.float32))


def laden_npz(pfad):
    # jedes d["..."] liest das Array NEU aus der Datei -> einmal lesen (11.10.: vorher 13 x ganzes Maskenarray, bis 14 s)
    with np.load(pfad, allow_pickle=False) as d:
        mk, kl, ki, cf = d["masken"], d["klasse"], d["klasse_id"], d["conf"]
    return [{"maske": mk[i], "klasse": str(kl[i]), "klasse_id": int(ki[i]), "conf": float(cf[i])} for i in range(len(cf))]


SOCKET = os.path.join(os.path.dirname(HIER), ".erkennung.sock")
STATUS = os.path.join(os.path.dirname(HIER), ".erkennung_status.json")


def dienst_anfrage(farbe, roi=None, sock_pfad=SOCKET, timeout=120.0):
    """Masken vom Erkennungsdienst (tools/erkennung_dienst.py, nur Linux/Jetson: Unix-Socket).
    -> (Liste wie segmentieren, Inferenzzeit [s]). Fehler (Dienst fehlt/nicht bereit) als RuntimeError."""
    import json, socket, tempfile
    if not os.path.exists(sock_pfad): raise RuntimeError("Dienst laeuft nicht (%s fehlt)" % sock_pfad)
    st = os.path.join(os.path.dirname(sock_pfad), ".erkennung_status.json")
    try:
        zustand = json.load(open(st)).get("zustand")
    except Exception:
        zustand = "?"
    if zustand != "bereit": raise RuntimeError("Dienst nicht bereit (Zustand %s)" % zustand)
    fd, bild = tempfile.mkstemp(suffix=".npz", prefix="welle_bild_", dir=AUSTAUSCH); os.close(fd)
    np.savez(bild, farbe=farbe)
    try:
        s_ = socket.socket(socket.AF_UNIX, socket.SOCK_STREAM); s_.settimeout(timeout); s_.connect(sock_pfad)
        s_.sendall((json.dumps({"bild_npz": bild, "roi": None if roi is None else [int(v) for v in roi]}) + "\n").encode())
        puffer = b""
        while not puffer.endswith(b"\n"):
            teil = s_.recv(65536)
            if not teil: break
            puffer += teil
        s_.close(); antwort = json.loads(puffer.decode())
    finally:
        try: os.remove(bild)
        except OSError: pass
    if not antwort.get("ok"): raise RuntimeError("Dienst meldet Fehler: %s" % antwort.get("fehler"))
    masken = laden_npz(antwort["masken_npz"])
    try: os.remove(antwort["masken_npz"])
    except OSError: pass
    return masken, float(antwort.get("t_inferenz_s", 0.0))
