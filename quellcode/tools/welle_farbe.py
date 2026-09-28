#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""Koerper- und STREIFENFARBE einer liegenden Welle aus dem Kamerabild (2026-09-13).

Warum: die Ablageplatte ist 5 Zeilen x 4 Spalten - Zeile = Koerperfarbe, Spalte = Farbe der
zwei parallelen Streifen am Kopf (farbplan.json). Ohne die Streifenfarbe kann der Ablauf die
Welle nur in irgendein Nest der richtigen Zeile stellen (nest_fuer_farbe.py warnt dann).

Wie: Eingabe ist die Wellenmaske aus welle_finden_schale.py (Bildpunkte + Laengsachse +
Kopfende). Der Kopf (13 mm, Ø 10) traegt die Streifen; der Schaft (Ø 7, 22 mm) ist reine
Koerperfarbe.
  1. Koerperfarbe  = Klasse der Mehrheit der Bildpunkte im SCHAFT (weit weg vom Kopf).
  2. Streifen      = Bildpunkte im KOPFBEREICH (--streifen-von .. --streifen-bis mm vom Kopfende),
                     deren Farbe sich deutlich vom Koerper-Median unterscheidet; ihre Mehrheits-
                     klasse ist die Streifenfarbe. Zu wenige (< --min-streifen-px) -> "unbekannt".
Farbklassen: schwarz, gelb, weiss, gruen, rot (die fuenf Farben des Plans; alles andere "?").
Die Zuordnung HSV -> Klasse kommt aus ~/ros2_ws/farbklassen.json (angelernte Referenzen je
Farbe: Median H/S/V), sonst aus festen Schwellen. Anlernen: eine Welle bekannter Farbe in die
Schale, dann
    python3 tools/welle_farbe.py --lernen schwarz            (Koerperfarbe aus dem Schaft)
    python3 tools/welle_farbe.py --lernen rot --streifen     (Streifenfarbe aus dem Kopf)
Pruefen / Entwicklung (Frame aus welle_finden_schale.py --speichern welle.npz):
    python3 tools/welle_farbe.py --npz welle.npz --bild farbe.png      # alle Wellen mit Farben
    python3 tools/welle_farbe.py --npz welle.npz --welle 3 --lernen gruen --streifen
Als Modul: farben_bestimmen(...) -> dict, benutzt von welle_finden_schale.py (welle_ziel.json
bekommt "koerper", "streifen", "farbe_sicher").

Grenzen: 1280x720 aus 61 cm = 0.55 mm/px, Kopf ~24 x 18 px, ein Streifen 1-2 mm = 2-4 px -
darum Mehrheit statt Einzelpixel, und Maske um 1 px erodiert (Randpixel sind Mischfarbe).
Schwarze Streifen auf schwarzem Koerper gibt es nicht (Plan); weiss auf weiss auch nicht.
"""
import argparse, json, math, os, sys
import numpy as np
import cv2

FARBEN = ["schwarz", "gelb", "weiss", "gruen", "rot"]
REFERENZ_DATEI = os.path.expanduser("~/ros2_ws/farbklassen.json")

# Feste Rueckfall-Schwellen (OpenCV-HSV: H 0..179, S/V 0..255). Werden von angelernten
# Referenzen (farbklassen.json) ueberstimmt, sobald die Datei da ist.
SCHWELLEN = {"v_schwarz": 70, "s_weiss": 60, "v_weiss": 140,
             "rot_h": [(0, 12), (160, 180)], "gelb_h": (15, 34), "gruen_h": (34, 95)}   # Grenze gelb/gruen 34 (13.9.)


def klasse_fest(h, s, v):
    if v < SCHWELLEN["v_schwarz"]: return "schwarz"
    if s < SCHWELLEN["s_weiss"]: return "weiss" if v >= SCHWELLEN["v_weiss"] else "schwarz"
    if any(lo <= h < hi for lo, hi in SCHWELLEN["rot_h"]): return "rot"
    if SCHWELLEN["gelb_h"][0] <= h < SCHWELLEN["gelb_h"][1]: return "gelb"
    if SCHWELLEN["gruen_h"][0] <= h < SCHWELLEN["gruen_h"][1]: return "gruen"
    return "?"


def referenzen_laden(pfad=REFERENZ_DATEI):
    """farbklassen.json: {"<farbe>": {"hsv": [h,s,v]}, ..., "streifen": {"<koerper>": {"<farbe>": {"hsv": [..]}}}}"""
    if not os.path.exists(pfad): return {}
    try:
        d = json.load(open(pfad)); return {k: v for k, v in d.items() if not k.startswith("_")}
    except Exception: return {}


def farbabstand(hsv_a, hsv_b):
    """Abstand zweier HSV-Werte: Farbton zirkular, fuer wenig gesaettigte (grau/weiss/schwarz)
    zaehlt der Farbton kaum - dort entscheiden S und V."""
    h1, s1, v1 = hsv_a; h2, s2, v2 = hsv_b
    dh = abs(h1 - h2); dh = min(dh, 180 - dh) / 90.0                      # 0..1
    gew_h = min(s1, s2) / 255.0 * min(v1, v2) / 255.0                       # Farbton nur, wenn beide bunt+hell
    return math.sqrt((3.0 * gew_h * dh) ** 2 + ((s1 - s2) / 255.0) ** 2 + ((v1 - v2) / 255.0) ** 2)


def klasse(hsv, referenzen):
    """Klasse eines HSV-Medians: naechste angelernte Referenz, sonst feste Schwellen."""
    h, s, v = [float(x) for x in hsv]
    refs = {k: v for k, v in (referenzen or {}).items() if isinstance(v, dict) and "hsv" in v}
    if refs:
        best = min(refs.items(), key=lambda kv: farbabstand((h, s, v), kv[1]["hsv"]))
        return best[0], farbabstand((h, s, v), best[1]["hsv"])
    return klasse_fest(h, s, v), None


def farben_bestimmen(farbe_bgr, ys, xs, l_mm, laenge_mm=None, koerper_klasse=None, referenzen=None,
                     ring_von=6.0, ring_bis=26.0, min_ring_mm=2, abstand_min=0.18):
    """Koerper- und Streifenfarbe + KOPFSEITE aus dem Farbprofil entlang der Achse.

    farbe_bgr       Kamerabild (BGR)
    ys, xs          farbige Bildpunkte im Rechteck um die Wellenachse
    l_mm            je Bildpunkt: Lage entlang der Achse [mm], 0 = Ende A, laenge_mm = Ende B
    koerper_klasse  Farbklasse der Maske (welle_finden_schale), sonst aus dem Median
    -> dict: koerper, streifen, sicher, kopf_bei_a (True: Kopf am Ende A; None: keine Ringe),
             ring_mm (Lage der Ringe vom Kopfende), hsv_koerper, hsv_streifen, px_streifen, profil

    Geometrie (Foto 13.9.): Kopf 13 mm einfarbig, dann die ZWEI RINGE der Streifenfarbe bei
    ~9-22 mm vom Kopfende, dann Flansch/Schaft einfarbig. Von der Spitze aus liegen die Ringe bei
    ~21-34 mm. Also: Ringe suchen (1-mm-Abschnitte, deren Farbe vom Koerper-Median abweicht),
    Kopf = das Ende, dem die Ringe naeher sind.
    """
    referenzen = referenzen if referenzen is not None else referenzen_laden()
    ys = np.asarray(ys); xs = np.asarray(xs); l = np.asarray(l_mm, dtype=np.float32)
    L = float(laenge_mm) if laenge_mm else float(l.max())
    out = {"koerper": koerper_klasse or "?", "streifen": "unbekannt", "sicher": False, "kopf_bei_a": None, "ring_mm": None,
           "hsv_koerper": None, "hsv_streifen": None, "px_streifen": 0, "px_streifen_klassen": {}, "profil": []}
    if len(ys) < 20: return out
    hsv = cv2.cvtColor(farbe_bgr, cv2.COLOR_BGR2HSV)[ys, xs].astype(np.float32)
    # Koerper-Median: die Ringe sind ~4 von 43 mm, der Median ueber alles ist also Koerper;
    # Farbton zirkular (rot liegt um 0/180)
    med_k = np.median(hsv, axis=0); ang = np.deg2rad(hsv[:, 0] * 2.0)
    med_k[0] = (math.degrees(math.atan2(np.sin(ang).mean(), np.cos(ang).mean())) / 2.0) % 180.0
    out["hsv_koerper"] = [round(float(x), 1) for x in med_k]
    if not koerper_klasse: out["koerper"], _ = klasse(med_k, referenzen)
    # Profil: je 1 mm Median-HSV und Abstand zum Koerper
    nb = max(3, int(math.ceil(L)) + 1); idx = np.clip(l.astype(int), 0, nb - 1)
    prof = np.zeros(nb); med_b = [None] * nb
    def median_hsv(px_):
        m = np.median(px_, axis=0); ang_ = np.deg2rad(px_[:, 0] * 2.0)          # Farbton ZIRKULAR: rot liegt um 0/180,
        m[0] = (math.degrees(math.atan2(np.sin(ang_).mean(), np.cos(ang_).mean())) / 2.0) % 180.0   # ein linearer Median springt auf 90
        return m
    for b in range(nb):
        sel = idx == b
        if sel.sum() < 3: continue
        m = median_hsv(hsv[sel]); med_b[b] = m; prof[b] = farbabstand(tuple(m), tuple(med_k))
    out["profil"] = [round(float(v), 2) for v in prof]
    # Ringe: Abschnitte mit deutlichem Abstand (absolut UND relativ zum Maximum), nicht ganz am Ende
    # (dort liegt bei beruehrenden Wellen die Nachbarfarbe, bei schwarzen der Schatten)
    # nur dort, wo Ringe liegen KOENNEN (6-26 mm von einem Ende): ganz am Ende liegt bei beruehrenden
    # Wellen die Nachbarfarbe (13.9.: schwarze Welle ueber roter -> "schwarze Ringe bei 4 mm")
    zone = [b for b in range(nb) if ring_von <= b <= ring_bis or ring_von <= (L - b) <= ring_bis]
    kand = np.array([b for b in zone if prof[b] > max(abstand_min, 0.45 * prof[zone].max())])
    if len(kand) < min_ring_mm: return out
    # nur die dichteste Gruppe (Ringpaar ~ 13 mm breit) - Fremdfarbe am anderen Ende ausschliessen
    beste = None
    for b0 in kand:
        grp = kand[(kand >= b0) & (kand <= b0 + 14)]
        if beste is None or len(grp) > len(beste): beste = grp
    kand = beste; mitte = float(kand.mean())
    kopf_bei_a = mitte < L / 2.0
    out["kopf_bei_a"] = bool(kopf_bei_a); out["ring_mm"] = round(mitte if kopf_bei_a else L - mitte, 1)
    # Streifenfarbe: Bildpunkte in den Ring-Abschnitten, die selbst vom Koerper abweichen
    im_ring = np.isin(idx, kand)
    abst = np.array([farbabstand(tuple(p), tuple(med_k)) for p in hsv[im_ring]])
    fremd = abst > max(abstand_min, 0.5 * prof[kand].mean())
    px = hsv[im_ring][fremd]
    out["px_streifen"] = int(len(px))
    if len(px) < 4: return out
    hsv_s = median_hsv(px)
    out["hsv_streifen"] = [round(float(x), 1) for x in hsv_s]
    # Klasse: angelernte Streifen-Referenzen JE KOERPERFARBE (farbklassen.json: "streifen": {koerper: {farbe: hsv}}),
    # sonst feste Schwellen auf den Streifen-Median
    ref_s = (referenzen.get("streifen") or {}).get(out["koerper"]) if isinstance(referenzen, dict) else None
    name, d_ = None, None
    if ref_s:
        name, d_ = min(((n_, farbabstand(tuple(hsv_s), tuple(v_["hsv"]))) for n_, v_ in ref_s.items()), key=lambda kv: kv[1])
        if d_ >= 0.30: name = None                    # keine angelernte Referenz passt -> feste Schwellen
    if name is None:
        # feste Schwellen auf den Ring-Median; Mischfarben duenner Ringe (gruen auf gelb: H 32 statt 26)
        # erkennen die nicht -> dafuer Referenz anlernen (welle_farbe.py --lernen <farbe> --streifen)
        n_ = klasse_fest(*hsv_s)
        if hsv_s[1] < SCHWELLEN["s_weiss"] and out["koerper"] not in ("weiss", "schwarz"):
            # unbunte Ringe auf buntem Koerper: heller als der Koerper = weiss, dunkler = schwarz
            # (weisse Ringe im Schatten haben V ~120 und faellen sonst als "schwarz" durch)
            n_ = "weiss" if hsv_s[2] > med_k[2] + 15 else ("schwarz" if hsv_s[2] < med_k[2] - 15 else "?")
        if n_ in FARBEN and n_ != out["koerper"]: name = n_; d_ = None
    if name: out["streifen"] = name; out["sicher"] = len(px) >= 8
    else: out["streifen"] = "unbekannt"
    out["ring_abstand"] = None if d_ is None else round(float(d_), 3)
    out["px_streifen_klassen"] = {out["streifen"]: int(len(px))}
    return out


def referenz_lernen(name, hsv, quelle, koerper=None, pfad=REFERENZ_DATEI):
    """koerper=None: Koerperfarben-Referenz; sonst Streifen-Referenz fuer diese Koerperfarbe."""
    d = json.load(open(pfad)) if os.path.exists(pfad) else {}
    d["_hinweis"] = ("Angelernte Farbreferenzen (OpenCV-HSV-Median) fuer welle_farbe.py. Koerper: je Farbe eine; "
                     "streifen[koerper][farbe] = Median der Ringpunkte auf dieser Koerperfarbe. Anlernen: welle_farbe.py --lernen <farbe> [--streifen].")
    e = {"hsv": [round(float(x), 1) for x in hsv], "quelle": quelle}
    if koerper: d.setdefault("streifen", {}).setdefault(koerper, {})[name] = e; wo = "streifen[%s][%s]" % (koerper, name)
    else: d[name] = e; wo = name
    json.dump(d, open(pfad, "w"), indent=1); print("  Referenz %s = HSV %s -> %s" % (wo, e["hsv"], pfad))


# ------------------------------------------------------------------ Standalone (Entwicklung / Anlernen)
if __name__ == "__main__":
    ap = argparse.ArgumentParser()
    ap.add_argument("--npz", default="", help="Frame aus welle_finden_schale.py --speichern; leer = Kamera (ruft welle_finden_schale auf)")
    ap.add_argument("--lernen", default="", help="Farbe (schwarz gelb weiss gruen rot): Median der gewaehlten Welle als Referenz speichern")
    ap.add_argument("--streifen", action="store_true", help="mit --lernen: Streifenfarbe (Referenz je Koerperfarbe) statt Koerperfarbe")
    ap.add_argument("--bild", default="", help="Overlay-PNG aller gefundenen Wellen mit Farben")
    ap.add_argument("--welle", type=int, default=1, help="N-te Welle der Liste (Form-Rang), auch wenn nicht greifbar")
    ap.add_argument("--hintergrund", default=REFERENZ_DATEI.replace("farbklassen.json", "schale_hintergrund.npz"))
    a = ap.parse_args()
    if a.lernen and a.lernen not in FARBEN: print("!! Farbe muss eine von %s sein" % " ".join(FARBEN)); sys.exit(2)
    sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
    npz = a.npz
    if not npz:
        # Frame ueber welle_finden_schale.py holen (es sichert ihn), damit hier nur EIN Codepfad existiert
        import subprocess, tempfile
        npz = os.path.join(tempfile.gettempdir(), "welle_farbe_frame.npz")
        cmd = [sys.executable, os.path.join(os.path.dirname(os.path.abspath(__file__)), "welle_finden_schale.py"),
               "--speichern", npz, "--ziel", os.path.join(tempfile.gettempdir(), "welle_farbe_ziel.json"), "--welle", "1"]
        rc = subprocess.call(cmd)
        if not os.path.exists(npz): print("!! welle_finden_schale fehlgeschlagen (%d)" % rc); sys.exit(1)
    import welle_finden_schale as wfs
    d = np.load(npz, allow_pickle=True)
    hg = np.load(a.hintergrund, allow_pickle=True)["farbe"] if os.path.exists(a.hintergrund) else None
    wl = wfs.welle_in_frame(d["farbe"], d["tiefe"], tuple(d["K"]), tuple(d["q"]), np.array(d["t"]), dict(d["schale"].item()),
                            wahl=a.welle, hintergrund=hg)
    if wl is None: print("!! keine Welle im Frame"); sys.exit(1)
    alle = [wl] + wl["weitere"]
    print("Welle %d: Koerper %-8s HSV %s | Streifen %-9s HSV %s (%d px, Ringe %s mm vom Kopf) | %s"
          % (a.welle, wl.get("koerper"), wl.get("hsv_koerper"), wl.get("streifen"), wl.get("hsv_streifen"), wl.get("px_streifen", 0),
             wl.get("ring_mm"), "SICHER" if wl.get("farbe_sicher") else "unsicher"))
    if a.lernen:
        hsv = wl.get("hsv_streifen") if a.streifen else wl.get("hsv_koerper")
        if hsv is None: print("!! nichts zum Lernen (keine %s-Punkte)" % ("Ring" if a.streifen else "Koerper")); sys.exit(1)
        referenz_lernen(a.lernen, hsv, "%s Welle %d" % (os.path.basename(npz), a.welle), koerper=wl.get("koerper") if a.streifen else None)
    if a.bild:
        img = d["farbe"].copy()
        for k, x in enumerate(alle, 1):
            kp, sp = x["kopf_px"], x["spitze_px"]; ok = not x["gruende"]
            cv2.line(img, (int(kp[0]), int(kp[1])), (int(sp[0]), int(sp[1])), (0, 255, 0) if ok else (0, 165, 255), 1)
            cv2.circle(img, (int(kp[0]), int(kp[1])), 3, (0, 0, 255), 1)
            cv2.putText(img, "%d %s/%s%s" % (k, x.get("koerper"), x.get("streifen"), "" if x.get("farbe_sicher") else "?"),
                        (int(x["mu_px"][0]) + 6, int(x["mu_px"][1]) - 6), cv2.FONT_HERSHEY_SIMPLEX, 0.35, (255, 255, 0), 1)
        ys = [int(x["mu_px"][1]) for x in alle]; xs = [int(x["mu_px"][0]) for x in alle]
        crop = img[max(0, min(ys) - 60):max(ys) + 60, max(0, min(xs) - 80):max(xs) + 80]
        cv2.imwrite(a.bild, cv2.resize(crop, None, fx=2, fy=2, interpolation=cv2.INTER_CUBIC)); print("  Overlay: %s" % a.bild)
