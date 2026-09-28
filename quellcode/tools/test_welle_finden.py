#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""Regressionsprobe fuer welle_finden_schale.py an GESICHERTEN Frames (testdaten/*.npz) - ohne Roboter.

Warum (2026-09-19, Benutzer): "Das Programm markiert Kopf, Mitte und Laengsachse der Wellen nicht
zuverlaessig." Damit eine Aenderung an der Bildverarbeitung nicht nur an einem Bild "besser aussieht",
laeuft hier jeder gesicherte Frame durch dieselbe Funktion (welle_in_frame) und es entsteht je Frame
ein MOSAIK: jede gefundene Welle als Ausschnitt, auf die Achse gedreht (Kopf links), mit den Marken
    rot   = Kopfende      blau = Spitze      gruen = Griffpunkt (Flansch)      magenta = Achse
und darunter die Zahlen (Laenge, Breite, Kopf per Ringe/Breite, Farbe, Konfidenz). Ein Blick auf das
Mosaik zeigt, welche Achse schief liegt oder welcher Kopf auf der falschen Seite sitzt.

Sollwerte: testdaten/<frame>.soll.json = {"wellen": [{"kopf_px": [u, v], "spitze_px": [u, v]}, ...]}
(von Hand angeklickt, --annotieren). Sind welche da, wird je Welle der naechste Sollwert gesucht und
Kopfseite (richtig/vertauscht), Achswinkel- und Endpunktfehler gezaehlt -> Trefferquote am Ende.

    python3 tools/test_welle_finden.py                               # alle Frames in testdaten/, Mosaike nach testdaten/mosaik/
    python3 tools/test_welle_finden.py --frames testdaten/wellen_11_kamerapose_2026-09-13.npz --aus /tmp/m
    python3 tools/test_welle_finden.py --annotieren testdaten/wellen_11_kamerapose_2026-09-13.npz   # Sollwerte klicken

Braucht KEIN ROS (welle_finden_schale importiert rclpy nur fuer main/IK-Probe).
"""
import argparse, glob, json, math, os, sys
import cv2
import numpy as np

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
import welle_finden_schale as wfs

ap = argparse.ArgumentParser()
ap.add_argument("--frames", nargs="*", default=None, help="npz-Frames (Vorgabe: testdaten/*.npz ausser Hintergrund)")
ap.add_argument("--hintergrund", default=os.path.expanduser("~/ros2_ws/testdaten/schale_hintergrund_2026-09-13.npz"))
ap.add_argument("--aus", default=os.path.expanduser("~/ros2_ws/testdaten/mosaik"))
ap.add_argument("--annotieren", default="", help="Frame: Kopf und Spitze je Welle anklicken, Sollwerte sichern")
ap.add_argument("--zoom", type=float, default=4.0)
a = ap.parse_args()
TD = os.path.expanduser("~/ros2_ws/testdaten")


def lade(pfad):
    d = np.load(pfad, allow_pickle=True)
    return (d["farbe"], d["tiefe"] if "tiefe" in d else None, tuple(d["K"]), tuple(d["q"]), np.array(d["t"]),
            dict(d["schale"].item()) if "schale" in d else None)


def alle_wellen(farbe, tiefe, K, q, t, schale, hintergrund):
    """welle_in_frame liefert die gewaehlte Welle + 'weitere'; hier alle in Bildreihenfolge."""
    w = wfs.welle_in_frame(farbe, tiefe, K, q, t, schale, drucken=False, hintergrund=hintergrund)
    if w is None or w == "keine_greifbar":
        # keine greifbare: trotzdem alle sehen -> erzwinge die erste
        w = wfs.welle_in_frame(farbe, tiefe, K, q, t, schale, drucken=False, hintergrund=hintergrund, wahl=1)
        if w is None or w == "keine_greifbar": return []
    wellen = [w] + list(w.get("weitere", []))
    return sorted(wellen, key=lambda x: x["i"])


def ausschnitt(farbe, wl, zoom):
    """Welle auf die Achse gedreht (Kopf links), 64 x 26 mm, mit Marken."""
    mm_px = wl["mm_px"]; kopf, spitze, griff = wl["kopf_px"], wl["spitze_px"], None
    mitte = (kopf + spitze) / 2.0; d = spitze - kopf; ang = math.degrees(math.atan2(d[1], d[0]))
    M = cv2.getRotationMatrix2D((float(mitte[0]), float(mitte[1])), ang, zoom)
    W, H = int(64 / mm_px * zoom), int(26 / mm_px * zoom)
    M[0, 2] += W / 2 - mitte[0]; M[1, 2] += H / 2 - mitte[1]
    aus = cv2.warpAffine(farbe, M, (W, H), flags=cv2.INTER_CUBIC)
    tr = lambda p: tuple(int(round(v)) for v in (M @ np.array([p[0], p[1], 1.0])))
    pk, ps = tr(kopf), tr(spitze)
    cv2.line(aus, pk, ps, (255, 0, 255), 1)
    cv2.circle(aus, pk, 5, (0, 0, 255), 2); cv2.circle(aus, ps, 5, (255, 128, 0), 2)
    # Griffpunkt liegt in robot_base; in Bildpunkten: Kopf + GRIFF_AB_KOPF laengs der Achse
    gpx = kopf + d / max(np.linalg.norm(d), 1e-9) * (wfs.GRIFF_AB_KOPF * 1e3 / mm_px)
    cv2.circle(aus, tr(gpx), 4, (0, 255, 0), 2)
    # Sollmarken: Ø-Skala unten (13 mm Kopf | 7 Hals | 1 Flansch | 22 Schaft)
    x0 = pk[0]; y = H - 6; s = zoom / mm_px
    for l0, l1, farbe_ in ((0, 13, (0, 0, 255)), (13, 20, (0, 200, 255)), (20, 21, (0, 255, 0)), (21, 43, (255, 128, 0))):
        cv2.line(aus, (int(x0 + l0 * s), y), (int(x0 + l1 * s), y), farbe_, 3)
    return aus


def annotieren(pfad):
    farbe = lade(pfad)[0]; soll = []; klick = []
    WIN = "Kopf, dann Spitze klicken (n = naechste Welle, u = zurueck, s = sichern, q = Ende)"
    cv2.namedWindow(WIN, cv2.WINDOW_NORMAL)
    def maus(ev, x, y, fl, prm):
        if ev == cv2.EVENT_LBUTTONDOWN: klick.append((x, y))
    cv2.setMouseCallback(WIN, maus)
    while True:
        im = farbe.copy()
        for w in soll:
            cv2.circle(im, tuple(w["kopf_px"]), 5, (0, 0, 255), 2); cv2.circle(im, tuple(w["spitze_px"]), 5, (255, 128, 0), 2)
            cv2.line(im, tuple(w["kopf_px"]), tuple(w["spitze_px"]), (255, 0, 255), 1)
        for p in klick: cv2.circle(im, p, 4, (0, 255, 255), 2)
        cv2.putText(im, "%d Wellen, %d Klick(s)" % (len(soll), len(klick)), (10, 28), cv2.FONT_HERSHEY_SIMPLEX, 0.8, (0, 255, 0), 2)
        cv2.imshow(WIN, im); k = cv2.waitKey(30) & 0xFF
        if len(klick) == 2: soll.append({"kopf_px": list(klick[0]), "spitze_px": list(klick[1])}); klick.clear()
        if k == ord("u") and soll: soll.pop()
        if k == ord("s"):
            out = pfad.replace(".npz", ".soll.json"); json.dump({"wellen": soll}, open(out, "w"), indent=1); print("gesichert:", out)
        if k == ord("q"): break
    cv2.destroyAllWindows()


def main():
    if a.annotieren: annotieren(a.annotieren); return 0
    frames = a.frames or sorted(f for f in glob.glob(os.path.join(TD, "*.npz")) if "hintergrund" not in f)
    hg = lade(a.hintergrund)[0] if os.path.exists(a.hintergrund) else None
    os.makedirs(a.aus, exist_ok=True)
    stat = {"wellen": 0, "kopf_richtig": 0, "kopf_falsch": 0, "achse_grad": [], "kopf_mm": [], "spitze_mm": [], "ohne_soll": 0}
    for pfad in frames:
        farbe, tiefe, K, q, t, schale = lade(pfad)
        if schale is None: print("%s: keine Schale im Frame - uebersprungen" % pfad); continue
        wellen = alle_wellen(farbe, tiefe, K, q, t, schale, hg)
        name = os.path.basename(pfad).replace(".npz", "")
        print("\n%s: %d Welle(n)" % (name, len(wellen)))
        soll = None
        sp = pfad.replace(".npz", ".soll.json")
        if os.path.exists(sp): soll = json.load(open(sp))["wellen"]
        kacheln = []
        for k_, wl in enumerate(wellen, 1):
            aus = ausschnitt(farbe, wl, a.zoom)
            zeile1 = "%d %s/%s  L %.0f B %.0f  %s" % (k_, wl.get("koerper", "?"), wl.get("streifen", "?"), wl["laenge_mm"], wl["breite_mm"], wl["kopf_wie"].split(" (")[0])
            zeile2 = "Konf %.0f %s" % (wl["konfidenz"], ("!! " + wl["gruende"][0][:38]) if wl["gruende"] else "")
            urteil = ""
            if soll:
                mm_px = wl["mm_px"]; best = None
                for s_ in soll:
                    sk, ss = np.array(s_["kopf_px"], float), np.array(s_["spitze_px"], float)
                    dm = np.linalg.norm((sk + ss) / 2 - (wl["kopf_px"] + wl["spitze_px"]) / 2) * mm_px
                    if best is None or dm < best[0]: best = (dm, sk, ss)
                if best and best[0] < 12:
                    dm, sk, ss = best
                    dk = np.linalg.norm(sk - wl["kopf_px"]) * mm_px; ds = np.linalg.norm(ss - wl["spitze_px"]) * mm_px
                    vert = np.linalg.norm(sk - wl["spitze_px"]) * mm_px < dk
                    v_s = ss - sk; v_i = wl["spitze_px"] - wl["kopf_px"]
                    ang = math.degrees(math.acos(np.clip(abs(np.dot(v_s, v_i)) / (np.linalg.norm(v_s) * np.linalg.norm(v_i) + 1e-9), 0, 1)))
                    stat["wellen"] += 1; stat["achse_grad"].append(ang)
                    if vert: stat["kopf_falsch"] += 1; urteil = "KOPF VERTAUSCHT"
                    else: stat["kopf_richtig"] += 1; stat["kopf_mm"].append(dk); stat["spitze_mm"].append(ds); urteil = "Kopf %.1f Spitze %.1f mm, Achse %.1f Grad" % (dk, ds, ang)
                else: stat["ohne_soll"] += 1; urteil = "kein Sollwert"
            print("  Welle %2d: %s | %s | %s" % (k_, zeile1, zeile2, urteil))
            H_ = aus.shape[0] + 44; tafel = np.zeros((H_, aus.shape[1], 3), np.uint8); tafel[:aus.shape[0]] = aus
            cv2.putText(tafel, zeile1, (4, aus.shape[0] + 16), cv2.FONT_HERSHEY_SIMPLEX, 0.45, (255, 255, 255), 1)
            cv2.putText(tafel, (zeile2 + " " + urteil)[:70], (4, aus.shape[0] + 34), cv2.FONT_HERSHEY_SIMPLEX, 0.42,
                        (0, 0, 255) if "VERTAUSCHT" in urteil else (200, 255, 200), 1)
            kacheln.append(tafel)
        if kacheln:
            sp_ = 3; zeilen = [np.hstack(kacheln[i:i + sp_] + [np.zeros_like(kacheln[0])] * (sp_ - len(kacheln[i:i + sp_]))) for i in range(0, len(kacheln), sp_)]
            mos = np.vstack(zeilen); out = os.path.join(a.aus, name + "_mosaik.png"); cv2.imwrite(out, mos); print("  Mosaik:", out)
    if stat["wellen"]:
        print("\nERGEBNIS gegen Sollwerte: %d Wellen, Kopfseite richtig %d / vertauscht %d, Achse median %.1f Grad (max %.1f), "
              "Kopfende median %.1f mm, Spitze median %.1f mm, ohne Sollwert %d"
              % (stat["wellen"], stat["kopf_richtig"], stat["kopf_falsch"], np.median(stat["achse_grad"]), max(stat["achse_grad"]),
                 np.median(stat["kopf_mm"] or [0]), np.median(stat["spitze_mm"] or [0]), stat["ohne_soll"]))
    return 0


if __name__ == "__main__": sys.exit(main())
