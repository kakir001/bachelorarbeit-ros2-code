#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""Eine Welle in der Schale im Kamerabild finden - ohne YOLO - und als MODELL in RViz zeigen.

Warum ohne YOLO: der Detektor (YOLO11l-seg) laeuft auf dem Nano ohne Luefter nicht (4 min
Warmup, 80 Grad). Fuer EINE Welle in der leeren, hellgrauen Schale reicht klassische
Bildverarbeitung: alles, was sich in Helligkeit oder Saettigung von der Schale abhebt.

Was bestimmt wird (alles in robot_base, ueber die Kamerakalibrierung):
  * Mitte und LAENGSACHSE (PCA der Maskenpixel, Gierwinkel)
  * KOPF-Ende: die Welle ist am Kopf (13 mm, Ø 10, mit den zwei Streifen) dicker als am
    Schaft (Ø 7) - die Haelfte mit der groesseren mittleren Breite ist der Kopf
  * GRIFFPUNKT = Flansch (Ø 13.5, die dickste Stelle): 20.5 mm vom Kopfende Richtung Spitze
    (Kopf 13 + Hals 7 + halber Flansch; Masse Benutzer 2026-09-09)
  * Hoehe: Schalen-Innenboden (TF `schale`) + halber Flansch (6.75 mm) - NICHT aus der Tiefe
Ausgabe: Terminal, ~/ros2_ws/welle_ziel.json, Overlay (--bild), und in RViz auf
/welle/modell (MarkerArray, latched): Kopf, Hals, Flansch, Schaft als Zylinder in echter
Groesse an der gefundenen Stelle - kein Punkt, ein Modell (Benutzerwunsch 12.9.).

    python3 tools/welle_finden_schale.py --bild welle.png
    python3 tools/hole_aus_schale.py --ziel-datei ~/ros2_ws/welle_ziel.json ...
Exit 0 gewaehlt, 1 Fehler, 2 keine Welle gefunden (Schale leer), 4 Wellen da, aber keine greifbar (Gruende).

Mehrere Wellen (2026-09-13): alle Flecken, die wie eine Welle aussehen, werden vermessen;
GEWAEHLT wird die mit dem besten Formmass, die weit genug von der Schalenwand liegt
(--wand-mm) und im Griffband r_robot --r-min..--r-max - nicht die am Rand (Greifer-Fussabdruck).
Die uebrigen stehen unter "weitere" in welle_ziel.json. --welle N waehlt die N-te der Liste.
KONFIDENZ (2026-09-14, Benutzerwunsch): jede greifbare Welle bekommt 0-100 Punkte, die hoechste wird
genommen: Wand in Schliessrichtung 30 (physisches Minimum ~6 mm -> 0, --wand-quer-gut 30 -> 30), Nachbar in
Schliessrichtung 20 (--nachbar-min = Oeffnung/2 + 3 -> 0, +15 -> 20), Form 20 (Laenge/Breite), Kopfseite 15
(Ringe) / 6 (Breite), Farbe sicher 10, Griffband 5 (r nahe 185). HART (= "gruende", nicht greifbar) sind nur
noch: Laenge +-7, Griffband r_robot (IK), Finger physisch in der Wand / auf der Nachbarwelle. Benutzer 14.9.
Mittag: KEINE 2-cm-Sperre - wandnahe Wellen werden nur nachrangig gewaehlt.
FARBE (2026-09-13): Koerper- und Streifenfarbe je Welle ueber tools/welle_farbe.py
("koerper", "streifen", "farbe_sicher" in welle_ziel.json) -> nest_fuer_farbe.py.
--speichern welle.npz sichert den Frame (Bild, Tiefe, K, TF, Schale) fuer welle_farbe.py --npz.

Grenzen: Wellen duerfen sich nicht beruehren (ein Fleck), Arm nicht ueber der Schale.
Bei 424x240 ist ein Pixel 2 mm - fuer den Versatz reicht das, fuer den Kopf knapp.
"""
import argparse, json, math, os, sys, time
import numpy as np
import cv2
import rclpy
from visualization_msgs.msg import Marker, MarkerArray
from geometry_msgs.msg import Point
from std_msgs.msg import ColorRGBA

sys.argv_backup = list(sys.argv)
ap = argparse.ArgumentParser()
ap.add_argument("--bild", default="")
ap.add_argument("--ausschnitt", default="", help="PNG: Ausschnitt um die GEWAEHLTE Welle (Kopf rot, Mitte gruen, Achse magenta, Farben) - fuer die GUI")
ap.add_argument("--uebersicht", default="", help="PNG: ganzes Kamerabild, NUR die gewaehlte Welle markiert (Kopf, Mittellinie laengs) + "
                                                 "die Sperrzone --wand-quer-min (20 mm) an der Schalenwand rot - zeigt dem Betrachter, warum Wellen am Rand nicht genommen werden")
ap.add_argument("--ziel", default=os.path.expanduser("~/ros2_ws/welle_ziel.json"))
ap.add_argument("--schwelle-hell", type=float, default=22.0, help="|grau - Schalenmedian| ab hier = Welle")
ap.add_argument("--schwelle-sat", type=float, default=70.0, help="HSV-Saettigung ab hier = Welle (farbige Koerper)")
ap.add_argument("--min-px", type=int, default=25)
ap.add_argument("--nur-hell", action="store_true",
                help="nur HELLERE (oder gesaettigte) Bildpunkte als Welle werten - schliesst den Schatten der Welle aus "
                     "(1280x720: Welle + Schatten wurden zu einem 58-mm-Fleck). Nicht fuer schwarze Wellen.")
ap.add_argument("--npz", default="", help="gesicherter Frame (Entwicklung)")
ap.add_argument("--speichern", default="", help="Frame als .npz sichern (Bild, Tiefe, K, TF, Schale) - fuer welle_farbe.py")
ap.add_argument("--welle", type=int, default=0, help="die N-te gefundene Welle nehmen (0 = beste geeignete)")
ap.add_argument("--wand-mm", type=float, default=8.0, help="[mm] Mindestabstand der Wellenmitte zur Schalenwand (irgendeine Richtung)")
ap.add_argument("--oeffnung-mm", type=float, default=17.0,
                help="[mm] Fingeroeffnung (Spitzen) beim Anfahren/Greifen; hole_aus_schale.py liest sie aus welle_ziel.json. "
                     "Benutzer 14.9.: 17 (= Welle 13.5 + 2x1.75), damit die Finger keine Nachbarwelle verschieben; Nachbar-Minimum "
                     "wird daraus 14 mm. 25 = mehr Spielraum fuer die Kamera->Modell-Abweichung (Griffe 13./14.9. liefen mit 40)")
ap.add_argument("--fuss-lang", type=float, default=None, help="[mm] Greifer-Fussabdruck QUER zur Wellenachse; Vorgabe = Oeffnung + 4 (2 mm Finger je Seite)")
ap.add_argument("--fuss-kurz", type=float, default=20.0, help="[mm] Fussabdruck laengs der Achse")
ap.add_argument("--wand-abstand", type=float, default=1.0, help="[mm] Sicherheitsabstand des Fussabdrucks zur Wand (+2 mm Wand); 14.9.: 4 -> 1 (Benutzer: keine 2-cm-Sperre)")
ap.add_argument("--r-min", type=float, default=150.0, help="[mm] Griffband r_robot (Bodengriff, gemessen 12.9.)")
ap.add_argument("--r-max", type=float, default=250.0, help="14.9.: 220 -> 250; ab 235 entscheidet die IK-Probe (--ohne-ik-probe schaltet sie ab)")
ap.add_argument("--ohne-ik-probe", action="store_true", help="keine IK-Probe (senkrechter Griff + Ueber-Punkt per /compute_ik) fuer die Kandidaten")
ap.add_argument("--ik-ueber", type=float, default=60.0, help="[mm] Ueber-Punkt der IK-Probe")
ap.add_argument("--abbildung-datei", default=os.path.expanduser("~/ros2_ws/kamera_modell_platte.json"),
                help="Kamera->Modell (aehnlichkeit s/theta/t) - NUR fuer die r_robot-Probe: das Griffband 150-220 gilt im MODELLRAUM "
                     "(14.9.: Kamera r 215 war hier ok, hole_aus_schale lehnte r 236 im Modell ab)")
ap.add_argument("--wand-quer-min", type=float, default=None,
                help="[mm] HARTE Grenze: Wellenoberflaeche -> Schalenwand in SCHLIESSRICHTUNG. Vorgabe = physisch: Fingerspitze "
                     "(Oeffnung/2 + 2 mm Finger + 2 mm Luft) minus Flanschradius 6.75 -> bei 17 mm Oeffnung 5.8 mm. Alles "
                     "darueber ist erlaubt; der Abstand zaehlt dann nur noch in der KONFIDENZ (30 Punkte bis --wand-quer-gut). "
                     "Benutzer 14.9.: keine 2-cm-Sperre mehr - wandnahe Wellen nur NACHRANGIG waehlen")
ap.add_argument("--wand-quer-gut", type=float, default=30.0, help="[mm] ab hier volle 30 Konfidenzpunkte fuer die Wand")
ap.add_argument("--nachbar-min", type=float, default=3.0,
                help="[mm] HARTE Grenze: Wellenachse -> Nachbaroberflaeche in Schliessrichtung; unter 3 mm liegt die Nachbarwelle "
                     "AUF/AN der Welle (Kreuzung, Beruehrung) -> nicht greifbar. Ab --nachbar-frei (Oeffnung/2 + 3) beruehrt der "
                     "Finger nichts; dazwischen greifbar, aber Konfidenz 0-20 (Benutzer 14.9. Mittag: lieber wandnah/eng als gar nicht)")
ap.add_argument("--nachbar-frei", type=float, default=None, help="[mm] ab hier beruehrt der Finger keinen Nachbarn (Vorgabe Oeffnung/2 + 3)")
ap.add_argument("--ohne-farbe", action="store_true", help="Farben nicht bestimmen")
ap.add_argument("--alte-maske", action="store_true", help="alte Ein-Masken-Suche (hell/dunkel/gesaettigt) statt Farbklassen")
ap.add_argument("--hintergrund", default=os.path.expanduser("~/ros2_ws/schale_hintergrund.npz"),
                help="Referenzbild der LEEREN Schale (Glanzstreifen der Rillen); fehlt es: Suche ohne, mit Warnung")
ap.add_argument("--hintergrund-speichern", action="store_true", help="JETZT (Schale leer!) das Referenzbild aufnehmen und beenden")
ap.add_argument("--ohne-hintergrund", action="store_true", help="Referenzbild ignorieren")
ap.add_argument("--diff-schwelle", type=float, default=28.0, help="max. Kanaldifferenz zum Referenzbild, ab der ein Bildpunkt 'nicht Schale' ist")
ap.add_argument("--streifen-von", type=float, default=1.0, help="[mm] Streifenbereich ab Kopfende")
ap.add_argument("--streifen-bis", type=float, default=13.0)
a = ap.parse_args() if __name__ == "__main__" else ap.parse_args([])
ABB = None
try:
    _ae = json.load(open(a.abbildung_datei))["aehnlichkeit"]
    ABB = (float(_ae["s"]), math.radians(float(_ae["theta_grad"])), np.array(_ae["t_mm"]) / 1e3)
except Exception as _e: print("  !! Abbildung %s nicht lesbar (%s) - r_robot im Kameraraum" % (a.abbildung_datei, _e))
if a.fuss_lang is None: a.fuss_lang = a.oeffnung_mm + 4.0
if a.nachbar_frei is None: a.nachbar_frei = a.oeffnung_mm / 2 + 3.0
# Wand in Schliessrichtung, physisch: Fingerinnenflaeche Oeffnung/2, Keil an der Wandoberkante (~15 mm ueber der Spitze)
# ~9 mm dick, 2 mm Luft, minus Flanschradius 6.75 -> 17 mm Oeffnung: 12.75 mm Oberflaeche->Wand (14.9.: Abfahrt brach bei
# 19 mm im MODELL ab - das Modell zeichnet die Finger ~5 mm je Seite zu weit; letzte 10 mm fahren blind, s. hole_aus_schale)
if a.wand_quer_min is None: a.wand_quer_min = max(0.0, a.oeffnung_mm / 2 + 9.0 + 2.0 - 6.75)
sys.argv = [sys.argv[0]]
sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
import schale_finden as sf                                    # Finder (Kamera + TF), qrot
from schale_finden_kontur import qrot, rot_inv

# Welle (Benutzer 2026-09-09, Kumpass): Kopf 13 (Ø10, Streifen) | Hals 7 (Ø7) | Flansch 1 (Ø13.5) | Schaft 21..22 (Ø7)
KOPF, HALS, FLANSCH, SCHAFT = 0.013, 0.007, 0.001, 0.022
R_KOPF, R_HALS, R_FLANSCH, R_SCHAFT = 0.005, 0.0035, 0.00675, 0.0035
LAENGE = KOPF + HALS + FLANSCH + SCHAFT
GRIFF_AB_KOPF = KOPF + HALS + FLANSCH / 2.0
# Schale
R_INNEN, R_AUSSEN, WINKEL = 0.1869, 0.3019, math.radians(45.3)


def zerteilen(fleck, mm_px):
    """Ein Fleck, der fuer EINE Welle zu lang oder zu breit ist (sich beruehrende Wellen GLEICHER Farbe),
    wird per Distanztransformation + Wasserscheide in Teile zerlegt: Marker = Kerne, die nach
    zunehmender Erosion auseinanderfallen; die Wasserscheide teilt den ganzen Fleck auf die Kerne auf.
    Rueckgabe: Liste boolescher Masken (bei passender Groesse: der Fleck selbst)."""
    ys, xs = np.nonzero(fleck)
    if len(ys) < 40: return [fleck]
    p_ = np.stack([xs, ys], 1).astype(float); mu_ = p_.mean(0)
    _, _, vt_ = np.linalg.svd(p_ - mu_, full_matrices=False)
    lg = (p_ - mu_) @ vt_[0]; qr = (p_ - mu_) @ vt_[1]
    L_ = (lg.max() - lg.min()) * mm_px; B_ = 2 * np.abs(qr).mean() * 1.25 * mm_px
    if L_ <= 1.35 * LAENGE and B_ <= 3.0 * R_KOPF: return [fleck]
    y0, y1, x0, x1 = ys.min(), ys.max() + 1, xs.min(), xs.max() + 1
    sub = fleck[y0:y1, x0:x1].astype(np.uint8)
    dt = cv2.distanceTransform(sub, cv2.DIST_L2, 5)
    teile = None
    for schwelle in (2.0, 3.0, 4.0, 5.0, 6.0):                  # die Erosion mit den MEISTEN Kernen gewinnt
        nk, labk, stk, _ = cv2.connectedComponentsWithStats((dt >= schwelle).astype(np.uint8), 8)
        kerne = [j for j in range(1, nk) if stk[j, 4] >= 40]
        if len(kerne) >= 2 and (teile is None or len(kerne) > len(teile[1])): teile = (labk, kerne)
    if teile is None: return [fleck]
    labk, kerne = teile
    marker = np.zeros(sub.shape, np.int32)
    for k_, j in enumerate(kerne, 2): marker[labk == j] = k_
    marker[sub == 0] = 1                                   # Hintergrund
    bild = cv2.cvtColor(sub * 255, cv2.COLOR_GRAY2BGR)
    cv2.watershed(bild, marker)
    out = []
    for k_ in range(2, len(kerne) + 2):
        t_ = np.zeros(fleck.shape, bool); t_[y0:y1, x0:x1] = (marker == k_) & (sub > 0)
        if t_.sum() >= 40: out.extend(zerteilen(t_, mm_px))          # Teile ggf. weiter zerlegen
    return out if len(out) >= 2 else [fleck]


# Breitenprofil der Welle laengs der Achse [mm], Kopf bei 0: Kopf O10 (13) | Hals O7 (7) | Flansch O13.5 (1) | Schaft O7 (22)
PROFIL_SOLL = np.array([2 * R_KOPF] * 13 + [2 * R_HALS] * 7 + [2 * R_FLANSCH] * 1 + [2 * R_SCHAFT] * 22) * 1e3
PROFIL_SOLL = np.convolve(PROFIL_SOLL, [0.25, 0.5, 0.25], mode="same"); PROFIL_SOLL[0] *= 4 / 3; PROFIL_SOLL[-1] *= 4 / 3


def stab_anpassen(pts, m_px, e1=None):
    """Einen 43-mm-Stab mit dem bekannten Breitenprofil in eine Punktwolke [px] einpassen.

    Warum (2026-09-19): Achse, Enden und Kopfseite kamen bisher aus der PCA ALLER Fleckpunkte und aus
    min/max der Laengsprojektion. Ein Schatten, eine kreuzende oder Kopf-an-Kopf liegende Nachbarwelle
    zog die Achse schief und die Enden auf 47-54 mm (Mosaike testdaten/mosaik, 13.9.-Frames). Hier:
      1. Achse durch GETRIMMTE PCA: nur Punkte innerhalb +-(Kopfradius + 1.5 mm) quer zur Achse,
         dreimal wiederholt - Auslaeufer und Nachbarn fallen heraus.
      2. Breitenprofil in 1-mm-Schritten laengs der Achse (Flaeche je Schritt = Breite).
      3. Das Sollprofil (PROFIL_SOLL, 43 mm) wird in beiden Richtungen entlang geschoben; die Lage mit
         der kleinsten mittleren Abweichung liefert BEIDE Enden und die Kopfseite (Kopf O10 gegen Schaft O7).
         Ist der Fleck laenger als eine Welle, gehoert der Rest einer anderen Welle (-> Aufteilen im Aufrufer).
    m_px: Meter je Bildpunkt. e1: Startachse (sonst PCA).
    -> dict: mu, e1, e2 (Einheitsvektoren), l_a < l_b (Enden als Laengsprojektion ab mu, px), kopf_plus
       (Kopf am Ende l_b), score_mm (mittlere Profilabweichung), sicher (Kopfseite aus dem Profil eindeutig),
       maske (bool je Punkt: gehoert zum eingepassten Stab), n_profil (Profillaenge mm)."""
    mm = m_px * 1e3; mu = pts.mean(0)
    if e1 is None:
        _, _, vt = np.linalg.svd(pts - mu, full_matrices=False); e1 = vt[0]
    r_trim = (R_KOPF * 1e3 + 1.5) / mm
    for _ in range(3):
        e2 = np.array([-e1[1], e1[0]]); qr = (pts - mu) @ e2
        keep = np.abs(qr) <= r_trim
        if keep.sum() < 30: break
        mu_n = pts[keep].mean(0); _, _, vt = np.linalg.svd(pts[keep] - mu_n, full_matrices=False)
        e_n = vt[0] if np.dot(vt[0], e1) >= 0 else -vt[0]
        fertig = abs(np.dot(e_n, e1)) > 0.99995 and np.linalg.norm(mu_n - mu) < 0.3
        e1, mu = e_n, mu_n
        if fertig: break
    e2 = np.array([-e1[1], e1[0]]); lg = (pts - mu) @ e1; qr = (pts - mu) @ e2
    keep = np.abs(qr) <= r_trim
    if keep.sum() < 10:
        return {"mu": mu, "e1": e1, "e2": e2, "l_a": lg.min(), "l_b": lg.max(), "kopf_plus": True, "score_mm": 99.0,
                "sicher": False, "maske": np.ones(len(pts), bool), "n_profil": (lg.max() - lg.min()) * mm}
    l0 = lg[keep].min(); n = int(math.ceil((lg[keep].max() - l0) * mm)) + 1
    bins = np.clip(((lg[keep] - l0) * mm).astype(int), 0, n - 1)
    profil = np.bincount(bins, minlength=n).astype(float) * mm * mm          # Breite [mm] je 1-mm-Schritt
    L = len(PROFIL_SOLL)
    # Fenster [s, s+L) ueber dem Profil (Indexraum mm); ausserhalb des Profils Breite 0. Ueberdeckung mindestens
    # 30 mm (bzw. das ganze kurze Profil), sonst laege die Welle "in der Luft".
    mind = min(30, n); best = None
    for s in range(-(L - mind), n - mind + 1):
        i0, i1 = max(0, s), min(n, s + L)
        w = np.zeros(L); w[i0 - s:i1 - s] = profil[i0:i1]
        d_vor = float(np.abs(w - PROFIL_SOLL).mean()); d_rueck = float(np.abs(w - PROFIL_SOLL[::-1]).mean())
        for d_, plus in ((d_vor, False), (d_rueck, True)):     # PROFIL_SOLL vorwaerts: Kopf bei s = Ende l_a -> kopf_plus False
            if best is None or d_ < best[0]: best = (d_, s, plus, abs(d_vor - d_rueck))
    score, s, kopf_plus, diff = best
    l_a = l0 + s / mm; l_b = l0 + (s + L - 1) / mm
    # Fleck kuerzer als der Stab: Enden nicht ueber die Punkte hinaus legen (verdeckte Welle)
    l_a = max(l_a, lg[keep].min()); l_b = min(l_b, lg[keep].max())
    maske = keep & (lg >= l_a - 1.0 / mm) & (lg <= l_b + 1.0 / mm)
    return {"mu": mu, "e1": e1, "e2": e2, "l_a": l_a, "l_b": l_b, "kopf_plus": bool(kopf_plus), "score_mm": score,
            "sicher": diff > 0.6, "maske": maske, "n_profil": n}


def welle_in_frame(farbe, tiefe, K, q, t, schale, nur_hell=False, schwelle_hell=None, schwelle_sat=None, min_px=None,
                   wand_mm=None, r_min=None, r_max=None, wahl=0, farben=True, streifen_von=None, streifen_bis=None,
                   drucken=True, alte_maske=False, hintergrund=None, diff_schwelle=None):
    """Alle Wellen im Frame vermessen, eine waehlen. Reine Bildverarbeitung, kein ROS.

    schale = {"cs": (x, y), "yaw": yaw_s, "z_boden": z}   (robot_base, Meter/rad)
    -> dict der gewaehlten Welle (Bildpunkte ys/xs, d_kopf_mm je Punkt, mm_px, Kopf/Spitze/Griff in
       robot_base, yaw, Laenge, Farben, "weitere": Liste der anderen) oder None.
    """
    schwelle_hell = a.schwelle_hell if schwelle_hell is None else schwelle_hell
    schwelle_sat = a.schwelle_sat if schwelle_sat is None else schwelle_sat
    min_px = a.min_px if min_px is None else min_px
    wand_mm = a.wand_mm if wand_mm is None else wand_mm
    r_min = a.r_min if r_min is None else r_min; r_max = a.r_max if r_max is None else r_max
    fuss_lang, fuss_kurz, wand_abstand = a.fuss_lang / 1e3, a.fuss_kurz / 1e3, a.wand_abstand / 1e3
    streifen_von = a.streifen_von if streifen_von is None else streifen_von
    streifen_bis = a.streifen_bis if streifen_bis is None else streifen_bis
    say = print if drucken else (lambda *x, **k: None)
    cs, yaw_s, z_boden = schale["cs"], schale["yaw"], schale["z_boden"]
    fx, fy, cx, cy = K; h, w = farbe.shape[:2]

    # Bildpunkte -> Ebene Innenboden -> Schalenmaske (Sektor innen, 8 mm Rand)
    vs, us = np.mgrid[0:h, 0:w]
    strahl = qrot(q, np.stack([((us - cx) / fx).ravel(), ((vs - cy) / fy).ravel(), np.ones(h * w)], 1))
    P = (t + strahl * ((z_boden - t[2]) / strahl[:, 2])[:, None])[:, :2]
    dx, dy = P[:, 0] - cs[0], P[:, 1] - cs[1]; r = np.hypot(dx, dy)
    th = (np.arctan2(dy, dx) - yaw_s + math.pi) % (2 * math.pi) - math.pi
    innen = ((r > R_INNEN + 0.008) & (r < R_AUSSEN - 0.008) & (np.abs(th) < WINKEL / 2 - 0.05)).reshape(h, w)
    if innen.sum() < 200: say("ABBRUCH: Schale kaum im Bild (%d px)" % innen.sum()); return None

    # Arm ueber der Schale? Dann ist NICHTS zu machen: (1) in der Nullstellung schattet der Arm den
    # IR-Projektor ab (Tiefe = NaN), (2) die zum Farbbild ausgerichtete Tiefe traegt am Rand des
    # nahen Greifers dessen Tiefe (200 mm) in Bildpunkte, die in der Farbe die Welle zeigen
    # (Parallaxe) - eine Tiefen-Sperre wuerde also die WELLE loeschen, nicht den Greifer
    # (2026-09-12 gemessen). Deshalb hier nur WARNEN; die Regel ist: Arm in die Kamerapose
    # (nest_11 +40), dann suchen.
    arm_im_bild = False
    if tiefe is not None and tiefe.shape[:2] == farbe.shape[:2]:
        tb = (t[2] - z_boden) * 1e3
        nah = np.isfinite(tiefe[innen]) & (np.abs(tiefe[innen] - tb) > 100.0)
        if np.isnan(tiefe[innen]).mean() > 0.3 or nah.mean() > 0.3:
            arm_im_bild = True
            say("  !! Arm/Greifer ueber der Schale (%.0f %% ohne Tiefe, %.0f %% mit fremder Tiefe) - "
                "Ergebnis unzuverlaessig, Arm erst in die Kamerapose (nest_11 +40)!"
                % (100 * np.isnan(tiefe[innen]).mean(), 100 * nah.mean()))
    grau = cv2.cvtColor(farbe, cv2.COLOR_BGR2GRAY).astype(np.float32)
    hsv = cv2.cvtColor(farbe, cv2.COLOR_BGR2HSV)
    med = float(np.median(grau[innen]))
    mm_px = (t[2] - z_boden) / fx                      # Meter je Bildpunkt auf Bodenhoehe
    # Maske je KOERPERFARBE (2026-09-13 Abend): eine gemeinsame "alles, was nicht Schale ist"-Maske
    # verschmolz sich beruehrende Wellen UND ihre Schatten zu einem Klumpen (11 Wellen -> 4 Flecken,
    # keiner mit Wellenlaenge). Je Farbklasse getrennt (rot/gelb/gruen/weiss/schwarz, Schatten ist
    # keine Klasse) bleiben Nachbarn verschiedener Farbe getrennt; CLOSE 7x7 ueberbrueckt die 2-4 px
    # breiten Streifen der ANDEREN Farbe, so dass Kopf und Schaft ein Fleck bleiben. Zwei sich
    # beruehrende Wellen GLEICHER Farbe bleiben ein Fleck (Laenge passt nicht -> abgelehnt; nach dem
    # naechsten Griff neu suchen).
    H_, S_, V_ = (hsv[:, :, i].astype(np.int32) for i in range(3))
    klassen = {"rot": ((H_ < 12) | (H_ >= 160)) & (S_ > 90),
               "gelb": (H_ >= 15) & (H_ < 34) & (S_ > 90),      # gelber Koerper H 26-30; gruener (oliv) 38-47 -> Grenze 34 (13.9.)
               "gruen": (H_ >= 34) & (H_ < 95) & (S_ > 60),
               "weiss": (S_ < 60) & (grau > med + schwelle_hell),
               "schwarz": V_ < 70}
    # Referenzbild der LEEREN Schale (13.9. Abend): die Glanzstreifen der polierten Rillen sind so hell
    # und unbunt wie eine weisse Welle und verbanden fuenf weisse Wellen zu einem Fleck. Was sich vom
    # Referenzbild nicht unterscheidet, ist Schale - egal wie hell. Aufnehmen: --hintergrund-speichern.
    anders = innen
    diff_schwelle = a.diff_schwelle if diff_schwelle is None else diff_schwelle
    if hintergrund is not None and hintergrund.shape == farbe.shape:
        diff = np.abs(farbe.astype(np.int16) - hintergrund.astype(np.int16)).max(axis=2)
        anders = innen & (diff > diff_schwelle)
        say("  Referenzbild: %d px in der Schale weichen ab (%.0f %%)" % (anders.sum(), 100.0 * anders.sum() / max(1, innen.sum())))
    kl_map = np.zeros(grau.shape, np.uint8); KL_NAMEN = ["-"] + list(klassen)
    for k_, (name, mk_) in enumerate(klassen.items(), 1): kl_map[mk_ & anders & (kl_map == 0)] = k_

    def klasse_px(ys_, xs_):
        """Koerperfarbe eines Punkthaufens = Mehrheit der klassifizierten Punkte (Ringe sind die Minderheit)."""
        z = np.bincount(kl_map[ys_, xs_], minlength=len(KL_NAMEN)); z[0] = 0
        return KL_NAMEN[int(z.argmax())] if z.sum() >= 10 else "?"
    if hintergrund is not None and not alte_maske:
        # MIT Referenzbild (bevorzugt): Objekte = alles, was vom leeren Bild abweicht - so bleibt jede
        # Welle MIT ihren Ringen ein Fleck (die Ringe haben eine andere Klasse als der Koerper). Klasse =
        # Mehrheit der klassifizierten Punkte. Zu grosse Flecken (beruehrende Wellen): erst nach Klassen
        # trennen (verschiedene Farben), was dann noch zu gross ist per Wasserscheide (gleiche Farbe).
        m = anders.astype(np.uint8)
        m = cv2.morphologyEx(m, cv2.MORPH_OPEN, np.ones((2, 2), np.uint8))
        m = cv2.morphologyEx(m, cv2.MORPH_CLOSE, np.ones((5, 5), np.uint8))
        n_, lab_, st_, _ = cv2.connectedComponentsWithStats(m, 8)
        lab = np.zeros(grau.shape, np.int32); stats = [np.zeros(5)]; lab_klasse = {0: "-"}; n = 1

        def klasse_von(mk_):
            z = np.bincount(kl_map[mk_], minlength=len(KL_NAMEN)); z[0] = 0
            return KL_NAMEN[int(z.argmax())] if z.sum() >= 10 else "?"

        def eintragen(mk_, name):
            nonlocal n
            lab[mk_ & (lab == 0)] = n; stats.append(np.array([0, 0, 0, 0, int(mk_.sum())])); lab_klasse[n] = name; n += 1

        def zu_gross(mk_):
            """False = passt; "lang" = zu lang, aber nicht breit (Wellen Kopf an Kopf / kreuzend -> spaeter per Stab
            abschaelen, NICHT nach Farben trennen: die Trennung zerlegte eine weisse Welle an ihren schwarzen Ringen,
            19.9.); "breit" = zu breit (nebeneinander) -> nach Farbklassen trennen."""
            ys_, xs_ = np.nonzero(mk_)
            if len(ys_) < 40: return False
            p_ = np.stack([xs_, ys_], 1).astype(float); mu_ = p_.mean(0)
            _, _, vt_ = np.linalg.svd(p_ - mu_, full_matrices=False)
            lg = (p_ - mu_) @ vt_[0]; qr = (p_ - mu_) @ vt_[1]
            breit = 2 * np.abs(qr).mean() * 1.25 * mm_px > 3.0 * R_KOPF
            lang = (lg.max() - lg.min()) * mm_px > 1.35 * LAENGE
            return "breit" if breit else ("lang" if lang else False)

        for j in range(1, n_):
            if st_[j, 4] < min_px: continue
            obj = lab_ == j
            if zu_gross(obj) != "breit": eintragen(obj, klasse_von(obj)); continue
            # 19.9.: auch beim BREITEN Fleck zuerst farbunabhaengig einen Stab einpassen (kreuzende Wellen verschiedener
            # Farbe): passt das Profil (Fehler < 2.5 mm), sind Stab und Rest eigene Flecken - die Farbtrennung darunter
            # zerschnitt sonst die weisse Welle an ihren schwarzen Ringen (11er-Frame 13.9., Wellen 8/9 je 25 mm).
            ys_o, xs_o = np.nonzero(obj); fit_o = stab_anpassen(np.stack([xs_o, ys_o], 1).astype(float), mm_px)
            if fit_o["score_mm"] < 2.5 and fit_o["maske"].sum() >= min_px and (~fit_o["maske"]).sum() >= min_px:
                for teil in (fit_o["maske"], ~fit_o["maske"]):
                    mk_t = np.zeros(obj.shape, bool); mk_t[ys_o[teil], xs_o[teil]] = True
                    eintragen(mk_t, klasse_px(ys_o[teil], xs_o[teil]))
                continue
            # nach Farbklassen trennen; Ringe/unklassifizierte Punkte bekommt der Teil, dessen CLOSE sie einschliesst
            teile = []
            for k_, name in enumerate(KL_NAMEN[1:], 1):
                mk = ((kl_map == k_) & obj).astype(np.uint8)
                if mk.sum() < min_px: continue
                mk = cv2.morphologyEx(mk, cv2.MORPH_CLOSE, np.ones((9, 9), np.uint8)) & obj.astype(np.uint8)
                nn, ll, ss, _ = cv2.connectedComponentsWithStats(mk, 8)
                for jj in range(1, nn):
                    if ss[jj, 4] < min_px: continue
                    for t_ in zerteilen(ll == jj, mm_px): teile.append((int(t_.sum()), t_, name))
            for groesse, t_, name in sorted(teile, key=lambda f_: -f_[0]):
                frei = t_ & (lab == 0)
                if frei.sum() >= min_px and frei.sum() >= 0.5 * groesse: eintragen(frei, name)
        stats = np.array(stats)
    elif alte_maske:
        if nur_hell: m = (((grau - med) > schwelle_hell) | (S_ > schwelle_sat)) & innen
        else: m = ((np.abs(grau - med) > schwelle_hell) | (S_ > schwelle_sat)) & innen
        m = cv2.morphologyEx(m.astype(np.uint8), cv2.MORPH_OPEN, np.ones((2, 2), np.uint8))
        m = cv2.morphologyEx(m, cv2.MORPH_CLOSE, np.ones((5, 5), np.uint8))
        n, lab, stats, _ = cv2.connectedComponentsWithStats(m, 8); lab_klasse = {j: "?" for j in range(n)}
    else:
        # Erst alle Flecken aller Klassen sammeln, dann die GROSSEN zuerst eintragen: die Streifen
        # (kleine Flecken der anderen Farbe) liegen INNERHALB des grossen Wellenflecks und gehoeren
        # zu ihm (Kopfseite, Streifenfarbe) - truegen sie zuerst ein, fehlten sie der Welle.
        lab = np.zeros(grau.shape, np.int32); stats = [np.zeros(5)]; lab_klasse = {0: "-"}; n = 1; flecken = []
        for k_, name in enumerate(KL_NAMEN[1:], 1):
            m = (kl_map == k_).astype(np.uint8)
            m = cv2.morphologyEx(m, cv2.MORPH_OPEN, np.ones((2, 2), np.uint8))
            m = cv2.morphologyEx(m, cv2.MORPH_CLOSE, np.ones((7, 7), np.uint8))
            if name in ("weiss", "schwarz") and hintergrund is None:
                # NUR OHNE Referenzbild (mit Referenzbild sind die Glanzstreifen schon weg, und die Kerne
                # zerlegen eine Welle an den Ringen in Kopf- und Schaftstueck):
                # Glanzlinien der Schalenrillen (hell) und Schattenkanten (dunkel) sind 1-3 px duenn und
                # verbanden weisse Wellen zu 64-mm-Flecken (13.9.). Ein staerkeres OPEN zerreisst aber die
                # Welle selbst (ihre schattigen Raender sind schmal). Darum: KERNE = OPEN 5x5 (nur dicke
                # Teile), je Kern die Welle durch begrenztes Wachsen (3 px) in der feinen Maske zurueckholen.
                kern = cv2.morphologyEx(m, cv2.MORPH_OPEN, np.ones((5, 5), np.uint8))
                nk, labk, _, _ = cv2.connectedComponentsWithStats(kern, 8)
                for j in range(1, nk):
                    kj = (labk == j).astype(np.uint8)
                    if kj.sum() < min_px: continue
                    reg = cv2.dilate(kj, np.ones((7, 7), np.uint8)) & m
                    n_, lab_, st_, _ = cv2.connectedComponentsWithStats(reg, 8)
                    jj = 1 + int(np.argmax(st_[1:, 4])) if n_ > 1 else 0
                    if jj: flecken.append((int(st_[jj, 4]), name, lab_ == jj, st_[jj].copy()))
                continue
            n_, lab_, st_, _ = cv2.connectedComponentsWithStats(m, 8)
            for j in range(1, n_):
                if st_[j, 4] < min_px: continue
                for teil in zerteilen(lab_ == j, mm_px):
                    st = np.array([0, 0, 0, 0, int(teil.sum())]); flecken.append((int(teil.sum()), name, teil, st))
        for groesse, name, mk_, st in sorted(flecken, key=lambda f_: -f_[0]):
            sel = mk_ & (lab == 0)
            if sel.sum() < min_px or sel.sum() < 0.5 * groesse: continue     # schon (als Streifen) vergeben
            lab[sel] = n; st[4] = sel.sum(); stats.append(st); lab_klasse[n] = name; n += 1
        stats = np.array(stats)
    if n < 2 or stats[1:, 4].max() < min_px:
        say("ABBRUCH: keine Welle in der Schale gefunden (Schalenmedian %.0f)" % med); return None
    # Nicht der GROESSTE Fleck, sondern die, die wie eine Welle aussehen: Laenge nahe 43 mm,
    # Breite nahe 10 mm (2026-09-12: der groesste Fleck war der Schattenstreifen an der Wand).
    kand = []
    stats = [np.array(r_) for r_ in stats]                     # Liste: Aufteilungen haengen neue Flecken an
    warte = list(range(1, n)); geschaelt = {}
    while warte:
        j = warte.pop(0)
        if stats[j][4] < min_px: continue
        ys_, xs_ = np.nonzero(lab == j); p_ = np.stack([xs_, ys_], 1).astype(float); mu_ = p_.mean(0)
        _, sv, vt_ = np.linalg.svd(p_ - mu_, full_matrices=False)
        lg = (p_ - mu_) @ vt_[0]; qr = (p_ - mu_) @ vt_[1]
        L_ = (lg.max() - lg.min()) * mm_px; B_ = 2 * np.abs(qr).mean() * 1.25 * mm_px
        # 14.9.: zwei Wellen GLEICHER Farbe dicht NEBENEINANDER verschmelzen zu einem 17-21 mm breiten Fleck (die
        # Wasserscheide trennt nur an Einschnuerungen). Ist der Fleck deutlich breiter als eine Welle (> 14 mm) und
        # nicht zu lang, wird er QUER zur Achse in zwei Haelften geteilt (1-D-2-Means auf der Querkoordinate) und
        # beide Haelften laufen als eigene Flecken weiter. Sonst laege der Griffpunkt ZWISCHEN den beiden Wellen.
        # 19.9.: Fleck deutlich LAENGER als eine Welle (Kopf an Kopf, kreuzend, Schattenschweif): den 43-mm-Stab mit dem
        # Breitenprofil einpassen (stab_anpassen) und ABSCHAELEN - Stab und Rest laufen als eigene Flecken weiter (der Rest
        # kann wieder eine Welle sein). Vorher wurde ein 47-54-mm-Fleck als EINE Welle mit falschen Enden vermessen.
        if L_ > LAENGE + 0.007 and len(lg) >= 2 * min_px and geschaelt.get(j, 0) < 3:
            fit = stab_anpassen(p_, mm_px)
            innen = fit["maske"]; rest = ~innen
            if innen.sum() >= min_px and rest.sum() >= min_px:
                say("  Fleck %d (%s): %.0f mm lang -> Stab (%.0f px, Profilfehler %.1f mm) abgeschaelt, Rest %d px"
                    % (j, lab_klasse[j], L_ * 1e3, innen.sum(), fit["score_mm"], rest.sum()))
                for teil in (innen, rest):
                    nn_ = len(stats); lab[ys_[teil], xs_[teil]] = nn_
                    stats.append(np.array([0, 0, 0, 0, int(teil.sum())])); lab_klasse[nn_] = klasse_px(ys_[teil], xs_[teil])
                    geschaelt[nn_] = geschaelt.get(j, 0) + 1; warte.append(nn_)
                continue
        if 1.4 * 2 * R_KOPF < B_ < 3.2 * 2 * R_KOPF and L_ < LAENGE + 0.015 and len(qr) >= 2 * min_px:
            c0, c1 = np.percentile(qr, 25), np.percentile(qr, 75)
            for _ in range(10):
                zu1 = np.abs(qr - c1) < np.abs(qr - c0)
                if zu1.all() or (~zu1).all(): break
                c0, c1 = qr[~zu1].mean(), qr[zu1].mean()
            if min(zu1.sum(), (~zu1).sum()) >= min_px and abs(c1 - c0) * mm_px > 0.006:
                say("  Fleck %d (%s): %.0f mm breit -> in zwei Wellen geteilt (Querabstand %.1f mm)" % (j, lab_klasse[j], B_ * 1e3, abs(c1 - c0) * mm_px * 1e3))
                for teil in (~zu1, zu1):
                    nn_ = len(stats); lab[ys_[teil], xs_[teil]] = nn_
                    stats.append(np.array([0, 0, 0, 0, int(teil.sum())])); lab_klasse[nn_] = lab_klasse[j]; warte.append(nn_)
                continue
        score = abs(L_ - LAENGE) + 0.5 * abs(B_ - 2 * R_KOPF)
        kand.append((score, j, L_, B_))
    stats = np.array(stats)
    kand.sort()
    for sc, j, L_, B_ in kand[:20]:
        say("  Fleck %d (%s): %d px, Laenge %.0f mm, Breite %.0f mm, Score %.3f" % (j, lab_klasse[j], stats[j, 4], L_ * 1e3, B_ * 1e3, sc))

    z_mitte = z_boden + R_FLANSCH
    def base(px):
        s = qrot(q, np.array([[(px[0] - cx) / fx, (px[1] - cy) / fy, 1.0]]))
        return (t + s * ((z_mitte - t[2]) / s[:, 2])[:, None])[0, :2]

    def vermessen(i):
        """Eine Komponente -> Welle (Geometrie in robot_base) oder None (Form passt nicht)."""
        ys, xs = np.nonzero(lab == i)
        pts = np.stack([xs, ys], 1).astype(float)
        # 19.9.: Achse, Enden und Breite aus dem eingepassten Stab (getrimmte PCA + Breitenprofil), nicht mehr aus der
        # PCA aller Punkte und min/max - s. stab_anpassen. Punkte ausserhalb des Stabs (Schatten, Nachbar) zaehlen nicht.
        fit = stab_anpassen(pts, mm_px); mu, e1, e2 = fit["mu"], fit["e1"], fit["e2"]
        laengs = (pts - mu) @ e1; quer = (pts - mu) @ e2
        stab = fit["maske"]; l_a, l_b = fit["l_a"], fit["l_b"]
        # Kopf + Farben ueber das FARBPROFIL entlang der Achse (tools/welle_farbe.py): die zwei Ringe der
        # Streifenfarbe liegen ~9-22 mm vom Kopfende (Foto 13.9.), von der Spitze aus ~21-34 mm -> das
        # Ende, dem die Ringe naeher sind, ist der Kopf. Bildpunkte dafuer NICHT nur aus dem Fleck, sondern
        # aus dem Rechteck um die Achse (Laenge x 0.8 Breite): beruehrt eine Welle anderer Farbe, gehoeren
        # die Ringe sonst zu DEREN Fleck (weisse Welle mit roten Ringen neben roter Welle -> 0 Ring-px).
        # Ohne Ringe (Farben aus): die dickere Haelfte ist der Kopf (Ø 10 gegen Ø 7).
        eigene = KL_NAMEN.index(lab_klasse[i]) if lab_klasse[i] in KL_NAMEN else 0
        laengs_c, quer_c = laengs[stab], quer[stab]                         # nur der Stab (Geometrie, Breite)
        L_px = l_b - l_a
        x0, x1 = max(0, int(xs.min()) - 3), min(w - 1, int(xs.max()) + 3); y0, y1 = max(0, int(ys.min()) - 3), min(h - 1, int(ys.max()) + 3)
        yy, xx = np.mgrid[y0:y1 + 1, x0:x1 + 1]; pp = np.stack([xx.ravel(), yy.ravel()], 1).astype(float) - mu
        lg_, qr_ = pp @ e1, pp @ e2
        halbbreite = 0.8 * np.abs(quer_c).mean() * 1.25
        im_r = (lg_ >= l_a) & (lg_ <= l_b) & (np.abs(qr_) <= halbbreite)
        flach = np.ravel_multi_index((yy.ravel(), xx.ravel()), kl_map.shape)
        if hintergrund is not None: im_r &= anders.ravel()[flach]                          # Schale nicht (Ringe jeder Farbe bleiben)
        elif eigene > 0: im_r &= (kl_map.ravel()[flach] != 0)                             # Schatten/Schale nicht
        ys_r, xs_r, laengs_r = yy.ravel()[im_r], xx.ravel()[im_r], lg_[im_r]
        l_mm = (laengs_r - l_a) * mm_px * 1e3                               # 0 = Ende A (l_a)
        fb = None
        if farben:
            try:
                import welle_farbe
                fb = welle_farbe.farben_bestimmen(farbe, ys_r, xs_r, l_mm, L_px * mm_px * 1e3,
                                                  koerper_klasse=lab_klasse[i] if eigene > 0 else None)
            except Exception as e:
                fb = {"koerper": lab_klasse[i], "streifen": "unbekannt", "sicher": False, "kopf_bei_a": None, "fehler": str(e)}
        # Kopfseite: Ringe (welle_farbe) zuerst; ohne Ringe das Breitenprofil des Stabs (Kopf O10 gegen Schaft O7).
        # Widersprechen sich beide und ist das Profil eindeutig, gilt das Profil (Ringe einer NACHBARWELLE im Rechteck).
        if fb and fb.get("kopf_bei_a") is not None:
            kopf_plus = not fb["kopf_bei_a"]; kopf_wie = "Ringe (%.0f mm vom Kopfende, %d px)" % (fb["ring_mm"], fb["px_streifen"])
            if fit["sicher"] and fit["kopf_plus"] != kopf_plus and fit["score_mm"] < 1.5:
                kopf_plus = fit["kopf_plus"]; kopf_wie = "Profil (Ringe widersprachen, Profilfehler %.1f mm)" % fit["score_mm"]
        else:
            kopf_plus = fit["kopf_plus"]; kopf_wie = "Profil (%.1f mm Fehler%s)" % (fit["score_mm"], "" if fit["sicher"] else ", unsicher")
        kopf_px = mu + e1 * (l_b if kopf_plus else l_a)
        spitze_px = mu + e1 * (l_a if kopf_plus else l_b)
        ys, xs = ys_r, xs_r
        d_kopf_mm = ((l_b - laengs_r) if kopf_plus else (laengs_r - l_a)) * mm_px * 1e3
        c_b, kopf_b, spitze_b = base(mu), base(kopf_px), base(spitze_px)
        achse = spitze_b - kopf_b; L = float(np.linalg.norm(achse)); achse = achse / max(L, 1e-9)   # Kopf -> Spitze
        yaw = math.atan2(achse[1], achse[0])
        # GRIFFPUNKT (14.9., Benutzer: "aus der MITTE greifen, Kopfrichtung nur fuer die Orientierung"): Mittelpunkt
        # der beiden Enden, 1 mm zum Kopf = Flanschmitte (Kopf 13 + Hals 7 + 0.5 = 20.5 von 43). Vorher kopf + 20.5:
        # der Kopfpunkt allein streut (Schatten, Ringe, Nachbar) - die Mitte mittelt beide Enden.
        griff_b = (kopf_b + spitze_b) / 2.0 - achse * (L / 2.0 - GRIFF_AB_KOPF)
        breite_mm = 2 * np.abs(quer_c).mean() * 1.25 * mm_px * 1e3
        # Lage in der Schale: Abstand zur Wand (radial und zu den Sektorflanken), Griffband
        ddx, ddy = c_b[0] - cs[0], c_b[1] - cs[1]; rr = math.hypot(ddx, ddy)
        tt = (math.atan2(ddy, ddx) - yaw_s + math.pi) % (2 * math.pi) - math.pi
        wand = min(rr - R_INNEN, R_AUSSEN - rr, rr * math.sin(max(0.0, WINKEL / 2 - abs(tt)))) * 1e3
        # r_robot im MODELLRAUM (dort gilt das Griffband): Kamera -> Modell wie hole_aus_schale (aehnlichkeit)
        if ABB is not None:
            cm_ = np.array([math.cos(ABB[1]) * c_b[0] - math.sin(ABB[1]) * c_b[1], math.sin(ABB[1]) * c_b[0] + math.cos(ABB[1]) * c_b[1]]) * ABB[0] + ABB[2]
            r_robot = math.hypot(*cm_) * 1e3
        else: r_robot = math.hypot(*c_b) * 1e3
        # Greifer-Fussabdruck am Griffpunkt (dieselbe Probe wie hole_aus_schale.py, damit hier schon
        # aussortiert wird, was dort abgelehnt wuerde): Finger schliessen QUER zur Achse
        def in_schale(px_, py_, rand=None):
            ddx_, ddy_ = px_ - cs[0], py_ - cs[1]; r_ = math.hypot(ddx_, ddy_); rand = (0.002 + wand_abstand) if rand is None else rand
            if not (R_INNEN + rand <= r_ <= R_AUSSEN - rand): return False
            t_ = (math.atan2(ddy_, ddx_) - yaw_s + math.pi) % (2 * math.pi) - math.pi
            return r_ * math.sin(max(0.0, WINKEL / 2 - abs(t_))) >= rand
        in_schale_innen = lambda px_, py_: in_schale(px_, py_, 0.002)          # nur die Wand selbst (2 mm)
        qx, qy = -achse[1], achse[0]; Lh, Kh = fuss_lang / 2, fuss_kurz / 2
        fuss_ok = all(in_schale(griff_b[0] + sl * Lh * qx + sk * Kh * achse[0], griff_b[1] + sl * Lh * qy + sk * Kh * achse[1])
                      for sl, sk in ((1, 1), (1, -1), (-1, 1), (-1, -1), (1, 0), (-1, 0)))
        # Wand in SCHLIESSRICHTUNG (quer zur Achse, beide Seiten): vom Griffpunkt in 0.5-mm-Schritten bis zur
        # Innenflaeche der Wand laufen; minus Flanschradius = Abstand Wellenoberflaeche -> Wand (Benutzer 14.9.:
        # Fingerspitzen offen +-20 mm -> mindestens 20 mm frei, sonst schlaegt der Finger beim Schliessen an)
        def bis_wand(rx, ry):
            for i_ in range(1, 300):
                d_ = i_ * 0.0005
                if not in_schale_innen(griff_b[0] + rx * d_, griff_b[1] + ry * d_): return d_
            return 0.15
        wand_quer = (min(bis_wand(qx, qy), bis_wand(-qx, -qy)) - R_FLANSCH) * 1e3
        wl = {"i": int(i), "px": int(stats[i, 4]), "ys": ys, "xs": xs, "d_kopf_mm": d_kopf_mm, "mm_px": mm_px * 1e3,
              "mu_px": mu, "kopf_px": kopf_px, "spitze_px": spitze_px, "pts": pts,
              "mitte": c_b, "kopf": kopf_b, "spitze": spitze_b, "griff": griff_b, "z_mitte": z_mitte,
              "achse": achse, "yaw": yaw, "laenge_mm": L * 1e3, "breite_mm": breite_mm, "kopf_wie": kopf_wie,
              "wand_mm": wand, "wand_quer_mm": wand_quer, "nachbar_mm": None, "r_robot": r_robot, "gruende": [],
              "klasse": lab_klasse[i], "form_score_mm": (abs(L - LAENGE) + 0.5 * abs(2 * np.abs(quer_c).mean() * 1.25 * mm_px - 2 * R_KOPF)) * 1e3}
        if abs(L - LAENGE) > 0.007: wl["gruende"].append("Laenge %.0f mm (Soll %.0f, +-7)" % (L * 1e3, LAENGE * 1e3))
        if wand < wand_mm: wl["gruende"].append("nur %.0f mm von der Schalenwand (min %.0f)" % (wand, wand_mm))
        if not fuss_ok: wl["gruende"].append("Greifer-Fussabdruck %.0fx%.0f ragt in die Schalenwand" % (fuss_lang * 1e3, fuss_kurz * 1e3))
        if wand_quer < a.wand_quer_min: wl["gruende"].append("Wand in Schliessrichtung nur %.0f mm (min %.0f)" % (wand_quer, a.wand_quer_min))
        if not (r_min <= r_robot <= r_max): wl["gruende"].append("r_robot %.0f ausserhalb %.0f-%.0f" % (r_robot, r_min, r_max))
        if farben and fb:
            wl.update({"koerper": fb.get("koerper"), "streifen": fb.get("streifen"), "farbe_sicher": bool(fb.get("sicher")),
                       "hsv_koerper": fb.get("hsv_koerper"), "hsv_streifen": fb.get("hsv_streifen"), "px_streifen": fb.get("px_streifen", 0),
                       "ring_mm": fb.get("ring_mm")})
            if fb.get("fehler"): wl["fehler_farbe"] = fb["fehler"]
        return wl

    wellen = [vermessen(j) for sc, j, L_, B_ in kand if abs(L_ - LAENGE) <= 0.020]    # grob unpassende gar nicht erst
    if not wellen: say("ABBRUCH: kein Fleck mit Wellenlaenge (%.0f mm +-20)" % (LAENGE * 1e3)); return None
    # NACHBARN in Schliessrichtung (Benutzer 14.9.: Finger offen +-20 mm um die Achse; ein Finger, der auf der
    # Nachbarwelle landet, verschiebt sie). Zwei Quellen: (a) die anderen VERMESSENEN Wellen als Strecke
    # Kopf->Spitze (Abstand Achse->Nachbaroberflaeche = Querabstand - Kopfradius); (b) grosse KLUMPEN
    # (Laenge > Soll + 20: sich beruehrende Wellen gleicher Farbe) ueber ihre Bildpunkte (jeden 6.).
    # Kleine Flecken (Ringe, Bruchstuecke, Schattenreste) sind keine Nachbarn. Zaehlt nur, was laengs der
    # Achse im Fingerbereich liegt (+-(Fussabdruck kurz/2 + 5 mm) um den Griffpunkt).
    klumpen = {}
    for sc, j, L_, B_ in kand:
        if L_ <= LAENGE + 0.020: continue
        ys_, xs_ = np.nonzero(lab == j); sel = slice(None, None, 6)
        klumpen[j] = np.stack([base(np.array([float(x_), float(y_)])) for x_, y_ in zip(xs_[sel], ys_[sel])], 0) if len(xs_) else np.zeros((0, 2))
    band = fuss_kurz / 2 + 0.005
    for wl in wellen:
        qv = np.array([-wl["achse"][1], wl["achse"][0]]); g = wl["griff"]; naechster = None; wer = ""
        def merke(d_, name):
            nonlocal naechster, wer
            if naechster is None or d_ < naechster: naechster, wer = d_, name
        for wo in wellen:
            if wo is wl: continue
            P = wo["kopf"][None, :] + (wo["spitze"] - wo["kopf"])[None, :] * np.linspace(0, 1, 13)[:, None]
            rel = P - g; l_ = rel @ wl["achse"]; q_ = rel @ qv; im_band = np.abs(l_) <= band
            if im_band.any(): merke((float(np.abs(q_[im_band]).min()) - R_KOPF) * 1e3, "Welle %d" % (wellen.index(wo) + 1))
        for j, P in klumpen.items():
            if j == wl["i"] or not len(P): continue
            rel = P - g; l_ = rel @ wl["achse"]; q_ = rel @ qv
            lm_ = (P - wl["mitte"]) @ wl["achse"]
            eigen = (np.abs(lm_) <= LAENGE / 2 + 0.004) & (np.abs(q_) <= R_FLANSCH + 0.002)   # eigene Huelle
            im_band = (np.abs(l_) <= band) & ~eigen
            if im_band.any(): merke(float(np.abs(q_[im_band]).min()) * 1e3, "Klumpen %d" % j)
        wl["nachbar_mm"] = naechster; wl["nachbar_wer"] = wer
        if naechster is not None and naechster < a.nachbar_min:
            wl["gruende"].append("Nachbar (%s) liegt an/auf der Welle (%.0f mm quer)" % (wer, naechster))
        elif naechster is not None and naechster < a.nachbar_frei:
            wl["hinweis"] = "Finger kann Nachbar (%s) beruehren: %.0f mm quer (frei ab %.0f)" % (wer, naechster, a.nachbar_frei)
        # KONFIDENZ 0-100 (s. Kopf der Datei)
        lin = lambda v, lo, hi: min(1.0, max(0.0, (v - lo) / (hi - lo)))
        teile = {"wand": 30 * lin(wl["wand_quer_mm"], a.wand_quer_min, a.wand_quer_gut),
                 "nachbar": 20 * (1.0 if naechster is None else lin(naechster, a.nachbar_min, a.nachbar_frei + 10)),
                 "form": 20 * (1 - lin(wl["form_score_mm"], 0, 7)),
                 "kopf": 15 if wl["kopf_wie"].startswith("Ringe") else (12 if "unsicher" not in wl["kopf_wie"] else 6),
                 "farbe": 10 if (farben and wl.get("farbe_sicher")) else 0,
                 "lage": 5 * (1 - lin(abs(wl["r_robot"] - (r_min + r_max) / 2), 0, (r_max - r_min) / 2))}
        wl["konfidenz_teile"] = {k_: round(v_, 1) for k_, v_ in teile.items()}
        wl["konfidenz"] = 0.0 if wl["gruende"] else round(sum(teile.values()), 1)
    for k_, wl in enumerate(wellen, 1):
        say("  Welle %d: Mitte (%.1f, %.1f) mm, Achse %.1f Grad, Laenge %.0f, Breite ~%.0f mm, Wand %.0f mm (quer %.0f), Nachbar %s, r %.0f; Kopf per %s%s"
            % (k_, wl["mitte"][0] * 1e3, wl["mitte"][1] * 1e3, math.degrees(wl["yaw"]), wl["laenge_mm"], wl["breite_mm"],
               wl["wand_mm"], wl["wand_quer_mm"], "-" if wl["nachbar_mm"] is None else "%.0f mm" % wl["nachbar_mm"], wl["r_robot"], wl["kopf_wie"],
               ("; Farbe %s/%s%s" % (wl["koerper"], wl["streifen"], "" if wl.get("farbe_sicher") else " (unsicher)")) if farben else ""))
        say("      KONFIDENZ %5.1f  (%s)" % (wl["konfidenz"], " ".join("%s %.0f" % kv for kv in wl["konfidenz_teile"].items())))
        for g in wl["gruende"]: say("      !! " + g)
        if wl.get("hinweis"): say("      (!) " + wl["hinweis"])
    if wahl > 0:
        if wahl > len(wellen): say("!! --welle %d, aber nur %d gefunden" % (wahl, len(wellen))); return None
        best = wellen[wahl - 1]
    else:
        # greifbare (keine "gruende"), hoechste KONFIDENZ zuerst
        geeignet = sorted([wl for wl in wellen if not wl["gruende"]], key=lambda wl: -wl["konfidenz"])
        if not geeignet:
            say("ABBRUCH: %d Welle(n) gefunden, aber keine greifbar (s. Gruende). Welle umlegen oder --welle N erzwingen." % len(wellen))
            return "keine_greifbar"          # Exit 4 (GUI/ablauf: Wellen da, aber keine greifbar - nicht 'Schale leer')
        best = geeignet[0]
    best["weitere"] = [wl for wl in wellen if wl is not best]; best["arm_im_bild"] = arm_im_bild
    best["anzahl"] = len(wellen); best["nr"] = wellen.index(best) + 1
    return best


def ik_probe(node, griff, z_mitte, yaw):
    """Senkrechter Griff im MODELLRAUM (Abbildung aehnlichkeit wie hole_aus_schale) am Griffpunkt und +--ik-ueber:
    /compute_ik, kollisionsgeprueft, Haltung J3 < 0 und |J5| < 45 Grad (wie hole_aus_schale.ik_haltung). -> (ok, Grund)."""
    from geometry_msgs.msg import PoseStamped
    from moveit_msgs.srv import GetPositionIK
    from builtin_interfaces.msg import Duration
    ARM_ = ['joint2_to_joint1', 'joint3_to_joint2', 'joint4_to_joint3', 'joint5_to_joint4', 'joint6_to_joint5', 'joint6output_to_joint6']
    ikc = node.create_client(GetPositionIK, "/compute_ik")
    if not ikc.wait_for_service(5): return True, "(/compute_ik fehlt - Probe uebersprungen)"
    s_, th, t_ = ABB
    xm = s_ * (math.cos(th) * griff[0] - math.sin(th) * griff[1]) + t_[0]; ym = s_ * (math.sin(th) * griff[0] + math.cos(th) * griff[1]) + t_[1]
    yaw_m = yaw + th
    # z wie hole_aus_schale: Schalenboden + griff_mm (2.0) statt Flanschmitte
    z_g = z_mitte - R_FLANSCH + 0.002
    eZ = np.array([math.cos(yaw_m), math.sin(yaw_m), 0.0]); eY = np.array([0.0, 0.0, -1.0]); eX = np.cross(eY, eZ); R = np.column_stack([eX, eY, eZ])
    tr = R[0, 0] + R[1, 1] + R[2, 2]
    if tr > 0: sq = math.sqrt(tr + 1) * 2; qx, qy, qz, qw = (R[2, 1] - R[1, 2]) / sq, (R[0, 2] - R[2, 0]) / sq, (R[1, 0] - R[0, 1]) / sq, 0.25 * sq
    else:
        i = int(np.argmax([R[0, 0], R[1, 1], R[2, 2]]))
        if i == 0: sq = math.sqrt(1 + R[0, 0] - R[1, 1] - R[2, 2]) * 2; qx, qy, qz, qw = 0.25 * sq, (R[0, 1] + R[1, 0]) / sq, (R[0, 2] + R[2, 0]) / sq, (R[2, 1] - R[1, 2]) / sq
        elif i == 1: sq = math.sqrt(1 + R[1, 1] - R[0, 0] - R[2, 2]) * 2; qx, qy, qz, qw = (R[0, 1] + R[1, 0]) / sq, 0.25 * sq, (R[1, 2] + R[2, 1]) / sq, (R[0, 2] - R[2, 0]) / sq
        else: sq = math.sqrt(1 + R[2, 2] - R[0, 0] - R[1, 1]) * 2; qx, qy, qz, qw = (R[0, 2] + R[2, 0]) / sq, (R[1, 2] + R[2, 1]) / sq, 0.25 * sq, (R[1, 0] - R[0, 1]) / sq
    try: ref = [float(v) for v in json.load(open(os.path.expanduser("~/ros2_ws/teach_punkte.json")))["nest_33"]["rad"]]
    except Exception: ref = [0.0] * 6
    for z_, was in ((z_g + a.ik_ueber / 1e3, "Ueber-Punkt"), (z_g, "Griffpunkt")):
        gefunden = False
        for versuch in range(4):
            rq = GetPositionIK.Request(); r = rq.ik_request; r.group_name = "arm"; r.ik_link_name = "tcp"; r.avoid_collisions = True
            r.timeout = Duration(sec=0, nanosec=400000000)
            r.robot_state.joint_state.name = ARM_ + ["gripper_controller"]; r.robot_state.joint_state.position = ref + [-0.464]
            ps = PoseStamped(); ps.header.frame_id = "robot_base"; pp = ps.pose; pp.position.x, pp.position.y, pp.position.z = float(xm), float(ym), float(z_)
            pp.orientation.x, pp.orientation.y, pp.orientation.z, pp.orientation.w = float(qx), float(qy), float(qz), float(qw); r.pose_stamped = ps
            f = ikc.call_async(rq); rclpy.spin_until_future_complete(node, f, timeout_sec=3); res = f.result()
            if res is None or res.error_code.val != 1: continue
            nm = list(res.solution.joint_state.name); vl = list(res.solution.joint_state.position); sol = [vl[nm.index(j)] for j in ARM_]
            if sol[2] < 0 and abs(sol[4]) < math.radians(45): gefunden = True; break
        if not gefunden: return False, "%s (%.0f, %.0f, z %.0f) mm im Modell nicht erreichbar (senkrecht, Ellbogen, |J5|<45, kollisionsfrei)" % (was, xm * 1e3, ym * 1e3, z_ * 1e3)
    return True, ""


def main():
    rclpy.init(); k = sf.Finder()
    if a.npz:
        d = np.load(a.npz, allow_pickle=True)
        farbe, K, q, t = d["farbe"], tuple(d["K"]), tuple(d["q"]), np.array(d["t"]); tiefe = d["tiefe"]
        schale = dict(d["schale"].item()) if "schale" in d else None
    else:
        print("warte auf Kamera + TF ...")
        ende = time.time() + 30
        while time.time() < ende and not (k.bereit() and k.farbbild is not None): rclpy.spin_once(k, timeout_sec=0.2)
        if k.farbbild is None: print("ABBRUCH: kein Bild"); return 1
        while time.time() < ende and not k.puffer.can_transform("robot_base", k.depth_frame, rclpy.time.Time()):
            rclpy.spin_once(k, timeout_sec=0.2)
        tf = k.puffer.lookup_transform("robot_base", k.depth_frame, rclpy.time.Time())
        r, tt = tf.transform.rotation, tf.transform.translation
        farbe, K, q, t = k.farbbild, k.K, (r.x, r.y, r.z, r.w), np.array([tt.x, tt.y, tt.z])
        st = np.stack(k.tiefen).astype(np.float32); st[st == 0] = np.nan
        import warnings
        with warnings.catch_warnings():
            warnings.simplefilter("ignore", RuntimeWarning); tiefe = np.nanmedian(st, axis=0)
        schale = None
    if schale is None:
        ende = time.time() + 10
        while time.time() < ende and not k.puffer.can_transform("robot_base", "schale", rclpy.time.Time()):
            rclpy.spin_once(k, timeout_sec=0.2)
        if not k.puffer.can_transform("robot_base", "schale", rclpy.time.Time()):
            print("ABBRUCH: kein TF robot_base->schale (SCHALE_MONTIERT=1?)"); return 1
        ts = k.puffer.lookup_transform("robot_base", "schale", rclpy.time.Time()); rs, tsl = ts.transform.rotation, ts.transform.translation
        yaw_s = math.atan2(2 * (rs.w * rs.z + rs.x * rs.y), 1 - 2 * (rs.y * rs.y + rs.z * rs.z))
        schale = {"cs": (tsl.x, tsl.y), "yaw": yaw_s, "z_boden": tsl.z}
    if a.hintergrund_speichern:
        np.savez_compressed(a.hintergrund, farbe=farbe, K=np.array(K), q=np.array(q), t=t, zeit=time.strftime("%Y-%m-%d %H:%M"))
        print("  Referenzbild der leeren Schale gesichert: %s (%dx%d). Bei anderem Licht neu aufnehmen." % (a.hintergrund, farbe.shape[1], farbe.shape[0]))
        k.destroy_node(); rclpy.shutdown(); return 0
    hintergrund = None
    if not a.ohne_hintergrund and os.path.exists(a.hintergrund):
        hg = np.load(a.hintergrund, allow_pickle=True); hintergrund = hg["farbe"]
        if hintergrund.shape != farbe.shape:
            # 14.9.: Referenz 1280x720, Kamera 848x480 - beide 16:9 mit gleichem Sichtfeld (D435i) -> skalieren statt
            # ignorieren. Ohne Referenz wurden die Glanzstreifen der Schalenrillen als weisse Wellen gefunden.
            if abs(hintergrund.shape[1] / hintergrund.shape[0] - farbe.shape[1] / farbe.shape[0]) < 0.02:
                hintergrund = cv2.resize(hintergrund, (farbe.shape[1], farbe.shape[0]), interpolation=cv2.INTER_AREA)
                print("  Referenzbild %s auf %dx%d skaliert (gleiches Sichtfeld); besser neu aufnehmen: --hintergrund-speichern"
                      % (a.hintergrund, farbe.shape[1], farbe.shape[0]))
            else:
                print("  !! Referenzbild %s hat %s statt %s - wird ignoriert (neu aufnehmen: --hintergrund-speichern)"
                      % (a.hintergrund, hintergrund.shape[1::-1], farbe.shape[1::-1])); hintergrund = None
        else: print("  Referenzbild der leeren Schale: %s (%s)" % (a.hintergrund, str(hg["zeit"]) if "zeit" in hg else "?"))
    elif not a.ohne_hintergrund:
        print("  !! kein Referenzbild der leeren Schale (%s) - Glanzstreifen koennen weisse Wellen verbinden; "
              "aufnehmen mit --hintergrund-speichern" % a.hintergrund)
    if a.speichern:
        np.savez_compressed(a.speichern, farbe=farbe, tiefe=tiefe, K=np.array(K), q=np.array(q), t=t,
                            schale=np.array(schale, dtype=object), nur_hell=a.nur_hell, zeit=time.strftime("%Y-%m-%d %H:%M"))
        print("  Frame gesichert: %s" % a.speichern)
    fx, fy, cx, cy = K

    wl = welle_in_frame(farbe, tiefe, K, q, t, schale, nur_hell=a.nur_hell, wahl=a.welle, farben=not a.ohne_farbe, alte_maske=a.alte_maske,
                        hintergrund=hintergrund)
    if wl is None: return 2
    if wl == "keine_greifbar": return 4
    # IK-PROBE (14.9.): Kandidaten in Konfidenz-Reihenfolge; der erste, fuer den MoveIt eine senkrechte Griffhaltung
    # (Ellbogen J3 < 0, |J5| < 45, kollisionsfrei) am Griffpunkt UND am Ueber-Punkt findet, wird genommen. Vorher
    # lehnte hole_aus_schale das Ziel erst nach der Fahrt zur Kamerapose ab -> Endlosschleife auf derselben Welle.
    if not a.ohne_ik_probe and not a.npz and ABB is not None:
        kand_ = sorted([wl] + [w_ for w_ in wl["weitere"] if not w_["gruende"]], key=lambda w_: -w_["konfidenz"])
        weitere_alle = [wl] + wl["weitere"]
        gewaehlt = None
        for w_ in kand_:
            ok, warum = ik_probe(k, w_["griff"], w_["z_mitte"], w_["yaw"])
            print("  IK-Probe %s/%s bei (%.0f, %.0f) mm, Konfidenz %.0f: %s" % (w_.get("koerper"), w_.get("streifen"), w_["mitte"][0] * 1e3, w_["mitte"][1] * 1e3, w_["konfidenz"], "ok" if ok else "NEIN - " + warum))
            if ok: gewaehlt = w_; break
            w_["gruende"].append("IK-Probe: " + warum); w_["konfidenz"] = 0.0
        if gewaehlt is None:
            print("ABBRUCH: keine der greifbaren Wellen ist mit senkrechtem Griff erreichbar (IK). Welle umlegen."); return 4
        if gewaehlt is not wl:
            nr = weitere_alle.index(gewaehlt) + 1
            gewaehlt["weitere"] = [w_ for w_ in weitere_alle if w_ is not gewaehlt]; gewaehlt["arm_im_bild"] = wl["arm_im_bild"]
            gewaehlt["anzahl"] = wl["anzahl"]; gewaehlt["nr"] = nr; wl = gewaehlt
    c_b, kopf_b, spitze_b, griff_b, yaw, z_mitte, achse = wl["mitte"], wl["kopf"], wl["spitze"], wl["griff"], wl["yaw"], wl["z_mitte"], wl["achse"]
    kopf_px, spitze_px = wl["kopf_px"], wl["spitze_px"]
    print("GEWAEHLT Welle %d von %d: %d px, Laenge %.0f mm (Soll %.0f), Breite ~%.0f mm; Kopf-Ende per %s"
          % (wl["nr"], wl["anzahl"], wl["px"], wl["laenge_mm"],
             LAENGE * 1e3, wl["breite_mm"], wl["kopf_wie"]))
    print("  Mitte      (%.1f, %.1f) mm" % (c_b[0] * 1e3, c_b[1] * 1e3))
    print("  Kopf-Ende  (%.1f, %.1f) mm   Spitze (%.1f, %.1f) mm" % (kopf_b[0] * 1e3, kopf_b[1] * 1e3, spitze_b[0] * 1e3, spitze_b[1] * 1e3))
    print("  Achse Kopf->Spitze %.1f Grad, r_robot %.0f mm, Wand %.0f mm (Schliessrichtung %.0f mm), Nachbar %s"
          % (math.degrees(yaw), wl["r_robot"], wl["wand_mm"], wl["wand_quer_mm"], "-" if wl["nachbar_mm"] is None else "%.0f mm" % wl["nachbar_mm"]))
    print("  KONFIDENZ %.1f / 100  (%s)" % (wl["konfidenz"], ", ".join("%s %.0f" % kv for kv in wl["konfidenz_teile"].items())))
    print("  GRIFFPUNKT (Flansch) (%.1f, %.1f, z %.1f) mm" % (griff_b[0] * 1e3, griff_b[1] * 1e3, z_mitte * 1e3))
    if not a.ohne_farbe:
        print("  FARBE: Koerper %s, Streifen %s (%s; HSV Koerper %s, Streifen %s, %d Streifen-px)"
              % (wl.get("koerper"), wl.get("streifen"), "sicher" if wl.get("farbe_sicher") else "UNSICHER",
                 wl.get("hsv_koerper"), wl.get("hsv_streifen"), wl.get("px_streifen", 0)))
    def eintrag(w_):
        e = {"mitte_mm": [round(w_["mitte"][0] * 1e3, 1), round(w_["mitte"][1] * 1e3, 1)],
             "kopf_mm": [round(w_["kopf"][0] * 1e3, 1), round(w_["kopf"][1] * 1e3, 1)],
             "spitze_mm": [round(w_["spitze"][0] * 1e3, 1), round(w_["spitze"][1] * 1e3, 1)],
             "griff_mm": [round(w_["griff"][0] * 1e3, 1), round(w_["griff"][1] * 1e3, 1), round(w_["z_mitte"] * 1e3, 1)],
             "yaw_grad": round(math.degrees(w_["yaw"]), 1), "laenge_mm": round(w_["laenge_mm"], 1),
             "wand_mm": round(w_["wand_mm"], 1), "wand_quer_mm": round(w_["wand_quer_mm"], 1),
             "nachbar_mm": None if w_["nachbar_mm"] is None else round(w_["nachbar_mm"], 1),
             "konfidenz": w_["konfidenz"], "konfidenz_teile": w_["konfidenz_teile"], "hinweis": w_.get("hinweis"),
             "r_robot_mm": round(w_["r_robot"], 1), "gruende": w_["gruende"]}
        if not a.ohne_farbe:
            e.update({"koerper": w_.get("koerper"), "streifen": w_.get("streifen"), "farbe_sicher": w_.get("farbe_sicher", False)})
        return e
    out = eintrag(wl)
    out.update({"anzahl": wl["anzahl"], "weitere": [eintrag(w_) for w_ in wl["weitere"]], "arm_im_bild": wl["arm_im_bild"],
                "oeffnung_mm": a.oeffnung_mm, "nachbar_min_mm": a.nachbar_min, "wand_quer_min_mm": a.wand_quer_min,
                "zeit": time.strftime("%Y-%m-%d %H:%M"), "quelle": "welle_finden_schale.py"})
    json.dump(out, open(a.ziel, "w"), indent=1)
    print("  -> %s" % a.ziel)

    # RViz: Modell in echter Groesse (Zylinder liegen: Achse = x-Achse des Markers)
    arr = MarkerArray()
    def zyl(idx, ab, laenge, radius, farbe_):
        mk = Marker(); mk.header.frame_id = "robot_base"; mk.ns = "welle_modell"; mk.id = idx; mk.type = Marker.CYLINDER
        mk.action = Marker.ADD
        p = kopf_b + achse * (ab + laenge / 2.0)
        mk.pose.position.x, mk.pose.position.y, mk.pose.position.z = float(p[0]), float(p[1]), float(z_mitte)
        # Zylinderachse (Marker z) auf die Wellenachse legen: q = q_yaw(z) * q_pitch(+90 um y), Hamilton-Produkt
        cy_, sy_ = math.cos(yaw / 2), math.sin(yaw / 2); cp, sp = math.cos(math.pi / 4), math.sin(math.pi / 4)
        ax, ay, az, aw = 0.0, 0.0, sy_, cy_; bx, by, bz, bw = 0.0, sp, 0.0, cp
        qx = aw * bx + ax * bw + ay * bz - az * by; qy = aw * by - ax * bz + ay * bw + az * bx
        qz = aw * bz + ax * by - ay * bx + az * bw; qw = aw * bw - ax * bx - ay * by - az * bz
        mk.pose.orientation.x, mk.pose.orientation.y, mk.pose.orientation.z, mk.pose.orientation.w = qx, qy, qz, qw
        mk.scale.x = mk.scale.y = 2 * radius; mk.scale.z = laenge
        mk.color = ColorRGBA(r=farbe_[0], g=farbe_[1], b=farbe_[2], a=0.95)
        arr.markers.append(mk)
    zyl(0, 0.0, KOPF, R_KOPF, (0.9, 0.9, 0.9)); zyl(1, KOPF, HALS, R_HALS, (0.8, 0.8, 0.8))
    zyl(2, KOPF + HALS, FLANSCH, R_FLANSCH, (1.0, 0.3, 0.3)); zyl(3, KOPF + HALS + FLANSCH, SCHAFT, R_SCHAFT, (0.8, 0.8, 0.8))
    g = Marker(); g.header.frame_id = "robot_base"; g.ns = "welle_modell"; g.id = 10; g.type = Marker.SPHERE; g.action = Marker.ADD
    g.pose.position.x, g.pose.position.y, g.pose.position.z = float(griff_b[0]), float(griff_b[1]), float(z_mitte + 0.02)
    g.pose.orientation.w = 1.0; g.scale.x = g.scale.y = g.scale.z = 0.006; g.color = ColorRGBA(r=0.0, g=1.0, b=0.0, a=1.0)
    arr.markers.append(g)
    # weitere Wellen: grauer Stab als Hinweis
    for n_, w_ in enumerate(wl["weitere"]):
        mk = Marker(); mk.header.frame_id = "robot_base"; mk.ns = "welle_weitere"; mk.id = 20 + n_; mk.type = Marker.LINE_LIST
        mk.action = Marker.ADD; mk.scale.x = 0.006; mk.color = ColorRGBA(r=0.6, g=0.6, b=0.6, a=0.8); mk.pose.orientation.w = 1.0
        mk.points = [Point(x=float(w_["kopf"][0]), y=float(w_["kopf"][1]), z=float(z_mitte)),
                     Point(x=float(w_["spitze"][0]), y=float(w_["spitze"][1]), z=float(z_mitte))]
        arr.markers.append(mk)
    from rclpy.qos import QoSProfile, DurabilityPolicy, ReliabilityPolicy, HistoryPolicy
    pub = k.create_publisher(MarkerArray, "/welle/modell", QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL,
                                                                     reliability=ReliabilityPolicy.RELIABLE, history=HistoryPolicy.KEEP_LAST))
    pub.publish(arr)
    for _ in range(10): rclpy.spin_once(k, timeout_sec=0.1)
    print("  RViz: /welle/modell (MarkerArray, latched) - Kopf/Hals/Flansch(rot)/Schaft, gruene Kugel = Griffpunkt, grau = weitere")

    if a.ausschnitt:
        Rq_ = rot_inv(q); mu_px = wl["mu_px"]
        x0, y0 = int(mu_px[0]) - 70, int(mu_px[1]) - 70
        x0, y0 = max(0, x0), max(0, y0); crop = farbe[y0:y0 + 140, x0:x0 + 140].copy()
        crop = cv2.resize(crop, None, fx=4, fy=4, interpolation=cv2.INTER_CUBIC)
        P_ = lambda p_: (int((p_[0] - x0) * 4), int((p_[1] - y0) * 4))
        cv2.line(crop, P_(kopf_px), P_(spitze_px), (255, 0, 255), 2)
        cv2.circle(crop, P_(kopf_px), 12, (0, 0, 255), 3); cv2.circle(crop, P_(mu_px), 7, (0, 255, 0), -1)
        Pc = (np.array([griff_b[0], griff_b[1], z_mitte]) - t) @ Rq_.T
        gu, gv = Pc[0] / Pc[2] * fx + cx, Pc[1] / Pc[2] * fy + cy
        cv2.drawMarker(crop, P_((gu, gv)), (0, 255, 255), cv2.MARKER_CROSS, 24, 2)
        cv2.putText(crop, "%s / %s%s" % (wl.get("koerper"), wl.get("streifen"), "" if wl.get("farbe_sicher") else " ?"), (8, 28), cv2.FONT_HERSHEY_SIMPLEX, 0.9, (0, 0, 0), 5)
        cv2.putText(crop, "%s / %s%s" % (wl.get("koerper"), wl.get("streifen"), "" if wl.get("farbe_sicher") else " ?"), (8, 28), cv2.FONT_HERSHEY_SIMPLEX, 0.9, (255, 255, 255), 2)
        cv2.putText(crop, "rot=Kopf gruen=Mitte gelb=Griff magenta=Achse", (8, crop.shape[0] - 10), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (255, 255, 255), 1)
        cv2.imwrite(a.ausschnitt, crop); print("  Ausschnitt: %s" % a.ausschnitt)
    if a.uebersicht and schale is not None:
        # Ganzes Bild x2; Projektion robot_base -> Bild in der Griffhoehe (dieselbe Ebene wie base())
        SK = 2; Rq_ = rot_inv(q); cs_, yaw_s_ = schale["cs"], schale["yaw"]
        def proj(xb, yb):
            Pc = (np.array([xb, yb, z_mitte]) - t) @ Rq_.T
            return (int(round((Pc[0] / Pc[2] * fx + cx) * SK)), int(round((Pc[1] / Pc[2] * fy + cy) * SK)))
        def sektor(r0, r1, rand):
            # Ringsektor r0..r1 mit Abstand rand zu den Flanken (r sin(WINKEL/2 - |t|) >= rand)
            pts = []
            for r_ in (r1, r0):
                hw = WINKEL / 2 - math.asin(min(1.0, rand / r_))
                ts_ = np.linspace(-hw, hw, 60) if r_ == r1 else np.linspace(hw, -hw, 60)
                pts += [proj(cs_[0] + r_ * math.cos(yaw_s_ + t_), cs_[1] + r_ * math.sin(yaw_s_ + t_)) for t_ in ts_]
            return np.array(pts, np.int32).reshape(-1, 1, 2)
        out_ = cv2.resize(farbe, None, fx=SK, fy=SK, interpolation=cv2.INTER_CUBIC)
        sperr = 0.002 + a.wand_quer_min / 1e3                       # Wand 2 mm + Sperrzone
        innen_k = sektor(R_INNEN + 0.002, R_AUSSEN - 0.002, 0.002)  # Innenflaeche der Wand
        frei_k = sektor(R_INNEN + sperr, R_AUSSEN - sperr, sperr)   # greifbarer Bereich
        lage = np.zeros_like(out_); cv2.fillPoly(lage, [innen_k], (0, 0, 255)); cv2.fillPoly(lage, [frei_k], (0, 0, 0))
        m_ = lage[:, :, 2] > 0; out_[m_] = (0.55 * out_[m_] + 0.45 * lage[m_]).astype(np.uint8)   # Sperrzone rot getoent
        cv2.polylines(out_, [innen_k], True, (255, 255, 255), 1); cv2.polylines(out_, [frei_k], True, (0, 0, 255), 2)
        # NUR die gewaehlte Welle: Mittellinie laengs (Kopf -> Spitze, ueber die Enden hinaus verlaengert), Kopf, Griffpunkt
        k_px, s_px = np.array(kopf_px) * SK, np.array(spitze_px) * SK; d_ = (s_px - k_px) / max(1e-6, np.linalg.norm(s_px - k_px))
        cv2.line(out_, tuple((k_px - d_ * 25).astype(int)), tuple((s_px + d_ * 25).astype(int)), (255, 0, 255), 2)
        cv2.circle(out_, tuple(k_px.astype(int)), 10, (0, 0, 255), 2)
        cv2.putText(out_, "Kopf", tuple((k_px + np.array([12, -8])).astype(int)), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 0, 255), 2)
        cv2.drawMarker(out_, proj(griff_b[0], griff_b[1]), (0, 255, 255), cv2.MARKER_CROSS, 18, 2)
        # auf die Schale zuschneiden (+ Rand), damit sie im GUI-Fenster gross ist
        bx, by, bw, bh = cv2.boundingRect(innen_k); m_ = 60
        x0_, y0_ = max(0, bx - m_), max(0, by - 2 * m_); x1_, y1_ = min(out_.shape[1], bx + bw + m_), min(out_.shape[0], by + bh + m_)
        out_ = out_[y0_:y1_, x0_:x1_]
        if out_.shape[1] < 1000: out_ = cv2.resize(out_, None, fx=1000 / out_.shape[1], fy=1000 / out_.shape[1], interpolation=cv2.INTER_CUBIC)
        z1 = "Gewaehlte Welle: %s / %s   Konfidenz %.0f/100" % (wl.get("koerper"), wl.get("streifen"), wl.get("konfidenz", 0))
        z2 = "rot = Sperrzone %.0f mm an der Wand (Finger offen %.0f mm)" % (a.wand_quer_min, a.oeffnung_mm)
        z3 = "Wellen in der Sperrzone werden nicht gegriffen"
        for i_, z_ in enumerate((z1, z2, z3)):
            cv2.putText(out_, z_, (10, 28 + 30 * i_), cv2.FONT_HERSHEY_SIMPLEX, 0.75, (0, 0, 0), 5)
            cv2.putText(out_, z_, (10, 28 + 30 * i_), cv2.FONT_HERSHEY_SIMPLEX, 0.75, (255, 255, 255), 2)
        cv2.imwrite(a.uebersicht, out_); print("  Uebersicht: %s" % a.uebersicht)
    if a.bild:
        out_ = cv2.resize(farbe, None, fx=3, fy=3, interpolation=cv2.INTER_CUBIC)
        for w_ in wl["weitere"]:
            cv2.line(out_, (int(w_["kopf_px"][0] * 3), int(w_["kopf_px"][1] * 3)), (int(w_["spitze_px"][0] * 3), int(w_["spitze_px"][1] * 3)), (160, 160, 160), 2)
        cv2.line(out_, (int(kopf_px[0] * 3), int(kopf_px[1] * 3)), (int(spitze_px[0] * 3), int(spitze_px[1] * 3)), (255, 0, 255), 2)
        cv2.circle(out_, (int(kopf_px[0] * 3), int(kopf_px[1] * 3)), 7, (0, 0, 255), 2)
        Rq = rot_inv(q)
        Pc = (np.array([griff_b[0], griff_b[1], z_mitte]) - t) @ Rq.T
        gu, gv = int((Pc[0] / Pc[2] * fx + cx) * 3), int((Pc[1] / Pc[2] * fy + cy) * 3)
        cv2.circle(out_, (gu, gv), 6, (0, 255, 0), -1)
        txt = "rot = Kopf, gruen = Griffpunkt (Flansch), magenta = Achse, grau = weitere Wellen"
        if not a.ohne_farbe: txt += " | %s/%s" % (wl.get("koerper"), wl.get("streifen"))
        cv2.putText(out_, txt, (8, 24), cv2.FONT_HERSHEY_SIMPLEX, 0.6, (255, 255, 255), 2)
        cv2.imwrite(a.bild, out_); print("  Overlay: %s" % a.bild)
    k.destroy_node(); rclpy.shutdown(); return 0


if __name__ == "__main__":
    sys.exit(main())
