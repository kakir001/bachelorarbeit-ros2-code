#!/usr/bin/env python3
"""Die Bereitstellungsschale ueber ihren UMRISS IM FARBBILD finden (zweiter Weg).

WARUM EIN ZWEITER WEG: tools/schale_finden.py sucht den Schalenrand im Tiefenbild
als oertliche Erhebung. Am Aufbau (Kamera 61 cm ueber der Platte, Wand 2 mm stark)
kommt die 17 mm hohe Wand im Tiefenbild aber nur noch als 5-9 mm hohe, 2-3 Zellen
breite Welle an - die Stereo-Korrelation der D435i mittelt die duenne Wand mit dem
Boden daneben weg (2026-09-12 gemessen: 424x240 gar nichts, 848x480 zu wenig fuer die
Hough-Abstimmung mit festem Radius). Im FARBBILD dagegen ist die hellgraue Schale auf
der dunkleren Platte scharf begrenzt. Also: Umriss aus dem Farbbild, Hoehe aus dem
Tiefenbild, Form aus schale.xacro.

VERFAHREN
 1. Bereich: nur Bildpunkte, deren Tiefe (zum Farbbild ausgerichtet) in robot_base in
    einem Rechteck auf der Platte und dicht ueber ihr liegt (--bereich, z -10..+40 mm).
    Damit fallen Arm, Trichter (115 mm hoch), Tisch und Wand weg.
 2. Helligkeit: Otsu-Schwelle im Bereich, morphologisch geoeffnet (die weissen
    Gitterlinien der Platte sind 1-2 Bildpunkte breit und verschwinden), groesste
    zusammenhaengende helle Flaeche = Schale. Ihr Umriss wird genommen.
 3. Jeder Umrisspunkt wird als Sehstrahl auf die Ebene z = Randoberkante geschnitten
    (Plattenhoehe aus dem Tiefenbild + 17 mm Wandhoehe). Von oben gesehen ist die
    Silhouette der Schale ihre Oberkante - auf der kameranahen wie auf der -fernen
    Seite, weil der Fuss der Wand jeweils hinter der Oberkante liegt.
 4. Die FESTE Form (Kreisringausschnitt 45.3 Grad, R 186.9..301.9 mm) wird mit drei
    Unbekannten (Mittelpunkt x, y, Gierwinkel) in die Umrisspunkte eingepasst:
    kleinste Quadrate des Abstands Punkt -> Formrand. Startwerte aus Schwerpunkt und
    Hauptachse der Flaeche, in allen vier Richtungen versucht, die beste gewinnt.

PROBEN (vor dem Schreiben)
  * Restfehler rms des Fits (Erwartung wenige mm; ein um eine Wand verrutschter
    oder falsch herum eingepasster Fit liegt bei > 10 mm)
  * Deckung: Anteil des Formrands, dem ein Umrisspunkt naeher als 6 mm kommt
  * Flaeche der hellen Region gegen die Sollflaeche der Schale (115 mm * mittlerer
    Bogen 170 mm = 19 600 mm2)
  * zweitbester Startwert deutlich schlechter (sonst ist die Richtung nicht eindeutig)

    python3 tools/schale_finden_kontur.py                 # zeichnet, fragt dann nach
    python3 tools/schale_finden_kontur.py --schreiben     # ohne Rueckfrage
    python3 tools/schale_finden_kontur.py --npz frame.npz # ohne Kamera (gesicherter Frame)

ECKEN KLICKEN (--ecken): der sichere Weg, wenn die Silhouette nicht sauber zu trennen
ist (2026-09-12: Klarsichtfolie neben der Schale ist heller als die Schale selbst, und
die Schalenunterseite ist zerkratzt - weder Helligkeit noch Glattheit trennen sauber).
Im Fenster (DISPLAY=:0) die VIER ECKEN der Schale an der OBERKANTE der Wand anklicken
(die Ecken sind mit ~1 cm verrundet - den gedachten Schnittpunkt der Kanten klicken; das
Nachziehen an den Kanten korrigiert den Rest),
beliebige Reihenfolge; u = letzten Klick zuruecknehmen, Enter = einpassen, q = Ende.
Die vier Ecken legen Mittelpunkt und Gierwinkel eindeutig fest (8 Gleichungen fuer 3
Unbekannte); die Probe ist der Restfehler der Ecken (Erwartung 2-4 mm; ein um eine
Wand verrutschter Fit liegt bei > 30 mm).
    DISPLAY=:0 python3 tools/schale_finden_kontur.py --ecken

Overlay: /schale/overlay (latched) bzw. --bild DATEI.png. Schreibt schale_pose.xacro
ueber tools/schale_finden.py (dieselbe Datei, dieselbe Konvention: Ursprung =
Ringmittelpunkt, z = Innenboden, +X = Winkelhalbierende).
"""
import argparse, math, os, sys
from datetime import date

import cv2
import numpy as np

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))

ap = argparse.ArgumentParser()
ap.add_argument("--depth", default="/camera/aligned_depth_to_color/image_raw")
ap.add_argument("--info", default="/camera/color/camera_info")
ap.add_argument("--farbe", default="/camera/color/image_raw")
ap.add_argument("--frame", default="robot_base")
ap.add_argument("--bereich", type=float, nargs=4, default=(-0.02, 0.35, -0.24, -0.02),
                metavar=("XMIN", "XMAX", "YMIN", "YMAX"),
                help="Suchrechteck in robot_base [m] (Platte ohne Roboterbasis)")
ap.add_argument("--z", type=float, nargs=2, default=(-0.012, 0.040),
                help="Tiefenfenster ueber der Platte [m]")
ap.add_argument("--mittelung", type=int, default=8)
ap.add_argument("--oeffnen", type=int, default=7, help="Kernel fuer morphologisches Oeffnen [px]")
ap.add_argument("--schreiben", action="store_true")
ap.add_argument("--nur-zeigen", action="store_true")
ap.add_argument("--stumm", action="store_true")
ap.add_argument("--bild", default="")
ap.add_argument("--npz", default="", help="gesicherter Frame statt Kamera (Entwicklung)")
ap.add_argument("--ecken", action="store_true", help="vier Ecken im Fenster anklicken statt Umriss suchen")
ap.add_argument("--wiederholen", type=int, default=5,
                help="so viele Frames nacheinander einpassen, Ergebnis = Median (Ecken-Modus; Streuung wird gemeldet)")
ap.add_argument("--klicks", type=int, nargs=8, metavar="PX", default=None,
                help="die vier Ecken als u1 v1 u2 v2 u3 v3 u4 v4 statt Fenster (Wiederholung ohne Klicken)")
a = ap.parse_args()
sys.argv = [sys.argv[0]]        # schale_finden.py parst beim Import selbst argparse


def _sf():
    import schale_finden
    return schale_finden

# Form - identisch zu schale.xacro / schale_finden.py (Silhouette = Aussenmasse).
R_INNEN, R_AUSSEN, WANDHOEHE, BODENDICKE = 0.1869, 0.3019, 0.017, 0.002
WINKEL = math.radians(45.3)   # Oberkante, Kamera 2026-09-12 (vorher 39.9); = schale_winkel im URDF
SOLL_FLAECHE = 0.5 * WINKEL * (R_AUSSEN ** 2 - R_INNEN ** 2)


def qrot(q, p):
    x, y, z, w = q
    R = np.array([[1 - 2 * (y * y + z * z), 2 * (x * y - z * w), 2 * (x * z + y * w)],
                  [2 * (x * y + z * w), 1 - 2 * (x * x + z * z), 2 * (y * z - x * w)],
                  [2 * (x * z - y * w), 2 * (y * z + x * w), 1 - 2 * (x * x + y * y)]])
    return p @ R.T


def form_rand(c, yaw, n=60, winkel=None):
    """Dicht abgetasteter Rand des Ringausschnitts (M,2)."""
    winkel = WINKEL if winkel is None else winkel
    w = np.linspace(yaw - winkel / 2, yaw + winkel / 2, n)
    r = np.linspace(R_INNEN, R_AUSSEN, n // 2)
    pts = [np.stack([c[0] + R_INNEN * np.cos(w), c[1] + R_INNEN * np.sin(w)], 1),
           np.stack([c[0] + R_AUSSEN * np.cos(w), c[1] + R_AUSSEN * np.sin(w)], 1)]
    for ww in (w[0], w[-1]):
        pts.append(np.stack([c[0] + r * math.cos(ww), c[1] + r * math.sin(ww)], 1))
    return np.concatenate(pts)


def rand_abstand(par, P):
    """Abstand jedes Punkts in P (N,2) zum Formrand, analytisch.
    par = (cx, cy, yaw) mit dem Modellwinkel, oder (cx, cy, yaw, winkel) mit freiem Winkel."""
    cx, cy, yaw = par[0], par[1], par[2]
    winkel = par[3] if len(par) > 3 else WINKEL
    dx, dy = P[:, 0] - cx, P[:, 1] - cy
    r = np.hypot(dx, dy)
    t = (np.arctan2(dy, dx) - yaw + math.pi) % (2 * math.pi) - math.pi   # -pi..pi um yaw
    im_sektor = np.abs(t) <= winkel / 2
    # Boegen (nur im Sektor), sonst Abstand zu den Bogenenden ueber die Seiten
    d = np.full(len(P), np.inf)
    d[im_sektor] = np.minimum(np.abs(r - R_INNEN), np.abs(r - R_AUSSEN))[im_sektor]
    # radiale Seiten: Strecke von R_INNEN bis R_AUSSEN in Richtung yaw +- WINKEL/2
    for s in (-1.0, 1.0):
        ww = yaw + s * winkel / 2
        ex, ey = math.cos(ww), math.sin(ww)
        laengs = dx * ex + dy * ey
        quer = -dx * ey + dy * ex
        lc = np.clip(laengs, R_INNEN, R_AUSSEN)
        ds = np.hypot(laengs - lc, quer)
        d = np.minimum(d, ds)
    return d


def fit(P, start):
    from scipy.optimize import least_squares
    erg = least_squares(lambda q: rand_abstand(q, P), start, loss="soft_l1", f_scale=0.005)
    return erg.x, float(np.sqrt(np.mean(rand_abstand(erg.x, P) ** 2)))


def flaeche_schwerpunkt_richtung(P):
    """Schwerpunkt und Hauptachse (Winkel) einer Punktmenge."""
    m = P.mean(0)
    u, s, vt = np.linalg.svd(P - m, full_matrices=False)
    return m, math.atan2(vt[0, 1], vt[0, 0])


def auswerten(farbe, tiefe, K, q, t):
    """-> dict mit c, yaw, z_boden, proben, kontur_px, maske  oder None."""
    fx, fy, cx, cy = K
    h, w = tiefe.shape
    vs, us = np.mgrid[0:h, 0:w]
    z = tiefe * 1e-3
    gut = np.isfinite(z) & (z > 0.15) & (z < 2.0)
    p_opt = np.stack([(us - cx) * z / fx, (vs - cy) * z / fy, z], -1).reshape(-1, 3)
    p_base = qrot(q, np.nan_to_num(p_opt)) + t
    X, Y, Z = (p_base[:, i].reshape(h, w) for i in range(3))
    x0, x1, y0, y1 = a.bereich
    im_bereich = gut & (X >= x0) & (X <= x1) & (Y >= y0) & (Y <= y1)
    # Plattenhoehe: Median der Tiefe im Bereich (die Schale ist klein gegen die Platte)
    z_platte = float(np.median(Z[im_bereich]))
    im_bereich &= (Z - z_platte >= a.z[0]) & (Z - z_platte <= a.z[1])
    if im_bereich.sum() < 2000:
        print("ABBRUCH: nur %d Bildpunkte im Bereich - Kamera/TF/Bereich pruefen." % im_bereich.sum())
        return None
    # Loecher im Tiefenbild (Wand!) innerhalb des Bereichs mitnehmen: Bereich schliessen
    bereich = cv2.morphologyEx(im_bereich.astype(np.uint8), cv2.MORPH_CLOSE,
                               np.ones((15, 15), np.uint8)).astype(bool)

    grau = cv2.cvtColor(farbe, cv2.COLOR_BGR2GRAY)
    schwelle, _ = cv2.threshold(grau[bereich], 0, 255, cv2.THRESH_BINARY + cv2.THRESH_OTSU)
    hell = (grau > schwelle) & bereich
    k = np.ones((a.oeffnen, a.oeffnen), np.uint8)
    hell = cv2.morphologyEx(hell.astype(np.uint8), cv2.MORPH_OPEN, k)
    hell = cv2.morphologyEx(hell, cv2.MORPH_CLOSE, k)
    n, lab, stats, _ = cv2.connectedComponentsWithStats(hell, 8)
    if n < 2:
        print("ABBRUCH: keine helle Flaeche im Bereich (Otsu-Schwelle %.0f)." % schwelle)
        return None
    idx = 1 + int(np.argmax(stats[1:, cv2.CC_STAT_AREA]))
    maske = (lab == idx).astype(np.uint8)
    kont, _ = cv2.findContours(maske, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_NONE)
    kont = max(kont, key=cv2.contourArea).reshape(-1, 2)
    print("Otsu-Schwelle %.0f, helle Flaeche %d px, Umriss %d Punkte"
          % (schwelle, stats[idx, cv2.CC_STAT_AREA], len(kont)))

    # Sehstrahlen der Umrisspunkte auf die Ebene z = Oberkante schneiden.
    z_kante = z_platte + BODENDICKE + WANDHOEHE
    strahl = np.stack([(kont[:, 0] - cx) / fx, (kont[:, 1] - cy) / fy, np.ones(len(kont))], 1)
    strahl = qrot(q, strahl)                      # Richtung in robot_base
    s = (z_kante - t[2]) / strahl[:, 2]
    P = (t + strahl * s[:, None])[:, :2]
    # Flaechenpunkte ebenso (fuer Schwerpunkt/Hauptachse und Flaechenprobe)
    fy_, fx_ = np.nonzero(maske)
    st = np.stack([(fx_ - cx) / fx, (fy_ - cy) / fy, np.ones(len(fx_))], 1)
    st = qrot(q, st); F = (t + st * ((z_kante - t[2]) / st[:, 2])[:, None])[:, :2]
    flaeche = float(cv2.contourArea(P.astype(np.float32)))

    m, phi = flaeche_schwerpunkt_richtung(F)
    # Schwerpunkt eines Ringausschnitts liegt r_s vom Mittelpunkt auf der Halbierenden.
    r_s = (2.0 / 3.0) * (R_AUSSEN ** 3 - R_INNEN ** 3) / (R_AUSSEN ** 2 - R_INNEN ** 2) \
        * math.sin(WINKEL / 2) / (WINKEL / 2)
    kandidaten = []
    for dphi in (0.0, math.pi / 2, math.pi, -math.pi / 2):
        yaw0 = phi + dphi
        start = np.array([m[0] - r_s * math.cos(yaw0), m[1] - r_s * math.sin(yaw0), yaw0])
        par, rms = fit(P, start)
        kandidaten.append((rms, par))
    kandidaten.sort(key=lambda kv: kv[0])
    rms, par = kandidaten[0]
    rms2 = kandidaten[1][0]
    par[2] = (par[2] + math.pi) % (2 * math.pi) - math.pi
    d = rand_abstand(par, P)
    # Deckung: Anteil des Formrands mit einem Umrisspunkt naeher als 6 mm
    R = form_rand(par[:2], par[2], n=80)
    dd = np.min(np.hypot(R[:, None, 0] - P[None, :, 0], R[:, None, 1] - P[None, :, 1]), 1)
    deckung = float((dd < 0.006).mean())
    return dict(c=par[:2], yaw=par[2], z_boden=z_platte + BODENDICKE, z_platte=z_platte, rms=rms, rms2=rms2,
                deckung=deckung, flaeche=flaeche, anteil_grob=float((d > 0.008).mean()),
                kontur_px=kont, maske=maske, P=P)


def form_ecken(c, yaw):
    """Die vier Ecken des Ringausschnitts (4,2)."""
    e = []
    for r in (R_INNEN, R_AUSSEN):
        for s_ in (-1.0, 1.0):
            ww = yaw + s_ * WINKEL / 2
            e.append([c[0] + r * math.cos(ww), c[1] + r * math.sin(ww)])
    return np.array(e)


def auswerten_ecken(px, tiefe, K, q, t):
    """Vier geklickte Ecken (px, (4,2)) -> Lage. Plattenhoehe aus dem Tiefenbild im Bereich."""
    fx, fy, cx, cy = K
    h, w = tiefe.shape
    vs, us = np.mgrid[0:h, 0:w]
    z = tiefe * 1e-3
    gut = np.isfinite(z) & (z > 0.15) & (z < 2.0)
    p_opt = np.stack([(us - cx) * z / fx, (vs - cy) * z / fy, z], -1).reshape(-1, 3)
    p_base = qrot(q, np.nan_to_num(p_opt)) + t
    X, Y, Z = (p_base[:, i].reshape(h, w) for i in range(3))
    x0, x1, y0, y1 = a.bereich
    im_bereich = gut & (X >= x0) & (X <= x1) & (Y >= y0) & (Y <= y1)
    z_platte = float(np.median(Z[im_bereich])) if im_bereich.sum() > 500 else 0.0
    z_kante = z_platte + BODENDICKE + WANDHOEHE
    px = np.asarray(px, dtype=float)
    strahl = qrot(q, np.stack([(px[:, 0] - cx) / fx, (px[:, 1] - cy) / fy, np.ones(len(px))], 1))
    P = (t + strahl * ((z_kante - t[2]) / strahl[:, 2])[:, None])[:, :2]

    def rest(par):
        E = form_ecken(par[:2], par[2])
        d = np.hypot(P[:, None, 0] - E[None, :, 0], P[:, None, 1] - E[None, :, 1])
        return d.min(1)

    from scipy.optimize import least_squares
    m, phi = flaeche_schwerpunkt_richtung(P)
    r_s = 0.5 * (R_INNEN + R_AUSSEN)
    kand = []
    for dphi in (0.0, math.pi / 2, math.pi, -math.pi / 2):
        yaw0 = phi + dphi
        start = np.array([m[0] - r_s * math.cos(yaw0), m[1] - r_s * math.sin(yaw0), yaw0])
        e = least_squares(rest, start)
        kand.append((float(np.sqrt(np.mean(rest(e.x) ** 2))), e.x))
    kand.sort(key=lambda kv: kv[0])
    rms, par = kand[0]
    par[2] = (par[2] + math.pi) % (2 * math.pi) - math.pi
    E = form_ecken(par[:2], par[2])
    d = np.hypot(P[:, None, 0] - E[None, :, 0], P[:, None, 1] - E[None, :, 1])
    eindeutig = len(set(d.argmin(1).tolist())) == 4
    return dict(c=par[:2], yaw=par[2], z_boden=z_platte + BODENDICKE, z_platte=z_platte, rms=rms, rms2=kand[1][0],
                deckung=1.0 if eindeutig else 0.0, flaeche=SOLL_FLAECHE, anteil_grob=float((rest(par) > 0.008).mean()),
                kontur_px=px.astype(np.int32), maske=None, P=P, ecken_mm=rest(par) * 1e3)


def nachziehen_an_kanten(farbe, K, q, t, erg):
    """Nach dem Eckenfit: Canny-Kanten im Schlauch um den Modellrand nehmen und die Form
    daran nachziehen (robuste kleinste Quadrate), Auswahl und Fit im Wechsel (ICP-artig,
    6 Runden, Schlauch 10 mm). Die Klicks geben die Lage grob (Klickfehler 3-9 mm), die
    Kanten sind auf den Bildpunkt genau (1.0 mm/px bei 848x480 und 61 cm).

    Der OEFFNUNGSWINKEL wird dabei MITGESCHAETZT (vierte Unbekannte). Grund (2026-09-12):
    mit dem alten Modellwinkel 39.9 Grad gab es zwei Loesungen 5 Grad auseinander - eine passte
    die eine Seitenwand, eine die andere; die Schale ist an der Oberkante tatsaechlich
    rund 45 Grad weit (Boegen 148/240 mm statt 130/210). Radien stimmen (rms gleich gut
    mit festen Radien). Weicht der gemessene Winkel > 2 Grad vom Modell ab, wird gewarnt:
    dann gehoert schale_winkel in mycobot_world.urdf.xacro nachgezogen. Mittelpunkt und
    Winkelhalbierende (das, was in schale_pose.xacro steht) sind davon unabhaengig.

    Wird nur uebernommen, wenn es plausibel bleibt: Verschiebung < 25 mm / 8 Grad (der
    Eckenfit mit falschem Modellwinkel liegt systematisch ~12 mm daneben), rms < 5 mm,
    Deckung > 60 %. Sonst bleibt der Eckenfit und es gibt eine Warnung."""
    from scipy.optimize import least_squares
    fx, fy, cx, cy = K
    g = cv2.GaussianBlur(cv2.cvtColor(farbe, cv2.COLOR_BGR2GRAY), (3, 3), 0)
    kanten = cv2.Canny(g, 60, 150)
    z = erg["z_boden"] + WANDHOEHE
    ev, eu = np.nonzero(kanten > 0)
    st = qrot(q, np.stack([(eu - cx) / fx, (ev - cy) / fy, np.ones(len(eu))], 1))
    Pall = (t + st * ((z - t[2]) / st[:, 2])[:, None])[:, :2]
    par = np.array([erg["c"][0], erg["c"][1], erg["yaw"], WINKEL])
    n_sel = 0
    for _ in range(6):
        sel = rand_abstand(par, Pall) < 0.010
        n_sel = int(sel.sum())
        if n_sel < 100:
            return erg, "Nachziehen: nur %d Kantenpunkte im Schlauch - Eckenfit bleibt." % n_sel
        P = Pall[sel]
        e = least_squares(lambda v: rand_abstand(v, P), par, loss="soft_l1", f_scale=0.005,
                          bounds=([-np.inf, -np.inf, -np.inf, WINKEL - math.radians(8)],
                                  [np.inf, np.inf, np.inf, WINKEL + math.radians(10)]))
        par = e.x
    d = rand_abstand(par, Pall)
    P = Pall[d < 0.006]
    rms = float(np.sqrt(np.mean(d[d < 0.010] ** 2)))
    Rr = form_rand(par[:2], par[2], n=80, winkel=par[3])
    dcov = np.min(np.hypot(Rr[:, None, 0] - P[None, :, 0], Rr[:, None, 1] - P[None, :, 1]), 1)
    deckung = float((dcov < 0.004).mean())
    schub = float(np.hypot(*(par[:2] - erg["c"]))) * 1e3
    dreh = math.degrees(abs((par[2] - erg["yaw"] + math.pi) % (2 * math.pi) - math.pi))
    text = ("Nachziehen an %d Kantenpunkten: rms %.1f mm, Deckung %.0f%%, Verschiebung %.1f mm / %.2f Grad, "
            "Oeffnungswinkel %.1f Grad (Modell %.1f)" % (len(P), rms * 1e3, deckung * 100, schub, dreh,
                                                        math.degrees(par[3]), math.degrees(WINKEL)))
    if schub > 25.0 or dreh > 8.0 or rms > 0.005 or deckung < 0.6:
        return erg, text + " -> NICHT uebernommen (Eckenfit bleibt)."
    neu = dict(erg); neu["c"] = par[:2]; neu["yaw"] = (par[2] + math.pi) % (2 * math.pi) - math.pi
    neu["winkel"] = float(par[3])
    neu["rms"] = rms; neu["deckung"] = deckung
    neu["kanten_px"] = np.stack([eu[d < 0.006], ev[d < 0.006]], 1)
    neu["anteil_grob"] = float((d[d < 0.010] > 0.008).mean())
    neu["nachgezogen"] = text
    return neu, text + " -> uebernommen."


def rot_inv(q):
    x, y, z, w = q
    R = np.array([[1 - 2 * (y * y + z * z), 2 * (x * y - z * w), 2 * (x * z + y * w)],
                  [2 * (x * y + z * w), 1 - 2 * (x * x + z * z), 2 * (y * z - x * w)],
                  [2 * (x * z - y * w), 2 * (y * z + x * w), 1 - 2 * (x * x + y * y)]])
    return R.T


def ecken_klicken(farbe):
    """Fenster: vier Ecken anklicken. -> Liste (u, v) oder None bei q."""
    WIN = "Schale: 4 Ecken der Wand-OBERKANTE anklicken  (u=zurueck, Enter=fertig, q=Ende)"
    klicks = []

    def maus(ev, x, y, flags, _):
        if ev == cv2.EVENT_LBUTTONDOWN and len(klicks) < 4:
            klicks.append((x, y))
    cv2.namedWindow(WIN, cv2.WINDOW_AUTOSIZE)
    cv2.setMouseCallback(WIN, maus)
    while True:
        out = farbe.copy()
        for i, (x, y) in enumerate(klicks):
            cv2.circle(out, (x, y), 5, (0, 0, 255), 2)
            cv2.putText(out, str(i + 1), (x + 8, y - 8), cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0, 0, 255), 2)
        txt = "Ecke %d von 4 anklicken" % (len(klicks) + 1) if len(klicks) < 4 else "Enter = einpassen"
        cv2.putText(out, txt, (8, 24), cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0, 0, 0), 3)
        cv2.putText(out, txt, (8, 24), cv2.FONT_HERSHEY_SIMPLEX, 0.7, (255, 255, 255), 1)
        cv2.imshow(WIN, out)
        k = cv2.waitKey(30) & 0xFF
        if k in (ord("q"), 27):
            cv2.destroyWindow(WIN); return None
        if k == ord("u") and klicks:
            klicks.pop()
        if k in (13, 10) and len(klicks) == 4:
            cv2.destroyWindow(WIN); return klicks


def zeichne_overlay(farbe, K, q, t, erg):
    fx, fy, cx, cy = K
    out = farbe.copy()
    if erg.get("maske") is not None:
        cv2.drawContours(out, [erg["kontur_px"].reshape(-1, 1, 2)], -1, (255, 0, 255), 1)
    else:
        for (x, y) in erg["kontur_px"]:
            cv2.circle(out, (int(x), int(y)), 5, (255, 0, 255), 2)
    # Formrand (Oberkante gruen, Innenboden gelb) ins Bild projizieren
    Rq = rot_inv(q)
    if erg.get("kanten_px") is not None:
        out[erg["kanten_px"][:, 1], erg["kanten_px"][:, 0]] = (255, 0, 255)
    for z, farbe_ in ((erg["z_boden"] + WANDHOEHE, (0, 255, 0)), (erg["z_boden"], (0, 255, 255))):
        R = form_rand(erg["c"], erg["yaw"], n=40, winkel=erg.get("winkel"))
        P3 = np.concatenate([R, np.full((len(R), 1), z)], 1) - t
        Pc = P3 @ Rq.T
        u = (Pc[:, 0] / Pc[:, 2] * fx + cx).astype(int); v = (Pc[:, 1] / Pc[:, 2] * fy + cy).astype(int)
        for i in range(len(u)):
            cv2.circle(out, (int(u[i]), int(v[i])), 2, farbe_, -1)
    zeilen = ["Schale (Umriss): x=%.0f y=%.0f mm, %.1f Grad" % (erg["c"][0] * 1e3, erg["c"][1] * 1e3, math.degrees(erg["yaw"])),
              "rms %.1f mm  Deckung %.0f%%  Flaeche %.0f%%  2.Start rms %.1f" % (
                  erg["rms"] * 1e3, erg["deckung"] * 100, erg["flaeche"] / SOLL_FLAECHE * 100, erg["rms2"] * 1e3),
              "magenta = Umriss, gruen = Oberkante, gelb = Innenboden"]
    for i, zl in enumerate(zeilen):
        cv2.putText(out, zl, (8, 20 + 18 * i), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 0, 0), 3)
        cv2.putText(out, zl, (8, 20 + 18 * i), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (255, 255, 255), 1)
    return out


def proben(erg):
    z = []
    if erg.get("maske") is None and "ecken_mm" in erg:          # Ecken-Modus
        if erg.get("nachgezogen"):
            if abs(math.degrees(erg["winkel"] - WINKEL)) > 2.0:   # Lage bleibt gueltig, Modellmass nicht
                z.append("HINWEIS: gemessener Oeffnungswinkel %.1f Grad, Modell %.1f - schale_winkel in "
                         "mycobot_world.urdf.xacro anpassen (Lage/Halbierende sind davon unabhaengig)."
                         % (math.degrees(erg["winkel"]), math.degrees(WINKEL)))
            return z
        if erg["rms"] > 0.012:
            z.append("WARNUNG: Ecken-Restfehler %.1f mm rms - Klicks pruefen (Oberkante? richtige Ecken?)." % (erg["rms"] * 1e3))
        if erg["deckung"] < 1.0:
            z.append("WARNUNG: zwei Klicks fallen auf dieselbe Modell-Ecke.")
        if erg["rms2"] < 3.0 * erg["rms"]:
            z.append("WARNUNG: zweitbeste Richtung rms %.1f mm gegen %.1f - Richtung nicht eindeutig." % (erg["rms2"] * 1e3, erg["rms"] * 1e3))
        return z
    if erg["rms"] > 0.006:
        z.append("WARNUNG: Restfehler %.1f mm rms - der Umriss passt nicht zur Form (Verdeckung? falsche Flaeche?)." % (erg["rms"] * 1e3))
    if erg["deckung"] < 0.8:
        z.append("WARNUNG: nur %.0f%% des Formrands sind mit Umrisspunkten belegt." % (erg["deckung"] * 100))
    fa = erg["flaeche"] / SOLL_FLAECHE
    if not 0.8 < fa < 1.25:
        z.append("WARNUNG: helle Flaeche = %.0f%% der Schalenflaeche - das ist wohl nicht (nur) die Schale." % (fa * 100))
    if erg["rms2"] < 1.5 * erg["rms"]:
        z.append("WARNUNG: zweitbeste Richtung fast gleich gut (rms %.1f gegen %.1f mm) - Gierwinkel unsicher." % (erg["rms2"] * 1e3, erg["rms"] * 1e3))
    return z


def hole_frame(knoten):
    """Naechsten gemittelten Frame vom Knoten holen -> (farbe, tiefe, K, q, t) oder None."""
    import rclpy
    knoten.tiefen = []
    ende = knoten.get_clock().now().nanoseconds + int(30e9)
    while rclpy.ok() and not (knoten.bereit() and knoten.farbbild is not None) \
            and knoten.get_clock().now().nanoseconds < ende:
        rclpy.spin_once(knoten, timeout_sec=0.2)
    if not (knoten.bereit() and knoten.farbbild is not None):
        return None
    ende = knoten.get_clock().now().nanoseconds + int(10e9)
    while rclpy.ok() and knoten.get_clock().now().nanoseconds < ende:
        if knoten.puffer.can_transform(a.frame, knoten.depth_frame, rclpy.time.Time()):
            break
        rclpy.spin_once(knoten, timeout_sec=0.2)
    tf = knoten.puffer.lookup_transform(a.frame, knoten.depth_frame, rclpy.time.Time())
    q = (tf.transform.rotation.x, tf.transform.rotation.y, tf.transform.rotation.z, tf.transform.rotation.w)
    t = np.array([tf.transform.translation.x, tf.transform.translation.y, tf.transform.translation.z])
    st = np.stack(knoten.tiefen).astype(np.float32); st[st == 0] = np.nan
    import warnings
    with warnings.catch_warnings():
        warnings.simplefilter("ignore", RuntimeWarning)
        tiefe = np.nanmedian(st, axis=0)
    farbe = knoten.farbbild
    if farbe.shape[:2] != tiefe.shape:
        print("ABBRUCH: Farbbild %s und Tiefe %s haben verschiedene Groessen." % (farbe.shape[:2], tiefe.shape))
        return None
    return farbe, tiefe, knoten.K, q, t


def ecken_auswerten_frame(klicks, farbe, tiefe, K, q, t, laut=True):
    """Eckenfit + Nachziehen fuer EINEN Frame. -> (erg, warnungen_der_ecken)"""
    erg = auswerten_ecken(klicks, tiefe, K, q, t)
    if laut:
        print("Ecken-Restfehler [mm]: " + "  ".join("%.1f" % v for v in erg["ecken_mm"]))
    warn_ecken = proben(erg)
    if laut:
        for w_ in warn_ecken:            # nur Hinweis - das Nachziehen entscheidet (eigene Proben)
            print(w_)
    erg, text = nachziehen_an_kanten(farbe, K, q, t, erg)
    if laut:
        print(text)
    return erg


def main():
    if a.npz:
        d = np.load(a.npz, allow_pickle=True)
        farbe, tiefe, K, q, t = d["farbe"], d["tiefe"], tuple(d["K"]), tuple(d["q"]), np.array(d["t"])
        knoten = None
    else:
        import rclpy
        sf = _sf()
        sf.a.depth, sf.a.info, sf.a.farbe, sf.a.mittelung = a.depth, a.info, a.farbe, a.mittelung
        rclpy.init(); knoten = sf.Finder()
        print("warte auf Farb-/Tiefenbild und camera_info ...")
        fr = hole_frame(knoten)
        if fr is None:
            print("ABBRUCH: kein Bild. Laeuft die Kamera?"); return 1
        farbe, tiefe, K, q, t = fr

    if a.ecken or a.klicks:
        klicks = [tuple(a.klicks[i:i + 2]) for i in range(0, 8, 2)] if a.klicks else ecken_klicken(farbe)
        if klicks is None:
            print("abgebrochen."); return 1
        print("Klicks (fuer --klicks): " + " ".join("%d %d" % k for k in klicks))
        erg = ecken_auswerten_frame(klicks, farbe, tiefe, K, q, t)
        # Mehrere Frames: jeder einzeln nachgezogen, dann Median (Bild-/Tiefenrauschen
        # zwischen zwei Frames machte 2026-09-12 rund 4 mm / 1 Grad aus).
        if knoten is not None and a.wiederholen > 1 and erg.get("nachgezogen"):
            reihe = [erg]
            for i in range(a.wiederholen - 1):
                fr = hole_frame(knoten)
                if fr is None:
                    break
                e_i = ecken_auswerten_frame(klicks, *fr, laut=False)
                if e_i.get("nachgezogen"):
                    reihe.append(e_i)
            if len(reihe) > 1:
                C = np.array([e["c"] for e in reihe]) * 1e3
                Y = np.degrees(np.unwrap([e["yaw"] for e in reihe]))
                W = np.degrees([e["winkel"] for e in reihe])
                for e in reihe:
                    print("  Frame: c=(%.1f, %.1f) yaw %.2f Oeffnung %.1f rms %.1f" % (e["c"][0] * 1e3, e["c"][1] * 1e3, math.degrees(e["yaw"]), math.degrees(e["winkel"]), e["rms"] * 1e3))
                print("%d Frames: Streuung Mittelpunkt %.1f/%.1f mm, Gierwinkel %.2f Grad, Oeffnung %.2f Grad (Spannweite)"
                      % (len(reihe), np.ptp(C[:, 0]), np.ptp(C[:, 1]), np.ptp(Y), np.ptp(W)))
                erg = dict(erg)
                erg["c"] = np.median(C, 0) / 1e3
                erg["yaw"] = (math.radians(float(np.median(Y))) + math.pi) % (2 * math.pi) - math.pi
                erg["winkel"] = math.radians(float(np.median(W)))
                erg["rms"] = float(np.median([e["rms"] for e in reihe]))
                erg["deckung"] = float(np.median([e["deckung"] for e in reihe]))
                erg["z_boden"] = float(np.median([e["z_boden"] for e in reihe]))
                erg["z_platte"] = float(np.median([e["z_platte"] for e in reihe]))
                erg["nachgezogen"] += " | Median aus %d Frames, Spannweite %.1f/%.1f mm, %.2f Grad" % (len(reihe), np.ptp(C[:, 0]), np.ptp(C[:, 1]), np.ptp(Y))
    else:
        erg = auswerten(farbe, tiefe, K, q, t)
    if erg is None:
        return 1
    warn = proben(erg)
    print("=" * 70 + "\nGEFUNDEN (%s)\n" % ("4 Ecken + Kanten" if (a.ecken or a.klicks) else "Umriss im Farbbild") + "=" * 70)
    print("Mittelpunkt   : x=%.1f mm  y=%.1f mm" % (erg["c"][0] * 1e3, erg["c"][1] * 1e3))
    print("Innenboden z  : %.1f mm  (Platte %.1f mm aus dem Tiefenbild + 2 mm Boden)" % (erg["z_boden"] * 1e3, erg["z_platte"] * 1e3))
    print("Gierwinkel    : %.2f Grad" % math.degrees(erg["yaw"]))
    if erg.get("winkel"):
        print("Oeffnung      : %.2f Grad gemessen (Modell %.1f)" % (math.degrees(erg["winkel"]), math.degrees(WINKEL)))
    print("-" * 70)
    print("Restfehler    : rms %.1f mm  (%.0f%% der Umrisspunkte > 8 mm daneben)" % (erg["rms"] * 1e3, erg["anteil_grob"] * 100))
    print("Deckung       : %.0f%% des Formrands" % (erg["deckung"] * 100))
    print("Flaeche       : %.0f%% der Sollflaeche" % (erg["flaeche"] / SOLL_FLAECHE * 100))
    print("2. Richtung   : rms %.1f mm" % (erg["rms2"] * 1e3))
    print("-" * 70)
    for w_ in warn:
        print(w_)
    out = zeichne_overlay(farbe, K, q, t, erg)
    if a.bild:
        cv2.imwrite(a.bild, out); print("Overlay: %s" % a.bild)
    if knoten is not None:
        from sensor_msgs.msg import Image
        msg = Image(); msg.height, msg.width = out.shape[:2]; msg.encoding = "bgr8"
        msg.step = out.shape[1] * 3; msg.data = out.tobytes(); msg.header.frame_id = "camera_color_optical_frame"
        knoten.overlay.publish(msg)
        sf = _sf()
        sf.zeichne(knoten, erg["c"], erg["yaw"], erg["z_boden"],
                   "Schale (Umriss): %.0f/%.0f mm, %.1f Grad" % (erg["c"][0] * 1e3, erg["c"][1] * 1e3, math.degrees(erg["yaw"])))
        print("Gezeichnet: /schale/erkennung (RViz) und /schale/overlay (Kamerabild).")

    if a.nur_zeigen:
        print("nichts geschrieben (--nur-zeigen)."); return 0
    if warn and not a.schreiben:
        print("ZWEIFELHAFT - nicht geschrieben. Mit --schreiben erzwingen, wenn das Overlay stimmt."); return 2
    if not a.schreiben:
        if a.stumm:
            print("nichts geschrieben (--stumm ohne --schreiben)."); return 0
        try:
            antwort = input("Umriss im Overlay pruefen. Schreiben? [j/N] ")
        except EOFError:
            antwort = ""
        if antwort.strip().lower() not in ("j", "ja", "y", "yes"):
            print("nichts geschrieben."); return 0
    sf = _sf()
    verfahren = ("4 Ecken geklickt" + (", an Kanten nachgezogen" if erg.get("nachgezogen") else "")
                 if (a.ecken or a.klicks) else "Umriss im Farbbild")
    bericht = ("  Verfahren (%s): Schnitt der Sehstrahlen mit der Ebene Oberkante,\n"
               "  Form mit festem Radius eingepasst. rms %.1f mm, Deckung %.0f%%, Flaeche %.0f%%.\n"
               "  Kamera %s, Plattenhoehe %.1f mm." % (verfahren, erg["rms"] * 1e3, erg["deckung"] * 100,
                                                       erg["flaeche"] / SOLL_FLAECHE * 100,
                                                       "npz" if a.npz else a.depth, erg["z_platte"] * 1e3))
    if erg.get("winkel"):
        bericht += "\n  Oeffnungswinkel gemessen: %.2f Grad (Modell %.1f)" % (math.degrees(erg["winkel"]), math.degrees(WINKEL))
    if erg.get("nachgezogen"):
        bericht += "\n  " + erg["nachgezogen"]
    if warn:
        bericht += "\n  TROTZ WARNUNG geschrieben (--schreiben):\n" + "\n".join("    " + w_ for w_ in warn)
    sf.schreibe(float(erg["c"][0]), float(erg["c"][1]), float(erg["z_boden"]), float(erg["yaw"]), bericht)
    print("geschrieben: %s  -> SCHALE_MONTIERT=1 und Stack neu starten." % sf.POSE_XACRO)
    return 0


if __name__ == "__main__":
    sys.exit(main())
