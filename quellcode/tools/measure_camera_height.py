#!/usr/bin/env python3
"""Misst die WIRKLICHE Kamerahoehe ueber der Plattform — direkt aus der Tiefenkarte.

Verfahren
---------
Die Plattformoberkante ist im URDF exakt die Ebene z = 0 des Frames robot_base
(platform_top: Mitte z = -0.0125, Dicke 0.025 -> Oberkante 0.000). Der Abstand
eines Punktes zu einer WAAGERECHTEN Ebene ist seine Hoehe ueber dieser Ebene —
unabhaengig davon, wie die Kamera gedreht ist. Also:

    1. Tiefenbilder mitteln (Median ueber viele Frames, Rauschen raus)
    2. alle gueltigen Pixel in den optischen 3D-Frame deprojizieren
    3. RANSAC-Ebene durch die dominante Flaeche (= Plattform) legen
    4. LOTRECHTER Abstand Kamerazentrum -> Ebene = KAMERAHOEHE

Der Kippwinkel der Ebene gegen die optische Achse faellt als Nebenprodukt ab und
zeigt, wie stark die Kamera geneigt montiert ist.

WICHTIG: Das ersetzt KEINE Hand-Auge-Kalibrierung. Gemessen wird nur die Hoehe
(und die Neigung gegen die Plattform). Die x/y-Lage und die volle Orientierung
der Kamera kommen weiterhin nur aus easy_handeye2 (Park/Horaud).

Aufruf:  python3 tools/measure_camera_height.py [--frames 40] [--profile 848x480x15]
"""
import argparse
import sys

import numpy as np

try:
    import pyrealsense2 as rs
except ImportError:
    sys.exit("FEHLER: pyrealsense2 fehlt.")

def urdf_z():
    """Aktuell im Modell eingetragene Kamerahoehe — aus der EINEN Quelle lesen.

    Frueher stand hier eine feste Zahl, die nach jedem Kamera-Umbau von Hand
    nachgezogen werden musste (und dabei veraltete). Jetzt kommt sie aus
    src/mycobot_world/urdf/camera_pose.xacro, das tools/auto_camera_calibration.py
    schreibt."""
    import os, re
    f = os.path.expanduser("~/ros2_ws/src/mycobot_world/urdf/camera_pose.xacro")
    try:
        m = re.search(r'name="cam_z"\s+value="([-0-9.eE+]+)"', open(f).read())
        if m:
            return float(m.group(1))
    except Exception:
        pass
    return float("nan")


URDF_Z = urdf_z()


def grab_depth(frames, profile, warmup=30):
    """Median-Tiefenbild (mm, float32) + Intrinsics holen."""
    w, h, fps = (int(x) for x in profile.split("x"))
    pipe = rs.pipeline()
    cfg = rs.config()
    cfg.enable_stream(rs.stream.depth, w, h, rs.format.z16, fps)
    prof = pipe.start(cfg)
    try:
        intr = prof.get_stream(rs.stream.depth).as_video_stream_profile().get_intrinsics()
        scale = prof.get_device().first_depth_sensor().get_depth_scale()
        print("Tiefenskala: %.6f m/Einheit" % scale)
        for _ in range(warmup):                    # Auto-Exposure einschwingen lassen
            pipe.wait_for_frames()
        stack = []
        for i in range(frames):
            f = pipe.wait_for_frames().get_depth_frame()
            if not f:
                continue
            stack.append(np.asanyarray(f.get_data()).astype(np.float32))
            if (i + 1) % 10 == 0:
                print("  %d/%d Frames" % (i + 1, frames), flush=True)
    finally:
        pipe.stop()
    if not stack:
        sys.exit("FEHLER: keine Tiefenbilder erhalten.")
    a = np.stack(stack)
    a[a == 0] = np.nan                              # 0 = ungueltig
    med = np.nanmedian(a, axis=0)
    return med * scale, intr                        # Meter


def to_points(depth_m, intr, step=3):
    """Gueltige Pixel -> Nx3 im optischen Frame (Z vorne, X rechts, Y unten)."""
    h, w = depth_m.shape
    vs, us = np.mgrid[0:h:step, 0:w:step]
    z = depth_m[::step, ::step]
    ok = np.isfinite(z) & (z > 0.05) & (z < 3.0)
    us, vs, z = us[ok], vs[ok], z[ok]
    x = (us - intr.ppx) * z / intr.fx
    y = (vs - intr.ppy) * z / intr.fy
    return np.column_stack([x, y, z]).astype(np.float64)


def fit_plane_ransac(pts, thresh=0.005, iters=400, seed=0):
    """Dominante Ebene: n.p + d = 0 mit |n| = 1. Rueckgabe (n, d, inlier-Maske)."""
    rng = np.random.default_rng(seed)
    best_n, best_d, best_in = None, None, None
    best_cnt = -1
    n_pts = pts.shape[0]
    for _ in range(iters):
        idx = rng.choice(n_pts, 3, replace=False)
        p0, p1, p2 = pts[idx]
        nrm = np.cross(p1 - p0, p2 - p0)
        ln = np.linalg.norm(nrm)
        if ln < 1e-9:
            continue
        nrm = nrm / ln
        d = -float(nrm @ p0)
        dist = np.abs(pts @ nrm + d)
        inl = dist < thresh
        cnt = int(inl.sum())
        if cnt > best_cnt:
            best_cnt, best_n, best_d, best_in = cnt, nrm, d, inl
    # Nachjustieren: kleinste Quadrate ueber alle Inlier (Schwerpunkt + kleinster Eigenvektor)
    q = pts[best_in]
    c = q.mean(axis=0)
    _, _, vt = np.linalg.svd(q - c, full_matrices=False)
    nrm = vt[2] / np.linalg.norm(vt[2])
    d = -float(nrm @ c)
    dist = np.abs(pts @ nrm + d)
    return nrm, d, dist < thresh


def plane_extent_cm(pts_inl, n):
    """Physische Ausdehnung der Inlier IN der Ebene (Laenge x Breite, cm)."""
    c = pts_inl.mean(axis=0)
    q = pts_inl - c
    q = q - np.outer(q @ n, n)              # in die Ebene projizieren
    _, _, vt = np.linalg.svd(q, full_matrices=False)
    a = q @ vt[0]
    b = q @ vt[1]
    # ROBUSTE Ausdehnung: 2..98-Perzentil statt max-min. Einzelne versprengte
    # Inlier am Rand (Kistenkante, Reflexion) blaehen max-min sonst um 20-30 % auf
    # und die Formzuordnung waehlt die falsche Ebene.
    return (100 * float(np.percentile(a, 98) - np.percentile(a, 2)),
            100 * float(np.percentile(b, 98) - np.percentile(b, 2)))


def analyse(pts, thresh, max_planes=4):
    """Ebenen nacheinander abschaelen. Liste von dicts, nach Punktzahl sortiert."""
    out = []
    rest = pts.copy()
    total = pts.shape[0]
    for k in range(max_planes):
        if rest.shape[0] < 3000:
            break
        n, d, inl = fit_plane_ransac(rest, thresh=thresh, seed=k)
        cnt = int(inl.sum())
        if cnt < 2000:
            break
        q = rest[inl]
        L, B = plane_extent_cm(q, n)
        out.append({
            "hoehe": abs(d), "n": n, "punkte": cnt, "anteil": cnt / total,
            "laenge_cm": L, "breite_cm": B,
            "kipp": np.degrees(np.arccos(min(1.0, abs(float(n[2]))))),
            "rms_mm": 1000 * float(np.sqrt((np.abs(q @ n + d) ** 2).mean())),
        })
        rest = rest[~inl]
    return out


def pick_platform(planes, soll=(50.0, 40.0), max_kipp=5.0):
    """Die Ebene waehlen, deren Ausdehnung am besten zur bekannten Plattform passt.

    Die Tischplatte ist groesser als das Bild und faellt ueber den Formfehler heraus;
    die Saeule und schraege Flaechen fallen ueber max_kipp heraus. max_kipp ist bewusst
    eng (5 Grad): die Plattform steht waagerecht, alles deutlich Schraegere ist etwas
    anderes und hat in einem Durchlauf faelschlich gewonnen."""
    best, best_err = None, None
    for pl in planes:
        if pl["kipp"] > max_kipp:
            continue
        L, B = sorted((pl["laenge_cm"], pl["breite_cm"]), reverse=True)
        err = abs(L - soll[0]) / soll[0] + abs(B - soll[1]) / soll[1]
        if best_err is None or err < best_err:
            best, best_err = pl, err
    return best, best_err


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--frames", type=int, default=40)
    ap.add_argument("--profile", default="848x480x15")
    ap.add_argument("--thresh", type=float, default=0.005, help="RANSAC-Inlier-Schwelle (m)")
    ap.add_argument("--repeat", type=int, default=1, help="Messung n-mal wiederholen")
    ap.add_argument("--platform-cm", default="50x40", help="bekannte Plattformgroesse LxB in cm")
    a = ap.parse_args()

    soll = tuple(float(x) for x in a.platform_cm.split("x"))
    print("=== Kamerahoehe messen (Ebenen in der Tiefenkarte) ===")
    print("Profil: %s, %d Frames, %d Durchlauf(e), Plattform-Soll %.0fx%.0f cm\n"
          % (a.profile, a.frames, a.repeat, soll[0], soll[1]))

    ergebnisse = []
    for run in range(a.repeat):
        if a.repeat > 1:
            print("---------- Durchlauf %d/%d ----------" % (run + 1, a.repeat))
        depth_m, intr = grab_depth(a.frames, a.profile)
        pts = to_points(depth_m, intr)
        if pts.shape[0] < 2000:
            sys.exit("FEHLER: zu wenige gueltige Tiefenpunkte — Sicht frei? Beleuchtung?")
        planes = analyse(pts, a.thresh)
        print("  gueltige Punkte: %d,  %d Ebene(n)" % (pts.shape[0], len(planes)))
        for i, pl in enumerate(planes, 1):
            print("   Ebene %d: %6.1f mm | %5d Pkt (%4.1f%%) | %5.1f x %5.1f cm | "
                  "Kipp %5.2f Grad | RMS %.2f mm"
                  % (i, 1000 * pl["hoehe"], pl["punkte"], 100 * pl["anteil"],
                     pl["laenge_cm"], pl["breite_cm"], pl["kipp"], pl["rms_mm"]))
        plat, err = pick_platform(planes, soll)
        if plat is None:
            sys.exit("FEHLER: keine waagerechte Ebene gefunden.")
        print("   -> als PLATTFORM gewaehlt: %.1f mm (%.1f x %.1f cm, Formfehler %.2f)\n"
              % (1000 * plat["hoehe"], plat["laenge_cm"], plat["breite_cm"], err))
        ergebnisse.append(plat["hoehe"])

    h = np.array(ergebnisse)
    print("=== ERGEBNIS ===")
    if a.repeat > 1:
        print("Durchlaeufe: " + ", ".join("%.1f" % (1000 * x) for x in h) + " mm")
        print("Streuung   : %.2f mm (max-min)" % (1000 * (h.max() - h.min())))
    hoehe = float(np.median(h))
    print("GEMESSENE KAMERAHOEHE ueber der Plattform : %.4f m  (%.1f mm)" % (hoehe, 1000 * hoehe))
    print("Im URDF eingetragen (sensor-Origin)       : %.4f m  (%.1f mm)" % (URDF_Z, 1000 * URDF_Z))
    print("Abweichung                                : %+.4f m  (%+.1f mm)"
          % (hoehe - URDF_Z, 1000 * (hoehe - URDF_Z)))
    print()
    print("HINWEIS: gemessen wird die HOEHE ueber der Plattformebene. Die x/y-Lage und die")
    print("         volle Orientierung liefert weiterhin nur die Hand-Auge-Kalibrierung.")


if __name__ == "__main__":
    main()
