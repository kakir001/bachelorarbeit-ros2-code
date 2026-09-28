#!/usr/bin/env python3
"""Misst die VOLLE 3D-Pose der Kamera ueber der bekannten Plattform-Platte.

Idee
----
Die Grundplatte (Baseboard) ist im URDF exakt bekannt: ein Rechteck von
50 x 40 cm, dessen Mitte im Frame robot_base bei (0.15, -0.125) liegt und dessen
Oberkante die Ebene z = 0 bildet. Ein bekanntes Rechteck in der Tiefenkarte
liefert damit ALLE sechs Freiheitsgrade der Kamera:

    Ebenennormale       -> 2 Rotations-DOF (Neigung/Rollen)
    Rechteck-Richtung   -> 1 Rotations-DOF (Gierwinkel)
    Rechteck-Mittelpunkt-> 2 Translations-DOF (x, y)
    Ebenenabstand       -> 1 Translations-DOF (z, = Hoehe)

Ablauf
------
    1. Tiefenbilder mitteln (Median ueber viele Frames)
    2. gueltige Pixel in den optischen 3D-Frame deprojizieren
    3. Ebenen nacheinander abschaelen (RANSAC), waagerechte Kandidaten behalten
    4. je Kandidat: Inlier-Maske im Bild -> groesste zusammenhaengende Flaeche
       (loest die Platte von gleich hohen Fremdflaechen im Bild ab)
    5. minAreaRect in der Ebene -> Mittelpunkt, Laengs-/Querachse, Abmessungen
    6. Abmessungen gegen 50 x 40 cm pruefen -> beste Ebene ist die Platte
    7. Pose zurueckrechnen; die 180-Grad-Mehrdeutigkeit des Rechtecks wird ueber
       die Lage des Roboters auf der Platte aufgeloest (der Roboter verdeckt die
       Platte an einer BEKANNTEN Stelle: robot_base liegt bei Platten-Koordinate
       (-0.15, +0.125)), zusaetzlich gegen die aktuelle URDF-Pose geprueft.

WICHTIG: Das ist eine geometrische MESSUNG gegen eine bekannte Referenzflaeche,
KEINE Hand-Auge-Kalibrierung. Sie ist deutlich besser als das blosse Verschieben
der alten Kalibrierung in z (dabei bleiben x/y/Rotation eines alten Aufbaus
stehen), ersetzt aber easy_handeye2 (Park/Horaud) nicht: sie kennt nur die
Plattform, nicht die Roboter-Kinematik.

Aufruf:
    python3 tools/measure_camera_pose.py --repeat 3
    python3 tools/measure_camera_pose.py --repeat 3 --json /tmp/pose.json --debug-png /tmp/pose.png
"""
import argparse
import json
import os
import subprocess
import sys
import xml.etree.ElementTree as ET

import numpy as np

try:
    import pyrealsense2 as rs
except ImportError:
    sys.exit("FEHLER: pyrealsense2 fehlt.")

try:
    import cv2
except ImportError:
    sys.exit("FEHLER: OpenCV (cv2) fehlt — wird fuer minAreaRect/Komponenten gebraucht.")

WS = os.path.expanduser("~/ros2_ws")
XACRO = os.path.join(WS, "src/mycobot_world/urdf/mycobot_world.urdf.xacro")

# Rueckfallwerte, falls das URDF nicht expandiert werden kann (ROS nicht gesourct).
# Quelle: mycobot_world.urdf.xacro — platform_top box 0.50x0.40x0.025,
# joint robot_base->platform_top origin xyz="0.15 -0.125 -0.0125".
FALLBACK_PLATE_SIZE = (0.50, 0.40)
FALLBACK_PLATE_CENTER = (0.15, -0.125)
# T_camera_bottom_screw <- camera_depth_optical_frame (realsense2_description D435i)
FALLBACK_T_BS_DEPTH = np.array([[0.0, 0.0, 1.0, 0.0106],
                                [-1.0, 0.0, 0.0, 0.0175],
                                [0.0, -1.0, 0.0, 0.0125],
                                [0.0, 0.0, 0.0, 1.0]])


# --------------------------------------------------------------------------
# URDF: Referenzgeometrie + Kamera-Innenkette aus der EINEN Quelle lesen
# --------------------------------------------------------------------------
def rpy_to_R(r, p, y):
    cr, sr = np.cos(r), np.sin(r)
    cp, sp = np.cos(p), np.sin(p)
    cy, sy = np.cos(y), np.sin(y)
    return np.array([[cy * cp, cy * sp * sr - sy * cr, cy * sp * cr + sy * sr],
                     [sy * cp, sy * sp * sr + cy * cr, sy * sp * cr - cy * sr],
                     [-sp,     cp * sr,                cp * cr]])


def load_urdf_reference():
    """Plattengeometrie + T(bottom_screw<-depth_optical) + aktuelle Kamera-Pose."""
    ref = {"quelle": "URDF (xacro)"}
    try:
        urdf = subprocess.run(["xacro", XACRO, "use_fake_hardware:=true"],
                              capture_output=True, text=True, timeout=120, check=True).stdout
        root = ET.fromstring(urdf)
    except Exception as e:
        print("WARNUNG: URDF konnte nicht expandiert werden (%s) — Rueckfallwerte." % e)
        return {"quelle": "Rueckfallwerte (URDF nicht lesbar)",
                "plate_size": FALLBACK_PLATE_SIZE, "plate_center": FALLBACK_PLATE_CENTER,
                "T_bs_depth": FALLBACK_T_BS_DEPTH, "T_rb_bs_alt": None, "T_rb_depth_alt": None}

    joints = {}
    for j in root.findall("joint"):
        child = j.find("child").get("link")
        o = j.find("origin")
        xyz = [0.0, 0.0, 0.0]
        rpy = [0.0, 0.0, 0.0]
        if o is not None:
            if o.get("xyz"):
                xyz = [float(v) for v in o.get("xyz").split()]
            if o.get("rpy"):
                rpy = [float(v) for v in o.get("rpy").split()]
        T = np.eye(4)
        T[:3, :3] = rpy_to_R(*rpy)
        T[:3, 3] = xyz
        joints[child] = (j.find("parent").get("link"), T)

    def T_from_to(anc, desc):
        T = np.eye(4)
        cur = desc
        while cur in joints:
            parent, Tj = joints[cur]
            T = Tj @ T
            cur = parent
            if cur == anc:
                return T
        raise RuntimeError("%s ist kein Vorfahre von %s" % (anc, desc))

    # Plattenmasse aus dem platform_top-Link
    size = FALLBACK_PLATE_SIZE
    for lk in root.findall("link"):
        if lk.get("name") == "platform_top":
            box = lk.find("./collision/geometry/box")
            if box is None:
                box = lk.find("./visual/geometry/box")
            if box is not None:
                s = [float(v) for v in box.get("size").split()]
                size = (s[0], s[1])
    T_rb_plate = T_from_to("robot_base", "platform_top")
    ref["plate_size"] = size
    ref["plate_center"] = (float(T_rb_plate[0, 3]), float(T_rb_plate[1, 3]))
    ref["T_bs_depth"] = T_from_to("camera_bottom_screw_frame", "camera_depth_optical_frame")
    ref["T_bs_color"] = T_from_to("camera_bottom_screw_frame", "camera_color_optical_frame")
    ref["T_rb_bs_alt"] = T_from_to("robot_base", "camera_bottom_screw_frame")
    ref["T_rb_depth_alt"] = T_from_to("robot_base", "camera_depth_optical_frame")
    return ref


# --------------------------------------------------------------------------
# Tiefenbild + Punktwolke
# --------------------------------------------------------------------------
def grab_depth(frames, profile, warmup=30):
    w, h, fps = (int(x) for x in profile.split("x"))
    pipe = rs.pipeline()
    cfg = rs.config()
    cfg.enable_stream(rs.stream.depth, w, h, rs.format.z16, fps)
    prof = pipe.start(cfg)
    try:
        intr = prof.get_stream(rs.stream.depth).as_video_stream_profile().get_intrinsics()
        scale = prof.get_device().first_depth_sensor().get_depth_scale()
        for _ in range(warmup):
            pipe.wait_for_frames()
        stack = []
        for i in range(frames):
            f = pipe.wait_for_frames().get_depth_frame()
            if f:
                stack.append(np.asanyarray(f.get_data()).astype(np.float32))
            if (i + 1) % 10 == 0:
                print("  %d/%d Frames" % (i + 1, frames), flush=True)
    finally:
        pipe.stop()
    if not stack:
        sys.exit("FEHLER: keine Tiefenbilder erhalten.")
    a = np.stack(stack)
    a[a == 0] = np.nan
    with np.errstate(invalid="ignore"):
        med = np.nanmedian(a, axis=0)
    return med * scale, intr


def to_points(depth_m, intr, step=2):
    """Gueltige Pixel -> (Nx3 im optischen Frame, Pixelzeilen, Pixelspalten)."""
    h, w = depth_m.shape
    vs, us = np.mgrid[0:h:step, 0:w:step]
    z = depth_m[::step, ::step]
    ok = np.isfinite(z) & (z > 0.05) & (z < 3.0)
    us, vs, z = us[ok], vs[ok], z[ok]
    x = (us - intr.ppx) * z / intr.fx
    y = (vs - intr.ppy) * z / intr.fy
    return np.column_stack([x, y, z]).astype(np.float64), vs, us


def fit_plane_ransac(pts, thresh=0.005, iters=400, seed=0):
    rng = np.random.default_rng(seed)
    best_in, best_cnt = None, -1
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
        inl = np.abs(pts @ nrm + d) < thresh
        cnt = int(inl.sum())
        if cnt > best_cnt:
            best_cnt, best_in = cnt, inl
    q = pts[best_in]
    c = q.mean(axis=0)
    _, _, vt = np.linalg.svd(q - c, full_matrices=False)
    nrm = vt[2] / np.linalg.norm(vt[2])
    d = -float(nrm @ c)
    return nrm, d, np.abs(pts @ nrm + d) < thresh


# --------------------------------------------------------------------------
# Rechteck-Auswertung
# --------------------------------------------------------------------------
def largest_component(mask_pix, shape, close_px=3):
    """Groesste zusammenhaengende Inlier-Flaeche im Bild -> Bool-Maske ueber die Punkte.

    Loest die Platte von gleich hohen, aber getrennten Flaechen (Tischbereiche,
    Ablageteile) ab, die die reine Ebenen-Segmentierung mitnimmt."""
    img = np.zeros(shape, np.uint8)
    img[mask_pix[0], mask_pix[1]] = 255
    k = np.ones((close_px, close_px), np.uint8)
    img = cv2.morphologyEx(img, cv2.MORPH_CLOSE, k)
    n, lab = cv2.connectedComponents(img, connectivity=8)
    if n <= 1:
        return np.ones(mask_pix[0].shape[0], bool), img
    sizes = [(lab == i).sum() for i in range(1, n)]
    best = int(np.argmax(sizes)) + 1
    keep = lab[mask_pix[0], mask_pix[1]] == best
    return keep, (lab == best).astype(np.uint8) * 255


def rect_in_plane(pts_inl, n):
    """minAreaRect in der Ebene. Rueckgabe: Mittelpunkt(3D), Achse1, Achse2, (L,B) in m."""
    c = pts_inl.mean(axis=0)
    q = pts_inl - c
    q = q - np.outer(q @ n, n)
    _, _, vt = np.linalg.svd(q, full_matrices=False)
    e1, e2 = vt[0], vt[1]
    a = q @ e1
    b = q @ e2
    pts2 = np.column_stack([a, b]).astype(np.float32) * 1000.0    # mm fuer cv2
    (cx, cy), (w, h), ang = cv2.minAreaRect(pts2)
    th = np.radians(ang)
    ax1 = np.cos(th) * e1 + np.sin(th) * e2        # Richtung der Kante "w"
    ax2 = -np.sin(th) * e1 + np.cos(th) * e2       # Richtung der Kante "h"
    # 2026-09-12: minAreaRect nimmt die AEUSSERSTEN Punkte. Ein paar Dutzend Streupunkte in
    # Plattenhoehe neben der Kante (Roboterfuss, Kabel) zogen die Platte auf 52-53.7 cm und
    # verschoben die Mitte um 1-2 cm - genau der Fehler, um den es hier geht. Deshalb die
    # Kanten aus der Punktdichte: 5-mm-Bins entlang jeder Achse, Kante = aeusserstes Bin mit
    # mindestens 25 % der Median-Belegung. Die Orientierung bleibt von minAreaRect.
    def kanten(s):
        b = 0.005
        lo, hi = float(s.min()), float(s.max())
        h_, edges = np.histogram(s, bins=np.arange(lo, hi + b, b))
        if h_.size < 4:
            return lo, hi
        schwelle = 0.25 * float(np.median(h_[h_ > 0]))
        ok = np.where(h_ >= schwelle)[0]
        return float(edges[ok[0]]), float(edges[ok[-1] + 1])
    lo1, hi1 = kanten(q @ ax1)
    lo2, hi2 = kanten(q @ ax2)
    center = c + 0.5 * (lo1 + hi1) * ax1 + 0.5 * (lo2 + hi2) * ax2
    return center, ax1, ax2, (hi1 - lo1, hi2 - lo2)


def build_pose(center, ax_long, ax_short, n_up, plate_center):
    """T_robot_base <- depth_optical aus Rechteck-Frame."""
    # Rechtshaendig machen: ax_long x ax_short = n_up
    if np.dot(np.cross(ax_long, ax_short), n_up) < 0:
        ax_short = -ax_short
    R_cam_plate = np.column_stack([ax_long, ax_short, n_up])       # Platte -> Kamera
    R = R_cam_plate.T                                              # Kamera-Rot in Plattenframe
    t = -R @ center
    T = np.eye(4)
    T[:3, :3] = R
    T[:3, 3] = t + np.array([plate_center[0], plate_center[1], 0.0])
    return T


def R_to_rpy(R):
    sy = -R[2, 0]
    sy = max(-1.0, min(1.0, sy))
    p = np.arcsin(sy)
    if abs(np.cos(p)) > 1e-6:
        r = np.arctan2(R[2, 1], R[2, 2])
        y = np.arctan2(R[1, 0], R[0, 0])
    else:
        r = np.arctan2(-R[1, 2], R[1, 1])
        y = 0.0
    return r, p, y


def R_to_quat(R):
    """(x, y, z, w)"""
    tr = R[0, 0] + R[1, 1] + R[2, 2]
    if tr > 0:
        s = np.sqrt(tr + 1.0) * 2
        w = 0.25 * s
        x = (R[2, 1] - R[1, 2]) / s
        y = (R[0, 2] - R[2, 0]) / s
        z = (R[1, 0] - R[0, 1]) / s
    elif R[0, 0] > R[1, 1] and R[0, 0] > R[2, 2]:
        s = np.sqrt(1.0 + R[0, 0] - R[1, 1] - R[2, 2]) * 2
        w = (R[2, 1] - R[1, 2]) / s
        x = 0.25 * s
        y = (R[0, 1] + R[1, 0]) / s
        z = (R[0, 2] + R[2, 0]) / s
    elif R[1, 1] > R[2, 2]:
        s = np.sqrt(1.0 + R[1, 1] - R[0, 0] - R[2, 2]) * 2
        w = (R[0, 2] - R[2, 0]) / s
        x = (R[0, 1] + R[1, 0]) / s
        y = 0.25 * s
        z = (R[1, 2] + R[2, 1]) / s
    else:
        s = np.sqrt(1.0 + R[2, 2] - R[0, 0] - R[1, 1]) * 2
        w = (R[1, 0] - R[0, 1]) / s
        x = (R[0, 2] + R[2, 0]) / s
        y = (R[1, 2] + R[2, 1]) / s
        z = 0.25 * s
    return np.array([x, y, z, w])


def quat_average(Rs):
    """Mittlere Rotation aus mehreren Messungen (Eigenvektor-Verfahren, Markley).

    Einfaches Mitteln der Matrizen liefert keine gueltige Rotation; hier wird der
    groesste Eigenvektor der Quaternionen-Streumatrix genommen. Reduziert das
    Messrauschen ueber mehrere Durchlaeufe."""
    Q = []
    for R in Rs:
        q = R_to_quat(R)
        if Q and float(q @ Q[0]) < 0:      # Vorzeichen angleichen (q und -q sind gleich)
            q = -q
        Q.append(q)
    A = np.zeros((4, 4))
    for q in Q:
        A += np.outer(q, q)
    w, v = np.linalg.eigh(A / len(Q))
    q = v[:, -1]
    return quat_to_R(q)


def quat_to_R(q):
    x, y, z, w = q / np.linalg.norm(q)
    return np.array([
        [1 - 2 * (y * y + z * z), 2 * (x * y - z * w),     2 * (x * z + y * w)],
        [2 * (x * y + z * w),     1 - 2 * (x * x + z * z), 2 * (y * z - x * w)],
        [2 * (x * z - y * w),     2 * (y * z + x * w),     1 - 2 * (x * x + y * y)]])


def ang_between(R1, R2):
    """Winkel (Grad) zwischen zwei Rotationen."""
    c = (np.trace(R1.T @ R2) - 1.0) / 2.0
    return float(np.degrees(np.arccos(max(-1.0, min(1.0, c)))))


# --------------------------------------------------------------------------
def measure_once(depth_m, intr, ref, thresh, max_planes, tol):
    pts, vv, uu = to_points(depth_m, intr)
    if pts.shape[0] < 2000:
        return None, "zu wenige gueltige Tiefenpunkte"
    L_soll, B_soll = ref["plate_size"]
    kand = []
    rest_idx = np.arange(pts.shape[0])
    for k in range(max_planes):
        if rest_idx.size < 3000:
            break
        sub = pts[rest_idx]
        n, d, inl = fit_plane_ransac(sub, thresh=thresh, seed=k)
        if int(inl.sum()) < 2000:
            break
        gid = rest_idx[inl]
        kipp = np.degrees(np.arccos(min(1.0, abs(float(n[2])))))
        if kipp <= 8.0:                                    # nur waagerechte Flaechen
            keep, comp_img = largest_component((vv[gid], uu[gid]), depth_m.shape)
            gid_k = gid[keep]
            if gid_k.size >= 1500:
                n_up = n if float(n @ (-pts[gid_k].mean(axis=0))) > 0 else -n
                center, a1, a2, (w, h) = rect_in_plane(pts[gid_k], n_up)
                dims = sorted((w, h), reverse=True)
                err = abs(dims[0] - L_soll) / L_soll + abs(dims[1] - B_soll) / B_soll
                ax_long, ax_short = (a1, a2) if w >= h else (a2, a1)
                kand.append({"n_up": n_up, "center": center, "ax_long": ax_long,
                             "ax_short": ax_short, "dims": dims, "err": err,
                             "kipp": kipp, "punkte": int(gid_k.size),
                             "hoehe": abs(d), "idx": gid_k, "comp": comp_img,
                             "rms_mm": 1000 * float(np.sqrt(((pts[gid_k] @ n + d) ** 2).mean()))})
        rest_idx = rest_idx[~inl]

    if not kand:
        return None, "keine waagerechte Ebene gefunden"
    kand.sort(key=lambda p: p["err"])
    best = kand[0]
    if best["err"] > tol:
        return None, ("Rechteck passt nicht zur Platte: %.1f x %.1f cm (Soll %.0f x %.0f), Formfehler %.2f"
                      % (100 * best["dims"][0], 100 * best["dims"][1],
                         100 * L_soll, 100 * B_soll, best["err"]))

    # 180-Grad-Mehrdeutigkeit: der Roboter steht auf der Platte bei Platten-Koordinate
    # (-0.15, +0.125) (robot_base minus Plattenmitte) und verdeckt sie dort. Die
    # Verdeckung = Punkte OBERHALB der Ebene innerhalb des Rechtecks.
    pc = np.array(ref["plate_center"])
    soll_rob = np.array([-pc[0], -pc[1]])              # robot_base im Plattenframe
    above = (pts @ best["n_up"] + (-float(best["n_up"] @ best["center"]))) > 0.02
    q = pts[above] - best["center"]
    la = q @ best["ax_long"]
    sh = q @ best["ax_short"]
    inside = (np.abs(la) < best["dims"][0] / 2) & (np.abs(sh) < best["dims"][1] / 2)
    hole = np.array([float(np.median(la[inside])), float(np.median(sh[inside]))]) \
        if int(inside.sum()) > 200 else None

    kandidaten = []
    for s1 in (1, -1):
        axl = s1 * best["ax_long"]
        axs = s1 * best["ax_short"]
        T = build_pose(best["center"], axl, axs, best["n_up"], ref["plate_center"])
        # Position des Roboters in diesem Plattenframe-Vorzeichen
        rob = np.array([s1 * 1.0, s1 * 1.0])           # nur Vorzeichen relevant
        kandidaten.append((s1, T, np.array([s1 * hole[0], s1 * hole[1]]) if hole is not None else None))

    gewaehlt = None
    grund = ""
    # 2026-09-12: ZUERST die bisherige URDF-Rotation als Prior - eine Kamera dreht sich nicht
    # aus Versehen um 180 Grad. Die Verdeckungs-Heuristik (Schwerpunkt der Punkte ueber der
    # Platte = Roboter) versagt, sobald Trichter, Schale und Ablageplatte auf der Platte
    # stehen (heute: Abstand 0.17 vs 0.23 m zu beiden Kandidaten, falsche Wahl -> 179.7 Grad
    # "Sprung", Kalibrierung verworfen). Verdeckung nur noch ohne Prior.
    if ref.get("T_rb_depth_alt") is not None:
        Ra = ref["T_rb_depth_alt"][:3, :3]
        d0 = ang_between(kandidaten[0][1][:3, :3], Ra)
        d1 = ang_between(kandidaten[1][1][:3, :3], Ra)
        if min(d0, d1) < 45.0:
            gewaehlt = kandidaten[0] if d0 <= d1 else kandidaten[1]
            grund = "naeher an bisheriger URDF-Rotation (%.1f vs %.1f Grad)" % (min(d0, d1), max(d0, d1))
    if gewaehlt is None and hole is not None:
        # Verdeckungsschwerpunkt soll bei soll_rob liegen
        d0 = np.linalg.norm(kandidaten[0][2] - soll_rob)
        d1 = np.linalg.norm(kandidaten[1][2] - soll_rob)
        gewaehlt = kandidaten[0] if d0 <= d1 else kandidaten[1]
        grund = "Roboter-Verdeckung (Abstand %.3f m vs %.3f m)" % (min(d0, d1), max(d0, d1))
    elif gewaehlt is None and ref.get("T_rb_depth_alt") is not None:
        Ra = ref["T_rb_depth_alt"][:3, :3]
        d0 = ang_between(kandidaten[0][1][:3, :3], Ra)
        d1 = ang_between(kandidaten[1][1][:3, :3], Ra)
        gewaehlt = kandidaten[0] if d0 <= d1 else kandidaten[1]
        grund = "naeher an bisheriger URDF-Rotation (%.1f vs %.1f Grad)" % (min(d0, d1), max(d0, d1))
    elif gewaehlt is None:
        gewaehlt = kandidaten[0]
        grund = "keine Aufloesung moeglich — Vorzeichen willkuerlich!"

    best["T_rb_depth"] = gewaehlt[1]
    best["vorzeichen_grund"] = grund
    best["hole"] = hole
    best["soll_rob"] = soll_rob
    best["hole_gewaehlt"] = gewaehlt[2]
    return best, None


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--frames", type=int, default=40)
    ap.add_argument("--profile", default="848x480x15")
    ap.add_argument("--thresh", type=float, default=0.005)
    ap.add_argument("--repeat", type=int, default=3)
    ap.add_argument("--max-planes", type=int, default=5)
    ap.add_argument("--tol", type=float, default=0.30, help="max. Formfehler des Rechtecks")
    ap.add_argument("--json", default="", help="Ergebnis als JSON speichern")
    ap.add_argument("--debug-png", default="", help="Inlier-/Rechteck-Bild speichern")
    a = ap.parse_args()

    ref = load_urdf_reference()
    L, B = ref["plate_size"]
    print("=== Volle Kamera-Pose gegen die bekannte Platte messen ===")
    print("Referenz: %s" % ref["quelle"])
    print("Platte  : %.0f x %.0f cm, Mitte in robot_base (%.3f, %.3f), Oberkante z = 0"
          % (100 * L, 100 * B, ref["plate_center"][0], ref["plate_center"][1]))
    print("Profil  : %s, %d Frames, %d Durchlauf(e)\n" % (a.profile, a.frames, a.repeat))

    laeufe = []
    letzte = None
    for run in range(a.repeat):
        print("---------- Durchlauf %d/%d ----------" % (run + 1, a.repeat))
        depth_m, intr = grab_depth(a.frames, a.profile)
        best, fehler = measure_once(depth_m, intr, ref, a.thresh, a.max_planes, a.tol)
        if best is None:
            print("  FEHLGESCHLAGEN: %s" % fehler)
            continue
        T = best["T_rb_depth"]
        print("  Platte erkannt: %.1f x %.1f cm | %d Pkt | Kipp %.2f Grad | RMS %.2f mm"
              % (100 * best["dims"][0], 100 * best["dims"][1], best["punkte"],
                 best["kipp"], best["rms_mm"]))
        print("  Vorzeichen aufgeloest ueber: %s" % best["vorzeichen_grund"])
        print("  depth_optical in robot_base: x=%+.4f y=%+.4f z=%+.4f m"
              % (T[0, 3], T[1, 3], T[2, 3]))
        laeufe.append(T)
        letzte = best

    if not laeufe:
        sys.exit("FEHLER: kein Durchlauf erfolgreich.")

    P = np.array([T[:3, 3] for T in laeufe])
    T_med = laeufe[int(np.argmin(np.linalg.norm(P - np.median(P, axis=0), axis=1)))]
    print("\n=== ERGEBNIS (Median-Durchlauf) ===")
    if len(laeufe) > 1:
        print("Streuung der Position: x %.1f mm, y %.1f mm, z %.1f mm (max-min)"
              % (1000 * np.ptp(P[:, 0]), 1000 * np.ptp(P[:, 1]), 1000 * np.ptp(P[:, 2])))
        winkel = [ang_between(laeufe[0][:3, :3], T[:3, :3]) for T in laeufe[1:]]
        if winkel:
            print("Streuung der Rotation : %.2f Grad (max gegen Durchlauf 1)" % max(winkel))

    T_bs = T_med @ np.linalg.inv(ref["T_bs_depth"])
    r, p, y = R_to_rpy(T_bs[:3, :3])
    print("\n-- camera_depth_optical_frame in robot_base --")
    print("   xyz = %.5f  %.5f  %.5f" % tuple(T_med[:3, 3]))
    print("\n-- URDF-Eintrag: sensor_d435i origin (= camera_bottom_screw_frame) --")
    print('   <origin xyz="%.5f %.5f %.5f" rpy="%.5f %.5f %.5f"/>' % (T_bs[0, 3], T_bs[1, 3], T_bs[2, 3], r, p, y))

    if ref.get("T_rb_bs_alt") is not None:
        A = ref["T_rb_bs_alt"]
        d = T_bs[:3, 3] - A[:3, 3]
        print("\n-- Vergleich mit dem AKTUELLEN URDF-Eintrag --")
        print("   aktuell : xyz = %.5f %.5f %.5f" % tuple(A[:3, 3]))
        print("   gemessen: xyz = %.5f %.5f %.5f" % tuple(T_bs[:3, 3]))
        print("   Differenz: dx=%+.1f mm  dy=%+.1f mm  dz=%+.1f mm | Rotation %.2f Grad"
              % (1000 * d[0], 1000 * d[1], 1000 * d[2],
                 ang_between(T_bs[:3, :3], A[:3, :3])))

    if ref.get("T_bs_color") is not None:
        T_col = T_bs @ ref["T_bs_color"]
        q = R_to_quat(T_col[:3, :3])
        print("\n-- robot_base -> camera_color_optical_frame (fuer die .calib-Datei) --")
        print("   translation: x=%.17f y=%.17f z=%.17f" % tuple(T_col[:3, 3]))
        print("   rotation   : x=%.17f y=%.17f z=%.17f w=%.17f" % tuple(q))

    if a.json:
        out = {"T_rb_depth_optical": T_med.tolist(),
               "T_rb_bottom_screw": T_bs.tolist(),
               "urdf_origin_xyz": [T_bs[0, 3], T_bs[1, 3], T_bs[2, 3]],
               "urdf_origin_rpy": [r, p, y],
               "plate_size": list(ref["plate_size"]),
               "plate_center": list(ref["plate_center"]),
               "laeufe": [T.tolist() for T in laeufe]}
        if ref.get("T_bs_color") is not None:
            T_col = T_bs @ ref["T_bs_color"]
            out["T_rb_color_optical"] = T_col.tolist()
            out["calib_translation"] = T_col[:3, 3].tolist()
            out["calib_rotation_xyzw"] = R_to_quat(T_col[:3, :3]).tolist()
        with open(a.json, "w") as f:
            json.dump(out, f, indent=2)
        print("\nJSON gespeichert: %s" % a.json)

    if a.debug_png and letzte is not None:
        cv2.imwrite(a.debug_png, letzte["comp"])
        print("Debug-Bild (erkannte Plattenflaeche): %s" % a.debug_png)

    print("\nHINWEIS: geometrische Messung gegen die Platte — KEINE Hand-Auge-Kalibrierung.")


if __name__ == "__main__":
    main()
