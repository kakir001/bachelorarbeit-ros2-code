#!/usr/bin/env python3
"""Automatische Kamera-Nachfuehrung: messen -> vergleichen -> Modell nachziehen.

Warum
-----
Jedes Mal, wenn die Kamera (oder der Stand) bewegt wird, standen bisher drei
Handgriffe an: URDF-Origin aendern, .calib-Datei aendern, Kamera-Arm nachziehen —
und bis dahin rechnete das ganze System mit falschen Koordinaten weiter. Dieses
Werkzeug macht das selbst:

    1. volle 3D-Pose der Kamera gegen die bekannte Grundplatte messen
       (tools/measure_camera_pose.py)
    2. mit dem Stand im Modell (urdf/camera_pose.xacro) vergleichen
    3. bei Abweichung ueber der Schwelle: sichern, camera_pose.xacro und die
       easy_handeye2-.calib neu schreiben, Historie fortschreiben

Weil die Kamera-Pose im Modell nur an EINER Stelle steht (camera_pose.xacro) und
mycobot_world.urdf.xacro Kamera UND Kamera-Arm daraus ableitet, genuegt das
Schreiben dieser einen Datei. Wegen --symlink-install ist KEIN Rebuild noetig,
nur ein Neustart des robot_state_publisher (also des Stacks).

Sicherheitsnetz (lieber nichts schreiben als Unsinn schreiben)
-------------------------------------------------------------
Geschrieben wird nur, wenn ALLE Pruefungen bestehen:
    * die erkannte Flaeche hat wirklich die Plattengroesse (Formfehler unter --tol)
    * mindestens --min-ok Durchlaeufe erfolgreich
    * Streuung zwischen den Durchlaeufen unter --max-streuung-mm / --max-streuung-grad
    * das Ergebnis liegt in einem plausiblen Bereich (Hoehe 0.2-1.2 m ueber der Platte)
    * der Sprung gegen den alten Stand ist nicht absurd (--max-sprung-m / --max-sprung-grad)
Faellt eine Pruefung durch, bleibt der alte Stand unveraendert stehen und das
Werkzeug meldet den Grund (Rueckgabewert 2).

GRENZE DES VERFAHRENS: gemessen wird die Kamera gegen die PLATTE. Der Roboter gilt
dabei als unveraendert auf der Platte verschraubt — er ist die Referenz, die das
Verfahren NICHT pruefen kann. Wird der ROBOTER selbst versetzt, liefert die Messung
weiterhin saubere Zahlen, die aber auf die falsche Roboterlage bezogen sind. Dann
ist eine echte Hand-Auge-Kalibrierung (easy_handeye2, Park/Horaud) faellig.

Aufruf
------
    python3 tools/auto_camera_calibration.py            # messen und ggf. nachziehen
    python3 tools/auto_camera_calibration.py --check    # nur berichten, nichts schreiben
    python3 tools/auto_camera_calibration.py --repeat 2 # schneller (Start-Hook)

Rueckgabewerte: 0 = alles in Ordnung (nachgezogen oder schon aktuell),
                2 = Messung verworfen (alter Stand bleibt), 3 = Kamera nicht verfuegbar.
"""
import argparse
import datetime
import os
import re
import shutil
import sys

import numpy as np

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
import measure_camera_pose as mcp   # noqa: E402

WS = os.path.expanduser("~/ros2_ws")
POSE_XACRO = os.path.join(WS, "src/mycobot_world/urdf/camera_pose.xacro")
CALIB = os.path.expanduser("~/.ros2/easy_handeye2/calibrations/mycobot_d435i_eob.calib")
BACKUP_ROOT = os.path.join(WS, "calibration_backups")
HISTORIE = os.path.join(BACKUP_ROOT, "auto_calib_historie.csv")

XACRO_VORLAGE = '''<?xml version="1.0"?>
<!--
  ================= AUTOMATISCH ERZEUGTE DATEI — NICHT VON HAND AENDERN =================

  Kamera-Pose (robot_base -> camera_bottom_screw_frame). Diese Datei ist die EINZIGE
  Quelle der Kamera-Lage im Modell; mycobot_world.urdf.xacro liest sie hier ein und
  fuehrt Kamera UND Kamera-Arm daraus nach.

  Geschrieben von : tools/auto_camera_calibration.py
  Zuletzt         : {zeit}
  Verfahren       : geometrische Messung gegen die bekannte Grundplatte
                    ({pl:.0f} x {pb:.0f} cm, Mitte in robot_base ({pcx:.3f}, {pcy:.3f}),
                    Oberkante z = 0), {laeufe} Durchlaeufe, {profil}, {frames} Frames.
  Messguete       : Platte erkannt als {dl:.1f} x {db:.1f} cm, RMS {rms:.2f} mm,
                    Streuung Position {sp:.1f} mm / Rotation {sr:.2f} Grad.
  Vorher          : xyz {ax:.6f} {ay:.6f} {az:.6f} | rpy {ar:.6f} {ap:.6f} {aw:.6f}
  Sprung          : dx {dx:+.1f} mm  dy {dy:+.1f} mm  dz {dz:+.1f} mm  Rotation {drot:.2f} Grad
  Sicherung       : {sicherung}

  ACHTUNG: Das ist eine geometrische MESSUNG gegen die Platte, KEINE Hand-Auge-
  Kalibrierung. Sie setzt voraus, dass der ROBOTER unveraendert auf der Platte sitzt.
  Fuer Praezisions-Picks bleibt easy_handeye2 (Park/Horaud; Tsai-Lenz verboten)
  das Mass der Dinge.
  ======================================================================================
-->
<robot xmlns:xacro="http://www.ros.org/wiki/xacro">

  <!-- robot_base -> camera_bottom_screw_frame (Meter / Radiant) -->
  <xacro:property name="cam_x"     value="{x:.6f}"/>
  <xacro:property name="cam_y"     value="{y:.6f}"/>
  <xacro:property name="cam_z"     value="{z:.6f}"/>
  <xacro:property name="cam_roll"  value="{roll:.6f}"/>
  <xacro:property name="cam_pitch" value="{pitch:.6f}"/>
  <xacro:property name="cam_yaw"   value="{yaw:.6f}"/>

</robot>
'''

CALIB_VORLAGE = '''# ===== AUTOMATISCH NACHGEFUEHRT — tools/auto_camera_calibration.py =====
# Zuletzt: {zeit}
# Quelle : geometrische Messung gegen die bekannte Grundplatte (siehe
#          src/mycobot_world/urdf/camera_pose.xacro — das ist die fuehrende Datei).
# Guete  : Platte {dl:.1f} x {db:.1f} cm, RMS {rms:.2f} mm, Streuung {sp:.1f} mm / {sr:.2f} Grad.
# KEINE Hand-Auge-Kalibrierung: die Messung kennt die Platte, nicht die Roboterkinematik.
# Fuer Praezisions-Picks bleibt easy_handeye2 (Park/Horaud; Tsai-Lenz verboten) massgeblich.
# Sicherung des vorherigen Standes: {sicherung}
parameters:
  name: mycobot_d435i_eob
  calibration_type: eye_on_base
  robot_base_frame: robot_base
  robot_effector_frame: tcp
  tracking_base_frame: camera_color_optical_frame
  tracking_marker_frame: charuco_board
  freehand_robot_movement: true
  move_group_namespace: /
  move_group: manipulator
transform:
  translation:
    x: {tx:.17f}
    y: {ty:.17f}
    z: {tz:.17f}
  rotation:
    x: {qx:.17f}
    y: {qy:.17f}
    z: {qz:.17f}
    w: {qw:.17f}
'''


def lies_aktuelle_pose():
    """cam_* aus camera_pose.xacro lesen. None, wenn die Datei fehlt."""
    if not os.path.exists(POSE_XACRO):
        return None
    txt = open(POSE_XACRO, encoding="utf8").read()
    werte = {}
    for schluessel in ("cam_x", "cam_y", "cam_z", "cam_roll", "cam_pitch", "cam_yaw"):
        m = re.search(r'name="%s"\s+value="([-0-9.eE+]+)"' % schluessel, txt)
        if not m:
            return None
        werte[schluessel] = float(m.group(1))
    T = np.eye(4)
    T[:3, :3] = mcp.rpy_to_R(werte["cam_roll"], werte["cam_pitch"], werte["cam_yaw"])
    T[:3, 3] = [werte["cam_x"], werte["cam_y"], werte["cam_z"]]
    return T, werte


def messen(a, ref):
    """--repeat Durchlaeufe. Rueckgabe (Liste T_rb_depth, letzte Guete-Infos)."""
    laeufe, guete = [], []
    for run in range(a.repeat):
        print("---------- Durchlauf %d/%d ----------" % (run + 1, a.repeat))
        depth_m, intr = mcp.grab_depth(a.frames, a.profile)
        best, fehler = mcp.measure_once(depth_m, intr, ref, a.thresh, a.max_planes, a.tol)
        if best is None:
            print("  verworfen: %s" % fehler)
            continue
        print("  Platte %.1f x %.1f cm | RMS %.2f mm | Kipp %.2f Grad | %s"
              % (100 * best["dims"][0], 100 * best["dims"][1], best["rms_mm"],
                 best["kipp"], best["vorzeichen_grund"]))
        laeufe.append(best["T_rb_depth"])
        guete.append(best)
    return laeufe, guete


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--frames", type=int, default=40)
    ap.add_argument("--profile", default="848x480x15")
    ap.add_argument("--thresh", type=float, default=0.005)
    ap.add_argument("--repeat", type=int, default=3)
    ap.add_argument("--max-planes", type=int, default=5)
    ap.add_argument("--tol", type=float, default=0.30, help="max. Formfehler des Rechtecks")
    ap.add_argument("--check", action="store_true", help="nur berichten, nichts schreiben")
    ap.add_argument("--ueberschreibe-handauge", action="store_true",
                    help="eine vorhandene Hand-Auge-Kalibrierung bewusst ueberschreiben")
    ap.add_argument("--min-ok", type=int, default=2, help="noetige erfolgreiche Durchlaeufe")
    ap.add_argument("--schwelle-mm", type=float, default=12.0,
                    help="ab dieser Positionsabweichung wird nachgezogen")
    ap.add_argument("--schwelle-grad", type=float, default=1.5,
                    help="ab dieser Winkelabweichung wird nachgezogen")
    ap.add_argument("--max-streuung-mm", type=float, default=15.0)
    ap.add_argument("--max-streuung-grad", type=float, default=2.0)
    ap.add_argument("--max-sprung-m", type=float, default=0.50)
    ap.add_argument("--max-sprung-grad", type=float, default=45.0)
    a = ap.parse_args()

    print("=== Automatische Kamera-Nachfuehrung ===")
    ref = mcp.load_urdf_reference()
    alt = lies_aktuelle_pose()
    if alt is None:
        print("WARNUNG: camera_pose.xacro nicht lesbar — es wird neu angelegt.")
        T_alt, alt_werte = None, None
    else:
        T_alt, alt_werte = alt
        print("Stand im Modell: xyz %.5f %.5f %.5f"
              % (T_alt[0, 3], T_alt[1, 3], T_alt[2, 3]))

    try:
        laeufe, guete = messen(a, ref)
    except Exception as e:
        print("\nFEHLER: Kamera nicht lesbar (%s)." % e)
        print("Laeuft ein realsense2_camera-Node? Dieses Werkzeug braucht die Kamera EXKLUSIV —")
        print("es gehoert VOR den Stack-Start.")
        return 3

    if len(laeufe) < a.min_ok:
        print("\nVERWORFEN: nur %d von %d Durchlaeufen brauchbar (noetig: %d)."
              % (len(laeufe), a.repeat, a.min_ok))
        print("Der alte Stand bleibt unveraendert. Ist die Grundplatte weitgehend frei und im Bild?")
        return 2

    P = np.array([T[:3, 3] for T in laeufe])
    streu_mm = 1000 * float(max(np.ptp(P[:, 0]), np.ptp(P[:, 1]), np.ptp(P[:, 2])))
    streu_grad = max([mcp.ang_between(laeufe[0][:3, :3], T[:3, :3]) for T in laeufe[1:]] or [0.0])
    # Alle Durchlaeufe MITTELN statt einen auszuwaehlen — halbiert bei 3-4 Laeufen
    # das Rauschen; die y-Achse (kurze Plattenkante, teils verdeckt) ist die
    # unsicherste Richtung und profitiert am meisten davon.
    T_neu_depth = np.eye(4)
    T_neu_depth[:3, :3] = mcp.quat_average([T[:3, :3] for T in laeufe])
    T_neu_depth[:3, 3] = P.mean(axis=0)
    T_neu = T_neu_depth @ np.linalg.inv(ref["T_bs_depth"])

    print("\n--- Messung ---")
    print("Streuung: Position %.1f mm, Rotation %.2f Grad (%d Durchlaeufe)"
          % (streu_mm, streu_grad, len(laeufe)))
    print("Ergebnis: xyz %.5f %.5f %.5f" % tuple(T_neu[:3, 3]))

    if streu_mm > a.max_streuung_mm or streu_grad > a.max_streuung_grad:
        print("\nVERWORFEN: Streuung zu gross (Grenzen %.1f mm / %.1f Grad)."
              % (a.max_streuung_mm, a.max_streuung_grad))
        print("Der alte Stand bleibt unveraendert. Sicht frei? Platte teilweise verdeckt?")
        return 2

    hoehe = float(T_neu[2, 3])
    if not (0.20 <= hoehe <= 1.20) or abs(T_neu[0, 3]) > 1.0 or abs(T_neu[1, 3]) > 1.0:
        print("\nVERWORFEN: unplausible Pose (Hoehe %.3f m)." % hoehe)
        return 2

    if T_alt is not None:
        d = T_neu[:3, 3] - T_alt[:3, 3]
        dpos = float(np.linalg.norm(d))
        drot = mcp.ang_between(T_neu[:3, :3], T_alt[:3, :3])
        print("\n--- Vergleich mit dem Modell ---")
        print("dx %+.1f mm  dy %+.1f mm  dz %+.1f mm  | Rotation %.2f Grad"
              % (1000 * d[0], 1000 * d[1], 1000 * d[2], drot))
        if dpos > a.max_sprung_m or drot > a.max_sprung_grad:
            print("\nVERWORFEN: Sprung zu gross (%.3f m / %.1f Grad) — das sieht nach einer"
                  % (dpos, drot))
            print("falsch erkannten Flaeche aus, nicht nach einem Kamera-Umbau.")
            print("Der alte Stand bleibt unveraendert. Bei echtem Umbau: --max-sprung-m erhoehen.")
            return 2
        if 1000 * dpos <= a.schwelle_mm and drot <= a.schwelle_grad:
            print("\nUnveraendert (unter %.1f mm / %.1f Grad) — nichts zu tun."
                  % (a.schwelle_mm, a.schwelle_grad))
            return 0
    else:
        d = np.zeros(3)
        dpos, drot = 0.0, 0.0

    if a.check:
        print("\n--check: es wird NICHTS geschrieben. Zum Uebernehmen ohne --check aufrufen.")
        return 0

    # ---- Sperre gegen das Ueberschreiben einer Hand-Auge-Kalibrierung ----
    # Dieses Werkzeug misst die Kamera gegen die GRUNDPLATTE und nimmt dabei an,
    # dass der Roboter genau so auf ihr sitzt, wie das URDF es sagt. Eine
    # Hand-Auge-Kalibrierung braucht diese Annahme nicht und ist die bessere
    # Quelle. Weil dieses Werkzeug bei jedem Start des Stacks laeuft, wuerde es
    # eine solche Kalibrierung sonst STILL ueberschreiben - beim ersten Start
    # nach getaner Arbeit waere sie weg, ohne dass es jemand merkt.
    if os.path.exists(POSE_XACRO) and not a.ueberschreibe_handauge:
        kopf = open(POSE_XACRO, encoding="utf8").read()[:3000]
        # 2026-09-12: nur die Zeile "Verfahren : ..." zaehlt - der Rueckname-Vermerk vom 10.9.
        # erwaehnt "Hand-Auge" ebenfalls und loeste die Sperre faelschlich aus.
        verfahren = [z for z in kopf.splitlines() if z.strip().lower().startswith("verfahren")]
        ist_handauge = any("hand-auge" in z.lower() or "handauge" in z.lower() for z in verfahren) \
            or (not verfahren and "HAND-AUGE" in kopf.upper())
        if ist_handauge:
            print("\n" + "=" * 70)
            print("ABBRUCH: in camera_pose.xacro steht eine HAND-AUGE-KALIBRIERUNG.")
            print("=" * 70)
            print("Sie wird nicht ueberschrieben. Die plattenbasierte Messung waere")
            print("ein Rueckschritt, denn sie setzt voraus, dass der Roboter genau so")
            print("auf der Platte sitzt wie im Modell - was nie geprueft wurde.")
            print("")
            print("Trotzdem ueberschreiben: --ueberschreibe-handauge")
            print("Nur berichten, was gemessen wurde: --check")
            return 3

    # ---- sichern ----
    stamp = datetime.datetime.now()
    sich = os.path.join(BACKUP_ROOT, stamp.strftime("%Y-%m-%d_auto_%H%M%S"))
    os.makedirs(sich, exist_ok=True)
    for f in (POSE_XACRO, CALIB):
        if os.path.exists(f):
            shutil.copy2(f, sich)
    sich_rel = os.path.relpath(sich, WS)

    g = guete[-1]
    r, p, y = mcp.R_to_rpy(T_neu[:3, :3])
    av = alt_werte or {"cam_x": float("nan"), "cam_y": float("nan"), "cam_z": float("nan"),
                       "cam_roll": float("nan"), "cam_pitch": float("nan"), "cam_yaw": float("nan")}
    with open(POSE_XACRO, "w", encoding="utf8") as f:
        f.write(XACRO_VORLAGE.format(
            zeit=stamp.strftime("%Y-%m-%d %H:%M"), laeufe=len(laeufe),
            profil=a.profile, frames=a.frames,
            pl=100 * ref["plate_size"][0], pb=100 * ref["plate_size"][1],
            pcx=ref["plate_center"][0], pcy=ref["plate_center"][1],
            dl=100 * g["dims"][0], db=100 * g["dims"][1], rms=g["rms_mm"],
            sp=streu_mm, sr=streu_grad,
            ax=av["cam_x"], ay=av["cam_y"], az=av["cam_z"],
            ar=av["cam_roll"], ap=av["cam_pitch"], aw=av["cam_yaw"],
            dx=1000 * d[0], dy=1000 * d[1], dz=1000 * d[2], drot=drot,
            sicherung=sich_rel,
            x=T_neu[0, 3], y=T_neu[1, 3], z=T_neu[2, 3], roll=r, pitch=p, yaw=y))
    print("\ngeschrieben: %s" % os.path.relpath(POSE_XACRO, WS))

    if ref.get("T_bs_color") is not None:
        T_col = T_neu @ ref["T_bs_color"]
        q = mcp.R_to_quat(T_col[:3, :3])
        os.makedirs(os.path.dirname(CALIB), exist_ok=True)
        with open(CALIB, "w", encoding="utf8") as f:
            f.write(CALIB_VORLAGE.format(
                zeit=stamp.strftime("%Y-%m-%d %H:%M"),
                dl=100 * g["dims"][0], db=100 * g["dims"][1], rms=g["rms_mm"],
                sp=streu_mm, sr=streu_grad, sicherung=sich_rel,
                tx=T_col[0, 3], ty=T_col[1, 3], tz=T_col[2, 3],
                qx=q[0], qy=q[1], qz=q[2], qw=q[3]))
        print("geschrieben: %s" % CALIB)

    neu = not os.path.exists(HISTORIE)
    with open(HISTORIE, "a", encoding="utf8") as f:
        if neu:
            f.write("zeit,x,y,z,roll,pitch,yaw,dx_mm,dy_mm,dz_mm,drot_grad,"
                    "streuung_mm,streuung_grad,platte_l_cm,platte_b_cm,rms_mm,laeufe\n")
        f.write("%s,%.6f,%.6f,%.6f,%.6f,%.6f,%.6f,%.2f,%.2f,%.2f,%.3f,%.2f,%.3f,%.2f,%.2f,%.2f,%d\n"
                % (stamp.strftime("%Y-%m-%d %H:%M:%S"), T_neu[0, 3], T_neu[1, 3], T_neu[2, 3],
                   r, p, y, 1000 * d[0], 1000 * d[1], 1000 * d[2], drot,
                   streu_mm, streu_grad, 100 * g["dims"][0], 100 * g["dims"][1],
                   g["rms_mm"], len(laeufe)))
    print("Historie   : %s" % os.path.relpath(HISTORIE, WS))
    print("Sicherung  : %s" % sich_rel)
    print("\nFERTIG. Kein Rebuild noetig (symlink-install) — laufende Nodes muessen aber neu")
    print("gestartet werden, damit robot_state_publisher/RViz die neue Pose sehen.")
    return 0


if __name__ == "__main__":
    sys.exit(main())
