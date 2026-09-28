#!/usr/bin/env python3
"""Lage der Bereitstellungsschale bestimmen - mit Klicks in RViz, ohne Lineal.

ZUERST tools/schale_finden.py versuchen: das findet die Schale im Tiefenbild von selbst.
Dieses Werkzeug hier ist der Rueckfallweg, wenn das nicht geht - Schale halb verdeckt,
halb aus dem Bild, oder der Arm steht davor. Gerechnet wird in beiden Faellen gleich:
Kreis mit FESTEM Radius.

WARUM NICHT MESSEN: die Schale steht auf derselben Platte wie der Roboter, aber ihre
Lage gegenueber robot_base laesst sich mit dem Lineal nur ueber eine Kette von
Annahmen bestimmen - genau die Kette, die sich bei der Kamera schon einmal als
44 mm falsch herausgestellt hat (siehe camera_pose.xacro). Die Kamera sieht die
Schale dagegen unmittelbar, und ihre Lage gegen den Roboter ist seit der
Hand-Auge-Kalibrierung bekannt.

WAS GEMESSEN WIRD: nur die LAGE (x, y, z, Gierwinkel). Die FORM steht fest und kommt
aus schale.xacro - Kreisringausschnitt, 45.3 Grad, R 186.9 bis 301.9 mm. Deshalb wird
der Kreis mit FESTEM Radius eingepasst: ein 40-Grad-Bogen ist ein kurzer Bogen, und
ein Kreisfit mit freiem Radius wird darauf sehr schnell sehr unzuverlaessig. Mit
festem Radius bleiben zwei Unbekannte (der Mittelpunkt), und die sind gutmuetig.

ABLAUF
    1. Stack mit Kamera starten (start_mycobot.sh), in RViz die PointCloud einblenden
    2. python3 tools/schale_pose_klicken.py
    3. in RViz "Publish Point" waehlen und auf die OBERKANTE des AEUSSEREN
       Schalenrandes klicken - mindestens 3 Punkte, moeglichst weit auseinander,
       und beide Ecken mitnehmen (aus ihnen kommt der Gierwinkel). Dann Enter.
    4. ein bis zwei Punkte auf dem INNEREN Rand klicken (nur als Probe, sie gehen
       nicht in die Rechnung ein). Dann Enter -> Auswertung und Schreiben.
    5. SCHALE_MONTIERT=1 setzen und den Stack neu starten
    6. in RViz gegen die PointCloud pruefen - ein um einen Rand verrutschtes Modell
       (115 mm) sieht man sofort

PROBEN
    * GEGENRAND (die einzige, die wirklich traegt): nach den Randpunkten wird noch auf
      den GEGENUEBERLIEGENDEN Rand geklickt. Vom eingepassten Mittelpunkt aus muss der
      186.9 mm (bzw. 301.9 mm) entfernt liegen. Diese Probe faengt den einen Fehler ab,
      den der Fit selbst NICHT bemerken kann: auf den falschen Rand geklickt. Dann
      naemlich sitzt der Mittelpunkt um die 115 mm Randabstand daneben, waehrend die
      Restfehler kaum steigen (nachgerechnet: 1.1 mm rms richtig gegen 2.5 mm falsch).
    * Restfehler zum eingepassten Kreis - findet den einzelnen danebengesetzten Klick.
    * ueberstrichener Winkel gegen die erwarteten 45.3 Grad - deutlich weniger heisst,
      die Ecken wurden nicht mitgenommen, und dann steht der Gierwinkel schief.
    * Der freie Radius wird nur nachrichtlich ausgegeben. Auf einem 40-Grad-Bogen ist er
      KEIN Pruefmass: schon 5 mm Tiefenrauschen lassen ihn zwischen 190 und 600 mm
      schwanken. Genau deshalb wird mit festem Radius eingepasst.
"""
import argparse, math, os, shutil, sys, time
from datetime import date

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import PointStamped
import tf2_ros
from tf2_ros import TransformException
from rclpy.duration import Duration
try:                      # registriert PointStamped bei tf2 - ohne das kann
    import tf2_geometry_msgs  # noqa: F401   Buffer.transform() den Typ nicht wandeln
except ImportError:
    tf2_geometry_msgs = None

WS = os.path.expanduser("~/ros2_ws")
POSE_XACRO = os.path.join(WS, "src/mycobot_world/urdf/schale_pose.xacro")

# Form der Schale - identisch zu schale.xacro. Wird hier nur zum Einpassen und
# fuer die Proben gebraucht, NICHT geschrieben.
R_INNEN = 0.1869
R_AUSSEN = 0.3019
WINKEL = math.radians(45.3)   # Oberkante, Kamera 2026-09-12 (vorher 39.9); = schale_winkel im URDF
WANDHOEHE = 0.017

ap = argparse.ArgumentParser()
ap.add_argument("--innen", action="store_true",
                help="auf den INNEREN Rand geklickt (R=186.9mm) statt auf den aeusseren")
ap.add_argument("--boden", action="store_true",
                help="auf den INNENBODEN geklickt statt auf die Oberkante "
                     "(dann wird die Wandhoehe nicht abgezogen)")
ap.add_argument("--frame", default="robot_base", help="Zielframe (Vorgabe robot_base)")
ap.add_argument("--topic", default="/clicked_point")
ap.add_argument("--trocken", action="store_true", help="nur rechnen, nichts schreiben")
a = ap.parse_args()

R_SOLL = R_INNEN if a.innen else R_AUSSEN
R_GEGEN = R_AUSSEN if a.innen else R_INNEN
RAND = "INNEREN" if a.innen else "AEUSSEREN"
RAND_GEGEN = "AEUSSEREN" if a.innen else "INNEREN"


def kreis_frei(pts):
    """Algebraischer Kreisfit (Kasa) mit freiem Radius - nur als Startwert und Probe."""
    n = len(pts)
    sx = sum(p[0] for p in pts) / n
    sy = sum(p[1] for p in pts) / n
    u = [p[0] - sx for p in pts]
    v = [p[1] - sy for p in pts]
    suu = sum(t * t for t in u); svv = sum(t * t for t in v)
    suv = sum(u[i] * v[i] for i in range(n))
    suuu = sum(t ** 3 for t in u); svvv = sum(t ** 3 for t in v)
    suvv = sum(u[i] * v[i] * v[i] for i in range(n))
    svuu = sum(v[i] * u[i] * u[i] for i in range(n))
    det = 2.0 * (suu * svv - suv * suv)
    if abs(det) < 1e-12:
        return None
    cu = (svv * (suuu + suvv) - suv * (svvv + svuu)) / det
    cv = (suu * (svvv + svuu) - suv * (suuu + suvv)) / det
    cx, cy = cu + sx, cv + sy
    r = sum(math.hypot(p[0] - cx, p[1] - cy) for p in pts) / n
    return cx, cy, r


def kreis_fest(pts, r_soll, start):
    """Mittelpunkt bei FESTEM Radius, Gauss-Newton auf sum((|p-c| - r)^2)."""
    cx, cy = start
    for _ in range(200):
        jtj = [[0.0, 0.0], [0.0, 0.0]]; jtr = [0.0, 0.0]
        for px, py in pts:
            dx, dy = px - cx, py - cy
            d = math.hypot(dx, dy)
            if d < 1e-9:
                continue
            rest = d - r_soll
            # d(rest)/dcx = -dx/d , d(rest)/dcy = -dy/d
            g = (-dx / d, -dy / d)
            for i in range(2):
                jtr[i] += g[i] * rest
                for j in range(2):
                    jtj[i][j] += g[i] * g[j]
        det = jtj[0][0] * jtj[1][1] - jtj[0][1] * jtj[1][0]
        if abs(det) < 1e-15:
            break
        sx = -(jtj[1][1] * jtr[0] - jtj[0][1] * jtr[1]) / det
        sy = -(jtj[0][0] * jtr[1] - jtj[1][0] * jtr[0]) / det
        cx += sx; cy += sy
        if math.hypot(sx, sy) < 1e-9:
            break
    return cx, cy


class Sammler(Node):
    def __init__(self):
        super().__init__("schale_pose_klicken")
        self.punkte = []
        self.puffer = tf2_ros.Buffer()
        self.lauscher = tf2_ros.TransformListener(self.puffer, self)
        self.create_subscription(PointStamped, a.topic, self._klick, 10)

    def _klick(self, msg):
        p = msg.point
        if msg.header.frame_id and msg.header.frame_id != a.frame:
            try:
                msg = self.puffer.transform(msg, a.frame, timeout=Duration(seconds=1.0))
                p = msg.point
            except TransformException as e:
                self.get_logger().error("Punkt nicht nach %s wandelbar: %s" % (a.frame, e))
                return
        self.punkte.append((p.x, p.y, p.z))
        print("  Punkt %d: x=%7.1f y=%7.1f z=%7.1f mm" % (len(self.punkte), p.x*1e3, p.y*1e3, p.z*1e3))


def schreibe(cx, cy, cz, yaw, bericht):
    if os.path.exists(POSE_XACRO):
        shutil.copy2(POSE_XACRO, POSE_XACRO + ".bak")
    txt = '''<?xml version="1.0"?>
<!--
  ================= ERZEUGTE DATEI - NICHT VON HAND AENDERN =================

  Lage der Bereitstellungsschale (robot_base -> schale). Diese Datei ist die EINZIGE
  Quelle der Schalenlage im Modell; mycobot_world.urdf.xacro liest sie hier ein.
  Die MASSE der Schale stehen NICHT hier, sondern in schale.xacro.

  Geschrieben von : tools/schale_pose_klicken.py
  Zuletzt         : %s
  Verfahren       : Kreis mit FESTEM Radius durch von Hand in RViz geklickte Punkte
                    auf der PointCloud der Kamera. Die Form ist bekannt, gesucht war
                    nur die Lage.

%s

  Der Ursprung des Frames `schale` ist der MITTELPUNKT des Kreisrings - er liegt
  neben der Schale, nicht in ihr. z=0 ist der Innenboden, +X die Winkelhalbierende.

  Einschalten: SCHALE_MONTIERT=1 (bzw. schale_montiert:=true).
  ==========================================================================
-->
<robot xmlns:xacro="http://www.ros.org/wiki/xacro">

  <!-- robot_base -> schale (Meter / Radiant) -->
  <xacro:property name="schale_x"   value="%.6f"/>
  <xacro:property name="schale_y"   value="%.6f"/>
  <xacro:property name="schale_z"   value="%.6f"/>
  <xacro:property name="schale_yaw" value="%.6f"/>

</robot>
''' % (date.today().isoformat(), bericht, cx, cy, cz, yaw)
    with open(POSE_XACRO, "w") as f:
        f.write(txt)


def warte_auf_enter(knoten):
    import threading
    fertig = threading.Event()
    threading.Thread(target=lambda: (sys.stdin.readline(), fertig.set()), daemon=True).start()
    while rclpy.ok() and not fertig.is_set():
        rclpy.spin_once(knoten, timeout_sec=0.2)


def main():
    rclpy.init()
    knoten = Sammler()
    print("=" * 70)
    print("SCHALENLAGE BESTIMMEN")
    print("=" * 70)
    print("SCHRITT 1 - in RViz 'Publish Point' waehlen und auf die OBERKANTE des")
    print("%s Schalenrandes klicken. Mindestens 3 Punkte, weit auseinander," % RAND)
    print("und BEIDE ECKEN mitnehmen - aus ihnen kommt der Gierwinkel.")
    if a.boden:
        print("(--boden: die Klicks liegen auf dem Innenboden, nicht auf der Oberkante)")
    print("Fertig? Enter druecken.\n")
    warte_auf_enter(knoten)
    pts = list(knoten.punkte)

    print("\nSCHRITT 2 (Probe) - jetzt ein bis zwei Punkte auf dem %s Rand klicken." % RAND_GEGEN)
    print("Nur damit geprueft werden kann, ob ueberhaupt der richtige Rand gemeint war;")
    print("in die Rechnung gehen diese Punkte NICHT ein. Ueberspringen: gleich Enter.\n")
    knoten.punkte = []
    warte_auf_enter(knoten)
    probe_pts = list(knoten.punkte)

    knoten.destroy_node()
    rclpy.shutdown()

    if len(pts) < 3:
        print("\nABBRUCH: %d Punkte - mindestens 3 werden gebraucht." % len(pts))
        return 1

    xy = [(p[0], p[1]) for p in pts]
    frei = kreis_frei(xy)
    if frei is None:
        print("\nABBRUCH: die Punkte liegen auf einer Geraden, daraus wird kein Kreis.")
        return 1
    fx, fy, fr = frei
    cx, cy = kreis_fest(xy, R_SOLL, (fx, fy))

    reste = [math.hypot(p[0] - cx, p[1] - cy) - R_SOLL for p in xy]
    rest_max = max(abs(r) for r in reste)
    rest_rms = math.sqrt(sum(r * r for r in reste) / len(reste))

    winkel = sorted(math.atan2(p[1] - cy, p[0] - cx) for p in xy)
    # Sektor kann ueber +-pi laufen: groesste Luecke suchen und dort auftrennen.
    n = len(winkel)
    luecken = [((winkel[(i + 1) % n] - winkel[i]) % (2 * math.pi), i) for i in range(n)]
    luecke, i0 = max(luecken)
    start = winkel[(i0 + 1) % n]
    spanne = 2 * math.pi - luecke
    yaw = (start + spanne / 2.0 + math.pi) % (2 * math.pi) - math.pi

    z_klick = sum(p[2] for p in pts) / len(pts)
    cz = z_klick if a.boden else z_klick - WANDHOEHE

    # Probe Gegenrand
    gegen_abw = None
    if probe_pts:
        rr = [math.hypot(p[0] - cx, p[1] - cy) for p in probe_pts]
        gegen_abw = sum(rr) / len(rr) - R_GEGEN

    print("\n" + "=" * 70)
    print("ERGEBNIS")
    print("=" * 70)
    print("Punkte                 : %d auf dem %s Rand" % (len(pts), RAND))
    print("Mittelpunkt (fester R) : x=%.1f mm  y=%.1f mm" % (cx * 1e3, cy * 1e3))
    print("Innenboden z           : %.1f mm  (geklickt %.1f, %s)"
          % (cz * 1e3, z_klick * 1e3,
             "Oberkante minus 17 mm Wandhoehe" if not a.boden else "direkt"))
    print("Gierwinkel             : %.2f Grad" % math.degrees(yaw))
    print("-" * 70)
    if gegen_abw is None:
        print("PROBE Gegenrand        : UEBERSPRUNGEN - der falsche Rand faellt damit nicht auf")
    else:
        print("PROBE Gegenrand        : %d Punkt(e), %+.1f mm gegen die erwarteten %.1f mm"
              % (len(probe_pts), gegen_abw * 1e3, R_GEGEN * 1e3))
    print("PROBE Restfehler       : rms %.1f mm , groesster %.1f mm"
          % (rest_rms * 1e3, rest_max * 1e3))
    print("PROBE Winkelspanne     : %.1f Grad (erwartet %.1f Grad)"
          % (math.degrees(spanne), math.degrees(WINKEL)))
    print("nachrichtlich, KEIN Pruefmass auf einem 40-Grad-Bogen:")
    print("         freier Radius : %.1f mm (fest eingepasst wurde mit %.1f mm)"
          % (fr * 1e3, R_SOLL * 1e3))
    print("-" * 70)

    zweifel = []
    if gegen_abw is None:
        zweifel.append("die Probe am Gegenrand wurde uebersprungen. Genau sie faengt den "
                       "einen Fehler ab, den der Fit nicht bemerkt: auf den falschen Rand "
                       "geklickt. Dann steht die Schale um 115 mm versetzt im Modell.")
    elif abs(gegen_abw) > 0.020:
        zweifel.append("der Gegenrand liegt %+.1f mm neben seinem Sollwert. Das ist der "
                       "typische Befund, wenn in Schritt 1 der falsche Rand geklickt wurde."
                       % (gegen_abw * 1e3))
    if math.degrees(WINKEL - spanne) > 10.0:
        zweifel.append("die Punkte ueberstreichen %.1f statt %.1f Grad - ohne beide Ecken "
                       "steht der Gierwinkel schief." % (math.degrees(spanne), math.degrees(WINKEL)))
    if rest_max > 0.015:
        zweifel.append("groesster Restfehler %.1f mm - ein Klick sitzt daneben "
                       "(Tiefenrauschen oder nicht auf dem Rand)." % (rest_max * 1e3))
    for z in zweifel:
        print("WARNUNG: " + z)
    if zweifel:
        print("-" * 70)
        if input("Trotzdem schreiben? [j/N] ").strip().lower() not in ("j", "ja", "y", "yes"):
            print("Nichts geschrieben.")
            return 1

    bericht = ("  Punkte          : %d, geklickt auf dem %s Rand\n"
               "  Probe Gegenrand : %s\n"
               "  Probe Restfehler: rms %.1f mm, groesster %.1f mm\n"
               "  Probe Winkel    : ueberstrichen %.1f Grad gegen erwartete %.1f Grad\n"
               "  nachrichtlich   : frei eingepasster Radius %.1f mm (fest: %.1f mm) -\n"
               "                    auf einem 40-Grad-Bogen kein Pruefmass"
               % (len(pts), RAND,
                  ("uebersprungen" if gegen_abw is None
                   else "%+.1f mm gegen die erwarteten %.1f mm" % (gegen_abw * 1e3, R_GEGEN * 1e3)),
                  rest_rms * 1e3, rest_max * 1e3,
                  math.degrees(spanne), math.degrees(WINKEL), fr * 1e3, R_SOLL * 1e3))

    if a.trocken:
        print("--trocken: nichts geschrieben.")
        return 0

    schreibe(cx, cy, cz, yaw, bericht)
    print("geschrieben: %s   (Sicherung: %s.bak)" % (POSE_XACRO, POSE_XACRO))
    print("\nJETZT einschalten und neu starten:")
    print("    SCHALE_MONTIERT=1 ./start_mycobot.sh")
    print("In RViz gegen die PointCloud pruefen: der Ringausschnitt muss auf der")
    print("wirklichen Schale liegen. Tut er das nicht, nochmal klicken.")
    return 0


if __name__ == "__main__":
    sys.exit(main())
