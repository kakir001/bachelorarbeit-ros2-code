#!/usr/bin/env python3
"""Die Bereitstellungsschale im Tiefenbild SELBST finden - ohne Klicken, ohne Lineal.

WAS GESUCHT WIRD: nur die LAGE (x, y, z, Gierwinkel). Die FORM steht fest und kommt aus
schale.xacro: Kreisringausschnitt, 45.3 Grad, R 186.9 bis 301.9 mm, Wand 2 mm, Wandhoehe
17 mm. Weil die Form bekannt ist, ist das Suchen einfach - man muss nur wissen, wonach.

VERFAHREN, in drei Schritten:

 1. OERTLICHE ERHEBUNG statt Hoehe ueber einer Ebene. Die Punkte werden in ein Raster in
    robot_base gelegt; je Zelle die hoechste und die tiefste Tiefe. Der Untergrund ist das
    20-Prozent-Quantil der Umgebung (etwa 8 cm Fenster). Erhebung = hoechster Punkt der
    Zelle ueber diesem Untergrund. Der Schalenrand hebt sich so mit seinen 17 mm heraus,
    UNABHAENGIG davon, worauf die Schale steht. Genau das ist der Punkt: eine feste
    z-Schranke wuerde die Schale verfehlen, sobald sie nicht auf der Grundplatte steht,
    sondern daneben auf dem Tisch - und danach sieht es aus (das zuletzt bestaetigte Ziel
    lag bei z = -14 mm, also UNTER der Plattenoberkante).

 2. HOUGH MIT FESTEM RADIUS. Jede Randzelle stimmt fuer alle Mittelpunkte ab, die 187.9
    bzw. 300.9 mm von ihr entfernt liegen - beide Boegen stimmen in DASSELBE Raster ab und
    verstaerken sich. Warum fester Radius: ein 40-Grad-Bogen ist ein kurzer Bogen; ein
    Kreisfit mit freiem Radius schwankt darauf schon bei 5 mm Tiefenrauschen zwischen 190
    und 600 mm. Mit festem Radius bleiben zwei Unbekannte, und die sind gutmuetig.

 3. NACHZIEHEN UND SEKTOR. Um den Rasterpunkt herum werden die Randzellen den beiden
    Radien zugeordnet und der Mittelpunkt mit Gauss-Newton nachgezogen (Rasterschritt
    faellt damit weg). Der Gierwinkel kommt aus einem 45.3-Grad-Fenster, das ueber die
    Winkelverteilung geschoben wird - robuster als der Abstand der beiden aeussersten
    Punkte, denn ein einzelner Ausreisser verdreht ihn nicht.

WAS ES SELBST PRUEFT - die beiden tragenden Proben sind an synthetischen Szenen geeicht
(Tisch, Roboterbasis, Schale, dazu Stoerkram; die Wahrheit war bekannt):

    Probe            echte Schale   Stoerkram / halb verdeckt
    Schaerfe            1.91            1.04 bis 1.12
    Ring ausserhalb     0.07            1.24 bis 3.87

  * SCHAERFE = Gipfel des Abstimmrasters gegen dessen 99.9-Prozent-Quantil. Ist im Bild
    keine Schale, gibt es keinen Gipfel, nur Rauschen.
  * RING AUSSERHALB = Randzellen, die auf dem Radius liegen, aber ausserhalb des
    45.3-Grad-Sektors, geteilt durch die darin. Eine wirkliche Schale hat dort NICHTS;
    ein aus Stoerkram zusammengewuerfelter Kreis ist rundum belegt. Diese Probe ist die
    wichtigste, denn sie faengt den Fall ab, in dem die Rechnung selbstsicher aussieht
    (Restfehler klein, beide Boegen einig) und trotzdem falsch ist.
  Dazu: Deckung beider Boegen, Restfehler bei bekanntem Radius, Abstand der beiden
  Einzelmittelpunkte, und ob ueberhaupt plausibel viele Randzellen gefunden wurden.

Auf derselben Eichung: bei sauberer Sicht liegt der Mittelpunkt 0.4 mm, der Gierwinkel
0.05 Grad und die Bodenhoehe 0.9 mm neben der Wahrheit.

Der eine Fehler, den KEINE Rechnung bemerken kann, ist eine Schale, die gar nicht im Bild
ist. Deshalb wird das Ergebnis IMMER gezeichnet - als Marker in RViz und als Umriss im
Kamerabild - und erst nach Bestaetigung geschrieben. Bei Zweifeln schreibt auch
--schreiben nicht.

WENN ES NICHT GEHT (Schale halb verdeckt, halb aus dem Bild, Arm davor): dann von Hand
mit tools/schale_pose_klicken.py. Dort klickt man die Randpunkte selbst, gerechnet wird
mit demselben festen Radius.

    ros2 topic list | grep aligned          # Kamera muss laufen

Die Aufloesung der Kamera geht unmittelbar in die Genauigkeit ein: die Wand ist nur 2 mm
stark. Bei 848x480 sind das rund 1 Pixel auf 60 cm Abstand, bei 424x240 (dem Profil des
Standard-Stacks) nur noch ein halbes. Zum Einmessen lohnt es sich, die Kamera kurz mit
848x480x15 und ohne PointCloud zu starten - der Nano traegt das, solange Farb- und
Tiefenprofil GLEICH sind.
    python3 tools/schale_finden.py          # zeichnet, fragt dann nach
    python3 tools/schale_finden.py --schreiben   # ohne Rueckfrage uebernehmen

Angeschaut wird das Ergebnis auf zwei Wegen:
    RViz : Add > By topic > /schale/erkennung > MarkerArray, Fixed Frame robot_base
    Bild : ros2 run rqt_image_view rqt_image_view /schale/overlay
Das Kamerabild ist auf diesem Rechner der bequemere Weg - eine PointCloud kostet den
Nano zu viel, der Umriss im Bild zeigt dasselbe.
"""
import argparse, math, os, shutil, sys
from datetime import date

import numpy as np
import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy, DurabilityPolicy
from sensor_msgs.msg import Image, CameraInfo
from visualization_msgs.msg import Marker, MarkerArray
from geometry_msgs.msg import Point
from std_msgs.msg import ColorRGBA
import tf2_ros

WS = os.path.expanduser("~/ros2_ws")
POSE_XACRO = os.path.join(WS, "src/mycobot_world/urdf/schale_pose.xacro")

# Form - identisch zu schale.xacro.
# Nachgemessen am Bauteil (Benutzer, 2026-09-10): Wandhoehe 18 mm, und die Wand ist
# NICHT ueberall gleich stark - am aeusseren Bogen 5 mm, zum inneren Bogen hin bis auf
# 1 mm auslaufend. Das ist kein Detail: gesucht wird der Kamm der Wand, und der liegt
# um die halbe Wandstaerke nach innen versetzt - aussen also 2.5 mm, innen 0.5 mm.
R_INNEN, R_AUSSEN = 0.1869, 0.3019
WAND_INNEN, WAND_AUSSEN = 0.001, 0.005
WANDHOEHE = 0.018
WINKEL = math.radians(45.3)   # Oberkante, Kamera 2026-09-12 (vorher 39.9); = schale_winkel im URDF
RI_MITTE = R_INNEN + WAND_INNEN / 2.0
RA_MITTE = R_AUSSEN - WAND_AUSSEN / 2.0

ap = argparse.ArgumentParser()
ap.add_argument("--depth", default="/camera/aligned_depth_to_color/image_raw")
ap.add_argument("--info", default="/camera/color/camera_info")
ap.add_argument("--farbe", default="/camera/color/image_raw")
ap.add_argument("--frame", default="robot_base")
ap.add_argument("--zelle", type=float, default=0.005, help="Rasterschritt [m]")
ap.add_argument("--fenster", type=float, default=0.08, help="Fenster fuer den Untergrund [m]")
ap.add_argument("--erhebung", type=float, nargs=2, default=(0.010, 0.028),
                help="Erhebung, die als Rand gilt [m]")
ap.add_argument("--rmin", type=float, default=0.13,
                help="alles naeher als das am Roboter wird verworfen (eigene Basis) [m]")
ap.add_argument("--reichweite", type=float, default=0.62, help="halbe Rasterbreite [m]")
ap.add_argument("--mittelpunkt-nahe", type=float, nargs=3, default=None,
                metavar=("X", "Y", "RADIUS"),
                help="Mittelpunkt nur in diesem Kreis suchen [m]. Sinnvoll, weil die "
                     "Schale den Reichweitenboegen des Roboters nachgebaut ist: ihr "
                     "Kruemmungsmittelpunkt liegt nahe der Roboterachse. "
                     "Zum Beispiel: --mittelpunkt-nahe 0 0 0.08")
ap.add_argument("--bereich", type=float, nargs=4, default=None,
                metavar=("XMIN", "XMAX", "YMIN", "YMAX"),
                help="nur Punkte in diesem Rechteck in robot_base verwenden [m]. "
                     "Fuer die Grundplatte: -0.09 0.39 -0.315 0.065")
ap.add_argument("--schritt", type=int, default=2, help="jeden n-ten Bildpunkt nehmen")
ap.add_argument("--mittelung", type=int, default=8, help="so viele Tiefenbilder mitteln")
ap.add_argument("--schreiben", action="store_true", help="ohne Rueckfrage uebernehmen")
ap.add_argument("--nur-zeigen", action="store_true", help="nie schreiben, nur zeichnen")
ap.add_argument("--bild", default="", help="Overlay zusaetzlich als PNG hierhin sichern")
ap.add_argument("--stumm", action="store_true", help="nicht nachfragen, nicht am Ende spinnen")
a = ap.parse_args()


def qrot_viele(q, p):
    """Punkte (N,3) mit Quaternion (x,y,z,w) drehen."""
    u = np.asarray(q[:3], dtype=np.float64)
    w = float(q[3])
    return (p + 2.0 * w * np.cross(u, p) + 2.0 * np.cross(u, np.cross(u, p)))


def kreis_fest(pts, r_soll, start, runden=200):
    """Mittelpunkt bei FESTEM Radius, Gauss-Newton auf sum((|p-c| - r)^2)."""
    c = np.array(start, dtype=np.float64)
    for _ in range(runden):
        d = pts - c
        n = np.hypot(d[:, 0], d[:, 1])
        gut = n > 1e-9
        if not gut.any():
            break
        d, n = d[gut], n[gut]
        rest = n - r_soll
        g = -d / n[:, None]                      # d(rest)/dc
        jtj = g.T @ g
        jtr = g.T @ rest
        try:
            s = -np.linalg.solve(jtj, jtr)
        except np.linalg.LinAlgError:
            break
        c = c + s
        if np.hypot(*s) < 1e-9:
            break
    return c


class Finder(Node):
    def __init__(self):
        super().__init__("schale_finden")
        self.tiefen = []
        self.K = None
        self.depth_frame = None
        qos = QoSProfile(depth=5, reliability=ReliabilityPolicy.BEST_EFFORT,
                         history=HistoryPolicy.KEEP_LAST)
        self.create_subscription(Image, a.depth, self._on_depth, qos)
        self.create_subscription(CameraInfo, a.info, self._on_info, qos)
        self.create_subscription(Image, a.farbe, self._on_color, qos)
        self.farbbild = None
        self.puffer = tf2_ros.Buffer()
        self.lauscher = tf2_ros.TransformListener(self.puffer, self)
        latch = QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL,
                           reliability=ReliabilityPolicy.RELIABLE,
                           history=HistoryPolicy.KEEP_LAST)
        self.marker = self.create_publisher(MarkerArray, "/schale/erkennung", latch)
        self.overlay = self.create_publisher(Image, "/schale/overlay", latch)

    def _on_info(self, msg):
        if self.K is None:
            self.K = (msg.k[0], msg.k[4], msg.k[2], msg.k[5])

    def _on_color(self, msg):
        enc = msg.encoding.lower()
        if enc not in ("rgb8", "bgr8"):
            return
        img = np.frombuffer(msg.data, dtype=np.uint8).reshape(msg.height, msg.width, 3)
        self.farbbild = np.ascontiguousarray(img[:, :, ::-1] if enc == "rgb8" else img)

    def _on_depth(self, msg):
        if msg.encoding.lower() not in ("16uc1", "mono16"):
            return
        self.depth_frame = msg.header.frame_id
        if len(self.tiefen) < a.mittelung:
            self.tiefen.append(
                np.frombuffer(msg.data, dtype=np.uint16).reshape(msg.height, msg.width).copy())

    def bereit(self):
        return self.K is not None and len(self.tiefen) >= a.mittelung


def punkte_in_base(knoten):
    """Gemitteltes Tiefenbild -> (N,3) Punkte in robot_base."""
    stapel = np.stack(knoten.tiefen).astype(np.float32)
    stapel[stapel == 0] = np.nan
    with np.errstate(all="ignore"):
        import warnings
        with warnings.catch_warnings():   # Pixel ohne jede gueltige Tiefe sind normal
            warnings.simplefilter("ignore", RuntimeWarning)
            tiefe = np.nanmedian(stapel, axis=0) * 1e-3      # m
    h, w = tiefe.shape
    fx, fy, cx, cy = knoten.K
    vs, us = np.mgrid[0:h:a.schritt, 0:w:a.schritt]
    z = tiefe[::a.schritt, ::a.schritt]
    gut = np.isfinite(z) & (z > 0.15) & (z < 2.0)
    u, v, z = us[gut].astype(np.float64), vs[gut].astype(np.float64), z[gut].astype(np.float64)
    p_opt = np.stack([(u - cx) * z / fx, (v - cy) * z / fy, z], axis=1)

    tf = knoten.puffer.lookup_transform(a.frame, knoten.depth_frame, rclpy.time.Time())
    q = tf.transform.rotation
    t = tf.transform.translation
    return qrot_viele((q.x, q.y, q.z, q.w), p_opt) + np.array([t.x, t.y, t.z])


def randzellen(p):
    """Zellen mit oertlicher Erhebung im Randfenster. -> (M,2) Mittelpunkte, (M,) Hoehe."""
    from scipy.ndimage import percentile_filter, gaussian_filter  # noqa: F401
    R, zl = a.reichweite, a.zelle
    innen = (np.abs(p[:, 0]) < R) & (np.abs(p[:, 1]) < R)
    p = p[innen]
    # --bereich: alles ausserhalb eines Rechtecks WEGWERFEN, bevor der Untergrund
    # gerechnet wird. Das ist kein Zurechtschneiden auf das gewuenschte Ergebnis,
    # sondern die Abhilfe gegen einen echten Rechenfehler am Plattenrand:
    # der Untergrund ist das 20-Prozent-Quantil in einem 8-cm-Fenster. Steht dieses
    # Fenster halb neben der 25 mm dicken Grundplatte, ist der Untergrund der TISCH,
    # und jede flache Zelle OBEN AUF der Platte bekommt dadurch 25 mm Erhebung
    # zugeschrieben. So entsteht rings um die Platte ein 8 cm breiter Streifen
    # falscher "Randzellen" (gemessen 2026-09-10: 1500 statt der rund 300 der Schale).
    # Fuer die Hough-Abstimmung ist das toedlich: eine GERADE Kante stimmt fuer alle
    # Mittelpunkte auf zwei zu ihr parallelen Geraden ab, zwei solche Kanten
    # schneiden sich in einem scharfen Gipfel - der den kurzen 40-Grad-Bogen der
    # Schale sicher schlaegt. Genau das ist dreimal passiert.
    # Werden die Punkte neben der Platte vorher entfernt, sieht das Fenster am Rand
    # nur noch Plattenpunkte (leere Zellen werden mit dem Median gefuellt, also
    # ebenfalls Plattenhoehe) und die Erhebung dort wird richtig zu rund 0.
    if a.bereich is not None:
        x0, x1, y0, y1 = a.bereich
        im_bereich = ((p[:, 0] >= x0) & (p[:, 0] <= x1)
                      & (p[:, 1] >= y0) & (p[:, 1] <= y1))
        print("Bereich %.2f..%.2f x %.2f..%.2f m: %d von %d Punkten bleiben"
              % (x0, x1, y0, y1, int(im_bereich.sum()), len(p)))
        p = p[im_bereich]
    n = int(2 * R / zl)
    ix = np.clip(((p[:, 0] + R) / zl).astype(np.int64), 0, n - 1)
    iy = np.clip(((p[:, 1] + R) / zl).astype(np.int64), 0, n - 1)
    flach = iy * n + ix

    hoch = np.full(n * n, -np.inf); np.maximum.at(hoch, flach, p[:, 2])
    tief = np.full(n * n, np.inf);  np.minimum.at(tief, flach, p[:, 2])
    leer = ~np.isfinite(hoch)
    fuell = np.median(tief[~leer]) if (~leer).any() else 0.0
    tief_g = np.where(np.isfinite(tief), tief, fuell).reshape(n, n)

    k = max(3, int(round(a.fenster / zl)) | 1)
    grund = percentile_filter(tief_g, 20, size=k, mode="nearest")
    erhebung = hoch.reshape(n, n) - grund

    lo, hi = a.erhebung
    ist_rand = np.isfinite(erhebung) & (erhebung > lo) & (erhebung < hi)
    jy, jx = np.nonzero(ist_rand)
    mx = (jx + 0.5) * zl - R
    my = (jy + 0.5) * zl - R
    weit = np.hypot(mx, my) > a.rmin
    return (np.stack([mx[weit], my[weit]], axis=1),
            hoch.reshape(n, n)[jy, jx][weit],
            erhebung[jy, jx][weit])


def hough(rand):
    """Abstimmung mit festem Radius -> (cx, cy) und Schaerfe des Gipfels.

    Schaerfe = Gipfel geteilt durch das 99.9-Prozent-Quantil des ganzen Rasters. An
    synthetischen Szenen gemessen: eine wirklich vorhandene Schale ergibt rund 2.1,
    blosser Stoerkram (oder eine halb verdeckte Schale) bleibt bei 1.06 bis 1.08. Der
    fruehere Vergleich mit der besten Stelle NEBENAN taugte dafuer nicht - das
    Abstimmraster hat um den Gipfel herum einen breiten Ruecken, sodass auch ein
    einwandfreier Fund nur auf 1.2 kam.
    """
    from scipy.ndimage import gaussian_filter
    R, zl = a.reichweite, 0.004
    n = int(2 * R / zl)
    winkel = np.linspace(0, 2 * math.pi, 720, endpoint=False)
    co, si = np.cos(winkel), np.sin(winkel)
    acc = np.zeros(n * n, dtype=np.int32)
    for r in (RI_MITTE, RA_MITTE):
        vx = rand[:, 0:1] + r * co[None, :]
        vy = rand[:, 1:2] + r * si[None, :]
        ix = ((vx + R) / zl).astype(np.int64)
        iy = ((vy + R) / zl).astype(np.int64)
        ok = (ix >= 0) & (ix < n) & (iy >= 0) & (iy < n)
        acc += np.bincount((iy[ok] * n + ix[ok]), minlength=n * n).astype(np.int32)
    g = gaussian_filter(acc.reshape(n, n).astype(np.float32), 1.2)
    # --mittelpunkt-nahe: die Suche auf einen Kreis um einen erwarteten Mittelpunkt
    # einschraenken. Grund ist kein Wunschdenken, sondern wie die Schale entstanden
    # ist: der Benutzer hat auf der Arbeitsplatte die per IK erreichbaren Punkte
    # markiert, durch sie zwei Kreise gelegt und die Schale dem Ring dazwischen
    # nachgebaut. Diese Kreise sind um die Roboterachse geschlagen - der
    # Kruemmungsmittelpunkt der Schale liegt also nahe robot_base (2026-09-10 an den
    # Randpunkten geprueft: ihre Radien um robot_base liegen im Band 170..272 mm,
    # also rund 2-3 cm innerhalb der Sollage - von Hand hingestellt).
    # Ohne diese Schranke gewinnt regelmaessig die Plattenkante: eine GERADE Kante
    # stimmt fuer alle Mittelpunkte auf zwei zu ihr parallelen Geraden ab, und zwei
    # solche Kanten schneiden sich in einem schaerferen Gipfel als der kurze
    # 40-Grad-Bogen der Schale.
    if a.mittelpunkt_nahe is not None:
        mx0, my0, rr = a.mittelpunkt_nahe
        gy_r, gx_r = np.mgrid[0:n, 0:n]
        cxs = (gx_r + 0.5) * zl - R
        cys = (gy_r + 0.5) * zl - R
        erlaubt = np.hypot(cxs - mx0, cys - my0) <= rr
        if not erlaubt.any():
            print("ABBRUCH: --mittelpunkt-nahe liegt ausserhalb des Rasters.")
            raise SystemExit(2)
        g = np.where(erlaubt, g, 0.0)
        print("Mittelpunkt nur im Kreis um (%.3f, %.3f) mit r=%.3f m gesucht"
              % (mx0, my0, rr))
    i = int(np.argmax(g))
    gy, gx = divmod(i, n)
    schaerfe = float(g.flat[i]) / max(float(np.percentile(g, 99.9)), 1e-6)
    return (gx + 0.5) * zl - R, (gy + 0.5) * zl - R, schaerfe


def sektor(winkel):
    """45.3-Grad-Fenster ueber die Winkel schieben, dort wo die meisten liegen."""
    schritte = np.radians(np.arange(0, 360, 1.0))
    w = np.sort(np.mod(winkel, 2 * math.pi))
    best, best_s = -1, 0.0
    for s in schritte:
        d = np.mod(w - s, 2 * math.pi)
        c = int(np.count_nonzero(d <= WINKEL))
        if c > best:
            best, best_s = c, s
    return best_s + WINKEL / 2.0, best


def bogen_punkte(c, r, yaw, z, n=40):
    return [Point(x=float(c[0] + r * math.cos(yaw - WINKEL / 2 + WINKEL * i / (n - 1))),
                  y=float(c[1] + r * math.sin(yaw - WINKEL / 2 + WINKEL * i / (n - 1))),
                  z=float(z)) for i in range(n)]


def zeichne(knoten, c, yaw, z_boden, text):
    arr = MarkerArray()
    gruen = ColorRGBA(r=0.1, g=0.9, b=0.3, a=1.0)
    i = 0
    for z in (z_boden, z_boden + WANDHOEHE):
        for r in (R_INNEN, R_AUSSEN):
            m = Marker()
            m.header.frame_id = a.frame
            m.ns, m.id, m.type, m.action = "schale", i, Marker.LINE_STRIP, Marker.ADD
            m.scale.x = 0.003
            m.color = gruen
            m.pose.orientation.w = 1.0
            m.points = bogen_punkte(c, r, yaw, z)
            arr.markers.append(m); i += 1
    # Seitenwaende und Ecken senkrecht
    for s in (-1, 1):
        w = yaw + s * WINKEL / 2
        for z in (z_boden, z_boden + WANDHOEHE):
            m = Marker()
            m.header.frame_id = a.frame
            m.ns, m.id, m.type, m.action = "schale", i, Marker.LINE_STRIP, Marker.ADD
            m.scale.x = 0.003
            m.color = gruen
            m.pose.orientation.w = 1.0
            m.points = [Point(x=float(c[0] + r * math.cos(w)), y=float(c[1] + r * math.sin(w)),
                              z=float(z)) for r in (R_INNEN, R_AUSSEN)]
            arr.markers.append(m); i += 1
        for r in (R_INNEN, R_AUSSEN):
            m = Marker()
            m.header.frame_id = a.frame
            m.ns, m.id, m.type, m.action = "schale", i, Marker.LINE_STRIP, Marker.ADD
            m.scale.x = 0.003
            m.color = gruen
            m.pose.orientation.w = 1.0
            m.points = [Point(x=float(c[0] + r * math.cos(w)), y=float(c[1] + r * math.sin(w)),
                              z=float(zz)) for zz in (z_boden, z_boden + WANDHOEHE)]
            arr.markers.append(m); i += 1
    t = Marker()
    t.header.frame_id = a.frame
    t.ns, t.id, t.type, t.action = "schale", i, Marker.TEXT_VIEW_FACING, Marker.ADD
    t.scale.z = 0.02
    t.color = ColorRGBA(r=0.1, g=0.9, b=0.3, a=1.0)
    t.pose.orientation.w = 1.0
    mr = (R_INNEN + R_AUSSEN) / 2
    t.pose.position.x = float(c[0] + mr * math.cos(yaw))
    t.pose.position.y = float(c[1] + mr * math.sin(yaw))
    t.pose.position.z = float(z_boden + 0.06)
    t.text = text
    arr.markers.append(t)
    knoten.marker.publish(arr)


def zeichne_overlay(knoten, c, yaw, z_boden, zeilen):
    """Den gefundenen Umriss ins Kamerabild zeichnen und auf /schale/overlay geben.

    Auf diesem Rechner der bequemere Blick als eine PointCloud: liegt der Umriss auf der
    wirklichen Schale, stimmt die Lage - und ein um einen Rand verrutschter Fund (115 mm)
    faellt sofort auf. Beschriftung deutsch und ASCII-sicher.
    """
    import cv2
    if knoten.farbbild is None or knoten.K is None:
        return False
    try:
        tf = knoten.puffer.lookup_transform(knoten.depth_frame, a.frame, rclpy.time.Time())
    except Exception:                                          # noqa: BLE001
        return False
    q = tf.transform.rotation
    t = tf.transform.translation
    fx, fy, cx, cy = knoten.K
    bild = knoten.farbbild.copy()
    h, w = bild.shape[:2]

    def proj(pts):
        po = qrot_viele((q.x, q.y, q.z, q.w), np.asarray(pts, dtype=np.float64))
        po = po + np.array([t.x, t.y, t.z])
        vor = po[:, 2] > 0.05
        u = np.full(len(po), -1e9); v = np.full(len(po), -1e9)
        u[vor] = po[vor, 0] * fx / po[vor, 2] + cx
        v[vor] = po[vor, 1] * fy / po[vor, 2] + cy
        return np.stack([u, v], 1)[vor]

    GRUEN, GELB = (60, 230, 80), (60, 220, 240)
    for z, farbe, dick in ((z_boden, GELB, 1), (z_boden + WANDHOEHE, GRUEN, 2)):
        for r in (R_INNEN, R_AUSSEN):
            pts = [(c[0] + r * math.cos(yaw - WINKEL / 2 + WINKEL * i / 59),
                    c[1] + r * math.sin(yaw - WINKEL / 2 + WINKEL * i / 59), z)
                   for i in range(60)]
            xy = proj(pts)
            if len(xy) > 1:
                cv2.polylines(bild, [xy.astype(np.int32)], False, farbe, dick, cv2.LINE_AA)
        for s_ in (-1, 1):
            ww = yaw + s_ * WINKEL / 2
            xy = proj([(c[0] + r * math.cos(ww), c[1] + r * math.sin(ww), z)
                       for r in (R_INNEN, R_AUSSEN)])
            if len(xy) > 1:
                cv2.polylines(bild, [xy.astype(np.int32)], False, farbe, dick, cv2.LINE_AA)

    for i, zeile in enumerate(zeilen):
        y = 22 + 20 * i
        cv2.putText(bild, zeile, (10, y), cv2.FONT_HERSHEY_SIMPLEX, 0.55, (0, 0, 0), 3, cv2.LINE_AA)
        cv2.putText(bild, zeile, (10, y), cv2.FONT_HERSHEY_SIMPLEX, 0.55, (255, 255, 255), 1, cv2.LINE_AA)

    msg = Image()
    msg.header.frame_id = knoten.depth_frame
    msg.header.stamp = knoten.get_clock().now().to_msg()
    msg.height, msg.width = h, w
    msg.encoding = "bgr8"
    msg.is_bigendian = 0
    msg.step = 3 * w
    msg.data = bild.tobytes()
    knoten.overlay.publish(msg)
    if a.bild:
        cv2.imwrite(a.bild, bild)
    return True


def schreibe(cx, cy, cz, yaw, bericht):
    bericht = bericht.replace("--", "- -")      # "--" ist in XML-Kommentaren verboten
    if os.path.exists(POSE_XACRO):
        shutil.copy2(POSE_XACRO, POSE_XACRO + ".bak")
    with open(POSE_XACRO, "w") as f:
        f.write('''<?xml version="1.0"?>
<!--
  ================= ERZEUGTE DATEI - NICHT VON HAND AENDERN =================

  Lage der Bereitstellungsschale (robot_base -> schale). Diese Datei ist die EINZIGE
  Quelle der Schalenlage im Modell; mycobot_world.urdf.xacro liest sie hier ein.
  Die MASSE der Schale stehen NICHT hier, sondern in schale.xacro.

  Geschrieben von : tools/schale_finden.py
  Zuletzt         : %s
  Verfahren       : im Tiefenbild selbst gefunden - oertliche Erhebung als Randfilter,
                    danach Hough mit FESTEM Radius (beide Boegen stimmen gemeinsam ab),
                    Mittelpunkt mit Gauss-Newton nachgezogen, Gierwinkel aus einem
                    45.3-Grad-Fenster ueber der Winkelverteilung.

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
''' % (date.today().isoformat(), bericht, cx, cy, cz, yaw))


def auswerten(p):
    """Punktwolke in robot_base -> Lage der Schale und alle Proben. None = nichts gefunden.

    Bewusst OHNE ROS, damit das ganze Verfahren an synthetischen Szenen geprueft
    werden kann - genau daher stammen die Schwellen fuer Schaerfe und Ring.
    """
    rand, rand_z, rand_h = randzellen(p)
    print("%d Randzellen (Erhebung %.0f bis %.0f mm)"
          % (len(rand), a.erhebung[0] * 1e3, a.erhebung[1] * 1e3))
    if len(rand) < 20:
        print("ABBRUCH: zu wenige Randzellen. Sieht die Kamera die Schale ueberhaupt?")
        return None

    gx, gy, schaerfe = hough(rand)
    d = np.hypot(rand[:, 0] - gx, rand[:, 1] - gy)
    ti = np.abs(d - RI_MITTE) < 0.012
    ta = np.abs(d - RA_MITTE) < 0.012
    if ti.sum() < 5 or ta.sum() < 5:
        print("ABBRUCH: nur %d innere und %d aeussere Treffer - das ist keine Schale."
              % (ti.sum(), ta.sum()))
        return None

    # Mittelpunkt nachziehen: beide Saetze abwechselnd, jeder mit SEINEM Radius.
    ci = kreis_fest(rand[ti], RI_MITTE, (gx, gy))
    ca = kreis_fest(rand[ta], RA_MITTE, (gx, gy))
    cc = (ci + ca) / 2.0
    for _ in range(30):
        ci = kreis_fest(rand[ti], RI_MITTE, cc)
        ca = kreis_fest(rand[ta], RA_MITTE, cc)
        neu_c = (ci + ca) / 2.0
        fertig = float(np.hypot(*(neu_c - cc))) < 1e-7
        cc = neu_c
        if fertig:
            break
    uneinig = float(np.hypot(*(ci - ca)))

    # Zuordnung mit dem NACHGEZOGENEN Mittelpunkt wiederholen - der Rasterschritt der
    # Abstimmung ist 4 mm, das verschiebt die Zuordnung sonst am Rand der Toleranz.
    d = np.hypot(rand[:, 0] - cc[0], rand[:, 1] - cc[1])
    ti = np.abs(d - RI_MITTE) < 0.012
    ta = np.abs(d - RA_MITTE) < 0.012

    treffer = rand[ti | ta]
    d2 = np.hypot(treffer[:, 0] - cc[0], treffer[:, 1] - cc[1])
    soll = np.where(np.abs(d2 - RI_MITTE) < np.abs(d2 - RA_MITTE), RI_MITTE, RA_MITTE)
    rest_rms = float(np.sqrt(np.mean((d2 - soll) ** 2)))

    w = np.arctan2(treffer[:, 1] - cc[1], treffer[:, 0] - cc[0])
    yaw, _ = sektor(w)
    yaw = (yaw + math.pi) % (2 * math.pi) - math.pi

    # Wie viel des Rings liegt AUSSERHALB des 45.3-Grad-Sektors? Eine wirkliche Schale
    # hat dort nichts; ein aus Stoerkram zusammengewuerfelter Gipfel hat den Ring rundum
    # belegt. An synthetischen Szenen gemessen: echte Schale 0.04, Stoerkram 2.4 bis 2.9.
    rel = np.mod(w - (yaw - WINKEL / 2), 2 * math.pi)
    drin = rel <= WINKEL
    ring_aussen = float((~drin).sum()) / max(int(drin.sum()), 1)

    def deckung(maske):
        """Anteil der 20 Winkelfaecher des Sektors, in denen ueberhaupt etwas liegt."""
        if not maske.any():
            return 0.0
        ww = np.mod(np.arctan2(rand[maske, 1] - cc[1], rand[maske, 0] - cc[0])
                    - (yaw - WINKEL / 2), 2 * math.pi)
        ww = ww[ww <= WINKEL]
        if len(ww) < 2:
            return 0.0
        h, _ = np.histogram(ww, bins=20, range=(0, WINKEL))
        return float(np.count_nonzero(h) / 20.0)

    d_i, d_a = deckung(ti), deckung(ta)

    # Hoehe aus den URSPRUNGSPUNKTEN am Randradius, NICHT aus dem Zellenhoechstwert:
    # das Maximum einer Zelle ist um das Tiefenrauschen nach oben verzerrt (gemessen
    # +3.4 mm), das 75-Prozent-Quantil der Randpunkte nur noch um +0.8 mm.
    rr = np.hypot(p[:, 0] - cc[0], p[:, 1] - cc[1])
    tt = np.mod(np.arctan2(p[:, 1] - cc[1], p[:, 0] - cc[0]) - (yaw - WINKEL / 2), 2 * math.pi)
    am_rand = (((np.abs(rr - RI_MITTE) < 0.004) | (np.abs(rr - RA_MITTE) < 0.004))
               & (tt <= WINKEL))
    if int(am_rand.sum()) > 50:
        z_rand = float(np.percentile(p[am_rand, 2], 75))
        z_quelle = "%d Randpunkte, 75-Prozent-Quantil" % int(am_rand.sum())
    else:
        z_rand = float(np.median(np.concatenate([rand_z[ti], rand_z[ta]])))
        z_quelle = "Zellenhoechstwerte (wenige Randpunkte)"
    z_boden = z_rand - WANDHOEHE

    print("\n" + "=" * 70)
    print("GEFUNDEN")
    print("=" * 70)
    print("Mittelpunkt   : x=%.1f mm  y=%.1f mm" % (cc[0] * 1e3, cc[1] * 1e3))
    print("Innenboden z  : %.1f mm   (Randoberkante %.1f mm minus 17 mm Wandhoehe;"
          % (z_boden * 1e3, z_rand * 1e3))
    print("                %s)" % z_quelle)
    print("Gierwinkel    : %.2f Grad" % math.degrees(yaw))
    print("-" * 70)
    print("Schaerfe      : %.2f   (Gipfel gegen 99.9-Prozent-Quantil; echte Schale ~2.1,"
          % schaerfe)
    print("                blosser Stoerkram ~1.07)")
    print("Ring aussen   : %.2f   (Randzellen ausserhalb des Sektors je Zelle darin;"
          % ring_aussen)
    print("                echte Schale ~0.04, Stoerkram ~2.5)")
    print("Deckung       : innen %.0f%% , aussen %.0f%% des 45.3-Grad-Bogens"
          % (d_i * 100, d_a * 100))
    print("Treffer       : %d innen , %d aussen (von %d Randzellen)"
          % (int(ti.sum()), int(ta.sum()), len(rand)))
    print("Restfehler    : rms %.1f mm bei bekanntem Radius" % (rest_rms * 1e3))
    print("beide Boegen  : Mittelpunkte %.1f mm auseinander" % (uneinig * 1e3))
    print("-" * 70)

    zweifel = []
    if schaerfe < 1.40:
        zweifel.append("Schaerfe nur %.2f - im Abstimmraster steht kein eindeutiger Gipfel. "
                       "So sieht es aus, wenn gar keine Schale im Bild ist oder sie zur "
                       "Haelfte verdeckt wird." % schaerfe)
    if ring_aussen > 0.5:
        zweifel.append("auf %.2f Randzellen ausserhalb des Sektors kommt eine darin - eine "
                       "wirkliche Schale hat ausserhalb NICHTS. Was hier gefunden wurde, ist "
                       "eher ein Kreis durch Stoerkram." % ring_aussen)
    if min(d_i, d_a) < 0.4:
        zweifel.append("nur %.0f%% / %.0f%% der beiden Boegen sind belegt. Verdeckt der Arm "
                       "die Schale, oder steht sie halb ausserhalb des Bildes?"
                       % (d_i * 100, d_a * 100))
    if uneinig > 0.020:
        zweifel.append("innerer und aeusserer Bogen meinen Mittelpunkte, die %.0f mm "
                       "auseinanderliegen - das ist nicht ein und dieselbe Schale."
                       % (uneinig * 1e3))
    if rest_rms > 0.008:
        zweifel.append("Restfehler %.1f mm rms - fuer einen bekannten Radius zu viel."
                       % (rest_rms * 1e3))
    if len(rand) > 1500:
        zweifel.append("%d Randzellen - so viel Erhebung gehoert nicht zu einer 2 mm "
                       "starken Wand. Steht viel auf dem Tisch, oder passt --erhebung nicht?"
                       % len(rand))
    for z in zweifel:
        print("WARNUNG: " + z)
    if zweifel:
        print("-" * 70)


    bericht = ("  Schaerfe        : %.2f (Gipfel gegen 99.9-Prozent-Quantil)\n"
               "  Ring ausserhalb : %.2f Randzellen ausserhalb des Sektors je Zelle darin\n"
               "  Deckung         : innen %.0f%%, aussen %.0f%% des 45.3-Grad-Bogens\n"
               "  Treffer         : %d innen, %d aussen von %d Randzellen\n"
               "  Restfehler      : rms %.1f mm bei bekanntem Radius\n"
               "  beide Boegen    : Mittelpunkte %.1f mm auseinander"
               % (schaerfe, ring_aussen, d_i * 100, d_a * 100,
                  int(ti.sum()), int(ta.sum()), len(rand), rest_rms * 1e3, uneinig * 1e3))

    return dict(c=cc, yaw=yaw, z_boden=z_boden, zweifel=zweifel, bericht=bericht,
                schaerfe=schaerfe, ring_aussen=ring_aussen, deckung=(d_i, d_a),
                rest_rms=rest_rms, uneinig=uneinig, randzellen=len(rand))


def main():
    rclpy.init()
    k = Finder()
    print("warte auf Tiefenbild (%s) und camera_info ..." % a.depth)
    ende = k.get_clock().now().nanoseconds + int(30e9)
    while rclpy.ok() and not k.bereit() and k.get_clock().now().nanoseconds < ende:
        rclpy.spin_once(k, timeout_sec=0.2)
    if not k.bereit():
        print("ABBRUCH: kein Tiefenbild/camera_info. Laeuft die Kamera?"
              "  (ros2 topic hz braucht --qos-reliability best_effort)")
        return 1
    # TF braucht einen Moment
    ende = k.get_clock().now().nanoseconds + int(10e9)
    while rclpy.ok() and k.get_clock().now().nanoseconds < ende:
        if k.puffer.can_transform(a.frame, k.depth_frame, rclpy.time.Time()):
            break
        rclpy.spin_once(k, timeout_sec=0.2)

    try:
        p = punkte_in_base(k)
    except Exception as e:                                    # noqa: BLE001
        print("ABBRUCH: %s -> %s nicht moeglich: %s" % (k.depth_frame, a.frame, e))
        return 1
    print("%d Punkte aus %d Tiefenbildern" % (len(p), len(k.tiefen)))

    erg = auswerten(p)
    if erg is None:
        return 1
    cc, yaw, z_boden = erg["c"], erg["yaw"], erg["z_boden"]
    zweifel, bericht = erg["zweifel"], erg["bericht"]

    zeichne(k, cc, yaw, z_boden,
            "Schale: %.0f/%.0f mm, %.1f Grad"
            % (cc[0] * 1e3, cc[1] * 1e3, math.degrees(yaw)))
    zeilen = ["Schale gefunden: x=%.0f y=%.0f mm, %.1f Grad"
              % (cc[0] * 1e3, cc[1] * 1e3, math.degrees(yaw)),
              "Schaerfe %.2f  Ring aussen %.2f  Deckung %.0f/%.0f%%"
              % (erg["schaerfe"], erg["ring_aussen"],
                 erg["deckung"][0] * 100, erg["deckung"][1] * 100),
              "gruen = Oberkante, gelb = Innenboden"]
    if zweifel:
        zeilen.append("ZWEIFELHAFT - siehe Terminal")
    if zeichne_overlay(k, cc, yaw, z_boden, zeilen):
        print("Gezeichnet: /schale/erkennung (RViz) und /schale/overlay (Kamerabild).")
    else:
        print("Gezeichnet: /schale/erkennung (RViz). Kein Farbbild - kein Overlay.")

    if zweifel and a.schreiben:
        print("--schreiben, aber es gibt Zweifel: es wird NICHT geschrieben.")
        a.schreiben = False

    nehmen = a.schreiben
    if not a.schreiben and not a.nur_zeigen and not a.stumm:
        for _ in range(20):
            rclpy.spin_once(k, timeout_sec=0.05)
        nehmen = input("uebernehmen und nach schale_pose.xacro schreiben? [j/N] "
                       ).strip().lower() in ("j", "ja", "y", "yes")
    if nehmen and not a.nur_zeigen:
        schreibe(cc[0], cc[1], z_boden, yaw, bericht)
        print("geschrieben: %s   (Sicherung: %s.bak)" % (POSE_XACRO, POSE_XACRO))
        print("JETZT: SCHALE_MONTIERT=1 ./start_mycobot.sh")
    else:
        print("nichts geschrieben.")

    if a.stumm:
        for _ in range(40):
            rclpy.spin_once(k, timeout_sec=0.05)
    else:
        print("\nMarker bleiben stehen. Beenden mit Strg-C.")
        try:
            rclpy.spin(k)
        except KeyboardInterrupt:
            pass
    k.destroy_node(); rclpy.shutdown()
    return 0


if __name__ == "__main__":
    sys.exit(main())
