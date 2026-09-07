"""vida_detector — Schraubenerkennung mit YOLO11-seg + 3D-Zielerzeugung aus D435i.

Phase 2: color topic → Erkennung → /vida/overlay (Visualisierung).
Phase 4: Mitte-Pixel der gewählten Schraube + aligned depth → Kamera-Frame 3D (deproject)
       → tf2 (camera_color_optical_frame → robot_base) → /vida/target (PoseStamped)
       + /vida/target_marker (RViz). Base-Koordinaten werden zur Prüfung geloggt.

cv_bridge WIRD NICHT BENUTZT — Image<->numpy von Hand.
"""
import math
import os

import numpy as np
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image, CameraInfo
from geometry_msgs.msg import PoseStamped, Point
from visualization_msgs.msg import Marker
from std_msgs.msg import String, Int32

import tf2_ros

from vida_vision.detection import detect_screws, CLASS_NAMES, head_aware_grip

CLASS_BGR = [
    (0, 255, 255),    # gelb
    (255, 255, 255),  # weiß
    (60, 60, 60),     # schwarz
    (0, 255, 0),      # grün
    (0, 0, 255),      # rot
]
# Deutsche Farbnamen fuer das Overlay (UI ist deutsch/ASCII; CLASS_NAMES bleiben
# die im Modell eingebetteten tuerkischen Klassennamen).
CLASS_DE = ["GELB", "WEISS", "SCHWARZ", "GRUEN", "ROT"]


def image_to_bgr(msg):
    h, w = msg.height, msg.width
    buf = np.frombuffer(msg.data, dtype=np.uint8)
    enc = msg.encoding.lower()
    if enc in ("rgb8", "bgr8"):
        img = buf.reshape(h, w, 3)
        if enc == "rgb8":
            img = img[:, :, ::-1]
        return np.ascontiguousarray(img)
    raise ValueError(f"nicht unterstuetztes color encoding: {msg.encoding}")


def depth_to_mm(msg):
    """aligned_depth_to_color (16UC1, mm) -> HxW uint16 numpy."""
    if msg.encoding.lower() not in ("16uc1", "mono16"):
        raise ValueError(f"nicht unterstuetztes depth encoding: {msg.encoding}")
    return np.frombuffer(msg.data, dtype=np.uint16).reshape(msg.height, msg.width)


def bgr_to_image(img, frame_id, stamp):
    msg = Image()
    msg.header.stamp = stamp
    msg.header.frame_id = frame_id
    msg.height, msg.width = img.shape[:2]
    msg.encoding = "bgr8"
    msg.is_bigendian = 0
    msg.step = img.shape[1] * 3
    msg.data = np.ascontiguousarray(img).tobytes()
    return msg


def qrot(q, v):
    """Vektor v(3) mit Quaternion (x,y,z,w) rotieren.

    MATHEMATIK — effiziente Quaternion-Vektor-Rotation ohne Aufbau der
    Rotationsmatrix. Statt der klassischen Formel v' = q * v * q^-1 wird die
    äquivalente, numerisch günstigere Variante nach der Rodrigues-Umformung genutzt:
        t  = 2 * (q_xyz × v)
        v' = v + w * t + (q_xyz × t)
    mit q_xyz = (x,y,z) dem Vektorteil und w dem Skalarteil des Quaternions.
    Voraussetzung: q ist normiert (Einheitsquaternion).
    """
    x, y, z, w = q
    vx, vy, vz = v
    # t = 2 * cross(q_xyz, v)  (Kreuzprodukt Vektorteil x Eingangsvektor)
    tx = 2.0 * (y * vz - z * vy)
    ty = 2.0 * (z * vx - x * vz)
    tz = 2.0 * (x * vy - y * vx)
    # v' = v + w*t + cross(q_xyz, t)
    rx = vx + w * tx + (y * tz - z * ty)
    ry = vy + w * ty + (z * tx - x * tz)
    rz = vz + w * tz + (x * ty - y * tx)
    return (rx, ry, rz)


def yaw_to_quat(yaw):
    return (0.0, 0.0, math.sin(yaw / 2.0), math.cos(yaw / 2.0))


def mat_to_quat(R):
    """3x3 Rotationsmatrix (Spalten = tcp-Achsen in base) -> quaternion (x,y,z,w).

    MATHEMATIK — Umrechnung Rotationsmatrix -> Quaternion nach Shepperd/Shoemake.
    Die Spur t = R00+R11+R22 hängt mit dem Skalarteil zusammen (t = 4w^2 - 1).
    Um numerische Instabilität (Division durch nahezu 0) zu vermeiden, wird nach
    Fallunterscheidung stets über die GROESSTE Diagonalkomponente aufgelöst:
      - t > 0            -> über w,
      - sonst der Fall, in dem R00, R11 bzw. R22 maximal ist -> über x, y bzw. z.
    Zum Schluss wird das Quaternion auf Einheitslänge normiert.
    """
    t = R[0, 0] + R[1, 1] + R[2, 2]
    if t > 0.0:
        s = math.sqrt(t + 1.0) * 2.0
        w = 0.25 * s
        x = (R[2, 1] - R[1, 2]) / s
        y = (R[0, 2] - R[2, 0]) / s
        z = (R[1, 0] - R[0, 1]) / s
    elif R[0, 0] > R[1, 1] and R[0, 0] > R[2, 2]:
        s = math.sqrt(1.0 + R[0, 0] - R[1, 1] - R[2, 2]) * 2.0
        w = (R[2, 1] - R[1, 2]) / s
        x = 0.25 * s
        y = (R[0, 1] + R[1, 0]) / s
        z = (R[0, 2] + R[2, 0]) / s
    elif R[1, 1] > R[2, 2]:
        s = math.sqrt(1.0 + R[1, 1] - R[0, 0] - R[2, 2]) * 2.0
        w = (R[0, 2] - R[2, 0]) / s
        x = (R[0, 1] + R[1, 0]) / s
        y = 0.25 * s
        z = (R[1, 2] + R[2, 1]) / s
    else:
        s = math.sqrt(1.0 + R[2, 2] - R[0, 0] - R[1, 1]) * 2.0
        w = (R[1, 0] - R[0, 1]) / s
        x = (R[0, 2] + R[2, 0]) / s
        y = (R[1, 2] + R[2, 1]) / s
        z = 0.25 * s
    n = math.sqrt(x * x + y * y + z * z + w * w)
    return (x / n, y / n, z / n, w / n)


def grasp_quat_from_axis(s_base, top_down=False):
    """Greifer-Grasp-Orientierung aus der 3D-Längsachse der Schraube in base aufbauen.

    Greifer-Konvention (passend zu URDF/gripperOrientation): tcp +Y = Annäherungsachse.
    Grasp-Frame:
      - tcp +Y (eY) = Annäherung = möglichst steil tischwärts-abwärts, ABER Komponente SENKRECHT zur Schraubenachse
      - tcp +Z (eZ) = entlang der Schraubenachse (-s)
      - tcp +X (eX) = Finger-Schließachse = eY × eZ  (SENKRECHT zur Schrauben-Längsachse)
    Flach liegende Schraube (s horizontal) -> eY=(0,0,-1), also gerade-abwärts (top-down) ergibt sich natürlich.
    Aufrechte Schraube (s ~vertikal) -> Annäherung undefiniert -> None (als unzuverlässig werten).

    top_down=True: Annäherungsachse eY wird IMMER gerade abwärts (0,0,-1) erzwungen;
      die gemessene PCA-Neigung (Seitenneigung) wird IGNORIERT, nur der Yaw (horizontale Längsachse
      der Schraube) wird beibehalten → Greifer fährt von oben SENKRECHT herab, Finger schließen
      senkrecht zum Schaft. Verhindert, dass beim schrägen Herabfahren Nachbarschrauben
      angestoßen und die Szene gestört wird (Nutzerwunsch).

    Returns (x,y,z,w) oder None.
    """
    s = np.asarray(s_base, dtype=np.float64)
    n = np.linalg.norm(s)
    if n < 1e-9:
        return None
    s = s / n
    if s[2] < 0.0:        # Achse so wählen, dass sie nach oben zeigt
        s = -s
    if top_down:
        # SENKRECHTE ANNAEHERUNG: eY = gerade abwärts; eZ = horizontale Längsachse der Schraube (Yaw).
        eY = np.array([0.0, 0.0, -1.0])
        sh = np.array([s[0], s[1], 0.0])
        nsh = np.linalg.norm(sh)
        eZ = sh / nsh if nsh > 1e-6 else np.array([1.0, 0.0, 0.0])  # ~vertikale Schraube: beliebiger Yaw
        eX = np.cross(eY, eZ)
        nx = np.linalg.norm(eX)
        if nx < 1e-9:
            return None
        eX = eX / nx
        eZ = np.cross(eX, eY)               # voll orthonormal
        return mat_to_quat(np.column_stack([eX, eY, eZ]))
    down = np.array([0.0, 0.0, -1.0])
    a = down - np.dot(down, s) * s      # zur Schraubenachse senkrechte Komponente von tisch-abwärts
    na = np.linalg.norm(a)
    if na < 1e-3:                        # Schraube ~vertikal: Annäherung undefiniert
        return None
    eY = a / na
    eZ = -s
    eX = np.cross(eY, eZ)
    nx = np.linalg.norm(eX)
    if nx < 1e-9:
        return None
    eX = eX / nx
    eZ = np.cross(eX, eY)               # voll orthonormal garantiert
    R = np.column_stack([eX, eY, eZ])
    return mat_to_quat(R)


class VidaDetector(Node):
    def __init__(self):
        super().__init__("vida_detector")

        self.declare_parameter("image_topic", "/camera/color/image_raw")
        self.declare_parameter("depth_topic", "/camera/aligned_depth_to_color/image_raw")
        self.declare_parameter("caminfo_topic", "/camera/color/camera_info")
        self.declare_parameter("base_frame", "robot_base")
        self.declare_parameter("weights", "")
        self.declare_parameter("conf", 0.70)   # Standard an run_pick_preview.sh angeglichen (0.85 war zu streng: reale Picks lagen bei 0.72-0.75)
        # WAHRNEHMUNGS-OFFSET (base-Frame, m): Die Kamera liest systematisch
        # versetzt (Lineal-Messung 2026-07-15: ca. +7/+11 mm in X/Y). Die Korrektur
        # wird beim VERÖFFENTLICHEN des Ziels addiert (interne Geometrie/Overlay
        # bleiben roh, damit die Rückprojektion weiter auf der Welle liegt).
        # Wertequelle: ~/ros2_ws/.place_offset.json — dieselbe Datei wie
        # click_place_moveit.py; das Ablaufskript reicht sie als Parameter durch.
        self.declare_parameter("offset_x", 0.0)
        self.declare_parameter("offset_y", 0.0)
        self.declare_parameter("offset_z", 0.0)
        self.declare_parameter("rate_hz", 1.0)
        self.declare_parameter("enable_3d", True)
        # Tiefenfilter (am kleinen/glänzenden Schraubenkopf ist das D435i-IR verrauscht):
        #   depth_win: Fensterradius (px) -> (2*win+1)^2 Proben. 5 = 11x11.
        #   depth_min_valid: mindestens so viele nicht-ungültige (>0) Proben nötig.
        #   depth_mad_k: Werte mit |x-med| > k*MAD gelten als Ausreißer und werden verworfen.
        self.declare_parameter("depth_win", 5)
        self.declare_parameter("depth_min_valid", 8)
        self.declare_parameter("depth_mad_k", 2.5)
        # Reichweiten-Kreis (in base-XY-Ebene, auf Plattformhöhe):
        #   reach_radius     = SICHERER Reichweitenradius für senkrechten Griff (grün) [m]
        #   reach_radius_max = KINEMATISCHES Limit für senkrechten Griff (amber) [m]
        #   reach_z          = Plattformhöhe (base z) [m]
        # Quelle: URDF-FK-Monte-Carlo + Hardware-Test (Greifer ganz senkrecht, 10cm pre-grasp):
        #   sicher=200mm (r<200 wird im ersten Yaw gelöst), kinematisches Senkrecht-Limit=260mm
        #   (früher 220/300; 300 war UNEINGESCHRAENKTES Limit, im top-down irreführend — Sitzung 21).
        #   DAUERHAFTER Standard: im Kamera-Overlay werden die Kreise stets mit diesen Werten gezeichnet.
        self.declare_parameter("reach_radius", 0.20)
        self.declare_parameter("reach_radius_max", 0.26)
        # reach_radius_min = INNERE Grenze: näher als das (r<min) faltet sich der Arm im
        #   top-down-Griff zu stark → Arme stoßen aneinander/an die base (self/base collision).
        #   Gültiger Bereich ist ein DONUT: [min, reach_radius] grün. DEVLOG: r≲130mm zu nah, 150-190mm ideal.
        self.declare_parameter("reach_radius_min", 0.15)
        self.declare_parameter("reach_z", 0.008)
        # 3D-Neigungsmessung (Schraubenachse) — PCA aus der Tiefe der Maskenpixel:
        #   axis_subsample   : nur jedes N-te Maskenpixel nehmen (Geschwindigkeit)
        #   axis_min_points  : Mindestanzahl Punkte mit gültiger Tiefe für PCA
        #   axis_min_linearity: (l1-l2)/l1 Vertrauensschwelle; darunter ist die Messung UNZUVERLAESSIG
        #                       -> /vida/target wird NICHT veröffentlicht (Roboter wartet, Nutzer mischt um)
        #   grasp_tilt_min_deg: Schwelle zur Information (darunter = praktisch top-down)
        self.declare_parameter("axis_subsample", 3)
        self.declare_parameter("axis_min_points", 25)
        self.declare_parameter("axis_min_linearity", 0.6)
        self.declare_parameter("grasp_tilt_min_deg", 15.0)
        # top_down=True: Grasp-Orientierung IMMER gerade-abwärts (schräge PCA wird ignoriert,
        #   nur Yaw beibehalten) → Greifer fährt von oben senkrecht herab, gleitet nicht schräg
        #   in Nachbarschrauben. Da die Schrauben in der Kiste ~flach liegen, ist der Standard an (Nutzerwunsch).
        self.declare_parameter("top_down", True)
        # Ziel-Veröffentlichungs-Gatter:
        #   target_z_min/max: liegt BASE z außerhalb dieses Bereichs, gilt depth als Müll (m)
        #   mode_consensus: /vida/target nicht veröffentlichen, solange nicht die letzten N Zyklen im SELBEN Modus übereinstimmen
        #     (Tiefenschwankungen verhindern ein fälschliches Ein-Frame-top-down/TILT-Latch)
        self.declare_parameter("target_z_min", -0.02)
        self.declare_parameter("target_z_max", 0.12)
        self.declare_parameter("mode_consensus", 3)
        #   sticky_radius: gibt es innerhalb dieses Radius (m) um das zuletzt veröffentlichte Ziel einen Kandidaten, DIESEN wählen
        #     (klebrige Verfolgung — conf-Jitter kann das Ziel nicht zwischen zwei Schrauben springen lassen)
        self.declare_parameter("sticky_radius", 0.05)
        # kopf-bewusster Griff: Greifpunkt vom Masken-Centroid auf den geraden Schaft
        #   direkt unterhalb des Schrauben-KOPFES verschieben (sicherster Parallelbacken-Griff).
        #   grip_under_head=False -> altes Verhalten (Centroid). grip_inset_frac =
        #   Anteil der Verschiebung von der Kopfkante Richtung tip nach innen (Vielfaches der Schraubenlänge).
        self.declare_parameter("grip_under_head", True)
        self.declare_parameter("grip_inset_frac", 0.30)
        # ZIELAUSWAHL — die eine "mittigste + bestsichtbare" Schraube:
        #   Score = center_weight * Mittigkeit + conf_weight * confidence.
        #   Mitte = Pixel-Centroid ALLER erkannten Schrauben (= Schraubenhaufen/Kistenmitte;
        #     keine Kalibrierung nötig, passt sich automatisch an, wo die Kiste steht). Mittigkeit=1 in der Mitte, 0 ganz außen.
        self.declare_parameter("center_weight", 1.0)
        self.declare_parameter("conf_weight", 1.0)
        # HARTE ECKEN-ELIMINIERUNG: die achsenparallele bbox der Kandidaten (= innere Kistengrenze) wird berechnet;
        #   Schrauben näher als corner_edge_frac*bbox am Rand (Kistenwand → breiter Greifer stößt an)
        #   werden KOMPLETT aus der Kandidatenliste entfernt. Sind alle am Rand, wird kein Ziel VEROEFFENTLICHT (Roboter wartet).
        #   corner_min_screws: gibt es weniger Kandidaten, ist die Kistengeometrie nicht bestimmbar → Eliminierung übersprungen
        #     (Sicherheit, um die letzten paar Schrauben nicht ewig als Ecke zu werten und festzusetzen).
        self.declare_parameter("corner_edge_frac", 0.15)
        self.declare_parameter("corner_min_screws", 4)
        # KAMERA-BILDRAND (FOV) ELIMINIERUNG: Schrauben sehr nah am Rand des Kamera-Sichtfelds
        #   ODER über den Rahmen hinausragend (Maske berührt den Rand → halb-abgeschnitten) können KEIN Ziel sein.
        #   Grund: am FOV-Rand ist die Tiefe unzuverlässig und eine abgeschnittene Maske verfälscht Centroid/PCA-Winkel/
        #   Greifpunkt → falsches Ziel. frame_margin_frac: sichere Zone um diesen Anteil der Bild-Breite/Höhe
        #   nach innen; Schrauben, deren Mitte AUSSERHALB dieser Zone liegt oder deren
        #   Maske den Bildrand berührt, werden eliminiert. 0 → aus.
        self.declare_parameter("frame_margin_frac", 0.04)
        # OVERLAY-VEREINFACHUNG: Schrauben UNTERHALB dieser conf werden auf dem Bildschirm GAR NICHT gezeichnet
        #   (Reduzierung der Unübersichtlichkeit). Bei den gezeichneten nur ein Punkt — kein Kreis/Text.
        #   (0.9 → 0.85 gesenkt, Nutzerwunsch 2026-07-16: auch etwas unsicherere
        #   Wellen im Fenster zeigen)
        self.declare_parameter("display_min_conf", 0.85)

        self.image_topic = self.get_parameter("image_topic").value
        self.depth_topic = self.get_parameter("depth_topic").value
        self.caminfo_topic = self.get_parameter("caminfo_topic").value
        self.base_frame = self.get_parameter("base_frame").value
        self.conf = float(self.get_parameter("conf").value)
        rate = float(self.get_parameter("rate_hz").value)
        self.enable_3d = bool(self.get_parameter("enable_3d").value)
        self._depth_win = int(self.get_parameter("depth_win").value)
        self._depth_min_valid = int(self.get_parameter("depth_min_valid").value)
        self._depth_mad_k = float(self.get_parameter("depth_mad_k").value)
        self.reach_radius = float(self.get_parameter("reach_radius").value)
        self.reach_radius_max = float(self.get_parameter("reach_radius_max").value)
        self.reach_radius_min = float(self.get_parameter("reach_radius_min").value)
        self.reach_z = float(self.get_parameter("reach_z").value)
        self._axis_subsample = max(1, int(self.get_parameter("axis_subsample").value))
        self._axis_min_points = int(self.get_parameter("axis_min_points").value)
        self._axis_min_linearity = float(self.get_parameter("axis_min_linearity").value)
        self._grasp_tilt_min_deg = float(self.get_parameter("grasp_tilt_min_deg").value)
        self._top_down = bool(self.get_parameter("top_down").value)
        self._target_z_min = float(self.get_parameter("target_z_min").value)
        self._target_z_max = float(self.get_parameter("target_z_max").value)
        self._sticky_radius = float(self.get_parameter("sticky_radius").value)
        self._grip_under_head = bool(self.get_parameter("grip_under_head").value)
        self._grip_inset_frac = float(self.get_parameter("grip_inset_frac").value)
        self._center_weight = float(self.get_parameter("center_weight").value)
        self._conf_weight = float(self.get_parameter("conf_weight").value)
        self._corner_edge_frac = float(self.get_parameter("corner_edge_frac").value)
        self._corner_min_screws = int(self.get_parameter("corner_min_screws").value)
        self._frame_margin_frac = float(self.get_parameter("frame_margin_frac").value)
        self._display_min_conf = float(self.get_parameter("display_min_conf").value)
        self._offset = (float(self.get_parameter("offset_x").value),
                        float(self.get_parameter("offset_y").value),
                        float(self.get_parameter("offset_z").value))
        if any(abs(v) > 1e-9 for v in self._offset):
            self.get_logger().info(
                f"Ziel-Offset AKTIV: ({self._offset[0]*1000:.1f}, "
                f"{self._offset[1]*1000:.1f}, {self._offset[2]*1000:.1f}) mm "
                "wird auf /vida/target addiert")
        self._last_pub_p = None  # BASE-Position des zuletzt veröffentlichten Ziels (klebrige Auswahl)
        self._last_axis_base = None  # BASE-3D-Achse des zuletzt veröffentlichten Ziels (gesperrtes Overlay)
        from collections import deque
        self._mode_hist = deque(maxlen=max(1, int(self.get_parameter("mode_consensus").value)))

        weights = self.get_parameter("weights").value
        if not weights:
            from ament_index_python.packages import get_package_share_directory
            weights = os.path.join(
                get_package_share_directory("vida_vision"), "weights", "vida_large.pt"
            )
        self.get_logger().info(f"Modell wird geladen: {weights}")
        from ultralytics import YOLO
        self.model = YOLO(weights)
        # WARMUP: die ERSTE CUDA/cuDNN-Inferenz dauert auf dem Nano ~100s (Kernel-
        # Initialisierung). Ohne Warmup fror das erste Kamerabild scheinbar minutenlang
        # ein (Sitzung 2026-07-16). Dummy-Inferenz hier → "Modell bereit" heisst WIRKLICH bereit.
        self.get_logger().info("CUDA-Warmup laeuft (erste Inferenz, auf dem Nano 1-2 min)...")
        self.model.predict(np.zeros((480, 848, 3), dtype=np.uint8), conf=0.5, verbose=False)
        self.get_logger().info(f"Modell bereit (task={self.model.task}, Klassen={CLASS_NAMES})")

        self._latest = None      # (bgr, stamp, frame_id)
        self._depth = None       # HxW uint16 (mm)
        self._K = None           # (fx, fy, cx, cy)

        self.sub_img = self.create_subscription(Image, self.image_topic, self._on_image, 1)
        self.pub_overlay = self.create_publisher(Image, "/vida/overlay", 1)

        if self.enable_3d:
            self.sub_depth = self.create_subscription(Image, self.depth_topic, self._on_depth, 1)
            self.sub_info = self.create_subscription(CameraInfo, self.caminfo_topic, self._on_info, 1)
            self.pub_target = self.create_publisher(PoseStamped, "/vida/target", 1)
            self.pub_marker = self.create_publisher(Marker, "/vida/target_marker", 1)
            # Ziel-FARBE/Info (damit nach Schließen des Kamerafensters aus dem Terminal/Latch
            # ersichtlich ist, zu welcher Schraube gefahren wird). target_latch capture hört dies ab + speichert.
            self.pub_target_info = self.create_publisher(String, "/vida/target_info", 1)
            # FARBKLASSE der Zielschraube (class_id, 0..4 = CLASS_NAMES-Reihenfolge). pick_tilt
            # liest dies GLEICHZEITIG mit /vida/target → legt die gegriffene Schraube in den richtigen Farbbehälter
            # (Sortierung nach Farbe). Wird gleichzeitig mit _publish_target veröffentlicht.
            self.pub_target_color = self.create_publisher(Int32, "/vida/target_color", 1)
            self.pub_axis = self.create_publisher(Marker, "/vida/axis_marker", 1)
            self.tf_buffer = tf2_ros.Buffer()
            self.tf_listener = tf2_ros.TransformListener(self.tf_buffer, self)

        self.timer = self.create_timer(1.0 / max(rate, 0.1), self._on_timer)
        self.get_logger().info(
            f"Lausche: {self.image_topic} | inference {rate:.1f} Hz | 3D={'an' if self.enable_3d else 'aus'}"
        )

    # ---- Abonnenten-Callbacks ----
    def _on_image(self, msg):
        try:
            self._latest = (image_to_bgr(msg), msg.header.stamp, msg.header.frame_id)
        except Exception as e:  # noqa: BLE001
            self.get_logger().warn(f"color Konvertierungsfehler: {e}", throttle_duration_sec=5.0)

    def _on_depth(self, msg):
        try:
            self._depth = depth_to_mm(msg)
        except Exception as e:  # noqa: BLE001
            self.get_logger().warn(f"depth Konvertierungsfehler: {e}", throttle_duration_sec=5.0)

    def _on_info(self, msg):
        k = msg.k  # 3x3 row-major
        self._K = (k[0], k[4], k[2], k[5])  # fx, fy, cx, cy

    # ---- Hilfsfunktionen ----
    def _depth_m(self, u, v, win=None):
        """Robuste Tiefe um (u,v): Median über breites Fenster + MAD-Ausreißer-
        Eliminierung. Verwirft die nah-unsinnigen IR-Messungen am kleinen/glänzenden
        Schraubenkopf; setzt sich mit den umliegenden Oberflächenproben auf die echte Tiefe. Meter oder None."""
        if self._depth is None:
            return None
        if win is None:
            win = self._depth_win
        h, w = self._depth.shape
        u, v = int(round(u)), int(round(v))
        if not (0 <= u < w and 0 <= v < h):
            return None
        patch = self._depth[max(0, v - win):v + win + 1, max(0, u - win):u + win + 1]
        vals = patch[patch > 0].astype(np.float32)
        if vals.size < self._depth_min_valid:
            return None
        med = float(np.median(vals))
        # Ausreißer-Eliminierung per MAD (nahe IR-Reflexionen + ferne Lücken-Pixel)
        mad = float(np.median(np.abs(vals - med)))
        if mad > 0.0:
            keep = vals[np.abs(vals - med) <= self._depth_mad_k * mad]
            if keep.size >= self._depth_min_valid:
                med = float(np.median(keep))
        return med / 1000.0

    def _deproject(self, u, v, z):
        """Pixel + Tiefe (m) -> optischer Frame 3D (m). Z vorne, X rechts, Y unten."""
        fx, fy, cx, cy = self._K
        x = (u - cx) * z / fx
        y = (v - cy) * z / fy
        return (x, y, z)

    def _to_base(self, p_opt, src_frame):
        """Punkt aus optischem Frame nach robot_base überführen. (x,y,z) oder None."""
        try:
            tf = self.tf_buffer.lookup_transform(
                self.base_frame, src_frame, rclpy.time.Time())
        except Exception as e:  # noqa: BLE001
            self.get_logger().warn(f"kein TF ({src_frame}->{self.base_frame}): {e}",
                                   throttle_duration_sec=5.0)
            return None
        t = tf.transform.translation
        r = tf.transform.rotation
        rx, ry, rz = qrot((r.x, r.y, r.z, r.w), p_opt)
        return (rx + t.x, ry + t.y, rz + t.z)

    # ---- Hilfsfunktionen Reichweiten-Kreis ----
    def _opt_tf(self, src_frame):
        """robot_base -> optischer Frame Transform als (quat, trans) mit EINEM lookup holen."""
        if self._K is None or not self.enable_3d:
            return None
        try:
            tf = self.tf_buffer.lookup_transform(src_frame, self.base_frame, rclpy.time.Time())
        except Exception:  # noqa: BLE001
            return None
        t = tf.transform.translation
        r = tf.transform.rotation
        return ((r.x, r.y, r.z, r.w), (t.x, t.y, t.z))

    def _proj(self, p_base, otf):
        """base-Punkt -> Pixel (u,v). Hinter der Kamera -> None."""
        q, t = otf
        x, y, z = qrot(q, p_base)
        x += t[0]; y += t[1]; z += t[2]
        if z <= 1e-3:
            return None
        fx, fy, cx, cy = self._K
        return (fx * x / z + cx, fy * y / z + cy)

    def _reach_polygon(self, otf, radius, n=72):
        """Kreis in base-XY auf Höhe reach_z in eine Pixelliste projizieren."""
        pts = []
        for i in range(n):
            a = 2.0 * math.pi * i / n
            p = self._proj((radius * math.cos(a), radius * math.sin(a), self.reach_z), otf)
            pts.append(None if p is None else (int(round(p[0])), int(round(p[1]))))
        return pts

    def _screw_base(self, d):
        """Die base-3D (mit Tiefe) einer Erkennung zurückgeben. (x,y,z) oder None."""
        if self._K is None or not self.enable_3d or self._latest is None:
            return None
        z = self._depth_m(*d["center_px"])
        if z is None:
            return None
        return self._to_base(self._deproject(*d["center_px"], z), self._latest[2])

    # ---- 3D-Achsen-(Neigungs-)Messung ----
    def _mask_points_3d(self, mask):
        """Pixel der Schraubenmaske mit roher aligned-depth in optischen-Frame-3D umwandeln.

        Unter-Sampling (Geschwindigkeit), Filter depth>0, eliminiert Tiefen-Ausreißer per MAD.
        Returns Nx3 (optischer Frame, m) oder None (zu wenige Punkte)."""
        if self._depth is None or self._K is None:
            return None
        ys, xs = np.where(mask > 127)
        if xs.size < self._axis_min_points:
            return None
        st = self._axis_subsample
        if st > 1:
            ys, xs = ys[::st], xs[::st]
        h, w = self._depth.shape
        ok = (xs >= 0) & (xs < w) & (ys >= 0) & (ys < h)
        ys, xs = ys[ok], xs[ok]
        d_mm = self._depth[ys, xs].astype(np.float32)
        valid = d_mm > 0
        ys, xs, d_mm = ys[valid], xs[valid], d_mm[valid]
        if d_mm.size < self._axis_min_points:
            return None
        # MAD-Ausreißer-Eliminierung (Hintergrund / nahe IR-Reflexion am Maskenrand)
        med = float(np.median(d_mm))
        mad = float(np.median(np.abs(d_mm - med)))
        if mad > 0.0:
            keep = np.abs(d_mm - med) <= self._depth_mad_k * mad
            ys, xs, d_mm = ys[keep], xs[keep], d_mm[keep]
        if d_mm.size < self._axis_min_points:
            return None
        fx, fy, cx, cy = self._K
        z = d_mm / 1000.0
        x = (xs.astype(np.float32) - cx) * z / fx
        y = (ys.astype(np.float32) - cy) * z / fy
        return np.column_stack([x, y, z]).astype(np.float64)

    def _axis_3d(self, pts):
        """PCA aus Punktwolke -> (größter Eigenvektor=Achsenrichtung(optisch), linearity, N).

        linearity = (l1-l2)/l1 ∈ [0,1]; nahe 1 = ausgeprägt linear (zuverlässig)."""
        n = pts.shape[0]
        c = pts.mean(axis=0)
        cov = np.cov((pts - c).T)
        w, V = np.linalg.eigh(cov)        # aufsteigend sortiert
        l1, l2 = float(w[2]), float(w[1])
        linearity = (l1 - l2) / l1 if l1 > 1e-12 else 0.0
        axis_opt = V[:, 2]                # größter Eigenvektor
        return axis_opt, linearity, n

    def _mask_dims_mm(self, mask, angle_deg, z):
        """Major/Minor px-Ausdehnung der Maske -> ABSOLUTE mm (Kamera-Maßstab z/fx).

        KEINE 44mm-Längen-ANNAHME; der Maßstab kommt direkt aus der Tiefe (z):
        bei Tiefe z ist 1px = z/fx Meter. Gibt (länge_mm, dicke_mm)
        oder None zurück. dicke = das die Silhouette senkrecht schneidende breiteste Maß = das Minimum,
        das der Greifer öffnen muss (Schraubenkopf = breiteste Stelle)."""
        if self._K is None:
            return None
        ys, xs = np.where(mask > 127)
        if xs.size < 10:
            return None
        fx, fy, cx, cy = self._K
        cxm, cym = float(np.mean(xs)), float(np.mean(ys))
        a = math.radians(angle_deg)
        ca, sa = math.cos(a), math.sin(a)
        major = (xs - cxm) * ca + (ys - cym) * sa     # Projektion auf Längsachse
        minor = -(xs - cxm) * sa + (ys - cym) * ca    # Kurzachse (senkrecht)
        len_px = float(major.max() - major.min())
        # Breite: ausreißer-robustes Perzentil (97-3). max-min bläht auf.
        wid_px = float(np.percentile(minor, 97) - np.percentile(minor, 3))
        mmpp = z / fx * 1000.0                         # z(m), fx(px) -> mm/px
        return len_px * mmpp, wid_px * mmpp

    # ---- Hauptschleife ----
    def _on_timer(self):
        if self._latest is None:
            self.get_logger().warn("noch kein color (Kamera an? use_camera:=true)",
                                   throttle_duration_sec=5.0)
            return
        bgr, stamp, frame_id = self._latest
        dets = detect_screws(bgr, self.model, conf=self.conf)

        if not dets:
            self.pub_overlay.publish(bgr_to_image(self._draw(bgr, []), frame_id, stamp))
            self.get_logger().info("keine Schrauben", throttle_duration_sec=3.0)
            return

        if not self.enable_3d:
            top = dets[0]
            self.pub_overlay.publish(bgr_to_image(self._draw(bgr, dets, chosen=top), frame_id, stamp))
            self.get_logger().info(
                f"{len(dets)} Schrauben | beste: {top['class_name']} {top['conf']:.2f} "
                f"Mitte=({top['center_px'][0]:.0f},{top['center_px'][1]:.0f}) Winkel={top['angle_deg']:.0f}°")
            return

        if self._K is None:
            self.pub_overlay.publish(bgr_to_image(self._draw(bgr, dets), frame_id, stamp))
            self.get_logger().warn("camera_info nicht eingetroffen", throttle_duration_sec=5.0)
            return

        # Auswahl (DETERMINISTISCH): aus den Kandidaten mit gültiger Tiefe+TF —
        #   1) der nächste Kandidat innerhalb sticky_radius zum zuletzt VEROEFFENTLICHTEN Ziel (klebrige
        #      Verfolgung: conf-Jitter kann das Ziel nicht zwischen zwei Schrauben springen lassen),
        #   2) ohne klebrige Übereinstimmung die höchste confidence (dets nach conf sortiert).
        cands = []
        for d in dets:
            z = self._depth_m(*d["center_px"])
            if z is None:
                continue
            p_opt = self._deproject(*d["center_px"], z)
            p_base = self._to_base(p_opt, frame_id)
            if p_base is None:
                continue
            cands.append((d, z, p_opt, p_base))
        if not cands:
            self.pub_overlay.publish(bgr_to_image(self._draw(bgr, dets), frame_id, stamp))
            self.get_logger().warn(f"{len(dets)} Schrauben, aber bei keiner ist Tiefe/TF gueltig",
                                   throttle_duration_sec=3.0)
            return

        # --- KAMERA-BILDRAND (FOV) ELIMINIERUNG ---
        # VOR der Markierung als Ziel: ist die Schraube in der sicheren Zone des Kamera-Sichtfelds
        # und GANZ im Bild? Am FOV-Rand ist die Tiefe unzuverlässig, eine abgeschnittene Maske verfälscht Winkel/Greifpunkt.
        # Schrauben, deren Mitte außerhalb der sicheren Zone liegt ODER deren Maske den Bildrand berührt, werden eliminiert.
        if self._frame_margin_frac > 0.0:
            H, W = bgr.shape[:2]
            mx = self._frame_margin_frac * W
            my = self._frame_margin_frac * H
            in_frame = []
            for c in cands:
                cu, cv = c[0]["center_px"]
                if not (mx <= cu <= W - mx and my <= cv <= H - my):
                    continue  # Mitte außerhalb der sicheren FOV-Zone
                m = c[0]["mask"] > 127
                if m[0, :].any() or m[-1, :].any() or m[:, 0].any() or m[:, -1].any():
                    continue  # Maske berührt den Bildrand → halb-abgeschnittene Schraube
                in_frame.append(c)
            dropped = len(cands) - len(in_frame)
            if not in_frame:
                self.pub_overlay.publish(bgr_to_image(self._draw(bgr, dets), frame_id, stamp))
                self.get_logger().warn(
                    f"{len(cands)} Schrauben, aber alle am Rand des Kamerabilds/ragen hinaus — "
                    "keine geeignete Schraube im sicheren FOV, Roboter wartet", throttle_duration_sec=3.0)
                return
            if dropped:
                self.get_logger().info(
                    f"Kamerarand: {dropped} Schrauben eliminiert (ausserhalb FOV/abgeschnitten), "
                    f"{len(in_frame)} Kandidaten verbleiben", throttle_duration_sec=3.0)
            cands = in_frame

        # --- REICHWEITEN- (IK) INNENGRENZEN-ELIMINIERUNG ---
        # Der ROTE Innenkreis im Overlay = reach_radius_min. Schrauben mit kleinerem base-XY-Radius
        # sind dem Roboter ZU NAH → im top-down-Griff biegt sich der Arm zu stark (IK-Limit).
        # Der Kreis wurde NUR gezeichnet; hier wenden wir ihn auch auf die Zielauswahl an:
        # Schrauben innerhalb des Innenkreises werden aus den Kandidaten AUSGESCHLOSSEN, nicht als Ziel markiert.
        if self.reach_radius_min > 0.0:
            in_reach = []
            for c in cands:
                pb = c[3]  # p_base (base XY, m)
                if math.hypot(pb[0], pb[1]) < self.reach_radius_min:
                    continue  # innerhalb des roten Innenkreises → zu nah, IK-Limit
                in_reach.append(c)
            dropped = len(cands) - len(in_reach)
            if not in_reach:
                self.pub_overlay.publish(bgr_to_image(self._draw(bgr, dets), frame_id, stamp))
                self.get_logger().warn(
                    f"{len(cands)} Schrauben, aber alle im roten Innenkreis "
                    f"(r<{self.reach_radius_min*1000:.0f}mm, zu nah am Roboter/IK-Limit) — "
                    "Roboter wartet", throttle_duration_sec=3.0)
                return
            if dropped:
                self.get_logger().info(
                    f"IK-Innengrenze: {dropped} Schrauben eliminiert (r<{self.reach_radius_min*1000:.0f}mm zu nah), "
                    f"{len(in_reach)} Kandidaten verbleiben", throttle_duration_sec=3.0)
            cands = in_reach

        # --- ZIELAUSWAHL: die eine "mittigste + bestsichtbare" Schraube ---
        # Mitte = Pixel-Centroid der Kandidatenschrauben (= Schraubenhaufen / Kistenmitte).
        centers = np.array([c[0]["center_px"] for c in cands], dtype=float)
        centroid = centers.mean(axis=0)
        dists = np.linalg.norm(centers - centroid, axis=1)

        # HARTE ECKEN-ELIMINIERUNG: bbox der Kandidaten (innere Kistengrenze); randnahe
        #   (Kistenwand → breiter Greifer stößt an) Schrauben aus den Kandidaten AUSSCHLIESSEN.
        keep = list(range(len(cands)))
        if len(cands) >= self._corner_min_screws:
            umin, vmin = centers.min(axis=0)
            umax, vmax = centers.max(axis=0)
            mx = self._corner_edge_frac * max(umax - umin, 1.0)
            my = self._corner_edge_frac * max(vmax - vmin, 1.0)
            keep = [i for i in range(len(cands))
                    if umin + mx <= centers[i][0] <= umax - mx
                    and vmin + my <= centers[i][1] <= vmax - my]
            if not keep:
                self.pub_overlay.publish(bgr_to_image(
                    self._draw(bgr, dets, centroid=centroid), frame_id, stamp))
                self.get_logger().warn(
                    f"{len(cands)} Schrauben, aber alle am Kistenrand (Ecke) — "
                    "keine geeignete Schraube in der Mitte, Roboter wartet", throttle_duration_sec=3.0)
                return

        # Score = center_weight*Mittigkeit + conf_weight*confidence. Höchste wählen.
        maxd = float(dists.max()) if float(dists.max()) > 1e-6 else 1.0

        def _score(i):
            centrality = 1.0 - dists[i] / maxd        # Mitte=1, am weitesten=0
            return (self._center_weight * centrality
                    + self._conf_weight * float(cands[i][0]["conf"]))
        chosen_i = max(keep, key=_score)

        # KLEBRIGE VERFOLGUNG: gibt es nach der Eliminierung einen Kandidaten innerhalb
        #   sticky_radius zum vorherigen Ziel, diesen beibehalten — Jitter soll das Ziel nicht zwischen zwei Schrauben springen lassen.
        if self._last_pub_p is not None and len(keep) > 1:
            best = min(keep, key=lambda i: math.dist(tuple(cands[i][3]), self._last_pub_p))
            if math.dist(tuple(cands[best][3]), self._last_pub_p) <= self._sticky_radius:
                chosen_i = best
        d, z, p_opt, p_base = cands[chosen_i]
        u, v = d["center_px"]

        # --- KOPF-BEWUSSTER GRIFF: Ziel auf den geraden Schaft unter dem Kopf verschieben ---
        # Aus der Maskenform die Kopfkante finden; Greifpunkt direkt unter den Kopf verschieben.
        # Bei gültiger Tiefe wird das Ziel aus diesem Punkt erzeugt (sonst bleibt der Centroid).
        grip_px = None
        head_px = None
        if self._grip_under_head:
            g = head_aware_grip(d["mask"] > 127, d["center_px"][0], d["center_px"][1],
                                d["angle_deg"], inset_frac=self._grip_inset_frac)
            head_px = g["head_px"]
            if g["head_known"]:
                gu, gv = g["grip_px"]
                gz = self._depth_m(gu, gv)
                if gz is not None:
                    gp_opt = self._deproject(gu, gv, gz)
                    gp_base = self._to_base(gp_opt, frame_id)
                    if gp_base is not None:
                        u, v = gu, gv
                        z, p_opt, p_base = gz, gp_opt, gp_base
                        grip_px = (gu, gv)

        # --- 3D-Achsen-(Neigungs-)Messung: PCA aus der Maskentiefe ---
        pts = self._mask_points_3d(d["mask"])
        axis_base = None
        linearity = 0.0
        npts = 0
        if pts is not None:
            axis_opt, linearity, npts = self._axis_3d(pts)
            tip_base = self._to_base(
                (p_opt[0] + axis_opt[0] * 0.05,
                 p_opt[1] + axis_opt[1] * 0.05,
                 p_opt[2] + axis_opt[2] * 0.05), frame_id)
            if tip_base is not None:
                v_axis = np.array([tip_base[0] - p_base[0],
                                   tip_base[1] - p_base[1],
                                   tip_base[2] - p_base[2]])
                nb = float(np.linalg.norm(v_axis))
                if nb > 1e-9:
                    axis_base = v_axis / nb

        grasp_q = (grasp_quat_from_axis(axis_base, top_down=self._top_down)
                   if axis_base is not None else None)
        z_ok = self._target_z_min <= p_base[2] <= self._target_z_max
        reliable = (pts is not None and grasp_q is not None
                    and npts >= self._axis_min_points
                    and linearity >= self._axis_min_linearity
                    and z_ok)

        tilt_deg = 0.0
        if axis_base is not None:
            s2 = abs(float(axis_base[2]))
            tilt_deg = math.degrees(math.asin(min(1.0, s2)))

        # --- Maßbestimmung (Länge + Dicke), absolute mm, KEINE 44mm-Annahme ---
        dims = self._mask_dims_mm(d["mask"], d["angle_deg"], z)
        dim_txt = f" lng={dims[0]:.0f} dck={dims[1]:.1f}mm" if dims else ""

        mode = None
        if reliable:
            if self._top_down:
                mode = "SENKRECHT"   # Annäherung immer gerade-abwärts (top_down erzwungen)
            else:
                mode = "NEIGUNGS-AUSGERICHTET" if tilt_deg >= self._grasp_tilt_min_deg else "top-down"
        self._mode_hist.append(mode)
        steady = (mode is not None
                  and len(self._mode_hist) == self._mode_hist.maxlen
                  and all(m == mode for m in self._mode_hist))

        if steady:
            base_txt = (f"x={p_base[0]*1000:.0f} y={p_base[1]*1000:.0f} z={p_base[2]*1000:.0f}mm "
                        f"tilt={tilt_deg:.0f} lin={linearity:.2f} [{mode}]{dim_txt}")
            self.pub_overlay.publish(
                bgr_to_image(self._draw(bgr, dets, chosen=d, base_txt=base_txt,
                                        axis3d=(p_base, axis_base),
                                        grip_px=grip_px, head_px=head_px,
                                        centroid=centroid), frame_id, stamp))
            # WAHRNEHMUNGS-OFFSET nur auf das VERÖFFENTLICHTE Ziel anwenden —
            # Overlay/sticky bleiben roh (Rückprojektion soll auf der Welle liegen).
            p_pub = (p_base[0] + self._offset[0],
                     p_base[1] + self._offset[1],
                     p_base[2] + self._offset[2])
            self._publish_target(p_pub, grasp_q, stamp)
            self.pub_target_color.publish(Int32(data=int(d["class_id"])))  # gleichzeitig mit /vida/target
            info = (f"{d['class_name'].upper()} Schraube conf={d['conf']:.2f} | "
                    f"x={p_pub[0]*1000:.0f} y={p_pub[1]*1000:.0f} z={p_pub[2]*1000:.0f}mm")
            self.pub_target_info.publish(String(data=info))
            self._last_pub_p = tuple(p_base)  # klebrige Auswahl + Referenz für gesperrtes Overlay (ROH)
            self._last_axis_base = tuple(axis_base) if axis_base is not None else None
            self._publish_marker(p_pub, grasp_q, d, stamp)
            self._publish_axis(p_pub, axis_base, stamp)
            self.get_logger().info(
                f"ZIEL {d['class_name']} conf={d['conf']:.2f} | BASE(korr.) "
                f"x={p_pub[0]*1000:.0f} y={p_pub[1]*1000:.0f} z={p_pub[2]*1000:.0f}mm "
                f"| tilt={tilt_deg:.0f}° lin={linearity:.2f} N={npts}{dim_txt} | mod={mode} -> /vida/target")
        elif reliable:
            # Messung gut, aber Modus noch nicht in Folge übereinstimmend — nicht veröffentlichen, warten
            base_txt = (f"x={p_base[0]*1000:.0f} y={p_base[1]*1000:.0f} z={p_base[2]*1000:.0f}mm "
                        f"tilt={tilt_deg:.0f} [{mode}?] Konsens wird erwartet{dim_txt}")
            self.pub_overlay.publish(
                bgr_to_image(self._draw(bgr, dets, chosen=d, base_txt=base_txt,
                                        axis3d=(p_base, axis_base),
                                        grip_px=grip_px, head_px=head_px,
                                        centroid=centroid), frame_id, stamp))
            self.get_logger().info(
                f"Konsens wird erwartet ({mode}, {sum(1 for m in self._mode_hist if m == mode)}"
                f"/{self._mode_hist.maxlen}) | tilt={tilt_deg:.0f}° (/vida/target NICHT)")
        else:
            if pts is not None and grasp_q is not None and not z_ok:
                warn = (f"BASE z={p_base[2]*1000:.0f}mm ausserhalb Grenze "
                        f"[{self._target_z_min*1000:.0f},{self._target_z_max*1000:.0f}] - depth Muell")
            elif pts is not None:
                warn = f"tilt UNZUVERLAESSIG lin={linearity:.2f} N={npts} - Schrauben umruehren"
            else:
                warn = "Tiefe unzureichend - Schrauben umruehren"
            self.pub_overlay.publish(
                bgr_to_image(self._draw(bgr, dets, chosen=d, base_txt=warn,
                                        grip_px=grip_px, head_px=head_px,
                                        centroid=centroid), frame_id, stamp))
            self.get_logger().warn(
                f"{warn} | conf={d['conf']:.2f} BASE x={p_base[0]*1000:.0f} "
                f"y={p_base[1]*1000:.0f} z={p_base[2]*1000:.0f}mm  (/vida/target NICHT)",
                throttle_duration_sec=2.0)

    def _publish_target(self, p, q, stamp):
        """q = Grasp-Orientierung (x,y,z,w), tcp +Y = Annäherungsachse. pick_tilt nutzt dies."""
        msg = PoseStamped()
        msg.header.stamp = stamp
        msg.header.frame_id = self.base_frame
        msg.pose.position.x, msg.pose.position.y, msg.pose.position.z = p
        msg.pose.orientation.x, msg.pose.orientation.y, msg.pose.orientation.z, msg.pose.orientation.w = q
        self.pub_target.publish(msg)

    def _publish_marker(self, p, q, d, stamp):
        """Grasp-ANNAEHERUNGSachse (tcp +Y) mit zweiseitigem Pfeil zeigen: der Greifer fährt
        in dieser Richtung zur Schraube. Pfeilspitze = Grasp-Punkt (p), Schaftende = Gegenrichtung der Annäherung."""
        ay = qrot(q, (0.0, 1.0, 0.0))    # Richtung von tcp +Y in base = Annäherung
        m = Marker()
        m.header.frame_id = self.base_frame
        m.header.stamp = stamp
        m.ns = "vida_target"
        m.id = 0
        m.type = Marker.ARROW
        m.action = Marker.ADD
        tail = Point(x=p[0] - ay[0] * 0.06, y=p[1] - ay[1] * 0.06, z=p[2] - ay[2] * 0.06)
        tip = Point(x=p[0], y=p[1], z=p[2])
        m.points = [tail, tip]
        m.scale.x, m.scale.y, m.scale.z = 0.004, 0.010, 0.0
        col = CLASS_BGR[d["class_id"]] if d["class_id"] < len(CLASS_BGR) else (128, 128, 128)
        m.color.b, m.color.g, m.color.r = col[0] / 255.0, col[1] / 255.0, col[2] / 255.0
        m.color.a = 1.0
        self.pub_marker.publish(m)

    def _publish_axis(self, p, axis_base, stamp):
        """Die gemessene 3D-Längsachse der Schraube mit magenta ARROW (zweiseitig) zeigen (Diagnose)."""
        m = Marker()
        m.header.frame_id = self.base_frame
        m.header.stamp = stamp
        m.ns = "vida_axis"
        m.id = 0
        m.type = Marker.ARROW
        m.action = Marker.ADD
        half = 0.03
        p0 = Point(x=p[0] - axis_base[0] * half, y=p[1] - axis_base[1] * half,
                   z=p[2] - axis_base[2] * half)
        p1 = Point(x=p[0] + axis_base[0] * half, y=p[1] + axis_base[1] * half,
                   z=p[2] + axis_base[2] * half)
        m.points = [p0, p1]
        m.scale.x, m.scale.y, m.scale.z = 0.004, 0.008, 0.0   # shaft, head Durchmesser
        m.color.r, m.color.g, m.color.b, m.color.a = 1.0, 0.0, 1.0, 1.0   # magenta
        self.pub_axis.publish(m)

    def _draw(self, bgr, dets, chosen=None, base_txt=None, axis3d=None,
              grip_px=None, head_px=None, centroid=None):
        import cv2
        ov = bgr.copy()

        # --- HAUFENMITTE: Mittenreferenz der Zielauswahl (magenta), Ecken-Eliminierung danach ---
        if centroid is not None:
            mcx, mcy = int(centroid[0]), int(centroid[1])
            cv2.drawMarker(ov, (mcx, mcy), (255, 0, 255), cv2.MARKER_CROSS, 22, 2)
            cv2.putText(ov, "Mitte", (mcx + 10, mcy - 6),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.42, (255, 0, 255), 1, cv2.LINE_AA)

        # --- REICHWEITEN-KREIS: um robot_base, auf Plattformhöhe ---
        # Das INNERE des gezeichneten Kreises = mit senkrechtem Griff erreichbare Zone für Schrauben.
        AMBER, GREENC, REDC = (0, 200, 255), (0, 200, 0), (0, 0, 255)
        otf = self._opt_tf(self._latest[2]) if self._latest else None
        if otf is not None:
            try:
                for radius, col, thick in ((self.reach_radius_max, AMBER, 1),
                                           (self.reach_radius, GREENC, 2),
                                           (self.reach_radius_min, REDC, 2)):
                    if radius <= 0:
                        continue
                    poly = self._reach_polygon(otf, radius)
                    valid = [p for p in poly if p is not None]
                    if len(valid) >= 2:
                        closed = all(p is not None for p in poly)
                        cv2.polylines(ov, [np.array(valid, np.int32)], closed,
                                      col, thick, cv2.LINE_AA)
                c = self._proj((0.0, 0.0, self.reach_z), otf)
                if c is not None:
                    cc = (int(c[0]), int(c[1]))
                    cv2.drawMarker(ov, cc, AMBER, cv2.MARKER_TILTED_CROSS, 16, 2)
                    cv2.putText(ov, f"ROBOTER  gruen {self.reach_radius_min*1000:.0f}-{self.reach_radius*1000:.0f}  amber<{self.reach_radius_max*1000:.0f}mm (innen rot=zu nah)",
                                (cc[0] + 8, cc[1] + 4), cv2.FONT_HERSHEY_SIMPLEX,
                                0.42, AMBER, 1, cv2.LINE_AA)
            except Exception:  # noqa: BLE001
                pass

        # --- GESPERRTES ZIEL: zuletzt veröffentlichtes Ziel + dessen Achse; bleibt im Overlay
        # FEST (cyan), selbst wenn der Roboter die Schraube verdeckt und die Live-Erkennung wegfällt ---
        if otf is not None and getattr(self, "_last_pub_p", None) is not None:
            CYAN = (255, 255, 0)
            lp = self._proj(self._last_pub_p, otf)
            if lp is not None:
                lx, ly = int(lp[0]), int(lp[1])
                cv2.drawMarker(ov, (lx, ly), CYAN, cv2.MARKER_TILTED_CROSS, 18, 1)
                cv2.putText(ov, "GESPERRT", (lx + 8, ly + 16),
                            cv2.FONT_HERSHEY_SIMPLEX, 0.42, CYAN, 1, cv2.LINE_AA)
                lab = getattr(self, "_last_axis_base", None)
                if lab is not None:
                    half = 0.03
                    pa = self._proj((self._last_pub_p[0] - lab[0] * half,
                                     self._last_pub_p[1] - lab[1] * half,
                                     self._last_pub_p[2] - lab[2] * half), otf)
                    pb = self._proj((self._last_pub_p[0] + lab[0] * half,
                                     self._last_pub_p[1] + lab[1] * half,
                                     self._last_pub_p[2] + lab[2] * half), otf)
                    if pa is not None and pb is not None:
                        cv2.line(ov, (int(pa[0]), int(pa[1])), (int(pb[0]), int(pb[1])),
                                 CYAN, 1, cv2.LINE_AA)

        shown = 0
        for d in dets:
            # OVERLAY-VEREINFACHUNG (Wunsch): Schrauben mit niedriger conf GAR NICHT zeichnen; bei den gezeichneten
            #   nur ein PUNKT — KEIN Kreis (Reichweitenring) und kein Text (Farbe/conf/r=).
            if float(d.get("conf", 1.0)) < self._display_min_conf:
                continue
            shown += 1
            color = CLASS_BGR[d["class_id"]] if d["class_id"] < len(CLASS_BGR) else (128, 128, 128)
            mb = d["mask"] > 127
            ov[mb] = (ov[mb] * 0.5 + np.array(color) * 0.5).astype(np.uint8)
            cx, cy = int(d["center_px"][0]), int(d["center_px"][1])
            cv2.circle(ov, (cx, cy), 3, (0, 0, 255), -1)   # nur Punkt

        # GEWAEHLTE Schraube: Längsachsen-Linie (immer) + Kopfmarkierung + Greif-Fadenkreuz.
        if chosen is not None:
            GREEN, ORANGE, REDH = (0, 255, 0), (0, 165, 255), (0, 0, 255)
            cx, cy = int(chosen["center_px"][0]), int(chosen["center_px"][1])

            # LAENGSACHSE (immer sichtbar): in welche Richtung die Schraube lang ist.
            ang = math.radians(float(chosen.get("angle_deg", 0.0)))
            half = 0.5 * float(chosen.get("major_len_px", 40.0))
            dx, dy = math.cos(ang) * half, math.sin(ang) * half
            cv2.line(ov, (int(cx - dx), int(cy - dy)), (int(cx + dx), int(cy + dy)),
                     ORANGE, 2, cv2.LINE_AA)

            # KOPFkante (falls erkannt): der Griff verschiebt sich direkt DARUNTER.
            if head_px is not None:
                hx, hy = int(head_px[0]), int(head_px[1])
                cv2.circle(ov, (hx, hy), 6, REDH, 2, cv2.LINE_AA)
                cv2.putText(ov, "KOPF", (hx + 7, hy - 6),
                            cv2.FONT_HERSHEY_SIMPLEX, 0.45, REDH, 1, cv2.LINE_AA)

            # GRIFF/ZIEL-Punkt: Schaft unter dem Kopf (sonst Centroid). BLAU (Wunsch: nicht mit
            #   den Schraubenfarben verwechseln → unterscheidbares blaues Kreuz + "ZIEL"-Text).
            BLUE = (255, 0, 0)
            gx, gy = (int(grip_px[0]), int(grip_px[1])) if grip_px is not None else (cx, cy)
            cv2.drawMarker(ov, (gx, gy), BLUE, cv2.MARKER_CROSS, 16, 2)
            # ZIEL + FARBE der gewaehlten Welle (Wunsch 2026-07-16): Farbname in der
            # Klassenfarbe mit dunkler Kontur (auf jedem Hintergrund lesbar), duenn (1).
            cid = int(chosen.get("class_id", -1))
            farbe = CLASS_DE[cid] if 0 <= cid < len(CLASS_DE) else "?"
            fcol = CLASS_BGR[cid] if 0 <= cid < len(CLASS_BGR) else (200, 200, 200)
            ziel_txt = f"ZIEL: {farbe}"
            cv2.putText(ov, ziel_txt, (gx + 10, gy - 6),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 0, 0), 3, cv2.LINE_AA)
            cv2.putText(ov, ziel_txt, (gx + 10, gy - 6),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.5, fcol, 1, cv2.LINE_AA)
            # Zusaetzlich fest oben rechts — auch wenn das Fadenkreuz am Rand liegt.
            cv2.putText(ov, ziel_txt, (ov.shape[1] - 150, 20),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 0, 0), 3, cv2.LINE_AA)
            cv2.putText(ov, ziel_txt, (ov.shape[1] - 150, 20),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.5, fcol, 1, cv2.LINE_AA)
            if base_txt:
                # Untere xyz-Info: klein, dünn (thickness=1), WEISS — soll nicht überladen.
                cv2.putText(ov, base_txt, (10, ov.shape[0] - 10),
                            cv2.FONT_HERSHEY_SIMPLEX, 0.4, (255, 255, 255), 1, cv2.LINE_AA)

        # --- 3D-ACHSE (PCA) der GEWAEHLTEN Schraube ins Bild projiziert (Prüfung) ---
        # MAGENTA-Linie = gemessene Schraubenachse; die Linie sollte entlang des Schraubenkörpers
        # verlaufen. GELBER Kreis = base->rückprojizierter Greifpunkt;
        # fällt er nicht auf die Schraube, sind deprojection/TF inkonsistent (Kalibrierung).
        if axis3d is not None and otf is not None:
            p0, ax = axis3d
            MAGENTA, YELLOW = (255, 0, 255), (0, 255, 255)
            L = 0.04  # Achse um ±4 cm verlängern (insgesamt ~8 cm Linie)
            a = self._proj((p0[0] - ax[0] * L, p0[1] - ax[1] * L, p0[2] - ax[2] * L), otf)
            b = self._proj((p0[0] + ax[0] * L, p0[1] + ax[1] * L, p0[2] + ax[2] * L), otf)
            if a is not None and b is not None:
                cv2.line(ov, (int(a[0]), int(a[1])), (int(b[0]), int(b[1])),
                         MAGENTA, 2, cv2.LINE_AA)
                cv2.circle(ov, (int(b[0]), int(b[1])), 4, MAGENTA, -1)
            g = self._proj(p0, otf)
            if g is not None:
                cv2.circle(ov, (int(g[0]), int(g[1])), 6, YELLOW, 2, cv2.LINE_AA)

        cv2.putText(ov, f"{shown}/{len(dets)} Schrauben (conf>={self._display_min_conf:.2f})", (10, 20),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.5, (255, 255, 255), 1, cv2.LINE_AA)
        return ov


def main():
    rclpy.init()
    node = VidaDetector()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.try_shutdown()


if __name__ == "__main__":
    main()
