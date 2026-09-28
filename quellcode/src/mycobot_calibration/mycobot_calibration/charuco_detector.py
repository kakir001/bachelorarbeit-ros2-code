#!/usr/bin/env python3
# ChArUco-Board-Detektor — publiziert die Transformation camera_optical_frame -> charuco_board als TF.
# Wird von easy_handeye2 als tracking_marker-Quelle verwendet: aus dem laufenden
# Kamerabild wird die 6-DoF-Pose des ChArUco-Boards geschätzt und im TF-Baum
# bereitgestellt, sodass die Hand-Auge-Kalibrierung Samples aufnehmen kann.
#
# cv_bridge wird bewusst NICHT verwendet: das System-cv_bridge auf diesem Jetson
# ist gegen das aktualisierte OpenCV (4.8) inkompatibel/defekt. Daher konvertieren
# wir sensor_msgs/Image direkt in ein numpy-Array (siehe image_msg_to_bgr).

import math
import time
from typing import Optional, Tuple

import cv2
import cv2.aruco as aruco
import numpy as np

import rclpy
from rclpy.node import Node
from rclpy.duration import Duration
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy, qos_profile_sensor_data

from sensor_msgs.msg import Image, CameraInfo
from std_msgs.msg import Int32
from geometry_msgs.msg import TransformStamped
from tf2_ros import StaticTransformBroadcaster


_ARUCO_DICT_BY_NAME = {
    'DICT_4X4_50':   aruco.DICT_4X4_50,
    'DICT_4X4_100':  aruco.DICT_4X4_100,
    'DICT_4X4_250':  aruco.DICT_4X4_250,
    'DICT_5X5_50':   aruco.DICT_5X5_50,
    'DICT_5X5_100':  aruco.DICT_5X5_100,
    'DICT_5X5_250':  aruco.DICT_5X5_250,
    'DICT_6X6_50':   aruco.DICT_6X6_50,
    'DICT_6X6_100':  aruco.DICT_6X6_100,
    'DICT_6X6_250':  aruco.DICT_6X6_250,
}


def image_msg_to_bgr(msg: Image) -> np.ndarray:
    """Wandelt eine sensor_msgs/Image direkt in ein BGR-numpy-Bild um.

    Ersatz für cv_bridge (auf diesem Jetson defekt). Der rohe Byte-Puffer der
    Nachricht wird ohne Kopie als uint8-Array interpretiert und je nach Encoding
    in die von OpenCV erwartete BGR-Reihenfolge gebracht. Unterstützt werden die
    von der RealSense D435i gelieferten Encodings bgr8, rgb8 und mono8.
    """
    arr = np.frombuffer(msg.data, dtype=np.uint8)
    enc = msg.encoding.lower()
    if enc == 'bgr8':
        return arr.reshape((msg.height, msg.width, 3)).copy()
    if enc == 'rgb8':
        return cv2.cvtColor(
            arr.reshape((msg.height, msg.width, 3)), cv2.COLOR_RGB2BGR
        )
    if enc == 'mono8':
        return cv2.cvtColor(arr.reshape((msg.height, msg.width)), cv2.COLOR_GRAY2BGR)
    raise ValueError(f'Unsupported image encoding: {msg.encoding}')


def bgr_to_image_msg(bgr: np.ndarray, header) -> Image:
    """Verpackt ein BGR-numpy-Bild wieder in eine sensor_msgs/Image.

    Gegenrichtung zu image_msg_to_bgr: dient zum Publizieren des annotierten
    Debug-Bildes (mit gezeichneten Ecken, Achsen und Statusbanner). Der
    übergebene header (Zeitstempel/frame_id des Originalbildes) wird übernommen,
    damit RViz das annotierte Bild korrekt zuordnet.
    """
    out = Image()
    out.header = header
    out.height = bgr.shape[0]
    out.width = bgr.shape[1]
    out.encoding = 'bgr8'
    out.is_bigendian = 0
    out.step = bgr.shape[1] * 3
    out.data = bgr.tobytes()
    return out


def rvec_tvec_to_quat_xyz(
    rvec: np.ndarray, tvec: np.ndarray
) -> Tuple[float, float, float, float, float, float, float]:
    """Wandelt das solvePnP-Ergebnis (rvec, tvec) in Translation + Quaternion um.

    solvePnP liefert die Orientierung als Rodrigues-Rotationsvektor rvec und die
    Position als Translationsvektor tvec. Für eine TF/geometry_msgs-Transformation
    wird jedoch ein Quaternion benötigt. rvec wird per cv2.Rodrigues in eine
    3x3-Rotationsmatrix R überführt und daraus mit Shepperds Methode ein
    numerisch stabiles Quaternion (qx, qy, qz, qw) berechnet. Rückgabe:
    (tx, ty, tz, qx, qy, qz, qw).
    """
    R, _ = cv2.Rodrigues(rvec)
    # OpenCV-Rotationsmatrix -> Quaternion (w, x, y, z) mit Shepperds Methode.
    # Es wird der Zweig mit der größten Diagonal-/Spurkomponente gewählt, um
    # eine Division durch (nahe) null und damit numerische Instabilität zu vermeiden.
    trace = R[0, 0] + R[1, 1] + R[2, 2]
    if trace > 0.0:
        s = 0.5 / math.sqrt(trace + 1.0)
        qw = 0.25 / s
        qx = (R[2, 1] - R[1, 2]) * s
        qy = (R[0, 2] - R[2, 0]) * s
        qz = (R[1, 0] - R[0, 1]) * s
    elif R[0, 0] > R[1, 1] and R[0, 0] > R[2, 2]:
        s = 2.0 * math.sqrt(1.0 + R[0, 0] - R[1, 1] - R[2, 2])
        qw = (R[2, 1] - R[1, 2]) / s
        qx = 0.25 * s
        qy = (R[0, 1] + R[1, 0]) / s
        qz = (R[0, 2] + R[2, 0]) / s
    elif R[1, 1] > R[2, 2]:
        s = 2.0 * math.sqrt(1.0 + R[1, 1] - R[0, 0] - R[2, 2])
        qw = (R[0, 2] - R[2, 0]) / s
        qx = (R[0, 1] + R[1, 0]) / s
        qy = 0.25 * s
        qz = (R[1, 2] + R[2, 1]) / s
    else:
        s = 2.0 * math.sqrt(1.0 + R[2, 2] - R[0, 0] - R[1, 1])
        qw = (R[1, 0] - R[0, 1]) / s
        qx = (R[0, 2] + R[2, 0]) / s
        qy = (R[1, 2] + R[2, 1]) / s
        qz = 0.25 * s
    tx, ty, tz = float(tvec[0]), float(tvec[1]), float(tvec[2])
    return tx, ty, tz, qx, qy, qz, qw


class CharucoDetector(Node):
    """ROS-2-Node zur ChArUco-Board-Detektion und TF-Publikation.

    Ablauf: abonniert das Farbbild und die CameraInfo der RealSense D435i,
    detektiert das ChArUco-Board im Bild, schätzt dessen Pose per solvePnP und
    publiziert die Transformation camera_frame -> board_frame als (statische) TF.
    Zusätzlich wird ein annotiertes Debug-Bild sowie die Anzahl erkannter Ecken
    veröffentlicht. Ein HOLD-Mechanismus republiziert die letzte gültige Pose
    mit frischem Zeitstempel, damit easy_handeye2 trotz der niedrigen
    Detektionsrate auf dem Jetson stabil Samples aufnehmen kann.
    """

    def __init__(self) -> None:
        super().__init__('charuco_detector')

        # Board-Geometrie — Standardwerte entsprechen config/charuco_params.yaml.
        # Werden zur Laufzeit über ROS-Parameter (bzw. die YAML-Datei) überschrieben.
        self.declare_parameter('squares_x', 5)
        self.declare_parameter('squares_y', 7)
        self.declare_parameter('square_length_m', 0.030)
        self.declare_parameter('marker_length_m', 0.022)
        self.declare_parameter('aruco_dict', 'DICT_5X5_100')

        # Frames + Topics: Namen der Koordinatensysteme (Kamera-optisch, Board)
        # sowie der abonnierten Bild- und CameraInfo-Topics der RealSense.
        self.declare_parameter('camera_frame', 'camera_color_optical_frame')
        # Vorgabe 'charuco_gemessen', NICHT 'charuco_board': letzteres ist seit
        # 2026-09-09 der Name des URDF-Links der Platte. Gleicher Name = derselbe
        # Frame mit zwei Eltern, und TF antwortet dann mit der MODELL-Lage - jede
        # Messung vergliche das Modell mit sich selbst, ohne Fehlermeldung.
        self.declare_parameter('board_frame', 'charuco_gemessen')
        self.declare_parameter('image_topic', '/camera/color/image_raw')
        self.declare_parameter('camera_info_topic', '/camera/color/camera_info')

        # Detektions-Drosselung (Hz) — die D435i streamt mit 30 Hz, für die
        # Kalibrierung genügen aber wenige Hz an stabilen Detektionen. Begrenzt
        # zugleich die CPU-Last auf dem Jetson.
        self.declare_parameter('publish_rate_hz', 5.0)
        # Mindestanzahl erkannter ChArUco-Ecken, bevor einer Pose vertraut wird.
        # Zu wenige Ecken -> schlecht konditionierte, unzuverlässige PnP-Lösung.
        self.declare_parameter('min_corners', 8)

        # HOLD: Der Detektor-Durchsatz liegt auf dem Jetson unter Volllast bei
        # ~0.7 fps. Dadurch veraltet die camera->board-TF zwischen zwei Detektionen
        # und der "Take Sample"-Button in easy_handeye2 flackert an/aus. Solange das
        # Board stillsteht (Servos während der Kalibrierung gesperrt), bleibt die
        # letzte Pose gültig, daher republizieren wir sie mit gleichmäßiger Rate
        # und frischem Zeitstempel. Trifft innerhalb von hold_timeout_s keine neue
        # Detektion ein, stoppt das Republizieren — so kann ein aus dem Bild
        # verschwundenes Board nicht fälschlich als veraltete Pose gesampelt werden.
        self.declare_parameter('hold_timeout_s', 2.0)
        self.declare_parameter('republish_rate_hz', 15.0)
        # Zeitstempel um diesen Betrag in die Zukunft datieren. handeye_server
        # schlägt die Transformation etwa bei now() nach; die frischeste Detektion
        # ist aber bis zu eine Republish-Periode alt, sodass ein Lookup bei now()
        # in die Zukunft extrapolieren müsste und tf2 dies ablehnt. Ein
        # vorgezogener Stempel garantiert, dass der Puffer now() stets abdeckt.
        # Unschädlich, solange das Board stillsteht (während des Samplings).
        self.declare_parameter('stamp_lead_s', 0.3)

        sx = self.get_parameter('squares_x').value
        sy = self.get_parameter('squares_y').value
        sq = float(self.get_parameter('square_length_m').value)
        mk = float(self.get_parameter('marker_length_m').value)
        dict_name = self.get_parameter('aruco_dict').value
        if dict_name not in _ARUCO_DICT_BY_NAME:
            raise ValueError(
                f'Unknown aruco_dict "{dict_name}". '
                f'Known: {sorted(_ARUCO_DICT_BY_NAME)}'
            )

        # ArUco-Wörterbuch, ChArUco-Board-Modell und Detektor aus den Parametern
        # aufbauen. Das Board-Modell hält die physische Geometrie (Feld-/Markermaße)
        # und wird später für matchImagePoints (2D<->3D-Korrespondenzen) genutzt.
        dictionary = aruco.getPredefinedDictionary(_ARUCO_DICT_BY_NAME[dict_name])
        self._board = aruco.CharucoBoard((sx, sy), sq, mk, dictionary)
        self._detector = aruco.CharucoDetector(self._board)

        self._camera_frame = self.get_parameter('camera_frame').value
        self._board_frame = self.get_parameter('board_frame').value
        self._min_corners = int(self.get_parameter('min_corners').value)
        self._min_period = 1.0 / float(self.get_parameter('publish_rate_hz').value)
        self._hold_timeout = float(self.get_parameter('hold_timeout_s').value)
        republish_rate = float(self.get_parameter('republish_rate_hz').value)
        self._stamp_lead = Duration(seconds=float(self.get_parameter('stamp_lead_s').value))

        # Kamera-Intrinsics: K (3x3-Kameramatrix) und D (Verzeichnungskoeffizienten)
        # werden erst aus der CameraInfo-Nachricht befüllt; bis dahin None.
        self._K: Optional[np.ndarray] = None
        self._D: Optional[np.ndarray] = None
        self._last_pub = 0.0
        self._frames_seen = 0
        self._detections = 0
        # Letzte gültige Pose (tx,ty,tz,qx,qy,qz,qw) und ihr Detektionszeitpunkt,
        # für den HOLD-Mechanismus (Republikation zwischen den Detektionen).
        self._last_tf: Optional[Tuple[float, float, float, float, float, float, float]] = None
        self._last_detect_t = 0.0

        # STATISCHER Broadcaster: tf2 behandelt statische Transformationen als für
        # ALLE Zeitpunkte gültig, sodass der exact-time-Lookup von handeye_server
        # stets gelingt, obwohl der Detektor auf dem ausgelasteten Jetson nur
        # ~0.6 fps liefert. Hier korrekt, weil Board + Roboter während des
        # Samplings stillstehen.
        self._tf_pub = StaticTransformBroadcaster(self)
        self._corners_pub = self.create_publisher(Int32, '~/corners_detected', 10)

        # Annotiertes Bild: grüne Ecken/Marker + Pose-Achsen + Bereit-zum-Aufnehmen-
        # Banner. In RViz per Image-Display auf /charuco_detector/image_annotated
        # abonnierbar (visuelle Kontrolle der Detektion).
        self._annotated_pub = self.create_publisher(
            Image, '~/image_annotated', 10
        )

        # CameraInfo: latched-äquivalente QoS — RELIABLE, KEEP_LAST, Tiefe 1.
        # Die Intrinsics ändern sich nicht, daher genügt die jeweils letzte Nachricht.
        qos = QoSProfile(
            reliability=ReliabilityPolicy.RELIABLE,
            history=HistoryPolicy.KEEP_LAST,
            depth=1,
        )
        self.create_subscription(
            CameraInfo,
            self.get_parameter('camera_info_topic').value,
            self._on_camera_info,
            qos,
        )
        # RealSense2 RGB stream wird RELIABLE publiziert. Das sensor_data Preset
        # (BEST_EFFORT) ist mit einem RELIABLE Publisher inkompatibel — DDS verbindet nicht.
        # SystemDefault (RELIABLE/KEEP_LAST/10) passt.
        self.create_subscription(
            Image,
            self.get_parameter('image_topic').value,
            self._on_image,
            10,
        )

        # Zwei Timer: periodische Statistik-Ausgabe und der HOLD-Republish-Tick.
        self.create_timer(5.0, self._log_stats)
        self.create_timer(1.0 / republish_rate, self._on_hold_tick)

        self.get_logger().info(
            f'ChArUco {sx}x{sy} square={sq*1000:.1f}mm marker={mk*1000:.1f}mm '
            f'dict={dict_name} -> {self._camera_frame} -> {self._board_frame}'
        )

    def _on_camera_info(self, msg: CameraInfo) -> None:
        """Callback für CameraInfo: speichert die Kamera-Intrinsics K und D.

        Ohne K/D ist keine PnP-Poseschätzung möglich, daher wartet _on_image,
        bis diese Werte einmal empfangen wurden. Zusätzlich wird der optische
        Frame aus dem Nachrichten-Header übernommen (maßgeblich gegenüber dem Parameter).
        """
        self._K = np.array(msg.k, dtype=np.float64).reshape(3, 3)
        self._D = np.array(msg.d, dtype=np.float64)
        # Den OPTISCHEN Frame aus dem Nachrichten-Header verwenden — überschreibt
        # bei Bedarf den gesetzten Parameter, damit die TF im korrekten Frame hängt.
        if msg.header.frame_id and msg.header.frame_id != self._camera_frame:
            self.get_logger().info(
                f'camera_info frame_id="{msg.header.frame_id}" '
                f'overrides param "{self._camera_frame}"'
            )
            self._camera_frame = msg.header.frame_id

    def _on_image(self, msg: Image) -> None:
        """Callback für jedes Kamerabild: Detektion, Poseschätzung, Publikation.

        Kernablauf des Nodes:
          1) Rate begrenzen (publish_rate_hz), Bild in BGR wandeln, in Graustufen konvertieren.
          2) Mit dem CharucoDetector die Board-Ecken (charuco_corners/ids) und die
             zugrunde liegenden ArUco-Marker detektieren.
          3) Genügend Ecken vorausgesetzt, per matchImagePoints 2D<->3D-Korrespondenzen
             bilden und mit cv2.solvePnP (SOLVEPNP_ITERATIVE) die Board-Pose lösen.
          4) Ein annotiertes Debug-Bild mit Markern, Ecken, Pose-Achsen und farbigem
             Statusbanner zeichnen und publizieren; Eckenzahl publizieren.
          5) Bei erfolgreicher Pose die TF broadcasten und als HOLD-Pose merken.
        """
        self._frames_seen += 1
        now = time.monotonic()
        # Ratenbegrenzung: nur alle _min_period Sekunden verarbeiten (spart CPU).
        if (now - self._last_pub) < self._min_period:
            return

        try:
            bgr = image_msg_to_bgr(msg)
        except ValueError as e:
            self.get_logger().warn(str(e), throttle_duration_sec=5.0)
            return
        gray = cv2.cvtColor(bgr, cv2.COLOR_BGR2GRAY)

        status = 'WAITING FOR CAMERA INFO'
        ok = False
        rvec = tvec = None
        charuco_corners = charuco_ids = None
        marker_corners = marker_ids = None

        # Nur schätzen, wenn die Kamera-Intrinsics bereits vorliegen.
        if self._K is not None:
            # ChArUco-Board im Graustufenbild detektieren: liefert die inneren
            # Schachbrettecken (subpixelgenau) samt IDs sowie die ArUco-Marker.
            charuco_corners, charuco_ids, marker_corners, marker_ids = \
                self._detector.detectBoard(gray)

            n_corners = 0 if charuco_corners is None else len(charuco_corners)
            if n_corners < self._min_corners:
                # Zu wenige Ecken -> keine vertrauenswürdige Pose möglich.
                status = f'NO BOARD  (corners {n_corners}/{self._min_corners})'
            else:
                # 2D-Bildpunkte den 3D-Board-Modellpunkten zuordnen (Korrespondenzen).
                obj_points, img_points = self._board.matchImagePoints(
                    charuco_corners, charuco_ids
                )
                if obj_points is None or len(obj_points) < 4:
                    # solvePnP benötigt mindestens 4 Korrespondenzen.
                    status = f'PARTIAL  (corners {n_corners})'
                else:
                    # Perspective-n-Point lösen: liefert Rotations- (rvec) und
                    # Translationsvektor (tvec) des Boards im Kamera-Frame.
                    ok, rvec, tvec = cv2.solvePnP(
                        obj_points, img_points, self._K, self._D,
                        flags=cv2.SOLVEPNP_ITERATIVE,
                    )
                    if ok:
                        status = (
                            f'READY TO SAMPLE  (corners {n_corners})  '
                            f'-> easy_handeye2 rqt: "Take Sample"'
                        )
                    else:
                        status = 'POSE SOLVE FAILED'

        # Overlays zeichnen — grüne Marker, grüne Ecken, RGB-Pose-Achsen.
        if marker_corners is not None and len(marker_corners) > 0:
            aruco.drawDetectedMarkers(
                bgr, marker_corners, marker_ids, borderColor=(0, 255, 0)
            )
        if charuco_corners is not None and len(charuco_corners) > 0:
            aruco.drawDetectedCornersCharuco(
                bgr, charuco_corners, charuco_ids, cornerColor=(0, 255, 0)
            )
        if ok:
            # Koordinatenachsen der geschätzten Board-Pose einzeichnen (Achsenlänge
            # = doppelte Feldkantenlänge), zur Sichtkontrolle der Orientierung.
            axis_len = float(self.get_parameter('square_length_m').value) * 2.0
            cv2.drawFrameAxes(bgr, self._K, self._D, rvec, tvec, axis_len, 3)

        # Statusbanner — grün wenn bereit, gelb bei Teil-Detektion, sonst rot.
        ready = ok
        partial = (not ok) and (charuco_corners is not None) and len(charuco_corners) > 0
        color = (0, 200, 0) if ready else ((0, 200, 200) if partial else (0, 0, 220))
        h, w = bgr.shape[:2]
        cv2.rectangle(bgr, (0, 0), (w, 36), color, thickness=-1)
        cv2.putText(
            bgr, status, (10, 25), cv2.FONT_HERSHEY_SIMPLEX,
            0.7, (255, 255, 255), 2, cv2.LINE_AA,
        )

        # Annotiertes Debug-Bild und Anzahl erkannter Ecken publizieren.
        self._annotated_pub.publish(bgr_to_image_msg(bgr, msg.header))

        corners_msg = Int32()
        corners_msg.data = n_corners if self._K is not None else 0
        self._corners_pub.publish(corners_msg)

        if ok:
            # Erfolgreiche Pose in Translation+Quaternion umrechnen, als letzte
            # gültige Pose (HOLD) merken, sofort broadcasten und mitzählen.
            self._last_tf = rvec_tvec_to_quat_xyz(
                rvec.flatten(), tvec.flatten()
            )
            self._last_detect_t = time.monotonic()
            self._broadcast_last()
            self._detections += 1

        self._last_pub = now

    def _broadcast_last(self) -> None:
        """Publiziert die gespeicherte Board-Pose als TransformStamped (TF).

        Der Zeitstempel wird bewusst mit der FRISCHEN Node-Clock gesetzt und NICHT
        aus dem Bild-Header übernommen: die RealSense-Hardware-Uhr driftet auf dem
        Jetson gegenüber der Systemuhr. handeye_server schlägt bei now()-200ms nach,
        daher muss im selben Uhren-Domain gestempelt werden, sonst schlägt der
        TF-Lookup fehl. Baut die Transformation camera_frame -> board_frame auf.
        """
        if self._last_tf is None:
            return
        tx, ty, tz, qx, qy, qz, qw = self._last_tf
        t = TransformStamped()
        t.header.stamp = self.get_clock().now().to_msg()
        t.header.frame_id = self._camera_frame
        t.child_frame_id = self._board_frame
        t.transform.translation.x = tx
        t.transform.translation.y = ty
        t.transform.translation.z = tz
        t.transform.rotation.x = qx
        t.transform.rotation.y = qy
        t.transform.rotation.z = qz
        t.transform.rotation.w = qw
        self._tf_pub.sendTransform(t)

    def _on_hold_tick(self) -> None:
        """HOLD-Timer: hält die camera->board-TF zwischen seltenen Detektionen frisch.

        Republiziert die letzte gültige Pose mit neuem Zeitstempel, damit der
        "Take Sample"-Button in easy_handeye2 stabil bleibt, solange das Board
        stillgehalten wird. Sicherheitsabbruch: liegt die letzte Detektion länger
        als hold_timeout_s zurück (Board evtl. aus dem Bild), wird NICHT weiter
        republiziert — so kann keine veraltete Pose fälschlich gesampelt werden.
        """
        if self._last_tf is None:
            return
        if (time.monotonic() - self._last_detect_t) > self._hold_timeout:
            return
        self._broadcast_last()

    def _log_stats(self) -> None:
        """Periodische Diagnose-Ausgabe: gesehene Frames, Detektionen, K geladen?"""
        self.get_logger().info(
            f'frames={self._frames_seen}  detections={self._detections}  '
            f'K_loaded={self._K is not None}'
        )


def main() -> None:
    """Einstiegspunkt: rclpy initialisieren, Node erzeugen und spinnen lassen.

    Bei Ctrl-C (KeyboardInterrupt) wird sauber heruntergefahren: Node zerstören
    und rclpy beenden, damit keine hängenden Ressourcen zurückbleiben.
    """
    rclpy.init()
    node = CharucoDetector()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
