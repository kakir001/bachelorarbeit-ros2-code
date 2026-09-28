#!/usr/bin/env python3
"""
Kamera-Bild anklicken -> MoveIt Plan -> in RViz sehen -> ausführen.

Pipeline:
  Pixel -> aligned depth -> 3D Kamera-Punkt -> TF robot_base
  -> MoveIt Plan (oranger Ghost in RViz) -> Benutzer-Bestätigung -> ausführen

Verwendung:
  Terminal 1: ~/ros2_ws/start_mycobot.sh
  Terminal 2: python3 ~/ros2_ws/src/mycobot_calibration/scripts/click_to_go.py

Mouse:
  Links klicken  — Zielpunkt wählen (gelber Marker)
  Rechts klicken — löschen
Keys:
  P  — Plan! Mit MoveIt Weg planen (in RViz sichtbar)
  E  — ausführen! Plan ausführen (Roboter bewegt sich)
  H  — Home (alle Gelenke 0)
  Q  — beenden
"""

import os
import threading
from typing import Optional, Tuple

import cv2
import numpy as np

import rclpy
from rclpy.node import Node
from rclpy.action import ActionClient
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy

from sensor_msgs.msg import Image, CameraInfo, JointState
from geometry_msgs.msg import PoseStamped, PointStamped
from visualization_msgs.msg import Marker
from control_msgs.action import FollowJointTrajectory
from trajectory_msgs.msg import JointTrajectoryPoint
from builtin_interfaces.msg import Duration
from moveit_msgs.action import MoveGroup
from moveit_msgs.msg import (
    Constraints, JointConstraint, DisplayTrajectory, RobotTrajectory,
)

import tf2_ros
import tf2_geometry_msgs  # noqa: F401

from mycobot_calibration.numerical_ik import NumericalIK

# --- Feste Frame- und Gruppennamen (müssen mit URDF/SRDF/MoveIt-Config übereinstimmen) ---
# CAMERA_FRAME: optischer Rahmen der RealSense-Farbkamera. In diesem Frame liefert die
#   Kamera-Rückprojektion (Pixel -> 3D) ihre Punkte. Achsenkonvention "optical": z nach
#   vorne (in die Szene), x nach rechts, y nach unten.
# ROBOT_BASE_FRAME: Roboter-Basis-Frame. Ziel-Frame nach der TF-Transformation; hier plant
#   MoveIt und hier wird der Reichweiten-Kreis definiert. Die statische eye-to-hand-Kalibrierung
#   liefert die konstante Transformation camera_color_optical_frame -> robot_base.
# TCP_FRAME: Tool Center Point (Werkzeugmittelpunkt) des Greifers.
# PLANNING_GROUP: Name der MoveIt-Planungsgruppe (die 6 Armgelenke), aus der SRDF.
CAMERA_FRAME = 'camera_color_optical_frame'
ROBOT_BASE_FRAME = 'robot_base'
# Schwebe-Aufschlag (m) über dem angeklickten Punkt, per Umgebungsvariable CLICK_HOVER_M
# (z.B. 0.04 = 4 cm). 0.0 = Ziel exakt auf angeklickter Höhe (bisheriges Verhalten).
CLICK_HOVER_M = float(os.environ.get('CLICK_HOVER_M', '0.0'))
TCP_FRAME = 'tcp'
PLANNING_GROUP = 'arm'

# Reihenfolge der 6 Armgelenke exakt wie in der URDF/MoveIt-Config. Diese Reihenfolge
# ist ein Vertrag: JointConstraints und Trajektorien-Punkte werden nach diesen Namen
# indiziert; eine Umsortierung würde die falschen Gelenke ansteuern.
ARM_JOINTS = [
    'joint2_to_joint1',
    'joint3_to_joint2',
    'joint4_to_joint3',
    'joint5_to_joint4',
    'joint6_to_joint5',
    'joint6output_to_joint6',
]
HOME_RADIANS = [0.0, 0.0, 0.0, 0.0, 0.0, 0.0]


def image_msg_to_bgr(msg: Image) -> np.ndarray:
    """Wandelt eine ROS sensor_msgs/Image (Farbe) in ein OpenCV-BGR-Array um.

    Der rohe Byte-Puffer der Nachricht wird ohne Kopie als uint8 interpretiert und
    in (Höhe, Breite, 3) umgeformt. RealSense kann je nach Konfiguration 'rgb8'
    liefern; da OpenCV intern mit BGR arbeitet, müssen in diesem Fall die Kanäle
    getauscht werden. Bei bereits vorliegendem BGR wird eine echte Kopie
    zurückgegeben, damit späteres Einzeichnen (Marker, Overlay) den zugrunde
    liegenden ROS-Nachrichtenpuffer nicht verändert.
    """
    arr = np.frombuffer(msg.data, dtype=np.uint8)
    enc = msg.encoding.lower()
    if enc == 'rgb8':
        return cv2.cvtColor(
            arr.reshape((msg.height, msg.width, 3)), cv2.COLOR_RGB2BGR
        )
    return arr.reshape((msg.height, msg.width, 3)).copy()


def depth_msg_to_array(msg: Image) -> np.ndarray:
    """Wandelt eine ROS-Tiefenbild-Nachricht in ein 2D-numpy-Array um.

    Zwei bei der RealSense übliche Kodierungen werden unterstützt:
      - '16UC1': Tiefe als vorzeichenlose 16-bit-Ganzzahl in Millimetern.
      - '32FC1': Tiefe als 32-bit-Gleitkomma in Metern.
    Die spätere Rückprojektion normiert beide Fälle auf Meter. Eine unbekannte
    Kodierung wird als Fehler gemeldet, statt stillschweigend falsche Werte zu liefern.
    """
    if msg.encoding == '16UC1':
        return np.frombuffer(msg.data, dtype=np.uint16).reshape(
            (msg.height, msg.width)
        )
    if msg.encoding == '32FC1':
        return np.frombuffer(msg.data, dtype=np.float32).reshape(
            (msg.height, msg.width)
        )
    raise ValueError(f'Unsupported depth encoding: {msg.encoding}')


class ClickToGo(Node):
    """ROS-2-Knoten für den Ablauf Kamera-Klick -> MoveIt-Plan -> Ausführung.

    Aufgabenteilung des Knotens:
      * Abonniert Farbbild, Tiefenbild, CameraInfo und /joint_states.
      * Ein Mausklick im OpenCV-Fenster erzeugt aus Pixel+Tiefe einen 3D-Punkt im
        robot_base-Frame (Rückprojektion + TF).
      * 'P' löst die numerische IK (senkrechter Greifer) und lässt MoveIt einen
        kollisionsfreien Weg planen; das Ergebnis wird als DisplayTrajectory in RViz
        angezeigt (oranger Ghost), aber noch NICHT gefahren.
      * 'E' führt den bestätigten Plan aus, indem das Gelenkziel über die
        Bridge-Kommando-Socket an den echten Roboter gesendet wird.
    Die GUI läuft im Haupt-Thread (OpenCV), rclpy.spin in einem Daemon-Thread;
    daher schützt ein Lock die von beiden Threads berührten Bildpuffer.
    """
    def __init__(self):
        super().__init__('click_to_go')

        # Kamera-Intrinsik (3x3), letzte Bild-/Tiefen-/Gelenkzustände. Werden von
        # den Subscriber-Callbacks (spin-Thread) geschrieben und von der GUI gelesen.
        self._K: Optional[np.ndarray] = None
        self._latest_bgr: Optional[np.ndarray] = None
        self._latest_depth: Optional[np.ndarray] = None
        self._latest_joints: Optional[JointState] = None
        self._lock = threading.Lock()  # schützt _latest_bgr/_latest_depth zwischen spin- und GUI-Thread

        # Zustand des Klick-/Plan-Ablaufs: angeklicktes Pixel, zugehöriger 3D-Zielpunkt
        # im robot_base-Frame und die zuletzt geplante (noch nicht ausgeführte) Trajektorie.
        self._click_px: Optional[Tuple[int, int]] = None
        self._target_3d_base: Optional[np.ndarray] = None
        self._planned_trajectory = None
        self._status = 'Warte auf Kamera...'

        # Reichweiten-Kreis (senkrechter Griff): grün=sicher, amber=kinematisches Limit.
        # in robot_base XY-Ebene, auf Plattformhöhe (reach_z). Quelle: URDF FK.
        # reach_radius: Radius (m), innerhalb dessen der myCobot 280 mit senkrechtem Griff
        #   zuverlässig planen kann. reach_radius_max: äußeres Limit, ab dem die
        #   Kinematik unsicher wird. reach_z: Höhe der XY-Ebene über der Basis.
        self.reach_radius = 0.22
        self.reach_radius_max = 0.30
        self.reach_z = 0.008

        # TF-Puffer + Listener empfangen die statische Kamera->Basis-Transformation
        # (eye-to-hand-Kalibrierung) und stellen sie für Punkt-Transformationen bereit.
        self._tf_buffer = tf2_ros.Buffer()
        self._tf_listener = tf2_ros.TransformListener(self._tf_buffer, self)

        # Numerischer IK-Löser (nutzt den MoveIt-Dienst compute_ik). Liefert für eine
        # kartesische Zielposition eine Gelenk-Lösung mit senkrecht nach unten zeigendem Greifer.
        self._nik = NumericalIK(self)
        self.get_logger().info('Numerical IK Solver geladen (MoveIt compute_ik)')

        # Publisher: Zielpunkt-Marker (RViz-Kugel) und geplante Trajektorie (oranger Ghost).
        self._marker_pub = self.create_publisher(
            Marker, '/click_to_go/target_marker', 10
        )
        self._traj_pub = self.create_publisher(
            DisplayTrajectory, '/display_planned_path', 10
        )

        # Action-Clients: MoveGroup für die Wegplanung, sowie die Controller-Actions
        # für Arm und Greifer (Trajektorien-Ausführung über ros2_control).
        self._move_group_client = ActionClient(
            self, MoveGroup, '/move_action'
        )
        self._arm_client = ActionClient(
            self, FollowJointTrajectory,
            '/arm_controller/follow_joint_trajectory'
        )
        self._gripper_client = ActionClient(
            self, FollowJointTrajectory,
            '/gripper_controller/follow_joint_trajectory'
        )

        # CameraInfo wird mit RELIABLE-QoS abonniert (latched-ähnlich, depth=1): die
        # Intrinsik ändert sich nicht, es genügt der letzte gültige Wert. Bild- und
        # Tiefen-Topics laufen dagegen mit dem Default-Sensor-Datenstrom (Tiefe 10).
        qos_ci = QoSProfile(
            reliability=ReliabilityPolicy.RELIABLE,
            history=HistoryPolicy.KEEP_LAST, depth=1,
        )
        self.create_subscription(
            CameraInfo, '/camera/color/camera_info', self._on_info, qos_ci
        )
        self.create_subscription(
            Image, '/camera/color/image_raw', self._on_color, 10
        )
        self.create_subscription(
            Image, '/camera/aligned_depth_to_color/image_raw',
            self._on_depth, 10
        )
        self.create_subscription(
            JointState, '/joint_states', self._on_joints, 10
        )

    def _on_info(self, msg: CameraInfo):
        # msg.k ist die 3x3-Intrinsik-Matrix (fx, fy, cx, cy) zeilenweise als 9er-Vektor.
        self._K = np.array(msg.k, dtype=np.float64).reshape(3, 3)

    def _on_color(self, msg: Image):
        # Callback im spin-Thread: neuestes Farbbild puffern; beim ersten Bild wird der
        # Startstatus auf "Bereit" gesetzt. Fehler werden verschluckt, damit ein einzelner
        # defekter Frame den Knoten nicht abbricht.
        try:
            with self._lock:
                self._latest_bgr = image_msg_to_bgr(msg)
                if self._status == 'Warte auf Kamera...':
                    self._status = 'Bereit — klicken!'
        except Exception:
            pass

    def _on_depth(self, msg: Image):
        try:
            with self._lock:
                self._latest_depth = depth_msg_to_array(msg)
        except Exception:
            pass

    def _on_joints(self, msg: JointState):
        # Aktuelle Gelenkstellungen; dienen später als IK-Seed (Startschätzung).
        self._latest_joints = msg

    def pixel_to_3d_base(self, u: int, v: int) -> Optional[np.ndarray]:
        """Rückprojektion eines angeklickten Pixels (u, v) in einen 3D-Punkt im robot_base-Frame.

        Ablauf:
          1. Tiefe am Klickpunkt aus dem (auf Farbe ausgerichteten) Tiefenbild lesen.
             Statt eines einzelnen Pixels wird der Median eines 7x7-Fensters genommen,
             um Rauschen und Tiefen-Löcher (Wert 0) robust zu überbrücken.
          2. Millimeter->Meter normieren (16UC1 liefert mm, 32FC1 bereits m).
          3. Mit dem Lochkamera-Modell (fx, fy, cx, cy) das Pixel in einen 3D-Punkt
             im Kamera-optical-Frame zurückprojizieren:  x=(u-cx)*z/fx, y=(v-cy)*z/fy.
          4. Diesen Punkt per TF in den robot_base-Frame transformieren.
        Rückgabe None, sobald Intrinsik/Tiefe fehlen, der Klick außerhalb des Bildes
        liegt oder das Fenster keine gültige Tiefe enthält.
        """
        if self._K is None:
            return None
        with self._lock:
            depth = self._latest_depth
        if depth is None:
            return None

        h, w = depth.shape
        if not (0 <= v < h and 0 <= u < w):
            return None

        # 7x7-Nachbarschaft um den Klick; nur Werte > 0 sind gültige Tiefen.
        region = depth[max(0, v - 3):v + 4, max(0, u - 3):u + 4]
        valid = region[region > 0]
        if len(valid) == 0:
            return None

        # Median unterdrückt Ausreißer; >100 => Wert ist in Millimetern (16UC1) -> in Meter.
        z_raw = float(np.median(valid))
        z = z_raw / 1000.0 if z_raw > 100 else z_raw

        # Inverse Lochkamera-Projektion in den Kamera-optical-Frame.
        fx, fy = self._K[0, 0], self._K[1, 1]
        cx, cy = self._K[0, 2], self._K[1, 2]
        x_cam = (u - cx) * z / fx
        y_cam = (v - cy) * z / fy

        pt_cam = PointStamped()
        pt_cam.header.frame_id = CAMERA_FRAME
        pt_cam.header.stamp = self.get_clock().now().to_msg()
        pt_cam.point.x = x_cam
        pt_cam.point.y = y_cam
        pt_cam.point.z = z

        # TF: Kamera-optical -> robot_base. Timeout gibt dem Buffer Zeit, die (statische)
        # Kalibrier-Transformation zu erhalten, falls sie noch nicht vorliegt.
        try:
            pt_base = self._tf_buffer.transform(
                pt_cam, ROBOT_BASE_FRAME,
                timeout=rclpy.time.Duration(seconds=2.0)
            )
        except Exception as e:
            self.get_logger().warn(f'TF-Transformation fehlgeschlagen: {e}')
            return None

        # Untergrenze für z: verhindert Ziele unter/auf der Tischplatte (Tiefenrauschen);
        # der Greifer soll nicht in die Plattform hineinfahren.
        z = pt_base.point.z
        if z < 0.005:
            z = 0.005
        # Optionaler Schwebe-Aufschlag (CLICK_HOVER_M, m): Ziel N cm ÜBER dem angeklickten
        # Punkt — für Kalibrier-Messungen (Feinausrichtung danach per cartesian_jog),
        # damit der Greifer bei XY-Abweichung nicht seitlich gegen die Welle fährt.
        # Default 0.0 = bisheriges Verhalten unverändert.
        z += CLICK_HOVER_M
        return np.array([
            pt_base.point.x, pt_base.point.y, z
        ], dtype=np.float64)

    # ---- Reichweiten-Kreis (base XY-Ebene -> Pixel-Projektion) ----
    # Diese drei Methoden projizieren den (im robot_base definierten) Arbeitsraum-Kreis
    # zurück ins Kamerabild, damit der Benutzer VOR dem Klick sieht, welche Punkte der
    # Roboter mit senkrechtem Griff sicher (grün) bzw. am kinematischen Limit (amber)
    # erreicht. Es ist die inverse Richtung zu pixel_to_3d_base: base-Punkt -> Pixel.
    def _base_to_cam_RT(self):
        """robot_base -> camera_optical (R 3x3, t 3). EIN TF-Lookup. Kann None sein.

        Holt die Transformation als Quaternion+Translation und baut daraus die
        Rotationsmatrix R und den Translationsvektor t, sodass ein base-Punkt p_base
        via  p_cam = R @ p_base + t  in den Kamera-optical-Frame überführt wird.
        Ein einziger Lookup wird pro Frame gemacht und an alle Projektionen weitergereicht.
        """
        if self._K is None:
            return None
        try:
            tf = self._tf_buffer.lookup_transform(
                CAMERA_FRAME, ROBOT_BASE_FRAME, rclpy.time.Time())
        except Exception:
            return None
        # Quaternion (x,y,z,w) -> Rotationsmatrix (Standard-Formel), t = Translation.
        q = tf.transform.rotation
        t = tf.transform.translation
        x, y, z, w = q.x, q.y, q.z, q.w
        R = np.array([
            [1 - 2 * (y * y + z * z), 2 * (x * y - z * w),     2 * (x * z + y * w)],
            [2 * (x * y + z * w),     1 - 2 * (x * x + z * z), 2 * (y * z - x * w)],
            [2 * (x * z - y * w),     2 * (y * z + x * w),     1 - 2 * (x * x + y * y)],
        ])
        return R, np.array([t.x, t.y, t.z])

    def _proj(self, pb, R, tv):
        """base Punkt (3,) -> Pixel (u,v). Hinter der Kamera -> None.

        Transformiert den Punkt in den Kamera-Frame und wendet dann das Lochkamera-Modell
        an (u=fx*x/z+cx, v=fy*y/z+cy). Punkte mit z<=0 liegen hinter/auf der Bildebene und
        können nicht projiziert werden -> None (verhindert Division durch ~0 und Spiegelartefakte).
        """
        pc = R @ pb + tv
        if pc[2] <= 1e-3:
            return None
        fx, fy = self._K[0, 0], self._K[1, 1]
        cx, cy = self._K[0, 2], self._K[1, 2]
        return (int(round(fx * pc[0] / pc[2] + cx)),
                int(round(fy * pc[1] / pc[2] + cy)))

    def _draw_reach(self, frame):
        """Reichweiten-Kreise (grün sicher + amber Limit) + Roboter-Zentrum zeichnen.

        Für jeden Radius werden 72 Punkte auf einem Kreis in der robot_base-XY-Ebene
        (auf Plattformhöhe reach_z) erzeugt, einzeln ins Bild projiziert und als
        Polygonzug gezeichnet. Der Kreis wird nur dann geschlossen gezeichnet, wenn ALLE
        Punkte sichtbar sind; ragt er teilweise aus dem Bild/hinter die Kamera, bleibt er
        offen. So bleibt die Anzeige auch bei starker Perspektive korrekt.
        """
        rt = self._base_to_cam_RT()
        if rt is None:
            return
        R, tv = rt
        AMBER, GREEN = (0, 200, 255), (0, 200, 0)
        # Zuerst den äußeren (amber) Limit-Kreis, dann den inneren (grünen) Sicherheitskreis.
        for radius, col, th in ((self.reach_radius_max, AMBER, 1),
                                (self.reach_radius, GREEN, 2)):
            if radius <= 0:
                continue
            pts = [self._proj(np.array([radius * np.cos(a), radius * np.sin(a), self.reach_z]),
                              R, tv)
                   for a in (2 * np.pi * i / 72 for i in range(72))]
            valid = [p for p in pts if p is not None]
            if len(valid) >= 2:
                closed = all(p is not None for p in pts)
                cv2.polylines(frame, [np.array(valid, np.int32)], closed,
                              col, th, cv2.LINE_AA)
        # Mittelpunkt (Roboter-Basis) als Kreuz + Legende mit den Radien in mm markieren.
        c = self._proj(np.array([0.0, 0.0, self.reach_z]), R, tv)
        if c is not None:
            cv2.drawMarker(frame, c, AMBER, cv2.MARKER_TILTED_CROSS, 16, 2)
            cv2.putText(frame, f'ROBOTER  g<{self.reach_radius * 1000:.0f} lim<{self.reach_radius_max * 1000:.0f}mm',
                        (c[0] + 8, c[1] + 4), cv2.FONT_HERSHEY_SIMPLEX,
                        0.4, AMBER, 1, cv2.LINE_AA)

    def on_click(self, u: int, v: int):
        """Linksklick-Handler: Pixel in 3D-Zielpunkt umrechnen und als gelben Marker anzeigen.

        Schlägt die Rückprojektion fehl (keine gültige Tiefe), wird der Klick nur als
        Kreuz gemerkt und eine Fehlermeldung gesetzt; ein vorheriges Ziel/Plan wird verworfen.
        Bei Erfolg wird der Zielpunkt (robot_base) gespeichert und der Status auf 'P: planen'
        gesetzt.
        """
        p_base = self.pixel_to_3d_base(u, v)
        if p_base is None:
            self._click_px = (u, v)
            self._target_3d_base = None
            self._planned_trajectory = None
            self._status = f'Keine Tiefe ({u},{v})'
            self.get_logger().warn(self._status)
            return

        self._click_px = (u, v)
        self._target_3d_base = p_base
        self._planned_trajectory = None
        self._status = (
            f'Ziel: [{p_base[0]:.3f}, {p_base[1]:.3f}, {p_base[2]:.3f}] '
            f'— P: planen'
        )
        self.get_logger().info(self._status)
        self._publish_marker(p_base, r=1.0, g=1.0, b=0.0)

    def _publish_marker(self, p_base: np.ndarray, r=1.0, g=1.0, b=0.0):
        # Veröffentlicht eine RViz-Kugel am Zielpunkt. Farbcodierung des Ablaufs:
        # gelb (r,g)=noch nicht geplant, grün=Plan bereit, rot=IK/Plan fehlgeschlagen.
        m = Marker()
        m.header.frame_id = ROBOT_BASE_FRAME
        m.header.stamp = self.get_clock().now().to_msg()
        m.ns = 'click_to_go'
        m.id = 0
        m.type = Marker.SPHERE
        m.action = Marker.ADD
        m.pose.position.x = float(p_base[0])
        m.pose.position.y = float(p_base[1])
        m.pose.position.z = float(p_base[2])
        m.pose.orientation.w = 1.0
        m.scale.x = 0.03
        m.scale.y = 0.03
        m.scale.z = 0.03
        m.color.r = r
        m.color.g = g
        m.color.b = b
        m.color.a = 0.9
        m.lifetime.sec = 60
        self._marker_pub.publish(m)

    def _delete_marker(self):
        # Entfernt den Zielpunkt-Marker in RViz (bei Rechtsklick/Löschen).
        m = Marker()
        m.header.frame_id = ROBOT_BASE_FRAME
        m.header.stamp = self.get_clock().now().to_msg()
        m.ns = 'click_to_go'
        m.id = 0
        m.action = Marker.DELETE
        self._marker_pub.publish(m)

    def plan(self):
        """'P'-Taste: Planung anstoßen (im Hintergrund-Thread, damit die GUI nicht blockiert).

        Es muss ein Ziel gewählt sein und es darf nicht bereits eine Planung laufen
        (Wiedereintritts-Schutz über _planning).
        """
        if self._target_3d_base is None:
            self._status = 'Erst einen Punkt waehlen!'
            return
        if hasattr(self, '_planning') and self._planning:
            self._status = 'Wird bereits geplant...'
            return
        self._planning = True
        threading.Thread(target=self._plan_thread, daemon=True).start()

    def _plan_thread(self):
        """Hintergrund-Thread: IK lösen und daraus einen MoveIt-Plan als Joint-Goal anfordern.

        Schritt 1 (hier): kartesisches Ziel -> Gelenkwinkel via numerischer IK
        (senkrechter Greifer). Schritt 2: die IK-Lösung als JointConstraints an MoveGroup
        senden -> MoveIt plant den kollisionsfreien Weg. Die Antwort kommt asynchron über
        add_done_callback (_plan_response_cb / _plan_result_cb).
        """
        tx = float(self._target_3d_base[0])
        ty = float(self._target_3d_base[1])
        tz = float(self._target_3d_base[2])
        if tz < 0.005:
            tz = 0.005

        self._status = f'IK + Plan [{tx:.3f},{ty:.3f},{tz:.3f}]...'
        self.get_logger().info(self._status)

        # Aktuelle Gelenk-Positionen als Seed verwenden: die IK startet nahe der Ist-Lage,
        # was Konvergenz beschleunigt und unnötig große Gelenk-Sprünge vermeidet.
        # Namen->Position-Abbildung, da /joint_states nicht zwingend in ARM_JOINTS-Reihenfolge kommt.
        current = None
        if self._latest_joints is not None:
            name_to_pos = dict(zip(
                self._latest_joints.name, self._latest_joints.position
            ))
            current = [name_to_pos.get(j, 0.0) for j in ARM_JOINTS]

        # Multi-Seed-IK: probiert mehrere Startschätzungen, um lokale Minima zu umgehen.
        # gripper_down=True erzwingt eine senkrecht nach unten zeigende Werkzeug-Orientierung.
        target_radians = self._nik.solve_multi_seed(
            [tx, ty, tz], gripper_down=True, current_joints=current, n_seeds=5
        )

        if target_radians is None:
            self._status = f'IK FEHLGESCHLAGEN — Punkt nicht erreichbar'
            self._planning = False
            self._publish_marker(self._target_3d_base, r=1.0, g=0.0, b=0.0)
            return

        self.get_logger().info(
            f'IK OK: joints={[round(r,3) for r in target_radians]}'
        )

        # 2) An MoveGroup als Joint-Goal übergeben — Wegplanung + Kollisions-Check
        if not self._move_group_client.wait_for_server(timeout_sec=3.0):
            self._status = 'MoveGroup nicht gefunden!'
            return

        # MoveGroup-Ziel zusammenstellen. num_planning_attempts+allowed_planning_time geben
        # dem Planer mehrere Versuche/Zeit; velocity/acceleration_scaling begrenzen die
        # Ausführungsgeschwindigkeit (0.3/0.2 = sicher/langsam für den echten Roboter).
        goal = MoveGroup.Goal()
        req = goal.request
        req.group_name = PLANNING_GROUP
        req.num_planning_attempts = 10
        req.allowed_planning_time = 5.0
        req.max_velocity_scaling_factor = 0.3
        req.max_acceleration_scaling_factor = 0.2

        # Arbeitsraum-Grenzen (+/-1 m um die Basis): Suchraum-Begrenzung für den Planer.
        req.workspace_parameters.header.frame_id = ROBOT_BASE_FRAME
        req.workspace_parameters.min_corner.x = -1.0
        req.workspace_parameters.min_corner.y = -1.0
        req.workspace_parameters.min_corner.z = -1.0
        req.workspace_parameters.max_corner.x = 1.0
        req.workspace_parameters.max_corner.y = 1.0
        req.workspace_parameters.max_corner.z = 1.0

        # Das IK-Ergebnis wird NICHT als kartesisches Pose-Ziel, sondern als Gelenk-Ziel
        # (JointConstraint pro Achse) übergeben. So plant MoveIt exakt zu der von uns
        # gewählten IK-Lösung (senkrechter Griff) und wählt nicht selbst eine andere.
        constraints = Constraints()
        for jname, jval in zip(ARM_JOINTS, target_radians):
            jc = JointConstraint()
            jc.joint_name = jname
            jc.position = jval
            jc.tolerance_above = 0.05
            jc.tolerance_below = 0.05
            jc.weight = 1.0
            constraints.joint_constraints.append(jc)
        req.goal_constraints.append(constraints)

        # plan_only=True: nur planen, NICHT ausführen (Vorschau in RViz, Bestätigung über 'E').
        goal.planning_options.plan_only = True
        goal.planning_options.look_around = False
        goal.planning_options.replan = False

        # Asynchron senden; die Ergebnis-Callbacks laufen im spin-Thread.
        self._status = 'MoveIt plant Weg...'
        future = self._move_group_client.send_goal_async(goal)
        future.add_done_callback(self._plan_response_cb)

    def _plan_response_cb(self, future):
        # Erste Antwort von MoveGroup: wurde das Planungsziel überhaupt angenommen?
        # Bei Ablehnung Ziel-Marker rot färben; sonst auf das eigentliche Ergebnis warten.
        goal_handle = future.result()
        if not goal_handle.accepted:
            self._status = 'Plan ABGELEHNT'
            if self._target_3d_base is not None:
                self._publish_marker(self._target_3d_base, r=1.0, g=0.0, b=0.0)
            return
        result_future = goal_handle.get_result_async()
        result_future.add_done_callback(self._plan_result_cb)

    def _plan_result_cb(self, future):
        # Planungsergebnis auswerten. error_code.val==1 (SUCCESS) => Trajektorie liegt vor.
        result = future.result().result
        error_code = result.error_code.val

        if error_code != 1:
            self._planning = False
            self._status = f'Plan FEHLGESCHLAGEN (code={error_code})'
            self.get_logger().error(self._status)
            self._planned_trajectory = None
            if self._target_3d_base is not None:
                self._publish_marker(self._target_3d_base, r=1.0, g=0.0, b=0.0)
            return

        # Letzter Trajektorien-Punkt = Ziel-Gelenkstellung; die wird bei 'E' angefahren.
        # Vollständige Trajektorie für die RViz-Vorschau merken.
        traj = result.planned_trajectory.joint_trajectory
        self._planned_joints = list(traj.points[-1].positions)
        self._planned_trajectory = result.planned_trajectory

        # DisplayTrajectory veröffentlichen -> RViz zeigt den geplanten Weg als orangen Ghost.
        display = DisplayTrajectory()
        display.trajectory_start = result.trajectory_start
        display.trajectory.append(result.planned_trajectory)
        self._traj_pub.publish(display)

        n_points = len(traj.points)
        self._planning = False
        self._status = (
            f'Plan BEREIT ({n_points} Punkte) — '
            f'in RViz sehen, E: ausfuehren'
        )
        self.get_logger().info(self._status)
        if self._target_3d_base is not None:
            self._publish_marker(self._target_3d_base, r=0.0, g=1.0, b=0.0)

    def execute(self):
        """'E'-Taste: den bestätigten Plan ausführen.

        Statt die volle MoveIt-Trajektorie über den Controller zu spielen, wird hier das
        Ziel-Gelenktupel direkt über die Bridge-Socket an den echten Roboter gesendet
        (flüssige send_radians-Bewegung, siehe _send_bridge_cmd). Der Plan wird verworfen,
        damit ein versehentliches zweites 'E' nicht erneut fährt.
        """
        if self._planned_trajectory is None or not hasattr(self, '_planned_joints'):
            self._status = 'Erst mit P planen!'
            return

        target = self._planned_joints
        self._status = 'Roboter faehrt zum Ziel...'
        self.get_logger().info(
            f'Execute: Ziel={[round(p,3) for p in target]}'
        )
        self._planned_trajectory = None
        self._send_bridge_cmd(target, speed=20)

    def _send_bridge_cmd(self, radians: list, speed: int = 20):
        """Ein send_radians an Bridge-Kommando-Socket senden — flüssige Bewegung.

        Der myCobot-Bridge-Prozess hält den seriellen Port offen und lauscht auf einem
        Unix-Domain-Socket. Wir schicken eine Textzeile im Wire-Protokoll
        'send_radians r1 r2 ... r6 speed'. Dieser Umweg (statt pymycobot direkt) vermeidet
        Port-Konkurrenz: nur die Bridge spricht mit der Hardware. Das Zeilenformat und der
        Socket-Pfad sind ein Laufzeit-Vertrag mit der Bridge und dürfen nicht geändert werden.
        """
        import socket as sock
        CMD_PATH = '/tmp/mycobot_bridge.sock.cmd'
        cmd = 'send_radians ' + ' '.join(f'{r:.6f}' for r in radians) + f' {speed}\n'
        try:
            s = sock.socket(sock.AF_UNIX, sock.SOCK_STREAM)
            s.connect(CMD_PATH)
            s.sendall(cmd.encode())
            s.close()
            self._status = 'Faehrt zum Ziel...'
            self.get_logger().info('Bridge cmd sent OK')
        except Exception as e:
            self._status = f'Bridge-Fehler: {e}'
            self.get_logger().error(self._status)

    def go_named_pose(self, pose_name: str):
        """Direkt über Bridge zu benannter Pose fahren — flüssige Bewegung.

        Vordefinierte Gelenkkonfigurationen ohne Planung: 'home' (Null-Stellung) und
        'ready' (Arbeitsvorbereitungspose). Wird über dieselbe Bridge-Socket wie execute()
        gesendet.
        """
        NAMED_POSES = {
            'home': [0.0, 0.0, 0.0, 0.0, 0.0, 0.0],
            'ready': [0.0, -0.5, -1.0, 0.0, 0.0, 0.0],
        }
        if pose_name not in NAMED_POSES:
            self._status = f'Unbekannte Pose: {pose_name}'
            return

        self._status = f'Faehrt zu Position {pose_name}...'
        self._planned_trajectory = None
        self._send_bridge_cmd(NAMED_POSES[pose_name], speed=15)

    def gripper_control(self, state: str):
        """Greifer öffnen/schließen über die gripper_controller-Action.

        Anders als die Armbewegung läuft der Greifer hier über ros2_control (Action
        FollowJointTrajectory) statt über die Bridge. Die Positionen (rad) sind
        firmware-/URDF-kalibrierte Anschlagwerte für 'open'/'closed'.
        """
        GRIPPER_POSES = {'open': 0.15, 'closed': -0.74}
        if state not in GRIPPER_POSES:
            return

        self._status = f'Greifer {state}...'
        if not self._gripper_client.wait_for_server(timeout_sec=3.0):
            self._status = 'Greifer Action-Server nicht gefunden!'
            return

        goal = FollowJointTrajectory.Goal()
        goal.trajectory.joint_names = ['gripper_controller']
        pt = JointTrajectoryPoint()
        pt.positions = [GRIPPER_POSES[state]]
        pt.time_from_start = Duration(sec=2, nanosec=0)
        goal.trajectory.points = [pt]

        future = self._gripper_client.send_goal_async(goal)
        future.add_done_callback(lambda f: self._gripper_cb(f, state))

    def _gripper_cb(self, future, state):
        goal_handle = future.result()
        if not goal_handle.accepted:
            self._status = f'Greifer {state} ABGELEHNT'
            return
        result_future = goal_handle.get_result_async()
        result_future.add_done_callback(
            lambda f: setattr(self, '_status', f'Greifer {state} OK')
        )

    def get_display_frame(self) -> Optional[np.ndarray]:
        """Baut das Anzeige-Bild für das OpenCV-Fenster (wird im Haupt-Thread aufgerufen).

        Zeichnet in Reihenfolge: Reichweiten-Kreise, Klick-Marker (grün=geplant,
        gelb=nur gewählt) mit 3D-Koordinaten und Reichweiten-Tag (ERREICHBAR/GRENZE/
        UNERREICHBAR), Status-Zeile oben und Tastenlegende unten. Der Bildpuffer wird unter
        Lock kopiert, damit der spin-Thread ihn nicht während des Zeichnens überschreibt.
        """
        with self._lock:
            if self._latest_bgr is None:
                return None
            frame = self._latest_bgr.copy()

        h, w = frame.shape[:2]

        # Reichweiten-Kreise (Bereich, den der Roboter mit senkrechtem Griff erreicht)
        try:
            self._draw_reach(frame)
        except Exception:
            pass

        if self._click_px is not None:
            u, v = self._click_px
            color = (0, 255, 0) if self._planned_trajectory else (0, 255, 255)
            cv2.circle(frame, (u, v), 10, color, 2)
            cv2.drawMarker(frame, (u, v), color,
                           cv2.MARKER_CROSS, 20, 2)

            if self._target_3d_base is not None:
                # r = horizontaler Abstand des Ziels von der Basisachse -> Reichweiten-Kategorie.
                p = self._target_3d_base
                r = float(np.hypot(p[0], p[1]))
                if r <= self.reach_radius:
                    rcol, rtag = (0, 200, 0), 'ERREICHBAR'
                elif r <= self.reach_radius_max:
                    rcol, rtag = (0, 200, 255), 'GRENZE'
                else:
                    rcol, rtag = (0, 0, 255), 'UNERREICHBAR'
                text = f'[{p[0]:.3f}, {p[1]:.3f}, {p[2]:.3f}]  r={r*1000:.0f}mm {rtag}'
                cv2.putText(frame, text, (u + 15, v - 15),
                            cv2.FONT_HERSHEY_SIMPLEX, 0.5,
                            rcol, 1, cv2.LINE_AA)

        if self._status:
            cv2.rectangle(frame, (0, 0), (w, 30), (40, 40, 40), -1)
            cv2.putText(frame, self._status, (8, 22),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.45,
                        (255, 255, 255), 1, cv2.LINE_AA)

        cv2.rectangle(frame, (0, h - 24), (w, h), (40, 40, 40), -1)
        cv2.putText(frame, 'Klick:waehlen P:Plan E:exe H:Home R:ready O:auf C:zu Q:beenden',
                    (4, h - 6), cv2.FONT_HERSHEY_SIMPLEX, 0.3,
                    (180, 180, 180), 1, cv2.LINE_AA)

        return frame


def mouse_cb(event, x, y, flags, param):
    # OpenCV-Maus-Callback: Links = Ziel wählen, Rechts = Ziel/Plan/Marker löschen.
    node = param
    if event == cv2.EVENT_LBUTTONDOWN:
        node.on_click(x, y)
    elif event == cv2.EVENT_RBUTTONDOWN:
        node._click_px = None
        node._target_3d_base = None
        node._planned_trajectory = None
        node._status = 'Bereit — klicken!'
        node._delete_marker()


def main():
    # ROS initialisieren, Knoten anlegen und rclpy.spin in einen Daemon-Thread auslagern,
    # damit der Haupt-Thread die (blockierende) OpenCV-GUI-Schleife fahren kann.
    rclpy.init()
    node = ClickToGo()

    spin_thread = threading.Thread(
        target=lambda: rclpy.spin(node), daemon=True
    )
    spin_thread.start()

    cv2.namedWindow('Click to Go', cv2.WINDOW_NORMAL)
    cv2.resizeWindow('Click to Go', 640, 380)
    cv2.setMouseCallback('Click to Go', mouse_cb, node)

    print('\n' + '=' * 55)
    print('  CLICK TO GO — Plan & Execute')
    print('=' * 55)
    print('  Links klicken : Ziel waehlen (gelber Marker)')
    print('  Rechts klicken: loeschen')
    print('  P             : Mit MoveIt planen (in RViz sehen)')
    print('  E             : Plan ausfuehren (Roboter faehrt)')
    print('  H             : Home (alle Gelenke 0)')
    print('  R             : ready (Arbeitsposition)')
    print('  O             : Greifer oeffnen')
    print('  C             : Greifer schliessen')
    print('  Q             : beenden')
    print('=' * 55 + '\n')

    # GUI-Hauptschleife: Bild anzeigen und Tastendrücke auf Knoten-Aktionen abbilden.
    # cv2.waitKey(50) taktet die Schleife auf ~20 Hz und liefert den gedrückten Tastencode.
    try:
        while rclpy.ok():
            frame = node.get_display_frame()
            if frame is not None:
                cv2.imshow('Click to Go', frame)

            key = cv2.waitKey(50) & 0xFF
            if key != 255:
                node.get_logger().info(f'Key pressed: {key} = {chr(key) if 32 <= key < 127 else "?"}')
            if key == ord('p') or key == ord('P'):
                node.plan()
            elif key == ord('e') or key == ord('E'):
                node.execute()
            elif key == ord('h') or key == ord('H'):
                node.go_named_pose('home')
            elif key == ord('r') or key == ord('R'):
                node.go_named_pose('ready')
            elif key == ord('o') or key == ord('O'):
                node.gripper_control('open')
            elif key == ord('c') or key == ord('C'):
                node.gripper_control('closed')
            elif key == ord('q') or key == ord('Q'):
                break

    except KeyboardInterrupt:
        pass
    finally:
        # Sauberes Herunterfahren: Fenster schließen, Knoten zerstören, ROS beenden.
        cv2.destroyAllWindows()
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
