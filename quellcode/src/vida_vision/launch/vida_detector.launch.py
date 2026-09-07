"""Startet den vida_detector Node (Phase 2 — nur Erkennung + Overlay).

HINWEIS: Die D435i Topics müssen aktiv sein. Starte den Haupt-Stack mit Kamera:
  ros2 launch mycobot_moveit_config demo.launch.py use_camera:=true
Danach diesen Launch in einem separaten Terminal ausführen.

Alle operativen Gate-/Schwellenwert-Parameter können überschrieben werden, z.B.:
  ros2 launch vida_vision vida_detector.launch.py conf:=0.6 mode_consensus:=5
"""
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    return LaunchDescription([
        # Alle folgenden DeclareLaunchArgument definieren überschreibbare Parameter mit
        # Standardwerten; sie werden weiter unten 1:1 an den vida_detector-Node durchgereicht.

        # image_topic: Eingangs-Farbbild vom RealSense-Treiber (Quelle der YOLO-Erkennung).
        DeclareLaunchArgument("image_topic", default_value="/camera/color/image_raw"),
        # conf: minimale YOLO-Konfidenz, damit eine Detektion überhaupt gilt. 0.75 ist ein
        # konservativer Standard gegen Fehldetektionen; für schwierige Szenen absenkbar.
        DeclareLaunchArgument("conf", default_value="0.75"),
        # rate_hz: Verarbeitungsfrequenz. 1 Hz genügt (Schrauben bewegen sich nicht) und
        # entlastet den Jetson Nano deutlich (YOLO11-seg ist rechenintensiv).
        DeclareLaunchArgument("rate_hz", default_value="1.0"),
        # enable_3d: bei true wird aus der Tiefe zusätzlich die 3D-Position/Greifachse
        # geschätzt (benötigt align_depth des Kameratreibers).
        DeclareLaunchArgument("enable_3d", default_value="true"),
        # depth_mad_k: Ausreißer-Filter für die Tiefenwerte im Segment (k*MAD um den Median).
        DeclareLaunchArgument("depth_mad_k", default_value="2.5"),
        # axis_min_linearity: Mindest-Linearität (0..1) der Segmentmaske, damit ihre
        # Haupt-Trägheitsachse als verlässliche Schraubenachse gilt.
        DeclareLaunchArgument("axis_min_linearity", default_value="0.6"),
        # grasp_tilt_min_deg: ab dieser Neigung (Grad) gegen die Senkrechte wird ein
        # gekippter Greifansatz statt eines reinen Top-Down-Griffs gewählt.
        DeclareLaunchArgument("grasp_tilt_min_deg", default_value="15.0"),
        # target_z_min/target_z_max: gültiger Höhenbereich (m) des Greifziels im Basis-Frame;
        # Detektionen außerhalb (Boden/zu hoch) werden verworfen — Plausibilitäts-Gate.
        DeclareLaunchArgument("target_z_min", default_value="-0.02"),
        DeclareLaunchArgument("target_z_max", default_value="0.12"),
        # mode_consensus: so viele aufeinanderfolgende Frames müssen dasselbe Ziel bestätigen,
        # bevor es als stabil ausgegeben wird (unterdrückt Flackern/Einzelaussetzer).
        DeclareLaunchArgument("mode_consensus", default_value="3"),
        # sticky_radius: Radius (m), innerhalb dessen das bisherige Ziel beibehalten wird, statt
        # bei geringfügiger Verschiebung auf eine andere Schraube umzuspringen (Hysterese).
        DeclareLaunchArgument("sticky_radius", default_value="0.05"),
        # Zielauswahl: am zentralsten + am besten sichtbar, harte Eck-Eliminierung
        # center_weight/conf_weight: Gewichtung von Bildzentrums-Nähe vs. Konfidenz im Score.
        DeclareLaunchArgument("center_weight", default_value="1.0"),
        DeclareLaunchArgument("conf_weight", default_value="1.0"),
        # corner_edge_frac: Randstreifen-Breite (Bruchteil der Bildkante), der als "Ecke" gilt.
        DeclareLaunchArgument("corner_edge_frac", default_value="0.15"),
        # corner_min_screws: erst ab so vielen Schrauben werden randständige/eckige Ziele hart
        # eliminiert (bei wenigen Schrauben ist auch eine Eck-Schraube noch akzeptabel).
        DeclareLaunchArgument("corner_min_screws", default_value="4"),
        # frame_margin_frac: Sicherheitsrand am Bildrand (Bruchteil), damit Ziele nicht auf der
        # Bildkante liegen (dort ist die Tiefe/Geometrie unzuverlässig).
        DeclareLaunchArgument("frame_margin_frac", default_value="0.04"),
        # display_min_conf: Konfidenzschwelle nur für das Overlay-Rendering (Anzeige), strenger
        # als conf — im Bild werden nur sehr sichere Detektionen eingezeichnet.
        DeclareLaunchArgument("display_min_conf", default_value="0.9"),
        # top_down: erzwingt bevorzugt einen senkrechten Greifansatz (Standard für flach
        # liegende Schrauben).
        DeclareLaunchArgument("top_down", default_value="true"),
        # vida_detector-Node: führt die YOLO11-seg-Erkennung durch, wählt das beste Greifziel,
        # schätzt (bei enable_3d) dessen 3D-Pose und zeichnet das Overlay. Alle oben
        # deklarierten Argumente werden über LaunchConfiguration als Parameter übergeben.
        Node(
            package="vida_vision",
            executable="vida_detector",
            name="vida_detector",
            output="screen",
            parameters=[{
                "image_topic": LaunchConfiguration("image_topic"),
                "conf": LaunchConfiguration("conf"),
                "rate_hz": LaunchConfiguration("rate_hz"),
                "enable_3d": LaunchConfiguration("enable_3d"),
                "depth_mad_k": LaunchConfiguration("depth_mad_k"),
                "axis_min_linearity": LaunchConfiguration("axis_min_linearity"),
                "grasp_tilt_min_deg": LaunchConfiguration("grasp_tilt_min_deg"),
                "target_z_min": LaunchConfiguration("target_z_min"),
                "target_z_max": LaunchConfiguration("target_z_max"),
                "mode_consensus": LaunchConfiguration("mode_consensus"),
                "sticky_radius": LaunchConfiguration("sticky_radius"),
                "center_weight": LaunchConfiguration("center_weight"),
                "conf_weight": LaunchConfiguration("conf_weight"),
                "corner_edge_frac": LaunchConfiguration("corner_edge_frac"),
                "corner_min_screws": LaunchConfiguration("corner_min_screws"),
                "frame_margin_frac": LaunchConfiguration("frame_margin_frac"),
                "display_min_conf": LaunchConfiguration("display_min_conf"),
                "top_down": LaunchConfiguration("top_down"),
            }],
        ),
    ])
