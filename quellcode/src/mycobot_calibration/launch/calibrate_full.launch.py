# Phase 5 — Hand-eye-Kalibrierung kompletter Stack.
#
# Was es startet:
#   1. robot_state_publisher  (URDF -> TF)
#   2. mycobot_bridge         (liest seriell, publiziert /joint_states,
#                              hört auf /joint_commands und führt send_radians aus)
#   3. joint_state_publisher_gui  (Slider-GUI -> /joint_commands remap)
#   4. realsense2_camera      (D435i live stream)
#   5. charuco_detector       (image -> camera->board TF)
#   6. easy_handeye2 calibrate (rqt sampling + compute UI)
#   7. RViz (vorhandene world.rviz config)
#
# Verwendung:
#   export LC_ALL=C
#   source /opt/ros/galactic/setup.bash
#   source ~/ros2_ws/install/setup.bash
#   ros2 launch mycobot_calibration calibrate_full.launch.py
#
# Die Servos bleiben IMMER GESPERRT. Gib per Slider eine Pose, der Roboter fährt dorthin+hält,
# klicke im rqt-Panel "Take sample", sammle mindestens 15 verschiedene Posen.
#
# WARNUNG: dieser Launch ERSETZT Phase 6 NICHT — nur ein minimaler Stack für die
# Phase-5-Kalibrierungssitzung. Nicht gleichzeitig mit demo.launch.py ausführen (Port-Konflikt).

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import Command, LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue
from launch_ros.substitutions import FindPackageShare


def _schale_montiert() -> str:
    """Bereitstellungsschale im Modell? Quelle ist die Umgebung (SCHALE_MONTIERT).

    Die Schale ist ein Kreisringausschnitt (39.9 Grad, R 186.9..301.9 mm, Wand 2 mm,
    Wandhoehe 17 mm). Sie steht als Kollisionskoerper im Modell, damit MoveIt Griffe
    dicht am Schalenrand von selbst verwirft, statt Erfolg zu melden und die Finger
    an die Wand zu fahren.

    Default "false" = xacro-Default. Solange die LAGE der Schale nicht gemessen ist
    (tools/schale_pose_klicken.py schreibt sie nach urdf/schale_pose.xacro), waere sie
    ein Kollisionskasten an der falschen Stelle - und der ist schlimmer als gar keiner,
    weil MoveIt dann gueltige Griffe stillschweigend verwirft. Akzeptiert 0/1/false/true.
    """
    v = os.environ.get("SCHALE_MONTIERT", "false").strip().lower()
    return "true" if v in ("1", "true", "yes", "ja", "on") else "false"


def _trichter_montiert() -> str:
    """Trichter im Modell? Quelle ist die Umgebung (TRICHTER_MONTIERT).

    Schraeger Hohlkegel mit Auslaufrohr und Fussplatte (mycobot_world/urdf/trichter.xacro).
    Am 2026-09-10 hat sich der Greifer am Trichter verbogen, weil er im Modell fehlte;
    mit ihm verwirft MoveIt Stellungen, die in den Kegel fahren.

    Default "false" = xacro-Default, solange die Neigungsrichtung (yaw in
    urdf/trichter_pose.xacro) nicht gemessen ist - ein Kegel, der nach der falschen Seite
    kippt, ist schlimmer als keiner. Akzeptiert 0/1/false/true wie SCHALE_MONTIERT.
    """
    v = os.environ.get("TRICHTER_MONTIERT", "false").strip().lower()
    return "true" if v in ("1", "true", "yes", "ja", "on") else "false"


def _charuco_montiert() -> str:
    """ChArUco-Platte im Modell? Quelle ist die Umgebung (CHARUCO_MONTIERT).

    Die Platte sitzt nur waehrend der Hand-Auge-Kalibrierung am Greifer. Ist sie
    abgeschraubt, muss sie auch aus dem Modell verschwinden - sonst plant MoveIt um
    einen Kollisionskasten herum, den es gar nicht mehr gibt. Genau das ist am
    2026-09-09 passiert: pick_tilt bekam die Platte weiterhin ins Modell, und JEDE
    Hover-Stellung wurde verworfen ("Unable to sample any valid states for goal
    tree") - gemeldet wurde aber "Ziel unerreichbar", was in die Irre fuehrt.
    Deshalb MUSS jede Stelle, die ein robot_description baut, diese Funktion nutzen.

    Default "true" = xacro-Default (Kalibrieraufbau). Akzeptiert 0/1/false/true.
    """
    v = os.environ.get("CHARUCO_MONTIERT", "true").strip().lower()
    return "false" if v in ("0", "false", "no", "nein", "off") else "true"


def generate_launch_description():
    pkg_cal = get_package_share_directory('mycobot_calibration')
    pkg_world = FindPackageShare('mycobot_world')
    pkg_moveit = FindPackageShare('mycobot_moveit_config')

    xacro_path = PathJoinSubstitution([pkg_world, 'urdf', 'mycobot_world.urdf.xacro'])
    # world.rviz — minimale Config ohne MoveIt (kein move_group, ruft das MotionPlanning-Panel
    # nicht auf). RobotModel + TF + Image Displays genügen.
    rviz_cfg = PathJoinSubstitution([pkg_world, 'rviz', 'world.rviz'])

    robot_description = {
        'robot_description': ParameterValue(
            Command(['xacro ', xacro_path,
                     ' charuco_montiert:=', _charuco_montiert(),
                     ' schale_montiert:=', _schale_montiert(),
                     ' trichter_montiert:=', _trichter_montiert()]),
            value_type=str,
        ),
    }

    # robot_state_publisher: liest die URDF aus dem 'robot_description'-Parameter und
    # publiziert daraus den kompletten statischen + gelenkabhängigen TF-Baum. Er
    # abonniert /joint_states (von der Bridge) und wandelt jeden Gelenkwinkel in eine
    # TF-Transformation um. Für die Hand-Auge-Kalibrierung ist dies die Quelle der
    # robot_base -> tcp Kette, die easy_handeye2 als "Roboter-Seite" der AX=XB-Gleichung
    # verwendet.
    rsp = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        name='robot_state_publisher',
        output='screen',
        parameters=[robot_description],
    )

    # mycobot_bridge: die Hardware-Schnittstelle zum echten Roboter. Sie liest über die
    # serielle Schnittstelle die realen Encoder-Winkel und publiziert sie auf /joint_states;
    # gleichzeitig hört sie auf /joint_commands und sendet Zielposen per send_radians an
    # die Servos (Servos bleiben dabei bestromt/gesperrt).
    bridge = Node(
        package='mycobot_calibration',
        executable='mycobot_bridge',
        name='mycobot_bridge',
        output='screen',
        parameters=[{
            # port/baudrate: der Jetson-UART /dev/ttyTHS1 bei 1 Mbaud — die von der
            # myCobot-Firmware erwartete Standard-Baudrate.
            'port': '/dev/ttyTHS1',
            'baudrate': 1000000,
            # rate_hz: Frequenz, mit der die Bridge /joint_states publiziert (10 Hz reicht
            # für die Kalibrierung, da nur bei stehendem Roboter Samples genommen werden).
            'rate_hz': 10.0,
            # command_speed: Servo-Fahrgeschwindigkeit im Freiraum (URDF-unabhängiger
            # Firmware-Wert 0..100). 30 ist ein ruhiger, aber nicht träger Kompromiss.
            'command_speed': 30,
            # min_command_interval_s: Mindestabstand zwischen zwei Fahrbefehlen (0.2 s), um
            # den seriellen Bus nicht zu überfluten und Firmware-Aussetzer zu vermeiden.
            'min_command_interval_s': 0.20,
        }],
    )

    # joint_state_publisher_gui: Slider-Fenster zum manuellen Einstellen jeder Gelenkpose.
    # Es publiziert normalerweise auf /joint_states — per Remap wird die Ausgabe stattdessen
    # auf /joint_commands umgeleitet, also genau das Topic, auf das die Bridge hört. So
    # bewegt ein Slider-Zug den echten Roboter, während die Bridge die realen Encoder-Werte
    # unabhängig auf /joint_states zurückmeldet.
    jsp_gui = Node(
        package='joint_state_publisher_gui',
        executable='joint_state_publisher_gui',
        name='joint_state_publisher_gui',
        output='screen',
        parameters=[robot_description],
        remappings=[('/joint_states', '/joint_commands')],
    )

    # realsense2_camera: startet den D435i-Kameratreiber (inkludiert dessen rs_launch.py).
    # Er publiziert den Farb-Stream auf /camera/color/image_raw und die Intrinsics auf
    # /camera/color/camera_info — die Eingänge des charuco_detector.
    realsense = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(
                get_package_share_directory('realsense2_camera'),
                'launch', 'rs_launch.py',
            )
        ),
        launch_arguments={
            'enable_color': 'true',
            # Die Kalibrierung benötigt nur den Farb-Stream. Der Tiefen-Stream ist reiner
            # Overhead (CPU/USB/RAM) und trug zum Einfrieren des Jetson bei — daher aus.
            'enable_depth': 'false',
            'pointcloud.enable': 'false',
        }.items(),
    )

    # charuco_detector: erkennt das ChArUco-Board im Farbbild und publiziert die gemessene
    # Transformation camera_color_optical_frame -> charuco_board als TF. Das ist die
    # "Tracking-Seite" der Kalibrierung. Die YAML enthält die physischen Board-Maße
    # (Feldgröße, Marker-Größe, Dictionary); die Inline-Parameter verdrahten die Frames
    # und Topics.
    charuco = Node(
        package='mycobot_calibration',
        executable='charuco_detector',
        name='charuco_detector',
        output='screen',
        parameters=[
            os.path.join(pkg_cal, 'config', 'charuco_params_gripper.yaml'),
            # camera_frame/board_frame: Namen der zu publizierenden TF-Kette.
            # image_topic/camera_info_topic: Eingänge vom RealSense-Treiber.
            # publish_rate_hz: 5 Hz genügt bei statischer Sampling-Aufnahme.
            # min_corners: mind. 8 erkannte Board-Ecken, sonst wird die Pose verworfen
            # (verhindert instabile Schätzungen bei teilweise verdecktem Board).
            {'camera_frame': 'camera_color_optical_frame',
             'board_frame':  'charuco_gemessen',
             'image_topic':  '/camera/color/image_raw',
             'camera_info_topic': '/camera/color/camera_info',
             'publish_rate_hz': 5.0,
             'min_corners': 8},
        ],
    )

    # easy_handeye2 calibrate: startet die eigentliche Kalibrier-Pipeline (rqt-Panel zum
    # Sample-Sammeln + Berechnen). Sie sammelt Paare aus Roboter-Pose (robot_base -> tcp) und
    # Tracking-Pose (camera -> board) und löst die AX=XB Hand-Auge-Gleichung.
    handeye = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(
                get_package_share_directory('easy_handeye2'),
                'launch', 'calibrate.launch.py',
            )
        ),
        launch_arguments={
            # name: eindeutige Kennung, unter der das Kalibrierergebnis gespeichert wird.
            'name': 'mycobot_d435i_eob',
            # calibration_type eye_on_base: Kamera ist ORTSFEST montiert (eye-to-hand), das
            # Board sitzt am Greifer/TCP — passend zur Standmontage der D435i in diesem Setup.
            'calibration_type': 'eye_on_base',
            # tracking_base_frame / tracking_marker_frame: die Tracking-Seite (Kamera sieht
            # Board), identisch mit der vom charuco_detector publizierten TF-Kette.
            'tracking_base_frame': 'camera_color_optical_frame',
            'tracking_marker_frame': 'charuco_gemessen',
            # robot_base_frame / robot_effector_frame: die Roboter-Seite (robot_state_publisher
            # liefert robot_base -> tcp aus der URDF + /joint_states).
            'robot_base_frame': 'robot_base',
            'robot_effector_frame': 'tcp',
        }.items(),
    )

    # RViz ist OPTIONAL — auf dem Jetson Nano sättigt es (zusammen mit dem restlichen Stack)
    # RAM/CPU und friert den Desktop ein. Standardmäßig aus; nur auf einer Maschine mit
    # Reserven einschalten:  use_rviz:=true
    use_rviz = LaunchConfiguration('use_rviz')
    # RViz-Node mit der minimalen world.rviz-Config; nur aktiv, wenn use_rviz:=true
    # (IfCondition wertet die LaunchConfiguration aus).
    rviz = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        output='screen',
        arguments=['-d', rviz_cfg],
        condition=IfCondition(use_rviz),
    )

    # Reihenfolge im LaunchDescription: zuerst das use_rviz-Argument deklarieren, dann alle
    # Nodes/Includes starten. ROS 2 startet sie parallel; der TF-Baum + Kamera-Stream sind
    # kurz nach dem Start verfügbar, sodass easy_handeye2 direkt Samples nehmen kann.
    return LaunchDescription([
        DeclareLaunchArgument('use_rviz', default_value='false'),
        rsp, bridge, jsp_gui, realsense, charuco, handeye, rviz,
    ])
