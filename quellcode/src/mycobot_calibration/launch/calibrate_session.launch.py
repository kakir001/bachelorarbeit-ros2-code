# Phase-5-Kalibriersitzung — schlanker Stack OHNE easy_handeye2.
#
# Zweck: Vorbereitungs-/Prüf-Sitzung, in der der Roboter, die Kamera und der
# ChArUco-Detektor zusammen laufen und in RViz visuell kontrolliert werden, BEVOR die
# eigentliche easy_handeye2-Sample-Aufnahme (siehe calibrate_full.launch.py) gestartet
# wird. Hier läuft der Tiefen-Stream mit, damit man die Board-Sicht und den TF-Baum
# vollständig verifizieren kann.
#
# Startet: robot_state_publisher, mycobot_bridge, joint_state_publisher_gui (Slider),
# realsense2_camera, charuco_detector und RViz mit der world.rviz-Config.

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import Command, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    pkg_cal = get_package_share_directory('mycobot_calibration')
    pkg_world = FindPackageShare('mycobot_world')

    # xacro_path: die Welt-URDF (Roboter + Greifer + fest montierte Kamera). Sie wird beim
    # Start durch xacro zu vollständigem URDF-XML expandiert.
    xacro_path = PathJoinSubstitution([pkg_world, 'urdf', 'mycobot_world.urdf.xacro'])
    # rviz_cfg: minimale RViz-Config ohne MoveIt-Panel (RobotModel + TF + Image genügen).
    rviz_cfg = PathJoinSubstitution([pkg_world, 'rviz', 'world.rviz'])

    # robot_description: URDF als String-Parameter, den mehrere Nodes (rsp, jsp_gui) teilen.
    robot_description = {
        'robot_description': ParameterValue(
            Command(['xacro ', xacro_path]),
            value_type=str,
        ),
    }

    # robot_state_publisher: erzeugt aus URDF + /joint_states den TF-Baum (robot_base -> tcp).
    rsp = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        name='robot_state_publisher',
        output='screen',
        parameters=[robot_description],
    )

    # mycobot_bridge: serielle Hardware-Schnittstelle; publiziert echte Encoder-Winkel auf
    # /joint_states und führt Fahrbefehle von /joint_commands aus.
    bridge = Node(
        package='mycobot_calibration',
        executable='mycobot_bridge',
        name='mycobot_bridge',
        output='screen',
        parameters=[{
            # port/baudrate: Jetson-UART /dev/ttyTHS1 bei 1 Mbaud (Firmware-Standard).
            'port': '/dev/ttyTHS1',
            'baudrate': 1000000,
            # rate_hz: 10 Hz Publikationsrate der Gelenkzustände.
            'rate_hz': 10.0,
            # command_speed: ruhige Fahrgeschwindigkeit der Servos (0..100).
            'command_speed': 30,
            # min_command_interval_s: Drosselung der Fahrbefehle, schützt den seriellen Bus.
            'min_command_interval_s': 0.20,
            # freehand=False: Servos bleiben bestromt/gesperrt (kein Handführ-Modus), damit
            # eine per Slider angefahrene Pose für die Sample-Aufnahme stabil gehalten wird.
            'freehand': False,
        }],
    )

    # joint_state_publisher_gui: Slider-GUI; Ausgabe auf /joint_commands remappt, sodass ein
    # Slider-Zug den echten Roboter fährt (die Bridge meldet die realen Werte auf
    # /joint_states zurück).
    jsp_gui = Node(
        package='joint_state_publisher_gui',
        executable='joint_state_publisher_gui',
        name='joint_state_publisher_gui',
        output='screen',
        parameters=[robot_description],
        remappings=[('/joint_states', '/joint_commands')],
    )

    # realsense2_camera: D435i-Treiber. Hier MIT Tiefen-Stream (enable_depth=true), damit die
    # Board-Distanz und Kamera-Sicht in dieser Prüfsitzung vollständig kontrollierbar sind.
    realsense = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(
                get_package_share_directory('realsense2_camera'),
                'launch', 'rs_launch.py',
            )
        ),
        launch_arguments={
            'enable_color': 'true',
            'enable_depth': 'true',
            'pointcloud.enable': 'false',
        }.items(),
    )

    # charuco_detector: erkennt das ChArUco-Board im Farbbild und publiziert
    # camera_color_optical_frame -> charuco_board als TF (Tracking-Seite).
    charuco = Node(
        package='mycobot_calibration',
        executable='charuco_detector',
        name='charuco_detector',
        output='screen',
        parameters=[
            os.path.join(pkg_cal, 'config', 'charuco_params_gripper.yaml'),
            # Frame-/Topic-Verdrahtung wie in calibrate_full.launch.py; min_corners=8 verwirft
            # instabile Board-Posen bei Teilverdeckung.
            {'camera_frame': 'camera_color_optical_frame',
             'board_frame': 'charuco_board',
             'image_topic': '/camera/color/image_raw',
             'camera_info_topic': '/camera/color/camera_info',
             'publish_rate_hz': 5.0,
             'min_corners': 8},
        ],
    )

    # RViz: hier IMMER an (im Gegensatz zu calibrate_full), da diese Sitzung gerade der
    # visuellen Kontrolle dient. Lädt die world.rviz-Config.
    rviz = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        output='screen',
        arguments=['-d', rviz_cfg],
    )

    return LaunchDescription([rsp, bridge, jsp_gui, realsense, charuco, rviz])
