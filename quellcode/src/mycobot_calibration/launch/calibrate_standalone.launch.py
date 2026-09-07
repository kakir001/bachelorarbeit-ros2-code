# Eigenständiger Basis-Stack — Roboter + Slider-GUI + D435i-Kamera OHNE Detektor/handeye.
#
# Zweck: nacktes Grundgerüst zum manuellen Fahren des Roboters und zur Kamera-Vorschau.
# Nützlich zum Testen der Hardware (serieller Bus, Kamera-Stream, TF-Baum), bevor die
# eigentlichen Kalibrier-Nodes hinzugeschaltet werden. Weder charuco_detector noch
# easy_handeye2 laufen hier.
#
# Startet: robot_state_publisher, mycobot_bridge, joint_state_publisher_gui, realsense2_camera.

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
    pkg_world = FindPackageShare('mycobot_world')

    # xacro_path: Welt-URDF (Roboter + Greifer + fest montierte Kamera), zur Laufzeit expandiert.
    xacro_path = PathJoinSubstitution([pkg_world, 'urdf', 'mycobot_world.urdf.xacro'])

    # robot_description: expandiertes URDF als geteilter String-Parameter (rsp + jsp_gui).
    robot_description = {
        'robot_description': ParameterValue(
            Command(['xacro ', xacro_path]),
            value_type=str,
        ),
    }

    # robot_state_publisher: baut aus URDF + /joint_states den TF-Baum.
    rsp = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        name='robot_state_publisher',
        output='screen',
        parameters=[robot_description],
    )

    # mycobot_bridge: serielle Hardware-Schnittstelle; publiziert /joint_states und führt
    # Fahrbefehle von /joint_commands aus.
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
            # command_speed: ruhige Servo-Fahrgeschwindigkeit (0..100).
            'command_speed': 30,
            # min_command_interval_s: Drosselung der Fahrbefehle, schützt den seriellen Bus.
            'min_command_interval_s': 0.20,
        }],
    )

    # joint_state_publisher_gui: Slider-GUI; Ausgabe auf /joint_commands remappt, sodass ein
    # Slider-Zug den echten Roboter fährt.
    jsp_gui = Node(
        package='joint_state_publisher_gui',
        executable='joint_state_publisher_gui',
        name='joint_state_publisher_gui',
        output='screen',
        parameters=[robot_description],
        remappings=[('/joint_states', '/joint_commands')],
    )

    # realsense2_camera: D435i-Treiber. Hier wird zusätzlich ein festes Farbprofil erzwungen
    # (640x480x30) — höhere Auflösung als der Jetson-Sparbetrieb, da dieser Standalone-Stack
    # ohne Detektor/handeye leichter ist und die USB-/CPU-Reserven für die Vorschau reichen.
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
            'color_module.profile': '640x480x30',
        }.items(),
    )

    return LaunchDescription([rsp, bridge, jsp_gui, realsense])
