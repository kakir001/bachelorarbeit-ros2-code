# Hand-Auge-Kalibrierung — eye-on-base (Kamera ortsfest, Board am Roboter/Greifer).
# Setzt voraus, dass demo.launch.py (Roboter + MoveIt + D435i-Kamera) BEREITS läuft; dieser
# Launch fügt nur die kalibrierspezifischen Nodes HINZU, ohne Treiber/Roboter erneut zu
# starten (vermeidet Port-Konflikt am seriellen /dev/ttyTHS1 und Kamera-Doppelbelegung).
# Startet:
#   1. charuco_detector          -> publiziert TF camera_color_optical_frame -> charuco_board
#   2. easy_handeye2 calibrate.launch.py — Sample-Aufnahme + Lösung der AX=XB-Gleichung
#   3. joint_pose_gui            -> Gelenk-Slider, fährt Servos direkt (an MoveIt vorbei)

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node


def generate_launch_description():
    pkg_share = get_package_share_directory('mycobot_calibration')
    # params_yaml: physische ChArUco-Board-Parameter (Feld-/Marker-Größe, Dictionary).
    params_yaml = os.path.join(pkg_share, 'config', 'charuco_params_gripper.yaml')

    # charuco_detector: erkennt das Board im Farbbild und publiziert die gemessene Pose
    # camera_color_optical_frame -> charuco_board als TF (Tracking-Seite der Kalibrierung).
    charuco_detector = Node(
        package='mycobot_calibration',
        executable='charuco_detector',
        name='charuco_detector',
        output='screen',
        parameters=[
            params_yaml,
            # Frame-/Topic-Verdrahtung: Eingänge vom bereits laufenden RealSense-Treiber
            # (image + camera_info), Ausgabe-TF-Kette camera -> board.
            # min_corners=8 verwirft instabile Posen bei teilweise verdecktem Board.
            {'camera_frame': 'camera_color_optical_frame',
             'board_frame':  'charuco_board',
             'image_topic':  '/camera/color/image_raw',
             'camera_info_topic': '/camera/color/camera_info',
             'publish_rate_hz': 5.0,
             'min_corners': 8},
        ],
    )

    # easy_handeye2 calibrate: rqt-Panel zum Sammeln der Pose-Paare und Berechnen der
    # eye-on-base-Transformation (Kamera -> Roboterbasis).
    easy_handeye2_calibrate = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(
                get_package_share_directory('easy_handeye2'),
                'launch', 'calibrate.launch.py',
            )
        ),
        launch_arguments={
            # name: Kennung, unter der das Ergebnis gespeichert/geladen wird.
            'name': 'mycobot_d435i_eob',
            # eye_on_base: Kamera fest, Board am Effektor — passt zur Standmontage der D435i.
            'calibration_type': 'eye_on_base',
            # Tracking-Seite (Kamera sieht Board) — identisch zur charuco_detector-TF-Kette.
            'tracking_base_frame': 'camera_color_optical_frame',
            'tracking_marker_frame': 'charuco_board',
            # Roboter-Seite (aus der laufenden demo.launch.py: robot_base -> tcp).
            'robot_base_frame': 'robot_base',
            'robot_effector_frame': 'tcp',
        }.items(),
    )

    # joint_pose_gui: Gelenk-Slider-GUI; der Bediener fährt jedes Gelenk direkt zum
    # Controller (an MoveIt vorbei). So lässt sich eine Pose präzise einstellen und dann ein
    # Sample nehmen, während die Servos gesperrt bleiben.
    joint_pose_gui = Node(
        package='mycobot_calibration',
        executable='joint_pose_gui',
        name='joint_pose_gui',
        output='screen',
    )

    return LaunchDescription([charuco_detector, easy_handeye2_calibrate, joint_pose_gui])
