"""myCobot 280 JN + adaptive gripper + eye-to-hand RealSense D435i — RViz display.

Verwendung:
    ros2 launch mycobot_world world.launch.py
    ros2 launch mycobot_world world.launch.py use_gui:=false
"""
import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.conditions import IfCondition
from launch.substitutions import Command, LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    pkg = FindPackageShare("mycobot_world")

    # xacro_path: die Welt-URDF (Roboter + Greifer + fest montierte D435i). Command(["xacro ..."])
    # expandiert sie zur Laufzeit zu vollständigem URDF-XML.
    xacro_path = PathJoinSubstitution([pkg, "urdf", "mycobot_world.urdf.xacro"])
    # rviz_config: Anzeige-Layout dieser Welt-Visualisierung.
    rviz_config = PathJoinSubstitution([pkg, "rviz", "world.rviz"])

    # use_gui: schaltet die Slider-GUI zu (true) oder lässt sie weg (false), z.B. wenn die
    # Gelenkzustände von einer anderen Quelle (echte Bridge) kommen.
    use_gui = LaunchConfiguration("use_gui")

    # robot_description: expandiertes URDF als geteilter String-Parameter für rsp + Slider-GUI.
    robot_description = {
        "robot_description": ParameterValue(
            Command(["xacro ", xacro_path]),
            value_type=str,
        ),
    }

    return LaunchDescription([
        DeclareLaunchArgument(
            "use_gui",
            default_value="true",
            description="Joint-Slider-GUI starten.",
        ),

        # robot_state_publisher: erzeugt aus URDF + /joint_states den TF-Baum, den RViz zum
        # Rendern von Roboter und Kamera-Frame nutzt.
        Node(
            package="robot_state_publisher",
            executable="robot_state_publisher",
            name="robot_state_publisher",
            output="screen",
            parameters=[robot_description],
        ),

        # joint_state_publisher_gui: reine Visualisierungs-GUI (KEIN Remap auf /joint_commands,
        # da hier kein echter Roboter angesprochen wird) — die Slider publizieren direkt auf
        # /joint_states, damit man das Modell in RViz durchbewegen kann. Nur bei use_gui:=true.
        Node(
            package="joint_state_publisher_gui",
            executable="joint_state_publisher_gui",
            name="joint_state_publisher_gui",
            output="screen",
            parameters=[robot_description],
            condition=IfCondition(use_gui),
        ),

        # rviz2: startet die 3D-Visualisierung mit der world.rviz-Config.
        Node(
            package="rviz2",
            executable="rviz2",
            name="rviz2",
            output="screen",
            arguments=["-d", rviz_config],
        ),
    ])
