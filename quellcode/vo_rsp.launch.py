"""Vision-only TF: nur robot_state_publisher (Kamera->robot_base statischer TF).
KEIN RViz/GUI/controller — BERUEHRT den echten Roboter NICHT. Veröffentlicht die
TF-Kette, die der Detector für die Base-Frame-Berechnung braucht.
"""
from launch import LaunchDescription
from launch.substitutions import Command, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    xacro_path = PathJoinSubstitution(
        [FindPackageShare("mycobot_world"), "urdf", "mycobot_world.urdf.xacro"]
    )
    robot_description = {
        "robot_description": ParameterValue(
            Command(["xacro ", xacro_path]), value_type=str
        )
    }
    return LaunchDescription([
        Node(
            package="robot_state_publisher",
            executable="robot_state_publisher",
            name="robot_state_publisher",
            output="screen",
            parameters=[robot_description],
        ),
    ])
