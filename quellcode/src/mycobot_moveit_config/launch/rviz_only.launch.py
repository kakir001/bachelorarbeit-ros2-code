"""Nur RViz (MotionPlanning-Panel) zu einem BEREITS LAUFENDEN Stack starten.

Wenn RViz auf dem Nano abstuerzt oder geschlossen wurde, muss nicht der ganze Stack
(Bridge, move_group, Kamera) neu - das kostet Minuten und die Kamera haengt sich beim
zweiten Knoten auf. Diese Datei baut dieselbe MoveIt-Konfiguration wie demo.launch.py
(gleiche Umgebungsvariablen CHARUCO_MONTIERT / SCHALE_MONTIERT / TRICHTER_MONTIERT!)
und startet nur den RViz-Knoten.

    ros2 launch mycobot_moveit_config rviz_only.launch.py
    ros2 launch mycobot_moveit_config rviz_only.launch.py rviz_config:=<eigene.rviz>
"""
import importlib.util
import os
from pathlib import Path

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, SetEnvironmentVariable
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare
from moveit_configs_utils import MoveItConfigsBuilder


def _demo():
    """demo.launch.py als Modul laden - die _*_montiert()-Helfer bleiben an EINER Stelle."""
    pfad = Path(get_package_share_directory("mycobot_moveit_config")) / "launch" / "demo.launch.py"
    spec = importlib.util.spec_from_file_location("demo_launch", pfad)
    mod = importlib.util.module_from_spec(spec); spec.loader.exec_module(mod)
    return mod


def generate_launch_description():
    demo = _demo()
    world_share = get_package_share_directory("mycobot_world")
    xacro_path = str(Path(world_share) / "urdf" / "mycobot_world.urdf.xacro")
    moveit_config = (
        MoveItConfigsBuilder("mycobot")
        .robot_description(
            file_path=xacro_path,
            mappings={
                "use_fake_hardware": os.environ.get("USE_FAKE_HARDWARE", "false"),
                "robot_port": os.environ.get("MYCOBOT_PORT", "/dev/ttyTHS1"),
                "charuco_montiert": demo._charuco_montiert(),
                "schale_montiert": demo._schale_montiert(),
                "trichter_montiert": demo._trichter_montiert(),
            },
        )
        .robot_description_semantic(file_path="config/mycobot.srdf")
        .robot_description_kinematics(file_path="config/kinematics.yaml")
        .planning_pipelines()
        .to_moveit_configs()
    )
    # TRAC-IK wie in demo.launch.py - sonst "No active joints or end effectors found"
    moveit_config.robot_description_kinematics = {
        "robot_description_kinematics": {
            "arm": {
                "kinematics_solver": "trac_ik_kinematics_plugin/TRAC_IKKinematicsPlugin",
                "kinematics_solver_search_resolution": float(0.005),
                "kinematics_solver_timeout": float(0.5),
                "position_only_ik": False,
                "solve_type": "Distance",
                "epsilon": float(1e-5),
            }
        }
    }
    rviz_node = Node(
        package="rviz2", executable="rviz2", name="rviz2", output="log",
        arguments=["-d", LaunchConfiguration("rviz_config")],
        parameters=[
            moveit_config.robot_description,
            moveit_config.robot_description_semantic,
            moveit_config.robot_description_kinematics,
            moveit_config.planning_pipelines,
        ],
    )
    return LaunchDescription([
        SetEnvironmentVariable("LC_NUMERIC", "C"),
        SetEnvironmentVariable("LC_ALL", "C"),
        DeclareLaunchArgument(
            "rviz_config",
            default_value=PathJoinSubstitution([FindPackageShare("mycobot_moveit_config"), "rviz", "moveit.rviz"]),
        ),
        rviz_node,
    ])
