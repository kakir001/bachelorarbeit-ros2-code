"""RViz-only launch — um sich mit einem BEREITS LAUFENDEN move_group zu verbinden.

Wird in run_pick_preview.sh Phase B verwendet: der Stack (move_group) läuft separat, dieser Launch
startet NUR RViz mit den MoveIt-Parametern (robot_description + SRDF + kinematics +
planning_pipelines). `ros2 run rviz2 rviz2 -d ...` erhält diese Parameter nicht und
gibt daher "No Planning Scene Loaded" + Roboter unsichtbar Fehler; dieser Launch löst das.

    ros2 launch mycobot_moveit_config moveit_rviz.launch.py rviz_config:=<path>

LC_NUMERIC=C erforderlich (Qt/RViz beschädigt Float-Parsing bei de_DE Locale).
"""
from pathlib import Path

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, SetEnvironmentVariable
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare
from moveit_configs_utils import MoveItConfigsBuilder
import os


def generate_launch_description():
    world_share = get_package_share_directory("mycobot_world")
    xacro_path = str(Path(world_share) / "urdf" / "mycobot_world.urdf.xacro")
    use_fake_hw = os.environ.get("USE_FAKE_HARDWARE", "false")
    robot_port = os.environ.get("MYCOBOT_PORT", "/dev/ttyTHS1")

    # Dieselbe MoveIt-Konfiguration wie in demo.launch.py aufbauen, ABER ohne
    # trajectory_execution/planning_scene_monitor — dieser Launch startet kein move_group,
    # sondern nur RViz. RViz braucht lediglich robot_description, SRDF, Kinematik und
    # Planungspipelines, um das Roboter- und Planungs-Panel korrekt anzuzeigen.
    moveit_config = (
        MoveItConfigsBuilder("mycobot")
        .robot_description(
            file_path=xacro_path,
            mappings={"use_fake_hardware": use_fake_hw, "robot_port": robot_port},
        )
        .robot_description_semantic(file_path="config/mycobot.srdf")
        .robot_description_kinematics(file_path="config/kinematics.yaml")
        .joint_limits(file_path="config/joint_limits.yaml")
        .planning_pipelines()
        .to_moveit_configs()
    )

    # Gleicher kinematics Override wie demo.launch.py (TRAC-IK, position_only_ik=False).
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

    # rviz_config_file: Pfad zur .rviz-Datei (per Argument überschreibbar). Der RViz-Node
    # erhält zusätzlich die MoveIt-Parameter, ohne die er "No Planning Scene Loaded" meldet
    # und den Roboter nicht rendern kann (siehe Modul-Docstring).
    rviz_config_file = LaunchConfiguration("rviz_config")
    rviz_node = Node(
        package="rviz2",
        executable="rviz2",
        name="rviz2",
        output="screen",
        arguments=["-d", rviz_config_file],
        parameters=[
            moveit_config.robot_description,
            moveit_config.robot_description_semantic,
            moveit_config.robot_description_kinematics,
            moveit_config.planning_pipelines,
        ],
    )

    return LaunchDescription([
        # Locale-Fix (siehe Docstring): Qt/RViz beschädigt sonst das Float-Parsing bei de_DE.
        SetEnvironmentVariable("LC_NUMERIC", "C"),
        SetEnvironmentVariable("LC_ALL", "C"),
        SetEnvironmentVariable("LANG", "C"),
        DeclareLaunchArgument(
            "rviz_config",
            default_value=PathJoinSubstitution(
                [FindPackageShare("mycobot_moveit_config"), "rviz", "moveit.rviz"]
            ),
            description="Pfad zur RViz-Config-Datei",
        ),
        rviz_node,
    ])
