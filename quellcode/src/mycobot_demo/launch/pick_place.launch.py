# Führt die Pick & Place Demo aus.
# Zuerst muss demo.launch.py laufen (move_group + RViz + controllers).
# Für MoveGroupInterface laden wir die robot_description + SRDF + kinematics Parameter.

import os
from pathlib import Path

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node
from moveit_configs_utils import MoveItConfigsBuilder


def generate_launch_description():
    world_share = get_package_share_directory("mycobot_world")
    xacro_path = str(Path(world_share) / "urdf" / "mycobot_world.urdf.xacro")

    use_fake_hw = os.environ.get("USE_FAKE_HARDWARE", "false")
    robot_port = os.environ.get("MYCOBOT_PORT", "/dev/ttyTHS1")

    moveit_config = (
        MoveItConfigsBuilder("mycobot")
        .robot_description(
            file_path=xacro_path,
            mappings={
                "use_fake_hardware": use_fake_hw,
                "robot_port": robot_port,
            },
        )
        .robot_description_semantic(file_path="config/mycobot.srdf")
        .robot_description_kinematics(file_path="config/kinematics.yaml")
        .to_moveit_configs()
    )

    moveit_config.robot_description_kinematics = {
        "robot_description_kinematics": {
            "arm": {
                "kinematics_solver": "trac_ik_kinematics_plugin/TRAC_IKKinematicsPlugin",
                "kinematics_solver_search_resolution": float(0.005),
                "kinematics_solver_timeout": float(0.5),
                "position_only_ik": True,
                "solve_type": "Distance",
                "epsilon": float(1e-5),
            }
        }
    }

    # place-Ziel + grasp z-offset können per env überschrieben werden (run_pick_test.sh).
    pick_params = {
        "place_x": float(os.environ.get("PLACE_X", "0.15")),
        "place_y": float(os.environ.get("PLACE_Y", "0.15")),
        "place_z": float(os.environ.get("PLACE_Z", "0.05")),
        "grasp_z_offset": float(os.environ.get("GRASP_Z_OFFSET", "0.0")),
    }

    return LaunchDescription([
        Node(
            package="mycobot_demo",
            executable="pick_place_cartesian",
            name="pick_place_cartesian",
            output="screen",
            parameters=[
                moveit_config.robot_description,
                moveit_config.robot_description_semantic,
                moveit_config.robot_description_kinematics,
                pick_params,
                {"use_sim_time": False},
            ],
        ),
    ])
