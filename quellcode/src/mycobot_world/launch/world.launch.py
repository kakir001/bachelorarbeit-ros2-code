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
    einen Kollisionskasten herum, den es gar nicht mehr gibt, und jeder Greifversuch
    scheitert, ohne dass eine Fehlermeldung darauf hinweist.

    Default "true" = xacro-Default (Kalibrieraufbau). Akzeptiert 0/1/false/true.
    """
    v = os.environ.get("CHARUCO_MONTIERT", "true").strip().lower()
    return "false" if v in ("0", "false", "no", "nein", "off") else "true"


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
            Command(["xacro ", xacro_path,
                     " charuco_montiert:=", _charuco_montiert(),
                     " schale_montiert:=", _schale_montiert(),
                     " trichter_montiert:=", _trichter_montiert()]),
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
