"""MoveIt 2 Demo Launch — Fake-Execution mit ros2_control fake components.

Ausführung:
    ros2 launch mycobot_moveit_config demo.launch.py

Komponenten:
- robot_state_publisher
- ros2_control_node (fake_components/GenericSystem)
- joint_state_broadcaster + arm_controller + gripper_controller spawner
- move_group
- RViz (MotionPlanning Panel)

WICHTIG: Alle Nodes werden mit LC_NUMERIC=C gestartet. Bei System-Locale de_DE
wird das Float-Parsing beschädigt (RViz "0.15" → "0", kinematics_solver_timeout
wird zu String → loadRobotModel Fehler).
"""
import os
from pathlib import Path

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument,
    IncludeLaunchDescription,
    SetEnvironmentVariable,
)
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from moveit_configs_utils import MoveItConfigsBuilder


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


def _ablageplatte_montiert() -> str:
    """Ablageplatte (5x4 Nester) im Modell? Umgebung ABLAGEPLATTE_MONTIERT, Default "true":
    die Platte ist seit 2026-09-11 festgeklebt, Lage aus den angelernten Nestern
    (tools/ablageplatte_modell.py -> urdf/ablageplatte_pose.xacro). 0 zum Abschalten."""
    v = os.environ.get("ABLAGEPLATTE_MONTIERT", "true").strip().lower()
    return "true" if v in ("1", "true", "yes", "ja", "on") else "false"


def _charuco_montiert() -> str:
    """ChArUco-Platte im Modell? Quelle ist die Umgebung (CHARUCO_MONTIERT).

    Die Platte sitzt nur waehrend der Hand-Auge-Kalibrierung am Greifer. Ist sie
    abgeschraubt, muss sie auch aus dem Modell verschwinden - sonst plant MoveIt um
    einen Kollisionskasten herum, den es gar nicht mehr gibt, und jeder Greifversuch
    scheitert, ohne dass eine Fehlermeldung darauf hinweist.

    Default "true" = xacro-Default (Kalibrieraufbau). Akzeptiert 0/1/false/true.
    """
    v = os.environ.get("CHARUCO_MONTIERT", "false").strip().lower()  # 2026-09-14: Platte abgeschraubt -> Default AUS
    return "false" if v in ("0", "false", "no", "nein", "off") else "true"


def generate_launch_description():
    # MoveItConfigsBuilder Galactic: robot_name="mycobot" → mycobot_moveit_config Paket
    world_share = get_package_share_directory("mycobot_world")
    moveit_share = get_package_share_directory("mycobot_moveit_config")
    xacro_path = str(Path(world_share) / "urdf" / "mycobot_world.urdf.xacro")
    ros2_controllers_path = str(Path(moveit_share) / "config" / "ros2_controllers.yaml")

    # Hardware-Auswahl per env — wird von start_mycobot.sh gesetzt.
    # Default: echter Roboter. "sim" mode → use_fake_hardware=true.
    use_fake_hw = os.environ.get("USE_FAKE_HARDWARE", "false")
    robot_port = os.environ.get("MYCOBOT_PORT", "/dev/ttyTHS1")
    charuco = _charuco_montiert()

    # MoveItConfigsBuilder bündelt die gesamte MoveIt-Konfiguration aus den YAML-/SRDF-Dateien
    # des Pakets zu einem Objekt, das anschließend an move_group und RViz weitergereicht wird.
    moveit_config = (
        MoveItConfigsBuilder("mycobot")
        # robot_description: URDF aus der Welt-xacro; die mappings reichen die
        # Hardware-Auswahl (fake vs. echt) und den seriellen Port in die xacro-Argumente durch.
        .robot_description(
            file_path=xacro_path,
            mappings={
                "use_fake_hardware": use_fake_hw,
                "robot_port": robot_port,
                "charuco_montiert": charuco,
                "schale_montiert": _schale_montiert(),
                "trichter_montiert": _trichter_montiert(),
                "ablageplatte_montiert": _ablageplatte_montiert(),
            },
        )
        # SRDF: semantische Beschreibung (Planungsgruppen "arm"/"gripper", disable_collisions).
        .robot_description_semantic(file_path="config/mycobot.srdf")
        # Kinematik-Solver-Konfiguration (wird unten durch den TRAC-IK-Override ersetzt).
        .robot_description_kinematics(file_path="config/kinematics.yaml")
        # Gelenkgrenzen (Position/Geschwindigkeit/Beschleunigung) für die Planung.
        .joint_limits(file_path="config/joint_limits.yaml")
        # Zuordnung MoveIt-Planungsgruppe -> ros2_control-Controller (arm/gripper).
        .trajectory_execution(file_path="config/moveit_controllers.yaml")
        # Planungspipelines (OMPL etc.).
        .planning_pipelines()
        # planning_scene_monitor: sorgt dafür, dass die Planungsszene, Geometrie-Updates,
        # Roboterzustand und TF-Änderungen publiziert werden — nötig, damit RViz und externe
        # Nodes (z.B. der Pick-Node, der Kollisionsobjekte einfügt) dieselbe Szene sehen.
        .planning_scene_monitor(
            publish_planning_scene=True,
            publish_geometry_updates=True,
            publish_state_updates=True,
            publish_transforms_updates=True,
        )
        .to_moveit_configs()
    )

    moveit_config.robot_description_kinematics = {
        "robot_description_kinematics": {
            "arm": {
                "kinematics_solver": "trac_ik_kinematics_plugin/TRAC_IKKinematicsPlugin",
                "kinematics_solver_search_resolution": float(0.005),
                "kinematics_solver_timeout": float(0.5),
                # False ERFORDERLICH (Sitzung 29 sim A/B-Test): der move_group pose-goal IK
                # Sampler nutzt diesen Parameter; bei True werden tilt-grasp hover
                # Ziele mit "goal konnte nicht gesampelt werden" SOFORT abgelehnt
                # (konsistent mit kinematics.yaml=false und pick_tilt.launch.py=False).
                "position_only_ik": False,
                "solve_type": "Distance",
                "epsilon": float(1e-5),
            }
        }
    }

    # robot_state_publisher: publiziert aus der URDF (robot_description) + /joint_states den
    # TF-Baum. move_group und RViz stützen sich auf diese TF, um Kollisionsgeometrie und
    # Roboterzustand räumlich einzuordnen.
    robot_state_publisher_node = Node(
        package="robot_state_publisher",
        executable="robot_state_publisher",
        output="screen",
        parameters=[moveit_config.robot_description],
    )

    # ros2_control_node (controller_manager): lädt die Hardware-Schnittstelle. Je nach
    # use_fake_hardware-Mapping in der URDF ist das die fake_components/GenericSystem
    # (Simulation) oder das echte serielle myCobot-Interface. Er bekommt sowohl die URDF als
    # auch ros2_controllers.yaml (Controller-Definitionen + Update-Rate).
    ros2_control_node = Node(
        package="controller_manager",
        executable="ros2_control_node",
        parameters=[moveit_config.robot_description, ros2_controllers_path],
        output="screen",
    )

    # joint_state_broadcaster_spawner: aktiviert den Broadcaster, der die Gelenkzustände der
    # Hardware-Schnittstelle auf /joint_states publiziert (Grundlage für den TF-Baum).
    joint_state_broadcaster_spawner = Node(
        package="controller_manager",
        executable="spawner",
        arguments=["joint_state_broadcaster", "--controller-manager", "/controller_manager"],
    )
    # arm_controller_spawner: aktiviert den JointTrajectoryController für die 6 Armgelenke —
    # er führt die von move_group geplanten Trajektorien aus.
    arm_controller_spawner = Node(
        package="controller_manager",
        executable="spawner",
        arguments=["arm_controller", "--controller-manager", "/controller_manager"],
    )
    # gripper_controller_spawner: aktiviert den Controller für das Greifergelenk.
    gripper_controller_spawner = Node(
        package="controller_manager",
        executable="spawner",
        arguments=["gripper_controller", "--controller-manager", "/controller_manager"],
    )

    # move_group_node: das Herzstück von MoveIt 2 — bietet Planungs-, IK- und
    # Ausführungs-Services (Planungspipeline, Kollisionsprüfung, Trajektorien-Execution
    # über die oben gespawnten Controller). Es erhält die gesamte MoveIt-Konfiguration
    # (URDF, SRDF, Kinematik-Override, Gelenkgrenzen, Controller-Zuordnung) via to_dict().
    move_group_node = Node(
        package="moveit_ros_move_group",
        executable="move_group",
        output="screen",
        parameters=[moveit_config.to_dict()],
    )

    # mit rviz_config:=<path> kann eine eigene .rviz übergeben werden (z.B. approach.rviz —
    # mit Ziel-Marker + 2 Kamera-Images). Leer → Standard moveit.rviz.
    rviz_config_file = LaunchConfiguration("rviz_config")
    rviz_node = Node(
        package="rviz2",
        executable="rviz2",
        name="rviz2",
        output="log",
        arguments=["-d", rviz_config_file],
        parameters=[
            moveit_config.robot_description,
            moveit_config.robot_description_semantic,
            moveit_config.robot_description_kinematics,
            moveit_config.planning_pipelines,
        ],
        # use_rviz:=false → ohne RViz laufen, um RAM-Thrashing auf dem Jetson Nano zu
        # vermeiden (Wellen-Verifikation mit joint_pose_gui + tf2_echo benötigt kein RViz).
        condition=IfCondition(LaunchConfiguration("use_rviz")),
    )

    # estop_relay: NOT-AUS-Brücke — abonniert das gelatchte /estop-Topic (estop_button
    # bzw. "ros2 topic pub") und leitet den Zustand als "estop"-Kommando an den
    # Kommando-Socket des seriellen Bridge-Prozesses weiter. Nur bei ECHTER Hardware
    # sinnvoll (fake hardware startet keinen Bridge); der Bridge stoppt dann sofort
    # jede Bewegung (mc.stop, Drehmoment bleibt) und blockiert weitere Kommandos.
    # arbeitsraum_marker: zeichnet die GEMESSENE Reichweitengrenze (aussen, innen und
    # den Totsektor von Gelenk 1) als gelatchte Marker auf /arbeitsraum/grenzen.
    # Bewusst NICHT im URDF: es ist eine Messung, kein Bauteil — im Kollisionsmodell
    # wuerde MoveIt an ihr planen. Als Marker laesst sie sich in RViz zuschalten.
    # Quelle: mycobot_world/config/arbeitsraum_ring.json (tools/arbeitsraum_grenze.py).
    arbeitsraum_marker_node = Node(
        package="mycobot_world",
        executable="arbeitsraum_marker.py",
        name="arbeitsraum_marker",
        output="screen",
    )

    # arbeitsraum_wolke: Punktwolke der D435i auf den Kasten ueber der Kaiser-Grundplatte
    # beschneiden -> /camera/arbeitsraum/points (RViz-DepthCloud zeigt nur den Arbeitsraum).
    arbeitsraum_wolke_node = Node(
        package="mycobot_world",
        executable="arbeitsraum_wolke.py",
        name="arbeitsraum_wolke",
        output="screen",
        condition=IfCondition(LaunchConfiguration("use_camera")),
    )

    estop_relay_node = Node(
        package="mycobot_hardware",
        executable="estop_relay.py",
        output="screen",
    )

    # ============== Optional: RealSense D435i Live-Kamera ==============
    # mit use_camera:=true starten Demo + Live-Kamera zusammen.
    # use_camera:=false (default) — nur Simulation, kein Kamera-Node wird gestartet.
    use_camera = LaunchConfiguration("use_camera")
    realsense_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            PathJoinSubstitution([
                FindPackageShare("realsense2_camera"),
                "launch",
                "rs_launch.py",
            ])
        ),
        launch_arguments={
            # pointcloud/align_depth: Punktwolke + zum Farbbild ausgerichtete Tiefe — nötig,
            # damit die Vision-Pipeline aus 2D-Detektionen 3D-Greifpunkte ableiten kann.
            # 2026-09-12: beides per Launch-Argument, weil die Kamera-Messwerkzeuge
            # (schale_finden_kontur.py, punkt_klicken.py) 848x480 brauchen (1 mm/px statt 2)
            # und der Nano das nur OHNE PointCloud traegt. start_mycobot.sh: KAMERA_PROFIL=848.
            "pointcloud.enable": LaunchConfiguration("pointcloud"),
            "align_depth.enable": "true",
            # 424x240x15: bewusst niedrige Auflösung/Framerate. Auf dem Jetson Nano führt die
            # volle Auflösung zu "Out of frame resources" bzw. RAM-/USB-Sättigung; 424x240@15
            # ist das getestete stabile Profil für den kompletten Stack (mit PointCloud).
            "depth_module.profile": LaunchConfiguration("camera_profile"),
            "rgb_camera.profile": LaunchConfiguration("camera_profile"),
            # Infrarot-, Gyro- und Accel-Streams sind für diese Anwendung ungenutzt und
            # werden abgeschaltet, um USB-Bandbreite und CPU zu sparen.
            "enable_infra1": "false",
            "enable_infra2": "false",
            "enable_gyro": "false",
            "enable_accel": "false",
            # initial_reset: setzt die Kamera-Firmware beim Start zurück — umgeht hängende
            # USB-Zustände nach einem vorherigen ungeplanten Beenden des Treibers.
            "initial_reset": "true",
        }.items(),
        # Nur starten, wenn use_camera:=true gesetzt ist.
        condition=IfCondition(use_camera),
    )

    launch_entities = [
        # Locale-Fix — Qt/RViz verlässt sich mehr auf LC_ALL/LANG als auf LC_NUMERIC.
        # Alle zusammen auf C festsetzen.
        SetEnvironmentVariable(name="LC_ALL", value="C"),
        SetEnvironmentVariable(name="LC_NUMERIC", value="C"),
        SetEnvironmentVariable(name="LANG", value="C"),
        DeclareLaunchArgument(
            "use_camera",
            default_value="false",
            description="true → RealSense D435i Kamera-Node mit starten",
        ),
        DeclareLaunchArgument(
            "camera_profile",
            default_value="424x240x15",
            description="RealSense Farb- UND Tiefenprofil (gleich halten!); 848x480x15 zum Einmessen",
        ),
        DeclareLaunchArgument(
            "pointcloud",
            default_value="true",
            description="false → keine PointCloud (noetig bei 848x480 auf dem Nano)",
        ),
        DeclareLaunchArgument(
            "use_rviz",
            default_value="true",
            description="false → RViz nicht starten (Jetson Nano RAM-Ersparnis)",
        ),
        DeclareLaunchArgument(
            "rviz_config",
            default_value=PathJoinSubstitution(
                [FindPackageShare("mycobot_moveit_config"), "rviz", "moveit.rviz"]
            ),
            description="Pfad zur RViz-Config-Datei (Override fuer eigene Ziel+Kamera-Config)",
        ),
        robot_state_publisher_node,
        ros2_control_node,
        joint_state_broadcaster_spawner,
        arm_controller_spawner,
        gripper_controller_spawner,
        move_group_node,
        rviz_node,
        realsense_launch,
        arbeitsraum_marker_node,
        arbeitsraum_wolke_node,
    ]
    # NOT-AUS-Relay nur bei echter Hardware anhängen (Python-Bedingung statt
    # IfCondition, weil die Hardware-Auswahl per Umgebungsvariable erfolgt).
    if use_fake_hw.strip().lower() not in ("true", "1"):
        launch_entities.append(estop_relay_node)

    return LaunchDescription(launch_entities)
