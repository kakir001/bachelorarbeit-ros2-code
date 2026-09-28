# Tilt-bewusster PICK (Sitzung 25): home → hover → Bestätigung → paralleler Abstieg → greifen → heben → halten.
# Zuerst muss demo.launch.py (move_group + controllers) laufen.
# Für MoveGroupInterface werden robot_description + SRDF + kinematics geladen.
# position_only_ik:False — Greifer-Ausrichtung (tilt-ausgerichteter Grasp) MUSS erzwungen werden.

import os
from pathlib import Path

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node
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


def _charuco_montiert() -> str:
    """ChArUco-Platte im Modell? Quelle ist die Umgebung (CHARUCO_MONTIERT).

    Die Platte sitzt nur waehrend der Hand-Auge-Kalibrierung am Greifer. Ist sie
    abgeschraubt, muss sie auch aus dem Modell verschwinden - sonst plant MoveIt um
    einen Kollisionskasten herum, den es gar nicht mehr gibt. Genau das ist am
    2026-09-09 passiert: pick_tilt bekam die Platte weiterhin ins Modell, und JEDE
    Hover-Stellung wurde verworfen ("Unable to sample any valid states for goal
    tree") - gemeldet wurde aber "Ziel unerreichbar", was in die Irre fuehrt.
    Deshalb MUSS jede Stelle, die ein robot_description baut, diese Funktion nutzen.

    Default "true" = xacro-Default (Kalibrieraufbau). Akzeptiert 0/1/false/true.
    """
    v = os.environ.get("CHARUCO_MONTIERT", "true").strip().lower()
    return "false" if v in ("0", "false", "no", "nein", "off") else "true"


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
                "charuco_montiert": _charuco_montiert(),
                "schale_montiert": _schale_montiert(),
                "trichter_montiert": _trichter_montiert(),
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
                "position_only_ik": False,
                "solve_type": "Distance",
                "epsilon": float(1e-5),
            }
        }
    }

    pick_params = {
        "approach_height": float(os.environ.get("APPROACH_HEIGHT", "0.10")),
        "grasp_z_offset": float(os.environ.get("GRASP_Z_OFFSET", "0.0")),
        # pregrasp_open: Greifer-Oeffnung im Hover (Gelenkeinheiten, 0.15 = ganz auf).
        # In einer vollen Schale schieben ganz geoeffnete Finger die Nachbarwellen weg.
        "pregrasp_open": float(os.environ.get("PREGRASP_OPEN", "0.15")),
        # PREVIEW_CONFIRM=1 → jede Bewegung (home/hover/Abstieg/Heben) wird zuerst in
        # RViz vorab angezeigt und NICHT ausgeführt, bis /pick/confirm (ENTER) kommt
        # (echter Roboter wird zuerst beobachtet). 0 → altes Verhalten (nur Abstieg+Greif-Bestätigung).
        "preview_confirm": os.environ.get("PREVIEW_CONFIRM", "0") in ("1", "true", "True"),
        # Der Abstiegs-Cartesian wird mit dieser (niedrigeren) Geschwindigkeit parametrisiert — präziser Grasp.
        "descend_vel_scale": float(os.environ.get("DESCEND_VEL_SCALE", "0.10")),
        # Freiraum (home/hover) Eilgang-Geschwindigkeit — Übergänge ohne Wellen-Kontakt.
        # Abstieg/Greifen/Heben sind nicht betroffen. Entspricht CNC G00.
        "rapid_vel_scale": float(os.environ.get("RAPID_VEL_SCALE", "0.40")),
        # PLACE (Sortierung nach Farbe): nach dem Greifen und Rückkehr zu home die Welle aus der Luft
        # in die ihrer FARBE zugehörige Box ablegen. Box-Mittelpunkte kommen aus box_map.yaml (teach_boxes.py).
        "place_enabled": os.environ.get("PLACE_ENABLED", "1") in ("1", "true", "True"),
        # ABSOLUTES z (von robot_base/Plattform): Boxen an der Reichweitengrenze → nicht das
        # gespeicherte z der Box, sondern feste Höhe. Ausrichtung wird per yaw+tilt-Scan gefunden.
        "place_hover_z": float(os.environ.get("PLACE_HOVER_Z", "0.12")),
        "place_drop_z": float(os.environ.get("PLACE_DROP_Z", "0.08")),
    }

    # Farbe→Box Zuordnung (ros params YAML). teach_boxes.py schreibt sie; falls vorhanden, wird sie geladen.
    # Zum Aktualisieren der Koordinaten ist KEIN REBUILD nötig — neu einlernen/ausführen genügt.
    box_map = os.environ.get("BOX_MAP", os.path.expanduser("~/ros2_ws/box_map.yaml"))
    node_params = [
        moveit_config.robot_description,
        moveit_config.robot_description_semantic,
        moveit_config.robot_description_kinematics,
        pick_params,
        {"use_sim_time": False},
    ]
    if os.path.exists(box_map):
        node_params.append(box_map)

    return LaunchDescription([
        Node(
            package="mycobot_demo",
            executable="pick_tilt",
            name="pick_tilt",
            output="screen",
            parameters=node_params,
        ),
    ])
