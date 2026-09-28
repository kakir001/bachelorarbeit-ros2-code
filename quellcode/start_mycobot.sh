#!/bin/bash
export CYCLONEDDS_URI="${CYCLONEDDS_URI:-file://$HOME/ros2_ws/cyclonedds_loopback.xml}"   # DDS nur loopback, s. Datei
# myCobot 280 JN: MoveIt2 + RViz + ros2_control + D435i camera.
# Default: ECHTER Roboter (mycobot_hardware/MyCobotHardware via pymycobot).
# Verwendung:
#   ./start_mycobot.sh         → echter Roboter (ttyTHS1)
#   ./start_mycobot.sh sim     → fake_components (Sim ohne Roboter)
# Locale C erforderlich (Qt/RViz Punkt-Komma-Problem).

export LC_ALL=C
export LC_NUMERIC=C
export LANG=C

# CycloneDDS: Jetson default rmem_max (212KB) reicht nicht für Bild-Nachrichten
sudo sysctl -w net.core.rmem_max=8388608 net.core.wmem_max=8388608 \
             net.core.rmem_default=8388608 net.core.wmem_default=8388608 \
             2>/dev/null

# Jetson GPU: railgate aus + fan max (RViz OGRE segfault-Vorbeugung)
echo 0 | sudo tee /sys/devices/57000000.gpu/railgate_enable >/dev/null 2>&1
sudo sh -c 'echo 255 > /sys/devices/pwm-fan/target_pwm' 2>/dev/null

# D435i USB autosuspend ausschalten
for dev in /sys/bus/usb/devices/*/idVendor; do
  [ "$(cat "$dev" 2>/dev/null)" = "8086" ] && echo on | sudo tee "$(dirname "$dev")/power/control" >/dev/null 2>&1
done

source /opt/ros/galactic/setup.bash
source "$HOME/ros2_ws/install/setup.bash"

cd "$HOME/ros2_ws" || exit 1

# --- Waisen-Wache ------------------------------------------------------------
# 2026-09-12: nach einem harten Stack-Ende (ros2 launch kam auf SIGINT nicht zu Ende und
# wurde gekillt) ueberlebten arbeitsraum_wolke.py (je ~30 % CPU!), arbeitsraum_marker.py
# und die serielle Bruecke - aus ZWEI alten Stacks. Doppelte Knoten = doppelte Topics,
# Hitze, und die Bruecke haelt den seriellen Port. Deshalb: alte Reste zuerst aufraeumen
# (./stop_mycobot.sh) - hier nur melden und abbrechen, nichts blind killen.
REST=$(pgrep -af "arbeitsraum_wolke|arbeitsraum_marker|mycobot_bridge|estop_relay|realsense2_camera_node|move_group|ros2_control_node|robot_state_publisher" | grep -v pgrep)
if [ -n "$REST" ]; then
  echo "[start_mycobot] !! Es laufen noch Reste eines alten Stacks:"
  echo "$REST" | cut -c1-110 | sed 's/^/    /'
  echo "[start_mycobot] !! Erst ./stop_mycobot.sh, dann neu starten. (STACK_REST_IGNORIEREN=1 uebergeht das.)"
  [ "${STACK_REST_IGNORIEREN:-0}" = "1" ] || exit 1
fi

if [ "${1:-}" = "sim" ]; then
  export USE_FAKE_HARDWARE="true"
  echo "[start_mycobot] SIM MODE — fake_components/GenericSystem"
else
  export USE_FAKE_HARDWARE="false"
  echo "[start_mycobot] ECHTER ROBOTER — mycobot_hardware/MyCobotHardware @ ${MYCOBOT_PORT:-/dev/ttyTHS1}"
fi

# --- ChArUco-Platte im Modell? ----------------------------------------------
# Die Platte sitzt nur waehrend der Hand-Auge-Kalibrierung am Greifer. Ist sie
# abgeschraubt, muss sie auch aus dem Modell — sonst plant MoveIt um einen
# Kollisionskasten herum, den es nicht mehr gibt. Ohne Platte starten:
#   CHARUCO_MONTIERT=0 ./start_mycobot.sh
# 2026-09-14: Default AUS - die Platte ist abgeschraubt (Benutzer: "was ist das weisse Rechteck?"). Zum Kalibrieren CHARUCO_MONTIERT=1.
export CHARUCO_MONTIERT="${CHARUCO_MONTIERT:-false}"
if [ "$CHARUCO_MONTIERT" = "0" ] || [ "$CHARUCO_MONTIERT" = "false" ]; then
  echo "[start_mycobot] ChArUco-Platte: NICHT im Modell (abgeschraubt)"
else
  echo "[start_mycobot] ChArUco-Platte: im Modell (Kalibrieraufbau)"
fi


# --- Bereitstellungsschale im Modell? ---------------------------------------
# Kreisringausschnitt (39.9 Grad, R 186.9..301.9 mm, Wand 2 mm, Wandhoehe 17 mm),
# damit MoveIt Griffe dicht am Schalenrand von selbst verwirft. Default AUS, weil
# die LAGE der Schale gemessen sein muss (tools/schale_pose_klicken.py schreibt sie
# nach urdf/schale_pose.xacro). Ein Kollisionskoerper an der falschen Stelle ist
# schlimmer als gar keiner - er verwirft gueltige Griffe, ohne dass es auffaellt.
export SCHALE_MONTIERT="${SCHALE_MONTIERT:-false}"
if [ "$SCHALE_MONTIERT" = "1" ] || [ "$SCHALE_MONTIERT" = "true" ]; then
  echo "[start_mycobot] Bereitstellungsschale: im Modell (Lage aus urdf/schale_pose.xacro)"
else
  echo "[start_mycobot] Bereitstellungsschale: NICHT im Modell - Lage noch nicht gemessen"
  echo "[start_mycobot]   Messen: tools/schale_pose_klicken.py , dann SCHALE_MONTIERT=1"
fi

# --- Trichter im Kollisionsmodell? ------------------------------------------
# Am 2026-09-10 hat sich der Greifer am Trichter verbogen, weil der Trichter im
# Modell fehlte. Default AUS, solange die Neigungsrichtung (yaw in
# urdf/trichter_pose.xacro) nicht gemessen ist. VOR jeder Fahrt in Trichternaehe
# einschalten: TRICHTER_MONTIERT=1
export TRICHTER_MONTIERT="${TRICHTER_MONTIERT:-false}"
if [ "$TRICHTER_MONTIERT" = "1" ] || [ "$TRICHTER_MONTIERT" = "true" ]; then
  echo "[start_mycobot] Trichter: im Modell (Lage aus urdf/trichter_pose.xacro)"
else
  echo "[start_mycobot] Trichter: NICHT im Modell - Neigungsrichtung noch nicht gemessen"
  echo "[start_mycobot]   !! NICHT in Trichternaehe fahren. Messen, dann TRICHTER_MONTIERT=1"
fi

# --- Kamera-Pose automatisch nachfuehren (vor dem Stack!) --------------------
# Misst die Kamera gegen die bekannte Grundplatte und schreibt bei echter
# Abweichung urdf/camera_pose.xacro + die .calib neu. Muss VOR dem Stack laufen:
# danach haelt realsense2_camera die Kamera belegt und der robot_state_publisher
# hat das URDF bereits gelesen. Abschalten: AUTO_CAM_CALIB=0 ./start_mycobot.sh
# Fehlschlag ist NICHT toedlich — dann bleibt der bisherige Stand stehen.
if [ "${AUTO_CAM_CALIB:-1}" = "1" ]; then
  echo "[start_mycobot] Kamera-Pose pruefen (tools/auto_camera_calibration.py)..."
  timeout 240 python3 "$HOME/ros2_ws/tools/auto_camera_calibration.py" \
      --repeat "${AUTO_CAM_REPEAT:-3}" 2>&1 | sed 's/^/[cam-calib] /'
  rc=${PIPESTATUS[0]}
  [ "$rc" = "0" ] || echo "[start_mycobot] Kamera-Nachfuehrung uebersprungen (rc=$rc) — bisheriger Stand gilt."
fi

# --- Ablageplatte im Modell (Standard AN, festgeklebt seit 11.9.) ------------------
export ABLAGEPLATTE_MONTIERT="${ABLAGEPLATTE_MONTIERT:-1}"
if [ "$ABLAGEPLATTE_MONTIERT" = "1" ] || [ "$ABLAGEPLATTE_MONTIERT" = "true" ]; then
  echo "[start_mycobot] Ablageplatte: im Modell (Lage aus urdf/ablageplatte_pose.xacro, 20 Nestbosse)"
else
  echo "[start_mycobot] Ablageplatte: NICHT im Modell (ABLAGEPLATTE_MONTIERT=0)"
fi

# --- Kameraprofil: KAMERA_PROFIL=848 ./start_mycobot.sh --------------------------
# 424x240 (Standard, mit PointCloud fuer RViz/arbeitsraum_wolke) oder 848x480 OHNE
# PointCloud zum Einmessen (schale_finden_kontur.py, punkt_klicken.py: 1 mm/px).
# Beide Profile sind gegen dieselbe Kalibrierung gueltig (camera_info skaliert mit).
if [ "${KAMERA_PROFIL:-424}" = "848" ]; then
  KAMERA_ARGS="camera_profile:=848x480x15 pointcloud:=false"
  echo "[start_mycobot] Kamera 848x480x15 OHNE PointCloud (KAMERA_PROFIL=848)"
elif [ "${KAMERA_PROFIL:-424}" = "1280" ]; then
  # ChArUco am Greifer (12-mm-Marker): erst ab 1280x720 sicher erkannt
  KAMERA_ARGS="camera_profile:=1280x720x15 pointcloud:=false"
  echo "[start_mycobot] Kamera 1280x720x15 OHNE PointCloud (KAMERA_PROFIL=1280, ChArUco)"
else
  KAMERA_ARGS="camera_profile:=424x240x15 pointcloud:=true"
fi

ros2 launch mycobot_moveit_config demo.launch.py use_camera:=true $KAMERA_ARGS &
LAUNCH_PID=$!

echo "[start_mycobot] Warten bis die Controller bereit sind..."
for i in $(seq 1 30); do
  if ros2 control list_controllers 2>/dev/null | grep -q "arm_controller.*active"; then
    break
  fi
  sleep 1
done

# --- Nullstellung beim Start: NUR auf Wunsch --------------------------------
# ⚠️ 2026-09-12 00:30: Bis hierher fuhr der Stack bei JEDEM Start alle Gelenke blind
# (reine Gelenkinterpolation, 4 s, ohne MoveIt) in die Nullstellung. Der Arm stand
# dabei mit dem Greifer IM TRICHTER (angelernter Griffpunkt, J6 136 Grad): der Greifer
# drehte sich im Trichter, stiess an die Wand und wickelte das Kabel um den Greifer.
# Deshalb: der Arm bleibt beim Start, wo er steht. Wer die Nullstellung will, sagt es:
#     NULLSTELLUNG_BEIM_START=1 ./start_mycobot.sh
# Und grundsaetzlich: Stack NICHT neu starten, solange der Arm nahe Trichter/Platte steht.
if [ "${NULLSTELLUNG_BEIM_START:-0}" = "1" ]; then
  echo "[start_mycobot] NULLSTELLUNG_BEIM_START=1: Alle Gelenke werden zum Nullpunkt gesendet..."
  ros2 action send_goal /arm_controller/follow_joint_trajectory \
    control_msgs/action/FollowJointTrajectory "{
    trajectory: {
      joint_names: [joint2_to_joint1, joint3_to_joint2, joint4_to_joint3, joint5_to_joint4, joint6_to_joint5, joint6output_to_joint6],
      points: [{positions: [0.0, 0.0, 0.0, 0.0, 0.0, 0.0], time_from_start: {sec: 4, nanosec: 0}}]
    }
  }" 2>/dev/null && echo "[start_mycobot] Arm → Null OK"
  ros2 action send_goal /gripper_controller/follow_joint_trajectory \
    control_msgs/action/FollowJointTrajectory "{
    trajectory: {
      joint_names: [gripper_controller],
      points: [{positions: [0.0], time_from_start: {sec: 2, nanosec: 0}}]
    }
  }" 2>/dev/null && echo "[start_mycobot] Gripper → Null OK"
else
  echo "[start_mycobot] Arm bleibt stehen, wo er steht (NULLSTELLUNG_BEIM_START nicht gesetzt)."
fi

wait $LAUNCH_PID
