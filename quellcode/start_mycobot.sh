#!/bin/bash
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

if [ "${1:-}" = "sim" ]; then
  export USE_FAKE_HARDWARE="true"
  echo "[start_mycobot] SIM MODE — fake_components/GenericSystem"
else
  export USE_FAKE_HARDWARE="false"
  echo "[start_mycobot] ECHTER ROBOTER — mycobot_hardware/MyCobotHardware @ ${MYCOBOT_PORT:-/dev/ttyTHS1}"
fi

ros2 launch mycobot_moveit_config demo.launch.py use_camera:=true &
LAUNCH_PID=$!

echo "[start_mycobot] Warten bis die Controller bereit sind..."
for i in $(seq 1 30); do
  if ros2 control list_controllers 2>/dev/null | grep -q "arm_controller.*active"; then
    break
  fi
  sleep 1
done

echo "[start_mycobot] Alle Gelenke werden zum Nullpunkt gesendet..."
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

wait $LAUNCH_PID
