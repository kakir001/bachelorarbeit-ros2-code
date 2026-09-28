#!/bin/bash
# Umgebung fuer ein Terminal, in dem tools/*.py gegen den laufenden Stack laufen sollen:
#     source ~/ros2_ws/ros_umgebung.sh
# LC_ALL=C: de_DE-Locale zerlegt Gleitkommazahlen (RViz/MoveIt). CycloneDDS nur Loopback:
# WLAN-Wechsel trennt sonst die Knoten. Ohne Logpuffer, damit Ausgaben sofort erscheinen.
export LC_ALL=C LC_NUMERIC=C LANG=C PYTHONUNBUFFERED=1 RCUTILS_LOGGING_BUFFERED_STREAM=0
export CYCLONEDDS_URI="${CYCLONEDDS_URI:-file://$HOME/ros2_ws/cyclonedds_loopback.xml}"
source /opt/ros/galactic/setup.bash
source "$HOME/ros2_ws/install/setup.bash"
cd "$HOME/ros2_ws"
