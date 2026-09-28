#!/bin/bash
# Stack sauber beenden - und NICHTS uebrig lassen.
# 2026-09-12: ros2 launch kam auf SIGINT teils 40 s lang nicht zu Ende (RViz/Bruecke),
# ein Kill des Launch-Prozesses liess dann arbeitsraum_wolke.py (~30 % CPU), arbeitsraum_marker.py
# und die serielle Bruecke als Waisen (ppid 1) zurueck - aus zwei Stacks vier Stueck.
# Ablauf: 1) SIGINT an start_mycobot.sh + ros2 launch, bis 30 s warten
#         2) was dann noch lebt, mit Namen einzeln SIGTERM, dann SIGKILL
#         3) Bericht. Der Arm bleibt stehen (Servos halten, kein release_all_servos).
#     ./stop_mycobot.sh
NAMEN="arbeitsraum_wolke|arbeitsraum_marker|mycobot_bridge|estop_relay|realsense2_camera_node|move_group|ros2_control_node|robot_state_publisher|rviz2|spawner"
me=$$
# Eigene Shell und deren Eltern (z.B. eine Claude-Bash-Huelle, deren Kommandozeile die
# Namen enthaelt!) ausnehmen - sonst killt das Skript den Aufrufer (Exit 144, 2026-09-13).
eltern=$(ps -o ppid= -p $$ | tr -d ' '); grosseltern=$(ps -o ppid= -p "$eltern" 2>/dev/null | tr -d ' ')
lebt() { pgrep -f "$1" | grep -v -x -e "$me" -e "$eltern" -e "${grosseltern:-0}" -e "$PPID" ; }

for p in $(lebt "ros2 launch mycobot_moveit_config") $(lebt "start_mycobot.sh"); do kill -INT "$p" 2>/dev/null; done
for i in $(seq 1 30); do
  [ -z "$(lebt "$NAMEN")" ] && break
  sleep 1
done
rest=$(lebt "$NAMEN")
if [ -n "$rest" ]; then
  echo "[stop_mycobot] nach 30 s noch da -> SIGTERM:"; ps -o pid=,args= -p $(echo $rest | tr ' ' ',') | cut -c1-100 | sed 's/^/    /'
  kill -TERM $rest 2>/dev/null; sleep 4
  rest=$(lebt "$NAMEN")
  [ -n "$rest" ] && { echo "[stop_mycobot] SIGKILL: $rest"; kill -KILL $rest 2>/dev/null; sleep 1; }
fi
for p in $(lebt "ros2 launch mycobot_moveit_config"); do kill -TERM "$p" 2>/dev/null; done
rest=$(lebt "$NAMEN|ros2 launch mycobot_moveit_config")
if [ -z "$rest" ]; then echo "[stop_mycobot] Stack beendet, keine Reste."; else echo "[stop_mycobot] !! noch da: $rest"; exit 1; fi
