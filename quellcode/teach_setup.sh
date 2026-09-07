#!/bin/bash
# =====================================================================
#  teach_setup.sh — Lernumgebung für das Anlernen von 5 Boxen zur Farb-SORTIERUNG (ISOLIERT, KLICK-UND-GO).
#
#  Teach-Umgebung mit einem Befehl:
#     1) Echt-Hardware-Stack (move_group + control + bridge)
#     2) goto_clicked_point  (RViz "Publish Point" → Roboter fährt 5cm über den Punkt)
#     3) LEICHTES rviz2 (teach_click.rviz) — KEIN MotionPlanning → stürzt nicht ab.
#  Kein Detector/Kamera/pick_tilt → es geht kein falsches Kommando an den Arm.
#
#  VERWENDUNG:  cd ~/ros2_ws && ./teach_setup.sh
#  ANLERNEN:
#     - In RViz mit dem Werkzeug "Publish Point" auf den Ort der Box klicken → Roboter fährt hin.
#     - Bei Bedarf erneut klicken, anpassen. Wenn genau über der Box, speichern mit:
#         python3 teach_boxes.py --once <farbe>     (farbe: sari/beyaz/siyah/yesil/kirmizi)
#  BEENDEN: diesen Prozess stoppen → Roboter wird zu 0 geparkt, alles wird beendet.
# =====================================================================
set -u
WS="$HOME/ros2_ws"
STAMP="$(date +%Y%m%d_%H%M%S)"
LOGDIR="$WS/logs/teach_$STAMP"; mkdir -p "$LOGDIR"
STACK_LOG="$LOGDIR/stack.log"; RVIZ_LOG="$LOGDIR/rviz.log"; CLICK_LOG="$LOGDIR/click2go.log"
RVIZ_CFG="$WS/teach_click.rviz"
ARM_JOINTS="[joint2_to_joint1, joint3_to_joint2, joint4_to_joint3, joint5_to_joint4, joint6_to_joint5, joint6output_to_joint6]"

log()  { echo -e "\033[1;36m[teach]\033[0m $*"; }
warn() { echo -e "\033[1;33m[teach] WARNUNG:\033[0m $*"; }
err()  { echo -e "\033[1;31m[teach] FEHLER:\033[0m $*"; }

export LC_ALL=C LC_NUMERIC=C LANG=C PYTHONUNBUFFERED=1
set +u
source /opt/ros/galactic/setup.bash
source "$WS/install/setup.bash"
set -u
export USE_FAKE_HARDWARE="${USE_FAKE_HARDWARE:-false}"
export MYCOBOT_PORT="${MYCOBOT_PORT:-/dev/ttyTHS1}"
export DISPLAY="${DISPLAY:-:0}"
CTRL_WAIT="${CTRL_WAIT:-60}"

STACK_PID=""; RVIZ_PID=""; CLICK_PID=""
cleanup() {
  log "AUFRAEUMEN — Roboter wird zu 0 geparkt, nodes werden beendet..."
  if ros2 action list 2>/dev/null | grep -q "/arm_controller/follow_joint_trajectory"; then
    timeout 20 ros2 action send_goal /arm_controller/follow_joint_trajectory \
      control_msgs/action/FollowJointTrajectory \
      "{trajectory: {joint_names: $ARM_JOINTS, points: [{positions: [0.0,0.0,0.0,0.0,0.0,0.0], time_from_start: {sec: 5}}]}}" \
      >/dev/null 2>&1 && log "Roboter bei Punkt 0." || warn "Konnte nicht zu 0 geparkt werden — Arm von Hand kontrollieren!"
  fi
  [ -n "$RVIZ_PID" ]  && kill -- -"$RVIZ_PID"  2>/dev/null
  [ -n "$CLICK_PID" ] && kill -- -"$CLICK_PID" 2>/dev/null
  [ -n "$STACK_PID" ] && kill -- -"$STACK_PID" 2>/dev/null
  pkill -f "rviz2 -d $RVIZ_CFG"          2>/dev/null
  pkill -f "goto_clicked_point"          2>/dev/null
  pkill -f "demo.launch.py"              2>/dev/null
  sleep 1
  pkill -9 -f "ros2_control_node"        2>/dev/null
  pkill -9 -f "mycobot_bridge.py"        2>/dev/null
  rm -f /tmp/mycobot_bridge.sock /tmp/mycobot_bridge.sock.cmd 2>/dev/null
  log "Fertig. Eintraege: cat $WS/box_map.yaml"
}
trap cleanup EXIT INT TERM

if pgrep -f "ros2_control_node" >/dev/null 2>&1; then
  err "Es laeuft bereits ein ros2_control_node. Erst diesen beenden, dann erneut versuchen."
  exit 1
fi

# --- 1) STACK ---
log "########## STACK (echte Hardware, ohne Kamera, ohne RViz) → $STACK_LOG ##########"
[ "$USE_FAKE_HARDWARE" = "true" ] && warn "FAKE HARDWARE — kein echter Roboter (teach sinnlos!)"
setsid bash -c "exec ros2 launch mycobot_moveit_config demo.launch.py \
  use_camera:=false use_rviz:=false use_fake_hardware:=$USE_FAKE_HARDWARE" >"$STACK_LOG" 2>&1 &
STACK_PID=$!

log "Auf Controller warten (max ${CTRL_WAIT}s)..."
ok=0
for i in $(seq 1 "$CTRL_WAIT"); do
  kill -0 "$STACK_PID" 2>/dev/null || { err "Stack zu frueh gestorben!"; tail -n 25 "$STACK_LOG"; exit 1; }
  ros2 control list_controllers 2>/dev/null | grep -q "arm_controller.*active" && { ok=1; log "Controller AKTIV (${i}s)"; break; }
  sleep 1
done
[ "$ok" = "1" ] || { err "Controller wurde nicht aktiv. Siehe $STACK_LOG"; exit 1; }

log "Roboter wird zu Punkt 0 gesendet..."
timeout 20 ros2 action send_goal /arm_controller/follow_joint_trajectory \
  control_msgs/action/FollowJointTrajectory \
  "{trajectory: {joint_names: $ARM_JOINTS, points: [{positions: [0.0,0.0,0.0,0.0,0.0,0.0], time_from_start: {sec: 5}}]}}" \
  >/dev/null 2>&1 && log "Roboter bei Punkt 0." || warn "Konnte nicht zu 0 gesendet werden (weiter)."

# --- 2) KLICK-UND-GO node ---
log "goto_clicked_point (RViz Publish Point → Roboter) → $CLICK_LOG"
setsid bash -c "exec ros2 launch mycobot_demo goto_clicked_point.launch.py" >"$CLICK_LOG" 2>&1 &
CLICK_PID=$!
sleep 4

# --- 3) LEICHTES RViz (KEIN MotionPlanning) ---
if [ -n "${DISPLAY:-}" ]; then
  log "RViz (leichte teach config, stuerzt nicht ab) → $RVIZ_LOG"
  setsid bash -c "exec ros2 run rviz2 rviz2 -d $RVIZ_CFG" >"$RVIZ_LOG" 2>&1 &
  RVIZ_PID=$!
  sleep 6
else
  warn "Kein DISPLAY — RViz konnte nicht geoeffnet werden."
fi

log "############################################################"
log " ANLERNEN BEREIT. In RViz mit 'Publish Point' auf die Box klicken → Roboter faehrt hin."
log " Zum Speichern (waehrend der Arm ueber der Box steht):"
log "    python3 $WS/teach_boxes.py --once <farbe>"
log " Vorhandene Eintraege: python3 $WS/teach_boxes.py --list"
log " Beenden: diesen Prozess stoppen (Roboter wird zu 0 geparkt)."
log "############################################################"

wait "$STACK_PID"
