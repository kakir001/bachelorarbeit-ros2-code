#!/bin/bash
# =====================================================================
#  run_place_calib.sh — PLACE (Schraube senkrecht einsetzen) XY-Offset-KALIBRIERUNG.
#
#  Zweck (Sitzung 35→36): click_place_moveit.py läuft live, aber die Schraube
#  fällt "hinter" das Loch (TCP ~2.5cm + Seitengriff-Geometrie → XY-Offset).
#  Dieses Skript baut den Kalibrier-Stack auf; der Benutzer richtet im Werkzeug per nudge (8/2/4/6,+/-)
#  die Schraube über das Loch aus, der funktionierende Offset wird gefunden → als PERMANENTER
#  Standard (DEFAULT_OFF) ins Werkzeug eingebettet.
#
#  ARCHITEKTUR (DEVLOG Sitzung 35): Stack FAKE hardware → ros2_control RUEHRT DEN PORT NICHT AN;
#  den Port (/dev/ttyTHS1) öffnet click_place_moveit.py per pymycobot.
#  move_group plant einen kollisionsfreien WEG, RViz (rviz_preview.rviz, Loop Anim)
#  zeigt die Vorschau, mit ENTER/E folgt pymycobot dem Plan und steuert den echten Roboter.
#
#  Komponenten:  stack(fake,ohne Kamera,ohne rviz) + RViz(preview) + Kamera
#               + click_place_moveit.py (FOREGROUND, öffnet DEN Port)
#
#  Verwendung:  cd ~/ros2_ws && ./run_place_calib.sh
#     COLOR_PROFILE=848x480x15 ./run_place_calib.sh   # wenn die Kamera zu stark auslastet
#  Logs: ~/ros2_ws/logs/place_calib_<zeit>/
# =====================================================================
set -u

# Kamera-Profile — in Sitzung 35 lief Farbe 1280x720 (detector aus, RAM reichlich).
# Bei "Out of frame resources" mit COLOR_PROFILE=848x480x15 erneut versuchen.
COLOR_PROFILE="${COLOR_PROFILE:-1280x720x15}"
DEPTH_PROFILE="${DEPTH_PROFILE:-848x480x15}"
CTRL_WAIT="${CTRL_WAIT:-60}"
CAM_WAIT="${CAM_WAIT:-40}"
MG_WAIT="${MG_WAIT:-40}"
export LC_ALL=C LC_NUMERIC=C LANG=C PYTHONUNBUFFERED=1
# KRITISCH: demo.launch.py liest die hardware-Auswahl aus der UMGEBUNGSVARIABLE USE_FAKE_HARDWARE
# (das launch-Argument 'use_fake_hardware:=' wird IGNORIERT). Ohne fake öffnet ros2_control
# MyCobotHardware → mycobot_bridge.py den Port → mit dem tool DOPPELTER ZUGRIFF → serial platzt.
export USE_FAKE_HARDWARE=true

WS="$HOME/ros2_ws"
STAMP="$(date +%Y%m%d_%H%M%S)"
LOGDIR="$WS/logs/place_calib_$STAMP"
mkdir -p "$LOGDIR"
RVIZ_CFG="$WS/rviz_preview.rviz"
TOOL="$WS/src/mycobot_calibration/scripts/click_place_moveit.py"

STACK_LOG="$LOGDIR/stack.log"; RVIZ_LOG="$LOGDIR/rviz.log"
CAM_LOG="$LOGDIR/camera.log"; TOOL_LOG="$LOGDIR/tool.log"
ESTOP_LOG="$LOGDIR/estop_button.log"
STACK_PID=""; RVIZ_PID=""; CAM_PID=""; ESTOP_PID=""

log()  { echo -e "\033[1;36m[place]\033[0m $*"; }
warn() { echo -e "\033[1;33m[place]\033[0m $*"; }
err()  { echo -e "\033[1;31m[place]\033[0m $*" >&2; }

kill_group() {
  local pid="$1" name="$2"; [ -z "$pid" ] && return 0
  if kill -0 "$pid" 2>/dev/null; then
    log "$name wird beendet (pgid=$pid)..."
    kill -TERM -- "-$pid" 2>/dev/null
    for _ in $(seq 1 10); do kill -0 "$pid" 2>/dev/null || break; sleep 0.5; done
    kill -0 "$pid" 2>/dev/null && { warn "$name erzwungen (KILL)"; kill -KILL -- "-$pid" 2>/dev/null; }
  fi
}

sweep_ros_ghosts() {
  pkill -f "estop_button"       2>/dev/null
  pkill -f "estop_relay"        2>/dev/null
  pkill -f "click_place_moveit" 2>/dev/null
  pkill -f "demo.launch.py"       2>/dev/null
  pkill -f "moveit_rviz.launch"   2>/dev/null
  pkill -f "rviz2"                2>/dev/null
  pkill -f "move_group"           2>/dev/null
  pkill -f "ros2_control_node"    2>/dev/null
  pkill -f "mycobot_bridge.py"    2>/dev/null
  pkill -f "realsense2_camera"    2>/dev/null
  pkill -f "robot_state_publisher" 2>/dev/null
  sleep 2
  pkill -9 -f "ros2_control_node" 2>/dev/null
  pkill -9 -f "move_group"        2>/dev/null
  pkill -9 -f "realsense2_camera" 2>/dev/null
  pkill -9 -f "rviz2"             2>/dev/null
  rm -f /tmp/mycobot_bridge.sock /tmp/mycobot_bridge.sock.cmd 2>/dev/null
}

park_zero_pymycobot() {
  local port="${MYCOBOT_PORT:-/dev/ttyTHS1}"
  for _ in $(seq 1 10); do fuser "$port" >/dev/null 2>&1 || break; sleep 0.5; done
  if fuser "$port" >/dev/null 2>&1; then
    warn "pymycobot Park: $port noch belegt — wird uebersprungen (Arm von Hand kontrollieren!)"; return 0
  fi
  log "GARANTIERTER Park zu 0 mit pymycobot (Servos unter Drehmoment, Arm faellt nicht)..."
  MYCOBOT_PORT="$port" timeout 35 python3 - <<'PYEOF'
import os, time
try:
    from pymycobot import MyCobot280
    mc = MyCobot280(os.environ.get("MYCOBOT_PORT", "/dev/ttyTHS1"), 1000000)
    time.sleep(0.6)
    try: mc.power_on(); time.sleep(0.4)
    except Exception: pass
    mc.send_radians([0.0]*6, 25)
    for _ in range(14):
        time.sleep(1.0)
        if mc.is_moving() == 0: break
    try:
        mc.set_gripper_value(100, 50, 1)   # beim Park Finger OFFEN (gehaltene Schraube loslassen)
        time.sleep(1.0)
    except Exception as ge: print("Greifer-Oeffnen Warnung:", ge)
    a = mc.get_radians()
    if a and len(a) == 6:
        print("groesster |Winkel| nach Park = %.3f rad" % max(abs(x) for x in a))
except Exception as e:
    print("pymycobot Park FEHLER:", e)
PYEOF
  log "pymycobot Park beendet — Roboter bei 0."
}

cleanup() {
  [ "${_CLEANED:-0}" = "1" ] && return 0
  _CLEANED=1; trap - EXIT INT TERM; echo
  log "AUFRAEUMEN — nodes werden beendet..."
  kill_group "$CAM_PID"   "camera"
  kill_group "$RVIZ_PID"  "rviz"
  kill_group "$STACK_PID" "stack"
  sweep_ros_ghosts
  fuser /dev/ttyTHS1 >/dev/null 2>&1 && fuser -k -9 /dev/ttyTHS1 2>/dev/null
  park_zero_pymycobot
  # NOT-AUS-Fenster: sweep hat es meist schon beendet; hier Rest-Aufräumen.
  kill_group "$ESTOP_PID" "estop_button"
  log "Logs: $LOGDIR"
}
trap cleanup EXIT INT TERM

[ -f "$TOOL" ] || { err "Werkzeug nicht vorhanden: $TOOL"; exit 1; }

# ---- Systemvorbereitung (rmem/gpu/fan/usb-power) ----
log "Systemvorbereitung (sudo evtl. noetig)..."
sudo sysctl -w net.core.rmem_max=8388608 net.core.wmem_max=8388608 \
             net.core.rmem_default=8388608 net.core.wmem_default=8388608 >/dev/null 2>&1
echo 0 | sudo tee /sys/devices/57000000.gpu/railgate_enable >/dev/null 2>&1
sudo sh -c 'echo 255 > /sys/devices/pwm-fan/target_pwm' 2>/dev/null
for dev in /sys/bus/usb/devices/*/idVendor; do
  [ "$(cat "$dev" 2>/dev/null)" = "8086" ] && \
    echo on | sudo tee "$(dirname "$dev")/power/control" >/dev/null 2>&1
done
sweep_ros_ghosts
fuser /dev/ttyTHS1 >/dev/null 2>&1 && { warn "Port belegt — wird freigegeben"; fuser -k -9 /dev/ttyTHS1 2>/dev/null; sleep 1; }

set +u
source /opt/ros/galactic/setup.bash
source "$WS/install/setup.bash"
set -u
cd "$WS" || { err "ros2_ws nicht vorhanden"; exit 1; }

# ---- Stack: move_group + FAKE control (RUEHRT DEN PORT NICHT AN), ohne Kamera, ohne rviz ----
log "Stack (move_group+FAKE control, ohne Kamera, KEIN RViz) → $STACK_LOG"
setsid bash -c "exec ros2 launch mycobot_moveit_config demo.launch.py \
  use_camera:=false use_rviz:=false use_fake_hardware:=true" >"$STACK_LOG" 2>&1 &
STACK_PID=$!

log "Auf Controller warten (max ${CTRL_WAIT}s)..."
ok=0
for i in $(seq 1 "$CTRL_WAIT"); do
  kill -0 "$STACK_PID" 2>/dev/null || { err "Stack zu frueh gestorben!"; tail -n 25 "$STACK_LOG"; exit 1; }
  ros2 control list_controllers 2>/dev/null | grep -q "arm_controller.*active" && { ok=1; log "Controller AKTIV (${i}s)"; break; }
  sleep 1
done
[ "$ok" = "1" ] || { err "Controller wurde nicht aktiv. Siehe $STACK_LOG"; exit 1; }

# NOT-AUS-Taster: publiziert /estop (gelatcht). Enforcement übernimmt hier das
# Werkzeug selbst (click_place_moveit.py abonniert /estop und stoppt pymycobot
# sofort) — der Stack ist FAKE, es läuft kein serieller Bridge.
if [ -n "${DISPLAY:-}" ]; then
  log "NOT-AUS-Taster (estop_button) → $ESTOP_LOG"
  setsid bash -c "exec ros2 run mycobot_calibration estop_button" >"$ESTOP_LOG" 2>&1 &
  ESTOP_PID=$!
else
  warn "Kein DISPLAY — NOT-AUS-Fenster nicht moeglich. Terminal-Alternative:"
  warn "  ros2 topic pub --once --qos-durability transient_local /estop std_msgs/msg/Bool \"{data: true}\""
fi

log "Auf move_group (/move_action) warten (max ${MG_WAIT}s)..."
ok=0
for i in $(seq 1 "$MG_WAIT"); do
  ros2 action list 2>/dev/null | grep -q "/move_action" && { ok=1; log "move_group BEREIT (${i}s)"; break; }
  sleep 1
done
[ "$ok" = "1" ] || warn "move_action nicht erschienen — du kannst im Werkzeug trotzdem mit P planen."

# ---- RViz-Vorschau (rviz_preview.rviz, Loop Animation) ----
if [ -n "${DISPLAY:-}" ]; then
  RVIZ_ARG=""; [ -f "$RVIZ_CFG" ] && RVIZ_ARG="rviz_config:=$RVIZ_CFG"
  log "RViz-Vorschau (moveit_rviz.launch.py, $RVIZ_CFG) → $RVIZ_LOG"
  setsid bash -c "exec ros2 launch mycobot_moveit_config moveit_rviz.launch.py $RVIZ_ARG" >"$RVIZ_LOG" 2>&1 &
  RVIZ_PID=$!
else
  warn "Kein DISPLAY — RViz uebersprungen (keine Vorschau moeglich)."
fi

# ---- Kamera (color $COLOR_PROFILE + aligned depth; pointcloud AUS — nicht nötig) ----
log "Kamera (color=$COLOR_PROFILE depth=$DEPTH_PROFILE, aligned) → $CAM_LOG"
setsid bash -c "exec ros2 launch realsense2_camera rs_launch.py \
  align_depth.enable:=true pointcloud.enable:=false \
  depth_module.profile:=$DEPTH_PROFILE rgb_camera.profile:=$COLOR_PROFILE" >"$CAM_LOG" 2>&1 &
CAM_PID=$!

log "Auf Kamera-Topic warten: /camera/color/image_raw (max ${CAM_WAIT}s)..."
ok=0
for i in $(seq 1 "$CAM_WAIT"); do
  kill -0 "$CAM_PID" 2>/dev/null || { err "Kamera zu frueh gestorben!"; tail -n 30 "$CAM_LOG"; \
    grep -qi "frame resources" "$CAM_LOG" && err ">> 'Out of frame resources' — mit COLOR_PROFILE=848x480x15 erneut versuchen."; exit 1; }
  [ "$(ros2 topic info /camera/color/image_raw 2>/dev/null | grep -c 'Publisher count: [1-9]')" -ge 1 ] && { ok=1; log "Kamera sendet (${i}s)"; break; }
  sleep 1
done
[ "$ok" = "1" ] || { err "Kamera sendet nicht innerhalb ${CAM_WAIT}s. Siehe $CAM_LOG"; exit 1; }
# auch aligned depth soll kommen
for i in $(seq 1 15); do
  [ "$(ros2 topic info /camera/aligned_depth_to_color/image_raw 2>/dev/null | grep -c 'Publisher count: [1-9]')" -ge 1 ] && { log "Aligned depth sendet."; break; }
  sleep 1
done

log "=============================================="
log " KALIBRIER-SCHLEIFE — Werkzeug wird FOREGROUND geoeffnet."
log " 1) Auf das Fixture-Loch (wo die Schraube eingesetzt wird) klicken."
log " 2) Mit O Greifer auf → Schraube horizontal laden → mit C greifen."
log " 3) P = planen (in RViz kollisionsfreier Weg + Schraube SENKRECHT Vorschau)."
log " 4) E = ausfuehren → in der Kamera die Lage der Schraube zum Loch ansehen."
log " 5) Mit 8/2/4/6 (xy 1cm), +/- (z 1cm) NUDGE → jeder nudge plant neu."
log "    Wiederholen, bis die Schraube genau ueber dem Loch ist, den FUNKTIONIERENDEN Offset NOTIEREN."
log "    (im Terminal-Log erscheint die Zeile 'offset x=.. y=.. z=..cm')"
log " 6) Q = beenden. Den funktionierenden Offset mir sagen → ich bette ihn PERMANENT ins Werkzeug ein."
log " Ctrl+C / Q = alles beenden + Roboter zu 0 parken."
log "=============================================="
# Werkzeug FOREGROUND — öffnet DEN Port; wenn der Benutzer Q drückt, kehrt es zurück, dann parkt cleanup.
python3 "$TOOL" 2>&1 | tee "$TOOL_LOG"

log "Werkzeug beendet — wird aufgeraeumt..."
