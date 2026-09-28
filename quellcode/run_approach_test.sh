#!/bin/bash
# =====================================================================
#  run_approach_test.sh — Wellen-ANNAEHERUNGSTEST (nur pre-grasp, KEIN Abstieg).
#  Identische Orchestrierung wie run_pick_test.sh; einziger Unterschied: statt pick_place
#  läuft approach_target → der Roboter WARTET APPROACH_HEIGHT UEBER der Welle,
#  Greifer 90° senkrecht (top-down), Greifer offen. Steht bis Ctrl+C.
#
#  Zweck: per AUGE prüfen, ob der Greifer genau über der Welle + genau 90° senkrecht ist.
#
#  Verwendung:
#    cd ~/ros2_ws && ./run_approach_test.sh            # 5 cm darüber (Standard)
#    APPROACH_HEIGHT=0.10 ./run_approach_test.sh       # 10 cm darüber
#    SIM=1 ./run_approach_test.sh                       # Simulation ohne Roboter
#
#  Bei Position PRUEFEN; danach IM TERMINAL Ctrl+C → alles wird aufgeräumt.
#  Logs: ~/ros2_ws/logs/approach_<zeit>/{stack,detector,approach}.log
# =====================================================================
set -u

# ---- Einstellbare Parameter ------------------------------------------
APPROACH_HEIGHT="${APPROACH_HEIGHT:-0.05}"   # wie viele m ueber der Welle er stehen wird
SIM="${SIM:-0}"
VIEW="${VIEW:-1}"
DETECTOR_WAIT="${DETECTOR_WAIT:-120}"
CTRL_WAIT="${CTRL_WAIT:-60}"

export LC_ALL=C
export LC_NUMERIC=C
export LANG=C

WS="$HOME/ros2_ws"
STAMP="$(date +%Y%m%d_%H%M%S)"
LOGDIR="$WS/logs/approach_$STAMP"
mkdir -p "$LOGDIR"

STACK_LOG="$LOGDIR/stack.log"
DET_LOG="$LOGDIR/detector.log"
APP_LOG="$LOGDIR/approach.log"

STACK_PID=""
DET_PID=""
VIEW_PID=""

log()  { echo -e "\033[1;36m[run_approach]\033[0m $*"; }
warn() { echo -e "\033[1;33m[run_approach]\033[0m $*"; }
err()  { echo -e "\033[1;31m[run_approach]\033[0m $*" >&2; }

kill_group() {
  local pid="$1" name="$2"
  [ -z "$pid" ] && return 0
  if kill -0 "$pid" 2>/dev/null; then
    log "$name wird beendet (pgid=$pid)..."
    kill -TERM -- "-$pid" 2>/dev/null
    for _ in $(seq 1 10); do kill -0 "$pid" 2>/dev/null || break; sleep 0.5; done
    kill -0 "$pid" 2>/dev/null && { warn "$name erzwungen (KILL)"; kill -KILL -- "-$pid" 2>/dev/null; }
  fi
}

sweep_ros_ghosts() {
  pkill -f "view_overlay.py"          2>/dev/null
  pkill -f "wellen_detektor"            2>/dev/null
  pkill -f "approach_target"          2>/dev/null
  pkill -f "approach_target.launch"   2>/dev/null
  pkill -f "pick_place_cartesian"     2>/dev/null
  pkill -f "demo.launch.py"           2>/dev/null
  pkill -f "move_group"               2>/dev/null
  pkill -f "ros2_control_node"        2>/dev/null
  pkill -f "mycobot_bridge.py"        2>/dev/null
  pkill -f "realsense2_camera"        2>/dev/null
  pkill -f "robot_state_publisher"    2>/dev/null
  sleep 2
  pkill -9 -f "mycobot_bridge.py"     2>/dev/null
  pkill -9 -f "ros2_control_node"     2>/dev/null
  pkill -9 -f "move_group"            2>/dev/null
  pkill -9 -f "realsense2_camera"     2>/dev/null
  rm -f /tmp/mycobot_bridge.sock /tmp/mycobot_bridge.sock.cmd 2>/dev/null
}

cleanup() {
  echo
  log "AUFRAEUMEN — Hintergrund-nodes werden beendet..."
  kill_group "$VIEW_PID"  "overlay-viewer"
  kill_group "$DET_PID"   "detector"
  kill_group "$STACK_PID" "stack"
  sweep_ros_ghosts
  if fuser /dev/ttyTHS1 >/dev/null 2>&1; then
    warn "/dev/ttyTHS1 NOCH belegt — verbleibenden Halter erzwungen toeten"
    fuser -k -9 /dev/ttyTHS1 2>/dev/null
  fi
  log "Logs: $LOGDIR"
  log "  lesen:  tail -n 80 $APP_LOG"
}
trap cleanup EXIT INT TERM

# ---- 1) Systemvorbereitung -------------------------------------------
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
if fuser /dev/ttyTHS1 >/dev/null 2>&1; then
  warn "/dev/ttyTHS1 vom vorherigen Lauf belegt — wird erzwungen freigegeben"
  fuser -k -9 /dev/ttyTHS1 2>/dev/null; sleep 1
fi

set +u
source /opt/ros/galactic/setup.bash
source "$WS/install/setup.bash"
set -u
cd "$WS" || { err "ros2_ws nicht vorhanden"; exit 1; }

if [ "$SIM" = "1" ]; then
  export USE_FAKE_HARDWARE="true"
  warn "SIM MODE — fake_components (kein Roboter)"
else
  export USE_FAKE_HARDWARE="false"
  log "ECHTER Roboter @ ${MYCOBOT_PORT:-/dev/ttyTHS1}"
fi

# ---- 2) Stack starten (RViz AUS) -------------------------------------
log "Stack wird gestartet (use_camera:=true use_rviz:=false) → $STACK_LOG"
setsid bash -c "exec ros2 launch mycobot_moveit_config demo.launch.py \
  use_camera:=true use_rviz:=false" >"$STACK_LOG" 2>&1 &
STACK_PID=$!

# ---- 3) Controller active ---------------------------------------------
log "Auf Controller warten (max ${CTRL_WAIT}s)..."
ok=0
for i in $(seq 1 "$CTRL_WAIT"); do
  if ! kill -0 "$STACK_PID" 2>/dev/null; then
    err "Stack zu frueh gestorben! Letzte Zeilen:"; tail -n 25 "$STACK_LOG"; exit 1
  fi
  if ros2 control list_controllers 2>/dev/null | grep -q "arm_controller.*active"; then
    ok=1; log "Controller AKTIV (${i}s)"; break
  fi
  sleep 1
done
[ "$ok" = "1" ] || { err "Controller wurde nicht aktiv innerhalb ${CTRL_WAIT}s. Siehe $STACK_LOG"; exit 1; }

# ---- 4) Detector starten ----------------------------------------------
log "wellen_detektor wird gestartet → $DET_LOG"
setsid bash -c "exec ros2 run wellenerkennung wellen_detektor" >"$DET_LOG" 2>&1 &
DET_PID=$!

log "Auf Detector-Bereitschaft warten: 'Modell bereit' (max ${DETECTOR_WAIT}s)..."
ok=0
for i in $(seq 1 "$DETECTOR_WAIT"); do
  kill -0 "$DET_PID" 2>/dev/null || { err "Detector zu frueh gestorben!"; tail -n 25 "$DET_LOG"; exit 1; }
  grep -q "Modell bereit" "$DET_LOG" 2>/dev/null && { ok=1; log "Modell geladen (${i}s)"; break; }
  sleep 1
done
[ "$ok" = "1" ] || { err "Modell nicht geladen innerhalb ${DETECTOR_WAIT}s. Siehe $DET_LOG"; exit 1; }

# ---- 4b) Live-Overlay-Fenster ----------------------------------------
if [ "$VIEW" = "1" ] && [ -n "${DISPLAY:-}" ]; then
  log "Overlay-Viewer wird geoeffnet (DISPLAY=$DISPLAY) — im Fenster 'q'/ESC = schliessen"
  setsid bash -c "exec python3 '$WS/view_overlay.py'" >"$LOGDIR/viewer.log" 2>&1 &
  VIEW_PID=$!
elif [ "$VIEW" = "1" ]; then
  warn "Kein DISPLAY — Overlay-Fenster uebersprungen."
fi

log "Auf erstes Wellen-Ziel ('ZIEL') warten (max 60s)..."
ok=0
for i in $(seq 1 60); do
  kill -0 "$DET_PID" 2>/dev/null || { err "Detector gestorben!"; tail -n 25 "$DET_LOG"; exit 1; }
  if grep -q "ZIEL" "$DET_LOG" 2>/dev/null; then
    ok=1; log "Welle gefunden → $(grep 'ZIEL' "$DET_LOG" | tail -n1)"; break
  fi
  [ $((i % 10)) -eq 0 ] && warn "  ...noch kein Ziel (${i}s). Letzte: $(tail -n1 "$DET_LOG")"
  sleep 1
done
[ "$ok" = "1" ] || warn "Innerhalb 60s keine Welle — approach wird trotzdem versucht (hat eigenen 30s-Timeout)."

# ---- 5) approach_target AUSFUEHREN (Roboter fährt zur Position, WARTET dort) --
log "=============================================="
log " ANNAEHERUNG wird ausgefuehrt — ${APPROACH_HEIGHT}m (=$(awk "BEGIN{printf \"%.0f\", $APPROACH_HEIGHT*100}")cm) ueber der Welle, 90° senkrecht"
log "  Wenn der Roboter die Position erreicht, WARTET er DORT. Pruefen; danach Ctrl+C."
log "  log: $APP_LOG"
log "=============================================="
export APPROACH_HEIGHT
ros2 launch mycobot_demo approach_target.launch.py 2>&1 | tee "$APP_LOG"

# Hierher kommt man nur bei Ctrl+C / wenn der node stirbt → cleanup trap läuft.
