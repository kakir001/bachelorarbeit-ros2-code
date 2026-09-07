#!/bin/bash
# =====================================================================
#  view_camera.sh — NUR Kamerabild + Roboter-Reach-RINGE.
#
#  Zweck: ohne pick, nur das D435i-Overlay ansehen — YOLO-Erkennung + gewählte
#  Schraube + Roboter-Basis-Reach-Kreise (grün/amber/rot). Um per Auge zu prüfen,
#  ob die Schrauben im grünen Band liegen.
#
#  RUEHRT DEN ECHTEN ROBOTER NICHT AN: use_fake_hardware:=true → TF (Kamera→robot_base) kommt aus dem URDF
#  (eye-to-hand, statisch), Port/Servo werden nicht genutzt, KEIN Servo-drop-Risiko.
#
#  Verwendung:  cd ~/ros2_ws && ./view_camera.sh        (Ctrl+C = beenden)
#             CONF=0.5 ./view_camera.sh               (Schwelle ändern)
# =====================================================================
set -u
export LC_ALL=C LC_NUMERIC=C LANG=C PYTHONUNBUFFERED=1

WS="$HOME/ros2_ws"
STAMP="$(date +%Y%m%d_%H%M%S)"
LOGDIR="$WS/logs/view_camera_$STAMP"; mkdir -p "$LOGDIR"
STACK_LOG="$LOGDIR/stack.log"; CAM_LOG="$LOGDIR/camera.log"
DET_LOG="$LOGDIR/detector.log"; CAMV_LOG="$LOGDIR/cam_viewer.log"
CONF="${CONF:-0.5}"
CAM_WAIT="${CAM_WAIT:-40}"; DETECTOR_WAIT="${DETECTOR_WAIT:-120}"

STACK_PID=""; CAM_PID=""; DET_PID=""; CAMV_PID=""

log()  { echo -e "\033[1;36m[view]\033[0m $*"; }
warn() { echo -e "\033[1;33m[view]\033[0m $*"; }
err()  { echo -e "\033[1;31m[view]\033[0m $*" >&2; }

kill_group() {
  local pid="$1" name="$2"; [ -z "$pid" ] && return 0
  if kill -0 "$pid" 2>/dev/null; then
    log "$name wird beendet (pgid=$pid)..."
    kill -TERM -- "-$pid" 2>/dev/null
    for _ in $(seq 1 10); do kill -0 "$pid" 2>/dev/null || break; sleep 0.5; done
    kill -0 "$pid" 2>/dev/null && { warn "$name erzwungen (KILL)"; kill -KILL -- "-$pid" 2>/dev/null; }
  fi
}

cleanup() {
  [ "${_CLEANED:-0}" = "1" ] && return 0
  _CLEANED=1; trap - EXIT INT TERM; echo
  log "AUFRAEUMEN — nodes werden beendet..."
  kill_group "$CAMV_PID" "cam_viewer"
  kill_group "$DET_PID"  "detector"
  kill_group "$CAM_PID"  "camera"
  kill_group "$STACK_PID" "stack"
  log "Fertig. (fake hardware — echter Roboter/Port wurde nicht genutzt.)"
}
trap cleanup EXIT INT TERM

# ROS setup.bash nutzt unbound-Variablen → unter set -u platzt es; das sourcen umschließen.
set +u
source /opt/ros/galactic/setup.bash
source "$WS/install/setup.bash"
set -u

# --- Stack (NUR für TF): fake hardware, kein RViz, Kamera wird separat gestartet ---
log "Stack (fake hardware, fuer TF) → $STACK_LOG"
setsid bash -c "exec ros2 launch mycobot_moveit_config demo.launch.py \
  use_camera:=false use_rviz:=false use_fake_hardware:=true" >"$STACK_LOG" 2>&1 &
STACK_PID=$!
for i in $(seq 1 30); do
  kill -0 "$STACK_PID" 2>/dev/null || { err "Stack zu frueh gestorben!"; tail -n 25 "$STACK_LOG"; exit 1; }
  ros2 topic list 2>/dev/null | grep -q "/joint_states" && { log "Stack/TF bereit (${i}s)"; break; }
  sleep 1
done

# --- Kamera (D435i, niedrige Auflösung — auf Jetson erforderlich) ---
log "Kamera (rs_launch 424x240x15) → $CAM_LOG"
setsid bash -c "exec ros2 launch realsense2_camera rs_launch.py \
  pointcloud.enable:=true align_depth.enable:=true \
  depth_module.profile:=424x240x15 rgb_camera.profile:=424x240x15" >"$CAM_LOG" 2>&1 &
CAM_PID=$!
ok=0
for i in $(seq 1 "$CAM_WAIT"); do
  kill -0 "$CAM_PID" 2>/dev/null || { err "Kamera zu frueh gestorben!"; tail -n 25 "$CAM_LOG"; exit 1; }
  [ "$(ros2 topic info /camera/color/image_raw 2>/dev/null | grep -c 'Publisher count: [1-9]')" -ge 1 ] && { ok=1; log "Kamera sendet (${i}s)"; break; }
  sleep 1
done
[ "$ok" = "1" ] || { err "Kamera sendet nicht innerhalb ${CAM_WAIT}s (evtl. USB aus-/einstecken noetig)"; tail -n 20 "$CAM_LOG"; exit 1; }

# --- Detector (zeichnet Overlay + Reach-Ringe, sendet an /vida/overlay) ---
log "vida_detector (conf=$CONF) → $DET_LOG"
setsid bash -c "exec ros2 run vida_vision vida_detector --ros-args -p conf:=$CONF" >"$DET_LOG" 2>&1 &
DET_PID=$!
ok=0
for i in $(seq 1 "$DETECTOR_WAIT"); do
  kill -0 "$DET_PID" 2>/dev/null || { err "Detector zu frueh gestorben!"; tail -n 25 "$DET_LOG"; exit 1; }
  grep -q "Modell bereit" "$DET_LOG" 2>/dev/null && { ok=1; log "Modell geladen (${i}s)"; break; }
  sleep 1
done
[ "$ok" = "1" ] || { err "Detector nicht bereit innerhalb ${DETECTOR_WAIT}s"; tail -n 20 "$DET_LOG"; exit 1; }

# --- cam_viewer: öffnet das /vida/overlay-Fenster (hier sind die Ringe sichtbar) ---
log "cam_viewer (Overlay-Fenster) → $CAMV_LOG"
setsid bash -c "exec python3 '$WS/cam_viewer.py'" >"$CAMV_LOG" 2>&1 &
CAMV_PID=$!

log "BEREIT — Kamerafenster offen. Gruen/amber Kreis = Roboter-Reach-Grenze."
log "Schraube innerhalb des GRUENEN Kreises = ideale pick-Zone. Ctrl+C = beenden."
# Warten, solange die nodes leben (wenn einer stirbt → raus → cleanup)
while kill -0 "$CAMV_PID" 2>/dev/null && kill -0 "$DET_PID" 2>/dev/null \
   && kill -0 "$CAM_PID" 2>/dev/null && kill -0 "$STACK_PID" 2>/dev/null; do
  sleep 1
done
warn "Ein node wurde unerwartet beendet — wird aufgeraeumt."
