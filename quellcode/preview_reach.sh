#!/bin/bash
# =====================================================================
#  preview_reach.sh — NUR BILD-Vorschau (RUEHRT DEN ROBOTER NICHT AN, KEIN IK).
#
#  Zweck: ohne das pick-Skript zu starten sehen, WOHIN man die Welle legen soll.
#  Kamera + statisches TF (vo_rsp) + detector + cam_viewer werden geöffnet; Roboter/move_group/
#  serieller Port (/dev/ttyTHS1) werden GAR NICHT genutzt → der Arm bewegt sich nicht, nichts wird geplant.
#
#  Im Kamerafenster (linkes Panel /welle/overlay):
#    GRUENER Kreis = sichere Senkrecht-Griff-Zone (r < 200mm) → IK löst im ersten Versuch.
#    AMBER Kreis  = kinematisches Senkrecht-Limit (200-260mm) → erreichbar, aber langsam/angestrengt.
#    Neben jeder Welle r=XXX (mm) + Ringfarbe: grün/amber/rot.
#  Lege die Welle so, dass ihr Ring GRUEN ist → belastet das IK nicht.
#
#  Verwendung:  cd ~/ros2_ws && ./preview_reach.sh      (Ctrl+C = beenden)
# =====================================================================
set -u

CONF="${CONF:-0.75}"
export LC_ALL=C LC_NUMERIC=C LANG=C PYTHONUNBUFFERED=1

WS="$HOME/ros2_ws"
STAMP="$(date +%Y%m%d_%H%M%S)"
LOGDIR="$WS/logs/preview_$STAMP"
mkdir -p "$LOGDIR"
CAM_LOG="$LOGDIR/camera.log"; TF_LOG="$LOGDIR/tf.log"
DET_LOG="$LOGDIR/detector.log"; CAMV_LOG="$LOGDIR/cam_viewer.log"
CAM_PID=""; TF_PID=""; DET_PID=""; CAMV_PID=""

log()  { echo -e "\033[1;36m[preview]\033[0m $*"; }
warn() { echo -e "\033[1;33m[preview]\033[0m $*"; }
err()  { echo -e "\033[1;31m[preview]\033[0m $*" >&2; }

kill_group() {
  local pid="$1" name="$2"
  [ -z "$pid" ] && return 0
  if kill -0 "$pid" 2>/dev/null; then
    log "$name wird beendet..."
    kill -TERM -- "-$pid" 2>/dev/null
    for _ in $(seq 1 8); do kill -0 "$pid" 2>/dev/null || break; sleep 0.5; done
    kill -0 "$pid" 2>/dev/null && kill -KILL -- "-$pid" 2>/dev/null
  fi
}

cleanup() {
  [ "${_CLEANED:-0}" = "1" ] && return 0
  _CLEANED=1; trap - EXIT INT TERM; echo
  log "AUFRAEUMEN..."
  kill_group "$CAMV_PID" "cam_viewer"
  kill_group "$DET_PID"  "detector"
  kill_group "$TF_PID"   "vo_rsp(TF)"
  kill_group "$CAM_PID"  "kamera"
  pkill -f "cam_viewer.py" 2>/dev/null
  pkill -f "wellen_detektor" 2>/dev/null
  pkill -f "robot_state_publisher" 2>/dev/null
  pkill -f "realsense2_camera" 2>/dev/null
  sleep 1
  pkill -9 -f "realsense2_camera" 2>/dev/null
  log "Logs: $LOGDIR"
}
trap cleanup EXIT INT TERM

# ---- D435i autosuspend-Ausgleich (damit die Kamera senden kann) ----
for dev in /sys/bus/usb/devices/*/idVendor; do
  [ "$(cat "$dev" 2>/dev/null)" = "8086" ] && \
    echo on | sudo tee "$(dirname "$dev")/power/control" >/dev/null 2>&1
done

set +u
source /opt/ros/galactic/setup.bash
source "$WS/install/setup.bash"
set -u
cd "$WS" || { err "ros2_ws nicht vorhanden"; exit 1; }

log "DER ROBOTER WIRD NICHT ANGERUEHRT — nur Kamera + TF + detector (KEIN IK/Bewegung)."

# ---- 1) Kamera (D435i, 424x240x15, aligned depth) ----
log "Kamera wird gestartet → $CAM_LOG"
setsid bash -c "exec ros2 launch realsense2_camera rs_launch.py \
  align_depth.enable:=true depth_module.profile:=424x240x15 \
  rgb_camera.profile:=424x240x15" >"$CAM_LOG" 2>&1 &
CAM_PID=$!

# ---- 2) Statisches TF (vo_rsp: Kamera↔robot_base, ohne Roboter) ----
log "Statisches TF (vo_rsp) wird gestartet → $TF_LOG"
setsid bash -c "exec ros2 launch '$WS/vo_rsp.launch.py'" >"$TF_LOG" 2>&1 &
TF_PID=$!

# ---- 3) Detector (YOLO + Reach-Kreis-Overlay) ----
log "wellen_detektor wird gestartet → $DET_LOG"
setsid bash -c "exec ros2 run wellenerkennung wellen_detektor --ros-args -p conf:=$CONF" >"$DET_LOG" 2>&1 &
DET_PID=$!

log "Auf Modell warten ('Modell bereit', max 120s)..."
ok=0
for i in $(seq 1 120); do
  kill -0 "$DET_PID" 2>/dev/null || { err "Detector zu frueh gestorben:"; tail -n 25 "$DET_LOG"; exit 1; }
  grep -q "Modell bereit" "$DET_LOG" 2>/dev/null && { ok=1; log "Modell geladen (${i}s)"; break; }
  sleep 1
done
[ "$ok" = "1" ] || { err "Modell nicht geladen. Siehe $DET_LOG"; exit 1; }

# ---- 4) Kamerafenster ----
if [ -n "${DISPLAY:-}" ]; then
  log "cam_viewer wird geoeffnet (linkes Panel = Reach-Kreise + Erkennung) → $CAMV_LOG"
  setsid bash -c "exec python3 '$WS/cam_viewer.py'" >"$CAMV_LOG" 2>&1 &
  CAMV_PID=$!
else
  warn "Kein DISPLAY — Fenster konnte nicht geoeffnet werden. Overlay-Topic: /welle/overlay"
fi

log "=============================================="
log " VORSCHAU BEREIT. Die Welle naeher zum Roboter bringen, bis ihr Ring GRUEN ist."
log " GRUEN = belastet IK nicht | AMBER = angestrengt | ROT = nicht erreichbar"
log " Beenden: Ctrl+C"
log "=============================================="

# Detector-Log live anzeigen (Ctrl+C = alles beenden)
tail -n +1 -F "$DET_LOG" --pid="$DET_PID"
