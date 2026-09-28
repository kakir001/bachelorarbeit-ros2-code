#!/bin/bash
export CYCLONEDDS_URI="${CYCLONEDDS_URI:-file://$HOME/ros2_ws/cyclonedds_loopback.xml}"   # DDS nur loopback, s. Datei
# =====================================================================
#  handsteuerung.sh — HANDSTEUERUNG: Arm von Hand verfahren, Kamerabild live sehen.
#
#  Zweck: den Arm aus dem Kamerabild fahren (oder irgendwohin stellen), BEVOR
#  tools/schale_finden.py laeuft. Am 2026-09-09 scheiterte der Schalen-Fund genau
#  daran: der Arm stand in der Nullstellung senkrecht mitten im Bild und die
#  Schale lag ueberhaupt nicht im Sichtfeld.
#
#  Was startet:
#    1) Stack, ECHTE Hardware, OHNE RViz, OHNE Kamera (RViz frisst auf dem Nano
#       den Speicher, den der Detektor spaeter braucht)
#    2) Kamera 848x480x15, aligned_depth AN, PointCloud AUS
#       — genau das Profil, das schale_finden.py erwartet. Farb- und Tiefenprofil
#         MUESSEN gleich sein, sonst wird aligned_depth unbrauchbar.
#    3) joint_pose_gui — sechs Schieberegler, ein "Send" je Fahrt
#    4) cam_viewer auf dem ROHEN Farbbild — kein YOLO, kein 4-Minuten-Warmup
#    5) NOT-AUS-Taster (estop_relay startet demo.launch.py bei echter Hardware selbst)
#
#  ⚠️ joint_pose_gui GEHT AN MoveIt VORBEI: keine Kollisionspruefung, keine
#     Gelenkgrenzen ausser +/- pi. Kleine Schritte, Fahrzeit >= 3 s, hinsehen.
#     J1 dreht den ganzen Arm um die Basis — die harmloseste Achse.
#
#  VERWENDUNG:  cd ~/ros2_ws && ./handsteuerung.sh
#               CHARUCO_MONTIERT=0 ./handsteuerung.sh   (Platte ist abgeschraubt)
#  BEENDEN: Ctrl+C — der Arm bleibt STEHEN, wo er ist (er wird NICHT zu 0 geparkt,
#           sonst stuende er gleich wieder mitten im Bild).
# =====================================================================
set -u
export LC_ALL=C LC_NUMERIC=C LANG=C PYTHONUNBUFFERED=1
export RCUTILS_LOGGING_BUFFERED_STREAM=0

WS="$HOME/ros2_ws"
STAMP="$(date +%Y%m%d_%H%M%S)"
LOGDIR="$WS/logs/handsteuerung_$STAMP"; mkdir -p "$LOGDIR"
STACK_LOG="$LOGDIR/stack.log"; CAM_LOG="$LOGDIR/camera.log"
GUI_LOG="$LOGDIR/joint_gui.log"; VIEW_LOG="$LOGDIR/cam_viewer.log"
ESTOP_LOG="$LOGDIR/estop_button.log"

CAM_WAIT="${CAM_WAIT:-60}"; CTRL_WAIT="${CTRL_WAIT:-60}"
CAM_PROFIL="${CAM_PROFIL:-848x480x15}"

log()  { echo -e "\033[1;36m[hand]\033[0m $*"; }
warn() { echo -e "\033[1;33m[hand]\033[0m $*"; }
err()  { echo -e "\033[1;31m[hand]\033[0m $*" >&2; }

STACK_PID=""; CAM_PID=""; GUI_PID=""; VIEW_PID=""; ESTOP_PID=""

# Nur Prozessgruppen beenden, KEIN pkill -f: ein Muster, das im selben Kommando
# auch als Text vorkommt, trifft die eigene Shell (DEVLOG: exit 144).
kill_group() {
  local pid="$1" name="$2"; [ -z "$pid" ] && return 0
  if kill -0 "$pid" 2>/dev/null; then
    log "$name wird beendet (pgid=$pid)..."
    kill -TERM -- "-$pid" 2>/dev/null
    for _ in $(seq 1 10); do kill -0 "$pid" 2>/dev/null || break; sleep 0.5; done
    kill -0 "$pid" 2>/dev/null && kill -KILL -- "-$pid" 2>/dev/null
  fi
}

cleanup() {
  [ "${_CLEANED:-0}" = "1" ] && return 0
  _CLEANED=1; trap - EXIT INT TERM; echo
  log "AUFRAEUMEN — der Arm bleibt stehen, wo er ist."
  kill_group "$VIEW_PID"  "cam_viewer"
  kill_group "$GUI_PID"   "joint_pose_gui"
  kill_group "$ESTOP_PID" "estop_button"
  kill_group "$CAM_PID"   "camera"
  kill_group "$STACK_PID" "stack"
  sleep 1
  rm -f /tmp/mycobot_bridge.sock /tmp/mycobot_bridge.sock.cmd 2>/dev/null
  log "Fertig. Log: $LOGDIR"
}
trap cleanup EXIT INT TERM

set +u
source /opt/ros/galactic/setup.bash
source "$WS/install/setup.bash"
set -u

export USE_FAKE_HARDWARE="${USE_FAKE_HARDWARE:-false}"
export MYCOBOT_PORT="${MYCOBOT_PORT:-/dev/ttyTHS1}"
export DISPLAY="${DISPLAY:-:0}"
export CHARUCO_MONTIERT="${CHARUCO_MONTIERT:-true}"
export SCHALE_MONTIERT="${SCHALE_MONTIERT:-false}"
export TRICHTER_MONTIERT="${TRICHTER_MONTIERT:-false}"   # siehe start_mycobot.sh

# Auf den PFAD der Binaerdatei pruefen, nicht auf den blossen Namen: sonst meldet
# jeder fremde Prozess, der die Zeichenkette nur im Kommando stehen hat (ein
# Wartebefehl, ein grep, ein Editor), faelschlich "laeuft schon" (2026-09-10 passiert).
if pgrep -f "lib/controller_manager/ros2_control_node" >/dev/null 2>&1; then
  err "Es laeuft bereits ein ros2_control_node. Erst diesen beenden."
  exit 1
fi

# --- 1) Stack: echte Hardware, kein RViz, keine Kamera --------------------
log "########## STACK (echte Hardware, ohne RViz, ohne Kamera) -> $STACK_LOG"
setsid bash -c "exec ros2 launch mycobot_moveit_config demo.launch.py \
  use_camera:=false use_rviz:=false use_fake_hardware:=$USE_FAKE_HARDWARE" >"$STACK_LOG" 2>&1 &
STACK_PID=$!
ok=0
for i in $(seq 1 "$CTRL_WAIT"); do
  kill -0 "$STACK_PID" 2>/dev/null || { err "Stack zu frueh gestorben!"; tail -n 25 "$STACK_LOG"; exit 1; }
  ros2 control list_controllers 2>/dev/null | grep -q "arm_controller.*active" && { ok=1; log "Controller AKTIV (${i}s)"; break; }
  sleep 1
done
[ "$ok" = "1" ] || { err "Controller wurde nicht aktiv. Siehe $STACK_LOG"; exit 1; }

# --- 2) Kamera: 848x480x15, aligned_depth AN, PointCloud AUS --------------
log "########## KAMERA ($CAM_PROFIL, aligned_depth, ohne PointCloud) -> $CAM_LOG"
setsid bash -c "exec ros2 launch realsense2_camera rs_launch.py \
  pointcloud.enable:=false align_depth.enable:=true \
  depth_module.profile:=$CAM_PROFIL rgb_camera.profile:=$CAM_PROFIL" >"$CAM_LOG" 2>&1 &
CAM_PID=$!
ok=0
for i in $(seq 1 "$CAM_WAIT"); do
  kill -0 "$CAM_PID" 2>/dev/null || { err "Kamera zu frueh gestorben!"; tail -n 25 "$CAM_LOG"; exit 1; }
  [ "$(ros2 topic info /camera/aligned_depth_to_color/image_raw 2>/dev/null | grep -c 'Publisher count: [1-9]')" -ge 1 ] \
    && { ok=1; log "Kamera sendet, aligned_depth da (${i}s)"; break; }
  sleep 1
done
[ "$ok" = "1" ] || { err "Kamera sendet nicht in ${CAM_WAIT}s"; tail -n 20 "$CAM_LOG"; exit 1; }

# --- 3) NOT-AUS-Taster ----------------------------------------------------
log "NOT-AUS-Taster -> $ESTOP_LOG"
setsid bash -c "exec ros2 run mycobot_calibration estop_button" >"$ESTOP_LOG" 2>&1 &
ESTOP_PID=$!
sleep 2

# --- 4) Gelenk-GUI --------------------------------------------------------
log "joint_pose_gui (sechs Schieberegler) -> $GUI_LOG"
setsid bash -c "exec ros2 run mycobot_calibration joint_pose_gui" >"$GUI_LOG" 2>&1 &
GUI_PID=$!
sleep 3

# --- 5) Live-Kamerabild (ROH, kein Detektor) ------------------------------
log "cam_viewer auf dem rohen Farbbild -> $VIEW_LOG"
setsid bash -c "CAM1_TOPIC=/camera/color/image_raw exec python3 '$WS/cam_viewer.py'" >"$VIEW_LOG" 2>&1 &
VIEW_PID=$!

log "############################################################"
log " BEREIT. Drei Fenster: Gelenk-Regler, Kamerabild, NOT-AUS."
log "   * Regler stellen -> 'Send' -> der Arm faehrt. Fahrzeit >= 3 s lassen."
log "   * J1 dreht um die Basis (harmlos). J2/J3 legen den Arm um — VORSICHT,"
log "     hier prueft NICHTS auf Kollision."
log "   * Ziel: Arm und Greifer aus dem Kamerabild, Schale frei sichtbar."
log " Danach in einem ZWEITEN Terminal (dieses hier laufen lassen):"
log "     cd ~/ros2_ws && python3 tools/schale_finden.py"
log " Beenden: Ctrl+C — der Arm bleibt stehen, wo er ist."
log "############################################################"

while kill -0 "$STACK_PID" 2>/dev/null && kill -0 "$CAM_PID" 2>/dev/null; do
  sleep 1
done
warn "Stack oder Kamera wurde beendet — wird aufgeraeumt."
