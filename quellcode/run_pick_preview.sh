#!/bin/bash
# =====================================================================
#  run_pick_preview.sh — ZWEIPHASIGER pick: VISION → (detector aus) → RViz VORSCHAU.
#
#  Idee (Benutzer, 2026-06-15): Auf dem Nano passen detector(2.1GB)+RViz nicht gleichzeitig
#  in den RAM. Schraube STATISCH → Koordinate einmal erfassen und SPEICHERN, detector+Kamera
#  BEENDEN (RAM wird frei), RViz öffnen, gespeicherten Punkt markieren, in RViz die Trajektorie+
#  KOLLISION (rot) sehen, bestätigen, echten Roboter ausführen.
#
#  PHASE A (detector an, kein RViz): stack + Kamera + cam_viewer + detector.
#     Schraube hinlegen, Erkennung im Overlay stabilisieren → in cam_viewer/Terminal ENTER (capture).
#  UEBERGANG: detector + cam_viewer + record_cam + Kamera BEENDEN (RAM wird frei).
#  PHASE B (RViz an, kein detector): RViz + Ziel-replay (marker) + pick_tilt.
#     In RViz Trajektorie+Kollision prüfen → im Terminal ENTER = BESTAETIGUNG (×2).
#
#  Verwendung:  cd ~/ros2_ws && ./run_pick_preview.sh
#     APPROACH_HEIGHT=0.05 GRASP_Z_OFFSET=-0.02 ./run_pick_preview.sh
#  Logs: ~/ros2_ws/logs/pick_preview_<zeit>/
# =====================================================================
set -u

APPROACH_HEIGHT="${APPROACH_HEIGHT:-0.10}"
# Standardwert des erfolgreichen pick-Rezepts (2026-06-15): TCP ~2.5cm überdefiniert → Pflaster.
# Override: GRASP_Z_OFFSET=-0.03 ./run_pick_preview.sh
GRASP_Z_OFFSET="${GRASP_Z_OFFSET:--0.025}"
# Modell-Laden (~15s) + ERSTE CUDA/cuDNN-Inferenz (~100s Warmup auf dem Nano) —
# der Detector meldet "Modell bereit" erst NACH dem Warmup (Sitzung 2026-07-16).
DETECTOR_WAIT="${DETECTOR_WAIT:-240}"
CTRL_WAIT="${CTRL_WAIT:-60}"
CAM_WAIT="${CAM_WAIT:-40}"
# Kamera-Profil: 848x480 statt 424x240 — schaerferes Fenster + bessere Erkennung
# kleiner Wellen. OHNE pointcloud (Phase A hat kein RViz, niemand braucht sie —
# spart CPU/RAM, 720p+720p@15 ohne pointcloud ist auf dem Nano getestet).
# Farb- und Tiefenprofil IMMER gleich halten (aligned_depth bricht sonst).
CAM_PROFILE="${CAM_PROFILE:-848x480x15}"
# 0.85 → 0.70 (2026-07-16 Live-Test): 0.85 war zu streng — reale Pick-Erkennungen
# lagen bei conf 0.72-0.75, mit 0.85 fand der Detector NICHTS. 0.70 filtert
# Unsicheres, laesst echte Wellen durch. Altes Rezept: CONF=0.5 ./run_pick_preview.sh
CONF="${CONF:-0.70}"
MODE_CONSENSUS="${MODE_CONSENSUS:-3}"
STICKY_RADIUS="${STICKY_RADIUS:-0.05}"
# IK-Innengrenze (m): der ROTE innere Kreis im Overlay. Schrauben mit Base-XY-Radius
# kleiner als dieser sind dem Roboter ZU NAHE (beim Top-Down-Grasp überbiegt der Arm) → kein Ziel.
# Override: REACH_MIN=0.16 ./run_pick_preview.sh
REACH_MIN="${REACH_MIN:-0.15}"

# Vorschau-Ablauf: jede Bewegung in RViz anzeigen und ENTER-Bestätigung abwarten.
PREVIEW_CONFIRM="${PREVIEW_CONFIRM:-1}"
DESCEND_VEL_SCALE="${DESCEND_VEL_SCALE:-0.10}"
# Rapid-traverse-Geschwindigkeit für freien Raum (home/hover/Rückkehr-zu-Null) — Abstieg/Greifen NICHT BETROFFEN.
# 1.00 = volles Rabbit: bei langsam zittert der Servo stick-slip, schnelle Fahrt ist glatt (kein Kontakt, kein Risiko).
# Bei Ruckeln: mit RAPID_VEL_SCALE=0.85 ./run_pick_preview.sh zurücknehmen.
RAPID_VEL_SCALE="${RAPID_VEL_SCALE:-1.00}"
export LC_ALL=C LC_NUMERIC=C LANG=C PYTHONUNBUFFERED=1
# rcutils-Logs UNGEPUFFERT in die Log-Dateien schreiben — sonst bleiben z.B.
# estop_button-Meldungen im 4KB-stdout-Puffer stecken und gehen bei SIGKILL
# verloren (estop_button.log war beim Vorfall 2026-07-16 deshalb LEER).
export RCUTILS_LOGGING_BUFFERED_STREAM=0
export APPROACH_HEIGHT GRASP_Z_OFFSET PREVIEW_CONFIRM DESCEND_VEL_SCALE RAPID_VEL_SCALE

WS="$HOME/ros2_ws"
STAMP="$(date +%Y%m%d_%H%M%S)"
LOGDIR="$WS/logs/pick_preview_$STAMP"
mkdir -p "$LOGDIR"
TARGET_FILE="$WS/.vida_target_latch.json"
RVIZ_CFG="$WS/rviz_preview.rviz"

STACK_LOG="$LOGDIR/stack.log";  CAM_LOG="$LOGDIR/camera.log"
REC_LOG="$LOGDIR/record_cam.log"; CAMV_LOG="$LOGDIR/cam_viewer.log"
DET_LOG="$LOGDIR/detector.log"; CAP_LOG="$LOGDIR/capture.log"
RVIZ_LOG="$LOGDIR/rviz.log"; REPLAY_LOG="$LOGDIR/replay.log"; NODE_LOG="$LOGDIR/pick_tilt.log"
ESTOP_LOG="$LOGDIR/estop_button.log"

STACK_PID=""; CAM_PID=""; REC_PID=""; CAMV_PID=""; DET_PID=""
RVIZ_PID=""; REPLAY_PID=""; NODE_PID=""; TAIL_PID=""; ESTOP_PID=""

log()  { echo -e "\033[1;36m[preview]\033[0m $*"; }
warn() { echo -e "\033[1;33m[preview]\033[0m $*"; }
err()  { echo -e "\033[1;31m[preview]\033[0m $*" >&2; }

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
  pkill -f "estop_button"         2>/dev/null
  pkill -f "estop_relay"          2>/dev/null
  pkill -f "record_cam_publisher" 2>/dev/null
  pkill -f "cam_viewer.py"        2>/dev/null
  pkill -f "target_latch.py"      2>/dev/null
  pkill -f "confirm_key.py"       2>/dev/null
  pkill -f "vida_detector"        2>/dev/null
  pkill -f "mycobot_demo/pick_tilt" 2>/dev/null
  pkill -f "pick_tilt.launch.py"  2>/dev/null
  pkill -f "demo.launch.py"       2>/dev/null
  pkill -f "rviz2"                2>/dev/null
  pkill -f "move_group"           2>/dev/null
  pkill -f "ros2_control_node"    2>/dev/null
  pkill -f "mycobot_bridge.py"    2>/dev/null
  pkill -f "realsense2_camera"    2>/dev/null
  pkill -f "robot_state_publisher" 2>/dev/null
  sleep 2
  pkill -9 -f "mycobot_bridge.py" 2>/dev/null
  pkill -9 -f "ros2_control_node" 2>/dev/null
  pkill -9 -f "move_group"        2>/dev/null
  pkill -9 -f "realsense2_camera" 2>/dev/null
  pkill -9 -f "rviz2"             2>/dev/null
  pkill -9 -f "mycobot_demo/pick_tilt" 2>/dev/null
  rm -f /tmp/mycobot_bridge.sock /tmp/mycobot_bridge.sock.cmd 2>/dev/null
}

ARM_JOINTS="[joint2_to_joint1, joint3_to_joint2, joint4_to_joint3, joint5_to_joint4, joint6_to_joint5, joint6output_to_joint6]"
goto_zero() {
  local secs="${1:-5}"; command -v ros2 >/dev/null 2>&1 || return 0
  if ros2 action list 2>/dev/null | grep -q "/arm_controller/follow_joint_trajectory"; then
    log "Roboter wird zu Punkt 0 gesendet (${secs}s)..."
    timeout 20 ros2 action send_goal /arm_controller/follow_joint_trajectory \
      control_msgs/action/FollowJointTrajectory \
      "{trajectory: {joint_names: $ARM_JOINTS, points: [{positions: [0.0,0.0,0.0,0.0,0.0,0.0], time_from_start: {sec: $secs}}]}}" \
      >/dev/null 2>&1 && log "Roboter bei Punkt 0." || warn "Senden zu 0 nicht abgeschlossen."
  else
    warn "arm_controller action nicht vorhanden — Senden zu 0 nicht moeglich."
  fi
}

# Ist NOT-AUS gerade aktiv? /estop ist transient_local (gelatcht) — solange das
# estop_button-Fenster lebt (cleanup beendet es zuletzt), ist der Zustand lesbar.
# Rueckgabe 0 = STOP aktiv. Beim Vorfall 2026-07-16 fuhr das Aufraeumen (goto_zero +
# pymycobot-Park) trotz gedruecktem NOT-AUS weiter, weil relay+bridge schon tot
# waren — diese Pruefung schliesst die Luecke.
estop_active() {
  command -v ros2 >/dev/null 2>&1 || return 1
  local out
  out=$(timeout 5 ros2 topic echo --once \
        --qos-durability transient_local --qos-reliability reliable \
        /estop std_msgs/msg/Bool 2>/dev/null)
  [ "${out#*data: }" != "$out" ] && echo "$out" | grep -q "data: true"
}

park_zero_pymycobot() {
  local port="${MYCOBOT_PORT:-/dev/ttyTHS1}"
  if estop_active; then
    err "NOT-AUS AKTIV — pymycobot-Park wird NICHT ausgefuehrt (Arm bleibt stehen)."
    err "Zum Parken: im NOT-AUS-Fenster FREIGABE druecken und ~/ros2_ws/go_to_zero.sh nutzen."
    return 0
  fi
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
  kill_group "$TAIL_PID"   "tail"
  kill_group "$NODE_PID"   "pick_tilt"
  kill_group "$REPLAY_PID" "target_replay"
  kill_group "$RVIZ_PID"   "rviz"
  kill_group "$DET_PID"    "detector"
  kill_group "$CAMV_PID"   "cam_viewer"
  kill_group "$REC_PID"    "record_cam"
  kill_group "$CAM_PID"    "camera"
  if estop_active; then
    err "NOT-AUS AKTIV — goto_zero wird uebersprungen (kein automatisches Fahren!)"
  else
    goto_zero 5
  fi
  kill_group "$STACK_PID"  "stack"
  sweep_ros_ghosts
  fuser /dev/ttyTHS1 >/dev/null 2>&1 && fuser -k -9 /dev/ttyTHS1 2>/dev/null
  park_zero_pymycobot
  # NOT-AUS-Fenster: sweep hat es meist schon beendet; hier Rest-Aufräumen.
  kill_group "$ESTOP_PID" "estop_button"
  log "Logs: $LOGDIR"
}
trap cleanup EXIT INT TERM

# ---- Systemvorbereitung ----
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
export USE_FAKE_HARDWARE="${USE_FAKE_HARDWARE:-false}"
[ "$USE_FAKE_HARDWARE" = "true" ] && warn "FAKE HARDWARE (sim) — kein Roboter"

# =====================================================================
#  PHASE A — VISION
# =====================================================================
log "########## PHASE A — VISION (detector an) ##########"
log "Stack (move_group+control+bridge, OHNE KAMERA, KEIN RViz) → $STACK_LOG"
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

# NOT-AUS-Taster: eigenes, immer-im-Vordergrund-Fenster; publiziert /estop.
# Enforcement: estop_relay (startet demo.launch.py bei echter Hardware automatisch)
# leitet den Zustand an den seriellen Bridge weiter -> Bewegung stoppt SOFORT.
if [ -n "${DISPLAY:-}" ]; then
  log "NOT-AUS-Taster (estop_button) → $ESTOP_LOG"
  setsid bash -c "exec ros2 run mycobot_calibration estop_button" >"$ESTOP_LOG" 2>&1 &
  ESTOP_PID=$!
else
  warn "Kein DISPLAY — NOT-AUS-Fenster nicht moeglich. Terminal-Alternative:"
  warn "  ros2 topic pub --once --qos-durability transient_local /estop std_msgs/msg/Bool \"{data: true}\""
fi

goto_zero 5

# Detector FRUEH starten: Modell-Laden (~15s) + CUDA-Warmup (~100s) laufen dann
# PARALLEL zum Kamera-Start statt danach (Sitzung 2026-07-16: 3-4min → ~2min).
# Er wartet einfach mit "noch kein color"-Warnungen, bis die Kamera sendet.
# Wahrnehmungs-Offset (Lineal-Messung: Kamera liest ~+7/+11mm versetzt) aus
# .place_offset.json — DIESELBE Quelle wie click_place_moveit.py. Der Detector
# addiert die Korrektur auf das veroeffentlichte /vida/target (2026-07-16:
# vorher wurde die gemessene Korrektur im Pick-Pfad NIE angewendet).
read -r OFF_X OFF_Y OFF_Z <<< "$(python3 -c "
import json
try:
    d = json.load(open('$WS/.place_offset.json'))
    print(d.get('x', 0.0), d.get('y', 0.0), d.get('z', 0.0))
except Exception:
    print(0.0, 0.0, 0.0)")"
log "Wahrnehmungs-Offset: x=$OFF_X y=$OFF_Y z=$OFF_Z m (aus .place_offset.json)"

log "vida_detector VORAB gestartet (conf=$CONF, reach_min=${REACH_MIN}m) → $DET_LOG"
setsid bash -c "exec ros2 run vida_vision vida_detector --ros-args \
  -p conf:=$CONF -p mode_consensus:=$MODE_CONSENSUS -p sticky_radius:=$STICKY_RADIUS \
  -p reach_radius_min:=$REACH_MIN \
  -p offset_x:=$OFF_X -p offset_y:=$OFF_Y -p offset_z:=$OFF_Z" >"$DET_LOG" 2>&1 &
DET_PID=$!

log "Kamera (rs_launch $CAM_PROFILE, OHNE pointcloud, eigene Gruppe) → $CAM_LOG"
setsid bash -c "exec ros2 launch realsense2_camera rs_launch.py \
  pointcloud.enable:=false align_depth.enable:=true \
  depth_module.profile:=$CAM_PROFILE rgb_camera.profile:=$CAM_PROFILE" >"$CAM_LOG" 2>&1 &
CAM_PID=$!

log "Auf Kamera-Topic warten: /camera/color/image_raw (max ${CAM_WAIT}s)..."
ok=0
for i in $(seq 1 "$CAM_WAIT"); do
  kill -0 "$CAM_PID" 2>/dev/null || { err "Kamera zu frueh gestorben!"; tail -n 25 "$CAM_LOG"; exit 1; }
  [ "$(ros2 topic info /camera/color/image_raw 2>/dev/null | grep -c 'Publisher count: [1-9]')" -ge 1 ] && { ok=1; log "Kamera sendet (${i}s)"; break; }
  sleep 1
done
[ "$ok" = "1" ] || { err "Kamera sendet nicht innerhalb ${CAM_WAIT}s. Siehe $CAM_LOG"; exit 1; }

# 2. Kamera (record_cam) entfernt — nicht verwendet (Benutzer, 2026-06-25).
if [ -n "${DISPLAY:-}" ]; then
  log "cam_viewer (Overlay-Anzeige) → $CAMV_LOG"
  setsid bash -c "exec python3 '$WS/cam_viewer.py'" >"$CAMV_LOG" 2>&1 &
  CAMV_PID=$!
fi

log "Auf Detector-Warmup warten (Modell + erste CUDA-Inferenz, max ${DETECTOR_WAIT}s)..."
ok=0
for i in $(seq 1 "$DETECTOR_WAIT"); do
  kill -0 "$DET_PID" 2>/dev/null || { err "Detector zu frueh gestorben!"; tail -n 25 "$DET_LOG"; exit 1; }
  grep -q "Modell bereit" "$DET_LOG" 2>/dev/null && { ok=1; log "Modell geladen (${i}s)"; break; }
  sleep 1
done
[ "$ok" = "1" ] || { err "Modell nicht geladen. Siehe $DET_LOG"; exit 1; }

log "=============================================="
log " SCHRAUBE HINLEGEN (150-190mm, 20-30° geneigt). Wenn die Erkennung im Overlay stabil ist:"
log " IN DIESEM TERMINAL ENTER = Ziel SPEICHERN (danach werden detector+Kamera beendet)."
log "=============================================="
# capture FOREGROUND — wenn der Benutzer ENTER drückt, wird das Ziel gespeichert und kehrt zurück
python3 "$WS/target_latch.py" capture "$TARGET_FILE" 2>&1 | tee "$CAP_LOG"
[ -f "$TARGET_FILE" ] || { err "Ziel nicht gespeichert — wird beendet."; exit 1; }

# =====================================================================
#  UEBERGANG — VISION BEENDEN (RAM freigeben)
# =====================================================================
log "########## UEBERGANG — detector+Kamera werden beendet (RAM wird frei) ##########"
kill_group "$DET_PID"  "detector"; DET_PID=""
kill_group "$CAMV_PID" "cam_viewer"; CAMV_PID=""
kill_group "$REC_PID"  "record_cam"; REC_PID=""
kill_group "$CAM_PID"  "camera"; CAM_PID=""
sleep 2
log "Freier RAM: $(free -h | awk 'NR==2{print $7}')"

# =====================================================================
#  PHASE B — RViz VORSCHAU + PICK
# =====================================================================
log "########## PHASE B — RViz VORSCHAU ##########"
if [ -n "${DISPLAY:-}" ]; then
  # RViz mit MoveIt-Parametern starten (moveit_rviz.launch.py) — standalone
  # 'ros2 run rviz2' bekam kein robot_description_semantic und lieferte "No Planning Scene
  # Loaded" + unsichtbarer Roboter (Sitzung 31 Fix).
  RVIZ_ARG=""; [ -f "$RVIZ_CFG" ] && RVIZ_ARG="rviz_config:=$RVIZ_CFG"
  log "RViz (moveit_rviz.launch.py, $RVIZ_CFG) → $RVIZ_LOG"
  setsid bash -c "exec ros2 launch mycobot_moveit_config moveit_rviz.launch.py $RVIZ_ARG" >"$RVIZ_LOG" 2>&1 &
  RVIZ_PID=$!
else
  warn "Kein DISPLAY — RViz uebersprungen (keine Vorschau moeglich, pick laeuft trotzdem)."
fi

log "Ziel-replay (/vida/target 5Hz + /vida/preview_marker) → $REPLAY_LOG"
setsid bash -c "exec python3 '$WS/target_latch.py' replay '$TARGET_FILE'" >"$REPLAY_LOG" 2>&1 &
REPLAY_PID=$!
sleep 1

log "pick_tilt (hover=${APPROACH_HEIGHT}m offset=${GRASP_Z_OFFSET}) → $NODE_LOG"
: > "$NODE_LOG"
setsid bash -c "exec ros2 launch mycobot_demo pick_tilt.launch.py" >>"$NODE_LOG" 2>&1 &
NODE_PID=$!
tail -n +1 -F "$NODE_LOG" --pid="$NODE_PID" &
TAIL_PID=$!

log "=============================================="
log " In RViz die Trajektorie + KOLLISION (rot) pruefen."
log " Fuer die BESTAETIGUNG in DIESEM TERMINAL ENTER druecken (pick_tilt wartet 2 mal):"
log "   1) nach HOVER — wenn die Trajektorie sicher ist ENTER → abstieg+greifen"
log "   2) nach dem Greifen — wenn gehalten ENTER → anheben"
log " Ctrl+C = alles beenden + Roboter zu 0 parken."
log "=============================================="
# confirm_key FOREGROUND — hält das Skript am Leben; ENTER → /pick/confirm
python3 "$WS/confirm_key.py"

log "confirm_key beendet — wird beendet..."
