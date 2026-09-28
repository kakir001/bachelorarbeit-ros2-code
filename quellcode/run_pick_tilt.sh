#!/bin/bash
# =====================================================================
#  run_pick_tilt.sh — TILT-BEWUSSTER PICK (KEIN RViz — Kamerafenster + Terminal).
#
#  Ablauf (Sitzung 25, Plan: ~/.claude/plans/shimmering-wondering-bachman.md):
#    home (0) → auf /welle/ziel warten (detector sendet, wenn er einen ZUVERLAESSIGEN tilt findet; sonst nicht
#               → Roboter wartet, Benutzer mischt die Wellen)
#    → HOVER (von der Grasp-Pose entlang der Annäherungsachse APPROACH_HEIGHT zurück, Finger offen)
#    → im cam_viewer-Fenster 'g' = BESTAETIGUNG → PARALLELER Abstieg → greifen → anheben → halten.
#
#  KEIN RViz (Nano RAM + Tegra segfault). Linkes Panel /welle/overlay (zeigt tilt=.. lin=.. [mod];
#  wenn unzuverlässig "Wellen mischen"), rechtes Panel Aufnahmekamera.
#  Detector bleibt AN (pick_tilt liest ständig das aktuellste Ziel).
#
#  Verwendung:  cd ~/ros2_ws && ./run_pick_tilt.sh
#    APPROACH_HEIGHT=0.05 ./run_pick_tilt.sh   # Hover-Höhe (m), 5cm
#    SIM=1 ./run_pick_tilt.sh                   # Simulation ohne Roboter
#  BESTAETIGUNG:  im Kamerafenster 'g'  (oder: ros2 topic pub --once /pick/confirm std_msgs/msg/Empty '{}')
#  Logs: ~/ros2_ws/logs/pick_tilt_<zeit>/
# =====================================================================
set -u

APPROACH_HEIGHT="${APPROACH_HEIGHT:-0.10}"
SIM="${SIM:-0}"
DETECTOR_WAIT="${DETECTOR_WAIT:-120}"
CTRL_WAIT="${CTRL_WAIT:-60}"
# Detector-Schwellen (Sitzung 29): per env override-bar.
CONF="${CONF:-0.75}"
MODE_CONSENSUS="${MODE_CONSENSUS:-3}"
STICKY_RADIUS="${STICKY_RADIUS:-0.05}"

export LC_ALL=C
export LC_NUMERIC=C
export LANG=C
# Python-Logs sofort ausgeben (detector-Log war 2min gepuffert — Lektion Sitzung 28).
export PYTHONUNBUFFERED=1

WS="$HOME/ros2_ws"
STAMP="$(date +%Y%m%d_%H%M%S)"
LOGDIR="$WS/logs/pick_tilt_$STAMP"
mkdir -p "$LOGDIR"

STACK_LOG="$LOGDIR/stack.log"
REC_LOG="$LOGDIR/record_cam.log"
CAMV_LOG="$LOGDIR/cam_viewer.log"
DET_LOG="$LOGDIR/detector.log"
NODE_LOG="$LOGDIR/pick_tilt.log"

STACK_PID=""
REC_PID=""
CAMV_PID=""
DET_PID=""
NODE_PID=""

log()  { echo -e "\033[1;36m[run_pick_tilt]\033[0m $*"; }
warn() { echo -e "\033[1;33m[run_pick_tilt]\033[0m $*"; }
err()  { echo -e "\033[1;31m[run_pick_tilt]\033[0m $*" >&2; }

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

# Verwaiste mycobot_bridge.py + veralteter Socket = segfault beim nächsten Lauf. Aufkehren.
sweep_ros_ghosts() {
  # ACHTUNG: das Muster "pick_tilt" passt auf den eigenen Namen des Wrappers (run_pick_tilt.sh)
  # und tötet das Skript → cleanup→sweep→tot Endlosschleife. Nur mit node ("mycobot_demo/pick_tilt"
  # = installierter Executable-Pfad) und launch ("pick_tilt.launch.py") Mustern matchen.
  pkill -f "record_cam_publisher" 2>/dev/null
  pkill -f "cam_viewer.py"        2>/dev/null
  pkill -f "wellen_detektor"        2>/dev/null
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

# Roboter zu Punkt 0 parken (während der Stack läuft; auch wenn beim Beenden die Servos frei werden, soll er nicht fallen).
ARM_JOINTS="[joint2_to_joint1, joint3_to_joint2, joint4_to_joint3, joint5_to_joint4, joint6_to_joint5, joint6output_to_joint6]"
goto_zero() {
  local secs="${1:-5}"
  command -v ros2 >/dev/null 2>&1 || return 0
  if ros2 action list 2>/dev/null | grep -q "/arm_controller/follow_joint_trajectory"; then
    log "Roboter wird zu Punkt 0 gesendet (${secs}s, Servo soll nicht frei werden)..."
    timeout 20 ros2 action send_goal /arm_controller/follow_joint_trajectory \
      control_msgs/action/FollowJointTrajectory \
      "{trajectory: {joint_names: $ARM_JOINTS, points: [{positions: [0.0,0.0,0.0,0.0,0.0,0.0], time_from_start: {sec: $secs}}]}}" \
      >/dev/null 2>&1 \
      && log "Roboter bei Punkt 0." || warn "Senden zu 0 nicht abgeschlossen (trotzdem weiter)."
  else
    warn "arm_controller action nicht vorhanden — Roboter kann nicht zu 0 gesendet werden."
  fi
}

# Nachdem Stack/bridge beendet sind, mit pymycobot GARANTIERT zu 0 parken + Servos UNTER DREHMOMENT lassen.
# Warum: action-basiertes goto_zero ist unzuverlässig — beim Beenden gibt mycobot_bridge die Servos
# frei und LAESST den Roboter FALLEN (Lektion Sitzung 30). Diese Funktion öffnet den Port direkt,
# sendet zu 0 und lässt die Servos energetisiert (haltend) → der Arm bleibt bei 0 hängen, fällt nicht.
park_zero_pymycobot() {
  [ "${SIM:-0}" = "1" ] && return 0
  local port="${MYCOBOT_PORT:-/dev/ttyTHS1}"
  for _ in $(seq 1 10); do fuser "$port" >/dev/null 2>&1 || break; sleep 0.5; done
  if fuser "$port" >/dev/null 2>&1; then
    warn "pymycobot Park: $port noch belegt — wird uebersprungen (Arm kann fallen, von Hand kontrollieren!)"; return 0
  fi
  log "GARANTIERTER Park zu 0 mit pymycobot (Servos bleiben unter Drehmoment, Arm faellt nicht)..."
  if MYCOBOT_PORT="$port" timeout 35 python3 - <<'PYEOF'
import os, time
try:
    from pymycobot import MyCobot280
    mc = MyCobot280(os.environ.get("MYCOBOT_PORT", "/dev/ttyTHS1"), 1000000)
    time.sleep(0.6)
    try:
        mc.power_on(); time.sleep(0.4)      # heruntergefallene/freie Servos neu energetisieren
    except Exception:
        pass
    mc.send_radians([0.0] * 6, 25)
    for _ in range(14):
        time.sleep(1.0)
        if mc.is_moving() == 0:
            break
    try:
        # Beim Park erst OEFFNEN (eine gehaltene Welle muss fallen koennen), dann
        # wieder SCHLIESSEN. Grund fuer das Schliessen (Benutzer, 2026-09-09): in der
        # Nullstellung haengt der Greifer direkt unter der Kamera; offene Finger
        # verdecken die Arbeitsflaeche und werden vom Detektor sogar selbst als Welle
        # erkannt. 100 = offen, 0 = zu, die 1 waehlt den adaptiven Greifer.
        mc.set_gripper_value(100, 50, 1)
        time.sleep(1.5)
        mc.set_gripper_value(0, 50, 1)
        time.sleep(1.0)
    except Exception as ge:
        print("Greifer-Oeffnen Warnung:", ge)
    a = mc.get_radians()
    if a and len(a) == 6:
        print("groesster |Winkel| nach Park = %.3f rad" % max(abs(x) for x in a))
except Exception as e:
    print("pymycobot Park FEHLER:", e)
PYEOF
  then
    log "pymycobot Park beendet — Roboter bei 0, Servos halten."
  else
    warn "pymycobot Park Timeout/fehlgeschlagen — Arm von Hand kontrollieren."
  fi
}

cleanup() {
  # Re-entry guard: falls trap EXIT/INT/TERM + ein mögliches self-signal von sweep cleanup
  # erneut auslöst, die Endlosschleife abbrechen (nur einmal laufen).
  [ "${_CLEANED:-0}" = "1" ] && return 0
  _CLEANED=1
  trap - EXIT INT TERM
  echo
  log "AUFRAEUMEN — Hintergrund-nodes werden beendet..."
  kill_group "$NODE_PID"  "pick_tilt node"
  kill_group "$DET_PID"   "detector"
  kill_group "$CAMV_PID"  "cam_viewer"
  kill_group "$REC_PID"   "record_cam"
  goto_zero 5                            # Park beim Beenden (Stack noch aktiv)
  kill_group "$STACK_PID" "stack"
  sweep_ros_ghosts
  if fuser /dev/ttyTHS1 >/dev/null 2>&1; then
    warn "/dev/ttyTHS1 NOCH belegt — erzwungen toeten"; fuser -k -9 /dev/ttyTHS1 2>/dev/null
  fi
  park_zero_pymycobot                      # GARANTIERTER Park zu 0 (Servos unter Drehmoment, Arm faellt nicht)
  log "Logs: $LOGDIR"
}
trap cleanup EXIT INT TERM

# ---- 1) Systemvorbereitung ----
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

log "KEIN RViz — um den RAM zu entlasten nur Kamerafenster + Terminal."

# ---- ROS-Umgebung ----
set +u
source /opt/ros/galactic/setup.bash
source "$WS/install/setup.bash"
set -u
cd "$WS" || { err "ros2_ws nicht vorhanden"; exit 1; }

if [ "$SIM" = "1" ]; then
  export USE_FAKE_HARDWARE="true"; warn "SIM MODE — fake_components (kein Roboter)"
else
  export USE_FAKE_HARDWARE="false"; log "ECHTER Roboter @ ${MYCOBOT_PORT:-/dev/ttyTHS1}"
fi

# ---- 2) Stack (KEIN RViz + Kamera) ----
log "Stack wird gestartet (use_camera:=true use_rviz:=false) → $STACK_LOG"
setsid bash -c "exec ros2 launch mycobot_moveit_config demo.launch.py \
  use_camera:=true use_rviz:=false" >"$STACK_LOG" 2>&1 &
STACK_PID=$!

# ---- 3) Warten bis Controller active sind ----
log "Auf Controller warten (max ${CTRL_WAIT}s)..."
ok=0
for i in $(seq 1 "$CTRL_WAIT"); do
  kill -0 "$STACK_PID" 2>/dev/null || { err "Stack zu frueh gestorben! Letzte Zeilen:"; tail -n 25 "$STACK_LOG"; exit 1; }
  if ros2 control list_controllers 2>/dev/null | grep -q "arm_controller.*active"; then
    ok=1; log "Controller AKTIV (${i}s)"; break
  fi
  sleep 1
done
[ "$ok" = "1" ] || { err "Controller wurde nicht aktiv innerhalb ${CTRL_WAIT}s. Siehe $STACK_LOG"; exit 1; }

# ---- 3a) START-PARK ----
goto_zero 5

# ---- 3a2) 0-PRUEFUNG (Sitzung 29): JTC sagt ohne Toleranz "success"; wenn der Roboter physisch
# nicht 0 erreicht hat (Klemmen/Anstoßen), blockiert MoveIt mit start-state-collision.
# Echte /joint_states werden gelesen und |Winkel| geprüft; >5° → ABORT.
if [ "$SIM" != "1" ]; then
  ZMAX=$(timeout 15 python3 - <<'PYEOF' 2>/dev/null
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import JointState

ARM = {"joint2_to_joint1","joint3_to_joint2","joint4_to_joint3",
       "joint5_to_joint4","joint6_to_joint5","joint6output_to_joint6"}
rclpy.init()
node = Node("zero_check")
got = {}
def cb(m):
    for n, p in zip(m.name, m.position):
        if n in ARM:
            got[n] = p
node.create_subscription(JointState, "/joint_states", cb, 10)
import time
t0 = time.time()
while time.time() - t0 < 10 and len(got) < len(ARM):
    rclpy.spin_once(node, timeout_sec=0.2)
print(f"{max(abs(v) for v in got.values()):.4f}" if got else "NaN")
node.destroy_node(); rclpy.shutdown()
PYEOF
)
  if [ -z "$ZMAX" ] || [ "$ZMAX" = "NaN" ]; then
    warn "0-Pruefung: /joint_states nicht lesbar (trotzdem weiter)."
  elif python3 -c "import sys; sys.exit(0 if float('$ZMAX') > 0.09 else 1)"; then
    err "ROBOTER NICHT BEI 0 (groesster |Winkel|=${ZMAX} rad > 0.09≈5°)."
    err "Der Arm koennte physisch klemmen/nicht erreicht haben — von Hand loesen, ggf."
    err "bei beendetem Stack mit pymycobot send_radians([0]*6,25) parken, dann neu starten."
    exit 1
  else
    log "0-Pruefung OK (groesster |Winkel|=${ZMAX} rad)."
  fi
fi

# ---- 3b) 2. Kamera-Publisher ----
log "record_cam_publisher (2. Kamera) wird gestartet → $REC_LOG"
setsid bash -c "exec python3 '$WS/record_cam_publisher.py'" >"$REC_LOG" 2>&1 &
REC_PID=$!

# ---- 3b2) Kamera-Viewer (2 Kameras + 'g' Bestätigungstaste) ----
if [ -n "${DISPLAY:-}" ]; then
  log "cam_viewer (2 Kameras, 'g'=BESTAETIGEN) wird gestartet → $CAMV_LOG"
  setsid bash -c "exec python3 '$WS/cam_viewer.py'" >"$CAMV_LOG" 2>&1 &
  CAMV_PID=$!
else
  warn "Kein DISPLAY — Kamerafenster uebersprungen. Bestaetigung: ros2 topic pub --once /pick/confirm std_msgs/msg/Empty '{}'"
fi

# ---- 4) Detector (YOLO) starten, auf Modell warten ----
log "wellen_detektor wird gestartet → $DET_LOG"
setsid bash -c "exec ros2 run wellenerkennung wellen_detektor --ros-args \
  -p conf:=$CONF -p mode_consensus:=$MODE_CONSENSUS -p sticky_radius:=$STICKY_RADIUS" >"$DET_LOG" 2>&1 &
DET_PID=$!

log "Auf Detector-Bereitschaft warten: 'Modell bereit' (max ${DETECTOR_WAIT}s)..."
ok=0
for i in $(seq 1 "$DETECTOR_WAIT"); do
  kill -0 "$DET_PID" 2>/dev/null || { err "Detector zu frueh gestorben! Letzte Zeilen:"; tail -n 25 "$DET_LOG"; exit 1; }
  grep -q "Modell bereit" "$DET_LOG" 2>/dev/null && { ok=1; log "Modell geladen (${i}s)"; break; }
  sleep 1
done
[ "$ok" = "1" ] || { err "Modell nicht geladen innerhalb ${DETECTOR_WAIT}s. Siehe $DET_LOG"; exit 1; }

warn "HINWEIS: pick_tilt erhaelt /welle/ziel nur bei ZUVERLAESSIGEM tilt. Wenn das Overlay 'Wellen mischen'"
warn "     sagt, die Welle deutlich GENEIGT hinlegen/mischen; der Roboter wartet im Hover."

# ---- 5) pick_tilt AUSFUEHREN ----
log "=============================================="
log " TILT-BEWUSSTER PICK (hover=${APPROACH_HEIGHT}m → 'g' Bestaetigung → paralleler Abstieg → greifen → anheben)"
log " BESTAETIGUNG: Taste 'g' im Kamerafenster."
log "=============================================="
export APPROACH_HEIGHT
: > "$NODE_LOG"   # tail-Race verhindern
setsid bash -c "exec ros2 launch mycobot_demo pick_tilt.launch.py" >>"$NODE_LOG" 2>&1 &
NODE_PID=$!

log "Node-Ausgabe (Ctrl+C = alles beenden):"
tail -n +1 -F "$NODE_LOG" --pid="$NODE_PID"

log "Wird beendet..."
