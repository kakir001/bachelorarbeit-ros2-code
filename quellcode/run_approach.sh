#!/bin/bash
# =====================================================================
#  run_approach.sh — ANNAEHERUNG an Ziel (KEIN RViz — nur Kamerafenster + Terminal).
#
#  Benutzerwunsch (2026-06-07, aktuell):
#    - KEIN RViz (RViz+YOLO+Kamera gleichzeitig füllten den Nano-RAM und froren ihn ein,
#      Erkennung war nicht möglich). Nur ein cv2-Fenster mit 2 Kameras + dieses Terminal reicht.
#    - Im Kamerafenster ist der GEWAEHLTE Punkt sichtbar (linkes Panel = YOLO-Overlay /vida/overlay,
#      mit "ZIEL" markiert). Rechtes Panel = Aufnahmekamera.
#    - SCHRITT 1: Roboter startet bei Punkt 0 (home, alle Gelenke 0).
#    - SCHRITT 2: bringt den Greifer GENAU UEBER das Ziel, 90° SENKRECHT, 10cm höher, OEFFNET, wartet.
#    - Der Roboter steht physisch senkrecht über diesem Punkt (am echten Roboter beobachtet).
#    - BEIM START und BEIM BEENDEN parkt der Roboter zu Punkt 0 (beim Beenden bevor der Stack
#      stirbt → auch wenn die Servos frei werden, FAELLT der Arm NICHT).
#
#  Ablauf:
#    1) Systemvorbereitung (locale, rmem, USB power, GPU fan)  [fragt sudo]
#    2) Stack: demo.launch.py use_camera:=true use_rviz:=false  (KEIN RViz)
#    3) record_cam_publisher.py (2. Kamera → /record_cam/image_raw) + cam_viewer.py
#    4) vida_detector (YOLO) — auf "Modell bereit" + erstes "ZIEL" warten
#    5) approach_target AUSFUEHREN (0 → 10cm über Ziel, Greifer auf, warten)
#       wenn das Ziel gelatcht ist, wird der DETECTOR beendet (Roboterbewegung + YOLO gleichzeitig = RAM-thrash)
#    6) Kamerafenster + Terminal bleiben offen; zum Beenden Ctrl+C → komplettes Aufräumen
#
#  Verwendung:  cd ~/ros2_ws && ./run_approach.sh
#    APPROACH_HEIGHT=0.10 ./run_approach.sh   # Annäherungshöhe (m)
#    SIM=1 ./run_approach.sh                   # Simulation ohne Roboter
#  Logs: ~/ros2_ws/logs/approach_<zeit>/
# =====================================================================
set -u

APPROACH_HEIGHT="${APPROACH_HEIGHT:-0.10}"
SIM="${SIM:-0}"
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
REC_LOG="$LOGDIR/record_cam.log"
CAMV_LOG="$LOGDIR/cam_viewer.log"
DET_LOG="$LOGDIR/detector.log"
NODE_LOG="$LOGDIR/approach.log"

STACK_PID=""
REC_PID=""
CAMV_PID=""
DET_PID=""
NODE_PID=""

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

# Verwaiste mycobot_bridge.py + veralteter Socket = segfault beim nächsten Lauf. Aufkehren.
sweep_ros_ghosts() {
  pkill -f "record_cam_publisher" 2>/dev/null
  pkill -f "cam_viewer.py"        2>/dev/null
  pkill -f "vida_detector"        2>/dev/null
  pkill -f "approach_target"      2>/dev/null
  pkill -f "approach_target.launch.py" 2>/dev/null
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
  rm -f /tmp/mycobot_bridge.sock /tmp/mycobot_bridge.sock.cmd 2>/dev/null
}

# Roboter über den arm_controller zu Punkt 0 (alle Gelenke 0) senden. Der Stack
# (ros2_control + arm_controller) muss AKTIV sein. Damit beim Beenden die Servos nicht frei werden
# und der Arm fällt, wird ZUERST zu 0 geparkt, DANN der Stack getötet.
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
        mc.set_gripper_value(100, 50, 1)   # beim Park Finger OFFEN (gehaltene Schraube loslassen)
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
  echo
  log "AUFRAEUMEN — Hintergrund-nodes werden beendet..."
  # Erst die App-nodes stoppen (sie geben das Armkommando frei), DANN während der Stack noch aktiv ist
  # den Roboter zu 0 parken, GANZ ZULETZT den Stack töten → auch wenn die Servos frei werden, fällt der Arm nicht.
  kill_group "$NODE_PID"  "approach node"
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

# ---- 3a) START-PARK — Roboter soll sofort zu Punkt 0 gehen ----
goto_zero 5

# ---- 3b) 2. Kamera-Publisher (Aufnahmekamera /dev/video3) ----
log "record_cam_publisher (2. Kamera) wird gestartet → $REC_LOG"
setsid bash -c "exec python3 '$WS/record_cam_publisher.py'" >"$REC_LOG" 2>&1 &
REC_PID=$!

# ---- 3b2) Kamera-Viewer (2 Kameras in einem cv2-Fenster) ----
# Linkes Panel = /vida/overlay (YOLO-Erkennung + GEWAEHLTER Punkt mit "ZIEL" markiert),
# rechtes Panel = Aufnahmekamera. (Sobald der Detector startet, füllt sich das Overlay.)
if [ -n "${DISPLAY:-}" ]; then
  log "cam_viewer (2-Kamera-Fenster) wird gestartet → $CAMV_LOG"
  setsid bash -c "exec python3 '$WS/cam_viewer.py'" >"$CAMV_LOG" 2>&1 &
  CAMV_PID=$!
else
  warn "Kein DISPLAY — Kamerafenster uebersprungen."
fi

# ---- 4) Detector (YOLO) starten, auf Modell + erstes ZIEL warten ----
log "vida_detector wird gestartet → $DET_LOG"
# Reach-Kreise (grün=200mm sicher, amber=260mm Senkrecht-Limit) sind jetzt in vida_detector_node.py
# als PERMANENTER Standard — werden immer im Overlay gezeichnet, kein extra -p nötig.
setsid bash -c "exec ros2 run vida_vision vida_detector" >"$DET_LOG" 2>&1 &
DET_PID=$!

log "Auf Detector-Bereitschaft warten: 'Modell bereit' (max ${DETECTOR_WAIT}s)..."
ok=0
for i in $(seq 1 "$DETECTOR_WAIT"); do
  kill -0 "$DET_PID" 2>/dev/null || { err "Detector zu frueh gestorben! Letzte Zeilen:"; tail -n 25 "$DET_LOG"; exit 1; }
  grep -q "Modell bereit" "$DET_LOG" 2>/dev/null && { ok=1; log "Modell geladen (${i}s)"; break; }
  sleep 1
done
[ "$ok" = "1" ] || { err "Modell nicht geladen innerhalb ${DETECTOR_WAIT}s. Siehe $DET_LOG"; exit 1; }

log "Auf erstes Schrauben-Ziel ('ZIEL') warten (max 60s)..."
ok=0
for i in $(seq 1 60); do
  kill -0 "$DET_PID" 2>/dev/null || { err "Detector gestorben! Siehe $DET_LOG"; exit 1; }
  if grep -q "ZIEL" "$DET_LOG" 2>/dev/null; then
    ok=1; log "Schraube gefunden → $(grep 'ZIEL' "$DET_LOG" | tail -n1)"; break
  fi
  [ $((i % 10)) -eq 0 ] && warn "  ...noch kein Ziel (${i}s). Letzte: $(tail -n1 "$DET_LOG")"
  sleep 1
done
[ "$ok" = "1" ] || warn "Innerhalb 60s kein Ziel — wird trotzdem versucht (node hat 30s-Timeout)."

# ---- 5) approach_target AUSFUEHREN (im Hintergrund) ----
log "=============================================="
log " ANNAEHERUNG wird ausgefuehrt (0 → ${APPROACH_HEIGHT}m ueber Ziel, Greifer senkrecht+offen)"
log " Detector bleibt AN → im linken Kamera-Panel ist der gewaehlte Punkt (ZIEL) live sichtbar."
log " Der Roboter steht physisch 10cm ueber dem Ziel 90° senkrecht und wartet."
log "=============================================="
export APPROACH_HEIGHT
: > "$NODE_LOG"   # tail-Race verhindern — Datei vorab anlegen (sonst beendet sich tail sofort und schliesst alles)
setsid bash -c "exec ros2 launch mycobot_demo approach_target.launch.py" >>"$NODE_LOG" 2>&1 &
NODE_PID=$!

# HINWEIS: detector wird JETZT NICHT MEHR GETOETET — /vida/overlay soll live bleiben, damit im linken
# Kamera-Panel der gewählte Punkt ständig sichtbar ist (Benutzerpriorität). Da RViz weg ist,
# ist der RAM ohnehin entspannt. Wenn der Roboter nicht erreicht, die Schraube näher zum Roboter legen (grüne Zone).

# Node-Ausgabe live anzeigen; der node wartet bis Ctrl+C (Roboter steht über dem Ziel).
log "Node-Ausgabe (Ctrl+C = alles beenden):"
tail -n +1 -F "$NODE_LOG" --pid="$NODE_PID"

# Hierher kommt man, wenn der node endet (Ctrl+C) → cleanup trap läuft.
log "Wird beendet..."
