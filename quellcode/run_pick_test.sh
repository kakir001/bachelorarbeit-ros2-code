#!/bin/bash
# =====================================================================
#  run_pick_test.sh — Wellen-pick&place-Test, KOMPLETT über Terminal.
#  KEIN Claude nötig. KEIN RViz (Nano RAM-Ersparnis). Alles wird geloggt.
#
#  Ablauf:
#    1) Systemvorbereitung (locale, rmem, USB power, GPU fan)  [fragt sudo]
#    2) Stack starten: demo.launch.py use_camera:=true use_rviz:=false
#    3) Warten bis Controller active sind
#    4) wellen_detektor starten, warten bis "Modell bereit" + erstes "ZIEL" kommt
#    5) pick_place_cartesian AUSFUEHREN (auf Bildschirm + Log)
#    6) Beim Beenden ALLE Hintergrund-nodes aufräumen (keine Geister hinterlassen)
#
#  Verwendung:
#    cd ~/ros2_ws && ./run_pick_test.sh
#    # Place-Punkt ändern:
#    PLACE_X=0.12 PLACE_Y=-0.10 PLACE_Z=0.05 ./run_pick_test.sh
#    # Abstiegs-Offset zur Welle (in Wellenoberseite versenken/anheben):
#    GRASP_Z_OFFSET=0.005 ./run_pick_test.sh
#    # NUR Simulation (ohne Roboter):
#    SIM=1 ./run_pick_test.sh
#
#  Logs: ~/ros2_ws/logs/pick_<zeit>/{stack.log,detector.log,pick_place.log}
# =====================================================================
set -u

# ---- Einstellbare Parameter (per env override) -----------------------
PLACE_X="${PLACE_X:-0.15}"
PLACE_Y="${PLACE_Y:-0.15}"
PLACE_Z="${PLACE_Z:-0.05}"
GRASP_Z_OFFSET="${GRASP_Z_OFFSET:-0.0}"
SIM="${SIM:-0}"
VIEW="${VIEW:-1}"                       # 1 = Live-Overlay-Fenster (ohne RViz). Wenn RAM sehr knapp VIEW=0
DETECTOR_WAIT="${DETECTOR_WAIT:-120}"   # max Wartezeit fuer detector-Modell+erstes Ziel (s)
CTRL_WAIT="${CTRL_WAIT:-60}"            # max Wartezeit fuer controller active (s)

# ---- Locale (damit Qt/MoveIt floats nicht kaputtgehen) ---------------
export LC_ALL=C
export LC_NUMERIC=C
export LANG=C

WS="$HOME/ros2_ws"
STAMP="$(date +%Y%m%d_%H%M%S)"
LOGDIR="$WS/logs/pick_$STAMP"
mkdir -p "$LOGDIR"

STACK_LOG="$LOGDIR/stack.log"
DET_LOG="$LOGDIR/detector.log"
PICK_LOG="$LOGDIR/pick_place.log"

# Gruppenleiter-PIDs der Hintergrundprozesse (per setsid jeder in eigener process group)
STACK_PID=""
DET_PID=""
VIEW_PID=""

log()  { echo -e "\033[1;36m[run_pick_test]\033[0m $*"; }
warn() { echo -e "\033[1;33m[run_pick_test]\033[0m $*"; }
err()  { echo -e "\033[1;31m[run_pick_test]\033[0m $*" >&2; }

# Eine process group erst sanft, dann erzwungen töten
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

# Alle ROS-node-Geister aufkehren (gemeinsam für preflight + exit genutzt).
# KRITISCH: wenn mycobot_bridge.py, ros2_control_node sterben, bleiben sie VERWAIST und
# halten /dev/ttyTHS1 weiter; beim nächsten Lauf SEGFAULTet eine frische bridge wegen
# Port/Socket-Konflikt. Daher bridge + veraltete Sockets UNBEDINGT aufräumen.
sweep_ros_ghosts() {
  pkill -f "view_overlay.py"        2>/dev/null
  pkill -f "wellen_detektor"          2>/dev/null
  pkill -f "pick_place_cartesian"   2>/dev/null
  pkill -f "pick_place.launch.py"   2>/dev/null
  pkill -f "demo.launch.py"         2>/dev/null
  pkill -f "move_group"             2>/dev/null
  pkill -f "ros2_control_node"      2>/dev/null
  pkill -f "mycobot_bridge.py"      2>/dev/null   # <-- verwaiste bridge (haelt Port)
  pkill -f "realsense2_camera"      2>/dev/null
  pkill -f "robot_state_publisher"  2>/dev/null
  sleep 2
  # Wenn etwas TERM widerstanden hat, erzwingen
  pkill -9 -f "mycobot_bridge.py"   2>/dev/null
  pkill -9 -f "ros2_control_node"   2>/dev/null
  pkill -9 -f "move_group"          2>/dev/null
  pkill -9 -f "realsense2_camera"   2>/dev/null
  rm -f /tmp/mycobot_bridge.sock /tmp/mycobot_bridge.sock.cmd 2>/dev/null
}

cleanup() {
  echo
  log "AUFRAEUMEN — Hintergrund-nodes werden beendet..."
  kill_group "$VIEW_PID"  "overlay-viewer"
  kill_group "$DET_PID"   "detector"
  kill_group "$STACK_PID" "stack"
  sweep_ros_ghosts   # inkl. bridge + verwaiste nodes + veralteter Socket
  # Prüfen: ist der Port noch belegt?
  if fuser /dev/ttyTHS1 >/dev/null 2>&1; then
    warn "/dev/ttyTHS1 NOCH belegt — verbleibenden Halter erzwungen toeten"
    fuser -k -9 /dev/ttyTHS1 2>/dev/null
  fi
  log "Logs: $LOGDIR"
  log "  lesen:  tail -n 80 $PICK_LOG"
}
trap cleanup EXIT INT TERM

# ---- 1) Systemvorbereitung (sudo — fragt evtl. Passwort, am Anfang erledigen) ----
log "Systemvorbereitung (sudo evtl. noetig)..."
sudo sysctl -w net.core.rmem_max=8388608 net.core.wmem_max=8388608 \
             net.core.rmem_default=8388608 net.core.wmem_default=8388608 >/dev/null 2>&1
echo 0 | sudo tee /sys/devices/57000000.gpu/railgate_enable >/dev/null 2>&1
sudo sh -c 'echo 255 > /sys/devices/pwm-fan/target_pwm' 2>/dev/null
for dev in /sys/bus/usb/devices/*/idVendor; do
  [ "$(cat "$dev" 2>/dev/null)" = "8086" ] && \
    echo on | sudo tee "$(dirname "$dev")/power/control" >/dev/null 2>&1
done

# ---- Geister aus dem vorherigen eingefrorenen Lauf aufkehren ----------
# (BESONDERS verwaiste mycobot_bridge.py + veralteter Socket — sonst SEGFAULTet
#  ein frisches ros2_control; siehe Diagnose 2026-06-04 Sitzung 17)
sweep_ros_ghosts
if fuser /dev/ttyTHS1 >/dev/null 2>&1; then
  warn "/dev/ttyTHS1 vom vorherigen Lauf belegt — wird erzwungen freigegeben"
  fuser -k -9 /dev/ttyTHS1 2>/dev/null; sleep 1
fi

# ---- ROS-Umgebung ----------------------------------------------------
# ROS setup.bash nutzt undefinierte Variablen → set -u beim sourcen ausschalten.
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

# ---- 3) Warten bis Controller active sind ----------------------------
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

# ---- 4) Detector starten, auf Modell + erstes ZIEL warten ------------
log "wellen_detektor wird gestartet → $DET_LOG"
setsid bash -c "exec ros2 run wellenerkennung wellen_detektor" >"$DET_LOG" 2>&1 &
DET_PID=$!

log "Auf Detector-Bereitschaft warten: 'Modell bereit' (max ${DETECTOR_WAIT}s)..."
ok=0
for i in $(seq 1 "$DETECTOR_WAIT"); do
  kill -0 "$DET_PID" 2>/dev/null || { err "Detector zu frueh gestorben! Letzte Zeilen:"; tail -n 25 "$DET_LOG"; exit 1; }
  grep -q "Modell bereit" "$DET_LOG" 2>/dev/null && { ok=1; log "Modell geladen (${i}s)"; break; }
  sleep 1
done
[ "$ok" = "1" ] || { err "Modell nicht geladen innerhalb ${DETECTOR_WAIT}s. Siehe $DET_LOG"; exit 1; }

# ---- 4b) Live-Overlay-Fenster (gewählte Welle GRUEN + ZIEL) ------
if [ "$VIEW" = "1" ]; then
  if [ -n "${DISPLAY:-}" ]; then
    log "Overlay-Viewer wird geoeffnet (DISPLAY=$DISPLAY) — im Fenster 'q'/ESC = schliessen"
    setsid bash -c "exec python3 '$WS/view_overlay.py'" \
      >"$LOGDIR/viewer.log" 2>&1 &
    VIEW_PID=$!
  else
    warn "Kein DISPLAY — Overlay-Fenster uebersprungen. Fuer Headless-PNG:"
    warn "  SAVE_ONLY=1 python3 $WS/view_overlay.py  (→ /tmp/welle/overlay_latest.png)"
  fi
fi

log "Auf erstes Wellen-Ziel ('ZIEL') warten (max 60s)..."
ok=0
for i in $(seq 1 60); do
  kill -0 "$DET_PID" 2>/dev/null || { err "Detector gestorben! Siehe $DET_LOG"; exit 1; }
  if grep -q "ZIEL" "$DET_LOG" 2>/dev/null; then
    ok=1
    log "Welle gefunden → $(grep 'ZIEL' "$DET_LOG" | tail -n1)"
    break
  fi
  # auch 'keine Welle' melden, damit wir nicht blind warten
  if [ $((i % 10)) -eq 0 ]; then
    warn "  ...noch kein Ziel (${i}s). Detector letzte Zeile: $(tail -n1 "$DET_LOG")"
  fi
  sleep 1
done
if [ "$ok" != "1" ]; then
  err "Innerhalb 60s keine Welle erkannt. Ist eine Welle im Kamerabild? Siehe $DET_LOG"
  err "pick_place wird trotzdem versucht (hat eigenen 10s-Timeout)..."
fi

# ---- 5) pick_place_cartesian AUSFUEHREN ------------------------------
log "=============================================="
log " PICK & PLACE wird ausgefuehrt"
log "  place=($PLACE_X,$PLACE_Y,$PLACE_Z) grasp_z_offset=$GRASP_Z_OFFSET"
log "  log: $PICK_LOG"
log "=============================================="
set +e
# HINWEIS: wir nutzen ros2 launch — pick_place.launch.py lädt das SRDF (robot_description_semantic)
# + kinematics-Parameter der MoveGroupInterface. Reines "ros2 run" blieb ohne SRDF
# und stürzte mit "Unable to construct robot model" ab.
export PLACE_X PLACE_Y PLACE_Z GRASP_Z_OFFSET
ros2 launch mycobot_demo pick_place.launch.py 2>&1 | tee "$PICK_LOG"
RC=${PIPESTATUS[0]}
set -e 2>/dev/null

# ros2 launch liefert 0, selbst wenn ein node darin stirbt → echten Status aus dem LOG ziehen.
# Ist ein node mit Nicht-Null beendet? ('process has died ... exit code N' N!=0)
if grep -qE "process has died.*exit code [1-9]" "$PICK_LOG" 2>/dev/null; then
  RC=1
fi

echo
if [ "$RC" = "0" ]; then
  log "✅ pick_place ERFOLGREICH beendet"
else
  err "❌ pick_place mit FEHLER beendet — obiges Log / $PICK_LOG ansehen"
fi

# cleanup trap läuft bei EXIT (detector + stack werden beendet)
exit "$RC"
