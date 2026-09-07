#!/usr/bin/env bash
# Fährt den echten Roboter in die Nullstellung: alle 6 Gelenke = 0 rad,
# Greifer OFFEN. send_radians lässt die Servos unter Drehmoment (kein Absturz
# beim Loslassen). Nutzt direkt pymycobot über /dev/ttyTHS1 — der ros2_control
# Stack darf dabei NICHT am selben Port hängen (Sim-Modus ist ok).
set -euo pipefail
export LC_ALL=C LC_NUMERIC=C

PORT="${MYCOBOT_PORT:-/dev/ttyTHS1}"
BAUD="${MYCOBOT_BAUD:-1000000}"
SPEED="${ZERO_SPEED:-30}"

python3 - "$PORT" "$BAUD" "$SPEED" <<'PY'
import sys, time
from pymycobot import MyCobot280

port, baud, speed = sys.argv[1], int(sys.argv[2]), int(sys.argv[3])
mc = MyCobot280(port, baud)
time.sleep(1.5)
print("aktuelle Winkel:", mc.get_angles())

mc.send_radians([0, 0, 0, 0, 0, 0], speed)   # Nullstellung, Servos unter Drehmoment
time.sleep(3)

mc.set_gripper_value(100, 50, 1)             # 100 = offen, adaptiver Greifer (Open-Loop)
time.sleep(2)

print("Nullstellung + Greifer offen fertig; Winkel:", mc.get_angles())
PY
