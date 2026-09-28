#!/usr/bin/env bash
# Fährt den echten Roboter in die REALE Nullstellung (teach_punkte.json "nullstellung", 20.9.: J2 +3, J3 +1 -
# Encoder-Nullpunkte verschoben; Rückfall 0 rad),
# Greifer ZU (die offenen Finger verdecken sonst die Kamerasicht). send_radians lässt die Servos unter Drehmoment (kein Absturz
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

import json, math
try: null = json.load(open("/home/er/ros2_ws/teach_punkte.json"))["nullstellung"]["rad"]   # REALE Nullstellung (20.9.)
except Exception: null = [0, 0, 0, 0, 0, 0]
mc.send_radians([float(v) for v in null], speed)   # Nullstellung, Servos unter Drehmoment
time.sleep(3)

# Finger ZU (0). In der Nullstellung steht der Greifer direkt unter der Kamera;
# offene Finger ragen dann ins Bild und verdecken die Arbeitsflaeche.
# 100 = offen, 0 = zu; die 1 waehlt den adaptiven Greifer (Open-Loop).
mc.set_gripper_value(0, 50, 1)
time.sleep(2)

print("Nullstellung + Greifer zu fertig; Winkel:", mc.get_angles())
PY
