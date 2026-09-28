#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""EINEN Servo stromlos schalten (von Hand drehbar) oder wieder festsetzen - ueber den Kommando-
Socket der Bruecke (kein Stack-Neustart, kein pymycobot-Zugriff am Port vorbei).

    python3 tools/servo_frei.py 6          # Servo 6 (Greiferdrehung) frei
    python3 tools/servo_frei.py --fest 6   # wieder unter Drehmoment
Der Servo bleibt frei, bis ein neues Fahrkommando kommt (send_radians setzt alle sechs).
Vorher: Arm steht, kein Skript faehrt. Servo 1-5 frei = der Arm kann in sich zusammensacken!
"""
import argparse, os, socket, sys
ap = argparse.ArgumentParser(); ap.add_argument("servo", type=int, choices=range(1, 7))
ap.add_argument("--fest", action="store_true", help="wieder festsetzen (focus_servo)")
a = ap.parse_args()
sock = os.environ.get('MYCOBOT_BRIDGE_SOCKET', '/tmp/mycobot_bridge.sock') + '.cmd'
if a.servo != 6 and not a.fest:
    print(f"!! Servo {a.servo} frei = Arm haelt dort nicht mehr. Nur mit Hand am Arm.")
cmd = f"{'focus_servo' if a.fest else 'release_servo'} {a.servo}\n"
try:
    s = socket.socket(socket.AF_UNIX, socket.SOCK_STREAM); s.connect(sock); s.sendall(cmd.encode()); s.close()
except OSError as e:
    sys.exit(f"!! {sock}: {e} (laeuft die Bruecke?)")
print(f"  gesendet: {cmd.strip()}")
