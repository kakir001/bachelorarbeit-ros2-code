#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""Servo-NULLPUNKT an der aktuellen Stellung setzen (set_servo_calibration, Potentialwert 2048).

Ablauf (2026-09-23, Achse 5): Achse mit nachstellen.py REAL senkrecht stellen (Handy-Wasserwaage),
dann  tools/servo_nullen.py 5 --ja . Der Encoder liest an dieser Stelle danach 0.
DANACH PFLICHT: alle Teach-Punkte dieser Achse um den ALTEN Encoderwert verschieben:
  tools/teach_gelenk_verschieben.py --gelenk 5 --grad -<alter Encoderwert>
(die Bruecke meldet Modell = Encoder - Versatz; Versatz dieser Achse in gelenk_nullpunkte.json muss 0 sein).
Bleibt im Servo gespeichert - nur mit Hand am Roboter und Absicht.
"""
import argparse, os, socket, sys
ap = argparse.ArgumentParser(); ap.add_argument("servo", type=int, choices=range(1, 7))
ap.add_argument("--ja", action="store_true", help="wirklich setzen (sonst nur Hinweis)")
a = ap.parse_args()
sock = os.environ.get('MYCOBOT_BRIDGE_SOCKET', '/tmp/mycobot_bridge.sock') + '.cmd'
if not a.ja:
    sys.exit(f"Wuerde Servo {a.servo} an der AKTUELLEN Stellung nullen. Mit --ja ausfuehren.")
cmd = f"calibrate_servo {a.servo}\n"
try:
    s = socket.socket(socket.AF_UNIX, socket.SOCK_STREAM); s.connect(sock); s.sendall(cmd.encode()); s.close()
except OSError as e:
    sys.exit(f"!! {sock}: {e} (laeuft die Bruecke?)")
print(f"  gesendet: {cmd.strip()}  -> Servo {a.servo}: aktuelle Stellung = Nullpunkt")
print("  Jetzt: Teach-Punkte verschieben (tools/teach_gelenk_verschieben.py), Bruecke-Log pruefen.")
