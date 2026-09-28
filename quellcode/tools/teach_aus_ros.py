#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""Aktuelle Gelenkstellung aus /joint_states speichern — waehrend der Stack laeuft.

tools/teach_schritt.py spricht die serielle Schnittstelle direkt an und kann
deshalb nur bei gestopptem Stack laufen. Dieses Werkzeug liest stattdessen
/joint_states und funktioniert damit im laufenden Betrieb — noetig, wenn die
Stellung mit dem Greifer angefahren wurde (statt von Hand gefuehrt) und der Arm
sie gerade unter Servodrehmoment haelt.

Das ist sogar der bessere Anlernfall: gespeichert wird die Stellung, die der Arm
AUS EIGENER KRAFT haelt — nicht die, in der ihn eine Hand gestuetzt hat. Der
Unterschied ist genau das Nachgeben, das sonst beim Zurueckfahren fehlt.

    python3 tools/teach_aus_ros.py trichter_greif2
    python3 tools/teach_aus_ros.py --zeig            # nur anzeigen
"""
import argparse
import json
import math
import os
import sys
import time

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy
from sensor_msgs.msg import JointState
import tf2_ros

ARM = ['joint2_to_joint1', 'joint3_to_joint2', 'joint4_to_joint3',
       'joint5_to_joint4', 'joint6_to_joint5', 'joint6output_to_joint6']

ap = argparse.ArgumentParser()
ap.add_argument("name", nargs="?", help="Punktname")
ap.add_argument("--zeig", action="store_true", help="nur anzeigen, nichts speichern")
ap.add_argument("--datei", default=os.path.expanduser("~/ros2_ws/teach_punkte.json"))
ap.add_argument("--proben", type=int, default=10)
ap.add_argument("--warte", type=float, default=0.0,
                help="vorher so viele Sekunden warten (Servos setzen sich noch)")
a = ap.parse_args()

if not a.zeig and not a.name:
    sys.exit("Name angeben, oder --zeig")

rclpy.init()
node = Node("teach_aus_ros")
proben = []


def on_js(m):
    d = dict(zip(m.name, m.position))
    if all(j in d for j in ARM):
        proben.append([d[j] for j in ARM])


# /joint_states kommt vom joint_state_broadcaster als BEST_EFFORT — mit dem
# Standardprofil (RELIABLE) kommt nichts an. Deshalb hier ausdruecklich.
qos = QoSProfile(depth=20, reliability=ReliabilityPolicy.BEST_EFFORT,
                 history=HistoryPolicy.KEEP_LAST)
node.create_subscription(JointState, "/joint_states", on_js, qos)

if a.warte > 0:
    print(f"  warte {a.warte:.0f} s (die Servos setzen sich nach der Fahrt noch) ...")
    t0 = time.time()
    while time.time() - t0 < a.warte:
        rclpy.spin_once(node, timeout_sec=0.2)
    proben.clear()

t0 = time.time()
while len(proben) < a.proben and time.time() - t0 < 20.0:
    rclpy.spin_once(node, timeout_sec=0.2)

if not proben:
    # Rueckfall: vielleicht doch RELIABLE
    qos2 = QoSProfile(depth=20, reliability=ReliabilityPolicy.RELIABLE,
                      history=HistoryPolicy.KEEP_LAST)
    node.create_subscription(JointState, "/joint_states", on_js, qos2)
    t0 = time.time()
    while len(proben) < a.proben and time.time() - t0 < 15.0:
        rclpy.spin_once(node, timeout_sec=0.2)

if not proben:
    sys.exit("!! nichts von /joint_states empfangen — laeuft der Stack?")

n = len(proben)
rad = [sum(p[i] for p in proben) / n for i in range(6)]
grad = [math.degrees(v) for v in rad]
streu = max(math.degrees(max(p[i] for p in proben) - min(p[i] for p in proben))
            for i in range(6))

print(f"  {n} Lesungen gemittelt")
print("  Gelenke [Grad]: " + ", ".join(f"{v:+7.2f}" for v in grad))
print(f"  Streuung: {streu:.3f} Grad")
if streu > 0.3:
    print("  ACHTUNG: der Arm bewegt sich noch — spaeter noch einmal aufnehmen.")

if a.zeig:
    node.destroy_node()
    rclpy.shutdown()
    sys.exit(0)

punkte = {}
if os.path.exists(a.datei):
    try:
        punkte = json.load(open(a.datei))
    except Exception:
        pass
eintrag = {
    "grad": [round(v, 3) for v in grad],
    "rad": [round(v, 5) for v in rad],
    "streuung_grad": round(streu, 3),
    "proben": n,
    "quelle": "joint_states (Arm haelt selbst)",
    "zeit": time.strftime("%Y-%m-%d %H:%M:%S"),
}

# TCP-Koordinate mitspeichern. Angefahren wird spaeter ueber die Gelenkwinkel
# (genauer), aber die ANFAHRT VON OBEN braucht x/y — sonst weiss kein Werkzeug,
# wo der Trichter steht. Der Gierwinkel wird ebenfalls abgelegt, damit der
# Greifer beim Anfahren nicht in eine andere Handstellung dreht.
try:
    buf = tf2_ros.Buffer()
    tf2_ros.TransformListener(buf, node)
    t0 = time.time()
    while time.time() - t0 < 6.0:
        rclpy.spin_once(node, timeout_sec=0.2)
        try:
            tr = buf.lookup_transform("robot_base", "tcp", rclpy.time.Time())
            p = tr.transform.translation
            eintrag["tcp_mm"] = [round(p.x * 1000, 2), round(p.y * 1000, 2),
                                 round(p.z * 1000, 2)]
            eintrag["yaw"] = 180.0     # in dieser Zelle bewaehrt; notfalls anpassen
            print("  TCP: x %+.1f  y %+.1f  z %+.1f mm" % tuple(eintrag["tcp_mm"]))
            break
        except Exception:
            pass
except Exception as e:
    print(f"  (TCP nicht ermittelbar: {e})")

punkte[a.name] = eintrag
with open(a.datei, "w") as f:
    json.dump(punkte, f, indent=2, sort_keys=True)
print(f"\n  '{a.name}' gespeichert -> {a.datei}")
print(f"  ({len(punkte)} Punkte: {', '.join(sorted(punkte))})")

node.destroy_node()
rclpy.shutdown()
