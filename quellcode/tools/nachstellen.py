#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""Nachstellen: die Gelenke so lange ueber das Ziel hinaus kommandieren, bis der
Encoder das Ziel meldet.

Warum
-----
Am 2026-09-11, Greifpunkt im Trichter: kommandiert (-40.41, -6.47, -114.26,
+31.20, +0.01, +130.42), Encoder nach 6 s Setzzeit (-39.13, -7.73, -114.88,
+28.65, +0.52, +129.26). Fehler J2 -1.26, J3 -0.62, J4 -2.55, J6 -1.16 Grad.
Die drei Nickachsen J2+J3+J4 summieren sich auf -4.4 Grad: der Greifer stand
sichtbar schief, obwohl das Modell "senkrecht" sagte. Der Benutzer: "das Spiel
in Achse 2 macht jede Genauigkeit zunichte - wir muessen es ausrechnen und die
Achse entsprechend weiter fahren."

Das ist der Teil des Fehlers, den der ENCODER SIEHT: die Servos bleiben in ihrem
Totband stehen, sobald sie dem Ziel nahe genug sind, und die Schwerkraft drueckt
sie an den unteren Rand dieses Bandes. Ein zweites Kommando auf DASSELBE Ziel
bewegt nichts (liegt ja schon im Band). Kommandiert man aber Ziel + Fehler,
laeuft der Servo weiter und landet - mit dem Fehler - auf dem Ziel. Das wird
hier bis zu dreimal wiederholt, bis alle Gelenke innerhalb der Toleranz sind.

Was das NICHT beseitigt: das Spiel HINTER dem Getriebe. Das sieht der Encoder
nicht (Elephant Robotics 2026-09-08; tools/measure_deflection_camera.py). Es
bleibt als konstanter, richtungsabhaengiger Versatz und muss von aussen (Kamera
+ ChArUco, tools/measure_model_vs_charuco.py) bestimmt und als Vorhalt
eingetragen werden - naechster Schritt, wenn dieser hier sitzt.

Aufruf (Ziel = die sechs Gelenkwinkel in Grad, z.B. aus zeige_punkt.py):
    python3 tools/nachstellen.py --ziel -40.41 -6.47 -114.26 31.20 0.01 130.42
    python3 tools/nachstellen.py --ziel ... --toleranz 0.3 --runden 3 --setzzeit 5
"""
import argparse
import math
import sys
import time

import rclpy
from rclpy.action import ActionClient
from rclpy.node import Node
from control_msgs.action import FollowJointTrajectory
from sensor_msgs.msg import JointState
from trajectory_msgs.msg import JointTrajectoryPoint
from builtin_interfaces.msg import Duration

ARM = ["joint2_to_joint1", "joint3_to_joint2", "joint4_to_joint3",
       "joint5_to_joint4", "joint6_to_joint5", "joint6output_to_joint6"]

ap = argparse.ArgumentParser()
ap.add_argument("--ziel", type=float, nargs=6, required=True, help="Gelenkwinkel [Grad]")
ap.add_argument("--toleranz", type=float, default=0.3, help="[Grad] je Gelenk, darunter fertig")
ap.add_argument("--runden", type=int, default=3)
ap.add_argument("--setzzeit", type=float, default=5.0, help="[s] warten nach jeder Fahrt")
ap.add_argument("--dauer", type=float, default=3.0, help="[s] Fahrzeit je Korrektur")
ap.add_argument("--max-korrektur", type=float, default=4.0,
                help="[Grad] mehr wird nicht vorgehalten - dann stimmt etwas anderes nicht")
a = ap.parse_args()


class N(Node):
    def __init__(self):
        super().__init__("nachstellen")
        self.js = {}
        self.create_subscription(JointState, "/joint_states", self._cb, 10)
        self.ac = ActionClient(self, FollowJointTrajectory, "/arm_controller/follow_joint_trajectory")
        if not self.ac.wait_for_server(timeout_sec=10):
            sys.exit("!! arm_controller nicht da")

    def _cb(self, m):
        self.js.update(dict(zip(m.name, m.position)))

    def ist(self):
        self.js = {}
        t = time.time()
        while time.time() - t < 3 and not all(j in self.js for j in ARM):
            rclpy.spin_once(self, timeout_sec=0.1)
        return [math.degrees(self.js[j]) for j in ARM]

    def fahre(self, grad, dauer):
        g = FollowJointTrajectory.Goal()
        g.trajectory.joint_names = ARM
        p = JointTrajectoryPoint()
        p.positions = [math.radians(v) for v in grad]
        p.time_from_start = Duration(sec=int(dauer), nanosec=int((dauer % 1) * 1e9))
        g.trajectory.points = [p]
        f = self.ac.send_goal_async(g)
        rclpy.spin_until_future_complete(self, f)
        r = f.result().get_result_async()
        rclpy.spin_until_future_complete(self, r, timeout_sec=dauer + 10)
        return r.result().result.error_code == 0


rclpy.init()
n = N()
ziel = list(a.ziel)
kommando = list(ziel)          # was tatsaechlich gesendet wird (Ziel + Vorhalt)
fmt = lambda v: " ".join(f"{x:+8.2f}" for x in v)
print(f"Ziel      : {fmt(ziel)}")
ist = n.ist()
fehler = [i - z for i, z in zip(ist, ziel)]
print(f"Ist       : {fmt(ist)}")
print(f"Fehler    : {fmt(fehler)}   (Nick J2+J3+J4 = {fehler[1]+fehler[2]+fehler[3]:+.2f} Grad)")

for runde in range(1, a.runden + 1):
    if all(abs(e) <= a.toleranz for e in fehler):
        print(f"fertig nach {runde-1} Korrektur(en), alle Gelenke innerhalb {a.toleranz} Grad")
        break
    # Vorhalt akkumulieren: Kommando = Ziel - (bisheriger Fehler), begrenzt
    kommando = [k - e for k, e in zip(kommando, fehler)]
    vorhalt = [k - z for k, z in zip(kommando, ziel)]
    if any(abs(v) > a.max_korrektur for v in vorhalt):
        print(f"!! Vorhalt {fmt(vorhalt)} ueberschreitet {a.max_korrektur} Grad - abgebrochen")
        break
    print(f"\nRunde {runde}: Vorhalt {fmt(vorhalt)}")
    if not n.fahre(kommando, a.dauer):
        print("!! Fahrt fehlgeschlagen"); break
    time.sleep(a.setzzeit)
    ist = n.ist()
    fehler = [i - z for i, z in zip(ist, ziel)]
    print(f"Ist       : {fmt(ist)}")
    print(f"Fehler    : {fmt(fehler)}   (Nick J2+J3+J4 = {fehler[1]+fehler[2]+fehler[3]:+.2f} Grad)")
else:
    if not all(abs(e) <= a.toleranz for e in fehler):
        print(f"!! nach {a.runden} Runden noch ausserhalb der Toleranz")

print(f"\nVorhalt am Ende (Kommando - Ziel): {fmt([k - z for k, z in zip(kommando, ziel)])}")
n.destroy_node()
rclpy.shutdown()
