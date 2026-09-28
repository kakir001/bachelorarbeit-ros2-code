#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""Jog-Fenster, das den Greifer NICHT dreht.

Warum noch eins: cartesian_jog.py loest jeden Schritt mit numerischer IK und
Zufalls-Seeds. Die Orientierung ist dabei nur "senkrecht nach unten" erzwungen,
der GIERWINKEL bleibt frei — also sucht sich der Loeser bei jedem Klick eine
andere Handstellung aus, und der Greifer dreht sich sichtbar mit. Am Trichter
ist das nicht hinnehmbar: die Teile an der Rueckseite des Greifers streifen dann
den Kegel (einmal wurde er dadurch schon verschoben).

Hier wird stattdessen /compute_cartesian_path benutzt. Der bekommt die AKTUELLE
Orientierung als Ziel-Orientierung mit — sie kann sich also gar nicht aendern,
der Loeser hat keine Wahl. Zusaetzlich faehrt der Arm eine gerade Linie statt
irgendeines Umwegs.

SOLL statt IST (derselbe Grund wie in cartesian_jog.py, Fehler von 2026-09-08):
Achse 2 gibt unter Last nach, der Arm steht tiefer als befohlen. Wer den
naechsten Schritt auf die GEMESSENE Lage rechnet, schleppt diesen Fehler mit und
sackt bei jedem Klick weiter ab. Deshalb fuehrt das Fenster eine eigene
Soll-Lage; das Nachgeben bleibt ein gleichbleibender Versatz und wird angezeigt,
statt sich aufzusummieren.

    python3 tools/jog_ohne_drehung.py
"""
import math
import threading
import time
import tkinter as tk
from tkinter import ttk

import rclpy
from rclpy.action import ActionClient
from rclpy.node import Node
from geometry_msgs.msg import Pose
from moveit_msgs.srv import GetCartesianPath
from moveit_msgs.msg import RobotState
from sensor_msgs.msg import JointState
from control_msgs.action import FollowJointTrajectory
import tf2_ros
import os, sys
sys.path.insert(0, os.path.dirname(os.path.abspath(__file__))); from bahnpruefung import pruefe_bahn  # Stetigkeit (Unfall 13.9.)


class Jog(Node):
    def __init__(self):
        super().__init__("jog_ohne_drehung")
        self.js = None
        self.create_subscription(JointState, "/joint_states",
                                 self._on_js, 10)
        self.buf = tf2_ros.Buffer()
        tf2_ros.TransformListener(self.buf, self)
        self.cart = self.create_client(GetCartesianPath, "/compute_cartesian_path")
        self.ac = ActionClient(self, FollowJointTrajectory,
                               "/arm_controller/follow_joint_trajectory")

    def _on_js(self, m):
        self.js = m

    def tcp(self):
        """(x,y,z) in m und die Orientierung als Quaternion-Msg, oder None."""
        try:
            t = self.buf.lookup_transform("robot_base", "tcp", rclpy.time.Time())
        except Exception:
            return None, None
        p = t.transform.translation
        return [p.x, p.y, p.z], t.transform.rotation


class App:
    def __init__(self, node):
        self.node = node
        self.soll = None
        self.busy = False

        self.root = tk.Tk()
        self.root.title("Jog ohne Drehung — Greifer bleibt wie er ist")

        top = ttk.Frame(self.root, padding=8)
        top.pack(fill="x")
        ttk.Label(top, text="Schritt (mm):").pack(side="left")
        self.step = tk.DoubleVar(value=2.0)
        ttk.Spinbox(top, from_=0.5, to=30.0, increment=0.5, width=6,
                    textvariable=self.step).pack(side="left", padx=4)
        ttk.Label(top, text="   Fahrzeit (s):").pack(side="left")
        self.dauer = tk.DoubleVar(value=2.5)
        ttk.Spinbox(top, from_=0.5, to=15.0, increment=0.5, width=6,
                    textvariable=self.dauer).pack(side="left", padx=4)

        mid = ttk.Frame(self.root, padding=8)
        mid.pack()
        for spalte, (ax, name) in enumerate(((0, "X"), (1, "Y"), (2, "Z"))):
            ttk.Button(mid, text=f"{name} +", width=8,
                       command=lambda a=ax: self.jog(a, +1)).grid(row=0, column=spalte, padx=6, pady=3)
            ttk.Button(mid, text=f"{name} −", width=8,
                       command=lambda a=ax: self.jog(a, -1)).grid(row=1, column=spalte, padx=6, pady=3)

        info = ttk.Frame(self.root, padding=8)
        info.pack(fill="x")
        self.l_soll = ttk.Label(info, text="SOLL:  —")
        self.l_soll.pack(anchor="w")
        self.l_ist = ttk.Label(info, text="IST:   —")
        self.l_ist.pack(anchor="w")
        self.l_diff = ttk.Label(info, text="Nachgeben: —")
        self.l_diff.pack(anchor="w")
        ttk.Button(info, text="SOLL = IST (uebernehmen)",
                   command=self.sync).pack(anchor="w", pady=4)

        self.status = ttk.Label(self.root, text="bereit", padding=8, foreground="blue")
        self.status.pack(fill="x")

        self.root.after(400, self.tick)

    def tick(self):
        p, _ = self.node.tcp()
        if p:
            self.l_ist.config(text="IST:   x=%.1f  y=%.1f  z=%.1f mm"
                              % (p[0]*1000, p[1]*1000, p[2]*1000))
            if self.soll:
                d = [(self.soll[i] - p[i]) * 1000 for i in range(3)]
                b = math.sqrt(sum(v*v for v in d))
                self.l_diff.config(
                    text="Nachgeben: dx %+.1f  dy %+.1f  dz %+.1f  (%.1f mm)"
                         % (d[0], d[1], d[2], b),
                    foreground=("red" if b > 8 else "orange" if b > 4 else "green"))
        if self.soll:
            self.l_soll.config(text="SOLL:  x=%.1f  y=%.1f  z=%.1f mm"
                               % tuple(v*1000 for v in self.soll))
        self.root.after(400, self.tick)

    def sync(self):
        p, _ = self.node.tcp()
        if p is None:
            self.status.config(text="kein TF", foreground="red")
            return
        self.soll = list(p)
        self.status.config(text="SOLL auf die gemessene Lage gesetzt", foreground="blue")

    def jog(self, achse, vz):
        if self.busy:
            self.status.config(text="beschaeftigt", foreground="orange")
            return
        p, q = self.node.tcp()
        if p is None:
            self.status.config(text="kein TF", foreground="red")
            return
        if self.soll is None:
            self.soll = list(p)
        self.soll[achse] += vz * max(0.5, float(self.step.get())) / 1000.0
        ziel = list(self.soll)
        self.busy = True
        self.status.config(text="faehrt ...", foreground="orange")
        threading.Thread(target=self._fahre, args=(ziel, q), daemon=True).start()

    def _fahre(self, ziel, q):
        try:
            if not self.node.cart.wait_for_service(timeout_sec=5.0):
                self._sag("compute_cartesian_path fehlt", "red")
                return
            if self.node.js is None:
                self._sag("keine /joint_states", "red")
                return
            rq = GetCartesianPath.Request()
            rq.header.frame_id = "robot_base"
            rq.group_name = "arm"
            rq.link_name = "tcp"
            rq.max_step = 0.002
            rq.jump_threshold = 0.0
            rq.avoid_collisions = True
            st = RobotState()
            st.joint_state = self.node.js
            rq.start_state = st
            pose = Pose()
            pose.position.x, pose.position.y, pose.position.z = ziel
            pose.orientation = q          # <- unveraendert: der Greifer dreht sich nicht
            rq.waypoints = [pose]

            fut = self.node.cart.call_async(rq)
            t0 = time.time()
            while not fut.done() and time.time() - t0 < 20.0:
                time.sleep(0.05)
            res = fut.result()
            if res is None:
                self._sag("keine Antwort vom Planer", "red")
                return
            ok_b, txt_b = pruefe_bahn(res.solution.joint_trajectory, ARM)
            if not ok_b:
                self._sag(txt_b + " - nicht gefahren", "red"); return
            if res.fraction < 0.9:
                self._sag(f"nur {res.fraction*100:.0f} % planbar — nicht gefahren", "red")
                return

            traj = res.solution.joint_trajectory
            n = len(traj.points)
            dur = max(0.5, float(self.dauer.get()))
            for i, pt in enumerate(traj.points):
                t = dur * (i + 1) / n
                pt.time_from_start.sec = int(t)
                pt.time_from_start.nanosec = int((t - int(t)) * 1e9)
                pt.velocities = []
                pt.accelerations = []

            if not self.node.ac.wait_for_server(timeout_sec=5.0):
                self._sag("arm_controller fehlt", "red")
                return
            g = FollowJointTrajectory.Goal()
            g.trajectory = traj
            f = self.node.ac.send_goal_async(g)
            t0 = time.time()
            while not f.done() and time.time() - t0 < 15.0:
                time.sleep(0.05)
            gh = f.result()
            if gh is None or not gh.accepted:
                self._sag("abgelehnt", "red")
                return
            rf = gh.get_result_async()
            t0 = time.time()
            while not rf.done() and time.time() - t0 < dur + 20.0:
                time.sleep(0.05)
            self._sag("angekommen (Greifer nicht gedreht)", "green")
        except Exception as e:
            self._sag(f"Fehler: {e}", "red")
        finally:
            self.busy = False

    def _sag(self, text, farbe):
        self.status.config(text=text, foreground=farbe)


def main():
    rclpy.init()
    node = Jog()
    t = threading.Thread(target=rclpy.spin, args=(node,), daemon=True)
    t.start()
    time.sleep(1.5)
    app = App(node)
    app.root.mainloop()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
