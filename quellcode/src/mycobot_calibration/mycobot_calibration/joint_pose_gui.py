#!/usr/bin/env python3
"""Tkinter-GUI mit Gelenk-Schiebereglern für den myCobot 280 JN.

Zweck
-----
Dieses Werkzeug umgeht MoveIt vollständig: Jeder Klick auf "Send" schickt
ein FollowJointTrajectory-Goal direkt an den ``/arm_controller``. Dadurch
entfällt die gesamte Bewegungsplanung (Kollisionsprüfung, OMPL usw.), und
der Roboter fährt exakt die per Regler eingestellten Gelenkwinkel an.

Motivation (Hand-Auge-Kalibrierung)
-----------------------------------
Für die Hand-Auge-Kalibrierung (eye-to-hand) muss der Roboter in vielen
verschiedenen, präzise reproduzierbaren Posen stehen, während jeweils ein
Kalibriermuster (z. B. ArUco/Charuco) von der Kamera erfasst wird. Ein
direkter Trajektorien-Befehl hält die Servos anschließend unter Drehmoment
(verriegelt), sodass der Arm während der Bildaufnahme absolut still steht.
Das ist entscheidend, weil jede Restbewegung die Rotations-/Translations-
Schätzung der Kalibrierung verfälschen würde.

Aufruf
------
    ros2 run mycobot_calibration joint_pose_gui
        (oder)
    python3 joint_pose_gui.py
"""

import math
import threading  # ROS-Spin läuft in eigenem Thread, damit die Tk-GUI nicht blockiert
import time
import tkinter as tk
from tkinter import ttk

import rclpy
from rclpy.action import ActionClient
from rclpy.node import Node

from builtin_interfaces.msg import Duration
from control_msgs.action import FollowJointTrajectory
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint
from sensor_msgs.msg import JointState


# Gelenknamen in der Reihenfolge J1..J6, exakt wie sie im URDF/Controller
# definiert sind. Diese Strings MUESSEN mit den Namen im FollowJointTrajectory-
# Goal und in den /joint_states übereinstimmen, sonst ordnet der Controller
# die Positionen falsch zu.
ARM_JOINT_NAMES = [
    'joint2_to_joint1',
    'joint3_to_joint2',
    'joint4_to_joint3',
    'joint5_to_joint4',
    'joint6_to_joint5',
    'joint6output_to_joint6',
]
# Konservative, symmetrische Reglergrenze (+/- pi) für alle Gelenke.
# Bewusst grob gehalten: Die exakten (asymmetrischen) URDF-Grenzen werden hier
# NICHT durchgesetzt — der Anwender ist für sinnvolle Posen verantwortlich, da
# ohne MoveIt keine Grenzwert-/Kollisionsprüfung stattfindet.
JOINT_LIMIT_RAD = math.pi


class JointPoseNode(Node):
    """ROS-2-Knoten: Kapselt die Kommunikation mit dem Arm-Controller.

    Verantwortlichkeiten:
      * Halten eines Action-Clients für FollowJointTrajectory, um Ziel-Posen
        zu senden.
      * Abonnieren von ``/joint_states``, um die aktuelle Ist-Pose des Roboters
        zu kennen (für die Anzeige und die "Sync"-Funktion der GUI).

    Der Knoten enthält bewusst KEINE Tkinter-Logik — Trennung von ROS-Schicht
    (dieser Knoten) und Präsentationsschicht (Klasse ``App``).
    """

    def __init__(self):
        super().__init__('joint_pose_gui')
        # Action-Client auf den Standard-Endpunkt des JointTrajectoryController.
        self.action_client = ActionClient(
            self, FollowJointTrajectory, '/arm_controller/follow_joint_trajectory'
        )
        # Zwischenspeicher der zuletzt empfangenen Ist-Gelenkwinkel (J1..J6).
        self.current_positions = [0.0] * 6
        # Flag: True, sobald mindestens eine /joint_states-Nachricht kam.
        # Verhindert, dass die GUI mit undefinierten Nullwerten synchronisiert.
        self.joint_states_received = False
        self.create_subscription(
            JointState, '/joint_states', self._on_joint_states, 10
        )

    def _on_joint_states(self, msg: JointState):
        """Callback für ``/joint_states``: aktualisiert die Ist-Positionen.

        Die Reihenfolge der Gelenke in der Nachricht ist NICHT garantiert
        (sie kann Greifer-Gelenke enthalten oder anders sortiert sein), daher
        wird die Zuordnung über den Gelenknamen vorgenommen und nicht über
        den Index.
        """
        # Zuordnung über den Namen (Reihenfolge kann variieren)
        for i, name in enumerate(ARM_JOINT_NAMES):
            if name in msg.name:
                idx = msg.name.index(name)
                # Schutz gegen inkonsistente Nachrichten, bei denen das
                # position-Array kürzer ist als das name-Array.
                if idx < len(msg.position):
                    self.current_positions[i] = msg.position[idx]
        self.joint_states_received = True

    def send_goal(self, positions, duration_s):
        """Sendet eine Ein-Punkt-Trajektorie als Ziel an den Controller.

        Es wird ein einziger JointTrajectoryPoint erzeugt: der Controller
        interpoliert selbst von der aktuellen Pose zum Ziel innerhalb der
        vorgegebenen Dauer ``duration_s``. Eine längere Dauer bedeutet also
        eine langsamere, sanftere Bewegung — wichtig, um die relativ schwachen
        Servos des myCobot 280 JN nicht zu überlasten.

        :param positions: Iterable mit 6 Zielwinkeln [rad] in Reihenfolge J1..J6.
        :param duration_s: Fahrzeit [s] von der Ist- zur Zielpose.
        :return: True bei erfolgreichem Absenden, False falls der Action-Server
                 nicht erreichbar ist.
        """
        # Kurzes Warten auf den Action-Server; schlägt fehl, wenn der
        # Controller nicht läuft (z. B. bridge/ros2_control nicht gestartet).
        if not self.action_client.wait_for_server(timeout_sec=2.0):
            self.get_logger().error('arm_controller action server not available')
            return False
        traj = JointTrajectory()
        traj.joint_names = list(ARM_JOINT_NAMES)
        pt = JointTrajectoryPoint()
        # Explizite float-Konvertierung: Tk-DoubleVar/Slider können numpy- oder
        # int-artige Werte liefern, die die ROS-Serialisierung ablehnen würde.
        pt.positions = [float(p) for p in positions]
        # time_from_start = absoluter Zeitpunkt (relativ zum Trajektorienstart),
        # zu dem dieser Punkt erreicht sein soll. Aufteilung in ganze Sekunden
        # und Nanosekunden-Rest, wie es die builtin_interfaces/Duration verlangt.
        pt.time_from_start = Duration(
            sec=int(duration_s),
            nanosec=int((duration_s - int(duration_s)) * 1e9),
        )
        traj.points = [pt]
        goal = FollowJointTrajectory.Goal()
        goal.trajectory = traj
        # Asynchrones Senden: Wir warten NICHT auf das Ergebnis, damit die GUI
        # reaktiv bleibt. Ein Feedback-/Result-Handling ist für diesen
        # Kalibrier-Anwendungsfall nicht nötig.
        self.action_client.send_goal_async(goal)
        return True


class App:
    """Tkinter-Präsentationsschicht: baut die GUI und verknüpft sie mit dem Knoten.

    Aufbau der Oberfläche (von oben nach unten):
      1. Spinbox für die Fahrzeit (Move duration).
      2. Sechs Zeilen mit je einem Schieberegler pro Gelenk, dem Sollwert-Label
         und dem Istwert-Label.
      3. Aktions-Buttons (Senden, Nullstellung, Synchronisieren).
      4. Eine Statuszeile für Rückmeldungen.
    """

    def __init__(self, root: tk.Tk, node: JointPoseNode):
        self.root = root
        self.node = node
        self.root.title('myCobot Joint Pose')
        # Parallele Listen, indexiert nach Gelenk J1..J6:
        self.sliders = []          # Tk-DoubleVars der Schieberegler (Sollwerte)
        self.value_labels = []     # Labels, die den Reglerwert numerisch zeigen
        self.current_labels = []   # Labels, die den Istwert aus /joint_states zeigen

        # --- Spinbox für die Fahrzeit -----------------------------------
        # Bestimmt, wie lange die Bewegung zur Zielpose dauert (s. send_goal).
        top = ttk.Frame(root, padding=8)
        top.pack(fill='x')
        ttk.Label(top, text='Move duration (s):').pack(side='left')
        self.duration_var = tk.DoubleVar(value=3.0)
        ttk.Spinbox(top, from_=0.5, to=20.0, increment=0.5,
                    textvariable=self.duration_var, width=6).pack(side='left', padx=4)

        # --- Ein Schieberegler pro Gelenk --------------------------------
        for i, name in enumerate(ARM_JOINT_NAMES):
            frame = ttk.Frame(root, padding=4)
            frame.pack(fill='x')

            ttk.Label(frame, text=f'J{i+1} ({name})', width=30, anchor='w').pack(side='left')

            var = tk.DoubleVar(value=0.0)
            slider = ttk.Scale(frame, from_=-JOINT_LIMIT_RAD, to=JOINT_LIMIT_RAD,
                               variable=var, orient='horizontal', length=300,
                               # idx=i bindet den aktuellen Schleifenindex fest
                               # (Late-Binding-Falle bei Lambdas in Schleifen).
                               command=lambda v, idx=i: self._on_slider_change(idx, float(v)))
            slider.pack(side='left', padx=4)
            self.sliders.append(var)

            # Zeigt den eingestellten SOLL-Wert des Reglers an.
            value_label = ttk.Label(frame, text='+0.000 rad', width=14, anchor='e')
            value_label.pack(side='left')
            self.value_labels.append(value_label)

            # Zeigt den IST-Wert (grau, bis Daten vorliegen; siehe _refresh_current).
            current_label = ttk.Label(frame, text='cur: --', width=14,
                                      anchor='e', foreground='gray')
            current_label.pack(side='left')
            self.current_labels.append(current_label)

        # --- Aktions-Buttons ---------------------------------------------
        btn_frame = ttk.Frame(root, padding=8)
        btn_frame.pack(fill='x')
        ttk.Button(btn_frame, text='Send to robot',
                   command=self.send).pack(side='left', padx=4)
        ttk.Button(btn_frame, text='All zero',
                   command=self.zero).pack(side='left', padx=4)
        ttk.Button(btn_frame, text='Sync sliders ← current',
                   command=self.sync_from_current).pack(side='left', padx=4)

        self.status = ttk.Label(root, text='ready', padding=8, foreground='gray')
        self.status.pack(fill='x')

        # Zyklische Aktualisierung der Ist-Positions-Labels über den
        # Tk-Event-Loop (kein eigener Thread nötig -> Tk ist nicht thread-safe).
        self.root.after(200, self._refresh_current)

    def _on_slider_change(self, idx, val):
        """Callback beim Verschieben eines Reglers: aktualisiert das Soll-Label."""
        self.value_labels[idx].config(text=f'{val:+.3f} rad')

    def _refresh_current(self):
        """Periodischer Tk-Timer (alle 200 ms): schreibt die Ist-Winkel in die GUI.

        Liest die vom ROS-Thread gefüllten ``current_positions`` und stellt sie
        dar. Sobald echte Daten vorliegen, wechselt die Schriftfarbe von grau
        auf schwarz. Der Aufruf plant sich am Ende selbst neu ein (Tk-Idiom für
        wiederkehrende Aufgaben).
        """
        if self.node.joint_states_received:
            for i, v in enumerate(self.node.current_positions):
                self.current_labels[i].config(
                    text=f'cur: {v:+.3f}', foreground='black'
                )
        self.root.after(200, self._refresh_current)

    def send(self):
        """Liest alle Regler aus und sendet die Zielpose an den Roboter."""
        positions = [v.get() for v in self.sliders]
        # Untergrenze von 0.5 s erzwingen: zu kurze Fahrzeiten führen zu
        # ruckartigen, für die Servos schädlichen Bewegungen bzw. zu vom
        # Controller verworfenen Trajektorien.
        duration = max(0.5, float(self.duration_var.get()))
        if self.node.send_goal(positions, duration):
            self.status.config(
                text=f'Sent: [{", ".join(f"{p:+.3f}" for p in positions)}]  in {duration:.1f}s',
                foreground='blue',
            )
        else:
            self.status.config(text='ERROR: action server not available',
                               foreground='red')

    def zero(self):
        """Setzt alle Regler auf 0 rad und fährt die Nullstellung an."""
        for v in self.sliders:
            v.set(0.0)
        for i, lbl in enumerate(self.value_labels):
            lbl.config(text='+0.000 rad')
        self.send()

    def sync_from_current(self):
        """Übernimmt die aktuelle Ist-Pose des Roboters in die Regler.

        Nützlich, um von der realen Stellung des Arms aus weiterzuarbeiten
        (statt von 0), ohne dass beim nächsten "Send" ein unerwarteter Sprung
        entsteht. Bricht ab, falls noch keine /joint_states empfangen wurden.
        """
        if not self.node.joint_states_received:
            self.status.config(text='no joint_states yet', foreground='red')
            return
        for i, v in enumerate(self.node.current_positions):
            self.sliders[i].set(v)
            self.value_labels[i].config(text=f'{v:+.3f} rad')
        self.status.config(text='sliders ← current pose', foreground='blue')


def ros_spin(node):
    """Ziel-Funktion des Hintergrund-Threads: verarbeitet ROS-Callbacks.

    ``rclpy.spin`` blockiert, solange der Knoten lebt. Es läuft deshalb in
    einem eigenen Thread, damit der Haupt-Thread ungestört den Tkinter-
    Event-Loop (``mainloop``) betreiben kann.
    """
    rclpy.spin(node)


def main():
    """Einstiegspunkt: initialisiert ROS, startet Spin-Thread und die GUI."""
    rclpy.init()
    node = JointPoseNode()
    # ROS-Spin als Daemon-Thread: endet automatisch mit dem Hauptprogramm,
    # ohne dass ein explizites Join nötig ist.
    spin_thread = threading.Thread(target=ros_spin, args=(node,), daemon=True)
    spin_thread.start()

    # Kurz warten, bis die ersten /joint_states eintreffen, damit die GUI
    # direkt mit der realen Roboter-Pose starten kann (siehe unten).
    deadline = time.time() + 2.0
    while not node.joint_states_received and time.time() < deadline:
        time.sleep(0.1)

    root = tk.Tk()
    app = App(root, node)
    # Falls Ist-Daten vorliegen: Regler initial auf die aktuelle Pose setzen,
    # um einen Sprung beim ersten "Send" zu vermeiden.
    if node.joint_states_received:
        app.sync_from_current()
    try:
        # Blockiert bis zum Schließen des Fensters.
        root.mainloop()
    finally:
        # Sauberes Herunterfahren des ROS-Knotens, egal wie die GUI endet
        # (verhindert hängende Knoten / Ressourcenlecks).
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
