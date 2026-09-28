#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""Handjog-Fenster: Arm in kleinen Schritten fahren - Gelenke UND kartesisch (dx/dy/dz/radial).

Schrittweite (mm bzw. Grad) und Fahrzeit (s) stellt der Benutzer im Fenster ein.

Regeln (aus den Nachtversuchen 2026-09-11):
  * Jeder Schritt rechnet auf SOLL (den zuletzt kommandierten Gelenkwinkeln), NICHT auf Ist -
    sonst wandert das Durchhaengen von Achse 2 mit jedem Schritt mit (tools/versetze_soll.py).
  * Kartesische Schritte laufen als Unterprozess ueber tools/versetze_soll.py (MoveIt-Bahn ab
    Soll, kollisionsgeprueft, erster Bahnpunkt = Ist, danach optional nachstellen). Das Fenster
    bleibt dabei bedienbar; die Ausgabe des Unterprozesses landet im Protokollfeld.
  * Gelenkschritte gehen direkt an den arm_controller (KEINE Kollisionspruefung - hinsehen).
  * "Senkrecht" = tools/senkrecht_stellen.py --hier auf der Soll-Hoehe (Greifer lotrecht).

    python3 tools/jog_gui.py            (Stack muss laufen; DISPLAY=:0 auf dem Jetson)
"""
import math, os, subprocess, sys, threading, time
import tkinter as tk
from tkinter import ttk

import rclpy
from rclpy.action import ActionClient
from rclpy.node import Node
from builtin_interfaces.msg import Duration
from control_msgs.action import FollowJointTrajectory
from trajectory_msgs.msg import JointTrajectoryPoint
from sensor_msgs.msg import JointState
from moveit_msgs.msg import RobotState
from moveit_msgs.srv import GetPositionFK

ARM = ['joint2_to_joint1', 'joint3_to_joint2', 'joint4_to_joint3',
       'joint5_to_joint4', 'joint6_to_joint5', 'joint6output_to_joint6']
TOOLS = os.path.dirname(os.path.abspath(__file__))
GREIFER_AUF, GREIFER_ZU = 0.15, -0.62      # Bereich [-0.74, 0.15]; -0.62 = greifer_zu (Welle)


class Knoten(Node):
    def __init__(s):
        super().__init__('jog_gui')
        s.ist = None; s.greifer_ist = None
        s.create_subscription(JointState, '/joint_states', s._js, 10)
        s.arm = ActionClient(s, FollowJointTrajectory, '/arm_controller/follow_joint_trajectory')
        s.grp = ActionClient(s, FollowJointTrajectory, '/gripper_controller/follow_joint_trajectory')
        s.fk = s.create_client(GetPositionFK, '/compute_fk')

    def _js(s, m):
        d = dict(zip(m.name, m.position))
        if all(j in d for j in ARM): s.ist = [d[j] for j in ARM]
        if 'gripper_controller' in d: s.greifer_ist = d['gripper_controller']

    def fahre(s, rad, dauer):
        if not s.arm.wait_for_server(timeout_sec=2.0): return False
        g = FollowJointTrajectory.Goal(); g.trajectory.joint_names = list(ARM)
        pt = JointTrajectoryPoint(); pt.positions = [float(v) for v in rad]
        pt.time_from_start = Duration(sec=int(dauer), nanosec=int((dauer % 1) * 1e9))
        g.trajectory.points = [pt]; s.arm.send_goal_async(g); return True

    def greifer(s, wert, dauer=3.0):
        if not s.grp.wait_for_server(timeout_sec=2.0): return False
        g = FollowJointTrajectory.Goal(); g.trajectory.joint_names = ['gripper_controller']
        pt = JointTrajectoryPoint(); pt.positions = [float(wert)]; pt.time_from_start = Duration(sec=int(dauer))
        g.trajectory.points = [pt]; s.grp.send_goal_async(g); return True

    def fk_async(s, rad):
        """Future fuer TCP-Pose (robot_base) der Gelenke rad; None wenn Dienst fehlt."""
        if not s.fk.service_is_ready(): return None
        rs = RobotState(); rs.joint_state.name = list(ARM); rs.joint_state.position = [float(v) for v in rad]
        rq = GetPositionFK.Request(); rq.header.frame_id = 'robot_base'; rq.fk_link_names = ['tcp']; rq.robot_state = rs
        return s.fk.call_async(rq)


class Fenster:
    def __init__(s, root, kn):
        s.root, s.kn = root, kn
        s.soll = list(kn.ist) if kn.ist else [0.0] * 6      # SOLL: zuletzt kommandierte Gelenke
        s.proz = None; s.fk_fut = {}
        root.title('Handjog myCobot')

        # --- Einstellungen ------------------------------------------------
        e = ttk.LabelFrame(root, text='Einstellungen', padding=6); e.pack(fill='x', padx=6, pady=4)
        s.mm = tk.DoubleVar(value=5.0); s.grad = tk.DoubleVar(value=2.0); s.dauer = tk.DoubleVar(value=4.0)
        s.nachst = tk.BooleanVar(value=True)
        for txt, var, lo, hi, inc in (('Schritt [mm]', s.mm, 0.5, 50, 0.5), ('Schritt [Grad]', s.grad, 0.1, 30, 0.5),
                                      ('Fahrzeit [s]', s.dauer, 1.0, 20, 0.5)):
            ttk.Label(e, text=txt).pack(side='left', padx=(8, 2))
            ttk.Spinbox(e, from_=lo, to=hi, increment=inc, textvariable=var, width=6).pack(side='left')
        ttk.Checkbutton(e, text='nachstellen (Totband)', variable=s.nachst).pack(side='left', padx=12)

        # --- kartesisch ---------------------------------------------------
        k = ttk.LabelFrame(root, text='Kartesisch (robot_base, ab SOLL, MoveIt-Bahn, kollisionsgeprueft)', padding=6)
        k.pack(fill='x', padx=6, pady=4)
        for r, (name, arg) in enumerate((('dx', '--dx'), ('dy', '--dy'), ('dz', '--dz'), ('radial (zum Roboter -)', '--radial'))):
            ttk.Label(k, text=name, width=22, anchor='w').grid(row=r, column=0)
            ttk.Button(k, text='-', width=6, command=lambda a=arg: s.kart(a, -1)).grid(row=r, column=1, padx=2, pady=1)
            ttk.Button(k, text='+', width=6, command=lambda a=arg: s.kart(a, +1)).grid(row=r, column=2, padx=2, pady=1)
        ttk.Button(k, text='Senkrecht (Greifer lotrecht, gleiche Hoehe)', command=s.senkrecht).grid(row=0, column=3, rowspan=2, padx=16)
        ttk.Button(k, text='Nur nachstellen (auf SOLL)', command=s.nur_nachstellen).grid(row=2, column=3, rowspan=2, padx=16)

        # --- Gelenke ------------------------------------------------------
        g = ttk.LabelFrame(root, text='Gelenke (direkt, OHNE Kollisionspruefung)', padding=6); g.pack(fill='x', padx=6, pady=4)
        s.l_soll, s.l_ist = [], []
        for i in range(6):
            ttk.Label(g, text=f'J{i+1}', width=4).grid(row=i, column=0)
            ttk.Button(g, text='-', width=6, command=lambda i=i: s.gelenk(i, -1)).grid(row=i, column=1, padx=2)
            ttk.Button(g, text='+', width=6, command=lambda i=i: s.gelenk(i, +1)).grid(row=i, column=2, padx=2)
            a = ttk.Label(g, text='', width=16, anchor='e'); a.grid(row=i, column=3); s.l_soll.append(a)
            b = ttk.Label(g, text='', width=16, anchor='e', foreground='gray'); b.grid(row=i, column=4); s.l_ist.append(b)
        ttk.Label(g, text='Soll [Grad]', foreground='gray').grid(row=6, column=3); ttk.Label(g, text='Ist [Grad]', foreground='gray').grid(row=6, column=4)

        # --- Greifer / Soll -------------------------------------------------
        h = ttk.Frame(root, padding=6); h.pack(fill='x')
        ttk.Button(h, text='Greifer AUF', command=lambda: s.greifer(GREIFER_AUF)).pack(side='left', padx=4)
        ttk.Button(h, text='Greifer ZU', command=lambda: s.greifer(GREIFER_ZU)).pack(side='left', padx=4)
        s.l_greifer = ttk.Label(h, text='', width=14); s.l_greifer.pack(side='left', padx=8)
        ttk.Button(h, text='SOLL <- Ist uebernehmen', command=s.soll_von_ist).pack(side='right', padx=4)
        ttk.Button(h, text='Soll kopieren (Terminal)', command=s.soll_drucken).pack(side='right', padx=4)

        s.l_tcp = ttk.Label(root, text='TCP: -', padding=(8, 2)); s.l_tcp.pack(fill='x')
        s.status = ttk.Label(root, text='bereit', padding=(8, 2), foreground='gray'); s.status.pack(fill='x')
        s.log = tk.Text(root, height=9, width=96, font=('monospace', 8)); s.log.pack(fill='both', expand=True, padx=6, pady=(0, 6))
        s.root.after(300, s.tick)

    # --- Helfer --------------------------------------------------------------
    def melde(s, txt, farbe='black'): s.status.config(text=txt, foreground=farbe)
    def schreibe(s, txt):
        s.log.insert('end', txt); s.log.see('end')
    def beschaeftigt(s):
        if s.proz is not None and s.proz.poll() is None:
            s.melde('noch beschaeftigt - warten', 'red'); return True
        return False
    def grad_str(s, rad): return ' '.join(f'{math.degrees(v):+.2f}' for v in rad)

    # --- Aktionen ------------------------------------------------------------
    def gelenk(s, i, sgn):
        if s.beschaeftigt(): return
        s.soll[i] += sgn * math.radians(s.grad.get())
        if s.kn.fahre(s.soll, max(1.0, s.dauer.get())): s.melde(f'J{i+1} {sgn*s.grad.get():+.1f} Grad -> Soll {s.grad_str(s.soll)}', 'blue')
        else: s.melde('arm_controller nicht erreichbar', 'red')

    def greifer(s, wert):
        s.kn.greifer(wert); s.melde(f'Greifer -> {wert:+.2f}', 'blue')

    def soll_von_ist(s):
        if s.kn.ist is None: s.melde('keine /joint_states', 'red'); return
        s.soll = list(s.kn.ist); s.melde('SOLL <- Ist: ' + s.grad_str(s.soll), 'blue')

    def soll_drucken(s):
        print('SOLL [Grad]:', s.grad_str(s.soll)); s.melde('Soll im Terminal ausgegeben', 'blue')

    def _starte(s, cmd, was):
        if s.beschaeftigt(): return
        s.schreibe(f'\n$ {" ".join(cmd[1:])}\n')
        s.proz = subprocess.Popen(cmd, cwd=os.path.dirname(TOOLS), stdout=subprocess.PIPE, stderr=subprocess.STDOUT,
                                  text=True, bufsize=1, env=dict(os.environ, PYTHONUNBUFFERED='1'))
        s.proz_was = was; s.melde(f'{was} laeuft ...', 'blue')
        threading.Thread(target=s._lese, args=(s.proz,), daemon=True).start()

    def _lese(s, p):
        """Unterprozess-Ausgabe mitschreiben; 'NEUE SOLL-GELENKE'/'Gelenke' liefern das neue Soll."""
        for zeile in p.stdout:
            if 'multicast' in zeile or 'Exception ignored' in zeile or 'Traceback' in zeile or zeile.startswith('  File') or 'InvalidHandle' in zeile:
                continue
            s.root.after(0, s.schreibe, zeile)
            if zeile.startswith('  NEUE SOLL-GELENKE'):
                s.neu_soll = [math.radians(float(v)) for v in zeile.split(':')[1].split()]
            elif '->  Gelenke' in zeile:                     # senkrecht_stellen.py: "... ->  Gelenke  a b c d e f"
                s.neu_soll = [math.radians(float(v)) for v in zeile.split('Gelenke')[1].split()]
        p.wait(); s.root.after(0, s._fertig, p.returncode)

    def _fertig(s, rc):
        if rc == 0 and getattr(s, 'neu_soll', None):
            s.soll = s.neu_soll; s.neu_soll = None; s.melde(f'{s.proz_was}: fertig, Soll uebernommen', 'green')
        elif rc == 0: s.melde(f'{s.proz_was}: fertig', 'green')
        else: s.melde(f'{s.proz_was}: FEHLER (Exit {rc}) - Soll unveraendert, Protokoll lesen', 'red')

    def kart(s, arg, sgn):
        s.neu_soll = None
        cmd = [sys.executable, os.path.join(TOOLS, 'versetze_soll.py'), '--grad', *[f'{math.degrees(v):.3f}' for v in s.soll],
               arg, f'{sgn * s.mm.get():.2f}', '--dauer', f'{max(1.0, s.dauer.get()):.1f}']
        if not s.nachst.get(): cmd.append('--ohne-nachstellen')
        s._starte(cmd, f'{arg[2:]} {sgn*s.mm.get():+.1f} mm')

    def senkrecht(s):
        z = s.tcp_soll[2] if getattr(s, 'tcp_soll', None) else None
        if z is None: s.melde('Soll-TCP noch unbekannt (FK)', 'red'); return
        s.neu_soll = None
        s._starte([sys.executable, os.path.join(TOOLS, 'senkrecht_stellen.py'), '--hier', '--z', f'{z:.1f}'], 'senkrecht')

    def nur_nachstellen(s):
        s.neu_soll = None
        s._starte([sys.executable, os.path.join(TOOLS, 'nachstellen.py'), '--ziel', *[f'{math.degrees(v):.3f}' for v in s.soll]], 'nachstellen')

    # --- Anzeige ---------------------------------------------------------------
    def tick(s):
        for i in range(6):
            s.l_soll[i].config(text=f'{math.degrees(s.soll[i]):+8.2f}')
            if s.kn.ist: s.l_ist[i].config(text=f'{math.degrees(s.kn.ist[i]):+8.2f}', foreground='black')
        if s.kn.greifer_ist is not None: s.l_greifer.config(text=f'Greifer {s.kn.greifer_ist:+.2f}')
        # FK fuer Soll und Ist (asynchron, jede Runde eine Anfrage je Stellung)
        for name, rad in (('soll', s.soll), ('ist', s.kn.ist)):
            fut = s.fk_fut.get(name)
            if fut is not None and fut.done():
                r = fut.result(); s.fk_fut[name] = None
                if r is not None and r.pose_stamped:
                    p = r.pose_stamped[0].pose.position; setattr(s, 'tcp_' + name, (p.x * 1e3, p.y * 1e3, p.z * 1e3))
            elif fut is None and rad is not None:
                s.fk_fut[name] = s.kn.fk_async(rad)
        ts, ti = getattr(s, 'tcp_soll', None), getattr(s, 'tcp_ist', None)
        f = lambda t: '-' if t is None else f'x {t[0]:+.1f}  y {t[1]:+.1f}  z {t[2]:+.1f}'
        s.l_tcp.config(text=f'TCP Soll: {f(ts)}   |   TCP Ist (Encoder): {f(ti)}   [mm, robot_base]')
        s.root.after(300, s.tick)


def main():
    rclpy.init(); kn = Knoten()
    threading.Thread(target=rclpy.spin, args=(kn,), daemon=True).start()
    t0 = time.time()
    while kn.ist is None and time.time() - t0 < 3: time.sleep(0.1)
    root = tk.Tk(); Fenster(root, kn)
    try: root.mainloop()
    finally: kn.destroy_node(); rclpy.shutdown()


if __name__ == '__main__':
    main()
