#!/usr/bin/env python3
"""Kartesischer Jog GUI — myCobot 280 JN.

Bewegt tcp im base frame in X/Y/Z-Schritten, während der Greifer SENKRECHT
(top-down) fixiert bleibt. Jeder Schritt: Ziel = aktuelles_tcp + delta -> numerical IK (gripper_down)
-> FollowJointTrajectory /arm_controller. KEIN send_coords; nutzt MoveIt compute_ik.
Für senkrechtes Berühren / präzise Positionierung.

Verwendung:
    python3 ~/ros2_ws/src/mycobot_calibration/scripts/cartesian_jog.py
"""
import threading
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
import tf2_ros

from mycobot_calibration.numerical_ik import NumericalIK, ARM_JOINTS


class CartJogNode(Node):
    def __init__(self):
        super().__init__('cartesian_jog')
        self.action_client = ActionClient(
            self, FollowJointTrajectory, '/arm_controller/follow_joint_trajectory')
        self.current_joints = [0.0] * 6
        self.joints_ok = False
        self.create_subscription(JointState, '/joint_states', self._on_js, 10)
        self.tf_buffer = tf2_ros.Buffer()
        tf2_ros.TransformListener(self.tf_buffer, self)
        self.ik = NumericalIK(self)

    def _on_js(self, msg: JointState):
        for i, n in enumerate(ARM_JOINTS):
            if n in msg.name:
                idx = msg.name.index(n)
                if idx < len(msg.position):
                    self.current_joints[i] = msg.position[idx]
        self.joints_ok = True

    def tcp_xyz(self):
        try:
            tf = self.tf_buffer.lookup_transform('robot_base', 'tcp', rclpy.time.Time())
            t = tf.transform.translation
            return [t.x, t.y, t.z]
        except Exception:
            return None

    def send_joints(self, positions, duration_s):
        if not self.action_client.wait_for_server(timeout_sec=2.0):
            return False
        traj = JointTrajectory()
        traj.joint_names = list(ARM_JOINTS)
        pt = JointTrajectoryPoint()
        pt.positions = [float(p) for p in positions]
        pt.time_from_start = Duration(
            sec=int(duration_s),
            nanosec=int((duration_s - int(duration_s)) * 1e9))
        traj.points = [pt]
        goal = FollowJointTrajectory.Goal()
        goal.trajectory = traj
        self.action_client.send_goal_async(goal)
        return True


class App:
    def __init__(self, root, node: CartJogNode):
        self.root = root
        self.node = node
        self.busy = False
        root.title('myCobot Kartesischer Jog (SENKRECHT)')

        top = ttk.Frame(root, padding=8); top.pack(fill='x')
        ttk.Label(top, text='Schritt (mm):').pack(side='left')
        self.step_var = tk.DoubleVar(value=5.0)
        ttk.Spinbox(top, from_=0.5, to=50.0, increment=0.5, width=6,
                    textvariable=self.step_var).pack(side='left', padx=4)
        ttk.Label(top, text='Dauer (s):').pack(side='left', padx=(12, 0))
        self.dur_var = tk.DoubleVar(value=2.5)
        ttk.Spinbox(top, from_=0.5, to=15.0, increment=0.5, width=6,
                    textvariable=self.dur_var).pack(side='left', padx=4)

        cur = ttk.Frame(root, padding=8); cur.pack(fill='x')
        self.cur_lbl = ttk.Label(cur, text='TCP: --', font=('TkDefaultFont', 11, 'bold'))
        self.cur_lbl.pack(side='left')

        # Jog-Buttons: +X/-X vorwärts/rückwärts, +Y/-Y links/rechts, +Z/-Z hoch/runter
        grid = ttk.Frame(root, padding=8); grid.pack()
        def jbtn(txt, ax, sgn, r, c, col='black'):
            b = tk.Button(grid, text=txt, width=10, height=2,
                          command=lambda: self.jog(ax, sgn))
            b.grid(row=r, column=c, padx=4, pady=4)
        jbtn('+X (vorwaerts)', 0, +1, 0, 1)
        jbtn('-X (rueckwaerts)',  0, -1, 2, 1)
        jbtn('+Y (links)',   1, +1, 1, 0)
        jbtn('-Y (rechts)',   1, -1, 1, 2)
        jbtn('+Z (hoch)',2, +1, 0, 3)
        jbtn('-Z (runter)', 2, -1, 2, 3)

        # Absolut anfahren
        ab = ttk.Frame(root, padding=8); ab.pack(fill='x')
        ttk.Label(ab, text='Absolut anfahren (mm)  x:').pack(side='left')
        self.gx = tk.StringVar(); ttk.Entry(ab, textvariable=self.gx, width=6).pack(side='left')
        ttk.Label(ab, text='y:').pack(side='left')
        self.gy = tk.StringVar(); ttk.Entry(ab, textvariable=self.gy, width=6).pack(side='left')
        ttk.Label(ab, text='z:').pack(side='left')
        self.gz = tk.StringVar(); ttk.Entry(ab, textvariable=self.gz, width=6).pack(side='left')
        ttk.Button(ab, text='FAHRE', command=self.goto_abs).pack(side='left', padx=6)

        self.status = ttk.Label(root, text='bereit', padding=8, foreground='gray')
        self.status.pack(fill='x')
        self.root.after(200, self._refresh)

    def _refresh(self):
        p = self.node.tcp_xyz()
        if p:
            self.cur_lbl.config(
                text=f'TCP:  x={p[0]*1000:.0f}  y={p[1]*1000:.0f}  z={p[2]*1000:.0f} mm')
        self.root.after(200, self._refresh)

    def jog(self, axis, sgn):
        p = self.node.tcp_xyz()
        if p is None:
            self.status.config(text='kein TF', foreground='red'); return
        step = max(0.5, float(self.step_var.get())) / 1000.0 * sgn
        target = list(p); target[axis] += step
        self._move(target, f'jog {"XYZ"[axis]}{"+" if sgn>0 else "-"}')

    def goto_abs(self):
        try:
            target = [float(self.gx.get())/1000.0, float(self.gy.get())/1000.0,
                      float(self.gz.get())/1000.0]
        except ValueError:
            self.status.config(text='ungueltige xyz', foreground='red'); return
        self._move(target, 'absolut anfahren')

    def _move(self, target, label):
        if self.busy:
            self.status.config(text='beschaeftigt, warten', foreground='orange'); return
        self.busy = True
        self.status.config(
            text=f'{label}: IK wird geloest -> ({target[0]*1000:.0f},{target[1]*1000:.0f},{target[2]*1000:.0f})',
            foreground='blue')
        dur = max(0.5, float(self.dur_var.get()))
        threading.Thread(target=self._solve_and_send,
                         args=(target, dur, label), daemon=True).start()

    def _solve_and_send(self, target, dur, label):
        sol = self.node.ik.solve_multi_seed(
            target, gripper_down=True, current_joints=list(self.node.current_joints))
        if sol is None:
            self._status(f'{label}: KEINE IK-LOESUNG (ausser Reichweite?)', 'red')
            self.busy = False; return
        ok = self.node.send_joints(sol, dur)
        if ok:
            self._status(
                f'{label}: GESENDET -> ({target[0]*1000:.0f},{target[1]*1000:.0f},{target[2]*1000:.0f}) mm, {dur:.1f}s', 'green')
        else:
            self._status(f'{label}: kein action server', 'red')
        time.sleep(dur)
        self.busy = False

    def _status(self, txt, color):
        self.root.after(0, lambda: self.status.config(text=txt, foreground=color))


def main():
    rclpy.init()
    node = CartJogNode()
    threading.Thread(target=rclpy.spin, args=(node,), daemon=True).start()
    t0 = time.time()
    while not node.joints_ok and time.time() - t0 < 3.0:
        time.sleep(0.1)
    root = tk.Tk()
    App(root, node)
    try:
        root.mainloop()
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
