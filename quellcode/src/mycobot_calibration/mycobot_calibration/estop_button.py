#!/usr/bin/env python3
# NOT-AUS Taster — eigenständiges, immer-im-Vordergrund Fenster.
#
# Großer roter Knopf -> publiziert /estop=true (STOP). Der Bridge hält sofort
# jede Bewegung an und blockiert weitere Kommandos. Grüner Knopf -> /estop=false
# (FREIGABE). Läuft UNABHAENGIG von RViz — bleibt aktiv, auch wenn RViz auf dem
# Jetson einfriert/abstürzt (deshalb eigenes Fenster statt nur RViz-Panel).
#
# /estop ist transient-local (gelatcht): ein später gestarteter Bridge übernimmt
# sofort den letzten Zustand.
#
# KETTEN-RUECKMELDUNG (Vorfall 2026-07-16): der Taster ZEIGT jetzt, ob der Stopp
# wirklich beim Bridge angekommen ist. estop_relay publiziert /estop_ack, sobald
# das Kommando auf dem Bridge-Socket zugestellt wurde. Solange kein Ack kommt,
# zeigt das Fenster "KEINE BRIDGE-ANTWORT" — dann sofort NETZSCHALTER benutzen!
# Merke: waehrend des Skript-AUFRAEUMENS (nach Ctrl+C) sind relay+bridge tot —
# in dieser Phase kann KEIN Software-Not-Aus wirken.
#
# Alle Meldungen werden zusaetzlich mit print(flush=True) geschrieben — die
# rcutils-stdout-Pufferung hatte die Log-Datei sonst leer gelassen (Forensik!).
#
# Start:  ros2 run mycobot_calibration estop_button
# Terminal-Alternative (immer verfügbar):
#   ros2 topic pub --once /estop std_msgs/msg/Bool "{data: true}"

import ctypes
import time
import tkinter as tk

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, DurabilityPolicy, ReliabilityPolicy
from std_msgs.msg import Bool


def _mlock_current():
    """Bereits belegten Prozess-Speicher sperren (best effort) — unter RAM-Druck/
    Swap-Thrash auf dem Nano bleibt das Not-Aus-Fenster sonst sekundenlang
    eingefroren. NUR MCL_CURRENT (=1)! MCL_FUTURE liesse pthread_create der
    DDS-Threads mit EAGAIN scheitern (Test 2026-07-16: Publisher war dann tot) —
    deshalb erst NACH rclpy.init/Node-Aufbau aufrufen und Zukunft nicht sperren."""
    try:
        libc = ctypes.CDLL("libc.so.6", use_errno=True)
        if libc.mlockall(1) != 0:
            print("NOT-AUS: mlockall nicht moeglich (Limit/Rechte) — weiter ohne", flush=True)
    except Exception as e:  # noqa: BLE001
        print(f"NOT-AUS: mlockall Fehler: {e} — weiter ohne", flush=True)


class EstopPublisher(Node):
    def __init__(self) -> None:
        super().__init__('estop_button')
        q = QoSProfile(depth=1)
        q.durability = DurabilityPolicy.TRANSIENT_LOCAL
        q.reliability = ReliabilityPolicy.RELIABLE
        self._pub = self.create_publisher(Bool, '/estop', q)
        # Ack vom estop_relay: Kommando wurde WIRKLICH auf den Bridge-Socket geschrieben.
        self.ack_time = 0.0
        self.ack_state = None
        self.create_subscription(Bool, '/estop_ack', self._on_ack, q)

    def _on_ack(self, msg: Bool) -> None:
        self.ack_time = time.monotonic()
        self.ack_state = bool(msg.data)

    def send(self, active: bool) -> None:
        self._pub.publish(Bool(data=active))
        txt = 'NOT-AUS -> STOP' if active else 'NOT-AUS -> FREIGABE'
        self.get_logger().warn(txt)
        print(txt, flush=True)   # rcutils-stdout ist gepuffert; das hier landet sofort in der Log-Datei


def main() -> None:
    rclpy.init()
    node = EstopPublisher()
    _mlock_current()   # NACH dem DDS-Aufbau (siehe Docstring: MCL_FUTURE-Falle)
    print('NOT-AUS-Fenster BEREIT', flush=True)

    root = tk.Tk()
    root.title('NOT-AUS')
    root.attributes('-topmost', True)   # immer im Vordergrund
    root.resizable(False, False)
    root.configure(bg='black')

    status = tk.Label(root, text='BEREIT', font=('DejaVu Sans', 16, 'bold'),
                      fg='white', bg='black')
    # Ketten-Status: kam das Ack des Relays an? (leer solange nichts gesendet wurde)
    chain = tk.Label(root, text='', font=('DejaVu Sans', 11),
                     fg='gray', bg='black')

    state = {'sent': None, 'sent_time': 0.0}

    def do_stop() -> None:
        state['sent'] = True
        state['sent_time'] = time.monotonic()
        node.send(True)
        status.config(text='!!! GESTOPPT !!!', fg='red')
        chain.config(text='warte auf Bridge-Bestaetigung...', fg='yellow')

    def do_reset() -> None:
        state['sent'] = False
        state['sent_time'] = time.monotonic()
        node.send(False)
        status.config(text='FREIGEGEBEN', fg='lime')
        chain.config(text='warte auf Bridge-Bestaetigung...', fg='yellow')

    stop_btn = tk.Button(root, text='NOT-AUS\nSTOP', command=do_stop,
                         font=('DejaVu Sans', 34, 'bold'),
                         bg='red', fg='white', activebackground='#aa0000',
                         activeforeground='white', width=12, height=3, bd=6)
    reset_btn = tk.Button(root, text='FREIGABE (RESET)', command=do_reset,
                          font=('DejaVu Sans', 14, 'bold'),
                          bg='#004400', fg='lime', activebackground='#006600',
                          activeforeground='white', width=22, height=1, bd=3)

    stop_btn.pack(padx=16, pady=(16, 8))
    reset_btn.pack(padx=16, pady=(0, 8))
    status.pack(padx=16, pady=(0, 4))
    chain.pack(padx=16, pady=(0, 12))

    # Leertaste / Escape als Tastatur-Not-Aus.
    root.bind('<space>', lambda e: do_stop())
    root.bind('<Escape>', lambda e: do_stop())

    # rclpy einfädig (kein Hintergrund-Thread — Galactic pybind/thread-Falle vermeiden)
    # über die Tk-Ereignisschleife bedienen.
    def spin() -> None:
        rclpy.spin_once(node, timeout_sec=0.0)
        # Ketten-Rueckmeldung auswerten: Ack nach dem letzten Senden eingetroffen?
        if state['sent'] is not None:
            if node.ack_time >= state['sent_time'] and node.ack_state == state['sent']:
                chain.config(text='BRIDGE BESTAETIGT — Kommando zugestellt', fg='lime')
            elif time.monotonic() - state['sent_time'] > 2.0:
                chain.config(text='KEINE BRIDGE-ANTWORT — NETZSCHALTER BENUTZEN!',
                             fg='red')
        root.after(50, spin)

    root.after(50, spin)
    try:
        root.mainloop()
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
