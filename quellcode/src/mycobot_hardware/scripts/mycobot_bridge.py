#!/usr/bin/env python3
"""myCobot 280 JN bridge — owns pymycobot, exposes state/commands over Unix socket.

Design: a single Python process holds the only `pymycobot.MyCobot280` instance.
All serial I/O happens here, in one worker thread. A separate accept loop
handles a single client (the C++ ros2_control plugin) via line-delimited JSON.

This isolates the C++ ros2_control_node from embedded-Python threading bugs
(pybind11 2.2.7 on Galactic segfaults when GIL is shared with bg threads).

Protocol — newline-delimited, space-separated, both directions:

  bridge → client (state, ~20 Hz):
    state <ts> <r1> <r2> <r3> <r4> <r5> <r6> <grip|-1>

  client → bridge (commands):
    send_radians <r1> <r2> <r3> <r4> <r5> <r6> <speed>
    set_gripper <value> <speed>
    power_on
    release_all
    shutdown
    estop <0|1>   # NOT-AUS: 1 = mc.stop() + Bewegungs-Kommandos blockieren, 0 = Freigabe
"""

import argparse
import os
import queue
import signal
import socket
import sys
import threading
import time

from pymycobot import MyCobot280


def log(msg):
    print(f'[mycobot_bridge] {msg}', file=sys.stderr, flush=True)


class Bridge:
    def __init__(self, mc, speed, rate_hz, socket_path):
        self.mc = mc
        self.default_speed = speed
        self.period = 1.0 / rate_hz
        self.socket_path = socket_path
        self.cmd_socket_path = socket_path + '.cmd'
        self.cmd_queue: queue.Queue = queue.Queue(maxsize=32)
        self.client_lock = threading.Lock()
        self.client_conn = None
        self.stop_event = threading.Event()
        # Greifer Open-Loop Reserve: wenn get_gripper_value() bei dieser Firmware oft
        # 255/-1 (ungültig) zurückgibt, melden wir den zuletzt kommandierten Wert.
        self.last_gripper_cmd = None
        self._grip_log_counter = 0
        # NOT-AUS-Zustand: solange True werden send_radians/set_gripper/release_all
        # verworfen. Gesetzt/gelöscht über das Wire-Kommando "estop <0|1>"
        # (kommt vom estop_relay-Node, der das /estop-Topic abonniert).
        self.estop = False

    def run(self):
        try:
            os.unlink(self.socket_path)
        except FileNotFoundError:
            pass

        worker = threading.Thread(target=self._worker, daemon=True)
        worker.start()

        # Kommando-Socket — jeder kann schreiben, legt in cmd_queue ab
        cmd_thread = threading.Thread(target=self._cmd_listener, daemon=True)
        cmd_thread.start()

        srv = socket.socket(socket.AF_UNIX, socket.SOCK_STREAM)
        srv.bind(self.socket_path)
        os.chmod(self.socket_path, 0o666)
        srv.listen(1)
        srv.settimeout(0.5)
        log(f'listening on {self.socket_path}')

        try:
            while not self.stop_event.is_set():
                try:
                    conn, _ = srv.accept()
                except socket.timeout:
                    continue
                log('client connected')
                self._handle_client(conn)
                log('client disconnected')
        except KeyboardInterrupt:
            pass
        finally:
            self.stop_event.set()
            srv.close()
            try:
                os.unlink(self.socket_path)
            except FileNotFoundError:
                pass
            try:
                os.unlink(self.cmd_socket_path)
            except FileNotFoundError:
                pass
            worker.join(timeout=2.0)
            try:
                self.mc.release_all_servos()
            except Exception:
                pass

    def _enqueue(self, line):
        """Einheitlicher Kommando-Eingang (beide Sockets laufen hier durch).

        NOT-AUS wird VORGEZOGEN: das Flag wird sofort im Netz-Thread gesetzt und
        die Warteschlange geleert, damit kein bereits eingereihtes Bewegungs-
        Kommando nach dem Stopp "nachläuft". Das eigentliche mc.stop() führt
        der Worker aus (einziger Serial-Thread, nächster Zyklus, <~100 ms).
        Während aktivem NOT-AUS werden Bewegungs-Kommandos hier verworfen.
        """
        if not line:
            return
        if line.startswith('estop'):
            parts = line.split()
            active = len(parts) > 1 and parts[1] not in ('0', 'false', 'False')
            self.estop = active
            # Warteschlange leeren — bei STOP darf nichts nachlaufen, bei
            # FREIGABE ist sie ohnehin nur mit Nicht-Bewegungs-Resten gefüllt.
            try:
                while True:
                    self.cmd_queue.get_nowait()
            except queue.Empty:
                pass
            self.cmd_queue.put('estop 1' if active else 'estop 0')
            return
        if self.estop and line.split()[0] in ('send_radians', 'set_gripper', 'release_all'):
            log(f'NOT-AUS aktiv — Kommando verworfen: {line.split()[0]}')
            return
        try:
            self.cmd_queue.put_nowait(line)
        except queue.Full:
            log('WARN cmd_queue full, dropping')

    def _handle_client(self, conn):
        with self.client_lock:
            if self.client_conn is not None:
                try:
                    self.client_conn.close()
                except OSError:
                    pass
            self.client_conn = conn

        buf = b''
        conn.settimeout(0.5)
        try:
            while not self.stop_event.is_set():
                try:
                    chunk = conn.recv(4096)
                except socket.timeout:
                    continue
                if not chunk:
                    break
                buf += chunk
                while b'\n' in buf:
                    line, buf = buf.split(b'\n', 1)
                    line = line.strip()
                    if not line:
                        continue
                    self._enqueue(line.decode('utf-8'))
        except OSError as e:
            log(f'client recv error: {e}')
        finally:
            with self.client_lock:
                if self.client_conn is conn:
                    self.client_conn = None
            try:
                conn.close()
            except OSError:
                pass

    def _cmd_listener(self):
        """Separater Kommando-Socket — empfängt Kommandos von einmaligen Verbindungen."""
        try:
            os.unlink(self.cmd_socket_path)
        except FileNotFoundError:
            pass
        srv = socket.socket(socket.AF_UNIX, socket.SOCK_STREAM)
        srv.bind(self.cmd_socket_path)
        os.chmod(self.cmd_socket_path, 0o666)
        srv.listen(5)
        srv.settimeout(1.0)
        log(f'cmd listener on {self.cmd_socket_path}')
        while not self.stop_event.is_set():
            try:
                conn, _ = srv.accept()
            except socket.timeout:
                continue
            try:
                data = conn.recv(4096).decode().strip()
                if data:
                    log(f'cmd recv: {data}')
                    for ln in data.splitlines():
                        self._enqueue(ln.strip())
            except Exception:
                pass
            finally:
                conn.close()
        srv.close()

    def _worker(self):
        next_t = time.monotonic()
        grip_counter = 0
        last_grip = -1
        while not self.stop_event.is_set():
            # Drain command queue first — commands take priority over polling.
            while True:
                try:
                    cmd = self.cmd_queue.get_nowait()
                except queue.Empty:
                    break
                self._exec(cmd)
                if self.stop_event.is_set():
                    return

            # Poll robot state.
            try:
                radians = self.mc.get_radians()
                if not isinstance(radians, list) or len(radians) < 6:
                    continue
                # Greifer nur alle 10 Zyklen lesen (serielle Last reduzieren)
                grip_counter += 1
                if grip_counter >= 10:
                    grip_counter = 0
                    val = self._read_gripper()
                    if val is not None:
                        last_grip = val
                parts = ['state', f'{time.time():.6f}']
                parts.extend(f'{float(r):.6f}' for r in radians)
                parts.append(str(last_grip))
                self._broadcast(' '.join(parts) + '\n')
            except Exception as e:
                log(f'WARN poll: {e}')

            # Pace.
            next_t += self.period
            sleep = next_t - time.monotonic()
            if sleep > 0:
                time.sleep(sleep)
            else:
                next_t = time.monotonic()

    def _read_gripper(self):
        """Gibt die Greifer-Position zurück — OPEN-LOOP.

        Die Atom-Firmware dieses Roboters meldet die Position des adaptiven
        Greifers NICHT ZURUECK: get_gripper_value() (mit/ohne Argument, auch
        nach init_gripper) gibt immer 255 (0xFF = "keine Daten") zurück. Mit
        einem Hardware-Probe bestätigt; is_gripper_moving / protect_current
        funktionieren, nur die Positionslesung fehlt. Eine echte Encoder-Lesung
        ist daher nicht möglich — wir melden den zuletzt KOMMANDIERTEN Wert
        (RViz zeigt die kommandierte Pose). Ein einziger Versuch wird trotzdem
        gemacht: falls die Firmware es künftig unterstützt, wird der echte
        Wert 0-100 automatisch verwendet.
        """
        try:
            g = self.mc.get_gripper_value(1)  # 1 = adaptive gripper
            if isinstance(g, int) and 0 <= g <= 100:
                self._grip_log(f'firmware reported value={g}')
                return g
            self._grip_log(f'no position from firmware (raw={g}); '
                           f'open-loop cmd={self.last_gripper_cmd}')
        except Exception as e:
            self._grip_log(f'read exc={e}; open-loop cmd={self.last_gripper_cmd}')
        return self.last_gripper_cmd

    def _grip_log(self, msg):
        """Die ersten paar Lesungen und danach jede 20. loggen (Spam vermeiden)."""
        self._grip_log_counter += 1
        if self._grip_log_counter <= 5 or self._grip_log_counter % 20 == 0:
            log(f'gripper read: {msg}')

    def _exec(self, line):
        parts = line.split()
        if not parts:
            return
        name = parts[0]
        # Zweite Verteidigungslinie (Race beim Flag-Setzen): auch hier blockieren.
        if self.estop and name in ('send_radians', 'set_gripper', 'release_all'):
            log(f'NOT-AUS aktiv — Kommando verworfen: {name}')
            return
        try:
            if name == 'estop':
                if len(parts) > 1 and parts[1] not in ('0', 'false', 'False'):
                    # Stopp DREIMAL senden (seriell ist gelegentlich verlustbehaftet);
                    # stop() hält die Bewegung an, lässt die Servos aber unter
                    # Drehmoment — der Arm fällt NICHT (kein release_all_servos!).
                    for _ in range(3):
                        try:
                            self.mc.stop()
                        except Exception as e:
                            log(f'WARN estop stop(): {e}')
                        time.sleep(0.02)
                    log('*** NOT-AUS AKTIV *** — Bewegung gestoppt, Kommandos blockiert')
                else:
                    log('NOT-AUS FREIGABE — Kommandos wieder aktiv')
            elif name == 'send_radians' and len(parts) >= 8:
                rads = [float(x) for x in parts[1:7]]
                speed = int(parts[7])
                self.mc.send_radians(rads, speed)
            elif name == 'set_gripper' and len(parts) >= 3:
                v = int(parts[1])
                self.mc.set_gripper_value(v, int(parts[2]))
                self.last_gripper_cmd = max(0, min(100, v))
            elif name == 'power_on':
                self.mc.power_on()
            elif name == 'release_all':
                self.mc.release_all_servos()
            elif name == 'shutdown':
                self.stop_event.set()
            else:
                log(f'WARN unknown cmd: {line!r}')
        except Exception as e:
            log(f'WARN cmd {name}: {e}')

    def _broadcast(self, line):
        payload = line.encode('utf-8')
        with self.client_lock:
            conn = self.client_conn
        if conn is None:
            return
        try:
            conn.sendall(payload)
        except (BrokenPipeError, OSError):
            pass  # accept loop will reset on next reconnect


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument('--port', default='/dev/ttyTHS1')
    ap.add_argument('--baud', type=int, default=1000000)
    ap.add_argument('--socket', default='/tmp/mycobot_bridge.sock')
    ap.add_argument('--rate', type=float, default=20.0,
                    help='polling Hz (serial round-trip ~50ms)')
    ap.add_argument('--speed', type=int, default=30, help='default send_radians speed 1-100')
    ap.add_argument('--no-power-on', action='store_true',
                    help='do not call power_on() at startup (leave robot in teach mode)')
    args = ap.parse_args()

    log(f'opening {args.port} @ {args.baud}')
    mc = MyCobot280(args.port, args.baud)
    time.sleep(0.2)  # let serial settle

    if not args.no_power_on:
        try:
            mc.power_on()
            log('power_on OK')
        except Exception as e:
            log(f'WARN power_on: {e}')

    bridge = Bridge(mc, args.speed, args.rate, args.socket)

    # Clean shutdown on SIGINT/SIGTERM (parent kill).
    def _sigterm(*_):
        log('signal received, stopping')
        bridge.stop_event.set()
    signal.signal(signal.SIGTERM, _sigterm)
    signal.signal(signal.SIGINT, _sigterm)

    bridge.run()
    log('exiting')


if __name__ == '__main__':
    main()
