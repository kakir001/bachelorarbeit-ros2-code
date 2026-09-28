#!/usr/bin/env python3
"""Aeussere Positionsschleife: faehrt ein Gelenkziel an, BIS es erreicht ist.

Warum es das gibt
-----------------
Die Servos des myCobot 280 kommen unter Last nicht am kommandierten Winkel an.
Gemessen am 2026-09-08 auf dem echten Roboter: nach dem Einschwingen blieben bis
zu 9.1 Grad Restfehler stehen, und zwar richtungsabhaengig — gegen die
Schwerkraft deutlich mehr als mit ihr. Das ist kein Spiel (Spiel waere
symmetrisch), sondern ein bleibender Regelfehler: `send_radians` ist eine offene
Schleife, und wenn der Servo zu kurz kommt, korrigiert ihn niemand.

Warum "dasselbe Ziel nochmal senden" NICHT reicht
-------------------------------------------------
`mycobot_hardware.cpp` vergleicht in `write()` das neue Kommando mit dem ZULETZT
GESENDETEN Kommando (`last_sent_cmds_`), nicht mit der gemessenen Lage. Ein
wortgleich wiederholtes Ziel ergibt `any_change == false` und wird still
verworfen — auch wenn der Arm 9 Grad daneben steht. Im Versuch lieferten
Durchgang 1 und 2 daher auf zwei Nachkommastellen dieselben Winkel.

Was stattdessen wirkt
---------------------
Der Fehler wird auf das KOMMANDO aufgerechnet, nicht auf das Ziel:

    kommando <- kommando - k * (ist - ziel)

Das ist ein I-Anteil: das Kommando laeuft so weit ueber das Ziel hinaus, wie der
Servo unter Last zurueckbleibt. Nebenbei ist der Wert dadurch zwangslaeufig ein
anderer und geht an der Aenderungserkennung vorbei.

Messwerte (echter Roboter, 2 Posen, Toleranz 0.5 Grad, hoechstens 5 Versuche)
----------------------------------------------------------------------------
    k = 0    (wortgleich)  9.34 -> 1.94 Grad  /  2.10 -> 2.10 Grad (nichts)
    k = 1.0                2.23 -> 0.60 -> 1.73 -> 2.45 Grad  SCHWINGT UEBER
    k = 0.5                1.89 -> ... -> 0.29 Grad  /  9.14 -> ... -> 0.42 Grad

k = 0.5 faellt in beiden Posen monoton unter die Toleranz; k = 1.0 wechselt das
Vorzeichen des Fehlers und kommt nicht zur Ruhe. Deshalb ist 0.5 der Vorgabewert
— groesser einzustellen macht es schlechter, nicht schneller.

Kosten und Anzahl der Durchgaenge
---------------------------------
Ein Durchgang dauert ~6-8 s (Fahrt + Stillstand abwarten) und wird nur bezahlt,
solange die Toleranz nicht erreicht ist — die Schleife bricht sofort ab, wenn
sie sitzt. Zwei Durchgaenge bringen typisch auf ~1 Grad.

Der Vorgabewert ist gemessen, nicht geraten. Mit 5 Durchgaengen blieb ein Fall
(Pose B, Start 7.24 Grad) bei 0.89 Grad stehen, obwohl der Fehler fiel
(7.24 -> 3.87 -> 3.41 -> 1.58 -> 0.89) — es fehlten schlicht Durchgaenge. Mit 7
kamen beide Posen an, der schlechteste Fall (Pose A, Start 15.24 Grad) brauchte
aber genau 7 und landete bei 0.13 Grad. 8 gibt darauf einen Durchgang Luft; ein
zusaetzlicher Durchgang kann nichts verschlechtern, weil bei Erreichen der
Toleranz sofort abgebrochen wird.

Der Abfall ist nicht streng monoton (beobachtet: 1.12 -> 1.41 -> 2.10 -> 0.92 ->
0.46). Das Ziel wird trotzdem erreicht — man darf nur nicht nach dem ersten
Anstieg abbrechen.

Fuer freie Bewegungen lohnt die Schleife nicht; fuer den Greifanflug und fuer
alles, was vermessen wird, lohnt sie sehr.

Was sie NICHT behebt
--------------------
Den versetzten J2-Nullpunkt (Modellfehler, nicht Regelfehler) und das echte
Gelenkspiel hinter dem Getriebe — beides sieht der Encoder nicht, also kann auch
diese Schleife es nicht sehen. Sie bringt den Arm dorthin, wo der Encoder sagt,
dass er sein soll.

Benutzung
---------
    from mycobot_calibration.positionieren import Positionierer

    pos = Positionierer(node)                 # nutzt vorhandene Clients, sonst eigene
    erg = pos.anfahren(ziel_rad)              # Fahrt + Nachfuehrung
    if not erg.erfolg:
        node.get_logger().warn(erg.text())

Der Knoten darf waehrenddessen NICHT anderswo gespinnt werden — die Schleife
spinnt ihn selbst (rclpy.spin_once). Nebenlaeufige Executor-Threads sind auf
Galactic ohnehin zu meiden.
"""
import math
import time
from dataclasses import dataclass, field

import rclpy
from control_msgs.action import FollowJointTrajectory
from moveit_msgs.srv import GetStateValidity
from rclpy.action import ActionClient
from sensor_msgs.msg import JointState
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint

from mycobot_calibration.numerical_ik import ARM_JOINTS

# Vorgaben, alle aus der Messung vom 2026-09-08 begruendet (siehe Kopf).
K_VORGABE = 0.5
TOLERANZ_GRAD = 0.5
MAX_VERSUCHE = 8              # gemessen, nicht geraten - siehe Kopf
MAX_KORREKTUR_GRAD = 15.0     # Sicherheitsdeckel je Gelenk
RUHE_S = 2.0                  # so lange muessen die Gelenke stillstehen
RUHE_SCHWELLE_GRAD = 0.06     # ab hier gilt ein Gelenk als stehend


@dataclass
class Ergebnis:
    erfolg: bool
    versuche: int
    rest_grad: float                       # groesster Restfehler ueber alle Gelenke
    fehler_grad: list = field(default_factory=list)   # je Gelenk
    kommando_rad: list = field(default_factory=list)  # zuletzt gesendetes Kommando
    grund: str = ""

    def text(self):
        if self.erfolg:
            return ("Ziel erreicht: Restfehler %.2f Grad nach %d Durchgang/Durchgaengen"
                    % (self.rest_grad, self.versuche))
        return ("Ziel NICHT erreicht: Restfehler %.2f Grad nach %d Durchgang/Durchgaengen%s"
                % (self.rest_grad, self.versuche,
                   " (%s)" % self.grund if self.grund else ""))


class Positionierer:
    """Faehrt Gelenkziele an und fuehrt nach, bis der Encoder sie bestaetigt.

    node        rclpy-Knoten, der gespinnt werden darf
    pruefen     ob das korrigierte Kommando gegen /check_state_validity geprueft
                wird. Das Kommando liegt ausserhalb des Ziels, also wurde es von
                der urspruenglichen Planung NICHT auf Kollision geprueft.
    """

    def __init__(self, node, traj_client=None, validity_client=None, pruefen=True,
                 js_getter=None):
        """js_getter: optionale Funktion, die die Armgelenke in RAD und in
        ARM_JOINTS-Reihenfolge liefert (oder None). Wird nichts uebergeben,
        abonniert der Positionierer /joint_states selbst.

        Frueher wurde per hasattr geraten, ob der Knoten das schon tut. Das ging
        schief, sobald ein Knoten ein gleichnamiges Attribut hatte, das nie
        gefuellt wurde: es kam nie ein Gelenkwert an und bereit() lief in den
        Timeout. Jetzt wird nicht geraten — entweder der Aufrufer sagt es, oder
        der Positionierer abonniert selbst.
        """
        self.node = node
        self._js = None
        self._js_getter = js_getter
        self._eigene_sub = None
        if js_getter is None:
            self._eigene_sub = node.create_subscription(
                JointState, "/joint_states", self._on_js, 20)
        self.traj = traj_client or ActionClient(
            node, FollowJointTrajectory, "/arm_controller/follow_joint_trajectory")
        self.pruefen = pruefen
        self.valid = validity_client
        if pruefen and self.valid is None:
            self.valid = node.create_client(GetStateValidity, "/check_state_validity")

    # ------------------------------------------------------------------ intern
    def _on_js(self, m):
        self._js = m

    def gelenke_rad(self):
        """Aktuelle Armgelenke in RAD, ARM_JOINTS-Reihenfolge, oder None."""
        if self._js_getter is not None:
            return self._js_getter()
        if self._js is None:
            return None
        d = dict(zip(self._js.name, self._js.position))
        try:
            return [d[j] for j in ARM_JOINTS]
        except KeyError:
            return None

    def _spin(self, sekunden):
        t0 = time.time()
        while time.time() - t0 < sekunden and rclpy.ok():
            rclpy.spin_once(self.node, timeout_sec=0.05)

    def bereit(self, sekunden=10.0):
        """Wartet auf /joint_states und den Action-Server."""
        t0 = time.time()
        while time.time() - t0 < sekunden and rclpy.ok():
            rclpy.spin_once(self.node, timeout_sec=0.1)
            if self.gelenke_rad() is not None:
                break
        else:
            return False
        if not self.traj.wait_for_server(timeout_sec=sekunden):
            return False
        if self.pruefen and not self.valid.wait_for_service(timeout_sec=sekunden):
            return False
        return True

    def einschwingen(self, max_s=25.0, ruhe_s=RUHE_S, schwelle_grad=RUHE_SCHWELLE_GRAD):
        """Wartet, BIS die Gelenke wirklich stillstehen.

        Der Trajektorien-Controller meldet fertig, sobald seine Zeit abgelaufen
        ist; die Servos fahren danach oft noch weiter (offene Schleife, feste
        Geschwindigkeit). Ohne dieses Warten misst man den Arm auf halbem Weg.
        """
        letzte = None
        still_seit = None
        t0 = time.time()
        q = None
        while time.time() - t0 < max_s and rclpy.ok():
            rclpy.spin_once(self.node, timeout_sec=0.05)
            q = self.gelenke_rad()
            if q is None:
                continue
            q = [math.degrees(v) for v in q]
            if letzte is not None:
                if max(abs(q[k] - letzte[k]) for k in range(6)) < schwelle_grad:
                    if still_seit is None:
                        still_seit = time.time()
                    elif time.time() - still_seit >= ruhe_s:
                        return q, True
                else:
                    still_seit = None
            letzte = q
        return q, False

    def fahre(self, ziel_rad, sekunden=3.0):
        """Schickt EIN Gelenkziel an den arm_controller und wartet das Ende ab."""
        g = FollowJointTrajectory.Goal()
        jt = JointTrajectory()
        jt.joint_names = list(ARM_JOINTS)
        pt = JointTrajectoryPoint()
        pt.positions = [float(v) for v in ziel_rad]
        pt.time_from_start.sec = int(sekunden)
        pt.time_from_start.nanosec = int((sekunden - int(sekunden)) * 1e9)
        jt.points = [pt]
        g.trajectory = jt
        fut = self.traj.send_goal_async(g)
        rclpy.spin_until_future_complete(self.node, fut, timeout_sec=15.0)
        h = fut.result()
        if h is None or not h.accepted:
            return False
        erg = h.get_result_async()
        rclpy.spin_until_future_complete(self.node, erg, timeout_sec=sekunden + 20.0)
        return True

    def zulaessig(self, q_rad, gruppe="arm"):
        if not self.pruefen:
            return True
        req = GetStateValidity.Request()
        req.group_name = gruppe
        js = JointState()
        js.name = list(ARM_JOINTS)
        js.position = [float(v) for v in q_rad]
        req.robot_state.joint_state = js
        req.robot_state.is_diff = True
        fut = self.valid.call_async(req)
        rclpy.spin_until_future_complete(self.node, fut, timeout_sec=5.0)
        r = fut.result()
        return bool(r and r.valid)

    def weg_zulaessig(self, q_von_rad, q_nach_rad, schritte=15):
        for i in range(schritte + 1):
            s = i / schritte
            qi = [(1 - s) * q_von_rad[k] + s * q_nach_rad[k] for k in range(6)]
            if not self.zulaessig(qi):
                return False
        return True

    # ------------------------------------------------------------------ oeffentlich
    def nachfuehren(self, ziel_rad, k=K_VORGABE, toleranz_grad=TOLERANZ_GRAD,
                    max_versuche=MAX_VERSUCHE, sekunden=3.0,
                    max_korrektur_grad=MAX_KORREKTUR_GRAD, log=None):
        """Fuehrt ein BEREITS angefahrenes Ziel nach, bis der Encoder es bestaetigt.

        Der Arm muss schon in der Naehe stehen — diese Schleife korrigiert, sie
        plant nicht. Fuer Fahrt + Nachfuehrung siehe anfahren().
        """
        ziel_g = [math.degrees(v) for v in ziel_rad]
        kommando_g = list(ziel_g)
        letzte_f = []
        versuch = 0

        for versuch in range(1, max_versuche + 1):
            ist_g, still = self.einschwingen()
            if ist_g is None:
                return Ergebnis(False, versuch, float("inf"), grund="keine /joint_states")
            f = [ist_g[i] - ziel_g[i] for i in range(6)]
            letzte_f = f
            rest = max(abs(v) for v in f)
            if log:
                log("  Durchgang %d: Rest %.2f Grad%s" % (
                    versuch, rest, "" if still else " (Gelenke kamen nicht zur Ruhe)"))
            if rest <= toleranz_grad:
                return Ergebnis(True, versuch, rest, f,
                                [math.radians(v) for v in kommando_g])
            if versuch >= max_versuche:
                break

            # Fehler auf das Kommando aufrechnen, am Deckel begrenzen.
            neu_g = []
            for i in range(6):
                v = kommando_g[i] - k * f[i]
                ab = max(-max_korrektur_grad, min(max_korrektur_grad, v - ziel_g[i]))
                neu_g.append(ziel_g[i] + ab)
            neu_rad = [math.radians(v) for v in neu_g]

            # Das korrigierte Kommando liegt ausserhalb des Ziels und wurde nie
            # geprueft — hier nachholen, sonst faehrt die Korrektur in eine
            # Kollision.
            ist_rad = [math.radians(v) for v in ist_g]
            if not self.weg_zulaessig(ist_rad, neu_rad):
                return Ergebnis(False, versuch, rest, f,
                                [math.radians(v) for v in kommando_g],
                                "korrigiertes Kommando unzulaessig")
            kommando_g = neu_g
            if not self.fahre(neu_rad, sekunden):
                return Ergebnis(False, versuch, rest, f, neu_rad,
                                "arm_controller hat das Kommando abgelehnt")

        rest = max(abs(v) for v in letzte_f) if letzte_f else float("inf")
        return Ergebnis(False, versuch, rest, letzte_f,
                        [math.radians(v) for v in kommando_g],
                        "Versuche aufgebraucht")

    def anfahren(self, ziel_rad, sekunden=7.0, weg_pruefen=True, **kw):
        """Faehrt das Ziel an und fuehrt anschliessend nach.

        weg_pruefen prueft den geraden Gelenkweg von der aktuellen Lage zum Ziel,
        BEVOR gefahren wird. Fuer Ziele, die schon von MoveIt geplant wurden, ist
        das ueberfluessig (dort wurde der echte Bahnverlauf geprueft, nicht die
        Gerade) — dann nachfuehren() direkt aufrufen oder weg_pruefen=False.
        """
        if weg_pruefen:
            jetzt = self.gelenke_rad()
            if jetzt is None:
                return Ergebnis(False, 0, float("inf"), grund="keine /joint_states")
            if not self.weg_zulaessig(jetzt, list(ziel_rad)):
                return Ergebnis(False, 0, float("inf"),
                                grund="Weg zum Ziel ist unzulaessig")
        if not self.fahre(ziel_rad, sekunden):
            return Ergebnis(False, 0, float("inf"),
                            grund="arm_controller hat die Fahrt abgelehnt")
        return self.nachfuehren(ziel_rad, **kw)
