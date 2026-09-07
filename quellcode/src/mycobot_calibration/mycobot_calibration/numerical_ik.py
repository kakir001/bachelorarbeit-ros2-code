"""
myCobot 280 JN — Multi-Seed-Wrapper um MoveIts compute_ik-Service.

Grundidee
---------
Die inverse Kinematik (IK) beantwortet die Frage: "Welche Gelenkwinkel
q = (q1..q6) führen den Endeffektor (TCP) an eine gewünschte kartesische
Pose (Position + Orientierung)?" Für einen 6-DOF-Arm ist dieses Problem im
Allgemeinen nichtlinear und kann mehrere, genau eine oder gar keine Lösung
besitzen (Mehrdeutigkeit durch Ellenbogen-oben/-unten, Handgelenk-Flip usw.).

Statt eine eigene analytische/numerische IK zu implementieren, delegiert
dieses Modul die Lösung an MoveIts ``/compute_ik``-Service (der intern einen
IK-Solver wie KDL oder TRAC-IK verwendet). Der Solver ist jedoch nur ein
lokaler, gradientenbasierter Löser: Er startet von EINEM Anfangswert (Seed)
und konvergiert zur nächstgelegenen Lösung — oder scheitert, wenn er in
einem schlechten Startbereich gefangen ist.

Multi-Seed-Strategie
--------------------
Um die Erfolgsquote und die Lösungsqualität zu erhöhen, ruft dieser Wrapper
den Solver mit MEHREREN, unterschiedlichen Startwerten (Seeds) auf und wählt
anschließend die beste zurückgelieferte Lösung anhand einer Kostenfunktion
(siehe ``solve_multi_seed``). So werden lokale Minima umgangen und eine
gutmütige, zur aktuellen Pose passende Lösung bevorzugt.

Weitere Eigenschaften
--------------------
  * Die Greifer-Orientierung wird fest auf "senkrecht nach unten" erzwungen
    (top-down grasp), was für das Aufnehmen von Objekten auf einer Ebene
    (Tisch/Stand) sinnvoll ist und einen Rotations-Freiheitsgrad festlegt.
  * FK- und IK-Konsistenz ist garantiert, da beide vom selben MoveIt-Modell
    stammen (identisches URDF/kinematisches Modell).
"""

import numpy as np

import rclpy
from rclpy.node import Node
from moveit_msgs.srv import GetPositionIK, GetPositionFK
from geometry_msgs.msg import PoseStamped
from sensor_msgs.msg import JointState

# Gelenknamen J1..J6 in kinematischer Reihenfolge (Basis -> TCP).
# Müssen mit dem URDF und der MoveIt-Gruppe 'arm' übereinstimmen, damit die
# vom Service zurückgegebenen Positionen korrekt zugeordnet werden können.
ARM_JOINTS = [
    'joint2_to_joint1', 'joint3_to_joint2', 'joint4_to_joint3',
    'joint5_to_joint4', 'joint6_to_joint5', 'joint6output_to_joint6',
]

# Untere/obere Gelenkgrenzen [rad] pro Achse J1..J6, entnommen aus dem URDF.
# Beachte: Diese Grenzen sind teils ASYMMETRISCH (z. B. J5). Sie werden hier
# zur Erzeugung plausibler Zufalls-Seeds genutzt; die eigentliche Einhaltung
# der Grenzen übernimmt der MoveIt-Solver selbst.
JOINT_LIMITS = [
    (-2.9321, 2.9321),
    (-2.4434, 2.4434),
    (-2.6179, 2.6179),
    (-2.6179, 2.6179),
    (-2.7052, 2.7925),
    (-3.14159, 3.14159),
]

# Ziel-Quaternion für die Greifer-Orientierung "senkrecht nach unten".
# KORRIGIERT 2026-07-16: der alte Wert (-0.7071, 0.7071, 0, 0) stammte per FK
# aus der Gelenkstellung [0,-1.57,0,-1.57,0,0] eines FRÜHEREN URDF-Stands.
# Nach den Greifer/tcp-Änderungen zeigt an dieser Referenzstellung KEINE
# tcp-Achse mehr nach unten (~48° gekippt) → /compute_ik lieferte für JEDE
# Position error -31 (NO_IK_SOLUTION), beide Klick/Jog-Werkzeuge wirkten kaputt.
# Aktuelles Modell: tcp liegt 110 mm in +Y von gripper_base — die ZEIGE-Richtung
# des Greifers ist die tcp-Y-Achse. "Senkrecht nach unten" = tcp-Y-Achse zeigt
# zum Boden = Rotation um X um -90°: (x,y,z,w) = (-sin45°, 0, 0, cos45°).
# Live verifiziert (TRAC-IK): löst am nahen Arbeitsbereich für ALLE Yaw-Winkel.
GRIPPER_DOWN_QUAT = (-0.70711, 0.0, 0.0, 0.70711)  # x, y, z, w


class NumericalIK:
    """Dünner Wrapper um MoveIts IK-/FK-Services mit Multi-Seed-Auswahl.

    Hält persistente Service-Clients auf ``/compute_ik`` (inverse Kinematik)
    und ``/compute_fk`` (vorwärts Kinematik). Der zentrale Mehrwert liegt in
    ``solve_multi_seed``, das den lokalen Solver mit vielen Startwerten aufruft
    und die günstigste Lösung auswählt.
    """

    def __init__(self, node: Node):
        # Referenz auf den aufrufenden ROS-Knoten (für Logging/Clients).
        self._node = node
        # Persistente Clients: einmal anlegen, mehrfach wiederverwenden, um
        # den Overhead wiederholter Client-Erzeugung zu vermeiden.
        self._ik_client = node.create_client(GetPositionIK, '/compute_ik')
        self._fk_client = node.create_client(GetPositionFK, '/compute_fk')

    def _call_ik(self, target_xyz, seed_joints=None):
        """Einzelner IK-Aufruf: eine Zielposition + optionaler Seed -> Gelenkwinkel.

        Baut die GetPositionIK-Anfrage auf, sendet sie synchronisiert (blockierend)
        an MoveIt und dekodiert die Antwort in eine Liste [q1..q6].

        :param target_xyz: Zielposition (x, y, z) des TCP im Frame 'robot_base' [m].
        :param seed_joints: Optionaler Startwert (6 Gelenkwinkel), von dem aus der
                            lokale Solver iteriert. Ein guter Seed erhöht die
                            Wahrscheinlichkeit einer Lösung erheblich.
        :return: Liste von 6 Gelenkwinkeln [rad] bei Erfolg, sonst None
                 (Service nicht verfügbar, Timeout oder IK-Fehlercode).
        """
        # Sicherstellen, dass der Service läuft (MoveIt gestartet?).
        if not self._ik_client.wait_for_service(timeout_sec=2.0):
            return None

        req = GetPositionIK.Request()
        req.ik_request.group_name = 'arm'      # Planungsgruppe aus der SRDF
        req.ik_request.ik_link_name = 'tcp'    # Ziel-Link = Werkzeugspitze (TCP)
        # Kollisionen ignorieren: Wir wollen eine reine kinematische Lösung.
        # Die Kollisionsprüfung erfolgt später separat (z. B. bei der Planung).
        req.ik_request.avoid_collisions = False
        req.ik_request.timeout.sec = 1         # Rechenzeit-Budget des Solvers [s]

        # Ziel-Pose zusammensetzen: Position aus dem Argument, Orientierung fest
        # auf "senkrecht nach unten" (siehe GRIPPER_DOWN_QUAT).
        pose = PoseStamped()
        pose.header.frame_id = 'robot_base'    # Referenzrahmen der Zielkoordinaten
        pose.pose.position.x = target_xyz[0]
        pose.pose.position.y = target_xyz[1]
        pose.pose.position.z = target_xyz[2]
        pose.pose.orientation.x = GRIPPER_DOWN_QUAT[0]
        pose.pose.orientation.y = GRIPPER_DOWN_QUAT[1]
        pose.pose.orientation.z = GRIPPER_DOWN_QUAT[2]
        pose.pose.orientation.w = GRIPPER_DOWN_QUAT[3]
        req.ik_request.pose_stamped = pose

        # Optionalen Seed als Roboterzustand übergeben. Der Greifer wird als
        # zusätzliches Gelenk mitgeliefert (Position 0.0), damit der übergebene
        # RobotState vollständig und für den Solver konsistent ist.
        if seed_joints is not None:
            js = JointState()
            js.name = ARM_JOINTS + ['gripper_controller']
            js.position = list(seed_joints) + [0.0]
            req.ik_request.robot_state.joint_state = js

        future = self._ik_client.call_async(req)

        # Blockierendes Warten auf das Ergebnis. Dies ist thread-safe, weil der
        # eigentliche ROS-Spin (der das Future erfüllt) in einem SEPARATEN
        # Thread läuft; hier wird nur gepollt. Harte Obergrenze von 3 s
        # verhindert ein Hängenbleiben, falls der Service nie antwortet.
        import time
        t0 = time.monotonic()
        while not future.done() and (time.monotonic() - t0) < 3.0:
            time.sleep(0.01)

        # Kein Ergebnis (Timeout) oder Aufruf fehlgeschlagen -> keine Lösung.
        if not future.done() or future.result() is None:
            return None
        res = future.result()
        # error_code.val == 1 entspricht moveit_msgs::MoveItErrorCodes::SUCCESS.
        # Jeder andere Wert (z. B. NO_IK_SOLUTION) bedeutet: keine Lösung.
        if res.error_code.val != 1:
            return None

        # Der Solver liefert die Gelenke evtl. in anderer Reihenfolge oder mit
        # Zusatzgelenken zurück. Über ein Name->Position-Mapping extrahieren
        # wir genau die sechs Armgelenke in der kanonischen Reihenfolge.
        name_to_pos = dict(zip(res.solution.joint_state.name,
                               res.solution.joint_state.position))
        return [name_to_pos.get(j, 0.0) for j in ARM_JOINTS]

    def solve_multi_seed(self, target_xyz, gripper_down=True, current_joints=None,
                         n_seeds=10):
        """Löst die IK robust über mehrere Startwerte und wählt die beste Lösung.

        Ablauf:
          1. Analytische Schätzung für J1 (Basisdrehung) aus der Zielrichtung.
          2. Zusammenstellen einer Seed-Liste aus (a) handverlesenen, bewährten
             Posen, (b) optional der aktuellen Pose und (c) Zufalls-Seeds.
          3. Für jeden Seed den lokalen IK-Solver aufrufen.
          4. Jede gefundene Lösung mit einer Kostenfunktion bewerten und die
             günstigste zurückgeben.

        :param target_xyz: Zielposition (x, y, z) des TCP [m].
        :param gripper_down: Beibehalten aus Kompatibilitätsgründen; die
                             Orientierung ist ohnehin fest nach unten erzwungen.
        :param current_joints: Optionale aktuelle Gelenkstellung. Wird sowohl als
                              (guter) Seed genutzt als auch in der Kostenfunktion
                              bevorzugt, um kleine Bewegungen zu belohnen.
        :param n_seeds: Mindestanzahl der auszuprobierenden Startwerte.
        :return: Beste Gelenklösung [q1..q6] oder None, falls kein Seed konvergiert.
        """
        # J1 (Basisdrehung) zeigt in Richtung der Ziel-Projektion in der
        # XY-Ebene. atan2 liefert den Winkel des Zielvektors und damit einen
        # sehr guten analytischen Startwert für das erste Gelenk.
        j1_guess = np.arctan2(target_xyz[1], target_xyz[0])

        # Handverlesene Start-Posen: mit dem geschätzten J1 kombiniert und
        # typischen "Ellenbogen"-Konfigurationen (J2/J3/J4), die für top-down-
        # Griffe auf dem Stand erfahrungsgemäß gut konvergieren. Der letzte
        # Seed (Nullstellung) dient als neutraler Rückfall.
        seeds = [
            [j1_guess, -0.5, -1.0, 0.0, 0.0, 0.0],
            [j1_guess, -1.0, -0.5, -0.5, 0.0, 0.0],
            [j1_guess, -0.3, -1.5, 0.5, 0.0, 0.0],
            [0.0, 0.0, 0.0, 0.0, 0.0, 0.0],
        ]

        # Die aktuelle Pose ganz vorne einfügen: Sie ist oft der beste Seed,
        # weil die nächstgelegene Lösung meist auch die gewünschte ist.
        if current_joints is not None:
            seeds.insert(0, current_joints)

        # Auffüllen mit Zufalls-Seeds, bis n_seeds erreicht ist. Dies erhöht
        # die Chance, auch schwierige Ziele zu lösen, indem der Lösungsraum
        # breiter abgetastet wird (Umgehung lokaler Minima).
        rng = np.random.default_rng()
        while len(seeds) < n_seeds:
            # J1 nah an der analytischen Schätzung streuen (+/- 0.3 rad).
            seed = [j1_guess + rng.uniform(-0.3, 0.3)]
            # Restliche Gelenke innerhalb 40 % ihrer Grenzen zufällig wählen:
            # bewusst konservativ, um unrealistische, gestreckte Posen (die
            # selten konvergieren) zu vermeiden.
            for lo, hi in JOINT_LIMITS[1:]:
                seed.append(rng.uniform(lo * 0.4, hi * 0.4))
            seeds.append(seed)

        best = None
        best_cost = float('inf')

        # Alle Seeds durchprobieren und die beste (kostenminimale) Lösung merken.
        for seed in seeds:
            result = self._call_ik(target_xyz, seed_joints=seed)
            if result is not None:
                # Kostenterm 1: Abweichung von J1 zur analytischen Schätzung.
                # Auf das kürzere Winkelintervall normieren (Wrap-around bei pi),
                # damit z. B. +179 Grad und -179 Grad als "nah" gelten.
                j1_diff = abs(result[0] - j1_guess)
                if j1_diff > np.pi:
                    j1_diff = 2 * np.pi - j1_diff
                # Gewichtete Gesamtkosten:
                #   * j1_diff stark gewichtet (5.0): eine unerwartete
                #     Basisdrehung ist besonders unerwünscht.
                #   * Summe der Beträge aller Gelenke schwach gewichtet (0.1):
                #     bevorzugt "kompakte", nicht ausladende Konfigurationen.
                cost = j1_diff * 5.0 + sum(abs(r) for r in result) * 0.1
                # Kostenterm 3 (nur falls Ist-Pose bekannt): Nähe zur aktuellen
                # Stellung belohnen -> minimiert die nötige Roboterbewegung und
                # vermeidet große, ruckartige Umkonfigurationen.
                if current_joints is not None:
                    diff = sum(abs(a - b) for a, b in zip(result, current_joints))
                    cost += diff * 0.3
                if cost < best_cost:
                    best_cost = cost
                    best = result

        return best
