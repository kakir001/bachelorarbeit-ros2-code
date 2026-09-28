#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""Stetigkeitsprobe fuer kartesische Bahnen (compute_cartesian_path) - Pflicht vor jeder Fahrt.

WARUM (2026-09-13 Abend, Unfall): versetze_soll.py sollte den TCP 40 mm senkrecht heben. Der
Startzustand war von Hand verschoben (J3 -152.5 Grad, ausserhalb der Grenzen). compute_cartesian_path
lieferte fraction 1.0 - aber die IK sprang von Wegpunkt zu Wegpunkt zwischen ZWEI Loesungsaesten
(J1 -39 <-> +65, J2 +44 <-> +94 ...). Mit jump_threshold = 0 prueft MoveIt das nicht. Der Arm
pendelte die ganze Bahn entlang zwischen den Aesten hin und her, der Greifer schlug an, der Benutzer
musste den Greifer abschrauben. Der TCP kam am Ende "richtig" an - der Weg dorthin war es nicht.

Diese Probe verwirft eine Bahn, in der irgendein Gelenk zwischen zwei aufeinanderfolgenden Punkten
mehr als `max_schritt_grad` springt oder sich ueber die ganze Bahn um mehr als `max_gesamt_grad`
bewegt (fuer die kurzen Verschiebungen, die diese Werkzeuge fahren, sind 45 Grad je Gelenk viel).

    from bahnpruefung import pruefe_bahn
    ok, text = pruefe_bahn(res.solution.joint_trajectory, ARM)
    if not ok: print("!! " + text); <nicht fahren>
"""
import math


def pruefe_bahn(traj, arm, max_schritt_grad=12.0, max_gesamt_grad=45.0, start=None):
    """traj: trajectory_msgs/JointTrajectory. arm: Gelenknamen. start: optionale Ist-Gelenke [rad]
    (wird als Punkt davor geprueft, damit auch der Sprung Ist -> erster Bahnpunkt auffaellt).
    -> (ok, Text)."""
    if traj is None or not traj.points: return False, "Bahn leer"
    idx = [traj.joint_names.index(j) for j in arm]
    pts = [[p.positions[i] for i in idx] for p in traj.points]
    if start is not None: pts = [list(start)] + pts
    schritt, wo = 0.0, 0
    for k, (p, q) in enumerate(zip(pts, pts[1:])):
        d = max(abs(b - a) for a, b in zip(p, q))
        if d > schritt: schritt, wo = d, k
    gesamt = max(max(p[j] for p in pts) - min(p[j] for p in pts) for j in range(len(arm)))
    # Hin-und-her: Summe der Gelenkwege deutlich groesser als der Netto-Weg = Pendeln
    summe = sum(sum(abs(b - a) for a, b in zip(p, q)) for p, q in zip(pts, pts[1:]))
    netto = sum(abs(b - a) for a, b in zip(pts[0], pts[-1]))
    text = ("%d Punkte, max. Gelenkschritt %.1f Grad (bei %d), max. Gelenkweg %.1f Grad, Weg/Netto %.1f"
            % (len(pts), math.degrees(schritt), wo, math.degrees(gesamt), summe / max(netto, 1e-6)))
    if math.degrees(schritt) > max_schritt_grad:
        return False, "Bahn UNSTETIG (IK springt zwischen Loesungsaesten): " + text
    if math.degrees(gesamt) > max_gesamt_grad:
        return False, "Bahn zu weit fuer eine kleine Verschiebung: " + text
    if netto > 1e-3 and summe / netto > 3.0:
        return False, "Bahn PENDELT (Gelenkweg >> Netto): " + text
    return True, text
