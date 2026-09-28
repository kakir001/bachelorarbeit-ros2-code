#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""Greifer-Glieder in der MoveIt-Kollisionsmatrix (ACM) freigeben oder zuruecksetzen.

Wann: der Greifer ist physisch ABGESCHRAUBT (13.9. Abend nach dem Pendel-Unfall), das Modell hat
ihn aber noch -> MoveIt sieht Kollisionen, die es nicht gibt, und plant nicht (Startzustand
"in Kollision"). Freigeben = alle Paare mit Greifer-Gliedern erlaubt; der Arm selbst bleibt
kollisionsgeprueft. ZURUECKSETZEN, sobald der Greifer wieder dran ist (Stand = SRDF).

    python3 tools/acm_greifer.py --frei        # Greifer abgeschraubt
    python3 tools/acm_greifer.py --zurueck     # Greifer wieder montiert (ACM wie SRDF)
    python3 tools/acm_greifer.py               # nur anzeigen
"""
import argparse, sys, subprocess, xml.etree.ElementTree as ET
import rclpy
from rclpy.node import Node
from moveit_msgs.srv import GetPlanningScene, ApplyPlanningScene
from moveit_msgs.msg import PlanningSceneComponents, PlanningScene
GREIFER = ['finger_links', 'finger_rechts', 'greifer_huelle', 'greifer_mechanik_breit', 'greifer_mechanik_schmal', 'gripper_base',
           'gripper_left1', 'gripper_left2', 'gripper_left3', 'gripper_right1', 'gripper_right2', 'gripper_right3', 'tcp']
ap = argparse.ArgumentParser(); ap.add_argument("--frei", action="store_true"); ap.add_argument("--zurueck", action="store_true")
ap.add_argument("--trichter-finger-frei", action="store_true", help="NUR finger_links/rechts x trichter erlauben (Trichter-Ablage; Modell liegt dort daneben). Danach --zurueck!")
ap.add_argument("--paar", nargs=2, metavar=("LINK1", "LINK2"), help="genau dieses Paar erlauben (14.9.: gripper_base joint4 in der Kontrollpose nach der J6-Drehung - "
                                                                    "das Modell sieht dort eine Selbstkollision, real ist Luft; MoveIt plant sonst nicht aus dem Startzustand). Danach --zurueck!")
a = ap.parse_args()
rclpy.init(); n = Node("acm_greifer")
g = n.create_client(GetPlanningScene, "/get_planning_scene"); g.wait_for_service(10)
rq = GetPlanningScene.Request(); rq.components.components = PlanningSceneComponents.ALLOWED_COLLISION_MATRIX
f = g.call_async(rq); rclpy.spin_until_future_complete(n, f, timeout_sec=10); acm = f.result().scene.allowed_collision_matrix
names = list(acm.entry_names)
frei = sum(1 for i, x in enumerate(names) for j, y in enumerate(names) if (x in GREIFER or y in GREIFER) and i < j and acm.entry_values[i].enabled[j])
print("ACM: %d Greifer-Paare erlaubt (von %d)" % (frei, sum(1 for i, x in enumerate(names) for j, y in enumerate(names) if (x in GREIFER or y in GREIFER) and i < j)))
if not (a.frei or a.zurueck or a.trichter_finger_frei or a.paar): sys.exit(0)
if a.paar:
    i, j = names.index(a.paar[0]), names.index(a.paar[1]); k = 0
    for p_, q_ in ((i, j), (j, i)):
        if not acm.entry_values[p_].enabled[q_]: acm.entry_values[p_].enabled[q_] = True; k += 1
    ps = PlanningScene(); ps.is_diff = True; ps.allowed_collision_matrix = acm
    ap_ = n.create_client(ApplyPlanningScene, "/apply_planning_scene"); ap_.wait_for_service(10)
    f = ap_.call_async(ApplyPlanningScene.Request(scene=ps)); rclpy.spin_until_future_complete(n, f, timeout_sec=10)
    print("%s x %s FREIGEGEBEN (%d Eintraege, ok=%s) - danach --zurueck!" % (a.paar[0], a.paar[1], k, f.result().success)); sys.exit(0)
if a.trichter_finger_frei:
    k = 0; j = names.index("trichter")
    for x in ("finger_links", "finger_rechts"):
        i = names.index(x)
        for p_, q_ in ((i, j), (j, i)):
            if not acm.entry_values[p_].enabled[q_]: acm.entry_values[p_].enabled[q_] = True; k += 1
    ps = PlanningScene(); ps.is_diff = True; ps.allowed_collision_matrix = acm
    ap_ = n.create_client(ApplyPlanningScene, "/apply_planning_scene"); ap_.wait_for_service(10)
    f = ap_.call_async(ApplyPlanningScene.Request(scene=ps)); rclpy.spin_until_future_complete(n, f, timeout_sec=10)
    print("finger x trichter FREIGEGEBEN (%d Eintraege, ok=%s) - nach der Ablage --zurueck!" % (k, f.result().success)); sys.exit(0)
if a.zurueck:
    srdf = subprocess.run(["ros2", "param", "get", "/move_group", "robot_description_semantic"], capture_output=True, text=True).stdout
    srdf = srdf[srdf.index("<"):] if "<" in srdf else ""
    paare = set()
    for d in ET.fromstring(srdf).iter("disable_collisions"): paare.add((d.get("link1"), d.get("link2"))); paare.add((d.get("link2"), d.get("link1")))
    print("SRDF: %d disable_collisions-Paare" % (len(paare) // 2))
k = 0
for i, x in enumerate(names):
    for j, y in enumerate(names):
        if not (x in GREIFER or y in GREIFER) or i == j: continue
        soll = True if a.frei else ((x, y) in paare)
        if acm.entry_values[i].enabled[j] != soll: acm.entry_values[i].enabled[j] = soll; k += 1
ps = PlanningScene(); ps.is_diff = True; ps.allowed_collision_matrix = acm
ap_ = n.create_client(ApplyPlanningScene, "/apply_planning_scene"); ap_.wait_for_service(10)
f = ap_.call_async(ApplyPlanningScene.Request(scene=ps)); rclpy.spin_until_future_complete(n, f, timeout_sec=10)
print("%s: %d Eintraege geaendert, ok=%s" % ("FREIGEGEBEN (Greifer abgeschraubt!)" if a.frei else "ZURUECKGESETZT (wie SRDF)", k, f.result().success))
