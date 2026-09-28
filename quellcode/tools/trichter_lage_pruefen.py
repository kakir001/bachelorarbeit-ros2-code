#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""Trichterlage pruefen: Kegel als Planning-Scene-Objekt, Neigungsrichtung (yaw) durchdrehen.

Nimmt die Trichter-Geometrie aus dem xacro (trichter_montiert:=true, gleiche Kaesten
wie im URDF), stellt sie als Kollisionsobjekt "trichter_test" an die Lage aus
trichter_pose.xacro und prueft fuer jeden yaw-Schritt, ob die angelernten Greifpunkte
(teach_punkte.json) gueltig sind. Dazu die FK des TCP je Punkt.

Zweck: den Platzhalter yaw in urdf/trichter_pose.xacro bestimmen, OHNE den Stack je
Winkel neu zu starten. Der Stack muss laufen (Sim reicht), TRICHTER_MONTIERT=0 -
sonst steht der Trichter doppelt in der Szene.

Befund 2026-09-10: bei KEINEM yaw gueltig - Finger und Greifermechanik stossen
immer an. Der Modell-TCP steht am Greifpunkt bei z = 3 mm (im Auslaufrohr), die
Welle wurde aber bei 33-42 mm ueber Grund gefasst. Das ist keine yaw-Frage,
sondern ein z-Versatz zwischen Modell und Wirklichkeit von rund 30 mm - oder der
Trichter steht 25 mm tiefer (auf dem Tisch neben der Grundplatte). Muss am Aufbau
gemessen werden, bevor der Trichter eingeschaltet wird.

    python3 tools/trichter_lage_pruefen.py            # yaw 0..345 in 15-Grad-Schritten
    python3 tools/trichter_lage_pruefen.py --schritt 5
"""
import argparse, os, subprocess, tempfile
import json, math, sys, xml.etree.ElementTree as ET
import rclpy
from rclpy.node import Node
from moveit_msgs.srv import GetStateValidity, ApplyPlanningScene, GetPositionFK
from moveit_msgs.msg import RobotState, PlanningScene, CollisionObject
from shape_msgs.msg import SolidPrimitive
from geometry_msgs.msg import Pose, Quaternion
from sensor_msgs.msg import JointState
from std_msgs.msg import Header
ARM=["joint2_to_joint1","joint3_to_joint2","joint4_to_joint3","joint5_to_joint4","joint6_to_joint5","joint6output_to_joint6"]
WS=os.path.expanduser("~/ros2_ws")

def lade_pose():
    """x/y/z aus trichter_pose.xacro (einzige Quelle der Lage)."""
    import re
    t=open(WS+"/src/mycobot_world/urdf/trichter_pose.xacro").read()
    g=lambda k: float(re.search(r'name="trichter_%s"\s+value="([^"]+)"'%k,t).group(1))
    return g("x"),g("y"),g("z")

def urdf_mit_trichter():
    out=subprocess.run(["xacro",WS+"/src/mycobot_world/urdf/mycobot_world.urdf.xacro",
                        "trichter_montiert:=true","charuco_montiert:=false"],
                       capture_output=True,text=True,check=True).stdout
    f=tempfile.NamedTemporaryFile("w",suffix=".urdf",delete=False); f.write(out); f.close(); return f.name

def lade_koerper():
    r=ET.parse(urdf_mit_trichter()).getroot()
    l=[x for x in r.findall('link') if x.get('name')=='trichter'][0]
    out=[]
    for c in l.findall('collision'):
        o=c.find('origin'); xyz=[float(v) for v in o.get('xyz').split()]; rpy=[float(v) for v in o.get('rpy').split()]
        g=c.find('geometry')[0]
        if g.tag=='box':
            p=SolidPrimitive(type=SolidPrimitive.BOX, dimensions=[float(v) for v in g.get('size').split()])
        else:
            p=SolidPrimitive(type=SolidPrimitive.CYLINDER, dimensions=[float(g.get('length')),float(g.get('radius'))])
        out.append((p,xyz,rpy[2]))
    return out

def quat_z(yaw): return Quaternion(x=0.0,y=0.0,z=math.sin(yaw/2),w=math.cos(yaw/2))

class N(Node):
    def __init__(s):
        super().__init__("yaw_sweep")
        s.v=s.create_client(GetStateValidity,"/check_state_validity"); s.v.wait_for_service(10)
        s.ps=s.create_client(ApplyPlanningScene,"/apply_planning_scene"); s.ps.wait_for_service(10)
        s.fk=s.create_client(GetPositionFK,"/compute_fk"); s.fk.wait_for_service(10)
        s.koerper=lade_koerper()
    def call(s,cli,rq):
        f=cli.call_async(rq); rclpy.spin_until_future_complete(s,f,timeout_sec=10); return f.result()
    def setze_trichter(s,yaw,entfernen=False):
        co=CollisionObject(); co.header=Header(frame_id="robot_base"); co.id="trichter_test"
        co.operation=CollisionObject.REMOVE if entfernen else CollisionObject.ADD
        if not entfernen:
            co.pose=Pose(); co.pose.position.x=TX; co.pose.position.y=TY; co.pose.position.z=TZ; co.pose.orientation=quat_z(yaw)
            for p,xyz,rz in s.koerper:
                pp=Pose(); pp.position.x,pp.position.y,pp.position.z=xyz; pp.orientation=quat_z(rz)
                co.primitives.append(p); co.primitive_poses.append(pp)
        sc=PlanningScene(is_diff=True); sc.world.collision_objects.append(co)
        rq=ApplyPlanningScene.Request(scene=sc); return s.call(s.ps,rq).success
    def rs(s,grad,greifer):
        rs=RobotState(); js=JointState(); js.name=ARM+["gripper_controller"]
        js.position=[math.radians(g) for g in grad]+[greifer]; rs.joint_state=js; return rs
    def gueltig(s,grad,greifer=0.15):
        rq=GetStateValidity.Request(robot_state=s.rs(grad,greifer),group_name="arm")
        r=s.call(s.v,rq); links=sorted({c.contact_body_1 if c.contact_body_2=="trichter_test" else c.contact_body_2 for c in r.contacts})
        return r.valid,links
    def tcp(s,grad):
        rq=GetPositionFK.Request(); rq.header.frame_id="robot_base"; rq.fk_link_names=["tcp","gripper_base"]; rq.robot_state=s.rs(grad,0.15)
        r=s.call(s.fk,rq); return {n:p.pose for n,p in zip(r.fk_link_names,r.pose_stamped)}

ap=argparse.ArgumentParser(); ap.add_argument("--schritt",type=int,default=15)
ap.add_argument("--z",type=float,default=None,help="z des Trichterfusses in mm UEBERSCHREIBEN (Probe: steht er tiefer?)")
args=ap.parse_args()
TX,TY,TZ=lade_pose()
if args.z is not None: TZ=args.z/1000.0
rclpy.init(); n=N()
d=json.load(open(WS+"/teach_punkte.json"))
for k,v in [(k,v) for k,v in d.items() if not k.startswith("_")]:
    p=n.tcp(v["grad"]); t=p["tcp"].position; g=p["gripper_base"].position
    q=p["gripper_base"].orientation
    # Richtung der Greifer-y-Achse (Fingerachse) in robot_base
    yaw_g=math.degrees(math.atan2(2*(q.w*q.z+q.x*q.y),1-2*(q.y*q.y+q.z*q.z)))
    print(f"{k}: TCP=({t.x*1000:.1f},{t.y*1000:.1f},{t.z*1000:.1f}) mm  gripper_base=({g.x*1000:.1f},{g.y*1000:.1f},{g.z*1000:.1f})  yaw_base={yaw_g:.1f}")
print(f"Auslaufmitte im Modell: ({TX*1000:.1f},{TY*1000:.1f}), Fuss z={TZ*1000:.1f} mm")
print()
print("yaw[deg]  " + "  ".join(f"{k:>16s}" for k in d))
for yaw in range(0,360,args.schritt):
    n.setze_trichter(math.radians(yaw))
    res=[]
    for k,v in [(k,v) for k,v in d.items() if not k.startswith("_")]:
        ok,links=n.gueltig(v["grad"])
        res.append("OK" if ok else ",".join(l.replace("greifer_","").replace("finger_","f_") for l in links))
    print(f"{yaw:5d}     " + "  ".join(f"{r:>16s}" for r in res))
n.setze_trichter(0,entfernen=True)
n.destroy_node(); rclpy.shutdown()
