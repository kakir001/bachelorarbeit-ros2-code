#!/usr/bin/env python3
"""
Ein-Klick PLACE — MoveIt KOLLISIONSFREIER Plan + RViz-Vorschau + BESTAETIGEN-ausführen.

Sichere Version von click_place (Kollisions-Problem: send_radians direkt, keine
Wegplanung): Zielpose wird mit Achsen-Ausrichtungs-IK gefunden, MoveIt move_group
plant kollisionsfreien WEG aus der ECHTEN Startpose (Plattform/Base im Modell),
Vorschau in RViz, mit BESTAETIGUNG (E) FOLGT pymycobot der geplanten Trajektorie.

Einrichtung:
  demo.launch use_fake_hardware:=true use_rviz:=true  (move_group + RViz + Modell)
  rs_launch 1280x720 + aligned depth                  (Kamera)
  python3 click_place_moveit.py                        (pymycobot Port wird HIER geöffnet)

Mouse: Links = Ablage-Punkt   Rechts = löschen
Keys:
  O / C = Greifer auf / zu (Schraube waagerecht laden, mit C greifen)
  P     = PLANEN: Schraube-senkrecht Pose -> kollisionsfreier Weg -> RViz-Vorschau
  E     = bestätigen & AUSFUEHREN (folgt dem geplanten Weg)
  A     = Schrauben-Achse wechseln (kommt Schraube nicht senkrecht: Z->X->Y)
  Pfeile / 8/2/4/6 = Ziel 1cm verschieben (xy)  ;  +/- = z 1cm  (oben=+X links=+Y)
  S     = aktuellen Offset als PERMANENTEN Standard speichern (.place_offset.json)
  H     = PLANEN home (Null) ; danach mit E ausführen
  R     = ABLEGEN (Greifer auf)
  Q     = beenden
"""
import math, os, json, threading, time
import cv2, numpy as np
import rclpy
from rclpy.node import Node
from rclpy.action import ActionClient
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy, DurabilityPolicy
from std_msgs.msg import Bool
from sensor_msgs.msg import Image, CameraInfo, JointState
from geometry_msgs.msg import PointStamped
from visualization_msgs.msg import Marker
from moveit_msgs.action import MoveGroup
from moveit_msgs.msg import Constraints, JointConstraint, DisplayTrajectory, RobotState
import tf2_ros, tf2_geometry_msgs  # noqa

CAMERA_FRAME='camera_color_optical_frame'; ROBOT_BASE='robot_base'
PORT='/dev/ttyTHS1'; BAUD=1000000; GROUP='arm'   # serieller Port/Baud + MoveIt-Planungsgruppe
# ARM_JOINTS: Reihenfolge der 6 Armgelenke exakt wie in URDF/MoveIt. Vertrag für
# JointConstraints, RobotState-Startpose und Trajektorien-Indizierung.
ARM_JOINTS=['joint2_to_joint1','joint3_to_joint2','joint4_to_joint3','joint5_to_joint4','joint6_to_joint5','joint6output_to_joint6']
HOVER_OFF=0.13; PLAYBACK_VEL=0.5    # TCP-Schwebehöhe (m) über Ziel; rad/s Trajektorien-Wiedergabe (langsam/sicher)
# Permanente XY(-Z) Offset-Korrektur (TCP ~2.5cm + Seiten-Griff-Geometrie Band-Aid).
# wird im robot_base Frame als konstant angenommen; mit S hierhin geschrieben, beim Start geladen.
# Grund: der reale TCP weicht ~2.5cm von der Modellspitze ab (Seiten-Griff-Geometrie); statt
# das Modell zu korrigieren, wird ein konstanter Offset auf das Ziel addiert (dokumentierter Band-Aid).
OFFSET_FILE=os.path.expanduser('~/ros2_ws/.place_offset.json')
def load_default_off():
    """Gespeicherten Standard-Offset (x,y,z in m) aus der JSON-Datei laden; sonst Null-Vektor."""
    try:
        with open(OFFSET_FILE) as f: d=json.load(f)
        return np.array([float(d['x']),float(d['y']),float(d['z'])])
    except Exception:
        return np.zeros(3)

# ---- FK + Achsen-Ausrichtungs-IK (mycobot_280_jn) ----
# Identische eigenständige FK/IK wie in click_place.py; hier dient sie NUR dazu, ein
# GELENK-Ziel (Schraube senkrecht) zu finden. Den kollisionsfreien WEG dorthin plant
# danach MoveIt (siehe _plan_thread). Zusätzlich gibt es hier fk_pts + in_base_box, um
# Kandidaten vorab auszusortieren, die in die Roboter-Basiskiste ragen würden.
def rpy(r,p,y):
    """Roll-Pitch-Yaw (URDF) -> 3x3-Rotation: Rz(y) @ Ry(p) @ Rx(r)."""
    cr,sr=np.cos(r),np.sin(r);cp,sp=np.cos(p),np.sin(p);cy,sy=np.cos(y),np.sin(y)
    return np.array([[cy,-sy,0],[sy,cy,0],[0,0,1]])@np.array([[cp,0,sp],[0,1,0],[-sp,0,cp]])@np.array([[1,0,0],[0,cr,-sr],[0,sr,cr]])
def Tt(xyz,rr):
    """4x4-Homogen-Transformation aus Translation xyz und rpy-Rotation rr."""
    M=np.eye(4);M[:3,:3]=rpy(*rr);M[:3,3]=xyz;return M
def Tz(q):
    """4x4-Rotation um lokale Z-Achse um Gelenkwinkel q."""
    M=np.eye(4);c,s=np.cos(q),np.sin(q);M[:3,:3]=[[c,-s,0],[s,c,0],[0,0,1]];return M
# _O: Joint-Origins aus URDF; _OC: als Matrizen; _FGB: Flansch->gripper_base; _FTCP: gripper_base->TCP.
_O=[((0,0,0.15756),(0,0,0)),((0,0,-0.001),(0,1.5708,-1.5708)),((-0.1104,0,0),(0,0,0)),
    ((-0.096,0,0.06462),(0,0,-1.5708)),((0,-0.07318,0),(1.5708,-1.5708,0)),((0,0.0456,0),(-1.5708,0,0))]
_OC=[Tt(*o) for o in _O]; _FGB=Tt((0,0,0.034),(1.579,0,0.8406)); _FTCP=Tt((0,0.110,-0.01),(0,0,0))
# _LU/_LL: obere/untere Gelenkgrenzen (rad) aus URDF.
_LU=np.array([2.9321,2.4434,2.6179,2.6179,2.7925,3.14159]); _LL=-np.array([2.9321,2.4434,2.6179,2.6179,2.7052,3.14159])
_DOWN=np.array([0,0,-1.0])   # "senkrecht nach unten" im robot_base-Frame
def fk(q):
    """FK: Gelenkwinkel q(6) -> (TCP-Position, gripper_base-Rotation, z-Höhen). Siehe click_place.fk."""
    M=np.eye(4); zs=[]
    for i in range(6): M=M@_OC[i]@Tz(q[i]); zs.append(M[2,3])
    Mgb=M@_FGB; return (Mgb@_FTCP)[:3,3], Mgb[:3,:3], zs+[Mgb[2,3]]
def fk_pts(q):
    """alle Gelenk-Origins + gripper_base Positionen (Base-Box Kollisions-Check).

    Liefert die 3D-Positionen aller Gelenk-Ursprungspunkte und des gripper_base. Damit
    prüft _place_joint_goal, ob eine IK-Lösung mit irgendeinem Kettenpunkt in die
    Basiskiste ragt (self-collision mit dem eigenen Sockel).
    """
    M=np.eye(4); P=[]
    for i in range(6): M=M@_OC[i]@Tz(q[i]); P.append(M[:3,3].copy())
    P.append((M@_FGB)[:3,3].copy()); return P
# robot_base Box: 0.15x0.11x0.11, z 0..0.11 (mycobot_world.urdf.xacro)
def in_base_box(p,pad=0.02):
    """True, wenn Punkt p (robot_base) innerhalb der Basiskiste (+ Sicherheitsrand pad) liegt.

    Die reale Basis ist ein Kasten von 0.15x0.11x0.11 m um den Ursprung; ein Kettenpunkt
    darin bedeutet Selbstkollision. pad=2cm gibt einen Sicherheitsabstand.
    """
    return abs(p[0])<0.075+pad and abs(p[1])<0.055+pad and -pad<p[2]<0.11+pad
def err_align(q,p,ax):
    """IK-Fehler: TCP-Positionsfehler (3) + Ausrichtung der Achse ax nach unten (3). Siehe click_place."""
    tcp,Rc,_=fk(q); return np.concatenate([tcp-p, np.cross(Rc[:,ax],_DOWN)])
def ik_align(p,ax,seed,it=120):
    """Gedämpfte Newton-IK (Levenberg-Marquardt), finite-Differenzen-Jacobi. Siehe click_place.ik_align."""
    q=np.array(seed,float)
    for _ in range(it):
        e=err_align(q,p,ax)
        if np.linalg.norm(e)<1e-4: break
        J=np.zeros((6,6))
        for k in range(6):
            qq=q.copy();qq[k]+=1e-6;J[:,k]=(err_align(qq,p,ax)-e)/1e-6
        q=np.clip(q+np.linalg.solve(J.T@J+0.01*np.eye(6),-J.T@e),_LL,_LU)
    return q,float(np.linalg.norm(err_align(q,p,ax)))
SEEDS=[[0,0.4,-2.0,0.3,0,0],[-1.0,0,-1.5,1.0,0,0],[0,-0.3,-1.0,0.8,0.8,0],[-0.5,0.3,-1.8,1.2,-0.5,0.5],
       [0.8,0.3,-1.8,0.5,0,0],[-0.3,-0.5,-0.8,1.0,1.2,0],[0.3,0.5,-2.2,0.0,-1.0,0.5],
       [-0.8,0.8,-2.2,0.3,0.5,-0.5],[0,0.8,-1.5,-0.5,1.0,0],[-1.2,0.3,-1.2,1.2,0.8,0.5]]

def img_bgr(m):
    """ROS-Farbbild -> OpenCV-BGR (rgb8 kanalgetauscht, sonst Kopie)."""
    a=np.frombuffer(m.data,np.uint8)
    if m.encoding.lower()=='rgb8': return cv2.cvtColor(a.reshape((m.height,m.width,3)),cv2.COLOR_RGB2BGR)
    return a.reshape((m.height,m.width,3)).copy()
def depth_arr(m):
    """ROS-Tiefenbild -> 2D-Array (16UC1=mm, 32FC1=m); unbekannte Kodierung -> Fehler."""
    if m.encoding=='16UC1': return np.frombuffer(m.data,np.uint16).reshape((m.height,m.width))
    if m.encoding=='32FC1': return np.frombuffer(m.data,np.float32).reshape((m.height,m.width))
    raise ValueError(m.encoding)


class ClickPlaceMoveit(Node):
    """SICHERE Ein-Klick-PLACE-Variante: eigenes IK-Gelenkziel + MoveIt-Wegplanung + Bestätigung.

    Unterschied zu click_place.py: statt direkt per send_radians zu fahren, wird das
    Schraube-senkrecht-Gelenkziel an MoveIt übergeben, das aus der ECHTEN Startpose
    (per pymycobot gelesen) einen KOLLISIONSFREIEN Weg plant und ihn in RViz als Ghost
    zeigt. Erst 'E' lässt pymycobot der geplanten Trajektorie (abgetastet) folgen.
    Zusätzlich: verschiebbarer Ziel-Offset ('nudge', Pfeile/8246/+-) mit persistentem
    Standard ('S' -> .place_offset.json), um TCP-Abweichung/Griff-Geometrie zu kompensieren.
    """
    def __init__(self):
        super().__init__('click_place_moveit')
        # Kamera-Intrinsik + Bild/Tiefe (spin-Thread schreibt, GUI liest; Lock schützt).
        self._K=None;self._bgr=None;self._depth=None;self._lock=threading.Lock()
        self._def_off=load_default_off()   # persistenter Standard-Offset (beim Start geladen)
        # _off: aktueller kumulativer Offset (startet auf Standard); _screw_axis: angenommene Schraubenachse.
        self._px=None;self._tgt=None;self._screw_axis=2;self._off=self._def_off.copy()
        self._plan=None;self._plan_kind=''   # zuletzt geplante Trajektorie + Art ('place' / 'home')
        self._planning=False;self._busy=False  # Reentranz-Flags: Planung läuft / Ausführung läuft
        self._status='Warte auf Kamera/MoveGroup...'
        # TF: statische Kamera->Basis-Transformation (eye-to-hand-Kalibrierung).
        self._tf=tf2_ros.Buffer(); tf2_ros.TransformListener(self._tf,self)
        self._mk=self.create_publisher(Marker,'/click_place/marker',5)          # Zielpunkt-Marker (RViz)
        self._disp=self.create_publisher(DisplayTrajectory,'/display_planned_path',5)  # geplanter Ghost (RViz)
        self._mg=ActionClient(self,MoveGroup,'/move_action')   # MoveIt-Wegplanung
        qos=QoSProfile(reliability=ReliabilityPolicy.RELIABLE,history=HistoryPolicy.KEEP_LAST,depth=1)
        self.create_subscription(CameraInfo,'/camera/color/camera_info',self._oninfo,qos)
        self.create_subscription(Image,'/camera/color/image_raw',self._oncolor,10)
        self.create_subscription(Image,'/camera/aligned_depth_to_color/image_raw',self._ondepth,10)
        # pymycobot: seriellen Port öffnen; hier NUR für Startpose-Lesung und Trajektorien-Wiedergabe.
        from pymycobot import MyCobot280
        self.mc=MyCobot280(PORT,BAUD); time.sleep(0.8)
        try: self.mc.power_on()
        except Exception: pass
        # NOT-AUS: dieses Werkzeug schreibt per pymycobot DIREKT auf den Port — der
        # Bridge-Enforcement-Weg greift hier nicht. Darum eigener /estop-Abonnent
        # (gelatcht, gleiche QoS wie estop_button): STOP -> sofort mc.stop() aus dem
        # Spin-Thread (pymycobot serialisiert intern per thread_lock) + Abbruch der
        # Wiedergabe-Schleife in _exec_thread über das Flag.
        eq=QoSProfile(depth=1)
        eq.durability=DurabilityPolicy.TRANSIENT_LOCAL
        eq.reliability=ReliabilityPolicy.RELIABLE
        self._estop=False
        self.create_subscription(Bool,'/estop',self._on_estop,eq)

    def _on_estop(self,m):
        """NOT-AUS-Callback: Flag setzen; bei STOP sofort mc.stop() — die Servos behalten
        ihr Drehmoment (der Arm fällt NICHT), nur die Bewegung wird angehalten. Der
        alte Plan wird verworfen (Startpose stimmt nach dem Abbruch nicht mehr)."""
        was=self._estop; self._estop=bool(m.data)
        if self._estop:
            self._plan=None
            for _ in range(3):
                try: self.mc.stop()
                except Exception as e: self.get_logger().warn(f'NOT-AUS stop(): {e}')
                time.sleep(0.02)
            self._status='*** NOT-AUS *** — Bewegung gestoppt'
            self.get_logger().warn(self._status)
        elif was:
            self._status='NOT-AUS freigegeben — mit P/H neu planen'
            self.get_logger().info(self._status)

    def _oninfo(self,m): self._K=np.array(m.k,np.float64).reshape(3,3)  # 3x3-Intrinsik puffern
    def _oncolor(self,m):
        try:
            with self._lock:
                self._bgr=img_bgr(m)
                if self._status.startswith('Warte'): self._status='Bereit — Ablage-Punkt anklicken'
        except Exception: pass
    def _ondepth(self,m):
        try:
            with self._lock: self._depth=depth_arr(m)
        except Exception: pass

    def _cur_real(self):
        """Aktuelle ECHTE Gelenkstellung (rad) vom Roboter lesen (pymycobot get_radians).

        Wird als MoveIt-Startpose (RobotState) benutzt, damit der Planer aus der tatsächlichen
        Ist-Lage plant und nicht aus einer fake /joint_states. Fallback [0]*6 bei Lesefehler.
        """
        try:
            a=self.mc.get_radians()
            if a and len(a)==6: return [float(x) for x in a]
        except Exception: pass
        return [0.0]*6

    def pixel_to_base(self,u,v):
        """Pixel (u,v) -> 3D-Punkt im robot_base-Frame (Median-Tiefe, Rückprojektion, TF). Siehe click_place."""
        if self._K is None: return None
        with self._lock: depth=self._depth
        if depth is None: return None
        h,w=depth.shape
        if not(0<=v<h and 0<=u<w): return None
        reg=depth[max(0,v-3):v+4,max(0,u-3):u+4]; val=reg[reg>0]
        if len(val)==0: return None
        z=float(np.median(val)); z=z/1000.0 if z>100 else z
        fx,fy=self._K[0,0],self._K[1,1];cx,cy=self._K[0,2],self._K[1,2]
        pc=PointStamped();pc.header.frame_id=CAMERA_FRAME;pc.header.stamp=self.get_clock().now().to_msg()
        pc.point.x=(u-cx)*z/fx;pc.point.y=(v-cy)*z/fy;pc.point.z=z
        try: pb=self._tf.transform(pc,ROBOT_BASE,timeout=rclpy.time.Duration(seconds=2.0))
        except Exception as e: self.get_logger().warn(f'TF:{e}'); return None
        return np.array([pb.point.x,pb.point.y,pb.point.z])

    def on_click(self,u,v):
        """Linksklick: Ziel setzen, Offset auf Standard zurücksetzen, RViz-Marker + Status. Ignoriert während Bewegung/Planung."""
        if self._busy or self._planning: return
        pb=self.pixel_to_base(u,v)
        if pb is None: self._status=f'Keine Tiefe ({u},{v})'; return
        self._px=(u,v);self._tgt=pb;self._plan=None;self._off=self._def_off.copy()
        self._status=f'Ziel [{pb[0]:.3f},{pb[1]:.3f},{pb[2]:.3f}] r={math.hypot(pb[0],pb[1])*1000:.0f}mm — P: planen'
        self.get_logger().info(self._status)
        m=Marker();m.header.frame_id=ROBOT_BASE;m.header.stamp=self.get_clock().now().to_msg()
        m.ns='place';m.id=0;m.type=Marker.SPHERE;m.action=Marker.ADD
        m.pose.position.x=float(pb[0]);m.pose.position.y=float(pb[1]);m.pose.position.z=float(pb[2])
        m.pose.orientation.w=1.0;m.scale.x=m.scale.y=m.scale.z=0.02;m.color.b=1.0;m.color.a=1.0
        self._mk.publish(m)

    def _place_joint_goal(self):
        """Gelenk-Zielkonfiguration für die aktuelle Ablage suchen (Schraube senkrecht, Base-Box-frei).

        Ziel = angeklickter Punkt + kumulativer Offset + HOVER_OFF in z (Schwebehöhe). Für
        jeden Seed wird ik_align gelöst und die Lösung akzeptiert, wenn: Restfehler klein,
        Gelenkgrenzen eingehalten, gewählte Greifer-Achse zeigt nach unten (Rc[2,ax]<-0.95),
        Bodenabstand mz>0.02 UND kein Kettenpunkt in der Basiskiste (in_base_box). Gewählt
        wird die Lösung mit größtem Bodenabstand. Rückgabe: (Gelenkliste, Info-Text) oder
        (None, Fehlertext).
        """
        if self._tgt is None: return None,'Erst einen Punkt waehlen!'
        p=np.array([self._tgt[0]+self._off[0],self._tgt[1]+self._off[1],self._tgt[2]+self._off[2]+HOVER_OFF]); ax=self._screw_axis; best=None
        for s in SEEDS:
            q,res=ik_align(p,ax,s)
            if res<1.5e-3 and np.all(q>=_LL-1e-6)and np.all(q<=_LU+1e-6):
                _,Rc,zs=fk(q); mz=min(zs)
                if (Rc[2,ax]<-0.95 and mz>0.02
                        and not any(in_base_box(pt) for pt in fk_pts(q))   # nicht in die Base-Box ragen
                        and (best is None or mz>best[1])): best=(q,mz)
        if best is None: return None,f'Keine IK (keine Schraube-senkrecht + Base-Box-freie Pose, Achse={["X","Y","Z"][ax]}) — ferner/hoeherer Punkt oder A'
        return list(best[0]),f'Schrauben-Achse={["X","Y","Z"][ax]}, Abstand {best[1]*1000:.0f}mm'

    def plan_place(self):
        """'P': Ablage-Gelenkziel suchen und (falls gefunden) MoveIt-Wegplanung dafür anstoßen."""
        if self._busy or self._planning: return
        goal,msg=self._place_joint_goal()
        if goal is None: self._status=msg; return
        self._plan_kind='place'; self._do_plan(goal,f'WIRD GEPLANT (place: {msg})')

    def plan_home(self):
        """'H': Weg zur Null-Stellung (Home) planen (danach mit 'E' ausführen)."""
        if self._busy or self._planning: return
        self._plan_kind='home'; self._do_plan([0.0]*6,'WIRD GEPLANT (home)')

    def _do_plan(self,joint_goal,msg):
        """Planung im Hintergrund-Thread starten (GUI bleibt reaktiv). Setzt _planning."""
        self._planning=True;self._plan=None;self._status=msg;self.get_logger().info(msg)
        threading.Thread(target=self._plan_thread,args=(joint_goal,),daemon=True).start()

    def _plan_thread(self,joint_goal):
        """MoveIt-Wegplanung (plan_only) für ein Gelenk-Ziel; synchron per Polling der Futures.

        Baut ein MoveGroup-Goal mit: Gruppe, Planungsversuchen/-zeit, konservativem
        Geschwindigkeits-/Beschleunigungs-Scaling (0.25). Wichtig: als Startpose wird die
        ECHTE, per pymycobot gelesene Ist-Lage gesetzt (statt der fake /joint_states aus
        use_fake_hardware) — nur so plant MoveIt kollisionsfrei relativ zur wahren Roboterlage.
        Das Ziel wird als JointConstraints (enge Toleranz 0.03) übergeben, plan_only=True.
        Bei Erfolg (error_code==1) wird die Trajektorie gespeichert und als DisplayTrajectory
        für die RViz-Vorschau publiziert; Fehler/Kollision -> Statusmeldung, kein Plan.
        """
        if not self._mg.wait_for_server(timeout_sec=4.0):
            self._status='move_group FEHLT!';self._planning=False;return
        goal=MoveGroup.Goal();req=goal.request;req.group_name=GROUP
        req.num_planning_attempts=10;req.allowed_planning_time=5.0
        req.max_velocity_scaling_factor=0.25;req.max_acceleration_scaling_factor=0.25
        # ECHTE Startpose (pymycobot-Lesung statt fake /joint_states)
        st=RobotState();js=JointState();js.name=ARM_JOINTS;js.position=self._cur_real()
        st.joint_state=js;st.is_diff=False;req.start_state=st
        # Arbeitsraum-Grenzen (+/-1 m) als Suchraum-Begrenzung.
        wp=req.workspace_parameters;wp.header.frame_id=ROBOT_BASE
        wp.min_corner.x=wp.min_corner.y=wp.min_corner.z=-1.0
        wp.max_corner.x=wp.max_corner.y=wp.max_corner.z=1.0
        # Gelenk-Ziel als JointConstraints (eng, damit MoveIt genau unsere IK-Lösung ansteuert).
        c=Constraints()
        for n,v in zip(ARM_JOINTS,joint_goal):
            jc=JointConstraint();jc.joint_name=n;jc.position=float(v)
            jc.tolerance_above=0.03;jc.tolerance_below=0.03;jc.weight=1.0
            c.joint_constraints.append(jc)
        req.goal_constraints.append(c)
        goal.planning_options.plan_only=True   # nur planen, nicht ausführen (Vorschau + 'E'-Bestätigung)
        # Goal senden und auf Annahme pollen (max 6 s), da dieser Thread synchron arbeitet.
        fut=self._mg.send_goal_async(goal)
        t0=time.time()
        while not fut.done() and time.time()-t0<6: time.sleep(0.05)
        gh=fut.result()
        if gh is None or not gh.accepted: self._status='Plan ABGELEHNT';self._planning=False;return
        # Auf das eigentliche Planungsergebnis warten (max 10 s).
        rf=gh.get_result_async();t0=time.time()
        while not rf.done() and time.time()-t0<10: time.sleep(0.05)
        res=rf.result().result if rf.done() else None
        if res is None or res.error_code.val!=1:
            code=res.error_code.val if res else '??'
            self._status=f'Plan FEHLGESCHLAGEN (code={code}) — evtl. Kollision/nicht erreichbar';self._plan=None;self._planning=False;return
        self._plan=res.planned_trajectory
        # Ghost in RViz anzeigen.
        d=DisplayTrajectory();d.trajectory_start=res.trajectory_start;d.trajectory.append(res.planned_trajectory)
        self._disp.publish(d)
        n=len(res.planned_trajectory.joint_trajectory.points)
        self._status=f'PLAN BEREIT ({n} Punkte) — RViz-Vorschau, E: bestaetigen&ausfuehren'
        self.get_logger().info(self._status);self._planning=False

    def execute(self):
        """'E': geplante Trajektorie ausführen (im Hintergrund-Thread). Nur wenn ein Plan vorliegt."""
        if self._estop:
            self._status='NOT-AUS aktiv — erst freigeben!'; return
        if self._busy or self._planning or self._plan is None:
            if self._plan is None: self._status='Erst mit P/H planen!'
            return
        self._busy=True;threading.Thread(target=self._exec_thread,daemon=True).start()

    def _exec_thread(self):
        """Der geplanten MoveIt-Trajektorie folgen, indem abgetastete Wegpunkte per send_radians angefahren werden.

        Die MoveIt-Trajektorie kann viele Punkte haben; sie wird auf ~18 Stützstellen
        heruntergerechnet (step), der Endpunkt aber immer angehängt, damit das exakte Ziel
        erreicht wird. names.index bildet die Trajektorien-Gelenkreihenfolge auf ARM_JOINTS ab
        (die Reihenfolge muss nicht übereinstimmen). Pro Wegpunkt wird die Wartezeit aus dem
        größten Gelenk-Delta und PLAYBACK_VEL (rad/s) berechnet -> gleichmäßig langsame,
        sichere Wiedergabe. Am Ende wird auf Stillstand gewartet (is_moving).
        """
        try:
            jt=self._plan.joint_trajectory;names=jt.joint_names
            idx=[names.index(j) for j in ARM_JOINTS]   # Trajektorien-Spalten -> ARM_JOINTS-Reihenfolge
            pts=jt.points
            step=max(1,len(pts)//18); sel=list(pts[::step])   # auf ~18 Stützstellen abtasten
            if sel[-1] is not pts[-1]: sel.append(pts[-1])     # Endpunkt garantiert mitnehmen
            self._status=f'LAEUFT (folgt geplantem Weg, {len(sel)} Punkte)...'
            self.get_logger().info(self._status)
            prev=self._cur_real()
            for p in sel:
                # NOT-AUS: Wiedergabe sofort abbrechen (mc.stop() hat der Callback
                # bereits gesendet); finally setzt _busy zurück.
                if self._estop:
                    self._status='*** NOT-AUS *** — Wiedergabe abgebrochen'
                    self.get_logger().warn(self._status)
                    return
                q=[p.positions[i] for i in idx]
                dmax=max(abs(a-b) for a,b in zip(q,prev))   # größte Gelenkänderung zum Vorpunkt
                self.mc.send_radians([float(x) for x in q],30)
                # Wartezeit ~ Weg/Geschwindigkeit — in 50-ms-Schritten, damit ein
                # NOT-AUS auch WAEHREND eines langen Wegpunkt-Schritts die Schleife beendet.
                t_wait=max(0.08,dmax/PLAYBACK_VEL); t0=time.time()
                while time.time()-t0<t_wait and not self._estop:
                    time.sleep(0.05)
                prev=q
            # Falls NOT-AUS genau beim letzten Wegpunkt kam: nicht "fertig" melden.
            if self._estop:
                self._status='*** NOT-AUS *** — Wiedergabe abgebrochen'
                self.get_logger().warn(self._status)
                return
            for _ in range(8):
                time.sleep(0.5)
                try:
                    if self.mc.is_moving()==0: break
                except Exception: pass
            self._plan=None   # Plan verbraucht -> kein versehentliches erneutes Ausführen
            self._status=('PLATZIERT — mit R ablegen' if self._plan_kind=='place' else 'HOME fertig')
            self.get_logger().info(self._status)
        finally: self._busy=False

    def grip(self,opn):
        """'O'/'C': Greifer öffnen (Wert 100) / schließen (Wert 0), Kraft 50, Modus 1."""
        if self._busy or self._estop: return
        try: self.mc.set_gripper_value(100 if opn else 0,50,1)
        except Exception as e: self.get_logger().warn(str(e))
        self._status='Greifer '+('AUF' if opn else 'ZU')
    def release(self):
        """'R': Greifer öffnen (Schraube ablegen). Danach H (home planen) + E (ausführen) zum Zurückfahren."""
        if self._busy or self._estop: return
        try: self.mc.set_gripper_value(100,50,1)
        except Exception as e: self.get_logger().warn(str(e))
        self._status='ABGELEGT (Greifer auf) — H dann E fuer home'
    def toggle_axis(self):
        """'A': angenommene Schraubenachse zyklisch wechseln (Z->X->Y); alten Plan verwerfen, neu planen nötig."""
        if self._busy or self._planning: return
        self._screw_axis=(self._screw_axis+1)%3;self._plan=None
        self._status=f'Schrauben-Achse={["X","Y","Z"][self._screw_axis]} — mit P erneut planen'

    def nudge(self,dx,dy,dz):
        """Ziel in robot_base 1cm verschieben + erneut planen (Greifer über das Loch bringen)."""
        if self._busy or self._planning or self._tgt is None: return
        self._off=self._off+np.array([dx,dy,dz])
        self._status=f'offset x={self._off[0]*100:+.0f} y={self._off[1]*100:+.0f} z={self._off[2]*100:+.0f}cm -> wird geplant'
        self.get_logger().info(f'>> NUDGE kumulativer offset (robot_base): x={self._off[0]*100:+.1f} y={self._off[1]*100:+.1f} z={self._off[2]*100:+.1f} cm  (mit S permanent speichern)')
        self.plan_place()

    def save_offset(self):
        """Aktuellen kumulativen Offset als permanenten Standard auf Platte schreiben."""
        try:
            with open(OFFSET_FILE,'w') as f:
                json.dump({'x':float(self._off[0]),'y':float(self._off[1]),'z':float(self._off[2])},f,indent=2)
            self._def_off=self._off.copy()
            self._status=f'OFFSET GESPEICHERT x={self._off[0]*100:+.1f} y={self._off[1]*100:+.1f} z={self._off[2]*100:+.1f}cm -> {OFFSET_FILE}'
            self.get_logger().info(self._status)
        except Exception as e:
            self._status=f'offset Speicher-FEHLER: {e}'; self.get_logger().error(self._status)

    def frame(self):
        """Anzeige-Bild bauen: Klick-Marker (grün=Plan bereit, gelb=nur gewählt), aktueller + Standard-Offset, Status, Legende."""
        with self._lock:
            if self._bgr is None: return None
            f=self._bgr.copy()
        h,w=f.shape[:2]
        if self._px is not None:
            u,v=self._px;col=(0,255,0) if self._plan is not None else (0,255,255)
            cv2.drawMarker(f,(u,v),col,cv2.MARKER_CROSS,22,2);cv2.circle(f,(u,v),10,col,2)
        cv2.rectangle(f,(0,0),(w,50),(40,40,40),-1)
        cv2.putText(f,self._status,(8,20),cv2.FONT_HERSHEY_SIMPLEX,0.5,(255,255,255),1,cv2.LINE_AA)
        off_txt=(f'offset x={self._off[0]*100:+.1f} y={self._off[1]*100:+.1f} z={self._off[2]*100:+.1f}cm'
                 f'  | Standard x={self._def_off[0]*100:+.1f} y={self._def_off[1]*100:+.1f} z={self._def_off[2]*100:+.1f}')
        cv2.putText(f,off_txt,(8,42),cv2.FONT_HERSHEY_SIMPLEX,0.42,(0,220,255),1,cv2.LINE_AA)
        cv2.rectangle(f,(0,h-22),(w,h),(40,40,40),-1)
        cv2.putText(f,'Links:Punkt O/C:Greifer P:Plan E:ausfuehren Pfeile|8/2/4/6:verschieben(xy) +/-:z A:Achse S:speichern H:home R:ablegen Q:beenden',
                    (4,h-6),cv2.FONT_HERSHEY_SIMPLEX,0.4,(180,180,180),1,cv2.LINE_AA)
        return f


# Pfeiltasten -> nudge (NumLock-unabhängig). waitKeyEx gibt VOLLES keysym zurück.
# ACHTUNG (2026-07-16, live beobachtet): dieser OpenCV/GTK-Build liefert Keysyms mit
# 0x100000-Präfix (KP_2 -> 1114034 = 0x10FFB2) — deshalb wird in der Hauptschleife
# mit kx&0xFFFF auf das nackte X11-Keysym normalisiert, sonst trifft KEIN Eintrag.
# Sowohl normale Pfeile (X11 0xFF51-54), Numpad-Pfeile (NumLock aus, KP_* 0xFF96-99)
# als auch Numpad-ZIFFERN (NumLock an, KP_0.. 0xFFB0-B9) und KP_+/- sind gebunden.
# Richtung = wie Numpad 8/2/4/6: oben=+X, unten=-X, links=+Y, rechts=-Y.
ARROW_KEYS={
    65362:(0.01,0,0),  65431:(0.01,0,0),    # Up    / KP_Up
    65364:(-0.01,0,0), 65433:(-0.01,0,0),   # Down  / KP_Down
    65361:(0,0.01,0),  65430:(0,0.01,0),    # Left  / KP_Left
    65363:(0,-0.01,0), 65432:(0,-0.01,0),   # Right / KP_Right
    65464:(0.01,0,0),                       # KP_8 (NumLock an)
    65458:(-0.01,0,0),                      # KP_2 (NumLock an)
    65460:(0,0.01,0),                       # KP_4 (NumLock an)
    65462:(0,-0.01,0),                      # KP_6 (NumLock an)
    65451:(0,0,0.01),                       # KP_Add      (+z)
    65453:(0,0,-0.01),                      # KP_Subtract (-z)
}

def mouse(ev,x,y,fl,node):
    # Maus-Callback: Links = Ziel wählen, Rechts = Ziel/Plan/Marker zurücksetzen.
    if ev==cv2.EVENT_LBUTTONDOWN: node.on_click(x,y)
    elif ev==cv2.EVENT_RBUTTONDOWN: node._px=None;node._tgt=None;node._plan=None;node._status='geloescht'

def main():
    # ROS + Knoten starten, spin in Daemon-Thread, OpenCV-GUI im Haupt-Thread.
    rclpy.init();node=ClickPlaceMoveit()
    threading.Thread(target=lambda: rclpy.spin(node),daemon=True).start()
    cv2.namedWindow('Click Place (MoveIt)',cv2.WINDOW_NORMAL);cv2.resizeWindow('Click Place (MoveIt)',1280,720)
    cv2.setMouseCallback('Click Place (MoveIt)',mouse,node)
    print('\n  CLICK PLACE (MoveIt) — Punkt anklicken, O/C laden, P planen(RViz), E ausfuehren\n')
    # GUI-Hauptschleife (~20 Hz). waitKeyEx liefert den VOLLEN Tastencode (für Pfeiltasten);
    # k=kx&0xFF ist der ASCII-Anteil für normale Buchstaben-/Zeichentasten.
    try:
        while rclpy.ok():
            fr=node.frame()
            if fr is not None: cv2.imshow('Click Place (MoveIt)',fr)
            kx=cv2.waitKeyEx(50); k=kx&0xFF
            # ksym: nacktes X11-Keysym (GTK-Build liefert Sondertasten mit 0x100000-Präfix)
            ksym=kx&0xFFFF
            if kx>255 and ksym in ARROW_KEYS: node.nudge(*ARROW_KEYS[ksym])
            elif k in (ord('o'),ord('O')): node.grip(True)
            elif k in (ord('c'),ord('C')): node.grip(False)
            elif k in (ord('p'),ord('P')): node.plan_place()
            elif k in (ord('e'),ord('E')): node.execute()
            elif k in (ord('a'),ord('A')): node.toggle_axis()
            elif k in (ord('s'),ord('S')): node.save_offset()
            elif k==ord('8'): node.nudge(0.01,0,0)
            elif k==ord('2'): node.nudge(-0.01,0,0)
            elif k==ord('4'): node.nudge(0,0.01,0)
            elif k==ord('6'): node.nudge(0,-0.01,0)
            elif k==ord('+'): node.nudge(0,0,0.01)
            elif k==ord('-'): node.nudge(0,0,-0.01)
            elif k in (ord('h'),ord('H')): node.plan_home()
            elif k in (ord('r'),ord('R')): node.release()
            elif k in (ord('q'),ord('Q')): break
            elif kx>255: node.get_logger().info(f'[Taste] nicht zugeordneter Sonder-Tastencode={kx} (falls Pfeiltaste, melde es mir, ich binde sie)')
    except KeyboardInterrupt: pass
    finally:
        cv2.destroyAllWindows();node.destroy_node();rclpy.shutdown()

if __name__=='__main__': main()
