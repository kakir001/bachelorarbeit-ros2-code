#!/usr/bin/env python3
"""
Ein-Klick PLACE: Ablage-Punkt in der Kamera anklicken -> Roboter dreht die
WAAGERECHT zwischen den Fingern gehaltene Schraube SENKRECHT (Anflug waagerecht)
und führt sie über diesen Punkt -> Sicht-Kontrolle (mit Roll aufrichten) -> ablegen.

Einrichtung:
  Terminal 1: ros2 launch <vo_rsp>  (TF: camera->robot_base, statisch eye-to-hand)
              ros2 launch realsense2_camera rs_launch.py (1280x720 + aligned depth)
  Terminal 2: python3 click_place.py   (Bewegung mit pymycobot; öffnet DEN Port)

Mouse: Links = Ablage-Punkt wählen    Rechts = löschen
Keys:
  O / C  = Greifer auf / zu (Schraube waagerecht laden, dann mit C greifen)
  G      = los: Schraube SENKRECHT halten, über das Ziel (langsam)  [prüft erst IK]
  A      = Schrauben-Achse wechseln (kommt Schraube nicht senkrecht: Z->X->Y testen)
  R      = ablegen: Greifer auf -> gerade nach oben zurück -> Home
  Z      = Home (Null)
  Q      = beenden
"""
import math, threading, time
from typing import Optional, Tuple
import cv2, numpy as np
import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy
from sensor_msgs.msg import Image, CameraInfo
from geometry_msgs.msg import PointStamped
from visualization_msgs.msg import Marker
import tf2_ros, tf2_geometry_msgs  # noqa

CAMERA_FRAME='camera_color_optical_frame'; ROBOT_BASE='robot_base'
PORT='/dev/ttyTHS1'; BAUD=1000000    # serieller Port + Baudrate zum myCobot (Jetson UART)
HOVER_OFF=0.06        # TCP so weit über dem angeklickten Punkt (m)
RELEASE_UP=0.12       # nach dem Ablegen gerade nach oben zurück (m)

# ---- URDF FK (mycobot_280_jn) ----
# Eigenständige Vorwärtskinematik (Forward Kinematics), aus der URDF von Hand
# übernommen, damit dieses Skript ohne MoveIt-Dienste FK/IK rechnen kann. Die
# festen Joint-Origins (_O) und die Greifer-/TCP-Offsets (_FGB, _FTCP) entsprechen
# exakt den <origin>-Angaben der URDF-Joints.
def rpy(r,p,y):
    """Roll-Pitch-Yaw (URDF-Konvention) -> 3x3-Rotationsmatrix.

    Reihenfolge wie in URDF: R = Rz(yaw) @ Ry(pitch) @ Rx(roll) (extrinsisch xyz).
    """
    cr,sr=np.cos(r),np.sin(r);cp,sp=np.cos(p),np.sin(p);cy,sy=np.cos(y),np.sin(y)
    return np.array([[cy,-sy,0],[sy,cy,0],[0,0,1]])@np.array([[cp,0,sp],[0,1,0],[-sp,0,cp]])@np.array([[1,0,0],[0,cr,-sr],[0,sr,cr]])
def Tt(xyz,rr):
    """Baut eine 4x4-Homogen-Transformation aus Translation xyz und rpy-Rotation rr."""
    M=np.eye(4);M[:3,:3]=rpy(*rr);M[:3,3]=xyz;return M
def Tz(q):
    """4x4-Rotation um die lokale Z-Achse um Winkel q (Gelenkbewegung eines Drehgelenks)."""
    M=np.eye(4);c,s=np.cos(q),np.sin(q);M[:3,:3]=[[c,-s,0],[s,c,0],[0,0,1]];return M
# _O: konstante Joint-Origins (Translation, rpy) der 6 Armgelenke aus der URDF.
_O=[((0,0,0.15756),(0,0,0)),((0,0,-0.001),(0,1.5708,-1.5708)),((-0.1104,0,0),(0,0,0)),
    ((-0.096,0,0.06462),(0,0,-1.5708)),((0,-0.07318,0),(1.5708,-1.5708,0)),((0,0.0456,0),(-1.5708,0,0))]
# _OC: dieselben Origins als vorberechnete 4x4-Matrizen. _FGB: fester Offset Flansch->
# gripper_base. _FTCP: Offset gripper_base->TCP (Werkzeugspitze zwischen den Fingern).
_OC=[Tt(*o) for o in _O]; _FGB=Tt((0,0,0.034),(1.579,0,0.8406)); _FTCP=Tt((0,0.110,-0.01),(0,0,0))
# _LU/_LL: obere/untere Gelenkwinkel-Grenzen (rad) aus der URDF; IK-Lösungen werden hierauf geklippt.
_LU=np.array([2.9321,2.4434,2.6179,2.6179,2.7925,3.14159]); _LL=-np.array([2.9321,2.4434,2.6179,2.6179,2.7052,3.14159])
def fk(q):
    """Vorwärtskinematik: Gelenkwinkel q(6) -> (TCP-Position, gripper_base-Rotation, z-Höhen).

    Verkettet die 6 Joint-Origins mit den jeweiligen Z-Drehungen, hängt den Greifer-
    Offset an und gibt zurück: TCP-Position (3), Rotationsmatrix von gripper_base (3x3,
    für die Achsen-Ausrichtung), sowie die z-Höhen aller Gelenk-Origins + gripper_base
    (für Kollisions-/Boden-Abstandsprüfungen).
    """
    M=np.eye(4); zs=[]
    for i in range(6): M=M@_OC[i]@Tz(q[i]); zs.append(M[2,3])
    Mgb=M@_FGB; Mt=Mgb@_FTCP
    return Mt[:3,3], Mgb[:3,:3], zs+[Mgb[2,3]]
_DOWN=np.array([0,0,-1.0])   # Ziel-Richtung "senkrecht nach unten" im robot_base-Frame
def err_align(q,p,axis):
    """pos(3) + Ausrichtung der gewählten Greifer-Achse nach UNTEN (Azimut frei).

    6er-Fehlervektor für die IK: die ersten 3 Komponenten sind der TCP-Positionsfehler
    (tcp - Ziel p). Die letzten 3 sind das Kreuzprodukt der gewählten Greifer-Achse mit
    der Nach-unten-Richtung; es wird 0, wenn die Achse exakt nach unten zeigt. Der
    Azimut (Drehung um die Vertikale) bleibt bewusst frei -> mehr Lösungen erreichbar.
    """
    tcp,Rc,_=fk(q)
    return np.concatenate([tcp-p, np.cross(Rc[:,axis],_DOWN)])
def ik_align(p,axis,seed,it=120):
    """Numerische inverse Kinematik (Gauß-Newton / Levenberg-Marquardt-Dämpfung).

    Sucht Gelenkwinkel q, sodass err_align klein wird. Pro Iteration wird die Jacobi-
    Matrix J per finiter Differenz (Störung 1e-6) gebildet und der gedämpfte
    Newton-Schritt  dq = -(J^T J + 0.01 I)^-1 J^T e  angewandt; das Ergebnis wird auf die
    Gelenkgrenzen geklippt. Rückgabe: geklippte Lösung q und der Restfehler (Norm).
    """
    q=np.array(seed,float)
    for _ in range(it):
        e=err_align(q,p,axis)
        if np.linalg.norm(e)<1e-4: break
        J=np.zeros((6,6))
        for k in range(6):
            qq=q.copy();qq[k]+=1e-6;J[:,k]=(err_align(qq,p,axis)-e)/1e-6
        q=np.clip(q+np.linalg.solve(J.T@J+0.01*np.eye(6),-J.T@e),_LL,_LU)
    return q,float(np.linalg.norm(err_align(q,p,axis)))
# SEEDS: mehrere Start-Gelenkkonfigurationen. Weil die IK lokal konvergiert, wird sie mit
# jedem Seed gestartet und die beste erreichbare Lösung gewählt (Multi-Start gegen lokale Minima).
SEEDS=[[0,0.4,-2.0,0.3,0,0],[-1.0,0,-1.5,1.0,0,0],[0,-0.3,-1.0,0.8,0.8,0],[-0.5,0.3,-1.8,1.2,-0.5,0.5],
       [0.8,0.3,-1.8,0.5,0,0],[-0.3,-0.5,-0.8,1.0,1.2,0],[0.3,0.5,-2.2,0.0,-1.0,0.5],
       [-0.8,0.8,-2.2,0.3,0.5,-0.5],[0,0.8,-1.5,-0.5,1.0,0],[-1.2,0.3,-1.2,1.2,0.8,0.5]]

def img_bgr(m):
    """ROS-Farbbild -> OpenCV-BGR (rgb8 wird kanalgetauscht, sonst Kopie). Siehe click_to_go."""
    a=np.frombuffer(m.data,np.uint8)
    if m.encoding.lower()=='rgb8': return cv2.cvtColor(a.reshape((m.height,m.width,3)),cv2.COLOR_RGB2BGR)
    return a.reshape((m.height,m.width,3)).copy()
def depth_arr(m):
    """ROS-Tiefenbild -> 2D-Array. 16UC1=mm (uint16), 32FC1=m (float32); sonst Fehler."""
    if m.encoding=='16UC1': return np.frombuffer(m.data,np.uint16).reshape((m.height,m.width))
    if m.encoding=='32FC1': return np.frombuffer(m.data,np.float32).reshape((m.height,m.width))
    raise ValueError(m.encoding)


class ClickPlace(Node):
    """Ein-Klick-PLACE OHNE MoveIt: IK-Ziel finden und direkt per pymycobot anfahren.

    Ablauf: Punkt anklicken -> pixel_to_base liefert 3D-Ziel im robot_base-Frame ->
    _solve_place sucht eine Pose, in der die Schraube SENKRECHT gehalten wird (gewählte
    Greifer-Achse zeigt nach unten) -> 'G' fährt mit send_radians dorthin -> Sichtkontrolle
    -> 'R' öffnet den Greifer und fährt gerade nach oben zurück zu Home.
    ACHTUNG: Diese Variante plant KEINEN kollisionsfreien Weg (daher gibt es die sichere
    MoveIt-Variante click_place_moveit.py). Die GUI läuft im Haupt-Thread, rclpy.spin im
    Daemon-Thread; _busy serialisiert die pymycobot-Bewegungen.
    """
    def __init__(self):
        super().__init__('click_place')
        # Kamera-Intrinsik + letzte Bild-/Tiefenpuffer (spin-Thread schreibt, GUI liest; Lock schützt).
        self._K=None;self._bgr=None;self._depth=None;self._lock=threading.Lock()
        self._px=None;self._tgt=None      # angeklicktes Pixel bzw. dazugehöriger 3D-Punkt (robot_base)
        self._screw_axis=2                # welche Achse die Schraube in gripper_base ist (2=Z Annahme)
        self._place_q=None                # gefundene Zielpose (6 Gelenkwinkel), None solange keine gültige IK
        self._status='Warte auf Kamera...'
        # TF-Buffer/Listener: statische Kamera->Basis-Transformation (eye-to-hand-Kalibrierung).
        self._tf=tf2_ros.Buffer(); tf2_ros.TransformListener(self._tf,self)
        self._mk=self.create_publisher(Marker,'/click_place/marker',5)  # Zielpunkt als RViz-Kugel
        # CameraInfo RELIABLE/depth=1: Intrinsik ist konstant, letzter Wert genügt.
        qos=QoSProfile(reliability=ReliabilityPolicy.RELIABLE,history=HistoryPolicy.KEEP_LAST,depth=1)
        self.create_subscription(CameraInfo,'/camera/color/camera_info',self._oninfo,qos)
        self.create_subscription(Image,'/camera/color/image_raw',self._oncolor,10)
        self.create_subscription(Image,'/camera/aligned_depth_to_color/image_raw',self._ondepth,10)
        # pymycobot öffnet HIER den seriellen Port (nur dieses Skript darf ihn belegen).
        # Bewegungen werden ausschließlich aus dem Haupt-Thread aufgerufen (serielle Kommunikation
        # ist nicht thread-sicher). time.sleep gibt der Firmware Zeit, power_on zu quittieren.
        from pymycobot import MyCobot280
        self.mc=MyCobot280(PORT,BAUD); time.sleep(0.8)
        try: self.mc.power_on()
        except Exception: pass
        self._busy=False   # True während einer laufenden Bewegung -> blockiert neue Kommandos

    def _oninfo(self,m): self._K=np.array(m.k,np.float64).reshape(3,3)  # 3x3-Intrinsik zwischenspeichern
    def _oncolor(self,m):
        try:
            with self._lock:
                self._bgr=img_bgr(m)
                if self._status=='Warte auf Kamera...': self._status='Bereit — Ablage-Punkt anklicken'
        except Exception: pass
    def _ondepth(self,m):
        try:
            with self._lock: self._depth=depth_arr(m)
        except Exception: pass

    def pixel_to_base(self,u,v):
        """Angeklicktes Pixel (u,v) -> 3D-Punkt im robot_base-Frame (identisch zu click_to_go).

        7x7-Median-Tiefe (robust gegen Rauschen/Löcher), mm->m, Lochkamera-Rückprojektion
        in den Kamera-optical-Frame, dann TF nach robot_base. None bei fehlender Intrinsik/
        Tiefe, Klick außerhalb des Bildes oder ungültigem Tiefenfenster.
        """
        if self._K is None: return None
        with self._lock: depth=self._depth
        if depth is None: return None
        h,w=depth.shape
        if not(0<=v<h and 0<=u<w): return None
        reg=depth[max(0,v-3):v+4,max(0,u-3):u+4]; val=reg[reg>0]  # nur gültige Tiefen (>0)
        if len(val)==0: return None
        z=float(np.median(val)); z=z/1000.0 if z>100 else z       # Median, mm->m falls nötig
        fx,fy=self._K[0,0],self._K[1,1];cx,cy=self._K[0,2],self._K[1,2]
        pc=PointStamped();pc.header.frame_id=CAMERA_FRAME;pc.header.stamp=self.get_clock().now().to_msg()
        pc.point.x=(u-cx)*z/fx;pc.point.y=(v-cy)*z/fy;pc.point.z=z  # inverse Lochkamera-Projektion
        try:
            pb=self._tf.transform(pc,ROBOT_BASE,timeout=rclpy.time.Duration(seconds=2.0))
        except Exception as e:
            self.get_logger().warn(f'TF: {e}'); return None
        return np.array([pb.point.x,pb.point.y,pb.point.z])

    def on_click(self,u,v):
        """Linksklick: Zielpunkt setzen, RViz-Marker publizieren, vorherige Lösung verwerfen.

        Während einer laufenden Bewegung (_busy) werden Klicks ignoriert. r = horizontaler
        Abstand von der Basisachse (Anzeige in mm).
        """
        if self._busy: return
        pb=self.pixel_to_base(u,v)
        if pb is None: self._status=f'Keine Tiefe ({u},{v})'; return
        self._px=(u,v); self._tgt=pb; self._place_q=None
        r=math.hypot(pb[0],pb[1])
        self._status=f'Ziel [{pb[0]:.3f},{pb[1]:.3f},{pb[2]:.3f}] r={r*1000:.0f}mm — G: los'
        self.get_logger().info(self._status)
        m=Marker();m.header.frame_id=ROBOT_BASE;m.header.stamp=self.get_clock().now().to_msg()
        m.ns='place';m.id=0;m.type=Marker.SPHERE;m.action=Marker.ADD
        m.pose.position.x=float(pb[0]);m.pose.position.y=float(pb[1]);m.pose.position.z=float(pb[2])
        m.pose.orientation.w=1.0;m.scale.x=m.scale.y=m.scale.z=0.02;m.color.b=1.0;m.color.a=1.0
        self._mk.publish(m)

    def _solve_place(self):
        """Pose über Ziel, Schraube SENKRECHT (Anflug waagerecht). Bester Azimut wird gesucht.

        Zielpunkt wird um HOVER_OFF angehoben (TCP schwebt über der Ablage). Für jeden Seed
        wird die Achsen-Ausrichtungs-IK gelöst und die Lösung nur akzeptiert, wenn:
          - Restfehler klein (res<1.5e-3) und alle Gelenke innerhalb der Grenzen,
          - die gewählte Greifer-Achse tatsächlich nach UNTEN zeigt (Rc[2,ax]<-0.95),
          - der niedrigste Punkt der Kette über dem Boden bleibt (mz>0.02 m).
        Unter den gültigen Lösungen wird die mit dem größten Bodenabstand (mz) gewählt
        (sicherste Pose). Kein Treffer -> Meldung 'UNERREICHBAR' (dann A-Achse wechseln).
        """
        if self._tgt is None: self._status='Erst einen Punkt waehlen!'; return False
        p=np.array([self._tgt[0],self._tgt[1],self._tgt[2]+HOVER_OFF])
        ax=self._screw_axis; best=None
        for s in SEEDS:
            q,res=ik_align(p,ax,s)
            if res<1.5e-3 and np.all(q>=_LL-1e-6)and np.all(q<=_LU+1e-6):
                _,Rc,zs=fk(q); mz=min(zs)
                if Rc[2,ax]<-0.95 and mz>0.02 and (best is None or mz>best[1]):  # Achse nach UNTEN + Bodenabstand
                    best=(q,mz)
        if best is None:
            self._place_q=None
            self._status=f'UNERREICHBAR (keine Schraube-senkrecht Pose, Achse={["X","Y","Z"][ax]}) — mit A Achse wechseln / anderer Punkt'
            return False
        self._place_q,mz=best
        self._status=f'Pose BEREIT (Schrauben-Achse={["X","Y","Z"][ax]}, Abstand {mz*1000:.0f}mm) — faehrt...'
        return True

    # ---- pymycobot Bewegung (Haupt-Thread) ----
    def _go_radians(self,q,speed=20,wait=12):
        """Blockierendes Anfahren einer Gelenkkonfiguration q (rad) mit Geschwindigkeit speed.

        Sendet send_radians und wartet, bis der Roboter still steht (is_moving()==0) oder
        das Zeitlimit (wait Sekunden) erreicht ist. Das Blockieren stellt sicher, dass die
        nächste Aktion erst nach abgeschlossener Bewegung startet.
        """
        self.mc.send_radians([float(x) for x in q],speed)
        for _ in range(wait):
            time.sleep(1.0)
            try:
                if self.mc.is_moving()==0: break
            except Exception: pass

    def go(self):
        """'G': Schraube-senkrecht-Pose lösen und (langsam) anfahren. _busy sperrt Reentranz."""
        if self._busy: return
        self._busy=True
        try:
            if not self._solve_place(): return
            self.get_logger().info('GO: faehrt zu Schraube-senkrecht Pose (langsam)')
            self._go_radians(self._place_q,18)
            self._status='Angekommen — Schraube SENKRECHT? [ ] mit Roll justieren, R ablegen'
        finally: self._busy=False

    def toggle_axis(self):
        """Wenn Schraube im Greifer auf anderer Achse (nicht senkrecht): Achse wechseln + erneut fahren.

        Wechselt zyklisch die als "Schraubenachse" angenommene gripper_base-Achse (Z->X->Y)
        und plant/fährt neu. Nötig, weil die Schraube je nach Ladeorientierung auf einer
        anderen lokalen Achse liegt und nur die richtige Achse "senkrecht nach unten" ergibt.
        """
        if self._busy or self._tgt is None: return
        self._screw_axis=(self._screw_axis+1)%3   # Z->X->Y Zyklus
        self._busy=True
        try:
            if self._solve_place(): self._go_radians(self._place_q,15)
            self._status=f'Schrauben-Achse={["X","Y","Z"][self._screw_axis]} — Schraube senkrecht? wenn nicht A, wenn ja R'
        finally: self._busy=False

    def grip(self,opn):
        """'O'/'C': Greifer öffnen (opn=True, Wert 100) bzw. schließen (Wert 0). Kraft 50, Modus 1."""
        if self._busy: return
        try: self.mc.set_gripper_value(100 if opn else 0,50,1)
        except Exception as e: self.get_logger().warn(str(e))
        self._status=('Greifer AUF' if opn else 'Greifer ZU')+' '

    def release(self):
        """'R': Ablegen. Greifer öffnen, gerade nach oben zurückziehen, dann Home anfahren.

        Nach dem Öffnen wird per FK die aktuelle TCP-Pose bestimmt und ein Punkt RELEASE_UP
        darüber angesteuert (gleiche Orientierung), damit der Greifer die abgelegte Schraube
        senkrecht verlässt und nicht seitlich streift. Danach zurück auf Null (Home).
        HINWEIS: ik6(...) ist hier nicht definiert -> dieser Zweig würde einen NameError
        werfen; die sichere/gepflegte Ablage-Logik steckt in click_place_moveit.py.
        """
        if self._busy or self._place_q is None: return
        self._busy=True
        try:
            self.get_logger().info('ABLEGEN: Greifer auf')
            self.mc.set_gripper_value(100,50,1); time.sleep(1.5)   # Greifer öffnen -> Schraube fällt/liegt ab
            tcp,Rc,_=fk(self._place_q)
            up=np.array([tcp[0],tcp[1],tcp[2]+RELEASE_UP])
            qup,res=ik6(up,Rc,self._place_q)        # gerade nach oben (gleiche Orientierung)
            if res<2e-3: self.get_logger().info('zurueck (nach oben)'); self._go_radians(qup,18)
            self.get_logger().info('home'); self._go_radians([0.0]*6,25)
            self.mc.set_gripper_value(100,50,1)
            self._status='ABGELEGT — Roboter in Null, Greifer auf'
        finally: self._busy=False

    def home(self):
        """'Z': alle Gelenke auf Null (Home) fahren."""
        if self._busy: return
        self._busy=True
        try: self._go_radians([0.0]*6,25)
        finally: self._busy=False
        self._status='HOME (Null)'

    def frame(self):
        """Anzeige-Bild bauen: Klick-Marker (grün=Pose bereit, gelb=nur gewählt) + Status/Legende."""
        with self._lock:
            if self._bgr is None: return None
            f=self._bgr.copy()
        h,w=f.shape[:2]
        if self._px is not None:
            u,v=self._px; col=(0,255,0) if self._place_q is not None else (0,255,255)
            cv2.drawMarker(f,(u,v),col,cv2.MARKER_CROSS,22,2); cv2.circle(f,(u,v),10,col,2)
        cv2.rectangle(f,(0,0),(w,28),(40,40,40),-1)
        cv2.putText(f,self._status,(8,20),cv2.FONT_HERSHEY_SIMPLEX,0.5,(255,255,255),1,cv2.LINE_AA)
        cv2.rectangle(f,(0,h-22),(w,h),(40,40,40),-1)
        cv2.putText(f,'Links:Punkt  O/C:Greifer  G:los  A:Schrauben-Achse  R:ablegen  Z:home  Q:beenden',
                    (4,h-6),cv2.FONT_HERSHEY_SIMPLEX,0.45,(180,180,180),1,cv2.LINE_AA)
        return f


def mouse(ev,x,y,fl,node):
    # Maus-Callback: Links = Ziel wählen, Rechts = Ziel/Lösung/Marker zurücksetzen.
    if ev==cv2.EVENT_LBUTTONDOWN: node.on_click(x,y)
    elif ev==cv2.EVENT_RBUTTONDOWN: node._px=None;node._tgt=None;node._place_q=None;node._status='geloescht'


def main():
    # ROS + Knoten starten, spin in Daemon-Thread, OpenCV-GUI im Haupt-Thread.
    rclpy.init(); node=ClickPlace()
    threading.Thread(target=lambda: rclpy.spin(node),daemon=True).start()
    cv2.namedWindow('Click Place',cv2.WINDOW_NORMAL); cv2.resizeWindow('Click Place',1280,720)
    cv2.setMouseCallback('Click Place',mouse,node)
    print('\n  CLICK PLACE — Punkt anklicken, O/C laden, G los, A Schrauben-Achse, R ablegen\n')
    # GUI-Hauptschleife (~20 Hz): Bild zeigen, Tasten auf Knoten-Aktionen abbilden.
    try:
        while rclpy.ok():
            fr=node.frame()
            if fr is not None: cv2.imshow('Click Place',fr)
            k=cv2.waitKey(50)&0xFF
            if k in (ord('o'),ord('O')): node.grip(True)
            elif k in (ord('c'),ord('C')): node.grip(False)
            elif k in (ord('g'),ord('G')): node.go()
            elif k in (ord('a'),ord('A')): node.toggle_axis()
            elif k in (ord('r'),ord('R')): node.release()
            elif k in (ord('z'),ord('Z')): node.home()
            elif k in (ord('q'),ord('Q')): break
    except KeyboardInterrupt: pass
    finally:
        cv2.destroyAllWindows(); node.destroy_node(); rclpy.shutdown()

if __name__=='__main__': main()
