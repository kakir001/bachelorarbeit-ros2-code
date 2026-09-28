#!/usr/bin/env python3
"""Veroeffentlicht die GEMESSENE Reichweitengrenze als Marker fuer RViz.

Warum als Marker und nicht im URDF: der Ring ist eine Messung, kein Bauteil. Er
gehoert nicht in das Kollisionsmodell (sonst wuerde MoveIt an ihm planen) und laesst
sich in RViz einzeln ein- und ausschalten.

Quelle ist config/arbeitsraum_ring.json — dieselbe Datei, aus der auch der Detektor
seine Overlay-Kreise zeichnet. Erzeugt wird sie von tools/arbeitsraum_grenze.py.

Gezeichnet werden drei Dinge:
  * aeussere Grenze (rot)   - dahinter ist NICHTS erreichbar
  * innere Grenze (rot)     - davor faltet sich der Arm in den eigenen Grundkasten
  * Totsektor (rot, gefuellt) - Bereich, den Gelenk 1 wegen seiner Drehgrenze
                                (+-168 Grad) nicht anfahren kann

Die Marker sind transient_local (gelatcht): RViz zeigt sie auch, wenn es spaeter
gestartet wird.
"""
import json
import math
import os

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, DurabilityPolicy
from geometry_msgs.msg import Point
from std_msgs.msg import ColorRGBA
from visualization_msgs.msg import Marker, MarkerArray


def _lade(pfad):
    with open(pfad, encoding="utf-8") as f:
        return json.load(f)


class ArbeitsraumMarker(Node):
    def __init__(self):
        super().__init__("arbeitsraum_marker")
        vor = os.path.join(
            os.path.dirname(os.path.dirname(os.path.abspath(__file__))),
            "config", "arbeitsraum_ring.json")
        try:
            from ament_index_python.packages import get_package_share_directory
            vor = os.path.join(get_package_share_directory("mycobot_world"),
                               "config", "arbeitsraum_ring.json")
        except Exception:
            pass
        self.declare_parameter("ring_datei", vor)
        # z_zeichnen: Hoehe, in der die Linien gezeichnet werden. Voreinstellung ist die
        # Hoehe, in der gemessen wurde — eine andere Hoehe waere schlicht falsch, denn
        # der Arbeitsraum ist ein Keil und der Ring gilt nur fuer seine Messhoehe.
        self.declare_parameter("z_zeichnen", -1.0)
        self.declare_parameter("rahmen", "robot_base")

        datei = self.get_parameter("ring_datei").value
        self.daten = _lade(datei)
        z = self.get_parameter("z_zeichnen").value
        self.z = float(self.daten["z"]) if z < 0 else float(z)
        self.rahmen = self.get_parameter("rahmen").value
        self.get_logger().info(
            f"Ring aus {datei} | z={self.z*1000:.0f} mm | {len(self.daten['ring'])} Azimute")

        qos = QoSProfile(depth=1)
        qos.durability = DurabilityPolicy.TRANSIENT_LOCAL
        self.pub = self.create_publisher(MarkerArray, "/arbeitsraum/grenzen", qos)
        self.create_timer(2.0, self._senden)
        self._senden()

    # ---- Hilfen ----
    def _punkte(self, schluessel, nur_gueltig=True):
        pts = []
        for e in self.daten["ring"]:
            r = e.get(schluessel)
            if r is None:
                if nur_gueltig:
                    continue
                r = 0.0
            th = math.radians(e["theta"])
            pts.append(Point(x=r / 1000.0 * math.cos(th),
                             y=r / 1000.0 * math.sin(th), z=self.z))
        if pts:
            pts.append(pts[0])          # Ring schliessen
        return pts

    def _linie(self, mid, ns, pts, farbe, breite=0.004):
        m = Marker()
        m.header.frame_id = self.rahmen
        m.ns = ns
        m.id = mid
        m.type = Marker.LINE_STRIP
        m.action = Marker.ADD
        m.scale.x = breite
        m.color = farbe
        m.pose.orientation.w = 1.0
        m.points = pts
        return m

    def _senden(self):
        rot = ColorRGBA(r=0.90, g=0.15, b=0.10, a=1.0)
        arr = MarkerArray()
        arr.markers.append(self._linie(0, "arbeitsraum", self._punkte("r_max"), rot))
        arr.markers.append(self._linie(1, "arbeitsraum", self._punkte("r_min"), rot))

        # Totsektor als gefuellten Keil zeichnen. Er steht als eigenes Feld in der
        # Datei und nicht als Luecke im Ring: bei 15 Grad Abtastung faellt er sonst
        # durch das Raster. Seine Kanten wurden einzeln nachgemessen (153 / 173 Grad).
        ts = self.daten.get("totsektor")
        if ts:
            m = Marker()
            m.header.frame_id = self.rahmen
            m.ns = "arbeitsraum"
            m.id = 2
            m.type = Marker.TRIANGLE_LIST
            m.action = Marker.ADD
            m.scale.x = m.scale.y = m.scale.z = 1.0
            m.color = ColorRGBA(r=0.90, g=0.15, b=0.10, a=0.25)
            m.pose.orientation.w = 1.0
            rmax = max((e["r_max"] for e in self.daten["ring"]
                        if e.get("r_max") is not None), default=270.0) / 1000.0
            von, bis = float(ts["von"]), float(ts["bis"])
            n_seg = max(2, int(round((bis - von) / 2.0)))
            for i in range(n_seg):
                a = math.radians(von + (bis - von) * i / n_seg)
                b = math.radians(von + (bis - von) * (i + 1) / n_seg)
                m.points += [Point(x=0.0, y=0.0, z=self.z),
                             Point(x=rmax * math.cos(a), y=rmax * math.sin(a), z=self.z),
                             Point(x=rmax * math.cos(b), y=rmax * math.sin(b), z=self.z)]
            arr.markers.append(m)
            self.get_logger().info(
                f"Totsektor {von:.0f}-{bis:.0f} Grad gezeichnet ({n_seg} Dreiecke)",
                once=True)
        self.pub.publish(arr)


def main():
    rclpy.init()
    n = ArbeitsraumMarker()
    try:
        rclpy.spin(n)
    except KeyboardInterrupt:
        pass
    n.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
