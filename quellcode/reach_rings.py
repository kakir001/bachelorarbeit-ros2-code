#!/usr/bin/env python3
"""
reach_rings.py — Reichweite-Grenzen des Roboters in RViz als Ring-Marker zeigen.
RViz-Äquivalent des Donuts (grün/amber/rot) im Detector-Overlay: horizontale
Kreise in der robot_base-Achse. Horizontal r = zeigt die hypot(x,y)-Bänder.

  red  r=0.15  : innere Grenze — darunter klappt der Arm ein/stößt an die Basis
  green r=0.18 : ideales Pick-Zentrum
  green r=0.20 : außerhalb des grünen Bandes (Reichweite-Kante)
  amber r=0.26 : absolutes Max (im freien Raum; bei Plattform top-down weniger)

Verwendung:  source /opt/ros/galactic/setup.bash && source install/setup.bash
             python3 reach_rings.py
In RViz:     Add → MarkerArray → Topic: /reach_rings
"""
import math
import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, DurabilityPolicy
from visualization_msgs.msg import Marker, MarkerArray
from geometry_msgs.msg import Point

FRAME = "robot_base"
Z = 0.002  # direkt über der Tischebene

# (id, Radius m, (r,g,b), Beschriftung)
RINGS = [
    (0, 0.15, (0.90, 0.10, 0.10), "innen 15cm"),
    (1, 0.18, (0.10, 0.90, 0.10), "ideal 18cm"),
    (2, 0.20, (0.10, 0.80, 0.10), "gruen 20cm"),
    (3, 0.26, (1.00, 0.65, 0.00), "max 26cm"),
]


def circle(mid, r, rgb, width=0.004, npts=96):
    m = Marker()
    m.header.frame_id = FRAME
    m.ns = "reach_ring"
    m.id = mid
    m.type = Marker.LINE_STRIP
    m.action = Marker.ADD
    m.scale.x = width
    m.color.r, m.color.g, m.color.b = rgb
    m.color.a = 0.9
    m.pose.orientation.w = 1.0
    for i in range(npts + 1):
        a = 2.0 * math.pi * i / npts
        p = Point()
        p.x = r * math.cos(a)
        p.y = r * math.sin(a)
        p.z = Z
        m.points.append(p)
    return m


def label(mid, r, rgb, text):
    m = Marker()
    m.header.frame_id = FRAME
    m.ns = "reach_label"
    m.id = mid
    m.type = Marker.TEXT_VIEW_FACING
    m.action = Marker.ADD
    m.scale.z = 0.018
    m.color.r, m.color.g, m.color.b = rgb
    m.color.a = 0.95
    m.pose.position.x = r * 0.707
    m.pose.position.y = -r * 0.707  # in den vorderen-rechten Quadranten schreiben
    m.pose.position.z = Z + 0.005
    m.pose.orientation.w = 1.0
    m.text = text
    return m


def main():
    rclpy.init()
    node = Node("reach_rings")
    qos = QoSProfile(depth=1)
    qos.durability = DurabilityPolicy.TRANSIENT_LOCAL  # latched → spät beitretendes RViz bekommt es
    pub = node.create_publisher(MarkerArray, "/reach_rings", qos)

    arr = MarkerArray()
    for mid, r, rgb, txt in RINGS:
        arr.markers.append(circle(mid, r, rgb))
        arr.markers.append(label(100 + mid, r, rgb, txt))

    def tick():
        for mk in arr.markers:
            mk.header.stamp = node.get_clock().now().to_msg()
        pub.publish(arr)

    tick()
    node.create_timer(2.0, tick)  # alle 2s auffrischen (für neue Abonnenten)
    node.get_logger().info(f"reach_rings aktiv → /reach_rings ({len(RINGS)} Ringe, frame={FRAME})")
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
