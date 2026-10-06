"""Platzhalter fuer rclpy.node (Offline-Demo)."""
from _stub import NurMitRos


class Node:
    """Nur als Basisklasse importierbar (z. B. class Finder(Node)); nicht instanziierbar."""

    def __init__(self, *args, **kwargs):
        raise NurMitRos("Ein ROS-2-Knoten laesst sich in der Offline-Demo nicht starten.")
