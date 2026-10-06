"""Platzhalter fuer tf2_ros (Offline-Demo)."""
from _stub import nur_mit_ros

Buffer = nur_mit_ros("tf2_ros.Buffer")
TransformListener = nur_mit_ros("tf2_ros.TransformListener")


class TransformException(Exception):
    pass
