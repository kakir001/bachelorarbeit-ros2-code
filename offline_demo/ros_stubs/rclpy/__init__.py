"""Platzhalter fuer rclpy (Offline-Demo, siehe ../_stub.py)."""
from _stub import nur_mit_ros

OFFLINE_PLATZHALTER = True

init = nur_mit_ros("rclpy.init")
shutdown = nur_mit_ros("rclpy.shutdown")
spin = nur_mit_ros("rclpy.spin")
spin_once = nur_mit_ros("rclpy.spin_once")
spin_until_future_complete = nur_mit_ros("rclpy.spin_until_future_complete")


def ok():
    return False
