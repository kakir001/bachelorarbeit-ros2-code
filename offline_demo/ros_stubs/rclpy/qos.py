"""Platzhalter fuer rclpy.qos (Offline-Demo)."""


class QoSProfile:
    def __init__(self, *args, **kwargs):
        self.__dict__.update(kwargs)


class _Wahl:
    BEST_EFFORT = RELIABLE = KEEP_LAST = KEEP_ALL = TRANSIENT_LOCAL = VOLATILE = SYSTEM_DEFAULT = 0


class ReliabilityPolicy(_Wahl):
    pass


class HistoryPolicy(_Wahl):
    pass


class DurabilityPolicy(_Wahl):
    pass
