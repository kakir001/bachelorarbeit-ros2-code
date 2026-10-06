"""Gemeinsame Basis der ROS-Platzhalter (nur fuer die Offline-Demo).

Diese Platzhalter ersetzen KEIN ROS 2. Sie sorgen nur dafuer, dass sich die
Module in quellcode/tools ohne ROS-Installation IMPORTIEREN lassen, damit die
reine Bildverarbeitung (numpy/OpenCV) auf gespeicherten Kamerabildern laeuft.
Wird eine ROS-Funktion wirklich aufgerufen (Knoten starten, Topic abonnieren),
bricht der Platzhalter mit einer klaren Meldung ab.
"""


class NurMitRos(RuntimeError):
    pass


def nur_mit_ros(name):
    def _f(*args, **kwargs):
        raise NurMitRos("%s braucht eine echte ROS-2-Installation - in der Offline-Demo nicht verfuegbar." % name)
    return _f


class Nachricht:
    """Platzhalter fuer einen ROS-Nachrichtentyp: nimmt beliebige Felder an."""

    def __init__(self, *args, **kwargs):
        for k, v in kwargs.items():
            setattr(self, k, v)
