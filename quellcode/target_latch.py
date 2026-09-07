#!/usr/bin/env python3
# =====================================================================
#  target_latch.py — /vida/target-Brücke für zweiphasigen (Vision→RViz) Pick.
#
#  Zwei Modi:
#    capture <Datei> : hört auf /vida/target (PoseStamped); zeigt das aktuellste
#                      Ziel. Wenn der Benutzer ENTER drückt, wird das letzte Ziel in
#                      <Datei> (JSON) GESPEICHERT und beendet. So bleibt die Koordinate
#                      auch dann erhalten, wenn der Detector geschlossen wird
#                      (Schraube statisch → Koordinate bleibt gültig).
#    replay  <Datei> : liest die Pose aus <Datei> und veröffentlicht sie mit 5 Hz
#                      ERNEUT auf /vida/target (pick_tilt empfängt das unverändert
#                      wartende Ziel) + erzeugt einen in RViz sichtbaren Marker
#                      (grüne Kugel + Pfeil der Grasp-Achse).
#
#  Verwendung:
#    python3 target_latch.py capture /tmp/vida_target.json
#    python3 target_latch.py replay  /tmp/vida_target.json
# =====================================================================
import sys, json, select, time
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import PoseStamped
from visualization_msgs.msg import Marker
from std_msgs.msg import String


def _fmt(p):
    return (f"BASE x={p.pose.position.x*1000:.0f} y={p.pose.position.y*1000:.0f} "
            f"z={p.pose.position.z*1000:.0f}mm  q=({p.pose.orientation.x:.3f},"
            f"{p.pose.orientation.y:.3f},{p.pose.orientation.z:.3f},{p.pose.orientation.w:.3f})"
            f"  frame={p.header.frame_id}")


def capture(path):
    rclpy.init()
    n = Node("target_capture")
    state = {"last": None, "n": 0, "info": None}

    def cb(m):
        state["last"] = m
        state["n"] += 1

    def info_cb(m):
        state["info"] = m.data   # "KIRMIZI Schraube conf=0.92 | x=.. y=.. z=..mm"

    n.create_subscription(PoseStamped, "/vida/target", cb, 1)
    n.create_subscription(String, "/vida/target_info", info_cb, 1)
    print("[target_latch] hoere auf /vida/target. Sobald die Erkennung im Overlay sitzt,")
    print("[target_latch] in diesem Terminal ENTER druecken → letztes Ziel wird gespeichert.")
    print("[target_latch] (wenn noch kein Ziel da ist, sagt ENTER 'kein Ziel', warten weiter.)")
    stdin_ok = True
    while rclpy.ok():
        rclpy.spin_once(n, timeout_sec=0.1)
        r, _, _ = (select.select([sys.stdin], [], [], 0.0) if stdin_ok else ([], [], []))
        if r:
            line = sys.stdin.readline()
            if line == "":   # EOF (Ctrl+D / nicht-interaktiv) — keinen Busy-Loop machen
                print("[target_latch] stdin geschlossen (EOF) — kein interaktives capture; warten.")
                stdin_ok = False
                continue
            if state["last"] is None:
                print("[target_latch] /vida/target noch nicht eingetroffen — Schraube schraeg legen/mischen, warten.")
                continue
            p = state["last"]
            data = {
                "frame_id": p.header.frame_id or "robot_base",
                "px": p.pose.position.x, "py": p.pose.position.y, "pz": p.pose.position.z,
                "ox": p.pose.orientation.x, "oy": p.pose.orientation.y,
                "oz": p.pose.orientation.z, "ow": p.pose.orientation.w,
                "info": state["info"] or "",
            }
            with open(path, "w") as f:
                json.dump(data, f, indent=2)
            print(f"[target_latch] GESPEICHERT → {path}")
            # ZU WELCHER Schraube gefahren wird (Farbe) — soll in PHASE B feststehen, wenn die Kamera aus ist.
            if state["info"]:
                print(f"[target_latch] >>> GEWAEHLTES ZIEL: {state['info']}")
            else:
                print("[target_latch] (keine Farbinfo eingetroffen — /vida/target_info fehlt)")
            print(f"[target_latch]   {_fmt(p)}  (insgesamt {state['n']} Ziele gesehen)")
            break
        if state["n"] and state["n"] % 10 == 1:
            pass
    n.destroy_node()
    rclpy.shutdown()


def replay(path):
    with open(path) as f:
        d = json.load(f)
    rclpy.init()
    n = Node("target_replay")
    pub = n.create_publisher(PoseStamped, "/vida/target", 1)
    mpub = n.create_publisher(Marker, "/vida/preview_marker", 1)
    frame = d.get("frame_id", "robot_base")
    print(f"[target_latch] replay: {path} → /vida/target (5Hz) + /vida/preview_marker  frame={frame}")
    if d.get("info"):
        print(f"[target_latch] >>> GEWAEHLTES ZIEL: {d['info']}")
    print(f"[target_latch]   x={d['px']*1000:.0f} y={d['py']*1000:.0f} z={d['pz']*1000:.0f}mm")

    def make_pose():
        m = PoseStamped()
        m.header.frame_id = frame
        m.header.stamp = n.get_clock().now().to_msg()
        m.pose.position.x = d["px"]; m.pose.position.y = d["py"]; m.pose.position.z = d["pz"]
        m.pose.orientation.x = d["ox"]; m.pose.orientation.y = d["oy"]
        m.pose.orientation.z = d["oz"]; m.pose.orientation.w = d["ow"]
        return m

    def make_marker(mid, mtype):
        mk = Marker()
        mk.header.frame_id = frame
        mk.header.stamp = n.get_clock().now().to_msg()
        mk.ns = "vida_preview"; mk.id = mid; mk.action = Marker.ADD
        mk.pose.position.x = d["px"]; mk.pose.position.y = d["py"]; mk.pose.position.z = d["pz"]
        mk.pose.orientation.x = d["ox"]; mk.pose.orientation.y = d["oy"]
        mk.pose.orientation.z = d["oz"]; mk.pose.orientation.w = d["ow"]
        if mtype == "sphere":
            mk.type = Marker.SPHERE
            mk.scale.x = mk.scale.y = mk.scale.z = 0.012
            mk.color.r = 0.1; mk.color.g = 1.0; mk.color.b = 0.1; mk.color.a = 0.95
        else:  # Pfeil der Grasp-Achse (tcp +Y = Annäherungsachse; pick_tilt-Auslegung)
            mk.type = Marker.ARROW
            mk.scale.x = 0.06; mk.scale.y = 0.006; mk.scale.z = 0.006
            mk.color.r = 1.0; mk.color.g = 0.55; mk.color.b = 0.0; mk.color.a = 0.9
        return mk

    try:
        while rclpy.ok():
            pub.publish(make_pose())
            mpub.publish(make_marker(0, "sphere"))
            mpub.publish(make_marker(1, "arrow"))
            rclpy.spin_once(n, timeout_sec=0.0)
            time.sleep(0.2)
    except KeyboardInterrupt:
        pass
    n.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    if len(sys.argv) < 3 or sys.argv[1] not in ("capture", "replay"):
        print("Verwendung: target_latch.py capture|replay <Datei>")
        sys.exit(2)
    (capture if sys.argv[1] == "capture" else replay)(sys.argv[2])
