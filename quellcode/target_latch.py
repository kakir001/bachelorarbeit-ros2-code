#!/usr/bin/env python3
# =====================================================================
#  target_latch.py — /welle/ziel-Brücke für zweiphasigen (Vision→RViz) Pick.
#
#  Zwei Modi:
#    capture <Datei> : hört auf /welle/ziel (PoseStamped); zeigt das aktuellste
#                      Ziel. Wenn der Benutzer ENTER drückt, wird das letzte Ziel in
#                      <Datei> (JSON) GESPEICHERT und beendet. So bleibt die Koordinate
#                      auch dann erhalten, wenn der Detector geschlossen wird
#                      (Welle statisch → Koordinate bleibt gültig).
#    replay  <Datei> : liest die Pose aus <Datei> und veröffentlicht sie mit 5 Hz
#                      ERNEUT auf /welle/ziel (pick_tilt empfängt das unverändert
#                      wartende Ziel) + erzeugt einen in RViz sichtbaren Marker
#                      (grüne Kugel + Pfeil der Grasp-Achse).
#
#  Verwendung:
#    python3 target_latch.py capture /tmp/welle_ziel.json
#    python3 target_latch.py replay  /tmp/welle_ziel.json
# =====================================================================
import sys, json, os, select, time
from datetime import datetime
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import PoseStamped
from sensor_msgs.msg import Image
from visualization_msgs.msg import Marker
from std_msgs.msg import String


def _overlay_to_bgr(m):
    """sensor_msgs/Image (bgr8|rgb8) -> numpy BGR. None, wenn nicht darstellbar."""
    import numpy as np
    if m.encoding not in ("bgr8", "rgb8"):
        return None
    a = np.frombuffer(m.data, dtype=np.uint8)
    if a.size < m.height * m.step:
        return None
    a = a[:m.height * m.step].reshape(m.height, m.step)[:, : m.width * 3]
    a = a.reshape(m.height, m.width, 3)
    return a if m.encoding == "bgr8" else a[:, :, ::-1]


def _screenshot(msg, info, target_path):
    """Overlay-Bild des BESTAETIGTEN Ziels sichern.

    Der Grund: nach dem ENTER wird die Kamera beendet und RViz uebernimmt; in der
    Anzeige steht dann nur noch die Farbe ("weiss Welle"). Liegen mehrere gleichfarbige
    Wellen in der Schale, ist damit nicht mehr zu erkennen, WELCHE gewaehlt wurde.
    Das Bild haelt genau diesen Moment fest, sodass sich hinterher vergleichen laesst,
    ob der Arm zur richtigen Welle gefahren ist.

    Ein Fehlschlag ist NICHT toedlich - das Ziel ist da wichtiger als das Bild.
    """
    if msg is None:
        print("[target_latch] (kein Overlay-Bild eingetroffen - kein Screenshot)")
        return None
    try:
        import cv2
        bgr = _overlay_to_bgr(msg)
        if bgr is None:
            print(f"[target_latch] (Overlay-Encoding '{msg.encoding}' nicht unterstuetzt)")
            return None
        d = os.environ.get("ZIEL_SCREENSHOT_DIR") or os.path.join(
            os.path.expanduser("~/ros2_ws"), "logs")
        os.makedirs(d, exist_ok=True)
        stamp = datetime.now().strftime("%Y%m%d_%H%M%S")
        out = os.path.join(d, f"ziel_{stamp}.png")
        cv2.imwrite(out, bgr)
        # zusaetzlich eine feste "letzte Wahl" neben der Ziel-Datei
        latest = os.path.splitext(target_path)[0] + ".png"
        cv2.imwrite(latest, bgr)
        print(f"[target_latch] SCREENSHOT  -> {out}")
        print(f"[target_latch]   (letzte Wahl auch: {latest})")
        return out
    except Exception as e:
        print(f"[target_latch] (Screenshot fehlgeschlagen: {type(e).__name__}: {e})")
        return None


def _fmt(p):
    return (f"BASE x={p.pose.position.x*1000:.0f} y={p.pose.position.y*1000:.0f} "
            f"z={p.pose.position.z*1000:.0f}mm  q=({p.pose.orientation.x:.3f},"
            f"{p.pose.orientation.y:.3f},{p.pose.orientation.z:.3f},{p.pose.orientation.w:.3f})"
            f"  frame={p.header.frame_id}")


def capture(path):
    rclpy.init()
    n = Node("target_capture")
    state = {"last": None, "n": 0, "info": None, "overlay": None}

    def cb(m):
        state["last"] = m
        state["n"] += 1

    def info_cb(m):
        state["info"] = m.data   # "ROT Welle conf=0.92 | x=.. y=.. z=..mm"

    n.create_subscription(PoseStamped, "/welle/ziel", cb, 1)
    n.create_subscription(String, "/welle/ziel_info", info_cb, 1)
    # Overlay mitlesen, damit beim ENTER ein Bild der bestaetigten Welle gesichert wird.
    n.create_subscription(Image, "/welle/overlay", lambda m: state.__setitem__("overlay", m), 1)
    print("[target_latch] hoere auf /welle/ziel. Sobald die Erkennung im Overlay sitzt,")
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
                print("[target_latch] /welle/ziel noch nicht eingetroffen — Welle schraeg legen/mischen, warten.")
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
            _screenshot(state["overlay"], state["info"], path)
            # ZU WELCHER Welle gefahren wird (Farbe) — soll in PHASE B feststehen, wenn die Kamera aus ist.
            if state["info"]:
                print(f"[target_latch] >>> GEWAEHLTES ZIEL: {state['info']}")
            else:
                print("[target_latch] (keine Farbinfo eingetroffen — /welle/ziel_info fehlt)")
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
    pub = n.create_publisher(PoseStamped, "/welle/ziel", 1)
    mpub = n.create_publisher(Marker, "/welle/vorschau_marker", 1)
    frame = d.get("frame_id", "robot_base")
    print(f"[target_latch] replay: {path} → /welle/ziel (5Hz) + /welle/vorschau_marker  frame={frame}")
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
        mk.ns = "welle_vorschau"; mk.id = mid; mk.action = Marker.ADD
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
