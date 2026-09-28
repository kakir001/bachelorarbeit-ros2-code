#!/usr/bin/env python3
# =====================================================================
#  teach_boxes.py — die 5 Box-Koordinaten für das SORTIEREN nach Farbe anlernen (teach).
#
#  Zweck: nachdem pick_tilt die Welle gegriffen hat und nach Home zurückgekehrt
#  ist, fährt es UEBER die zur FARBE der Welle gehörende Box und lässt sie aus
#  der Luft fallen (SCHRITT 8). Dafür wird das (x,y,z)-Zentrum der Box jeder Farbe
#  in robot_base benötigt. Dieses Skript lernt sie an.
#
#  METHODE: den Roboter mit RViz MoveIt MotionPlanning (interaktiver Marker → Plan &
#  Execute, IK in MoveIt) über die OEFFNUNG der jeweiligen Box bringen, dann speichern →
#  die aktuelle robot_base→tcp-Pose wird unter dieser Farbe abgelegt. Dieses Skript
#  STEUERT den Roboter NICHT.
#
#  ZWEI MODI:
#    (interaktiv)  python3 teach_boxes.py
#        Fragt für jede Farbe der Reihe nach per ENTER (vom Terminal).
#    (einmalig) python3 teach_boxes.py --once rot
#        Speichert NUR die aktuelle TCP-Pose von 'rot', behält die anderen.
#        (Damit Claude es aufruft, wenn du "speichern" sagst — du musst nicht ins Terminal tippen.)
#
#  VORBEDINGUNG: der Stack (move_group + controllers + TF) muss LAUFEN, damit
#  der robot_base→tcp-TF veröffentlicht wird. (teach_setup.sh richtet das ein.)
#
#  SPEICHERUNG: die kanonischen Daten liegen in ~/ros2_ws/.box_teach.json; bei jeder
#  Speicherung wird sowohl diese aktualisiert als auch die von pick_tilt geladene
#  box_map.yaml neu erzeugt.
#
#  Verwendung:
#    python3 teach_boxes.py                 # interaktiv, 5 Farben
#    python3 teach_boxes.py --once gruen    # nur gruen speichern
#    python3 teach_boxes.py --list          # vorhandene Einträge zeigen
# =====================================================================
import argparse
import json
import os

import rclpy
from rclpy.node import Node
import tf2_ros

# EXAKT dieselbe Reihenfolge wie detection.py (model.names): class_id 0..4
CLASS_NAMES = ["gelb", "weiss", "schwarz", "gruen", "rot"]
ROBOT_BASE = "robot_base"
TCP = "tcp"

WS = os.path.expanduser("~/ros2_ws")
JSON_PATH = os.path.join(WS, ".box_teach.json")
YAML_PATH = os.path.join(WS, "box_map.yaml")


class TeachNode(Node):
    def __init__(self):
        super().__init__("teach_boxes")
        self.tf_buffer = tf2_ros.Buffer()
        tf2_ros.TransformListener(self.tf_buffer, self)

    def tcp_pose(self, settle_ticks=12):
        """robot_base→tcp momentan (x,y,z). Kurzes Spin, um den TF-Buffer zu füllen."""
        for _ in range(settle_ticks):
            rclpy.spin_once(self, timeout_sec=0.1)
        try:
            tf = self.tf_buffer.lookup_transform(ROBOT_BASE, TCP, rclpy.time.Time())
            t = tf.transform.translation
            return [float(t.x), float(t.y), float(t.z)]
        except Exception as e:
            self.get_logger().warn(f"TF robot_base→tcp nicht lesbar: {e}")
            return None


def load_boxes():
    if os.path.exists(JSON_PATH):
        try:
            with open(JSON_PATH) as f:
                return json.load(f)
        except Exception:
            pass
    return {}


def save_all(boxes):
    """Kanonisches JSON + das von pick_tilt gelesene ros-params-YAML zusammen schreiben."""
    with open(JSON_PATH, "w") as f:
        json.dump(boxes, f, indent=2)
    lines = [
        "# mit teach_boxes.py angelernt — Box-Zentren fuer das Sortieren nach Farbe (robot_base, m).",
        "# pick_tilt.launch.py laedt sie per BOX_MAP env (Standard ~/ros2_ws/box_map.yaml).",
        "/**:",
        "  ros__parameters:",
    ]
    for name in CLASS_NAMES:
        xyz = boxes.get(name)
        if xyz is None:
            lines.append(f"    # box_{name}: UEBERSPRUNGEN (nicht angelernt)")
        else:
            lines.append(f"    box_{name}: [{xyz[0]:.4f}, {xyz[1]:.4f}, {xyz[2]:.4f}]")
    with open(YAML_PATH, "w") as f:
        f.write("\n".join(lines) + "\n")


def print_list(boxes):
    print("=== Vorhandene Box-Eintraege ===")
    for name in CLASS_NAMES:
        xyz = boxes.get(name)
        if xyz is None:
            print(f"  {name:8s}: —  (nicht angelernt)")
        else:
            print(f"  {name:8s}: x={xyz[0]*1000:.0f} y={xyz[1]*1000:.0f} z={xyz[2]*1000:.0f} mm")


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--once", metavar="FARBE", help=f"nur diese Farbe speichern ({'/'.join(CLASS_NAMES)})")
    ap.add_argument("--list", action="store_true", help="vorhandene Eintraege zeigen und beenden")
    args = ap.parse_args()

    boxes = load_boxes()

    if args.list:
        print_list(boxes)
        return

    rclpy.init()
    node = TeachNode()

    if args.once:
        color = args.once.strip().lower()
        if color not in CLASS_NAMES:
            print(f"⚠ Ungueltige Farbe '{color}'. Gueltig: {', '.join(CLASS_NAMES)}")
        else:
            pose = node.tcp_pose()
            if pose is None:
                print("⚠ Kein TF — laeuft der Stack? Speichern fehlgeschlagen.")
            else:
                boxes[color] = pose
                save_all(boxes)
                print(f"✓ {color}: x={pose[0]*1000:.0f} y={pose[1]*1000:.0f} "
                      f"z={pose[2]*1000:.0f} mm → gespeichert ({YAML_PATH})")
                print_list(boxes)
        node.destroy_node(); rclpy.shutdown()
        return

    # --- interaktiver Modus ---
    print("\n=== BOX ANLERNEN (teach) — Sortieren nach Farbe ===")
    print("Mit RViz MotionPlanning den Roboter ueber die Box-Oeffnung bringen, dann ENTER.")
    print("('s'=ueberspringen, 'q'=speichern und beenden)\n")
    try:
        for name in CLASS_NAMES:
            while True:
                ans = input(f"Ueber Box [{name.upper()}]? ENTER=speichern / s=ueberspringen / q=beenden: ").strip().lower()
                if ans == "q":
                    raise KeyboardInterrupt
                if ans == "s":
                    print(f"  {name} UEBERSPRUNGEN."); break
                pose = node.tcp_pose()
                if pose is None:
                    print("  ⚠ Kein TF — laeuft der Stack? Erneut versuchen."); continue
                boxes[name] = pose
                save_all(boxes)
                print(f"  ✓ {name}: x={pose[0]*1000:.0f} y={pose[1]*1000:.0f} z={pose[2]*1000:.0f} mm")
                break
    except KeyboardInterrupt:
        print("\n(wird beendet)")

    print_list(boxes)
    print(f"\n→ {YAML_PATH}")
    node.destroy_node(); rclpy.shutdown()


if __name__ == "__main__":
    main()
