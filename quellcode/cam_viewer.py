#!/usr/bin/env python3
# cam_viewer.py — D435i Kamera (Detector Overlay) in einem einzigen cv2-Fenster.
#
# Warum nicht RViz: Das RViz Image-Display löst auf diesem Jetson Tegra einen
# OGRE/X11 Render-Fenster-Resize-Bug aus und bringt RViz zum Segfault (DEVLOG
# Sitzung 7 + Sitzung 21). Deshalb werden die Kameras AUSSERHALB von RViz in einem
# leichten OpenCV-Fenster gezeigt (gleich wie view_overlay.py, auf dieser Maschine
# bewährte Methode). Ohne cv_bridge.
#
# Verwendung: python3 ~/ros2_ws/cam_viewer.py   (q / ESC zum Schließen)
#   env: CAM1_TOPIC, CAM2_TOPIC, PANEL_H
import os
import numpy as np
import cv2
import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy
from sensor_msgs.msg import Image
from std_msgs.msg import Empty

# CAM1 = Detector Overlay (YOLO-Erkennung + AUSGEWAEHLTE Welle als "ZIEL" markiert).
#   Wenn der Detector stirbt, friert das letzte Bild ein (der ausgewählte Punkt
#   bleibt sichtbar). Ein best-effort Sub empfängt auch von einem reliable Pub
#   → am kompatibelsten.
CAM1 = os.environ.get("CAM1_TOPIC", "/welle/overlay")             # D435i + Erkennungs-Overlay
# Rohbild-Fallback: solange der Detector noch laedt/warmuppt (auf dem Nano 1-2 min),
# wird das ROHE Kamerabild gezeigt statt eines schwarzen "warten..."-Fensters.
RAW = os.environ.get("RAW_TOPIC", "/camera/color/image_raw")
# 480 = native Hoehe des 848x480-Streams → 1:1, kein unscharfes Hochskalieren.
PANEL_H = int(os.environ.get("PANEL_H", "480"))


def img_to_bgr(msg):
    h, w = msg.height, msg.width
    enc = msg.encoding
    buf = np.frombuffer(msg.data, dtype=np.uint8)
    try:
        if enc == "rgb8":
            return cv2.cvtColor(buf.reshape(h, w, 3), cv2.COLOR_RGB2BGR)
        if enc == "bgr8":
            return buf.reshape(h, w, 3)
        if enc in ("mono8", "8UC1"):
            return cv2.cvtColor(buf.reshape(h, w), cv2.COLOR_GRAY2BGR)
        return buf.reshape(h, w, -1)[:, :, :3]
    except Exception:
        return None


class CamViewer(Node):
    def __init__(self):
        super().__init__("cam_viewer")
        self.f1 = None
        self.raw = None            # Rohbild-Fallback bis das erste Overlay eintrifft
        self._overlay_seen = False
        be = QoSProfile(depth=1, reliability=ReliabilityPolicy.BEST_EFFORT,
                        history=HistoryPolicy.KEEP_LAST)
        self.create_subscription(Image, CAM1, self.cb1, be)
        self.create_subscription(Image, RAW, self.cb_raw, be)
        # pick_tilt HOVER-Bestätigung: Taste 'g' -> /pick/confirm (std_msgs/Empty).
        self.pub_confirm = self.create_publisher(Empty, "/pick/confirm", 1)
        self._confirm_flash = 0    # kurzer "BESTAETIGUNG GESENDET" Zähler am Bildschirm
        self.win = "Kamera D435i + Erkennung   -   g=BESTAETIGEN  q/ESC=schliessen"
        cv2.namedWindow(self.win, cv2.WINDOW_NORMAL)
        cv2.resizeWindow(self.win, int(PANEL_H * 848 / 480), PANEL_H + 20)
        self.timer = self.create_timer(1.0 / 15.0, self.tick)
        self.get_logger().info(f"cam_viewer: {CAM1} (Fallback: {RAW})")

    def cb1(self, m):
        self.f1 = img_to_bgr(m)
        self._overlay_seen = True

    def cb_raw(self, m):
        # Nur solange kein Overlay kam dekodieren — danach unnoetige CPU-Last sparen.
        if not self._overlay_seen:
            self.raw = img_to_bgr(m)

    def _panel(self, img, label, show_label=True):
        h = PANEL_H
        if img is None:
            p = np.zeros((h, int(h * 1.4), 3), np.uint8)
            cv2.putText(p, label + ": warten...", (10, h // 2),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0, 0, 255), 2)
            return p
        sc = h / float(img.shape[0])
        # INTER_AREA beim Verkleinern (scharf), INTER_CUBIC beim Vergroessern (weniger
        # Treppen als das Standard-INTER_LINEAR) — Bildqualitaets-Wunsch 2026-07-16.
        interp = cv2.INTER_AREA if sc < 1.0 else cv2.INTER_CUBIC
        if abs(sc - 1.0) < 1e-3:
            r = img.copy()
        else:
            r = cv2.resize(img, (max(1, int(img.shape[1] * sc)), h), interpolation=interp)
        # show_label=False: im D435i-Overlay-Panel kein Label zeichnen — das Detector-
        # Overlay schreibt bereits seinen eigenen "N Wellen" + ZIEL Text, damit es
        # sich nicht überlagert.
        if show_label:
            cv2.putText(r, label, (8, 26), cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0, 255, 0), 2)
        return r

    def tick(self):
        frame = self.f1
        if frame is None and self.raw is not None:
            # Detector laedt noch — Rohbild zeigen, mit Hinweis.
            frame = self.raw.copy()
            cv2.putText(frame, "Detector laedt (CUDA-Warmup ~1-2 min)...", (8, 24),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.55, (0, 0, 0), 3, cv2.LINE_AA)
            cv2.putText(frame, "Detector laedt (CUDA-Warmup ~1-2 min)...", (8, 24),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.55, (0, 255, 255), 1, cv2.LINE_AA)
        combo = self._panel(frame, "Kamera D435i + Erkennung (ZIEL)", show_label=False)
        # Bestätigungs-Hinweis + Flash (duenn, mit Kontur — Wunsch 2026-07-16: Texte nicht fett)
        hint = "g = BESTAETIGEN  (1: Hover->ab   2: gegriffen->heben)"
        cv2.putText(combo, hint, (8, combo.shape[0] - 12),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 0, 0), 3, cv2.LINE_AA)
        cv2.putText(combo, hint, (8, combo.shape[0] - 12),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 255, 255), 1, cv2.LINE_AA)
        if self._confirm_flash > 0:
            cv2.putText(combo, "BESTAETIGUNG GESENDET", (combo.shape[1] // 2 - 120, 40),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.8, (0, 0, 0), 3, cv2.LINE_AA)
            cv2.putText(combo, "BESTAETIGUNG GESENDET", (combo.shape[1] // 2 - 120, 40),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.8, (0, 255, 0), 1, cv2.LINE_AA)
            self._confirm_flash -= 1
        cv2.imshow(self.win, combo)
        k = cv2.waitKey(1) & 0xFF
        if k in (27, ord('q')):
            rclpy.shutdown()
        elif k == ord('g'):
            self.pub_confirm.publish(Empty())
            self._confirm_flash = 15    # ~1 s (15 Hz)
            self.get_logger().info("/pick/confirm gesendet (Bestaetigung: Hover->ab ODER gegriffen->heben)")


def main():
    rclpy.init()
    node = CamViewer()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        try:
            cv2.destroyAllWindows()
        except Exception:
            pass
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
