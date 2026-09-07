#!/usr/bin/env python3
"""Öffnet das D435i-Farbbild und markiert das exakte MITTE-Pixel mit Punkt + Kreuz.
Jetson Nano: 424x240@15fps zwingend (volle Auflösung gibt 'Out of frame resources').
Beenden: im Fenster ESC oder q. Mit 's' wird das Bild als ~/camera_center.png gespeichert.
"""
import os
import cv2
import numpy as np
import pyrealsense2 as rs

W, H, FPS = 424, 240, 15

pipe = rs.pipeline()
cfg = rs.config()
cfg.enable_stream(rs.stream.color, W, H, rs.format.bgr8, FPS)
profile = pipe.start(cfg)

# Echte Stream-Auflösung aus dem Profil holen (falls das Gerät anders liefert, bleibt kompatibel)
vsp = profile.get_stream(rs.stream.color).as_video_stream_profile()
w, h = vsp.width(), vsp.height()
cx, cy = w // 2, h // 2
print(f"[camera_center] Stream {w}x{h}  |  MITTE-Pixel = ({cx}, {cy})")

win = "D435i - MITTE-PUNKT (q/ESC beenden, s speichern)"
cv2.namedWindow(win, cv2.WINDOW_NORMAL)
cv2.resizeWindow(win, w * 2, h * 2)  # zum bequemen Sehen 2x vergrößern

try:
    while True:
        frames = pipe.wait_for_frames()
        color = frames.get_color_frame()
        if not color:
            continue
        img = np.asanyarray(color.get_data())

        # Kreuz (dünn, gelb) — über das ganze Bild
        cv2.line(img, (cx, 0), (cx, h), (0, 255, 255), 1)
        cv2.line(img, (0, cy), (w, cy), (0, 255, 255), 1)
        # Exakter Mittelpunkt — roter gefüllter Kreis + weißer Ring
        cv2.circle(img, (cx, cy), 5, (0, 0, 255), -1)
        cv2.circle(img, (cx, cy), 7, (255, 255, 255), 1)
        cv2.putText(img, f"({cx},{cy})", (cx + 10, cy - 10),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.5, (255, 255, 255), 1, cv2.LINE_AA)

        cv2.imshow(win, img)
        k = cv2.waitKey(1) & 0xFF
        if k in (27, ord('q')):
            break
        if k == ord('s'):
            out = os.path.expanduser("~/camera_center.png")
            cv2.imwrite(out, img)
            print(f"[camera_center] Gespeichert: {out}")
finally:
    pipe.stop()
    cv2.destroyAllWindows()
