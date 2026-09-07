#!/usr/bin/env python3
# ChArUco 6x3 Greifer-Marker-Generator — für die Neukalibrierung (eye-to-hand).
# Kleines Board zum Aufkleben auf den Greifer: Außenmaß 96x48 mm (passt in ~10x5 cm).
# Mit 100% Skalierung auf A4 drucken; NACH dem Druck ein Quadrat mit dem Lineal prüfen (muss 16.0 mm sein).
#
# Kompatibel mit charuco_detector.py: gleiches DICT_4X4_50 Wörterbuch.
# Aktive config/charuco_params.yaml wird NICHT überschrieben — neue Parameter werden in eine separate Datei geschrieben.
# Nach dem Drucken und Anbringen am Greifer den Detector mit dieser neuen Datei starten.

import sys
from pathlib import Path

import cv2.aruco as aruco
import numpy as np
from PIL import Image, ImageDraw, ImageFont

# --- Board-Parameter (nach dem ersten Druck NIEMALS ändern) ---
SQUARES_X = 6              # Anzahl Spalten (Breitenrichtung) -> 6 * 16 = 96 mm
SQUARES_Y = 3              # Anzahl Zeilen (Höhenrichtung) -> 3 * 16 = 48 mm
SQUARE_LENGTH_MM = 16.0    # Kantenlänge eines Quadrats
MARKER_LENGTH_MM = 12.0    # ArUco Marker-Kante (im Quadrat, 0.75x square)
DICT_ID = aruco.DICT_4X4_50
DICT_NAME = "DICT_4X4_50"

# --- A4 Druck ---
DPI = 300
A4_W_MM = 210.0
A4_H_MM = 297.0


def mm2px(mm: float) -> int:
    return int(round(mm * DPI / 25.4))


def main() -> int:
    pkg_root = Path(__file__).resolve().parent.parent
    boards_dir = pkg_root / "boards"
    cfg_dir = pkg_root / "config"
    boards_dir.mkdir(parents=True, exist_ok=True)
    cfg_dir.mkdir(parents=True, exist_ok=True)

    dictionary = aruco.getPredefinedDictionary(DICT_ID)
    board = aruco.CharucoBoard(
        (SQUARES_X, SQUARES_Y),
        SQUARE_LENGTH_MM / 1000.0,
        MARKER_LENGTH_MM / 1000.0,
        dictionary,
    )

    board_w_px = mm2px(SQUARES_X * SQUARE_LENGTH_MM)
    board_h_px = mm2px(SQUARES_Y * SQUARE_LENGTH_MM)
    board_img = board.generateImage((board_w_px, board_h_px), marginSize=0, borderBits=1)

    a4_w_px = mm2px(A4_W_MM)
    a4_h_px = mm2px(A4_H_MM)
    canvas = Image.new("RGB", (a4_w_px, a4_h_px), "white")

    off_x = (a4_w_px - board_w_px) // 2
    off_y = mm2px(25)
    canvas.paste(Image.fromarray(board_img).convert("RGB"), (off_x, off_y))

    draw = ImageDraw.Draw(canvas)
    try:
        font_sm = ImageFont.truetype(
            "/usr/share/fonts/truetype/dejavu/DejaVuSansMono.ttf", 24
        )
    except OSError:
        font_sm = ImageFont.load_default()

    text_y = off_y + board_h_px + mm2px(12)
    lines = [
        f"ChArUco {SQUARES_X}x{SQUARES_Y}  (gripper marker)",
        f"square = {SQUARE_LENGTH_MM:.1f} mm   marker = {MARKER_LENGTH_MM:.1f} mm",
        f"dict   = {DICT_NAME}",
        f"board  = {SQUARES_X*SQUARE_LENGTH_MM:.0f} x {SQUARES_Y*SQUARE_LENGTH_MM:.0f} mm",
        "Print at 100 % scale (NO fit-to-page).",
    ]
    for ln in lines:
        draw.text((mm2px(15), text_y), ln, fill="black", font=font_sm)
        text_y += mm2px(7)

    # 50 mm Referenzlineal — nach dem Druck mit einem Lineal prüfen
    ruler_y = text_y + mm2px(6)
    ruler_x0 = mm2px(15)
    ruler_x1 = ruler_x0 + mm2px(50)
    draw.line([(ruler_x0, ruler_y), (ruler_x1, ruler_y)], fill="black", width=4)
    for i in range(6):
        x = ruler_x0 + mm2px(10 * i)
        h = mm2px(5) if i % 5 == 0 else mm2px(3)
        draw.line([(x, ruler_y), (x, ruler_y + h)], fill="black", width=3)
    draw.text(
        (ruler_x0, ruler_y + mm2px(7)),
        "Reference 0-50 mm (verify after print)",
        fill="black",
        font=font_sm,
    )

    png_path = boards_dir / f"charuco_{SQUARES_X}x{SQUARES_Y}_gripper.png"
    pdf_path = boards_dir / f"charuco_{SQUARES_X}x{SQUARES_Y}_gripper.pdf"
    canvas.save(png_path, dpi=(DPI, DPI))
    canvas.save(pdf_path, "PDF", resolution=float(DPI))

    # Aktive charuco_params.yaml NICHT überschreiben — in separate Datei schreiben.
    yaml_path = cfg_dir / "charuco_params_gripper.yaml"
    yaml_path.write_text(
        "# ChArUco 6x3 gripper board parameters — passt zum gedruckten PDF.\n"
        "# Erzeugt von generate_charuco_gripper.py. Bei jeder Wertaenderung neu erzeugen.\n"
        "# Bei der Neukalibrierung den Detector mit DIESER Datei starten (statt dem alten 5x5).\n"
        "/**:\n"
        "  ros__parameters:\n"
        f"    squares_x: {SQUARES_X}\n"
        f"    squares_y: {SQUARES_Y}\n"
        f"    square_length_m: {SQUARE_LENGTH_MM / 1000.0}\n"
        f"    marker_length_m: {MARKER_LENGTH_MM / 1000.0}\n"
        f"    aruco_dict: {DICT_NAME}\n"
    )

    print(f"PNG : {png_path}")
    print(f"PDF : {pdf_path}")
    print(f"YAML: {yaml_path}")
    print()
    print("Mit 100% Skalierung drucken (fit-to-page AUS).")
    print(f"Nach dem Druck: ein Quadrat = {SQUARE_LENGTH_MM:.1f} mm muss sein (mit dem Lineal pruefen).")
    return 0


if __name__ == "__main__":
    sys.exit(main())
