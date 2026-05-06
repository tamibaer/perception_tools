#!/usr/bin/env python3

import cv2
import numpy as np
from reportlab.pdfgen import canvas
from reportlab.lib.pagesizes import A4

def main():
  
    markers_x = 4
    markers_y = 6

    marker_length = 0.04       # [m] = 40 mm
    marker_separation = 0.01   # [m] = 10 mm

    aruco_dict = cv2.aruco.getPredefinedDictionary(cv2.aruco.DICT_4X4_50)

    board = cv2.aruco.GridBoard_create(
        markersX=markers_x,
        markersY=markers_y,
        markerLength=marker_length,
        markerSeparation=marker_separation,
        dictionary=aruco_dict
    )

    board_width_m = markers_x * marker_length + (markers_x - 1) * marker_separation
    board_height_m = markers_y * marker_length + (markers_y - 1) * marker_separation

    board_width_mm = board_width_m * 1000
    board_height_mm = board_height_m * 1000

    print(f"Boardgröße: {board_width_mm:.1f} mm x {board_height_mm:.1f} mm")

    pixels_per_meter = 5000  # ↑ erhöhen für besseren Druck

    width_px = int(board_width_m * pixels_per_meter)
    height_px = int(board_height_m * pixels_per_meter)

    margin_px = int(0.02 * pixels_per_meter)  # 2 cm Rand

    img = cv2.aruco.drawPlanarBoard(
        board,
        (width_px, height_px),
        margin_px,
        borderBits=1
    )

    image_filename = "aruco_gridboard.png"
    cv2.imwrite(image_filename, img)
    print(f"Bild gespeichert: {image_filename} ({width_px} x {height_px} px)")

    mm_to_pt = 72 / 25.4

    board_width_pt = board_width_mm * mm_to_pt
    board_height_pt = board_height_mm * mm_to_pt

    pdf_filename = "aruco_board.pdf"
    c = canvas.Canvas(pdf_filename, pagesize=A4)

    page_width, page_height = A4

    x = (page_width - board_width_pt) / 2
    y = (page_height - board_height_pt) / 2

    c.drawImage(
        image_filename,
        x,
        y,
        width=board_width_pt,
        height=board_height_pt
    )

    c.save()

    print(f"PDF gespeichert: {pdf_filename}")

    print("\nWICHTIG:")
    print("- Drucke mit '100%' / 'Actual Size'")
    print("- Marker sollten 40 mm groß sein")
    print("- Abstand sollte 10 mm sein")


if __name__ == "__main__":
    main()