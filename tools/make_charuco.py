"""Writes the calibration board texture assets/calib/charuco_7x5.png.

7 × 5 squares of 30 mm with 22 mm markers (DICT_5X5_100), no margin, so the
image maps exactly onto the board's 0.21 × 0.15 m face. Dev-time only:
  .venv/bin/python tools/make_charuco.py
"""

from pathlib import Path

import cv2

OUT = Path(__file__).resolve().parents[1] / "assets/calib/charuco_7x5.png"


def main() -> None:
  board = cv2.aruco.CharucoBoard(
      (7, 5),
      0.03,
      0.022,
      cv2.aruco.getPredefinedDictionary(cv2.aruco.DICT_5X5_100),
  )
  image = board.generateImage((1400, 1000), marginSize=0)
  OUT.parent.mkdir(parents=True, exist_ok=True)
  cv2.imwrite(str(OUT), image)
  print(f"{OUT}: {image.shape[1]}x{image.shape[0]}")


if __name__ == "__main__":
  main()
