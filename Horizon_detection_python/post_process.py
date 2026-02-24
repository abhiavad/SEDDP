#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
Created on Fri May 30 15:06:38 2025

@author: emiel
"""

import sys
import numpy as np
import horizon_detection_V6 as hd
from pathlib import Path

CSV_EXT = ".csv"
PIXELS = 768
IMG_SHAPE = (24, 32)
PITCH_IDX = 770
ROLL_IDX = 771

PITCH_OFFSET = 90 # Offset between simulation and real results

def process_csv(csv_path: Path) -> None:
    """Read *csv_path*, compute pitch/roll for every row and write it back."""

    try:
        with csv_path.open("r", newline="") as f:
            header = f.readline()
    except OSError as err:
        print(f"[ERROR] Cannot open {csv_path}: {err}", file=sys.stderr)
        return

    try:
        data = np.loadtxt(csv_path, dtype=float, delimiter=",", skiprows=1)
    except ValueError as err:
        print(f"[ERROR] Failed reading {csv_path}: {err}", file=sys.stderr)
        return

    # Ensure 2‑D even if only one data row exists.
    if data.ndim == 1:
        data = data.reshape(1, -1)

    rows, cols = data.shape
    if cols <= ROLL_IDX:
        print(
            f"[WARN] {csv_path} has {cols} columns; expected ≥ {ROLL_IDX + 1}. Skipping.",
            file=sys.stderr,
        )
        return

    # Row‑wise processing.
    for row in data:
        image = row[:PIXELS].reshape(IMG_SHAPE)

        vec = hd.vector(image)
        area = hd.integrate_angles(image)
        pitch, roll = hd.convert_coordinates(vec, area)
        
        # Pitch offset
        pitch -= PITCH_OFFSET
        
        # Console feedback
        print(f"{csv_path},{pitch},{roll}")

        row[PITCH_IDX] = pitch
        row[ROLL_IDX] = roll

    # Save back – write to temporary file first, then replace original.
    tmp = csv_path.with_suffix(".tmp")
    np.savetxt(tmp, data, delimiter=",", fmt="%.6f")

    with tmp.open("r") as r, csv_path.open("w", newline="") as w:
        w.write(header)
        w.writelines(r.readlines())
    tmp.unlink(missing_ok=True)


def main(folder: str) -> None:
    """Process all CSV files located inside folder."""
    root = Path(folder)
    if not root.is_dir():
        print(f"[ERROR] {folder} is not a directory", file=sys.stderr)
        return

    for csv_file in root.iterdir():
        if csv_file.is_file() and csv_file.suffix.lower() == CSV_EXT:
            process_csv(csv_file)

if __name__ == "__main__":
    main("/home/emiel/Documents/AE/Space Engineering Practical/result_post_process/Survivability/Vacuum/Vacuum_5_23_13.20")
