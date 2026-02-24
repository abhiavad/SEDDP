#!/usr/bin/env python3
"""
MLX‑90640 serial reader / live‑plotter / automatic 4‑s logger.

Packet on the wire:
  * 777 little‑endian float32 values  (0 … 776)
  * followed by ASCII "\r\n"

That is 3 108 payload bytes + 2 delimiter bytes = 3 110 bytes per frame.
"""

import argparse
import csv
import queue
import struct
import sys
import threading
import time
from itertools import islice
from pathlib import Path

import matplotlib.pyplot as plt
import numpy as np
import serial
from matplotlib.animation import FuncAnimation

# ──────────────────────────────────────────────────────────────────────
#  hard‑coded filename pools
# ──────────────────────────────────────────────────────────────────────
static_test_names = []
for p in [0, 30, 60, 90]:
    for r in [30, 45, 60, 75, 90, 105]:
        static_test_names.append(f"p{p}_r{r}")

dynamic_test_names = []
for s in [5.0, 0.1]:
    for p in [0, 30, 60, 90]:
        dynamic_test_names.append(f"s{s}_p{p}")


# ──────────────────────────────────────────────────────────────────────
#  command‑line
# ──────────────────────────────────────────────────────────────────────
ap = argparse.ArgumentParser(description="MLX‑90640 logger & live plotter")
ap.add_argument("--port", "-p", default="/dev/cu.usbmodem14203", help="serial port")
ap.add_argument("--baud", "-b", type=int, default=460800, help="baud rate")
ap.add_argument("--set",  "-s", choices=["static", "dynamic"], default="static",
                help="select which hard‑coded filename list to use")
args = ap.parse_args()

name_pool = static_test_names if args.set == "static" else dynamic_test_names
name_iter = iter(name_pool)                      # StopIteration -> list exhausted

# ──────────────────────────────────────────────────────────────────────
#  constants
# ──────────────────────────────────────────────────────────────────────
DELIMITER          = b"\r\n"
PAYLOAD_BYTES      = 777 * 4        # 3 108
FRAME_BYTES        = PAYLOAD_BYTES + len(DELIMITER)
SENSOR_W, SENSOR_H = 32, 24

# ──────────────────────────────────────────────────────────────────────
#  queues and stop flag
# ──────────────────────────────────────────────────────────────────────
frame_q   = queue.Queue(maxsize=1)   # latest frame for plot
packet_q  = queue.Queue()            # full packets for logger
stop_evt  = threading.Event()

# ──────────────────────────────────────────────────────────────────────
#  serial reader thread
# ──────────────────────────────────────────────────────────────────────
def serial_reader():
    buf = bytearray()
    try:
        with serial.Serial(args.port, args.baud, timeout=1) as ser:
            while not stop_evt.is_set():
                buf.extend(ser.read(ser.in_waiting or 1))

                while True:
                    idx = buf.find(DELIMITER)
                    if idx < 0:
                        break                        # delimiter not yet present

                    packet = bytes(buf[:idx])
                    del buf[:idx + len(DELIMITER)]   # drop packet + terminator

                    if len(packet) != PAYLOAD_BYTES:
                        print(f"[WARN] mis‑aligned packet: {len(packet)} bytes "
                              f"(expected {PAYLOAD_BYTES})", file=sys.stderr)
                        continue                     # discard and continue sync

                    floats = np.asarray(struct.unpack("<777f", packet),
                                         dtype=np.float32)

                    # fan‑out
                    try:
                        frame_q.put_nowait(floats[:768])
                    except queue.Full:
                        pass
                    packet_q.put(floats)
    finally:
        stop_evt.set()

threading.Thread(target=serial_reader, daemon=True).start()

# ──────────────────────────────────────────────────────────────────────
#  logger thread
# ──────────────────────────────────────────────────────────────────────
def logger():
    """
    Start a new CSV file when byte‑775 is 1.
    Keep writing *every* packet while it stays 1.
    Close the file the first time byte‑775 returns to 0.
    Use the next name from the chosen filename list; stop creating new
    sessions once that list is exhausted.
    """
    global name_iter                           # iterator defined at top level
    filenames_exhausted = False                # set to True after list is used

    current_file   = None                      # open file handle
    csv_writer     = None
    logging_active = False                     # True while we are inside a 775==1 window

    while not stop_evt.is_set():
        try:
            pkt = packet_q.get(timeout=0.2)    # numpy array (777,)
        except queue.Empty:
            continue

        # ── show diagnostics ────────────────────────────────────────────────
        pitch, roll, ta, vdd, ttime = pkt[770:775]
        intval = pkt[775]
        sys.stdout.write(
            f"\rPitch {pitch:+6.1f}°  Roll {roll:+6.1f}°  "
            f"Ta {ta:5.1f}°C  Vdd {vdd:4.1f} V  testTime {ttime:7.0f} ms int {intval:.1f}")
        sys.stdout.flush()
        # -------------------------------------------------------------------

        # ── START new session ───────────────────────────────────────────────
        if not logging_active and intval > 0.5 and not filenames_exhausted:
            try:
                fname = next(name_iter) + ".csv"
            except StopIteration:
                print("\n[INFO] All filenames exhausted – logging disabled.")
                filenames_exhausted = True
            else:
                current_file = open(fname, "w", newline="")
                csv_writer   = csv.writer(current_file)
                # header: idx_0 … idx_776
                csv_writer.writerow([f"idx_{i}" for i in range(777)])
                logging_active = True
                print(f"\n>>> logging to {fname}")

        # ── WRITE packets while 775 == 1 ────────────────────────────────────
        if logging_active:
            csv_writer.writerow(pkt.tolist())

            # ── STOP session when 775 returns to 0 ─────────────────────────
            if intval < 0.5:
                current_file.close()
                print("<<< session finished\n")
                current_file   = None
                csv_writer     = None
                logging_active = False

threading.Thread(target=logger, daemon=True).start()

# ──────────────────────────────────────────────────────────────────────
#  live heat‑map
# ──────────────────────────────────────────────────────────────────────
fig, ax = plt.subplots(figsize=(8, 6))
im = ax.imshow(np.zeros((SENSOR_H, SENSOR_W)), interpolation='nearest')
fig.colorbar(im, ax=ax)

hover_txt = ax.text(0, -2, "", ha="center", fontsize=10, fontweight="bold")

def on_hover(evt):
    if evt.inaxes is not ax or evt.xdata is None or evt.ydata is None:
        return
    x, y = int(evt.xdata), int(evt.ydata)
    if 0 <= x < SENSOR_W and 0 <= y < SENSOR_H:
        val = im.get_array()[y, x]
        hover_txt.set_text(f"{val:5.1f}°C")
        hover_txt.set_position((x, y - 1))
        fig.canvas.draw_idle()

fig.canvas.mpl_connect("motion_notify_event", on_hover)

def update(_):
    latest = None
    while not frame_q.empty():
        latest = frame_q.get_nowait()

    if latest is not None:
        latest = latest.reshape(SENSOR_H, SENSOR_W)
        im.set_data(latest)
        im.set_clim(float(latest.min()), float(latest.max()))

    return (im,)

ani = FuncAnimation(fig, update, interval=100, blit=True, cache_frame_data=False)

# ──────────────────────────────────────────────────────────────────────
#  graceful shutdown
# ──────────────────────────────────────────────────────────────────────
def shutdown(*_):
    stop_evt.set()
    plt.close(fig)

import signal; signal.signal(signal.SIGINT, shutdown)

print("Ready – streaming packets …   (Ctrl‑C to quit)")
plt.show()
