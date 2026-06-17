#!/usr/bin/env python3
"""
visualize_local_map.py

Fetches /snapshot from the local_map_server and displays it
as a live grayscale image using matplotlib.

Usage:
    python3 visualize_local_map.py
    python3 visualize_local_map.py --host 192.168.1.10 --port 8765 --hz 5
"""

import argparse
import signal
import sys
import numpy as np
import requests
import matplotlib.pyplot as plt
import matplotlib.animation as animation

def _quit(sig, frame):
    plt.close("all")
    sys.exit(0)

signal.signal(signal.SIGINT, _quit)
signal.signal(signal.SIGTERM, _quit)

parser = argparse.ArgumentParser()
parser.add_argument("--host", default="localhost")
parser.add_argument("--port", default=8765, type=int)
parser.add_argument("--hz",   default=5, type=float)
args = parser.parse_args()

URL = f"http://{args.host}:{args.port}/snapshot"

fig, ax = plt.subplots(figsize=(6, 6))
fig.patch.set_facecolor("#0d1117")
ax.set_facecolor("#0d1117")
ax.set_title("local map", color="white", fontsize=10)
ax.tick_params(colors="white")
for spine in ax.spines.values():
    spine.set_edgecolor("#30363d")

# Placeholder image
dummy = np.zeros((200, 200), dtype=np.uint8)
im = ax.imshow(dummy, cmap="gray", vmin=0, vmax=255,
               interpolation="nearest", origin="upper")
status_text = ax.text(0.01, 0.99, "", transform=ax.transAxes,
                      color="lime", fontsize=7, va="top",
                      fontfamily="monospace")

def fetch():
    r = requests.get(URL, timeout=1.0)
    return r.json()

def update(_frame):
    try:
        d = fetch()
        size = d["local_size"]
        arr  = np.array(d["data"], dtype=np.int8).reshape((size, size))

        # -1 (unknown) → 128, 0 (free) → 255, 100 (occupied) → 0
        vis = np.where(arr == -1, 128,
              np.where(arr == 0,  255,
              np.clip(255 - arr.astype(np.int16) * 255 // 100, 0, 255))
              ).astype(np.uint8)

        im.set_data(vis)
        im.set_extent([-0.5, size - 0.5, size - 0.5, -0.5])

        tf = "✓" if d["robot_found"] else "✗ no TF"
        status_text.set_text(
            f"x={d['robot_x']:.2f}m  y={d['robot_y']:.2f}m  "
            f"yaw={d['robot_yaw']*57.3:.1f}°  tf={tf}  "
            f"res={d['resolution']*100:.1f}cm/cell"
        )
        ax.set_title(
            f"local map  {size}×{size}",
            color="white", fontsize=10
        )
    except Exception as e:
        status_text.set_text(f"error: {e}")
        print(e.with_traceback())

    return [im, status_text]

interval_ms = int(1000 / args.hz)
ani = animation.FuncAnimation(fig, update, interval=interval_ms, blit=True)
plt.tight_layout()
plt.show()