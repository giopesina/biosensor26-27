import matplotlib
matplotlib.use('TkAgg')

import matplotlib.pyplot as plt
import matplotlib.animation as animation
import serial
import re
from collections import deque

# ── Config ────────────────────────────────────────────────────────────────────
PORT            = 'COM5'
BAUD            = 115200
MAX_POINTS      = 500
VISIBLE_POINTS  = 30        # only the newest 30 points shown at any time
INTERVAL_MS     = 100
LINES_PER_FRAME = 20
PADDING_FRAC    = 0.15      # 15% padding around data range on each axis

# ── Serial ────────────────────────────────────────────────────────────────────
ser = serial.Serial(PORT, BAUD, timeout=0.05)

# ── Data ──────────────────────────────────────────────────────────────────────
cv_v = deque(maxlen=MAX_POINTS)
cv_i = deque(maxlen=MAX_POINTS)

pattern = re.compile(
    r'Vtia\s*=\s*(-?[\d.]+)\s*V\s*\|\s*I\s*=\s*(-?[\d.]+)\s*uA'
)

# ── Figure ────────────────────────────────────────────────────────────────────
fig, ax = plt.subplots(figsize=(9, 6))
fig.patch.set_facecolor('#0f0f0f')

def padded_limits(values, frac=PADDING_FRAC):
    """Return (lo, hi) centered on the data range with fractional padding."""
    lo, hi = min(values), max(values)
    span = hi - lo
    pad  = span * frac if span > 0 else abs(lo) * frac or 0.1
    center = (lo + hi) / 2
    half   = (span / 2) + pad
    return center - half, center + half

def update(frame):
    for _ in range(LINES_PER_FRAME):
        raw = ser.readline()
        if not raw:
            break
        m = pattern.search(raw.decode(errors='ignore'))
        if m:
            cv_v.append(float(m.group(1)))
            cv_i.append(float(m.group(2)))

    ax.clear()
    ax.set_facecolor('#0f0f0f')
    ax.set_title("Vtia vs Current  (last 30 points)", color='white', fontsize=13, pad=12)
    ax.set_xlabel("Vtia  (V)",     color='#aaaaaa', fontsize=11)
    ax.set_ylabel("Current  (µA)", color='#aaaaaa', fontsize=11)
    ax.tick_params(colors='#aaaaaa')
    for spine in ax.spines.values():
        spine.set_edgecolor('#333333')
    ax.axhline(0, color='#333333', linewidth=0.8)
    ax.axvline(0, color='#333333', linewidth=0.8)
    ax.grid(True, color='#1e1e1e', linewidth=0.8)

    if len(cv_v) >= 2:
        # Slice to newest VISIBLE_POINTS only
        xs = list(cv_v)[-VISIBLE_POINTS:]
        ys = list(cv_i)[-VISIBLE_POINTS:]

        ax.plot(xs, ys, lw=2, color='#00d4ff')

        # Mark the newest point
        ax.plot(xs[-1], ys[-1], 'o', color='#ff4f4f', markersize=6, zorder=5)

        # Axis limits centered on visible data range
        ax.set_xlim(*padded_limits(xs))
        ax.set_ylim(*padded_limits(ys))

        ax.text(
            0.02, 0.97,
            f"Vtia = {xs[-1]:+.6f} V    I = {ys[-1]:+.4f} µA    "
            f"n = {len(cv_v)}  (showing {len(xs)})",
            transform=ax.transAxes,
            fontsize=8, color='#aaaaaa',
            verticalalignment='top', family='monospace'
        )

    fig.tight_layout()

ani = animation.FuncAnimation(
    fig, update,
    interval=INTERVAL_MS,
    cache_frame_data=False
)

try:
    plt.show()
finally:
    ser.close()
