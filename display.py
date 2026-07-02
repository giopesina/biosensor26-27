import matplotlib
matplotlib.use('TkAgg')

import matplotlib.pyplot as plt
import matplotlib.animation as animation
import serial
import re
import numpy as np
from collections import deque

# ── Config ────────────────────────────────────────────────────────────────────
PORT            = 'COM5'
BAUD            = 115200
MAX_POINTS      = 2000
INTERVAL_MS     = 15
AVG_WINDOW      = 1
MAX_OLD_RUNS    = 10        # cap history so old traces don't pile up forever
MAX_LINES_READ  = 2000      # hard cap per frame so a runaway buffer can't stall the GUI

# Fixed axis limits (do not autoscale every frame)
X_LIM = (-0.35, 0.65)     # Volts
Y_LIM = (-75, 75)       # microamps

# ── Serial ────────────────────────────────────────────────────────────────────
ser = serial.Serial(PORT, BAUD, timeout=0.05)

# ── Data ──────────────────────────────────────────────────────────────────────
fwd_v = deque(maxlen=MAX_POINTS)
fwd_i = deque(maxlen=MAX_POINTS)
rev_v = deque(maxlen=MAX_POINTS)
rev_i = deque(maxlen=MAX_POINTS)

# Archived completed runs, kept for overlay comparison.
# Each entry: {'fv': [...], 'fi': [...], 'rv': [...], 'ri': [...]}
old_runs = []

cv_active     = False
cv_done       = False
status_msg    = "Waiting for CV data... Press 'r' in plot window to start."
baseline_uA   = None   # most recently reported baseline from the firmware

# Tracks the last voltage seen per sweep direction, so we can detect when a
# new cycle restarts mid-run (voltage jumps back toward CV_CODE_START instead
# of continuing its sweep) and break the line there instead of drawing a
# straight connector across the whole plot.
last_fwd_v    = None
last_rev_v    = None
CYCLE_JUMP_V  = 0.10   # volts — bigger than a normal per-step delta, smaller than a full sweep span

# Dirty flags — only touch the plot when something actually changed
data_dirty      = True   # live traces need redraw
old_runs_dirty  = True   # archived traces need redraw
status_dirty    = True   # status text needs redraw

# ── Line parsing ──────────────────────────────────────────────────────────────
csv_pattern = re.compile(
    r'^\s*(-?[\d.]+)\s*,\s*(-?[\d.]+)\s*,\s*(fwd|rev)\s*$'
)
# Matches both "Baseline = X uA" and "Baseline used for this sweep = X uA"
baseline_pattern = re.compile(
    r'Baseline[^=]*=\s*(-?[\d.]+)\s*uA'
)

# Debug/status prefixes emitted by the firmware that are not CSV data
# and should never be fed to the CSV parser.
IGNORE_PREFIXES = ('ADC=', 'Baseline', 'Done', 'Send', 'ADIID', 'CHIPID',
                    'Debug', 'Requested', 'DAC', '--', 'AD5941', '=====')

# ── Running average helper ────────────────────────────────────────────────────
def running_avg(values, window):
    arr    = np.array(values, dtype=float)
    if len(arr) == 0:
        return arr
    cumsum = np.cumsum(arr)
    cumsum[window:] = cumsum[window:] - cumsum[:-window]
    result = np.empty_like(arr)
    result[:window - 1] = cumsum[:window - 1] / np.arange(1, window)
    result[window - 1:] = cumsum[window - 1:] / window
    return result

# ── Keypress handler ──────────────────────────────────────────────────────────
def on_key(event):
    global status_msg, status_dirty
    if event.key == 'r':
        ser.write(b'r')
        status_msg = "Sent 'r' — waiting for CV to start..."
        status_dirty = True
        print("Sent 'r' to Arduino")
    elif event.key == 'c':
        old_runs.clear()
        status_msg = "Cleared run history."
        status_dirty = True
        globals()['old_runs_dirty'] = True
        print("Cleared old runs")

# ── Figure (static chrome built ONCE, not per frame) ───────────────────────────
fig, ax = plt.subplots(figsize=(9, 6))
fig.patch.set_facecolor('#0f0f0f')
fig.canvas.mpl_connect('key_press_event', on_key)

ax.set_facecolor('#0f0f0f')
ax.set_title("Cyclic Voltammogram", color='white', fontsize=13, pad=12)
ax.set_xlabel("Applied Potential (V)", color='#aaaaaa', fontsize=11)
ax.set_ylabel("Current (µA)",          color='#aaaaaa', fontsize=11)
ax.tick_params(colors='#aaaaaa')
for spine in ax.spines.values():
    spine.set_edgecolor('#333333')
ax.axhline(0, color='#444444', linewidth=0.8)
ax.axvline(0, color='#444444', linewidth=0.8)
ax.grid(True, color='#1e1e1e', linewidth=0.8)
ax.set_xlim(*X_LIM)
ax.set_ylim(*Y_LIM)

# Persistent artists for archived (faded) runs — pre-allocated pool, hidden until used
old_fwd_lines = [ax.plot([], [], lw=0.8, color='#00d4ff', alpha=0.12)[0]
                  for _ in range(MAX_OLD_RUNS)]
old_rev_lines = [ax.plot([], [], lw=0.8, color='#ff9900', alpha=0.12)[0]
                  for _ in range(MAX_OLD_RUNS)]

# Persistent artists for the live sweep
line_fwd_raw, = ax.plot([], [], lw=0.8, color='#00d4ff', alpha=0.30, label='Forward (raw)')
line_fwd_avg, = ax.plot([], [], lw=2.4, color='#00ff88', label=f'Forward avg ({AVG_WINDOW}pt)')
marker_fwd,   = ax.plot([], [], 'o', color='#ff4f4f', markersize=6, zorder=5)

line_rev_raw, = ax.plot([], [], lw=0.8, color='#ff9900', alpha=0.30, label='Reverse (raw)')
line_rev_avg, = ax.plot([], [], lw=2.4, color='#aaff00', label=f'Reverse avg ({AVG_WINDOW}pt)')
marker_rev,   = ax.plot([], [], 'o', color='#ff4f4f', markersize=6, zorder=5)

legend = ax.legend(facecolor='#1a1a1a', edgecolor='#333333', labelcolor='white',
                    fontsize=8.5, loc='lower right', framealpha=0.85)

status_text = ax.text(
    0.02, 0.97, "",
    transform=ax.transAxes,
    fontsize=8, color='#aaaaaa',
    verticalalignment='top', family='monospace'
)


def parse_line(line):
    """
    Update global data structures from a single serial line.
    Returns True if the line was a recognized CV START/END/CSV/baseline
    line, False if it was ignored.
    """
    global cv_active, cv_done, status_msg, baseline_uA
    global data_dirty, old_runs_dirty, status_dirty
    global last_fwd_v, last_rev_v

    if '=== CV START ===' in line:
        # Archive the previous run (if it has data) instead of discarding it
        if len(fwd_v) >= 2 or len(rev_v) >= 2:
            old_runs.append({
                'fv': list(fwd_v), 'fi': list(fwd_i),
                'rv': list(rev_v), 'ri': list(rev_i),
            })
            if len(old_runs) > MAX_OLD_RUNS:
                old_runs.pop(0)
            old_runs_dirty = True

        cv_active  = True
        cv_done    = False
        status_msg = f"CV running... ({len(old_runs)} previous run(s) shown faded)"
        fwd_v.clear(); fwd_i.clear()
        rev_v.clear(); rev_i.clear()
        last_fwd_v = None
        last_rev_v = None
        data_dirty = True
        status_dirty = True
        return True

    if '=== CV END ===' in line:
        cv_done    = True
        cv_active  = False
        status_msg = "CV complete. Press 'r' to run again, 'c' to clear history."
        status_dirty = True
        return True

    # Baseline lines carry useful info even though they're not CSV data.
    # Checked before IGNORE_PREFIXES since "Baseline" is also in that list.
    m_base = baseline_pattern.search(line)
    if m_base:
        baseline_uA = float(m_base.group(1))
        status_dirty = True
        return True

    # Skip all other known debug/status lines.
    if line.startswith(IGNORE_PREFIXES):
        return False

    if not cv_active:
        return False

    m = csv_pattern.match(line)
    if m:
        v         = float(m.group(1))
        i         = float(m.group(2))
        direction = m.group(3)
        if direction == 'fwd':
            # Forward sweep normally moves in one direction (voltage rising).
            # A backward jump bigger than a normal step means a new cycle
            # started — break the line instead of connecting straight across.
            if last_fwd_v is not None and (v - last_fwd_v) < -CYCLE_JUMP_V:
                fwd_v.append(float('nan'))
                fwd_i.append(float('nan'))
            fwd_v.append(v)
            fwd_i.append(i)
            last_fwd_v = v
        else:
            # Reverse sweep normally moves the opposite direction (voltage
            # falling). A forward jump bigger than a normal step means a new
            # cycle's reverse leg started — break the line there too.
            if last_rev_v is not None and (v - last_rev_v) > CYCLE_JUMP_V:
                rev_v.append(float('nan'))
                rev_i.append(float('nan'))
            rev_v.append(v)
            rev_i.append(i)
            last_rev_v = v
        data_dirty = True
        status_dirty = True
        return True

    return False


def update(frame):
    global data_dirty, old_runs_dirty, status_dirty

    # ── Drain EVERYTHING currently sitting in the OS serial buffer ─────────────
    # (fixed-count loops fall behind if the firmware bursts faster than
    #  INTERVAL_MS; this drains until empty or a hard safety cap)
    n_read = 0
    while ser.in_waiting and n_read < MAX_LINES_READ:
        raw = ser.readline()
        if not raw:
            break
        line = raw.decode(errors='ignore').strip()
        n_read += 1
        if line:
            parse_line(line)

    changed = []

    # ── Archived runs — only touch when the archive actually changed ───────────
    if old_runs_dirty:
        for idx, ln in enumerate(old_fwd_lines):
            if idx < len(old_runs):
                ln.set_data(old_runs[idx]['fv'], old_runs[idx]['fi'])
            else:
                ln.set_data([], [])
            changed.append(ln)
        for idx, ln in enumerate(old_rev_lines):
            if idx < len(old_runs):
                ln.set_data(old_runs[idx]['rv'], old_runs[idx]['ri'])
            else:
                ln.set_data([], [])
            changed.append(ln)
        old_runs_dirty = False

    # ── Live traces — only touch when new data arrived ──────────────────────────
    if data_dirty:
        fv = list(fwd_v)
        fi = list(fwd_i)
        rv = list(rev_v)
        ri = list(rev_i)

        if len(fv) >= 2:
            line_fwd_raw.set_data(fv, fi)
            if len(fi) >= AVG_WINDOW:
                line_fwd_avg.set_data(fv, running_avg(fi, AVG_WINDOW))
            marker_fwd.set_data([fv[-1]], [fi[-1]])
        else:
            line_fwd_raw.set_data([], [])
            line_fwd_avg.set_data([], [])
            marker_fwd.set_data([], [])

        if len(rv) >= 2:
            line_rev_raw.set_data(rv, ri)
            if len(ri) >= AVG_WINDOW:
                line_rev_avg.set_data(rv, running_avg(ri, AVG_WINDOW))
            marker_rev.set_data([rv[-1]], [ri[-1]])
        else:
            line_rev_raw.set_data([], [])
            line_rev_avg.set_data([], [])
            marker_rev.set_data([], [])

        changed += [line_fwd_raw, line_fwd_avg, marker_fwd,
                    line_rev_raw, line_rev_avg, marker_rev]
        data_dirty = False

        # ── Status panel — piggybacks on data_dirty since it reports point counts ──
        # (filter out NaN cycle-break markers so they don't skew min/max)
        all_i = [x for x in (fi + ri) if not np.isnan(x)]
        n_fwd, n_rev = len(fv), len(rv)
        n_total = n_fwd + n_rev

        baseline_str = f"{baseline_uA:.3f} uA" if baseline_uA is not None else "n/a"
        min_str = f"{min(all_i):.3f} uA" if all_i else "n/a"
        max_str = f"{max(all_i):.3f} uA" if all_i else "n/a"

        status_text.set_text("\n".join([
            status_msg,
            f"Forward pts: {n_fwd}    Reverse pts: {n_rev}    Total: {n_total}",
            f"Baseline: {baseline_str}    Min: {min_str}    Max: {max_str}",
            f"Archived runs: {len(old_runs)}   ('r' = new run, 'c' = clear history)",
        ]))
        changed.append(status_text)
        status_dirty = False

    elif status_dirty:
        # status_msg changed (e.g. keypress) without new data points
        status_text.set_text(status_msg)
        changed.append(status_text)
        status_dirty = False

    return changed


ani = animation.FuncAnimation(
    fig, update,
    interval=INTERVAL_MS,
    blit=False,
    cache_frame_data=False
)

try:
    plt.show()
finally:
    ser.close()
