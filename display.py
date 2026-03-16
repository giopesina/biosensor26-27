import serial
import matplotlib
matplotlib.use('TkAgg')
import matplotlib.pyplot as plt
import matplotlib.animation as animation
import re

# --- config ---
PORT        = 'COM5'
BAUD        = 115200
LINES_PER_FRAME = 20
INTERVAL_MS     = 20
MAX_POINTS      = 200
SMOOTH_N        = 3     # number of points to average for smoothing

ser = serial.Serial(PORT, BAUD, timeout=0.01)

# CV data — paired correctly per block
a0_sweep = []
a0_i     = []
a2_sweep = []
a2_i     = []
sweep_trace = []

# one block buffer — only commit on '---'
block = {'sweep': None, 'a0_i': None, 'a2_i': None}

fig, ax = plt.subplots(figsize=(10, 7))
fig.suptitle('Cyclic Voltammogram — Live', fontsize=13)

def smooth(data, n=SMOOTH_N):
    if len(data) < n:
        return data
    return [sum(data[max(0, i - n):i + 1]) / len(data[max(0, i - n):i + 1])
            for i in range(len(data))]

def update(frame):
    try:
        for _ in range(LINES_PER_FRAME):
            raw = ser.readline()
            if not raw:
                continue
            line = raw.decode('utf-8', errors='ignore').strip()

            # capture sweep voltage
            m = re.match(r'sweep:\s*(-?[\d.]+)\s*V', line)
            if m:
                block['sweep'] = float(m.group(1))

            # capture A0 current
            m = re.match(r'A0:\s*(-?[\d.]+)\s*V\s*\|\s*(-?[\d.]+)\s*nA', line)
            if m:
                block['a0_i'] = float(m.group(2))

            # capture A2 current
            m = re.match(r'A2:\s*(-?[\d.]+)\s*V\s*\|\s*(-?[\d.]+)\s*nA', line)
            if m:
                block['a2_i'] = float(m.group(2))

            # commit only when block separator is hit
            if line == '---' and block['sweep'] is not None:
                sweep_trace.append(block['sweep'])
                if len(sweep_trace) > MAX_POINTS:
                    del sweep_trace[:len(sweep_trace) - MAX_POINTS]

                if block['a0_i'] is not None:
                    a0_sweep.append(block['sweep'])
                    a0_i.append(block['a0_i'])
                    if len(a0_sweep) > MAX_POINTS:
                        del a0_sweep[0]
                        del a0_i[0]

                if block['a2_i'] is not None:
                    a2_sweep.append(block['sweep'])
                    a2_i.append(block['a2_i'])
                    if len(a2_sweep) > MAX_POINTS:
                        del a2_sweep[0]
                        del a2_i[0]

                # reset block
                block['sweep'] = None
                block['a0_i']  = None
                block['a2_i']  = None

        if not a0_i and not a2_i:
            return

        ax.cla()

        # A0 — oxidation curve (positive sweep, positive current)
        if len(a0_i) > 1:
            ax.plot(smooth(a0_i), a0_sweep,
                    color='#2196F3',
                    linewidth=2,
                    label='A0 — oxidation')

        # A2 — reduction curve (negative sweep, negative current)
        if len(a2_i) > 1:
            ax.plot(smooth(a2_i), a2_sweep,
                    color='#E91E63',
                    linewidth=2,
                    label='A2 — reduction')

        # sweep reference — dashed semi-transparent overlay
        if len(sweep_trace) > 1:
            n = len(sweep_trace)
            x_ref = [-80 + 160 * k / (n - 1) for k in range(n)]
            ax.plot(x_ref, sweep_trace,
                    color='gray',
                    linewidth=1.2,
                    linestyle='--',
                    alpha=0.3,
                    label='sweep ref')

        # zero reference lines
        ax.axhline(0, color='gray', linewidth=0.5, linestyle='-', alpha=0.3)
        ax.axvline(0, color='gray', linewidth=0.5, linestyle='-', alpha=0.3)

        ax.set_xlabel('current (nA)', fontsize=11)
        ax.set_ylabel('voltage (V)', fontsize=11)
        ax.set_xlim(-80, 80)
        ax.set_ylim(-1.8, 1.8)
        ax.legend(loc='upper left', fontsize=10)
        ax.grid(True, alpha=0.15)
        fig.tight_layout()

    except Exception as e:
        print(f"Error: {e}")

ani = animation.FuncAnimation(
    fig,
    update,
    interval=INTERVAL_MS,
    cache_frame_data=False,
    save_count=MAX_POINTS
)

plt.show(block=True)
ser.close()