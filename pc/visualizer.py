#!/usr/bin/env python3
"""
IMU real-time visualizer — reads imu_data.h5 written by receiver.py.

Layout:
  - Left panel:  3D cube rotated by quaternion (drawn via draw commands)
  - Right panel: scrolling time-series plots for accel and gyro
  - Bottom bar:  latest packet values

Run alongside receiver.py:
  Terminal 1: imu-recv
  Terminal 2: python3 visualizer.py
"""

import time
import math
import h5py
import numpy as np
import dearpygui.dearpygui as dpg

HDF5_PATH   = "imu_data.h5"
POLL_MS     = 50        # refresh rate (~20 Hz)
HISTORY_LEN = 200       # samples in scrolling plots

# ── Quaternion → rotation matrix ─────────────────────────────────────────────
def quat_to_matrix(qi, qj, qk, qr):
    """Return 3×3 rotation matrix from unit quaternion (qr = scalar part)."""
    w, x, y, z = qr, qi, qj, qk
    return np.array([
        [1-2*(y*y+z*z),   2*(x*y-w*z),   2*(x*z+w*y)],
        [  2*(x*y+w*z), 1-2*(x*x+z*z),   2*(y*z-w*x)],
        [  2*(x*z-w*y),   2*(y*z+w*x), 1-2*(x*x+y*y)],
    ], dtype=np.float32)

# ── Unit cube vertices (centred at origin, half-side = 1) ────────────────────
CUBE_VERTS = np.array([
    [-1,-1,-1],[+1,-1,-1],[+1,+1,-1],[-1,+1,-1],  # back face
    [-1,-1,+1],[+1,-1,+1],[+1,+1,+1],[-1,+1,+1],  # front face
], dtype=np.float32)

CUBE_EDGES = [
    (0,1),(1,2),(2,3),(3,0),   # back
    (4,5),(5,6),(6,7),(7,4),   # front
    (0,4),(1,5),(2,6),(3,7),   # sides
]

FACE_QUADS = [
    ([0,1,2,3], (200, 80, 80, 180)),   # back   — red
    ([4,5,6,7], (80, 200, 80, 180)),   # front  — green
    ([0,1,5,4], (80, 80, 200, 180)),   # bottom — blue
    ([3,2,6,7], (200,200, 80, 180)),   # top    — yellow
    ([0,3,7,4], (200, 80,200, 180)),   # left   — magenta
    ([1,2,6,5], (80, 200,200, 180)),   # right  — cyan
]

def project(v, cx, cy, scale=60, fov=4.0):
    """Simple perspective projection onto 2D canvas."""
    x, y, z = v
    d = fov / (fov + z)
    return (cx + x * d * scale, cy - y * d * scale)

def draw_cube(R, cx, cy, draw_layer):
    dpg.delete_item(draw_layer, children_only=True)
    verts = (CUBE_VERTS @ R.T)   # rotate
    proj  = [project(v, cx, cy) for v in verts]

    # Draw filled faces (painter's algorithm: sort by avg z)
    face_order = sorted(
        FACE_QUADS,
        key=lambda f: np.mean([verts[i][2] for i in f[0]])
    )
    for indices, color in face_order:
        pts = [proj[i] for i in indices]
        dpg.draw_polygon(pts, color=(0,0,0,255), fill=color, parent=draw_layer)

    # Draw edges
    for a, b in CUBE_EDGES:
        dpg.draw_line(proj[a], proj[b], color=(0,0,0,255), thickness=1.5,
                      parent=draw_layer)

# ── Scrolling history buffers ─────────────────────────────────────────────────
class History:
    def __init__(self, n):
        self.n   = n
        self.t   = []
        self.ax  = []; self.ay  = []; self.az  = []
        self.gx  = []; self.gy  = []; self.gz  = []
        self.qi  = []; self.qj  = []; self.qk  = []; self.qr  = []

    def push(self, row):
        ts = float(row['timestamp_us']) / 1e6
        for buf, val in [
            (self.t,  ts),
            (self.ax, float(row['ax'])), (self.ay, float(row['ay'])), (self.az, float(row['az'])),
            (self.gx, float(row['gx'])), (self.gy, float(row['gy'])), (self.gz, float(row['gz'])),
            (self.qi, float(row['qi'])), (self.qj, float(row['qj'])),
            (self.qk, float(row['qk'])), (self.qr, float(row['qr'])),
        ]:
            buf.append(val)
            if len(buf) > self.n:
                buf.pop(0)

# ── Main ──────────────────────────────────────────────────────────────────────
def main():
    hist      = History(HISTORY_LEN)
    last_idx  = 0
    last_R    = np.eye(3, dtype=np.float32)

    dpg.create_context()
    dpg.create_viewport(title='IMU Visualizer', width=1100, height=650)
    dpg.setup_dearpygui()

    CUBE_W, CUBE_H = 340, 340
    cx, cy = CUBE_W // 2, CUBE_H // 2

    with dpg.window(label='IMU Visualizer', tag='main_win',
                    no_close=True, no_move=True, no_resize=True,
                    width=1100, height=650, pos=(0, 0)):

        with dpg.group(horizontal=True):
            # ── Left: cube ──────────────────────────────────────────────
            with dpg.group():
                dpg.add_text('Orientation (quaternion → 3D)')
                with dpg.drawlist(width=CUBE_W, height=CUBE_H, tag='cube_canvas'):
                    with dpg.draw_layer(tag='cube_layer'):
                        pass

            dpg.add_spacer(width=20)

            # ── Right: plots ─────────────────────────────────────────────
            with dpg.group():
                dpg.add_text('Linear Acceleration (m/s²)')
                with dpg.plot(width=680, height=150, no_mouse_pos=True, tag='accel_plot'):
                    dpg.add_plot_axis(dpg.mvXAxis, label='', no_tick_labels=True, tag='accel_x')
                    with dpg.plot_axis(dpg.mvYAxis, label='', tag='accel_y'):
                        dpg.add_line_series([], [], label='ax', tag='s_ax')
                        dpg.add_line_series([], [], label='ay', tag='s_ay')
                        dpg.add_line_series([], [], label='az', tag='s_az')
                    dpg.add_plot_legend()

                dpg.add_text('Gyro (rad/s)')
                with dpg.plot(width=680, height=150, no_mouse_pos=True, tag='gyro_plot'):
                    dpg.add_plot_axis(dpg.mvXAxis, label='', no_tick_labels=True, tag='gyro_x')
                    with dpg.plot_axis(dpg.mvYAxis, label='', tag='gyro_y'):
                        dpg.add_line_series([], [], label='gx', tag='s_gx')
                        dpg.add_line_series([], [], label='gy', tag='s_gy')
                        dpg.add_line_series([], [], label='gz', tag='s_gz')
                    dpg.add_plot_legend()

                dpg.add_text('Quaternion')
                with dpg.plot(width=680, height=150, no_mouse_pos=True, tag='quat_plot'):
                    dpg.add_plot_axis(dpg.mvXAxis, label='', no_tick_labels=True, tag='quat_x')
                    with dpg.plot_axis(dpg.mvYAxis, label='', tag='quat_y'):
                        dpg.add_line_series([], [], label='qi', tag='s_qi')
                        dpg.add_line_series([], [], label='qj', tag='s_qj')
                        dpg.add_line_series([], [], label='qk', tag='s_qk')
                        dpg.add_line_series([], [], label='qr', tag='s_qr')
                    dpg.add_plot_legend()

        dpg.add_separator()
        dpg.add_text('-- waiting for data --', tag='status_text')

    def refresh():
        nonlocal last_idx, last_R
        try:
            with h5py.File(HDF5_PATH, 'r', swmr=True) as hf:
                ds   = hf['imu']
                ds.id.refresh()
                n    = ds.shape[0]
                if n <= last_idx:
                    return
                new_rows = ds[last_idx:n]
                last_idx = n
        except Exception:
            return

        for row in new_rows:
            hist.push(row)

        if not hist.t:
            return

        # Latest quaternion → rotation matrix
        qi, qj, qk, qr = hist.qi[-1], hist.qj[-1], hist.qk[-1], hist.qr[-1]
        mag = math.sqrt(qi*qi + qj*qj + qk*qk + qr*qr)
        if mag > 0.01:
            last_R = quat_to_matrix(qi/mag, qj/mag, qk/mag, qr/mag)

        draw_cube(last_R, cx, cy, 'cube_layer')

        t = hist.t
        dpg.set_value('s_ax', [t, hist.ax])
        dpg.set_value('s_ay', [t, hist.ay])
        dpg.set_value('s_az', [t, hist.az])
        dpg.set_axis_limits('accel_x', t[0], t[-1])
        dpg.fit_axis_data('accel_y')

        dpg.set_value('s_gx', [t, hist.gx])
        dpg.set_value('s_gy', [t, hist.gy])
        dpg.set_value('s_gz', [t, hist.gz])
        dpg.set_axis_limits('gyro_x', t[0], t[-1])
        dpg.fit_axis_data('gyro_y')

        dpg.set_value('s_qi', [t, hist.qi])
        dpg.set_value('s_qj', [t, hist.qj])
        dpg.set_value('s_qk', [t, hist.qk])
        dpg.set_value('s_qr', [t, hist.qr])
        dpg.set_axis_limits('quat_x', t[0], t[-1])
        dpg.set_axis_limits('quat_y', -1.1, 1.1)

        dpg.set_value('status_text',
            f'node={int(new_rows[-1]["node_id"])}  '
            f'q=({qi:.3f},{qj:.3f},{qk:.3f},{qr:.3f})  '
            f'acc=({hist.ax[-1]:.2f},{hist.ay[-1]:.2f},{hist.az[-1]:.2f}) m/s²  '
            f'gyro=({hist.gx[-1]:.2f},{hist.gy[-1]:.2f},{hist.gz[-1]:.2f}) rad/s  '
            f'| total={last_idx}')

    with dpg.item_handler_registry(tag='frame_handler'):
        dpg.add_item_clicked_handler()

    dpg.show_viewport()
    dpg.set_primary_window('main_win', True)

    last_poll = 0.0
    while dpg.is_dearpygui_running():
        now = time.time()
        if now - last_poll >= POLL_MS / 1000.0:
            refresh()
            last_poll = now
        dpg.render_dearpygui_frame()

    dpg.destroy_context()


if __name__ == '__main__':
    main()
