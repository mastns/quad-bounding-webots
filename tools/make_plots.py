#!/usr/bin/env python3
"""Figures from the controllers' CSV logs in plots/data (written by tools/collect_plot_data.py).

Per run (plots/oct2014, plots/feb2013): forward velocity, CoM height, pitch, phase planes, gait
diagram, leg angles, per-gait convergence and CoM path; the same for the MuJoCo runs
(plots/mujoco_oct2014, plots/mujoco_feb2013). Comparisons (plots/comparisons): speed tracking for
three desired speeds, the Oct world as converted versus tuned, and Webots versus MuJoCo.
Every figure is saved as PNG (200 dpi, for slides) and PDF (vector, for LaTeX).

usage: ~/.local/opt/quadsim-venv/bin/python tools/make_plots.py      (needs matplotlib and numpy)
"""
import csv
import glob
import os

import matplotlib
matplotlib.use('Agg')
import matplotlib.lines  # noqa: E402
import matplotlib.pyplot as plt  # noqa: E402
import matplotlib.ticker  # noqa: E402
import numpy as np  # noqa: E402

ROOT = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
DATA = os.path.join(ROOT, 'plots', 'data')
OUT = os.path.join(ROOT, 'plots')

# palette: reference categorical slots 1-3 (validated all-pairs), text and grid tokens
BLUE, ORANGE, AQUA = '#2a78d6', '#eb6834', '#1baf7a'
TEXT, TEXT2, MUTED, GRID, BAND = '#0b0b0b', '#52514e', '#a3a29d', '#e4e3df', '#f0efeb'
FRONT, BACK = BLUE, ORANGE          # same colours as the leg-pair rings on the showcase robot

RUNS = {
    'oct2014': dict(label='Oct 2014 controller (tuned world)', xddes=0.9, h=0.32, tol=0.001),
    'feb2013': dict(label='Feb 2013 thesis version (tuned world)', xddes=0.7, h=0.32, tol=0.0001),
    'oct2014_xddes0.6': dict(label='Oct 2014, xddes 0.6', xddes=0.6, h=0.32, tol=0.001),
    'oct2014_xddes1.1': dict(label='Oct 2014, xddes 1.1', xddes=1.1, h=0.32, tol=0.001),
    'oct2014_as_converted': dict(label='Oct 2014, as converted', xddes=0.9, h=0.32, tol=0.001),
    'mujoco_oct2014': dict(label='Oct 2014 controller on MuJoCo 4.0', xddes=0.9, h=0.32, tol=0.001),
    'mujoco_feb2013': dict(label='Feb 2013 thesis version on MuJoCo 4.0', xddes=0.7, h=0.32, tol=0.0001),
}
MAX_TORQUE = 6.5

plt.rcParams.update({
    'figure.facecolor': 'white', 'axes.facecolor': 'white', 'savefig.facecolor': 'white',
    'font.size': 9.5, 'axes.titlesize': 9.5, 'axes.titlelocation': 'left', 'axes.titlecolor': TEXT2,
    'axes.labelcolor': TEXT2, 'axes.labelsize': 9.5, 'axes.edgecolor': MUTED, 'axes.linewidth': 0.6,
    'axes.spines.top': False, 'axes.spines.right': False, 'axes.grid': True, 'axes.axisbelow': True,
    'grid.color': GRID, 'grid.linewidth': 0.6, 'grid.linestyle': '-',
    'xtick.color': TEXT2, 'ytick.color': TEXT2, 'xtick.labelsize': 8.5, 'ytick.labelsize': 8.5,
    'xtick.major.width': 0.6, 'ytick.major.width': 0.6, 'xtick.major.size': 3, 'ytick.major.size': 3,
    'lines.linewidth': 1.4, 'legend.frameon': False, 'legend.fontsize': 8.5, 'text.color': TEXT,
    'figure.constrained_layout.use': True, 'savefig.dpi': 200,
})


# ----------------------------------------------------------------------------- data
def read_csv(path):
    rows = list(csv.reader(open(path)))
    cols = {name: [] for name in rows[0]}
    for r in rows[1:]:
        for name, v in zip(rows[0], r):
            if v != '':
                cols[name].append(float(v))
    return {k: np.array(v) for k, v in cols.items()}


class Run:
    def __init__(self, name):
        info = RUNS[name]
        self.name, self.label, self.xddes, self.h, self.tol = name, info['label'], info['xddes'], info['h'], info['tol']
        d = os.path.join(DATA, name)
        steps = read_csv(sorted(glob.glob(os.path.join(d, '*_steps.csv')))[-1])
        gaits = read_csv(sorted(glob.glob(os.path.join(d, '*_gaits.csv')))[-1])
        s = {k: v[1:] for k, v in steps.items()}             # row 0 is the MATLAB initialisation
        g = {k: v[1:] for k, v in gaits.items()}
        self.t = np.arange(1, len(s['Wyp']) + 1) / 1000.0    # one row per 1 ms control step
        self.xd, self.y, self.yd = s['Wxdp'], s['Wyp'], s['Wydp']
        self.th, self.thd = s['Wthp'], s['Wthdp']
        self.gamb, self.gamf = s['Wgambp'], s['Wgamfp']
        self.spring = [s['Wspring%d_pos_p' % i] for i in (1, 2, 3, 4)]
        self.back = (self.spring[0] > self.tol) | (self.spring[3] > self.tol)
        self.front = (self.spring[1] > self.tol) | (self.spring[2] > self.tol)
        self.x = np.cumsum(self.xd) / 1000.0                  # forward displacement from the velocity
        self.t_lo = g['Wgaitnumbertime'] / 1000.0
        self.xdlo, self.gambtd, self.gamftd, self.tor = g['Wxdlop'], g['Wgambtdp'], g['Wgamftdp'], g['Wtorbctrp']
        idx = np.clip((g['Wgaitnumbertime'] - 1).astype(int), 0, len(self.t) - 1)
        self.lo_idx = idx

    def window(self, length=1.5):
        """A steady-state window: the last `length` seconds before the final liftoff."""
        end = self.t_lo[-1] if len(self.t_lo) else self.t[-1]
        return end - length, end

    def apexes(self):
        """Apex height of each flight: max height between successive liftoffs."""
        out = []
        for a, b in zip(self.lo_idx[:-1], self.lo_idx[1:]):
            out.append(self.y[a:b].max())
        return np.array(out)


# ----------------------------------------------------------------------------- helpers
def save(fig, folder, name):
    for ax in fig.axes:                       # target labels go at the final right edge of each axis
        for value, text, below in getattr(ax, 'quad_targets', []):
            ax.annotate(text, (ax.get_xlim()[1], value), xytext=(-2, -3 if below else 3), textcoords='offset points',
                        ha='right', va='top' if below else 'bottom', fontsize=8, color=TEXT2)
    os.makedirs(os.path.join(OUT, folder), exist_ok=True)
    for ext in ('png', 'pdf'):
        fig.savefig(os.path.join(OUT, folder, '%s.%s' % (name, ext)))
    plt.close(fig)
    print('wrote plots/%s/%s.{png,pdf}' % (folder, name))


def header(fig, title):
    fig.suptitle(title, x=0.0, ha='left', fontsize=12, fontweight='bold', color=TEXT)


DASH = (0, (4, 3))


def target(ax, value, text=None, color=TEXT2, below=False):
    """Dashed reference line (a target, not a grid line); its direct label is placed at save time."""
    line = ax.axhline(value, color=color, linewidth=0.9, linestyle=DASH, zorder=1)
    if text:
        ax.quad_targets = getattr(ax, 'quad_targets', []) + [(value, text, below)]
    return line


def bottom_legend(fig, ax, ncol, handles=None, labels=None):
    """Legend below the plot, for charts whose data fill the axes."""
    if handles is None:
        handles, labels = ax.get_legend_handles_labels()
    fig.legend(handles, labels, loc='outside lower center', ncol=ncol)


def shade_stance(ax, run, t0, t1):
    """Light bands where any leg is in stance."""
    m = (run.t >= t0) & (run.t <= t1)
    stance = (run.back | run.front)[m]
    tt = run.t[m]
    edges = np.flatnonzero(np.diff(stance.astype(int)))
    starts = [0] + list(edges + 1)
    for i, s0 in enumerate(starts):
        s1 = (starts[i + 1] - 1) if i + 1 < len(starts) else len(tt) - 1
        if stance[s0]:
            ax.axvspan(tt[s0], tt[s1], color=BAND, linewidth=0, zorder=0)


def bars(mask, t):
    """(start, width) runs where mask is true."""
    edges = np.flatnonzero(np.diff(np.r_[0, mask.astype(int), 0]))
    return [(t[a], t[min(b, len(t) - 1)] - t[a]) for a, b in zip(edges[::2], edges[1::2])]


# ----------------------------------------------------------------------------- per-run figures
def fig_forward_velocity(run, folder):
    fig, ax = plt.subplots(figsize=(7.5, 3.0))
    header(fig, 'Forward velocity')
    ax.set_title('%s, desired %.1f m/s' % (run.label, run.xddes))
    ax.plot(run.t, run.xd, color=BLUE, linewidth=0.9, label='body velocity (physics plugin, 1 kHz)')
    ax.plot(run.t_lo, run.xdlo, 'o', color=ORANGE, markersize=3.2, markeredgewidth=0,
            label='liftoff speed (controller input)')
    ax.set_xlim(0, run.t[-1])
    target(ax, run.xddes, 'desired %.1f m/s' % run.xddes)
    ax.set_xlabel('time (s)')
    ax.set_ylabel('forward velocity (m/s)')
    ax.legend(loc='lower right')
    save(fig, folder, 'forward_velocity')


def fig_com_height(run, folder):
    fig, (a1, a2) = plt.subplots(2, 1, figsize=(7.5, 4.8), gridspec_kw={'height_ratios': [1, 1.1]})
    header(fig, 'Body (CoM) height')
    a1.set_title('%s: whole run' % run.label)
    a1.plot(run.t, run.y, color=BLUE, linewidth=0.8)
    a1.set_xlim(0, run.t[-1])
    target(a1, run.h, 'desired apex %.2f m' % run.h)
    a1.set_ylabel('height (m)')
    t0, t1 = run.window()
    m = (run.t >= t0) & (run.t <= t1)
    a2.set_title('steady state, %.1f to %.1f s (shaded: a leg pair in stance)' % (t0, t1))
    shade_stance(a2, run, t0, t1)
    a2.plot(run.t[m], run.y[m], color=BLUE)
    a2.set_xlim(t0, t1)
    target(a2, run.h, 'desired apex %.2f m' % run.h)
    a2.set_xlabel('time (s)')
    a2.set_ylabel('height (m)')
    save(fig, folder, 'com_height')


def fig_pitch(run, folder):
    fig, (a1, a2) = plt.subplots(2, 1, figsize=(7.5, 4.6), sharex=True)
    header(fig, 'Body pitch')
    a1.set_title('%s (controller sign convention)' % run.label)
    a1.plot(run.t, run.th, color=BLUE, linewidth=0.8)
    a1.set_ylabel('pitch angle (rad)')
    a2.plot(run.t, run.thd, color=BLUE, linewidth=0.8)
    a2.set_ylabel('pitch rate (rad/s)')
    a2.set_xlabel('time (s)')
    a2.set_xlim(0, run.t[-1])
    save(fig, folder, 'pitch')


def fig_phase_planes(run, folder):
    fig, (a1, a2) = plt.subplots(1, 2, figsize=(7.5, 3.6))
    header(fig, 'Phase planes')
    early = run.t <= 3.0
    late = run.t >= run.t[-1] - 3.0
    for ax, (u, v, xl, yl, title) in zip((a1, a2), ((run.y, run.yd, 'height (m)', 'vertical velocity (m/s)', 'vertical motion'),
                                                    (run.th, run.thd, 'pitch angle (rad)', 'pitch rate (rad/s)', 'pitch'))):
        ax.set_title(('%s: %s' % (run.label, title)) if ax is a1 else title)
        ax.plot(u[early], v[early], color=MUTED, linewidth=0.7, label='first 3 s (transient)')
        ax.plot(u[late], v[late], color=BLUE, linewidth=1.2, label='last 3 s (steady state)')
        lo = run.lo_idx[run.t[run.lo_idx] >= run.t[-1] - 3.0]
        ax.plot(u[lo], v[lo], 'o', color=ORANGE, markersize=3.5, markeredgewidth=0, label='liftoff')
        ax.set_xlabel(xl)
        ax.set_ylabel(yl)
    bottom_legend(fig, a1, 3)
    save(fig, folder, 'phase_planes')


def fig_gait_diagram(run, folder):
    t0, t1 = run.window()
    m = (run.t >= t0) & (run.t <= t1)
    fig, (a1, a2) = plt.subplots(2, 1, figsize=(7.5, 4.2), sharex=True, gridspec_kw={'height_ratios': [2.2, 1]})
    header(fig, 'Leg springs and gait')
    a1.set_title('%s, steady state' % run.label)
    a1.plot(run.t[m], 1000 * run.spring[0][m], color=BACK, label='back legs (1, 4)')
    a1.plot(run.t[m], 1000 * run.spring[1][m], color=FRONT, label='front legs (2, 3)')
    target(a1, 1000 * run.tol).set_label('touchdown threshold (%g mm)' % (1000 * run.tol))
    a1.set_ylabel('spring compression (mm)')
    bottom_legend(fig, a1, 3)
    for row, (mask, color) in enumerate(((run.back[m], BACK), (run.front[m], FRONT))):
        a2.broken_barh(bars(mask, run.t[m]), (1 - row - 0.32, 0.64), facecolors=color, linewidth=0)
    a2.set_yticks([1, 0], ['back', 'front'])
    a2.set_ylim(-0.6, 1.6)
    a2.grid(axis='y', visible=False)
    a2.set_title('stance (bars) and flight (gaps)')
    a2.set_xlabel('time (s)')
    a2.set_xlim(t0, t1)
    save(fig, folder, 'gait_diagram')


def fig_leg_angles(run, folder):
    t0, t1 = run.window()
    m = (run.t >= t0) & (run.t <= t1)
    fig, ax = plt.subplots(figsize=(7.5, 3.2))
    header(fig, 'Hip joint angles')
    ax.set_title('%s, steady state (shaded: stance); positive = foot ahead of the hip' % run.label)
    shade_stance(ax, run, t0, t1)
    ax.plot(run.t[m], run.gamb[m], color=BACK, label='back legs')
    ax.plot(run.t[m], run.gamf[m], color=FRONT, label='front legs')
    lo = (run.t_lo >= t0) & (run.t_lo <= t1)
    ax.plot(run.t_lo[lo], run.gambtd[lo], 'D', color=BACK, markersize=4, markeredgewidth=0, label='touchdown setpoint, back')
    ax.plot(run.t_lo[lo], run.gamftd[lo], 'D', color=FRONT, markersize=4, markeredgewidth=0, label='touchdown setpoint, front')
    ax.set_xlim(t0, t1)
    ax.set_xlabel('time (s)')
    ax.set_ylabel('joint angle (rad)')
    bottom_legend(fig, ax, 4)
    save(fig, folder, 'leg_angles')


def fig_per_gait(run, folder):
    n = np.arange(len(run.t_lo))
    ap = run.apexes()
    fig, axes = plt.subplots(4, 1, figsize=(7.5, 7.2), sharex=True)
    header(fig, 'Gait-to-gait convergence')
    a1, a2, a3, a4 = axes
    a1.set_title('%s: one point per liftoff' % run.label)
    a1.plot(n, run.xdlo, '-o', color=BLUE, markersize=2.8, linewidth=1.0)
    target(a1, run.xddes, 'desired %.1f m/s' % run.xddes)
    a1.set_ylabel('liftoff speed\n(m/s)')
    a2.plot(n[:-1], ap, '-o', color=BLUE, markersize=2.8, linewidth=1.0)
    target(a2, run.h, 'desired %.2f m' % run.h)
    a2.set_ylabel('apex height\n(m)')
    a2.yaxis.set_major_formatter(matplotlib.ticker.FormatStrFormatter('%.3f'))
    a3.plot(n, run.gambtd, '-o', color=BACK, markersize=2.8, linewidth=1.0, label='back legs')
    a3.plot(n, run.gamftd, '-o', color=FRONT, markersize=2.8, linewidth=1.0, label='front legs')
    a3.set_ylabel('touchdown angle\n(rad)')
    a3.legend(loc='lower right', ncol=2)
    a4.plot(n, run.tor, '-o', color=BLUE, markersize=2.8, linewidth=1.0)
    if np.nanmax(np.abs(run.tor)) > 0.8 * MAX_TORQUE:
        target(a4, MAX_TORQUE, 'motor limit %.1f N m' % MAX_TORQUE)
    a4.set_ylabel('stance torque\nper motor (N m)')
    a4.set_xlabel('gait number')
    a4.set_xlim(-1, len(n))
    save(fig, folder, 'per_gait')


def fig_com_path(run, folder):
    fig, ax = plt.subplots(figsize=(7.5, 2.6))
    header(fig, 'Body (CoM) path, side view')
    stop = min(len(run.t) - 1, np.searchsorted(run.x, 4.0))
    ax.set_title('%s: first 4 m (vertical axis exaggerated)' % run.label)
    ax.plot(run.x[:stop], run.y[:stop], color=BLUE, linewidth=1.0)
    lo = run.lo_idx[run.lo_idx < stop]
    ax.plot(run.x[lo], run.y[lo], 'o', color=ORANGE, markersize=3.2, markeredgewidth=0, label='liftoff')
    target(ax, run.h, 'desired apex %.2f m' % run.h)
    ax.set_xlabel('forward distance (m)')
    ax.set_ylabel('height (m)')
    ax.set_xlim(0, run.x[stop])
    bottom_legend(fig, ax, 1)
    save(fig, folder, 'com_path')


# ----------------------------------------------------------------------------- comparisons
def fig_speed_tracking(runs):
    fig, ax = plt.subplots(figsize=(7.5, 3.4))
    header(fig, 'Speed tracking')
    ax.set_title('Oct 2014 controller, only the desired speed xddes changed; liftoff speed per gait')
    t_end = 0
    for run, color in zip(runs, (AQUA, BLUE, ORANGE)):
        ax.plot(run.t_lo, run.xdlo, '-o', color=color, markersize=2.6, linewidth=1.0,
                label='desired %.1f m/s, steady %.2f m/s' % (run.xddes, np.mean(run.xdlo[len(run.xdlo) // 2:])))
        target(ax, run.xddes, color=color)
        t_end = max(t_end, run.t_lo[-1])
    ax.set_xlim(0, t_end)
    ax.set_xlabel('time (s)')
    ax.set_ylabel('liftoff speed (m/s)')
    handles, labels = ax.get_legend_handles_labels()
    handles.append(matplotlib.lines.Line2D([], [], color=TEXT2, linewidth=0.9, linestyle=DASH))
    labels.append('dashed: desired speed, same colour')
    bottom_legend(fig, ax, 2, handles, labels)
    save(fig, 'comparisons', 'speed_tracking')


def fig_before_after(before, after):
    fig, (a1, a2) = plt.subplots(2, 1, figsize=(7.5, 4.8), sharex=True)
    header(fig, 'Effect of the R2025a tuning (world CFM 1e-10 to 1e-5)')
    a1.set_title('Oct 2014 world: as converted versus tuned')
    a1.plot(before.t, before.xd, color=MUTED, linewidth=0.8, label='as converted')
    a1.plot(after.t, after.xd, color=BLUE, linewidth=0.8, label='tuned')
    target(a1, after.xddes, 'desired %.1f m/s' % after.xddes)
    a1.set_ylabel('forward velocity (m/s)')
    bottom_legend(fig, a1, 2)
    a2.plot(before.t, before.y, color=MUTED, linewidth=0.8, label='as converted')
    a2.plot(after.t, after.y, color=BLUE, linewidth=0.8, label='tuned')
    a2.set_ylabel('height (m)')
    a2.set_xlabel('time (s)')
    a2.set_xlim(0, min(before.t[-1], after.t[-1]))
    save(fig, 'comparisons', 'before_after_tuning')


def fig_engines(pairs):
    """Same controller file on Webots R2025a (tuned world) and on the MuJoCo model."""
    fig, axes = plt.subplots(2, len(pairs), figsize=(4.9 * len(pairs), 6.0), squeeze=False)
    header(fig, 'Webots R2025a versus MuJoCo 4.0, same controller code')
    for col, (wb, mj, title) in enumerate(pairs):
        a, b = axes[0, col], axes[1, col]
        a.set_title('%s: liftoff speed per gait' % title)
        a.plot(wb.t_lo, wb.xdlo, '-o', color=BLUE, markersize=2.4, linewidth=0.9, label='Webots R2025a (tuned world)')
        a.plot(mj.t_lo, mj.xdlo, '-o', color=AQUA, markersize=2.4, linewidth=0.9, label='MuJoCo 4.0')
        a.set_xlim(0, min(wb.t_lo[-1], mj.t_lo[-1]))
        target(a, wb.xddes, 'desired %.1f m/s' % wb.xddes)
        a.set_xlabel('time (s)')
        a.set_ylabel('liftoff speed (m/s)')
        b.set_title('vertical phase plane, last 3 s')
        for run, color in ((wb, BLUE), (mj, AQUA)):
            late = run.t >= run.t[-1] - 3.0
            b.plot(run.y[late], run.yd[late], color=color, linewidth=1.0)
        b.set_xlabel('height (m)')
        b.set_ylabel('vertical velocity (m/s)')
    bottom_legend(fig, axes[0, 0], 2)
    save(fig, 'comparisons', 'webots_vs_mujoco')


def main():
    loaded = {name: Run(name) for name in RUNS if glob.glob(os.path.join(DATA, name, '*_steps.csv'))}
    for name in ('oct2014', 'feb2013', 'mujoco_oct2014', 'mujoco_feb2013'):
        if name in loaded:
            run = loaded[name]
            for fig in (fig_forward_velocity, fig_com_height, fig_pitch, fig_phase_planes, fig_gait_diagram,
                        fig_leg_angles, fig_per_gait, fig_com_path):
                fig(run, name)
    sweep = [loaded[n] for n in ('oct2014_xddes0.6', 'oct2014', 'oct2014_xddes1.1') if n in loaded]
    if len(sweep) == 3:
        fig_speed_tracking(sweep)
    if 'oct2014_as_converted' in loaded and 'oct2014' in loaded:
        fig_before_after(loaded['oct2014_as_converted'], loaded['oct2014'])
    pairs = [(loaded[w], loaded[m], t) for w, m, t in (('oct2014', 'mujoco_oct2014', 'Oct 2014 controller'),
                                                      ('feb2013', 'mujoco_feb2013', 'Feb 2013 thesis version'))
             if w in loaded and m in loaded]
    if pairs:
        fig_engines(pairs)


if __name__ == '__main__':
    main()
