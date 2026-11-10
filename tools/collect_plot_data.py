#!/usr/bin/env python3
"""Run the simulations behind the figures in plots/ and keep the controllers' CSV logs.

Webots runs and, with the same controller files, MuJoCo runs (mujoco/run_controller.py).

Each run goes headless through run_headless.sh (fast mode, no rendering) with the controller logging
to plots/data/<run>/. The speed sweep changes xddes in a temporary copy of the project, so the
controller files in this folder are never edited.

usage: collect_plot_data.py [RUN ...]      (default: all runs below)
"""
import os
import re
import shutil
import subprocess
import sys
import tempfile
import time

ROOT = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
OCT = 'Webots_FINAL_2/worlds/SUPERVISOR_webots743_Oct_tuned.wbt'
RUNS = {
    # name: (world, seconds, xddes override or None)
    'oct2014': (OCT, 30, None),
    'feb2013': ('Webots_FINAL/worlds/SUPERVISOR_webots730_Feb_FINAL_tuned.wbt', 30, None),
    'oct2014_xddes0.6': (OCT, 30, 0.6),
    'oct2014_xddes1.1': (OCT, 30, 1.1),
    'oct2014_as_converted': ('Webots_FINAL_2/worlds/SUPERVISOR_webots743_Oct.wbt', 30, None),
}
# the same controllers on the MuJoCo model (mujoco/run_controller.py); needs mujoco + numpy
MUJOCO_RUNS = {
    'mujoco_oct2014': ['--controller', 'Webots_FINAL_2/controllers/controller_webots743_Oct_THALIS', '--seconds', '30'],
    'mujoco_feb2013': ['--controller', 'Webots_FINAL/controllers/controller_webots730_Feb_FINAL', '--seconds', '30',
                       '--start-height', '0.34', '--start-rotation', '-0.01'],
}
PYTHON = os.environ.get('QUADSIM_PYTHON', os.path.expanduser('~/.local/opt/quadsim-venv/bin/python'))


def start(name):
    world, seconds, xddes = RUNS[name]
    out = os.path.join(ROOT, 'plots', 'data', name)
    os.makedirs(out, exist_ok=True)
    for f in os.listdir(out):
        if f.endswith('.csv'):
            os.remove(os.path.join(out, f))
    root, tmp = ROOT, None
    if xddes is not None:   # copy the project and change xddes in the copy's controller options
        tmp = tempfile.mkdtemp(prefix='quadsim_%s_' % name)
        project = world.split('/')[0]
        shutil.copytree(os.path.join(ROOT, project), os.path.join(tmp, project), symlinks=True,
                        ignore=shutil.ignore_patterns('data', '*.mp4'))
        shutil.copytree(os.path.join(ROOT, 'tools'), os.path.join(tmp, 'tools'))
        shutil.copy(os.path.join(ROOT, 'run_headless.sh'), tmp)
        opts = os.path.join(tmp, project, 'controllers', 'controller_webots743_Oct_THALIS', 'controller_options.py')
        text, n = re.subn(r'^xddes = [0-9.]+', 'xddes = %s' % xddes, open(opts).read(), flags=re.M)
        assert n == 1, opts
        open(opts, 'w').write(text)
        root = tmp
    env = dict(os.environ, QUADSIM_LOG_DIR=out, QUADSIM_AT_STOP='continue', QUADSIM_LABELS='off')
    log = open(os.path.join(out, 'webots.log'), 'w')
    proc = subprocess.Popen(['nice', '-n', '12', os.path.join(root, 'run_headless.sh'),
                             os.path.join(root, world), str(seconds)], env=env, stdout=log, stderr=subprocess.STDOUT)
    return proc, tmp, out


def run_mujoco(name):
    out = os.path.join(ROOT, 'plots', 'data', name)
    os.makedirs(out, exist_ok=True)
    for f in os.listdir(out):
        if f.endswith('.csv'):
            os.remove(os.path.join(out, f))
    args = [a if not a.startswith('Webots_') else os.path.join(ROOT, a) for a in MUJOCO_RUNS[name]]
    env = dict(os.environ, QUADSIM_LABELS='off')
    with open(os.path.join(out, 'mujoco.log'), 'w') as log:
        rc = subprocess.call([PYTHON, os.path.join(ROOT, 'mujoco', 'run_controller.py'), '--log-dir', out] + args,
                             env=env, stdout=log, stderr=subprocess.STDOUT)
    logs = sorted(f for f in os.listdir(out) if f.endswith('.csv'))
    print('%-22s exit %d, logs: %s' % (name, rc, ', '.join(logs) or 'NONE'))


def main():
    names = sys.argv[1:] or list(RUNS) + list(MUJOCO_RUNS)
    for name in [n for n in names if n in MUJOCO_RUNS]:
        run_mujoco(name)
    names = [n for n in names if n in RUNS]
    running = []
    for name in names:
        running.append((name,) + start(name))
        time.sleep(4)        # let each run claim its own Xvfb display
    for name, proc, tmp, out in running:
        proc.wait()
        if tmp:
            shutil.rmtree(tmp, ignore_errors=True)
        logs = sorted(f for f in os.listdir(out) if f.endswith('.csv'))
        print('%-22s exit %d, logs: %s' % (name, proc.returncode, ', '.join(logs) or 'NONE'))


if __name__ == '__main__':
    main()
