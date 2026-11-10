#!/usr/bin/env python3
"""Write the *_tuned.wbt copies of the converted worlds (README, "Tuned worlds").

Each tuned world is its converted twin with one documented edit:
  cfm        WorldInfo CFM 1e-10 -> 1e-5 (the Webots default). Joints stay rigid (about 10 um under
             100 N at ERP 0.1 and a 1 ms step); it only keeps the constraint solver well conditioned
             around the 0.1 g touch-sensor bodies at the feet, which made R2025a lose the gait.
  springs_up leg springs 2-4 along +y like leg 1 and every 2014 world. In this Feb 2013 file they
             point down, so ground contact pushed them into their 0 stop and the legs were rigid.
Usage: make_tuned_worlds.py   (run from anywhere; paths are relative to the R2025a folder)
"""
import os
import re

ROOT = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
TUNED = [
    ('Webots_FINAL_2/worlds/SUPERVISOR_webots743_Oct.wbt', 'cfm'),
    ('Webots_FINAL_2/worlds/SUPERVISOR_webots743_May_(flat).wbt', 'cfm'),
    ('Webots_FINAL/worlds/SUPERVISOR_webots730_Feb_FINAL.wbt', 'springs_up'),
]
NOTES = {
    'cfm': 'WorldInfo CFM 1e-10 -> 1e-5 (Webots default): keeps the solver well conditioned; joints stay rigid',
    'springs_up': 'leg springs 2-4 axis 0 -1 0 -> 0 1 0, as leg 1 and every 2014 world (they were locked on their stop)',
}


def edit(text, pattern, repl, count):
    new, n = re.subn(pattern, repl, text, flags=re.M)
    if n != count:
        raise SystemExit('%r matched %d times, expected %d' % (pattern, n, count))
    return new


for rel, kind in TUNED:
    src = os.path.join(ROOT, rel)
    text = open(src).read()
    if kind == 'cfm':
        text = edit(text, r'^  CFM 1e-10$', '  CFM 1e-05', 1)
    else:
        text = edit(text, r'axis 0 -1 0(\n\s+maxStop 0\.035)', r'axis 0 1 0\1', 3)
    header, rest = text.split('\n', 1)
    text = header + '\n# Tuned copy of %s: %s (tools/make_tuned_worlds.py)\n' % (os.path.basename(rel), NOTES[kind]) + rest
    dst = src[:-4] + '_tuned.wbt'
    open(dst, 'w').write(text)
    print('wrote', os.path.relpath(dst, ROOT))
