# Plots

Figures drawn from the controllers' own logs (the MATLAB `W*p` arrays, one row per 1 ms control
step), recorded in Webots R2025a with the tuned worlds. Each figure comes as a PNG (200 dpi, for
slides) and a PDF (vector, for LaTeX).

| Folder | Run |
|---|---|
| `oct2014/` | `SUPERVISOR_webots743_Oct_tuned.wbt`, `controller_webots743_Oct_THALIS`, desired speed 0.9 m/s |
| `feb2013/` | `SUPERVISOR_webots730_Feb_FINAL_tuned.wbt` (thesis version), desired speed 0.7 m/s |
| `mujoco_oct2014/`, `mujoco_feb2013/` | the same two controllers on the MuJoCo 4.0 model (`../mujoco`) |
| `comparisons/` | speed tracking for three desired speeds; the Oct world as converted versus tuned; Webots versus MuJoCo |
| `data/` | the CSV logs behind every figure, one folder per run, with that run's console log (written by `collect_plot_data.py`; not in the repository) |

The logs stop at the controller's own stop condition (gait 101, about 24 s), as the MATLAB did.

## Figures in each run folder

- `forward_velocity`: body velocity at 1 kHz and the liftoff speed the controller uses, against the
  desired speed.
- `com_height`: body height over the whole run, plus a steady-state close-up with stance shaded,
  against the desired apex height.
- `pitch`: pitch angle and pitch rate.
- `phase_planes`: height against vertical velocity, and pitch angle against pitch rate. Each shows
  the first 3 s against the last 3 s, with the liftoff states marked.
- `gait_diagram`: spring compression and stance bars for the back and front leg pairs.
- `leg_angles`: hip joint angles, with the touchdown setpoints computed at each liftoff.
- `per_gait`: liftoff speed, apex height, touchdown angles and stance torque, one point per gait.
- `com_path`: side view of the body path over the first 4 m.

Back legs are orange and front legs blue, as on the showcase robot. Dashed lines are targets.
"CoM" is the body centre (the controllers' `CoM_Gps`, on the 10.5 kg body; the legs add 0.41 kg).
The forward distance in `com_path` is integrated from the body velocity. Pitch keeps the
controller's sign convention.

## Steady state (second half of each run)

| Run | Liftoff speed | Apex height | Stance torque per motor | Gait period | Back legs land first by |
|---|---|---|---|---|---|
| Oct 2014, desired 0.9 m/s | 0.780 ± 0.009 m/s | 0.320 m | 1.88 N m | 0.242 s | 13 ms |
| Oct 2014, desired 0.6 m/s | 0.506 ± 0.008 m/s | 0.318 m | 2.31 N m | 0.239 s | 12 ms |
| Oct 2014, desired 1.1 m/s | 0.923 ± 0.004 m/s | 0.322 m | 2.08 N m | 0.249 s | 25 ms |
| Feb 2013, desired 0.7 m/s | 0.665 ± 0.011 m/s | 0.317 m | 1.78 N m | 0.235 s | 4 ms |

On MuJoCo the same controllers reach 0.834 ± 0.004 m/s (Oct) and 0.675 ± 0.008 m/s (Feb), with apex
heights of 0.320 m and 0.319 m (`comparisons/webots_vs_mujoco`).

Apex height reaches its 0.32 m target within about three gaits. Speed settles 5–16 % under the
target, the steady offset of the controller in this simulation. Pitch stays within ±0.04 rad.

## Regenerating

```bash
python3 tools/collect_plot_data.py                           # the 5 Webots + 2 MuJoCo runs; about 1-2 min
~/.local/opt/quadsim-venv/bin/python tools/make_plots.py     # the figures (needs matplotlib, numpy)
```

`collect_plot_data.py` also takes run names (`oct2014`, `feb2013`, `oct2014_xddes0.6`,
`oct2014_xddes1.1`, `oct2014_as_converted`, `mujoco_oct2014`, `mujoco_feb2013`). The speed sweep edits `xddes` in a temporary copy of
the project, so the controller files here never change.
