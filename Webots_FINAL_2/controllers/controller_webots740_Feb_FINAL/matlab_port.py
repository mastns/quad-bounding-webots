"""Small helpers that stand in for MATLAB behaviour the original controllers relied on."""
import csv
import datetime
import math
import os
import re

# What to do when the MATLAB code reached `desktop; keyboard;` (stop time or gait count).
# pause (default): write the logs, pause the simulation once; press Play to keep going.
# quit: write the logs and close Webots (handy for batch runs). continue: write the logs only.
AT_STOP = os.environ.get('QUADSIM_AT_STOP', 'pause')


def set_force_webots7(motor, torque):
    """wb_motor_set_force as Webots 7.4 applied it on these hip joints.

    Up to Webots 8.5, a torque-controlled HingeJoint whose parent and endPoint both have a
    Physics node turned the wrong way; 8.6.0 fixed it ("Fixed HingeJoint and Hinge2Joint
    rotation direction in case of motor torque control and if the parent and endPoint Solid
    nodes have a Physics node"). The MATLAB controllers were tuned against the old behaviour
    (their "% Match Webots Positive direction" sign flip), so the ports keep that direction.
    """
    motor.setForce(-torque)


def radtodeg(x):
    return x * 180.0 / math.pi


def rotz(degrees):
    """MATLAB rotz(): 3x3 rotation about z, angle in degrees (list of rows)."""
    c, s = math.cos(math.radians(degrees)), math.sin(math.radians(degrees))
    return [[c, -s, 0.0], [s, c, 0.0], [0.0, 0.0, 1.0]]


def matmul(a, b):
    """Matrix product of row lists; b may be a column vector given as a flat list."""
    if not isinstance(b[0], list):
        return [sum(a[i][k] * b[k] for k in range(len(b))) for i in range(len(a))]
    return [[sum(a[i][k] * b[k][j] for k in range(len(b))) for j in range(len(b[0]))] for i in range(len(a))]


def cross(u, v):
    return [u[1] * v[2] - u[2] * v[1], u[2] * v[0] - u[0] * v[2], u[0] * v[1] - u[1] * v[0]]


def norm(u):
    return math.sqrt(sum(x * x for x in u))


# Supervisor labels (QUADSIM_LABELS): clean (default) redraws the controllers' labels as a small HUD
# with a fixed, colour-coded phase line and aligned values with units; original draws them as the
# MATLAB did; off hides them.
LABELS = os.environ.get('QUADSIM_LABELS', 'clean')
_PHASES = {'-Flight-': ('FLIGHT', 0x1565C0), '-Double Stance-': ('DOUBLE STANCE', 0x2E7D32),
           '-Back_Stance-': ('BACK STANCE', 0xD84315), '-Front_Stance-': ('FRONT STANCE', 0x6A1B9A)}
_ROWS = {11: 0, 22: 1, 33: 2, 43: 3, 53: 4, 63: 5, 73: 6, 83: 7}
_UNITS = {'gambtd': 'rad', 'gamftd': 'rad', 'torbctr': 'N m', 'tq': 'N m', 'xdlo': 'm/s', 'ylo': 'm',
          'ydlo': 'm/s', 'thlo': 'rad', 'thdlo': 'rad/s'}


def set_label(robot, id, text, x, y, size, color, transparency=0.0):
    """robot.setLabel() of the MATLAB code, drawn in the QUADSIM_LABELS style."""
    if LABELS == 'off':
        return
    if LABELS == 'original':
        robot.setLabel(id, text, x, y, size, color, transparency)
        return
    if id == 1:
        name, col = _PHASES.get(text, (text.strip('-').replace('_', ' ').upper(), 0x37474F))
        robot.setLabel(1, name, 0.012, 0.012, 0.072, col, 0.0, 'Arial Black')
        return
    m = re.match(r'^\s*([A-Za-z_]\w*)\s*=\s*(\S+)\s*$', text)
    name, value = m.groups() if m else ('torbctr' if id == 33 else 'value', text.strip())  # label 33 showed a bare number
    try:
        value = '%+.3f' % float(value)
    except ValueError:
        pass
    line = '%-8s%s %s' % (name, value, _UNITS.get(name, ''))
    robot.setLabel(id, line, 0.012, 0.058 + 0.032 * _ROWS.get(id, 8), 0.052, 0x263238, 0.0, 'Lucida Console')


def rgb(color):
    """MATLAB label colour [r g b] (0..1) -> Webots 0xRRGGBB."""
    r, g, b = (max(0, min(255, int(round(c * 255)))) for c in color)
    return (r << 16) | (g << 8) | b


def num2str(x):
    """Close to MATLAB num2str for a scalar double (about 5 significant digits)."""
    return '%.5g' % x


def strcat(*parts):
    """MATLAB strcat drops trailing blanks of each char-array argument."""
    return ''.join(p.rstrip(' \t') for p in parts)


def all_equal(values, scalar):
    """MATLAB `if A == scalar` on a vector: true only if A is non-empty and every element matches."""
    return len(values) > 0 and all(v == scalar for v in values)


def save_logs(controller_name, series, per_gait):
    """Write the W*p arrays the MATLAB workspace kept: one CSV per time step, one per gait."""
    log_dir = os.environ.get('QUADSIM_LOG_DIR', 'data')  # default: controllers/<name>/data
    os.makedirs(log_dir, exist_ok=True)
    stamp = datetime.datetime.now().strftime('%Y%m%d_%H%M%S')
    paths = []
    # per-gait table first: it is small, and Webots gives a closing controller only 1 s
    for suffix, table in (('gaits', per_gait), ('steps', series)):
        if not table:
            continue
        path = os.path.join(log_dir, '%s_%s_%s.csv' % (controller_name, stamp, suffix))
        names = list(table)
        rows = max(len(v) for v in table.values())
        with open(path, 'w', newline='') as f:
            w = csv.writer(f)
            w.writerow(names)
            for i in range(rows):
                w.writerow([table[n][i] if i < len(table[n]) else '' for n in names])
        paths.append(os.path.abspath(path))
    for p in paths:
        print('logged data written to ' + p)
    return paths


def at_stop(robot):
    if AT_STOP == 'quit':
        robot.simulationQuit(0)
    elif AT_STOP == 'pause':
        robot.simulationSetMode(robot.SIMULATION_MODE_PAUSE)


# MATLAB `keyboard` inside the control loop: pause | ignore (QUADSIM_KEYBOARD).
KEYBOARD = os.environ.get('QUADSIM_KEYBOARD', 'pause')


def keyboard(robot):
    """Stand-in for MATLAB `keyboard`: pause the simulation; press Step or Play to go on."""
    if KEYBOARD == 'pause':
        robot.simulationSetMode(robot.SIMULATION_MODE_PAUSE)


def optional_device(robot, name, sampling_period=None):
    """robot.getDevice() for a device some of the worlds lack. MATLAB would have stopped on the
    missing device; the port returns None so the rest of the controller still runs."""
    names = {robot.getDeviceByIndex(i).getName() for i in range(robot.getNumberOfDevices())}
    if name not in names:
        print('note: this world has no device "%s"; its readings are left out' % name)
        return None
    device = robot.getDevice(name)
    if sampling_period is not None:
        device.enable(sampling_period)
    return device


class _MissingPositionSensor:
    """Reads NaN, like a Webots 7 MATLAB read on a device that does not exist: every
    `pos > tol` / `pos <= tol` test on it is false."""

    def getValue(self):
        return math.nan


def position_sensor(motor, sampling_period):
    """The joint PositionSensor of `motor` (enabled), or a NaN reader when the motor is missing."""
    if motor is None:
        return _MissingPositionSensor()
    sensor = motor.getPositionSensor()
    sensor.enable(sampling_period)
    return sensor


def values_or_zeros(device, n=3):
    """getValues() as a list of n floats, zeros for a missing device. Indexes explicitly: in R2025a
    TouchSensor.getValues() returns a bare ctypes pointer, and list() on it reads past the end."""
    if device is None:
        return [0.0] * n
    v = device.getValues()
    return [v[i] for i in range(n)]


def contact_point(node, index):
    """wb_supervisor_node_get_contact_point(node, index) of Webots 7: [x y z] or NaNs."""
    points = node.getContactPoints() if node is not None else []
    return list(points[index].point) if index < len(points) else [math.nan] * 3
