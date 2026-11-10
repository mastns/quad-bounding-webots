"""The part of the Webots controller API that the ported quadruped controllers use, on MuJoCo.

run_controller.py builds a World (MuJoCo model + data) and then runs an unchanged controller file;
its `from controller import Supervisor` lands here. Every reading is converted to the Webots frames
the controllers were written for:
    Webots x = -X,  y = Z,  z = Y      (MuJoCo X forward, Y left, Z up; R below, its own inverse)
The Webots physics plugin is reproduced in World.physics_step: the 4 ms push (-1101 N along the body
x axis) and, every step, the three packets sent to the robot's receiver (body velocity [-vx, -vy],
the 1234 marker, and the body rotation matrix).
"""
import math
import struct

import mujoco
import numpy as np

R = np.array([[-1.0, 0, 0], [0, 0, 1.0], [0, 1.0, 0]])     # MuJoCo <-> Webots, R @ R = identity


class World:
    current = None

    def __init__(self, model, data, seconds, push_force=-1101.0, push_ms=4, basic_time_step_ms=1, on_step=None):
        self.m, self.d = model, data
        self.dt_ms = basic_time_step_ms            # Webots basicTimeStep: controller and plugin period
        self.substeps = max(1, int(round(basic_time_step_ms / (model.opt.timestep * 1000.0))))
        self.end_time = seconds
        self.push_force, self.push_ms = push_force, push_ms
        self.on_step = on_step or []
        self.steps = 0
        self.quit = False
        self.motors = []
        self.receivers = []
        self.labels = {}
        self.torso = model.body('torso').id
        World.current = self

    # -- Webots physics plugin (physics_webots743_May_2.c), called once per basic time step
    def plugin(self):
        d, m = self.d, self.m
        if self.steps * self.dt_ms < self.push_ms:   # dBodyAddRelForce(body, xForce, 0, 0) for 4 ms
            body_x = R @ np.array([1.0, 0, 0])                 # Webots body x axis in MuJoCo body axes
            d.xfrc_applied[self.torso, :3] = d.xmat[self.torso].reshape(3, 3) @ (self.push_force * body_x)
        else:
            d.xfrc_applied[self.torso, :3] = 0.0
        vx = d.qvel[m.joint('rootx').dofadr[0]]
        vz = d.qvel[m.joint('rootz').dofadr[0]]
        v_webots = R @ np.array([vx, 0.0, vz])                   # world velocity in Webots axes
        rot = R @ d.xmat[self.torso].reshape(3, 3) @ R          # body rotation in Webots axes
        packets = [struct.pack('2d', -v_webots[0], -v_webots[1]), struct.pack('d', 1234.0),
                   struct.pack('9d', *rot.reshape(-1))]
        for r in self.receivers:
            if r.sampling:
                r.queue.extend(packets)

    def physics_step(self):
        """One Webots basic time step: the plugin runs once, MuJoCo runs `substeps` finer steps."""
        self.plugin()
        for _ in range(self.substeps):
            for motor in self.motors:
                motor.apply()
            mujoco.mj_step(self.m, self.d)
        self.steps += 1
        for hook in self.on_step:
            hook(self)

    def time(self):
        return self.d.time


# ----------------------------------------------------------------------------- devices
class Device:
    def __init__(self, world, name):
        self.world, self.name, self.sampling = world, name, 0

    def getName(self):
        return self.name

    def enable(self, sampling_period):
        self.sampling = sampling_period

    def disable(self):
        self.sampling = 0

    def getSamplingPeriod(self):
        return self.sampling


class PositionSensor(Device):
    def __init__(self, world, name, joint):
        super().__init__(world, name)
        self.adr = world.m.joint(joint).qposadr[0]

    def getValue(self):
        return float(self.world.d.qpos[self.adr])


class Motor(Device):
    """Webots RotationalMotor/LinearMotor: position control (P velocity servo) or force control."""

    def __init__(self, world, name, joint, p_gain=60.0, max_velocity=12.0, min_position=-0.435,
                 max_position=0.435, max_force=6.5, actuated=True):
        super().__init__(world, name)
        self.sensor = PositionSensor(world, name + '_sensor', joint)
        self.qadr, self.vadr = world.m.joint(joint).qposadr[0], world.m.joint(joint).dofadr[0]
        self.p, self.vmax, self.lo, self.hi, self.fmax = p_gain, max_velocity, min_position, max_position, max_force
        self.actuated = actuated
        self.mode, self.target, self.force = 'position', 0.0, 0.0
        self.clamped_positions = self.clamped_forces = 0
        if actuated:
            self.vel = world.m.actuator(name.replace('servo', 'hip').replace('_rot', '') + '_vel').id
            self.trq = world.m.actuator(name.replace('servo', 'hip').replace('_rot', '') + '_torque').id
            self.kv = float(world.m.actuator_gainprm[self.vel, 0])
            world.motors.append(self)

    def getPositionSensor(self):
        return self.sensor

    def setPosition(self, position):
        if position > self.hi or position < self.lo:   # Webots: "too big/low requested position"
            self.clamped_positions += 1
            position = min(self.hi, max(self.lo, position))
        self.mode, self.target = 'position', float(position)

    def setForce(self, force):
        if abs(force) > self.fmax:                     # Webots: "requested motor torque exceeds maxTorque"
            self.clamped_forces += 1
            force = math.copysign(self.fmax, force)
        self.mode, self.force = 'force', float(force)

    setTorque = setForce

    def apply(self):
        m, d = self.world.m, self.world.d
        if self.mode == 'position':                    # v = P (target - q), limited to maxVelocity
            v = max(-self.vmax, min(self.vmax, self.p * (self.target - d.qpos[self.qadr])))
            m.actuator_gainprm[self.vel, 0], m.actuator_biasprm[self.vel, 2] = self.kv, -self.kv
            d.ctrl[self.vel], d.ctrl[self.trq] = v, 0.0
        else:                                          # force control: the velocity servo is off
            m.actuator_gainprm[self.vel, 0], m.actuator_biasprm[self.vel, 2] = 0.0, 0.0
            d.ctrl[self.vel], d.ctrl[self.trq] = 0.0, self.force


class _SensorDevice(Device):
    def __init__(self, world, name, sensor):
        super().__init__(world, name)
        self.adr = world.m.sensor(sensor).adr[0]

    def raw(self):
        return np.array(self.world.d.sensordata[self.adr:self.adr + 3])


class Gyro(_SensorDevice):
    def getValues(self):
        return list(R @ self.raw())                    # body-frame angular velocity, Webots axes


class Accelerometer(_SensorDevice):
    def getValues(self):
        return list(R @ self.raw())


class GPS(_SensorDevice):
    def getValues(self):
        return list(R @ self.raw())                    # world position, Webots axes


class TouchSensor(Device):
    """force-3d is approximated by the normal force along the Webots +y axis (all the touch sensor
    sees on flat ground with these feet)."""

    def __init__(self, world, name, sensor):
        super().__init__(world, name)
        self.adr = world.m.sensor(sensor).adr[0]

    def getValues(self):
        return [0.0, float(self.world.d.sensordata[self.adr]), 0.0]

    def getValue(self):
        return float(self.world.d.sensordata[self.adr])


class DistanceSensor(Device):
    """CoM_DistanceSensor: laser 4 cm below the body centre pointing down the body, 0 to 0.4 m."""

    def __init__(self, world, name, site='height_sensor', max_range=0.4):
        super().__init__(world, name)
        self.site, self.max_range = world.m.site(site).id, max_range
        self.geomid = np.zeros(1, np.int32)
        self.groups = np.array([1, 0, 0, 0, 0, 0], np.uint8)     # collision geoms only (group 0)

    def getValue(self):
        m, d = self.world.m, self.world.d
        down = -d.site_xmat[self.site].reshape(3, 3)[:, 2]
        dist = mujoco.mj_ray(m, d, d.site_xpos[self.site], down, self.groups, 1, self.world.torso, self.geomid)
        return self.max_range if dist < 0 else min(self.max_range, dist)


class Receiver(Device):
    def __init__(self, world, name):
        super().__init__(world, name)
        self.queue = []
        world.receivers.append(self)

    def getQueueLength(self):
        return len(self.queue)

    def getBytes(self):
        return self.queue[0]

    def getFloats(self):
        b = self.queue[0]
        return struct.unpack('%dd' % (len(b) // 8), b)

    def getDataSize(self):
        return len(self.queue[0])

    def nextPacket(self):
        self.queue.pop(0)


class Emitter(Device):
    def send(self, message, length=None):              # "serial" emitter: nothing listens to it
        return 1


# ----------------------------------------------------------------------------- robot
def _devices(world):
    devs = {}
    for i in (1, 2, 3, 4):
        devs['servo%d_rot' % i] = Motor(world, 'servo%d_rot' % i, 'hip%d' % i)
        devs['spring%d' % i] = Motor(world, 'spring%d' % i, 'spring%d' % i, actuated=False,
                                     min_position=0.0, max_position=0.035, max_force=0.0)
        devs['leg_gyro_%d_1' % i] = Gyro(world, 'leg_gyro_%d_1' % i, 'leg_gyro_%d_1' % i)
        devs['TOUCHSENSOR%d' % i] = TouchSensor(world, 'TOUCHSENSOR%d' % i, 'TOUCHSENSOR%d' % i)
    for name, dev in list(devs.items()):
        if isinstance(dev, Motor):
            devs[dev.sensor.name] = dev.sensor
    devs['CoM_Gyro'] = Gyro(world, 'CoM_Gyro', 'CoM_Gyro')
    devs['CoM_Accelerometer'] = Accelerometer(world, 'CoM_Accelerometer', 'CoM_Accelerometer')
    devs['CoM_Gps'] = GPS(world, 'CoM_Gps', 'CoM_Gps')
    devs['CoM_Receiver'] = Receiver(world, 'CoM_Receiver')
    devs['CoM_Emitter'] = Emitter(world, 'CoM_Emitter')
    devs['CoM_DistanceSensor'] = DistanceSensor(world, 'CoM_DistanceSensor')
    return devs


class Robot:
    def __init__(self):
        self.world = World.current
        if self.world is None:
            raise RuntimeError('no MuJoCo world: run the controller through run_controller.py')
        self.devices = _devices(self.world)
        self.device_list = list(self.devices.values())

    def getDevice(self, name):
        if name not in self.devices:
            print('Device "%s" was not found on robot "QUAD_ROBOT"' % name)
            return None
        return self.devices[name]

    def getNumberOfDevices(self):
        return len(self.device_list)

    def getDeviceByIndex(self, i):
        return self.device_list[i]

    def getBasicTimeStep(self):
        return self.world.dt_ms

    def getTime(self):
        return self.world.time()

    def getName(self):
        return 'QUAD_ROBOT'

    def step(self, duration=None):
        w = self.world
        n = max(1, int(round((duration or w.dt_ms) / w.dt_ms)))
        for _ in range(n):
            if w.quit or w.time() >= w.end_time - 1e-9:
                return -1
            w.physics_step()
        return 0


class Supervisor(Robot):
    SIMULATION_MODE_PAUSE, SIMULATION_MODE_REAL_TIME, SIMULATION_MODE_FAST = 0, 1, 2

    def setLabel(self, id, label, x, y, size, color, transparency=0, font='Arial'):
        """Kept with the Webots geometry (top-left corner at x, y of the view; size relative to the view
        height); run_controller.py draws them on the video frames with hud.py."""
        self.world.labels[id] = dict(text=str(label), x=x, y=y, size=size, color=color,
                                     transparency=transparency, font=font)

    def simulationSetMode(self, mode):
        pass                                           # no pause outside Webots: the run continues

    def simulationGetMode(self):
        return self.SIMULATION_MODE_FAST

    def simulationQuit(self, status):
        self.world.quit = True

    def getFromDef(self, name):
        return None
