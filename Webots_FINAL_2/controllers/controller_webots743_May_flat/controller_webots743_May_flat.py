"""Python port of controller_webots743_May_flat.m (MATLAB controller for Webots, Athanasios Mastrogeorgiou).

Work-in-progress version (May 2014) with the force-angle-margin computation. Ported to Webots
R2025a, keeping the MATLAB structure line for line. Changes:
  * wb_motor_enable_position / wb_motor_get_position no longer exist: the converted world gives
    each joint a PositionSensor named "<motor>_sensor", read here instead.
  * wb_receiver_get_data(..., 'double') -> Receiver.getFloats().
  * MATLAB's `keyboard` at the stop condition -> the logged W*p arrays are written to data/*.csv
    and the simulation pauses (QUADSIM_AT_STOP=quit|continue changes that).
  * wb_motor_set_force goes through set_force_webots7(): Webots 8.6 reversed the torque direction
    on these hip joints (a Webots bug fix), and this controller was tuned on the old direction.
  * BodyRotationMatrix starts as the identity, in case no physics-plugin packet arrived yet.
  * Supervisor labels go through matlab_port.set_label: a tidy HUD by default,
    QUADSIM_LABELS=original for the 2014 look, QUADSIM_LABELS=off to hide them.
  * The MATLAB read Foot1_Gps..Foot4_Gps but had the lines that create them commented out, so it
    stopped at the first step. Here they are fetched when the world has FOOT<n>_GPS devices (only
    SUPERVISOR_webots743_May_touch66 has all four); without them, and without the TOUCHSENSOR<n>
    devices some worlds lack, those readings are zero and the force-angle-margin block is skipped.
  * SUPERVISOR_thalis_sept has no spring on leg 1; its reading is NaN (as an invalid device read in
    Webots 7), so the stance/flight tests that involve it are false and the state machine stays idle.
  * rotz, radtodeg, cross, norm, transp (outer product) -> matlab_port / inline Python.
  * `figure;` (an empty MATLAB figure) is dropped.
"""
import math
import struct

from controller import Supervisor

from controller_options import *  # noqa: F401,F403  (MATLAB: controller_options)
from gtd_controller8 import gtd_controller8
from matlab_port import (all_equal, at_stop, cross, matmul, norm, num2str, optional_device, position_sensor,
                         radtodeg, rgb, rotz, save_logs, set_force_webots7, set_label, strcat, values_or_zeros)

CONTROLLER = __file__.rsplit('/', 1)[-1][:-3]
robot = Supervisor()
TIME_STEP = 1
# %% ------------------set_Webots_params-------------------------------------
# Webots_options( 3400, 7, 0.0035 );  %keyboard;
# %% --------------Get_and_Enable_Motors_and_Springs-------------------------
# Contact Points
f1 = robot.getFromDef('FOOT_1')
f2 = robot.getFromDef('FOOT_2')
f3 = robot.getFromDef('FOOT_3')
f4 = robot.getFromDef('FOOT_4')
# motors
motors = {i: robot.getDevice('servo%d_rot' % i) for i in (1, 2, 3, 4)}
# springs (SUPERVISOR_thalis_sept has no leg-1 spring: its reading is NaN, see matlab_port)
springs = {i: optional_device(robot, 'spring%d' % i) for i in (1, 2, 3, 4)}
# springs enable position (Webots 8+: through the joint's PositionSensor)
spring_sensors = {i: position_sensor(springs[i], TIME_STEP) for i in springs}
# motors enable position
motor_sensors = {i: motors[i].getPositionSensor() for i in motors}
for s in motor_sensors.values():
    s.enable(TIME_STEP)
# %% --------------Get_and_Enable_Sensors------------------------------------
CoM_DistanceSensor = robot.getDevice('CoM_DistanceSensor')
CoM_DistanceSensor.enable(TIME_STEP)
CoM_Accelerometer = robot.getDevice('CoM_Accelerometer')
CoM_Accelerometer.enable(TIME_STEP)
# get and enable TOUCHSENSOR device
touch1 = optional_device(robot, 'TOUCHSENSOR1', TIME_STEP)
touch2 = optional_device(robot, 'TOUCHSENSOR2', TIME_STEP)
touch3 = optional_device(robot, 'TOUCHSENSOR3', TIME_STEP)
touch4 = optional_device(robot, 'TOUCHSENSOR4', TIME_STEP)
# get and enable CoM_Gyro device
CoM_Gyro = robot.getDevice('CoM_Gyro')
CoM_Gyro.enable(TIME_STEP)
# get and enable CoM_Gps device
CoM_Gps = robot.getDevice('CoM_Gps')
CoM_Gps.enable(TIME_STEP)
# get and enable CoM_Receiver device
CoM_Receiver = robot.getDevice('CoM_Receiver')
CoM_Receiver.enable(TIME_STEP)
# get and enable CoM_Emitter device
CoM_Emitter = robot.getDevice('CoM_Emitter')
# get and enable leg1 Gyro1_1 device
LEG_GYRO_1_1 = robot.getDevice('leg_gyro_1_1')
LEG_GYRO_1_1.enable(TIME_STEP)
# get and enable leg2 Gyro2_1 device
LEG_GYRO_2_1 = robot.getDevice('leg_gyro_2_1')
LEG_GYRO_2_1.enable(TIME_STEP)
# get and enable Foot1_Gps .. Foot4_Gps devices (commented out in the MATLAB, see the header)
Foot1_Gps = optional_device(robot, 'FOOT1_GPS', TIME_STEP)
Foot2_Gps = optional_device(robot, 'FOOT2_GPS', TIME_STEP)
Foot3_Gps = optional_device(robot, 'FOOT3_GPS', TIME_STEP)
Foot4_Gps = optional_device(robot, 'FOOT4_GPS', TIME_STEP)
have_fam_inputs = None not in (Foot1_Gps, Foot2_Gps, Foot3_Gps, Foot4_Gps, touch1, touch2, touch3, touch4)

BodyRotationMatrix = [1, 0, 0, 0, 1, 0, 0, 0, 1]
stopped = False


def logs():
    series = {'Wxp': Wxp, 'Wyp': Wyp, 'Wzp': Wzp, 'Wxdp': Wxdp, 'Wydp': Wydp, 'Wthp': Wthp, 'Wthdp': Wthdp,
              'Wgambp': Wgambp, 'Wgamfp': Wgamfp, 'Wgambdp': Wgambdp, 'Wgamfdp': Wgamfdp,
              'Wspring1_pos_p': Wspring1_pos_p, 'Wspring2_pos_p': Wspring2_pos_p,
              'Wspring3_pos_p': Wspring3_pos_p, 'Wspring4_pos_p': Wspring4_pos_p,
              'WCoM_DistanceSensor_valuesp': WCoM_DistanceSensor_valuesp,
              'Wf1xp': Wf1xp, 'Wf1yp': Wf1yp, 'Wf1zp': Wf1zp, 'Wf2xp': Wf2xp, 'Wf2yp': Wf2yp, 'Wf2zp': Wf2zp,
              'Wf3xp': Wf3xp, 'Wf3yp': Wf3yp, 'Wf3zp': Wf3zp, 'Wf4xp': Wf4xp, 'Wf4yp': Wf4yp, 'Wf4zp': Wf4zp,
              'Wtouch1xp': Wtouch1xp, 'Wtouch1yp': Wtouch1yp, 'Wtouch1zp': Wtouch1zp,
              'Wtouch2xp': Wtouch2xp, 'Wtouch2yp': Wtouch2yp, 'Wtouch2zp': Wtouch2zp,
              'Wtouch3xp': Wtouch3xp, 'Wtouch3yp': Wtouch3yp, 'Wtouch3zp': Wtouch3zp,
              'Wtouch4xp': Wtouch4xp, 'Wtouch4yp': Wtouch4yp, 'Wtouch4zp': Wtouch4zp}
    per_gait = {'Wgaitnumbertime': Wgaitnumbertime, 'Wgambtdp': Wgambtdp, 'Wgamftdp': Wgamftdp,
                'Wtorbctrp': Wtorbctrp, 'Wtorfctrp': Wtorfctrp, 'Wxdlop': Wxdlop}
    return series, per_gait


def unit(v):
    n = norm(v)
    return [x / n for x in v] if n > 0 else [math.nan] * len(v)


# %% --------------------------Main_Control_Loop-----------------------------
# figure;
while robot.step(TIME_STEP) != -1:
    time = time + 1
    if (time >= stop_sim_in or gait_number > stop_number) and not stopped:
        # MATLAB: keyboard;
        stopped = True
        print('stop condition reached (time %d ms, gait %d)' % (time, gait_number))
        save_logs(CONTROLLER, *logs())
        at_stop(robot)
    # %% ----------------------Get_Spring/servo_positions------------------------
    spring1_pos = spring_sensors[1].getValue()
    spring2_pos = spring_sensors[2].getValue()
    spring3_pos = spring_sensors[3].getValue()
    spring4_pos = spring_sensors[4].getValue()
    # rotational Servos. Back (1 & 4), Front (2 & 3)
    back_servo_pos1 = motor_sensors[1].getValue()
    back_servo_pos4 = motor_sensors[4].getValue()
    front_servo_pos2 = motor_sensors[2].getValue()
    front_servo_pos3 = motor_sensors[3].getValue()
    # %% ----------------------CoM_Sensors---------------------------------------
    # CoM_DistanceSensor
    CoM_DistanceSensor_values = CoM_DistanceSensor.getValue() + 0.04  # the sensor is placed 4cm in front of the CoM
    WCoM_DistanceSensor_valuesp.append(CoM_DistanceSensor_values)
    # CoM_Accelerometer
    CoM_accelerometer_values = CoM_Accelerometer.getValues()
    # CoM_Gps
    CoM_gps_values = CoM_Gps.getValues()
    # Foot GPS
    foot1_gps_values = values_or_zeros(Foot1_Gps)
    foot2_gps_values = values_or_zeros(Foot2_Gps)
    foot3_gps_values = values_or_zeros(Foot3_Gps)
    foot4_gps_values = values_or_zeros(Foot4_Gps)
    # CoM_Gyro
    CoM_gyro_values = CoM_Gyro.getValues()
    # CoM_Receiver. Listens to channel 0.
    while CoM_Receiver.getQueueLength() > 0:
        A = CoM_Receiver.getFloats()
        if all_equal(A, 1234):
            CoM_Receiver.nextPacket()
            A = CoM_Receiver.getFloats()
            BodyRotationMatrix = A
        else:
            vBody = A
        CoM_Receiver.nextPacket()
    # %% ---------------------Leg_Sensors----------------------------------------
    gyro_values_1_1 = LEG_GYRO_1_1.getValues()
    gyro_values_2_1 = LEG_GYRO_2_1.getValues()
    # Touch Sensors
    touch_values_1 = values_or_zeros(touch1)
    touch_values_2 = values_or_zeros(touch2)
    touch_values_3 = values_or_zeros(touch3)
    touch_values_4 = values_or_zeros(touch4)
    T = [[1, 0, -CoM_gps_values[0]],
         [0, 1, CoM_gps_values[1]],
         [0, 0, 1]]
    # %% -----------------------Force_Angle_Margin-------------------------------
    # Rotation and Translation Matrices
    Tnew = [[1, 0, 0, CoM_gps_values[0]],
            [0, 1, 0, -CoM_gps_values[1]],
            [0, 0, 1, -CoM_gps_values[2]],
            [0, 0, 0, 1]]
    angleInDegrees1 = radtodeg(math.atan2(BodyRotationMatrix[1], BodyRotationMatrix[0]))
    Rnew = rotz(-angleInDegrees1)
    Rnew = Rnew + [[0, 0, 0]]                                   # Rnew = [Rnew; 0, 0, 0];
    Rnew = [row + [clmn[i]] for i, row in enumerate(Rnew)]      # Rnew = [Rnew, clmn];
    # CoM Point
    P = [-CoM_gps_values[0], CoM_gps_values[1], CoM_gps_values[2], 1]
    # P = (0,0)
    Pnew = matmul(Rnew, matmul(Tnew, P))
    Pf1g = [-foot1_gps_values[0], foot1_gps_values[1], foot1_gps_values[2], 1]
    Pf2g = [-foot2_gps_values[0], foot2_gps_values[1], foot2_gps_values[2], 1]
    Pf3g = [-foot3_gps_values[0], foot3_gps_values[1], foot3_gps_values[2], 1]
    Pf4g = [-foot4_gps_values[0], foot4_gps_values[1], foot4_gps_values[2], 1]
    Pf1 = matmul(Rnew, matmul(Tnew, Pf1g))
    Pf2 = matmul(Rnew, matmul(Tnew, Pf2g))
    Pf3 = matmul(Rnew, matmul(Tnew, Pf3g))
    Pf4 = matmul(Rnew, matmul(Tnew, Pf4g))
    Wtouch1xp.append(touch_values_1[0])
    Wtouch1yp.append(touch_values_1[1])
    Wtouch1zp.append(touch_values_1[2])
    Wtouch2xp.append(touch_values_2[0])
    Wtouch2yp.append(touch_values_2[1])
    Wtouch2zp.append(touch_values_2[2])
    Wtouch3xp.append(touch_values_3[0])
    Wtouch3yp.append(touch_values_3[1])
    Wtouch3zp.append(touch_values_3[2])
    Wtouch4xp.append(touch_values_4[0])
    Wtouch4yp.append(touch_values_4[1])
    Wtouch4zp.append(touch_values_4[2])
    if have_fam_inputs:
        Pvleg1 = [(Pf1[i] + Pf4[i]) / 2 for i in range(3)]
        Pvleg2 = [(Pf2[i] + Pf3[i]) / 2 for i in range(3)]
        a1 = [Pvleg2[i] - Pvleg1[i] for i in range(3)]
        a2 = [Pvleg1[i] - Pvleg2[i] for i in range(3)]
        ahat1 = unit(a1)
        ahat2 = unit(a2)
        # l = (eye(3) - ahat * transp(ahat)) * Pvleg2
        l_1 = [Pvleg2[i] - ahat1[i] * sum(ahat1[j] * Pvleg2[j] for j in range(3)) for i in range(3)]
        l_2 = [Pvleg2[i] - ahat2[i] * sum(ahat2[j] * Pvleg2[j] for j in range(3)) for i in range(3)]  # .......??????
        Fr = [touch_values_1[i] + touch_values_2[i] + touch_values_3[i] + touch_values_4[i] for i in range(3)]  # FrX FrY FrZ
        FrA = [touch_values_1[i] + touch_values_4[i] for i in range(3)]
        FrB = [touch_values_2[i] + touch_values_3[i] for i in range(3)]
        rA = [0.27 + 0.3 * math.sin(back_servo_pos1), 0.3 * math.cos(back_servo_pos1), 0]
        rB = [0.27 - 0.3 * math.sin(back_servo_pos1), 0.3 * math.cos(back_servo_pos1), 0]
        n1 = cross(rA, FrA)
        n2 = cross(rB, FrB)
        Ihat1 = unit(l_1)
        Ihat2 = unit(l_2)
        Fn1 = [x / norm(l_1) if norm(l_1) else math.nan for x in cross(Ihat1, n1)]
        Fn2 = [x / norm(l_2) if norm(l_2) else math.nan for x in cross(Ihat2, n2)]
    # %% -----------------------Data_Gathering-----------------------------------
    Pf1p.extend(Pf1)
    Pf2p.extend(Pf2)
    Pf3p.extend(Pf3)
    Pf4p.extend(Pf4)
    Pnewp.extend(Pnew)
    Wxp.append(-CoM_gps_values[0])
    Wyp.append(CoM_gps_values[1])
    Wzp.append(CoM_gps_values[2])
    Wxdp.append(vBody[0])
    Wydp.append(-vBody[1])
    Wthp.append(math.atan2(BodyRotationMatrix[1], BodyRotationMatrix[0]))
    Wthdp.append(-CoM_gyro_values[2])
    Wgambp.append(back_servo_pos1)
    Wgamfp.append(front_servo_pos2)
    Wgambdp.append(-gyro_values_1_1[2])
    Wgamfdp.append(-gyro_values_2_1[2])
    Wspring1_pos_p.append(spring1_pos)
    Wspring2_pos_p.append(spring2_pos)
    Wspring3_pos_p.append(spring3_pos)
    Wspring4_pos_p.append(spring4_pos)
    Wf1xp.append(-foot1_gps_values[0])
    Wf1yp.append(foot1_gps_values[1])
    Wf1zp.append(foot1_gps_values[2])
    Wf2xp.append(-foot2_gps_values[0])
    Wf2yp.append(foot2_gps_values[1])
    Wf2zp.append(foot2_gps_values[2])
    Wf3xp.append(-foot3_gps_values[0])
    Wf3yp.append(foot3_gps_values[1])
    Wf3zp.append(foot3_gps_values[2])
    Wf4xp.append(-foot4_gps_values[0])
    Wf4yp.append(foot4_gps_values[1])
    Wf4zp.append(foot4_gps_values[2])
    # emiter
    CoM_Emitter.send(struct.pack('d', spring1_pos))
    # --------------------------------------------------------------------------
    if time > 5:
        # %% ------------------------States------------------------------------------
        if spring1_pos > touch_tol and spring2_pos > touch_tol and \
           spring3_pos > touch_tol and spring4_pos > touch_tol and \
           double_stance == 0:
            set_label(robot, 1, '-Double Stance-', 0, 0.4, 0.1, rgb([1, 1, 0]), 0.5)
            # Flags
            double_stance = 1
            front_stance = 0
            back_stance = 0
            touchdown = 1
            liftoff = 0
            # Apply torque
            # Back
            set_force_webots7(motors[1], torbctr)
            set_force_webots7(motors[4], torbctr)
            # Front
            set_force_webots7(motors[2], torfctr)
            set_force_webots7(motors[3], torfctr)
        elif spring1_pos <= touch_tol and spring2_pos <= touch_tol and \
                spring3_pos <= touch_tol and spring4_pos <= touch_tol and \
                liftoff == 0:
            # Indication
            set_label(robot, 1, '-Flight-', 0, 0, 0.1, rgb([1, 0, 0]), 0.1)
            print('--------------| %d ' % gait_number)
            Wgaitnumbertime.append(time)
            # Flags
            liftoff = 1
            back_stance = 0
            front_stance = 0
            double_stance = 0
            # ---------------Liftoff Parameters----------------------------------
            if time < 10:
                ylo = CoM_gps_values[1]
            else:
                ylo = CoM_DistanceSensor_values
            ydlo_tp = matmul(R, [0, -vBody[1], 0])
            ydlo = ydlo_tp[1]
            xdlo_tp = matmul(R, [vBody[0], 0, 0])
            xdlo = xdlo_tp[0]
            thlo = math.atan2(BodyRotationMatrix[1], BodyRotationMatrix[0]) - psia
            thdlo = (-CoM_gyro_values[2])
            gamblo = back_servo_pos1
            gamflo = front_servo_pos2
            gambdlo = -gyro_values_1_1[2]
            gamfdlo = -gyro_values_2_1[2]
            # --------------Call Controller--------------------------------------
            out = gtd_controller8(m, ib, mt, mul, ml, il, l1, d, l0, g, k, b, spl,
                                  psia, xddes, h, xdlo, ylo, ydlo, thlo,
                                  thdlo, gamblo, gamflo, gambdlo, gamfdlo)
            # Match Webots Positive direction
            gambtd = out[0]
            gamftd = out[1]
            torbctr = -out[2] / legs
            torfctr = -out[3] / legs
            # ------------------------Data_Gathering-----------------------------
            Wgambtdp.append(out[0])
            Wgamftdp.append(out[1])
            Wtorbctrp.append(torbctr)
            Wtorfctrp.append(torfctr)
            Wxdlop.append(xdlo)
            # _Label colors [1 1 0] yellow [0 0 0] black [0 1 1] cyan [1 0 0] red
            #               [0 1 0] green  [0 0 1] blue  [1 1 1] white
            #               [1 0 1] magenta
            str_gambtd = num2str(gambtd)
            str_gamftd = num2str(gamftd)
            str_torbctr = num2str(torbctr)
            str_xdlo = num2str(xdlo)
            str_ylo = num2str(ylo)
            str_ydlo = num2str(ydlo)
            str_thlo = num2str(thlo)
            str_thdlo = num2str(thdlo)
            str_gambtd = strcat('gambtd= ', str_gambtd)
            str_gamftd = strcat('gamftd= ', str_gamftd)
            str_torbctr = strcat('tq= ', str_torbctr)
            str_xdlo = strcat('xdlo= ', str_xdlo)
            str_ylo = strcat('ylo= ', str_ylo)
            str_ydlo = strcat('ydlo= ', str_ydlo)
            str_thlo = strcat('thlo= ', str_thlo)
            str_thdlo = strcat('thdlo= ', str_thdlo)
            set_label(robot, 11, str_gambtd, 0.14, 0.00, 0.1, rgb([0, 0, 0]), 0.1)
            set_label(robot, 22, str_gamftd, 0.46, 0.00, 0.1, rgb([0, 0, 0]), 0.1)
            set_label(robot, 33, str_torbctr, 0.76, 0.00, 0.1, rgb([0, 0, 0]), 0.0)
            set_label(robot, 43, str_xdlo, 0.76, 0.05, 0.1, rgb([0, 0, 0]), 0.05)
            set_label(robot, 53, str_ylo, 0.76, 0.10, 0.1, rgb([0, 0, 0]), 0.05)
            set_label(robot, 63, str_ydlo, 0.76, 0.15, 0.1, rgb([0, 0, 0]), 0.05)
            set_label(robot, 73, str_thlo, 0.76, 0.20, 0.1, rgb([0, 0, 0]), 0.05)
            set_label(robot, 83, str_thdlo, 0.76, 0.25, 0.1, rgb([0, 0, 0]), 0.05)
            # -------------------------------------------------------------------
            # Prepare Legs for Touchdown
            # Front Legs
            motors[3].setPosition(gamftd)
            motors[2].setPosition(gamftd)
            # Back Legs
            motors[1].setPosition(gambtd)
            motors[4].setPosition(gambtd)
            # increase gait number
            gait_number = gait_number + 1
            # reset parameters
            back_servo_pos1 = 0
            front_servo_pos2 = 0
        elif (spring1_pos > touch_tol and spring4_pos > touch_tol) and \
             (spring2_pos <= touch_tol and spring3_pos <= touch_tol) and \
                back_stance == 0:
            # -back_stance-
            set_label(robot, 1, '-Back_Stance-', 0, 0.3, 0.1, rgb([1, 0, 0]), 0.5)
            double_stance = 0
            front_stance = 0
            back_stance = 1
            touchdown = 1
            liftoff = 0
            # get front leg position
            front_servo_pos2 = motor_sensors[2].getValue()
            # apply torque and leg positions
            # Back legs
            set_force_webots7(motors[4], torbctr)
            set_force_webots7(motors[1], torbctr)
            # Front legs
            motors[3].setPosition(front_servo_pos2)
            motors[2].setPosition(front_servo_pos2)
        elif (spring2_pos > touch_tol and spring3_pos > touch_tol) and \
             (spring1_pos <= touch_tol and spring4_pos <= touch_tol) and \
                front_stance == 0:
            # -front_stance-
            set_label(robot, 1, '-Front_Stance-', 0, 0.2, 0.1, rgb([1, 0, 0]), 0.5)
            double_stance = 0
            back_stance = 0
            front_stance = 1
            touchdown = 1
            liftoff = 0
            # get back leg position
            back_servo_pos1 = motor_sensors[1].getValue()
            # apply torque and leg positions
            # Front legs
            set_force_webots7(motors[2], torfctr)
            set_force_webots7(motors[3], torfctr)
            # Back legs
            motors[1].setPosition(back_servo_pos1)
            motors[4].setPosition(back_servo_pos1)
        else:
            touchdown = 0

# cleanup code goes here: write data to files, etc.
if not stopped:
    save_logs(CONTROLLER, *logs())
