"""Python port of controller_webots741_Mar_FINAL.m (MATLAB controller for Webots).

Original: - supervisor_trot.m / Dec 2012 / Quad Controller / Athanasios Mastrogeorgiou
(the same MATLAB file was used as controller_webots730_Feb_FINAL, controller_webots740_Feb_FINAL,
controller_webots741_Mar_FINAL and controller_webots743_Oct_THALIS).

Ported to Webots R2025a, keeping the MATLAB structure line for line. Changes:
  * wb_motor_enable_position / wb_motor_get_position no longer exist: the converted world gives
    each joint a PositionSensor named "<motor>_sensor", read here instead.
  * wb_receiver_get_data(..., 'double') -> Receiver.getFloats().
  * MATLAB's `desktop; keyboard;` at the stop condition -> the logged W*p arrays are written to
    data/*.csv and the simulation pauses (QUADSIM_AT_STOP=quit|continue changes that).
  * wb_motor_set_force goes through set_force_webots7(): Webots 8.6 reversed the torque direction
    on these hip joints (a Webots bug fix), and this controller was tuned on the old direction.
  * BodyRotationMatrix starts as the identity, in case no physics-plugin packet arrived yet.
  * Supervisor labels go through matlab_port.set_label: a tidy HUD by default,
    QUADSIM_LABELS=original for the 2014 look, QUADSIM_LABELS=off to hide them.
"""
import math
import struct

from controller import Supervisor

from controller_options import *  # noqa: F401,F403  (MATLAB: controller_options)
from gtd_controller8 import gtd_controller8
from matlab_port import all_equal, at_stop, num2str, rgb, save_logs, set_force_webots7, set_label, strcat

CONTROLLER = __file__.rsplit('/', 1)[-1][:-3]
robot = Supervisor()
TIME_STEP = 1
# %% ------------------set_Webots_params-------------------------------------
# Webots_options( 3400, 7, 0.0035 );  %keyboard;
# %% --------------Get_and_Enable_Motors_and_Springs-------------------------
# motors
motors = {i: robot.getDevice('servo%d_rot' % i) for i in (1, 2, 3, 4)}
# springs
springs = {i: robot.getDevice('spring%d' % i) for i in (1, 2, 3, 4)}
# springs enable position (Webots 8+: through the joint's PositionSensor)
spring_sensors = {i: springs[i].getPositionSensor() for i in springs}
for s in spring_sensors.values():
    s.enable(TIME_STEP)
# motors enable position
motor_sensors = {i: motors[i].getPositionSensor() for i in motors}
for s in motor_sensors.values():
    s.enable(TIME_STEP)
# %% --------------Get_and_Enable_Sensors------------------------------------
CoM_Accelerometer = robot.getDevice('CoM_Accelerometer')
CoM_Accelerometer.enable(TIME_STEP)
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

BodyRotationMatrix = [1, 0, 0, 0, 1, 0, 0, 0, 1]
stopped = False


def logs():
    series = {'Wxdp': Wxdp, 'Wyp': Wyp, 'Wydp': Wydp, 'Wthp': Wthp, 'Wthdp': Wthdp,
              'Wgambp': Wgambp, 'Wgamfp': Wgamfp, 'Wgambdp': Wgambdp, 'Wgamfdp': Wgamfdp,
              'Wspring1_pos_p': Wspring1_pos_p, 'Wspring2_pos_p': Wspring2_pos_p,
              'Wspring3_pos_p': Wspring3_pos_p, 'Wspring4_pos_p': Wspring4_pos_p}
    per_gait = {'Wgaitnumbertime': Wgaitnumbertime, 'Wgambtdp': Wgambtdp, 'Wgamftdp': Wgamftdp,
                'Wtorbctrp': Wtorbctrp, 'Wtorfctrp': Wtorfctrp, 'Wxdlop': Wxdlop}
    return series, per_gait


# %% --------------------------Main_Control_Loop-----------------------------
while robot.step(TIME_STEP) != -1:
    time = time + 1
    if (time >= stop_sim_in or gait_number > stop_number) and not stopped:
        # MATLAB: desktop; keyboard;
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
    CoM_accelerometer_values = CoM_Accelerometer.getValues()
    CoM_gps_values = CoM_Gps.getValues()
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
    # %% -----------------------Data_Gathering-----------------------------------
    Wxdp.append(vBody[0])
    Wyp.append(CoM_gps_values[1])
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
            xdlo = vBody[0]
            ylo = CoM_gps_values[1]
            ydlo = -vBody[1]
            thlo = math.atan2(BodyRotationMatrix[1], BodyRotationMatrix[0])
            thdlo = -CoM_gyro_values[2]
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
            # -----------------------Data_Gathering-----------------------------
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
            str_gambtd = strcat('gambtd= ', str_gambtd)
            str_gamftd = strcat('gamftd= ', str_gamftd)
            str_torctr = strcat('torbctr= ', str_torbctr)  # (as in the MATLAB: label 33 shows the bare number)
            set_label(robot, 11, str_gambtd, 0.14, 0, 0.1, rgb([0.5, 0, 1]), 0.1)
            set_label(robot, 22, str_gamftd, 0.46, 0, 0.1, rgb([0.5, 0, 1]), 0.1)
            set_label(robot, 33, str_torbctr, 0.76, 0, 0.1, rgb([0.5, 0, 1]), 0.05)
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
