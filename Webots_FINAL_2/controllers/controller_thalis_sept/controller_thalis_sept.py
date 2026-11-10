"""Python port of the empty MATLAB controller template (my_controller_mat.m / my_controller_TEST.m /
controller_thalis_sept.m): it only steps the simulation every 64 ms, like the original."""
from controller import Robot

# uncomment the next two lines if you want to use
# MATLAB's desktop to interact with the controller:
# desktop;
# keyboard;

TIME_STEP = 64
robot = Robot()

# get and enable devices, e.g.:
#  camera = robot.getDevice('camera')
#  camera.enable(TIME_STEP)

# main loop:
# perform simulation steps of TIME_STEP milliseconds
# and leave the loop when Webots signals the termination
while robot.step(TIME_STEP) != -1:
    # read the sensors, process the data, send actuator commands
    # (MATLAB `drawnow;` flushed figures; nothing to do here)
    pass

# cleanup code goes here: write data to files, etc.
