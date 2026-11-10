"""Python port of Webots_options.m: set the leg springs' constant, damping and maxStop from the supervisor.

k = spring constant, b = damping constant, mS = maxStop.
The controllers keep the call commented out, as the MATLAB did: Webots_options(robot, 3400, 7, 0.0035).
R2025a: these fields live in the SliderJoint's jointParameters node, so they are reached through it
(the MATLAB asked the SliderJoint itself for 'springConstant').
"""


def Webots_options(robot, k, b, mS):
    for i in (1, 2, 3, 4):
        joint = robot.getFromDef('SPRING%d' % i)
        params = joint.getField('jointParameters').getSFNode()
        params.getField('springConstant').setSFFloat(k)   # Set Spring Constant
        params.getField('dampingConstant').setSFFloat(b)  # Set Damping Constant
        params.getField('maxStop').setSFFloat(mS)         # Set Spring Length (mS = max Stop)
