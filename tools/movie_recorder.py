"""Helper supervisor used by run_headless.sh (not part of the original project).

run_headless.sh adds `Robot { name "movie recorder" supervisor TRUE controller "movie_recorder" }`
to a temporary copy of the world. This controller records the 3D view to QUADSIM_MOVIE (if set)
for QUADSIM_SECONDS of simulated time, then closes Webots. It also writes the engine's name
(QUADSIM_BADGE, default "Webots"; empty for none) in the lower right corner of the view.
"""
import os

from controller import Supervisor

robot = Supervisor()
step = int(robot.getBasicTimeStep())
movie = os.environ.get('QUADSIM_MOVIE', '')
seconds = float(os.environ.get('QUADSIM_SECONDS', '10'))
width, height = (int(v) for v in os.environ.get('QUADSIM_MOVIE_SIZE', '1280x720').split('x'))

badge = os.environ.get('QUADSIM_BADGE', 'Webots')


if badge:
    # Webots draws a label's line box 0.5 * size * height tall from its top-left corner (measured on
    # R2025a movies). For "Webots" in Arial Black at size 0.085 the text is 0.070 of a 16:9 view wide,
    # so this puts it 0.012 of the view from the right and bottom edges, like the HUD's margins.
    # mujoco/hud.py draws the MuJoCo badge with the same font, size, colour and margins.
    # Label id 9: the controllers use 1, 11, 22, ... and R2025a silently drops ids in the hundreds.
    robot.setLabel(9, badge, 0.918 - 0.0116 * (len(badge) - 6), 0.9455, 0.085, 0x37474F, 0.0, 'Arial Black')

zoom = float(os.environ.get('QUADSIM_VIEW_ZOOM', '1'))  # >1 moves the camera away from the robot
if zoom != 1:
    viewpoint = next(n for n in (robot.getRoot().getField('children').getMFNode(i)
                                 for i in range(robot.getRoot().getField('children').getCount()))
                     if n.getTypeName() == 'Viewpoint')
    target = robot.getFromDef('QUAD_ROBOT').getPosition()
    position = viewpoint.getField('position').getSFVec3f()
    viewpoint.getField('position').setSFVec3f([t + zoom * (p - t) for p, t in zip(position, target)])
if movie:
    robot.movieStartRecording(movie, width, height, codec=0, quality=90, acceleration=1, caption=False)
while robot.step(step) != -1 and robot.getTime() < seconds:
    pass
if movie:
    robot.movieStopRecording()
    while not robot.movieIsReady():
        if robot.step(step) == -1:
            break
    print('movie %s: %s' % ('FAILED' if robot.movieFailed() else 'written', movie))
robot.simulationQuit(0)
