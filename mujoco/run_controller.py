#!/usr/bin/env python3
"""Run one of the ported Webots controllers, unchanged, on the MuJoCo model (quad.xml).

    run_controller.py [--controller DIR] [--seconds 30] [--video out.mp4] [--viewer] ...

The controller file is executed as-is; its `from controller import Supervisor` resolves to
webots_api/controller.py, which serves the same devices and Webots frames on top of MuJoCo.
Logs (the controller's W*p CSV files) go to mujoco/data/<controller>/ unless --log-dir is given.
A video shows the controller's supervisor labels (the same HUD as in Webots, QUADSIM_LABELS) and
the engine badge in the lower right corner.
Needs the mujoco, numpy and Pillow packages (on this machine: ~/.local/opt/quadsim-venv/bin/python).
"""
import argparse
import math
import os
import runpy
import subprocess
import sys
import time

HERE = os.path.dirname(os.path.abspath(__file__))
OCT = os.path.join(HERE, '..', 'Webots_FINAL_2', 'controllers', 'controller_webots743_Oct_THALIS')


class GlideCamera:
    """The model's tracking camera, dollying smoothly along the lane instead of being bolted to the
    body: it looks at a low-pass filtered body position at a fixed height, so the body's hops and
    pitching show in the frame instead of shaking the whole view (as Webots' viewpoint follow does)."""

    def __init__(self, mujoco, model, data, name='track', tau=0.3, target_height=-0.10):
        cid = model.camera(name).id
        offset = model.cam_pos[cid].copy()                      # from the body's CoM (mode trackcom)
        forward = -model.cam_mat0[cid].reshape(3, 3)[:, 2]      # a camera looks along its -z axis
        self.cam = mujoco.MjvCamera()
        self.cam.type = mujoco.mjtCamera.mjCAMERA_FREE
        self.cam.azimuth = math.degrees(math.atan2(forward[1], forward[0]))
        self.cam.elevation = math.degrees(math.asin(forward[2]))
        self.cam.distance = (target_height - offset[2]) / forward[2]   # pivot where the tracking camera aims
        self.pivot = offset + self.cam.distance * forward                # relative to the body at rest
        self.torso, self.tau = model.body('torso').id, tau
        self.vx = model.joint('rootx').dofadr[0]
        self.height = data.subtree_com[self.torso][2]
        self.x, self.v, self.t = data.subtree_com[self.torso][0], 0.0, data.time
        self.update(data)

    def update(self, data):
        dt, self.t = data.time - self.t, data.time
        a = min(1.0, dt / self.tau)
        self.x += a * (data.subtree_com[self.torso][0] - self.x)
        self.v += a * (data.qvel[self.vx] - self.v)
        # x + tau v: no lag at steady speed, so the body stays framed as at the start
        self.cam.lookat[:] = (self.x + self.tau * self.v + self.pivot[0], self.pivot[1], self.height + self.pivot[2])
        return self.cam


def main():
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument('--controller', default=OCT, help='controller folder (default: controller_webots743_Oct_THALIS)')
    ap.add_argument('--seconds', type=float, default=30.0, help='simulated time (default 30)')
    ap.add_argument('--start-height', type=float, default=0.33, help='body height at start, m (Feb 2013 world: 0.34)')
    ap.add_argument('--start-rotation', type=float, default=0.0,
                    help='start rotation about the Webots z axis, rad (Feb 2013 world: -0.01)')
    ap.add_argument('--log-dir', help='where the controller CSV logs go')
    ap.add_argument('--video', help='record the tracking camera to this .mp4 (headless, EGL)')
    ap.add_argument('--size', default='1280x720', help='video size (default 1280x720)')
    ap.add_argument('--fps', type=int, default=30)
    ap.add_argument('--camera', default='glide',
                    help="video camera: 'glide' (default) is the model's track camera moving smoothly along the "
                         "lane; otherwise a camera of the model, e.g. 'track' (fixed to the body)")
    ap.add_argument('--badge', default='MuJoCo', help='word in the lower right corner of the video ("" for none)')
    ap.add_argument('--viewer', action='store_true', help='open the interactive MuJoCo viewer (needs a display)')
    ap.add_argument('--model', default=os.path.join(HERE, 'quad.xml'), help='MJCF file (default quad.xml)')
    args = ap.parse_args()

    video = os.path.abspath(args.video) if args.video else None
    if args.video and 'MUJOCO_GL' not in os.environ:
        os.environ['MUJOCO_GL'] = 'egl'
    import mujoco
    sys.path.insert(0, os.path.join(HERE, 'webots_api'))
    import controller as api

    model = mujoco.MjModel.from_xml_path(os.path.abspath(args.model))
    data = mujoco.MjData(model)
    data.qpos[model.joint('rootz').qposadr[0]] = args.start_height - model.body('torso').pos[2]
    data.qpos[model.joint('rooty').qposadr[0]] = args.start_rotation    # Webots z axis = MuJoCo +Y
    mujoco.mj_forward(model, data)

    hooks, closers = [], []
    if args.video:
        sys.path.insert(0, HERE)
        from hud import Hud
        w, h = (int(v) for v in args.size.split('x'))
        renderer = mujoco.Renderer(model, h, w)
        renderer.scene.flags[mujoco.mjtRndFlag.mjRND_FOG] = True   # the floor fades into the sky's haze
        glide = GlideCamera(mujoco, model, data) if args.camera == 'glide' else None
        hud = Hud(w, h, args.badge)
        ffmpeg = subprocess.Popen(['ffmpeg', '-v', 'error', '-y', '-f', 'rawvideo', '-pix_fmt', 'rgb24', '-s', '%dx%d' % (w, h),
                                   '-r', str(args.fps), '-i', '-', '-c:v', 'libx264', '-pix_fmt', 'yuv420p', '-crf', '18',
                                   video], stdin=subprocess.PIPE)
        state = {'next': 0.0}

        def record(world):
            if world.d.time + 1e-9 >= state['next']:
                renderer.update_scene(world.d, camera=glide.update(world.d) if glide else args.camera)
                renderer.scene.flags[mujoco.mjtRndFlag.mjRND_FOG] = True
                ffmpeg.stdin.write(hud.draw(renderer.render(), world.labels).tobytes())
                state['next'] += 1.0 / args.fps
        hooks.append(record)
        closers.append(lambda: (ffmpeg.stdin.close(), ffmpeg.wait()))
    if args.viewer:
        import mujoco.viewer
        viewer = mujoco.viewer.launch_passive(model, data)
        start = time.time()

        def show(world):
            if world.steps % 16 == 0:                  # ~60 Hz redraw, real-time pace
                viewer.sync()
                lag = world.d.time - (time.time() - start)
                if lag > 0:
                    time.sleep(lag)
        hooks.append(show)
        closers.append(viewer.close)

    world = api.World(model, data, args.seconds, on_step=hooks)
    folder = os.path.abspath(args.controller)
    name = os.path.basename(folder.rstrip('/'))
    os.environ.setdefault('QUADSIM_LOG_DIR', os.path.abspath(args.log_dir or os.path.join(HERE, 'data', name)))
    os.environ.setdefault('QUADSIM_AT_STOP', 'continue')
    sys.path.insert(0, folder)
    os.chdir(folder)
    t0 = time.time()
    try:
        runpy.run_path(os.path.join(folder, name + '.py'), run_name='__main__')
    finally:
        for close in closers:
            close()
    hips = [m for m in world.motors if m.name.startswith('servo')]
    print('MuJoCo run: %.1f s simulated in %.1f s; clamped requests: %d torque, %d position'
          % (data.time, time.time() - t0, sum(m.clamped_forces for m in hips), sum(m.clamped_positions for m in hips)))
    if args.video:
        print('video written to ' + video)


if __name__ == '__main__':
    main()
