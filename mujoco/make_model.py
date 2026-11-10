#!/usr/bin/env python3
"""Write quad.xml: the 2014 quadruped (SUPERVISOR_webots743_Oct.wbt) as a MuJoCo model.

Frames. Webots used a Y-up world with the robot running towards -x. MuJoCo is Z-up, so the model uses
    X = -x_webots (forward),  Y = z_webots (left),  Z = y_webots (up)
which keeps every joint sign: the hips turn about (0, -1, 0) (Webots axis 0 0 -1, positive = foot
ahead of the hip) and the leg springs slide along +Z (Webots 0 1 0, positive = compression).
webots_api/controller.py applies the same mapping to every sensor the controller reads.

Physics, all from the Webots world:
  body 10.5 kg, pitch inertia 0.8 kg m^2, 0.6 x 0.35 x 0.08 m box; start at 0.33 m
  planar motion (the physics plugin projected the body onto the sagittal plane) -> root joints
      slide X, slide Z, hinge Y instead of a free joint
  hips at x = +-0.27 m, z = +-0.195 m, range +-0.435 rad; motor: P = 60 velocity servo,
      12 rad/s, 6.5 N m; upper leg 0.07 kg, CoM 0.1233 m below the hip, inertia 0.00122 kg m^2
  springs: 3400 N/m, 8 N s/m, travel 0 to 0.035 m; lower leg 0.03 kg + foot 0.0025 kg + touch
      sensor 0.0001 kg; foot sphere r = 0.01 m, so the leg is 0.30 m from hip to ground
  1 ms controller step with 0.25 ms physics substeps; feet do not slip (Webots coulombFriction -1)
      and do not bounce
The Webots-only hacks (inertia 10 kg m^2 on the leg parts, the 1 um foot slider, separate 0.1 g
touch-sensor body) are dropped: with planar root joints they have nothing left to do.

Look (nothing here touches the physics): the showcase worlds' textures, studio sky, key and fill lights
and camera framing; MuJoCo has no tone mapping, so the lane texture gets a lighter copy in textures/.

Usage: make_model.py   (writes quad.xml and textures/lane.png next to this script)
"""
import math
import os

HERE = os.path.dirname(os.path.abspath(__file__))
WEBOTS_TEX = '../Webots_FINAL_2/worlds/textures'   # the showcase worlds' textures, relative to quad.xml
LANE_GAIN, LANE_DESATURATION = 1.17, 0.4            # lane texture adjustment for MuJoCo (see lane_texture)


def lane_texture():
    """textures/lane.png: the showcase lane texture, lighter and a little greyer. Webots' rendering
    brightens and desaturates mid tones (linear lighting with tone mapping); MuJoCo shows a texture as
    it is, so the same file would make the lane much darker and bluer than in the Webots movies. The
    numbers were set by measuring the lane in both engines' frames. Returns the path to use in
    quad.xml, falling back to the Webots texture when PIL is not installed."""
    src = os.path.join(HERE, WEBOTS_TEX, 'lane_5m.png')
    try:
        from PIL import Image
    except ImportError:
        print('PIL not installed: quad.xml uses the Webots lane texture (darker)')
        return WEBOTS_TEX + '/lane_5m.png'
    image = Image.open(src).convert('RGB')
    grey = image.convert('L').convert('RGB')
    image = Image.blend(image, grey, LANE_DESATURATION).point(lambda v: min(255, int(round(v * LANE_GAIN))))
    os.makedirs(os.path.join(HERE, 'textures'), exist_ok=True)
    image.save(os.path.join(HERE, 'textures', 'lane.png'))
    return 'textures/lane.png'

# leg -> (X, Y) of the hip in the body frame; Webots legs 1,4 are the back pair, 2,3 the front pair
HIPS = {1: (-0.27, 0.195), 2: (0.27, 0.195), 3: (0.27, -0.195), 4: (-0.27, -0.195)}
BACK, FRONT = '0.92 0.41 0.20 1', '0.16 0.47 0.84 1'


def f(*v):
    return ' '.join(('%.6g' % x) for x in v)


def camera(pos, target):
    """xyaxes of a camera at pos looking at target, horizon level (MuJoCo cameras look along -Z)."""
    fwd = [t - p for p, t in zip(pos, target)]
    n = math.sqrt(sum(c * c for c in fwd))
    fwd = [c / n for c in fwd]
    right = [fwd[1], -fwd[0], 0.0]
    n = math.sqrt(sum(c * c for c in right))
    right = [c / n for c in right]
    up = [right[1] * fwd[2] - right[2] * fwd[1], right[2] * fwd[0] - right[0] * fwd[2],
          right[0] * fwd[1] - right[1] * fwd[0]]
    return f(*(right + up))


def logo_disc(name, side, r=0.034, n=48):
    """Flat disc mesh on the body side (normal +-Y) mapping the round CSL logo onto it."""
    verts, uv, faces = [0.0, 0.0, 0.0], [0.5, 0.5], []
    for i in range(n):
        a = 2 * math.pi * i / n
        x, z = r * math.cos(a), r * math.sin(a)
        verts += [x, 0.0, z]
        uv += [0.5 - side * x / (2 * r), 0.5 - z / (2 * r)]      # upright when seen from outside
    for i in range(n):
        a, b = 1 + i, 1 + (i + 1) % n
        faces += [0, a, b] if side < 0 else [0, b, a]
    return ('    <mesh name="%s" vertex="%s" texcoord="%s" face="%s" inertia="shell"/>'
            % (name, f(*verts), f(*uv), ' '.join(str(i) for i in faces)))


def leg(i):
    x, y = HIPS[i]
    accent = BACK if i in (1, 4) else FRONT
    s = y / abs(y)                                  # +1 on the left side, -1 on the right
    return '''
      <body name="leg{i}" pos="{x} {y} -0.04">
        <joint name="hip{i}" type="hinge" pos="0 0 0.04" axis="0 -1 0" range="-0.435 0.435" limited="true"/>
        <inertial pos="0 0 -0.0833" mass="0.07" diaginertia="0.00122 0.00122 0.00122"/>
        <site name="gyro{i}" pos="0 0 0.04"/>
        <geom class="visual" type="cylinder" pos="0 {my:.4f} 0.04" euler="1.5707963 0 0" size="0.022 0.022" material="anodized"/>
        <geom class="visual" type="cylinder" pos="0 {ring:.4f} 0.04" euler="1.5707963 0 0" size="0.0225 0.003" rgba="{accent}"/>
        <geom class="visual" type="cylinder" pos="0 {cap:.4f} 0.04" euler="1.5707963 0 0" size="0.015 0.002" material="aluminium"/>
        <geom class="visual" type="box" pos="0 {mark:.4f} 0.032" size="0.0018 0.0006 0.008" rgba="0.97 0.97 0.97 1"/>
        <geom class="visual" type="cylinder" fromto="0 0 0.03 0 0 -0.115" size="0.0085" material="carbon"/>
        <geom class="visual" type="cylinder" fromto="0 0 0.014 0 0 0.010" size="0.0092" rgba="{accent}"/>
        <geom class="visual" type="cylinder" fromto="0 0 -0.107 0 0 -0.115" size="0.0105" material="aluminium"/>
        <body name="shank{i}">
          <joint name="spring{i}" type="slide" axis="0 0 1" range="0 0.035" limited="true"
                 stiffness="3400" damping="8" springref="0" solreflimit="0.0005 1"/>
          <inertial pos="0 0 -0.0838" mass="0.0326" diaginertia="1e-7 1e-7 1e-7"/>
          <geom class="visual" type="cylinder" fromto="0 0 -0.115 0 0 -0.25" size="0.005" material="chrome"/>
          <geom class="visual" type="cylinder" fromto="0 0 -0.147 0 0 -0.153" size="0.0095" rgba="0.92 0.92 0.92 1"/>
          <geom class="visual" type="cylinder" fromto="0 0 -0.232 0 0 -0.244" size="0.0072" material="rubber"/>
          <geom name="foot{i}" class="foot" type="sphere" pos="0 0 -0.25" size="0.01"/>
          <site name="foot{i}" pos="0 0 -0.25" size="0.012"/>
        </body>
      </body>'''.format(i=i, x=x, y=y, accent=accent, my=0.0, ring=s * 0.012, cap=s * 0.0235,
                        mark=s * 0.0258)


def model(start_height=0.33, lane='textures/lane.png'):
    cam_pos, cam_target = (1.55, 2.05, 0.40), (0.30, 0.0, -0.10)   # same framing as the showcase worlds
    return '''<mujoco model="NTUA CSL quadruped (2014 Webots model)">
  <!-- generated by make_model.py: see that file for the frame mapping and where each number comes from -->
  <compiler angle="radian" texturedir="." inertiafromgeom="false"/>
  <!-- 0.25 ms physics steps (4 per 1 ms controller step) keep the spring stops close to Webots' rigid
       ones: MuJoCo limits need a time constant of at least two steps -->
  <option timestep="0.00025" gravity="0 0 -9.81" integrator="implicitfast" cone="elliptic"/>
  <!-- robot-scale extent: it sets the near/far clipping planes and the shadow map size -->
  <statistic extent="1" center="0 0 0.3"/>
  <visual>
    <!-- fovy 25 = the showcase worlds' 0.75 rad horizontal field of view on a 16:9 frame; the fog
         (run_controller.py turns it on for videos) fades the floor into the haze colour of the sky -->
    <global offwidth="1920" offheight="1080" fovy="25" azimuth="120" elevation="-12"/>
    <quality shadowsize="8192" offsamples="8"/>
    <headlight ambient="0.3 0.3 0.31" diffuse="0.18 0.18 0.18" specular="0 0 0"/>
    <map znear="0.01" zfar="200" haze="0.3" shadowclip="4" fogstart="30" fogend="170"/>
    <rgba haze="0.86 0.885 0.915 1" fog="0.86 0.885 0.915 1"/>
  </visual>
  <default>
    <geom contype="0" conaffinity="1" condim="3" friction="1.5 0.005 0.0001" solref="0.0005 1"/>
    <default class="visual"><geom contype="0" conaffinity="0" group="1" mass="0"/></default>
    <default class="foot"><geom material="rubber" priority="1"/></default>
  </default>
  <asset>
    <texture name="sky" type="skybox" fileright="{tex}/studio_sky_right.png" fileleft="{tex}/studio_sky_left.png"
             fileup="{tex}/studio_sky_top.png" filedown="{tex}/studio_sky_bottom.png"
             filefront="{tex}/studio_sky_front.png" fileback="{tex}/studio_sky_back.png"/>
    <texture name="floor" type="2d" file="{tex}/floor_tiles.png"/>
    <texture name="lane" type="2d" file="{lane}"/>
    <texture name="alum" type="2d" file="{tex}/alum.png"/>
    <texture name="logo" type="2d" file="{tex}/ntua3.png"/>
    <material name="floor" texture="floor" texrepeat="1 1" texuniform="true" reflectance="0.02"/>
    <material name="lane" texture="lane" texrepeat="40 1" specular="0.1" reflectance="0.1"/>
    <material name="aluminium" texture="alum" rgba="0.86 0.87 0.89 1" specular="0.7" shininess="0.6"/>
    <material name="anodized" rgba="0.12 0.12 0.13 1" specular="0.4" shininess="0.5"/>
    <material name="carbon" rgba="0.07 0.07 0.08 1" specular="0.3" shininess="0.4"/>
    <material name="chrome" rgba="0.9 0.9 0.92 1" specular="1" shininess="0.9" reflectance="0.2"/>
    <material name="rubber" rgba="0.04 0.04 0.04 1" specular="0.05" shininess="0.1"/>
    <material name="logo" texture="logo" specular="0.3" shininess="0.4"/>
{logo_l}
{logo_r}
  </asset>
  <worldbody>
    <!-- the showcase worlds' lights in MuJoCo axes: warm key from the camera side, cool fill from behind -->
    <light name="fill" directional="true" castshadow="false" pos="0 0 5" dir="0.6 0.7 -0.7" diffuse="0.26 0.28 0.32" specular="0 0 0"/>
    <geom name="floor" type="plane" size="150 150 0.05" material="floor" contype="1" conaffinity="1"/>
    <!-- a (non-colliding) plane rather than a box: MuJoCo mirrors the scene in reflective planes -->
    <geom name="lane" class="visual" type="plane" pos="95 0 0.0015" size="100 0.4 1" material="lane"/>
    <geom name="crate" type="box" pos="-1 0 0.16" size="0.16 0.16 0.16" rgba="0.6 0.43 0.26 1" contype="1" conaffinity="1"/>
    <body name="torso" pos="0 0 {h}">
      <joint name="rootx" type="slide" axis="1 0 0"/>
      <joint name="rootz" type="slide" axis="0 0 1"/>
      <joint name="rooty" type="hinge" axis="0 1 0"/>
      <inertial pos="0 0 0" mass="10.5" diaginertia="10 0.8 10"/>
      <geom name="torso" type="box" size="0.3 0.175 0.04" material="aluminium"/>
      <geom class="visual" type="box" pos="-0.02 0 0.056" size="0.12 0.085 0.016" material="anodized"/>
      <geom class="visual" type="box" pos="0.08 0.06 0.0735" size="0.006 0.006 0.002" rgba="0.2 0.75 0.35 1"/>
      <geom class="visual" type="box" pos="0.3065 0 0" size="0.0065 0.16 0.025" rgba="0.05 0.05 0.055 1"/>
      <geom class="visual" type="box" pos="-0.3065 0 0" size="0.0065 0.16 0.025" rgba="0.05 0.05 0.055 1"/>
      <geom class="visual" type="box" pos="0 0.1755 0.0385" size="0.3 0.003 0.003" rgba="0.05 0.05 0.055 1"/>
      <geom class="visual" type="box" pos="0 -0.1755 0.0385" size="0.3 0.003 0.003" rgba="0.05 0.05 0.055 1"/>
      <geom class="visual" type="mesh" mesh="logo_left" pos="0 0.1762 0" material="logo"/>
      <geom class="visual" type="mesh" mesh="logo_right" pos="0 -0.1762 0" material="logo"/>
      <light name="sun" mode="trackcom" directional="true" castshadow="true" pos="0.75 0.9 1.65" dir="-0.45 -0.55 -1"
             diffuse="0.9 0.87 0.82" specular="0.4 0.4 0.4"/>
      <site name="imu" pos="0 0 0"/>
      <site name="height_sensor" pos="0 0 -0.04"/>
      <camera name="track" mode="trackcom" pos="{cam_pos}" xyaxes="{cam_axes}" fovy="25"/>
{legs}
    </body>
  </worldbody>
  <actuator>
{actuators}
  </actuator>
  <sensor>
    <gyro name="CoM_Gyro" site="imu"/>
    <accelerometer name="CoM_Accelerometer" site="imu"/>
    <framepos name="CoM_Gps" objtype="site" objname="imu"/>
{sensors}
  </sensor>
</mujoco>
'''.format(tex=WEBOTS_TEX, lane=lane, h=f(start_height), logo_l=logo_disc('logo_left', 1), logo_r=logo_disc('logo_right', -1),
           legs=''.join(leg(i) for i in (1, 2, 3, 4)), cam_pos=f(*cam_pos), cam_axes=camera(cam_pos, cam_target),
           actuators='\n'.join(
               '    <velocity name="hip{i}_vel" joint="hip{i}" kv="15" forcelimited="true" forcerange="-6.5 6.5"/>\n'
               '    <motor name="hip{i}_torque" joint="hip{i}" forcelimited="true" forcerange="-6.5 6.5"/>'.format(i=i)
               for i in (1, 2, 3, 4)),
           sensors='\n'.join('    <gyro name="leg_gyro_{i}_1" site="gyro{i}"/>\n    <touch name="TOUCHSENSOR{i}" site="foot{i}"/>'
                             .format(i=i) for i in (1, 2, 3, 4)))


if __name__ == '__main__':
    path = os.path.join(HERE, 'quad.xml')
    open(path, 'w').write(model(lane=lane_texture()))
    print('wrote', path)
