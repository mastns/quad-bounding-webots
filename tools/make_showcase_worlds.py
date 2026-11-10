#!/usr/bin/env python3
"""Write *_showcase.wbt copies with a cleaner look. Only visual nodes change; physics stays the same.

Robot: the shapes that double as bounding objects (body box, upper-leg rod, lower-leg rod, foot
sphere) keep their geometry and only get new PBR appearances. Purely visual parts (spring case,
servo spheres) are replaced, and visual-only parts are added as siblings, never inside a node that
is used as a boundingObject: hip motors, a carbon sleeve, a chrome rod, collars that show the spring
travel, a top plate, and the NTUA Control Systems Lab logo on both sides.
Scene: soft gradient sky with image-based lighting, a sun with shadows plus a fill light, a light
floor with a 1 m grid, a running lane with metre marks along the robot's path, a wooden crate, and a
3/4 tracking camera.

Usage: make_showcase_worlds.py [WORLD.wbt ...]   (default: the three *_tuned.wbt worlds)
"""
import math
import os
import random
import struct
import sys

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
from convert_v7_world import MF, SF, Node, Parser, axis_angle, num, sf, string, write_node  # noqa: E402
from make_textures import write_png  # noqa: E402

ROOT = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
DEFAULT = ['Webots_FINAL_2/worlds/SUPERVISOR_webots743_Oct_tuned.wbt',
           'Webots_FINAL_2/worlds/SUPERVISOR_webots743_May_(flat)_tuned.wbt',
           'Webots_FINAL/worlds/SUPERVISOR_webots730_Feb_FINAL_tuned.wbt']
HALF_PI = math.pi / 2


# ----------------------------------------------------------------------------- node helpers
def image(name, repeat=True):
    t = Node('ImageTexture')
    t.set('url', MF([['"textures/%s"' % name]]))
    if not repeat:
        t.set('repeatS', sf('FALSE'))
        t.set('repeatT', sf('FALSE'))
    return t


def pbr(color, roughness, metalness, texture=None, scale=None, ibl=None, name=None):
    a = Node('PBRAppearance')
    a.set('baseColor', sf(*[num(c) for c in color]))
    if texture:
        a.set('baseColorMap', image(texture))
    if scale:
        tt = Node('TextureTransform')
        tt.set('scale', sf(*[num(s) for s in scale]))
        a.set('textureTransform', tt)
    a.set('roughness', sf(num(roughness)))
    a.set('metalness', sf(num(metalness)))
    if ibl is not None:
        a.set('IBLStrength', sf(num(ibl)))
    if name:
        a.set('name', string(name))
    return a


def shape(appearance, geometry, cast_shadows=True):
    s = Node('Shape')
    s.set('appearance', appearance)
    s.set('geometry', geometry)
    if not cast_shadows:
        s.set('castShadows', sf('FALSE'))
    return s


def cylinder(radius, height, sub=32):
    c = Node('Cylinder')
    c.set('height', sf(num(height)))
    c.set('radius', sf(num(radius)))
    c.set('subdivision', sf(sub))
    return c


def box(x, y, z):
    b = Node('Box')
    b.set('size', sf(num(x), num(y), num(z)))
    return b


def pose(children, translation=None, rotation=None, def_name=None):
    p = Node('Pose', def_name)
    if translation:
        p.set('translation', sf(*[num(v) for v in translation]))
    if rotation:
        p.set('rotation', sf(*[num(v) for v in rotation]))
    p.set('children', MF(children))
    return p


def cyl_y(radius, height, y, appearance, x=0.0, z=0.0):
    """A cylinder along the parent's y axis (R2025a cylinders run along z)."""
    return pose([shape(appearance, cylinder(radius, height))], (x, y, z), (1, 0, 0, -HALF_PI))


def look_at(position, target):
    """Viewpoint orientation (camera looks along its +x, z up) in a Y-up world."""
    f = [t - p for p, t in zip(position, target)]
    n = math.sqrt(sum(v * v for v in f))
    f = [v / n for v in f]
    up = (0.0, 1.0, 0.0)
    left = [up[1] * f[2] - up[2] * f[1], up[2] * f[0] - up[0] * f[2], up[0] * f[1] - up[1] * f[0]]
    n = math.sqrt(sum(v * v for v in left))
    left = [v / n for v in left]
    z = [f[1] * left[2] - f[2] * left[1], f[2] * left[0] - f[0] * left[2], f[0] * left[1] - f[1] * left[0]]
    m = [[f[i], left[i], z[i]] for i in range(3)]
    axis, angle = axis_angle(m)
    return [axis[0], axis[1], axis[2], angle]


# ----------------------------------------------------------------------------- materials
ALUMINIUM = lambda: pbr((0.86, 0.87, 0.89), 0.32, 0.85, 'alum.png', name='brushed aluminium')  # noqa: E731
DARK_ANODIZED = lambda: pbr((0.11, 0.11, 0.12), 0.38, 0.55, name='black anodized')  # noqa: E731
CARBON = lambda: pbr((0.06, 0.06, 0.07), 0.42, 0.15, name='carbon')  # noqa: E731
CHROME = lambda: pbr((0.93, 0.93, 0.95), 0.12, 1.0, name='chrome')  # noqa: E731
RUBBER = lambda: pbr((0.035, 0.035, 0.035), 0.88, 0.0, name='rubber')  # noqa: E731
BACK_ACCENT = (0.95, 0.42, 0.08)    # back legs (1, 4)
FRONT_ACCENT = (0.12, 0.47, 0.90)   # front legs (2, 3)


def accent(color):
    return pbr(color, 0.45, 0.0, name='accent')


# ----------------------------------------------------------------------------- textures
def write_hdr(path, size, radiance):
    """Uncompressed Radiance RGBE image; radiance(x, y) -> linear (r, g, b)."""
    rows = []
    for y in range(size):
        row = bytearray()
        for x in range(size):
            r, g, b = radiance(x, y)
            v = max(r, g, b)
            if v < 1e-32:
                row.extend((0, 0, 0, 0))
            else:
                m, e = math.frexp(v)
                s = m * 256.0 / v
                row.extend((int(r * s), int(g * s), int(b * s), e + 128))
        rows.append(bytes(row))
    with open(path, 'wb') as f:
        f.write(b'#?RADIANCE\nFORMAT=32-bit_rle_rgbe\n\n-Y %d +X %d\n' % (size, size))
        f.write(b''.join(rows))


def face_elevation(face, x, y, size):
    """Elevation angle of the cube-map texel (Y-up cube)."""
    u = (x + 0.5) / size * 2 - 1
    v = 1 - (y + 0.5) / size * 2
    if face == 'top':
        return math.atan2(1, math.hypot(u, v))
    if face == 'bottom':
        return -math.atan2(1, math.hypot(u, v))
    return math.atan2(v, math.hypot(u, 1))


def lerp(a, b, t):
    return tuple(a[i] + (b[i] - a[i]) * t for i in range(3))


def sky_color(e):
    if e >= 0:
        return lerp((0.93, 0.95, 0.97), (0.60, 0.74, 0.90), (e / HALF_PI) ** 0.55)
    return lerp((0.88, 0.89, 0.90), (0.62, 0.63, 0.65), (-e / HALF_PI) ** 0.5)


def sky_radiance(e):
    if e >= 0:
        return lerp((1.05, 1.05, 1.05), (0.95, 1.05, 1.25), (e / HALF_PI) ** 0.7)
    return lerp((0.85, 0.85, 0.85), (0.32, 0.31, 0.30), (-e / HALF_PI) ** 0.5)


def make_textures(tex_dir):
    rnd = random.Random(7)
    faces = ('front', 'back', 'left', 'right', 'top', 'bottom')
    for face in faces:
        write_png(os.path.join(tex_dir, 'studio_sky_%s.png' % face), 256, 256,
                  lambda x, y, face=face: tuple(int(255 * c) for c in sky_color(face_elevation(face, x, y, 256))))
        write_hdr(os.path.join(tex_dir, 'studio_sky_%s.hdr' % face), 32,
                  lambda x, y, face=face: sky_radiance(face_elevation(face, x, y, 32)))
    noise = [[rnd.uniform(-1, 1) for _ in range(64)] for _ in range(64)]

    def floor(x, y, n=512):                      # one 1 m tile: light concrete, 1 m and 25 cm lines
        base = 0.66 + 0.018 * noise[(y * 64) // n][(x * 64) // n] + rnd.uniform(-0.012, 0.012)
        if x < 3 or y < 3 or x >= n - 3 or y >= n - 3:
            base -= 0.13
        elif x % 128 in (0, 127) or y % 128 in (0, 127):
            base -= 0.045
        c = int(255 * max(0.0, min(1.0, base)))
        return (c, c, min(255, c + 3))
    write_png(os.path.join(tex_dir, 'floor_tiles.png'), 512, 512, floor)

    def lane(x, y, w=1024, h=256):               # 5 m of lane along x (u), 0.8 m across (v)
        if y < 7 or y >= h - 7:
            return (236, 236, 234)                # edge lines
        metre = x / (w / 5.0)
        if metre % 1.0 < 0.015 or metre < 0.035:  # 1.5 cm tick every metre, 3.5 cm at the 5 m mark
            return (228, 228, 226)
        g = 0.43 + rnd.uniform(-0.012, 0.012)
        return (int(255 * g * 0.93), int(255 * g * 1.0), int(255 * g * 1.10))
    write_png(os.path.join(tex_dir, 'lane_5m.png'), 1024, 256, lane)

    def crate(x, y, n=256):                       # four planks with gaps and grain
        plank = y // (n // 4)
        if y % (n // 4) < 3:
            return (40, 26, 16)
        grain = 0.06 * math.sin(x * 0.11 + plank * 2.3 + 3 * math.sin(y * 0.05 + plank)) + rnd.uniform(-0.03, 0.03)
        r, g, b = 0.60 + grain, 0.42 + grain * 0.8, 0.25 + grain * 0.5
        return tuple(int(255 * max(0, min(1, c))) for c in (r, g, b))
    write_png(os.path.join(tex_dir, 'crate_wood.png'), 256, 256, crate)


# ----------------------------------------------------------------------------- scene
def background():
    b = Node('Background')
    b.set('skyColor', MF([['0.75', '0.84', '0.94']]))
    for face in ('back', 'bottom', 'front', 'left', 'right', 'top'):
        b.set('%sUrl' % face, MF([['"textures/studio_sky_%s.png"' % face]]))
    for face in ('back', 'bottom', 'front', 'left', 'right', 'top'):
        b.set('%sIrradianceUrl' % face, MF([['"textures/studio_sky_%s.hdr"' % face]]))
    b.set('luminosity', sf('0.8'))
    return b


def lights():
    sun = Node('DirectionalLight')
    sun.set('color', sf('1', '0.97', '0.92'))
    sun.set('direction', sf('0.45', '-1', '-0.55'))
    sun.set('intensity', sf('2.4'))
    sun.set('castShadows', sf('TRUE'))
    fill = Node('DirectionalLight')
    fill.set('color', sf('0.88', '0.93', '1'))
    fill.set('direction', sf('-0.6', '-0.7', '0.7'))
    fill.set('intensity', sf('0.5'))
    return [sun, fill]


def restyle_floor(floor):
    visual = floor.get('children').items[0]           # Pose { rotation 1 0 0 -pi/2 children [ Shape ] }
    plane_shape = visual.get('children').items[0]
    plane_shape.set('appearance', pbr((1, 1, 1), 0.9, 0.0, 'floor_tiles.png', (400, 400), name='lab floor'))
    plane = plane_shape.get('geometry')
    plane.set('size', sf('400', '400'))
    plane_shape.set('castShadows', sf('FALSE'))
    # running lane: 0.8 m wide along the robot's path (it runs towards -x), 1 mm above the floor
    x0, x1, w, y = -195.0, 5.0, 0.4, 0.001
    coord = Node('Coordinate')
    coord.set('point', MF([[num(x1), num(y), num(w)], [num(x1), num(y), num(-w)],
                           [num(x0), num(y), num(-w)], [num(x0), num(y), num(w)]]))
    tex = Node('TextureCoordinate')
    tex.set('point', MF([['0', '1'], ['0', '0'], [num((x1 - x0) / 5), '0'], [num((x1 - x0) / 5), '1']]))
    ifs = Node('IndexedFaceSet')
    ifs.set('coord', coord)
    ifs.set('texCoord', tex)
    ifs.set('coordIndex', MF([['0', '1', '2', '3', '-1']]))
    ifs.set('texCoordIndex', MF([['0', '1', '2', '3', '-1']]))
    lane = shape(pbr((1, 1, 1), 0.75, 0.0, 'lane_5m.png', name='lane'), ifs, cast_shadows=False)
    floor.get('children').items.append(lane)


def restyle_crate(crate):
    crate.get('children').items[0].set('appearance', pbr((1, 1, 1), 0.7, 0.0, 'crate_wood.png', name='wood'))


# ----------------------------------------------------------------------------- robot
def find(items, def_name):
    for i in items:
        if isinstance(i, Node) and i.def_name == def_name:
            return i
    return None


def restyle_upper_leg(link, leg):
    kids = link.get('children').items
    color = BACK_ACCENT if leg in (1, 4) else FRONT_ACCENT
    upp = find(kids, 'UPP_LEG')                     # bounding object: appearance only
    if upp is not None:
        upp.get('children').items[0].set('appearance', CARBON())
    servo = find(kids, 'SERVO')                     # visual only: hip motor at the joint
    if servo is not None:
        motor = [shape(DARK_ANODIZED(), cylinder(0.022, 0.044)),
                 pose([shape(accent(color), cylinder(0.0225, 0.006))], (0, 0, 0.012)),
                 pose([shape(ALUMINIUM(), cylinder(0.015, 0.004))], (0, 0, 0.0235)),
                 pose([shape(pbr((0.97, 0.97, 0.97), 0.5, 0.0), box(0.0035, 0.016, 0.0012))], (0, -0.008, 0.0258))]
        servo.set('children', MF(motor))
    case = find(kids, 'SPRING_CASE_TRANS')          # visual only: carbon sleeve with a collar
    if case is not None:
        # leg frame: hip at y = +0.04, original spring case from -0.04 down to -0.115
        sleeve = pose([cyl_y(0.0085, 0.145, -0.0425, CARBON()),
                       cyl_y(0.0105, 0.008, -0.111, ALUMINIUM()),
                       cyl_y(0.0092, 0.004, 0.012, accent(color))], def_name='SPRING_CASE_TRANS')
        kids[kids.index(case)] = sleeve


def restyle_spring_link(link):
    kids = link.get('children').items
    leg = find(kids, 'LEG_TRANS')                   # bounding object: appearance only
    if leg is None:
        return False
    leg.get('children').items[0].set('appearance', CHROME())
    kids.append(cyl_y(0.005, 0.135, -0.1825, CHROME()))            # visual rod around it
    kids.append(cyl_y(0.0095, 0.006, -0.150, accent((0.92, 0.92, 0.92))))  # stop collar: spring travel
    return True


def restyle_foot(foot):
    kids = foot.get('children').items
    trans = find(kids, 'FOOT_TRANS')                # bounding object: appearance only
    if trans is None:
        return
    trans.get('children').items[0].set('appearance', RUBBER())
    kids.append(cyl_y(0.0072, 0.012, -0.238, RUBBER()))             # boot above the foot sphere


def restyle_robot(robot):
    kids = robot.get('children').items
    body = find(kids, 'ROBOT_SHAPE')                # bounding object: appearance only
    body.set('appearance', ALUMINIUM())
    logo = lambda: pbr((1, 1, 1), 0.45, 0.25, 'ntua3.png', name='CSL logo')  # noqa: E731
    trim = lambda: pbr((0.05, 0.05, 0.055), 0.6, 0.0, name='trim')  # noqa: E731
    # body is 0.6 x 0.08 x 0.35 (x forward-back, y up, z sideways); everything here sits outside it
    kids.append(pose([shape(DARK_ANODIZED(), box(0.24, 0.032, 0.17))], (0.02, 0.056, 0)))   # electronics module
    kids.append(pose([shape(accent((0.20, 0.75, 0.35)), box(0.012, 0.004, 0.012))], (-0.08, 0.0735, 0.06)))  # status LED
    for x in (0.3065, -0.3065):                     # front/back bumpers
        kids.append(pose([shape(trim(), box(0.013, 0.05, 0.32))], (x, 0, 0)))
    for z in (0.1755, -0.1755):                     # trims along the top edges
        kids.append(pose([shape(trim(), box(0.6, 0.006, 0.006))], (0, 0.0385, z)))
    for side in (1, -1):                            # NTUA Control Systems Lab logo on both sides
        rotation = None if side > 0 else (0, 1, 0, math.pi)
        kids.append(pose([shape(logo(), cylinder(0.034, 0.002))], (0, 0, side * 0.1762), rotation))
    for joint in kids:
        if isinstance(joint, Node) and joint.type == 'HingeJoint' and (joint.def_name or '').startswith('SERVO'):
            leg = int(joint.def_name[-1])
            link = joint.get('endPoint')
            restyle_upper_leg(link, leg)
            for spring in link.get('children').items:
                if isinstance(spring, Node) and spring.type == 'SliderJoint':
                    s_link = spring.get('endPoint')
                    if restyle_spring_link(s_link):
                        for foot_joint in s_link.get('children').items:
                            if isinstance(foot_joint, Node) and foot_joint.type == 'SliderJoint':
                                restyle_foot(foot_joint.get('endPoint'))


# ----------------------------------------------------------------------------- world
def restyle(src, dst):
    text = open(src).read()
    header = [line for line in text.split('\n') if line.startswith('#')]
    nodes = Parser(text).parse_world()
    out_nodes = []
    for n in nodes:
        if not isinstance(n, Node):
            out_nodes.append(n)
        elif n.type in ('DirectionalLight', 'PointLight'):
            continue
        elif n.type == 'Background':
            out_nodes.append(background())
            out_nodes.extend(lights())
        elif n.type == 'Viewpoint':
            robot = next(r for r in nodes if isinstance(r, Node) and r.def_name == 'QUAD_ROBOT')
            rx, ry, rz = (float(v) for v in robot.get('translation').tokens)
            cam = (rx - 1.55, ry + 0.40, rz + 2.05)
            n.set('fieldOfView', sf('0.75'))
            n.set('orientation', SF([num(v) for v in look_at(cam, (rx - 0.30, ry - 0.10, rz))]))
            n.set('followSmoothness', sf('0.05'))
            n.set('position', SF([num(v) for v in cam]))
            n.set('near', sf('0.02'))
            out_nodes.append(n)
        elif n.type == 'Solid' and n.get('name') and n.get('name').tokens[0] == '"floor"':
            restyle_floor(n)
            out_nodes.append(n)
        elif n.type == 'Solid' and n.get('name') and n.get('name').tokens[0] == '"wood box"':
            restyle_crate(n)
            out_nodes.append(n)
        elif n.def_name == 'QUAD_ROBOT':
            restyle_robot(n)
            out_nodes.append(n)
        else:
            out_nodes.append(n)
    out = [header[0], '# Showcase copy of %s: new look, same physics (tools/make_showcase_worlds.py)'
           % os.path.basename(src)] + header[1:]
    for n in out_nodes:
        write_node(n, 0, out)
    open(dst, 'w', newline='\n').write('\n'.join(out) + '\n')


def main():
    worlds = sys.argv[1:] or [os.path.join(ROOT, w) for w in DEFAULT]
    done_dirs = set()
    for w in worlds:
        tex_dir = os.path.join(os.path.dirname(os.path.abspath(w)), 'textures')
        if tex_dir not in done_dirs:
            make_textures(tex_dir)
            done_dirs.add(tex_dir)
        dst = w[:-4].replace('_tuned', '') + '_showcase.wbt'
        restyle(w, dst)
        print('wrote', os.path.relpath(dst, ROOT))


if __name__ == '__main__':
    main()
