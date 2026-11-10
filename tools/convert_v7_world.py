#!/usr/bin/env python3
"""Upgrade the 2013-2014 quadruped worlds (Webots 7.3/7.4, "#VRML_SIM V7.x") to Webots R2025a.

The world keeps its original Y-up frame (WorldInfo coordinateSystem "NUE"), so the robot
geometry, the sensor axes, the physics plugin and the controller math stay unchanged.

Changes applied, oldest Webots change first:
  * Supervisor -> Robot { supervisor TRUE }; showRobotWindow dropped (R2020b / R2018).
  * Joint "device" becomes a list holding the motor plus a PositionSensor named
    "<motor name>_sensor": Webots 8 moved position feedback out of the motors
    (wb_motor_enable_position / wb_motor_get_position no longer exist).
  * Every Cylinder gets wrapped in Pose { rotation 1 0 0 -pi/2 }: R2022a turned the cylinder
    axis from local y to local z. A DEF on the Shape moves to the wrapper, so USEs (for example
    "boundingObject USE UPP_LEG") keep the old orientation.
  * Viewpoint orientation re-expressed for the R2022a camera (looks along +x with z up,
    instead of -z with y up).
  * Transform -> Pose (a Transform may no longer be a boundingObject since R2023b).
  * PROTOs that no longer exist are replaced by plain nodes, so the worlds need no download:
    Floor / MetricFloor -> Solid with an infinite Plane, WoodBox -> static box Solid,
    SphericalSkyDome dropped (the Background sky colour stays).
  * Absolute Windows texture paths -> "textures/<file>".
  * Fields that no longer exist are dropped and reported; a single value given to a field that
    is now multi-valued is put in brackets (ContactProperties.coulombFriction).

Usage: convert_v7_world.py IN.wbt OUT.wbt [--controller NAME] [--physics NAME]
"""

import argparse
import math
import os
import re
import sys

WEBOTS_HOME = os.environ.get('WEBOTS_HOME', os.path.expanduser('~/webots'))
NODE_SPEC_DIR = os.path.join(WEBOTS_HOME, 'resources', 'nodes')
HALF_PI = math.pi / 2


# ----------------------------------------------------------------------------- tree
class Node:
    def __init__(self, type_, def_name=None):
        self.type = type_
        self.def_name = def_name
        self.fields = []  # [name, value] pairs, file order

    def get(self, name, default=None):
        for f in self.fields:
            if f[0] == name:
                return f[1]
        return default

    def set(self, name, value):
        for f in self.fields:
            if f[0] == name:
                f[1] = value
                return
        self.fields.append([name, value])

    def remove(self, name):
        self.fields = [f for f in self.fields if f[0] != name]


class Use:
    def __init__(self, name):
        self.name = name


class SF:
    """Single value: a list of tokens (numbers, strings, TRUE/FALSE)."""

    def __init__(self, tokens):
        self.tokens = list(tokens)


class MF:
    """Multiple value: items are Node / Use, or token lists (one list per printed line)."""

    def __init__(self, items=None):
        self.items = items if items is not None else []


def sf(*tokens):
    return SF([str(t) for t in tokens])


def num(x):
    s = ('%.6f' % x).rstrip('0').rstrip('.')
    return '0' if s in ('-0', '') else s


def string(s):
    return SF(['"%s"' % s])


# ----------------------------------------------------------------------------- parsing
def tokenize(text):
    tokens = []
    i, n, line = 0, len(text), 1
    while i < n:
        c = text[i]
        if c == '\n':
            line += 1
            i += 1
        elif c in ' \t\r,':
            i += 1
        elif c == '#':
            while i < n and text[i] != '\n':
                i += 1
        elif c in '{}[]':
            tokens.append((c, line))
            i += 1
        elif c == '"':
            j = i + 1
            while text[j] != '"':
                if text[j] == '\\':
                    j += 1
                elif text[j] == '\n':
                    line += 1
                j += 1
            tokens.append((text[i:j + 1], line))
            i = j + 1
        else:
            j = i
            while j < n and text[j] not in ' \t\r\n,{}[]"#':
                j += 1
            tokens.append((text[i:j], line))
            i = j
    return tokens


def is_value_token(tok):
    return tok[0] == '"' or tok[0] in '0123456789-+.' or tok in ('TRUE', 'FALSE')


class Parser:
    def __init__(self, text):
        self.toks = tokenize(text)
        self.pos = 0

    def peek(self, k=0):
        p = self.pos + k
        return self.toks[p][0] if p < len(self.toks) else None

    def take(self, expected=None):
        tok = self.toks[self.pos][0]
        if expected is not None and tok != expected:
            raise SyntaxError('line %d: expected %r, got %r' % (self.toks[self.pos][1], expected, tok))
        self.pos += 1
        return tok

    def parse_world(self):
        nodes = []
        while self.peek() is not None:
            nodes.append(self.parse_node())
        return nodes

    def parse_node(self):
        tok = self.peek()
        if tok == 'USE':
            self.take()
            return Use(self.take())
        if tok == 'NULL':
            self.take()
            return None
        def_name = None
        if tok == 'DEF':
            self.take()
            def_name = self.take()
        node = Node(self.take(), def_name)
        self.take('{')
        while self.peek() != '}':
            name = self.take()
            node.fields.append([name, self.parse_value()])
        self.take('}')
        return node

    def starts_node(self):
        tok = self.peek()
        return tok in ('DEF', 'USE', 'NULL') or (tok[0].isupper() and self.peek(1) == '{')

    def parse_value(self):
        if self.peek() == '[':
            self.take()
            items, cur, cur_line = [], [], None
            while self.peek() != ']':
                if self.starts_node():
                    if cur:
                        items.append(cur)
                        cur = []
                    items.append(self.parse_node())
                else:
                    line = self.toks[self.pos][1]
                    if cur and line != cur_line:
                        items.append(cur)
                        cur = []
                    cur_line = line
                    cur.append(self.take())
            if cur:
                items.append(cur)
            self.take(']')
            return MF(items)
        if self.starts_node():
            return self.parse_node()
        tokens = []
        while self.peek() is not None and is_value_token(self.peek()):
            tokens.append(self.take())
        if not tokens:
            raise SyntaxError('line %d: empty value' % self.toks[self.pos][1])
        return SF(tokens)


# ----------------------------------------------------------------------------- writing
def write_node(node, indent, out, lead=''):
    pad = ' ' * indent
    if node is None:
        out.append(lead + 'NULL')
        return
    if isinstance(node, Use):
        out.append(lead + 'USE ' + node.name)
        return
    head = ('DEF %s ' % node.def_name if node.def_name else '') + node.type + ' {'
    out.append(lead + head)
    for name, value in node.fields:
        write_field(name, value, indent + 2, out)
    out.append(pad + '}')


def write_field(name, value, indent, out):
    pad = ' ' * indent
    if isinstance(value, SF):
        out.append(pad + name + ' ' + ' '.join(value.tokens))
    elif isinstance(value, MF):
        if not value.items:
            out.append(pad + name + ' []')
            return
        out.append(pad + name + ' [')
        for item in value.items:
            if isinstance(item, list):
                out.append(pad + '  ' + ' '.join(item))
            else:
                write_node(item, indent + 2, out, pad + '  ')
        out.append(pad + ']')
    else:  # SFNode
        write_node(value, indent, out, pad + name + ' ')


# ----------------------------------------------------------------------------- helpers
def walk(nodes, fn, parent=None, field=None):
    """Depth-first, document order. fn(node, parent, field_name)."""
    for n in nodes:
        if isinstance(n, Node):
            fn(n, parent, field)
            for fname, value in list(n.fields):
                if isinstance(value, Node):
                    walk([value], fn, n, fname)
                elif isinstance(value, MF):
                    walk([i for i in value.items if isinstance(i, Node)], fn, n, fname)


def rot_matrix(axis, angle):
    x, y, z = axis
    norm = math.sqrt(x * x + y * y + z * z)
    if norm == 0:
        return [[1, 0, 0], [0, 1, 0], [0, 0, 1]]
    x, y, z = x / norm, y / norm, z / norm
    c, s, t = math.cos(angle), math.sin(angle), 1 - math.cos(angle)
    return [[t * x * x + c, t * x * y - s * z, t * x * z + s * y],
            [t * x * y + s * z, t * y * y + c, t * y * z - s * x],
            [t * x * z - s * y, t * y * z + s * x, t * z * z + c]]


def matmul(a, b):
    return [[sum(a[i][k] * b[k][j] for k in range(3)) for j in range(3)] for i in range(3)]


def axis_angle(m):
    angle = math.acos(max(-1.0, min(1.0, (m[0][0] + m[1][1] + m[2][2] - 1) / 2)))
    if angle < 1e-9:
        return (0.0, 0.0, 1.0), 0.0
    if math.pi - angle < 1e-6:  # 180 degrees: axis from the diagonal
        xx, yy, zz = [math.sqrt(max(0.0, (m[i][i] + 1) / 2)) for i in range(3)]
        if xx >= yy and xx >= zz:
            axis = (xx, m[0][1] / (2 * xx), m[0][2] / (2 * xx))
        elif yy >= zz:
            axis = (m[0][1] / (2 * yy), yy, m[1][2] / (2 * yy))
        else:
            axis = (m[0][2] / (2 * zz), m[1][2] / (2 * zz), zz)
        return axis, angle
    s = 2 * math.sin(angle)
    return ((m[2][1] - m[1][2]) / s, (m[0][2] - m[2][0]) / s, (m[1][0] - m[0][1]) / s), angle


def load_node_specs():
    """{node type: {field name: VRML type}} from WEBOTS_HOME/resources/nodes/*.wrl."""
    specs = {}
    pattern = re.compile(r'^\s*(?:field|w3dField|hiddenField|deprecatedField)\s+([SM]F\w+)(?:\{[^}]*\})?\s+(\w+)')
    for fname in os.listdir(NODE_SPEC_DIR):
        if fname.endswith('.wrl'):
            fields = {}
            with open(os.path.join(NODE_SPEC_DIR, fname)) as f:
                for line in f:
                    m = pattern.match(line)
                    if m:
                        fields[m.group(2)] = m.group(1)
            specs[fname[:-4]] = fields
    return specs


# ----------------------------------------------------------------------------- conversion
class Converter:
    def __init__(self, controller=None, physics=None):
        self.controller = controller
        self.physics = physics
        self.notes = []
        self.specs = load_node_specs()

    def note(self, msg):
        if msg not in self.notes:
            self.notes.append(msg)

    def convert(self, nodes):
        nodes = self.replace_removed_protos(nodes)
        walk(nodes, self.upgrade_node)
        self.wrap_cylinders(nodes)
        walk(nodes, self.name_solids)
        walk(nodes, self.check_fields)
        return nodes

    # -- removed PROTOs
    def replace_removed_protos(self, nodes):
        out = []
        for n in nodes:
            if not isinstance(n, Node):
                out.append(n)
            elif n.type == 'SphericalSkyDome':
                self.note('SphericalSkyDome PROTO no longer exists: dropped (Background skyColor kept)')
            elif n.type in ('Floor', 'MetricFloor'):
                out.append(self.make_floor(n))
            elif n.type == 'WoodBox':
                out.append(self.make_box(n))
            else:
                out.append(n)
        return out

    def make_floor(self, old):
        metric = old.type == 'MetricFloor'
        size = old.get('size', SF(['100', '100'] if not metric else ['20', '20'])).tokens
        texture = 'metric_floor.png' if metric else 'checker_floor.png'
        floor = Node('Solid', old.def_name)
        for f in ('translation', 'rotation'):
            if old.get(f):
                floor.set(f, old.get(f))
        plane = Node('Plane')
        plane.set('size', SF(size))
        shape = Node('Shape')
        appearance = Node('Appearance')
        image = Node('ImageTexture')
        image.set('url', MF([['"textures/%s"' % texture]]))
        transform = Node('TextureTransform')
        transform.set('scale', SF(size))
        appearance.set('texture', image)
        appearance.set('textureTransform', transform)
        shape.set('appearance', appearance)
        shape.set('geometry', plane)
        visual = Node('Pose')
        visual.set('rotation', sf(1, 0, 0, num(-HALF_PI)))
        visual.set('children', MF([shape]))
        floor.set('children', MF([visual]))
        floor.set('name', string('floor'))
        floor.set('contactMaterial', old.get('contactMaterial', string('default')))
        bounding = Node('Pose')
        bounding.set('rotation', sf(1, 0, 0, num(-HALF_PI)))
        bplane = Node('Plane')
        bplane.set('size', SF(size))
        bounding.set('children', MF([bplane]))
        floor.set('boundingObject', bounding)
        floor.set('locked', sf('TRUE'))
        self.note('%s PROTO no longer exists: replaced by a Solid with an infinite Plane (same contactMaterial)' % old.type)
        return floor

    def make_box(self, old):
        size = old.get('boxSize', SF(['0.6', '0.6', '0.6']))
        box = Node('Solid', old.def_name)
        for f in ('translation', 'rotation'):
            if old.get(f):
                box.set(f, old.get(f))
        material = Node('Material')
        material.set('diffuseColor', sf(0.62, 0.45, 0.26))
        appearance = Node('Appearance')
        appearance.set('material', material)
        geometry = Node('Box')
        geometry.set('size', size)
        shape = Node('Shape')
        shape.set('appearance', appearance)
        shape.set('geometry', geometry)
        bgeometry = Node('Box')
        bgeometry.set('size', size)
        box.set('children', MF([shape]))
        box.set('name', string('wood box'))
        box.set('boundingObject', bgeometry)
        self.note('WoodBox PROTO no longer exists: replaced by a static box Solid of the same size')
        return box

    # -- per-node upgrades
    def upgrade_node(self, node, parent, field):
        t = node.type
        if t == 'Supervisor':
            node.type = 'Robot'
            node.set('supervisor', sf('TRUE'))
            self.note('Supervisor node -> Robot { supervisor TRUE }')
            t = 'Robot'
        if t == 'Robot':
            if node.get('showRobotWindow'):
                node.remove('showRobotWindow')
            if self.controller:
                node.set('controller', string(self.controller))
            elif node.get('controller') is None:
                node.set('controller', string('<none>'))  # Webots 7 default was "void"
        if t == 'WorldInfo':
            node.set('coordinateSystem', string('NUE'))
            if self.physics:
                node.set('physics', string(self.physics))
        if t in ('HingeJoint', 'SliderJoint', 'Hinge2Joint', 'BallJoint'):
            dev = node.get('device')
            if isinstance(dev, Node) or isinstance(dev, Use):
                items = [dev]
                if isinstance(dev, Node) and dev.type in ('RotationalMotor', 'LinearMotor'):
                    motor_name = dev.get('name')
                    base = motor_name.tokens[0].strip('"') if motor_name else dev.type.lower()
                    sensor = Node('PositionSensor')
                    sensor.set('name', string(base + '_sensor'))
                    items.append(sensor)
                node.set('device', MF(items))
                self.note('joint devices: motor + PositionSensor "<motor>_sensor" (position feedback left the motors in Webots 8)')
        if t == 'Transform':
            scale = node.get('scale')
            if scale is None or [float(v) for v in scale.tokens] == [1.0, 1.0, 1.0]:
                node.type = 'Pose'
                node.remove('scale')
                self.note('Transform -> Pose (required in boundingObject since R2023b)')
        if t == 'Viewpoint' and node.get('orientation'):
            v = [float(x) for x in node.get('orientation').tokens]
            camera = matmul(rot_matrix(v[:3], v[3]), matmul(rot_matrix((1, 0, 0), -HALF_PI), rot_matrix((0, 0, 1), HALF_PI)))
            axis, angle = axis_angle(camera)
            node.set('orientation', SF([num(axis[0]), num(axis[1]), num(axis[2]), num(angle)]))
            self.note('Viewpoint orientation converted to the R2022a camera convention')
        if t == 'ImageTexture' and node.get('url'):
            fixed = []
            for item in node.get('url').items:
                line = []
                for tok in item:
                    path = tok.strip('"').replace('\\\\', '/').replace('\\', '/')
                    if '/textures/' in '/' + path:
                        path = 'textures/' + path.split('textures/')[-1]
                    line.append('"%s"' % path)
                fixed.append(line)
            node.set('url', MF(fixed))

    # -- R2022a cylinder axis change
    def wrap_cylinders(self, nodes):
        """Keep every Cylinder along its parent's y axis.

        A Pose whose only child is a cylinder Shape gets the extra rotation folded into its own
        rotation; any other cylinder Shape is wrapped in a new Pose. A DEF on the Shape moves to
        the wrapper only when the name is USEd somewhere, so USE sites keep the fix too.
        """
        defs, used = {}, set()

        def collect_uses(n):
            for _, value in n.fields:
                items = value.items if isinstance(value, MF) else [value]
                for i in items:
                    if isinstance(i, Use):
                        used.add(i.name)
                    elif isinstance(i, Node):
                        collect_uses(i)

        for n in nodes:
            if isinstance(n, Node):
                collect_uses(n)

        def resolve(item):
            return defs.get(item.name) if isinstance(item, Use) else item

        def is_cylinder(geometry):
            g = resolve(geometry)
            return isinstance(g, Node) and g.type == 'Cylinder'

        def wrapper(child, def_name=None):
            pose = Node('Pose', def_name)
            pose.set('rotation', sf(1, 0, 0, num(-HALF_PI)))
            pose.set('children', MF([child]))
            pose.cylinder_wrapper = True
            return pose

        def fix(item):
            """Return the replacement for a children/boundingObject item."""
            if isinstance(item, Node):
                if item.type == 'Shape' and item.get('geometry') is not None and is_cylinder(item.get('geometry')):
                    visit(item)
                    self.note('Cylinder axis kept along y with an extra rotation 1 0 0 -pi/2 (R2022a moved it to z)')
                    if item.def_name in used:
                        pose = wrapper(item, item.def_name)
                        item.def_name = None
                        defs[pose.def_name] = pose
                        return pose
                    return wrapper(item)
                if item.type == 'Cylinder':
                    visit(item)
                    return wrapper(item)
                visit(item)
                return item
            if isinstance(item, Use) and is_cylinder(item):
                return wrapper(item)  # bare USE of a Cylinder geometry
            return item

        def visit(node):
            if node.def_name:
                defs[node.def_name] = node
            for f in node.fields:
                name, value = f
                if isinstance(value, MF):
                    value.items = [fix(i) if not isinstance(i, list) else i for i in value.items]
                elif name == 'boundingObject' and value is not None:
                    f[1] = fix(value)
                elif isinstance(value, Node):
                    visit(value)
            children = node.get('children')
            if node.type == 'Pose' and not getattr(node, 'cylinder_wrapper', False) and isinstance(children, MF) \
                    and len(children.items) == 1 and getattr(children.items[0], 'cylinder_wrapper', False) \
                    and children.items[0].def_name is None:
                inner = children.items[0]
                old = [float(v) for v in node.get('rotation', sf(0, 0, 1, 0)).tokens]
                axis, angle = axis_angle(matmul(rot_matrix(old[:3], old[3]), rot_matrix((1, 0, 0), -HALF_PI)))
                node.set('rotation', SF([num(axis[0]), num(axis[1]), num(axis[2]), num(angle)]))
                node.set('children', inner.get('children'))

        for i, n in enumerate(nodes):
            nodes[i] = fix(n)

    # -- unique names for unnamed sibling solids (R2025a warns otherwise)
    def name_solids(self, node, parent, field):
        if field == 'endPoint' and node.type == 'Solid' and node.get('name') is None:
            label = node.def_name or (parent.def_name + '_link' if parent and parent.def_name else None)
            if label:
                node.set('name', string(label))

    # -- drop fields R2025a does not know, bracket SF values of MF fields
    def check_fields(self, node, parent, field):
        spec = self.specs.get(node.type)
        if spec is None:
            raise ValueError('node type %s has no R2025a definition' % node.type)
        kept = []
        for name, value in node.fields:
            if name not in spec:
                self.note('dropped field %s.%s (no longer exists)' % (node.type, name))
                continue
            if spec[name].startswith('MF') and isinstance(value, SF):
                value = MF([value.tokens])
            kept.append([name, value])
        node.fields = kept


def convert_file(src, dst, controller=None, physics=None):
    with open(src, encoding='utf-8', errors='replace') as f:
        text = f.read()
    header = text.split('\n', 1)[0].strip()
    if not header.startswith('#VRML_SIM V7'):
        raise ValueError('%s: unexpected header %r' % (src, header))
    conv = Converter(controller, physics)
    nodes = conv.convert(Parser(text).parse_world())
    out = ['#VRML_SIM R2025a utf8',
           '# Converted from Webots %s by tools/convert_v7_world.py (original: %s)'
           % (header.split()[1], os.path.basename(src))]
    for n in nodes:
        write_node(n, 0, out)
    with open(dst, 'w', newline='\n') as f:
        f.write('\n'.join(out) + '\n')
    return conv.notes


def main():
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument('src')
    ap.add_argument('dst')
    ap.add_argument('--controller', help='set the robot controller (default: keep the world value)')
    ap.add_argument('--physics', help='set the WorldInfo physics plugin (default: keep the world value)')
    args = ap.parse_args()
    for line in convert_file(args.src, args.dst, args.controller, args.physics):
        print('  - ' + line)


if __name__ == '__main__':
    sys.exit(main())
