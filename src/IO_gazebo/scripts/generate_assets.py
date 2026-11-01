#!/usr/bin/python3
"""Deterministic SI meshes, collision decomposition and inertia.

The breaker housing intentionally has no product-label decals. Cabinet
identification nameplates are generated separately by total_cabinet_gazebo.

No AI-generated geometry or proprietary CAD dependency. Run after editing
config/breaker.yaml, then expand breaker.urdf.xacro to refresh the plain URDF.
"""
import itertools
import json
import math
from pathlib import Path
import struct
import xml.etree.ElementTree as ET

import numpy as np
from scipy.spatial import ConvexHull
import yaml

ROOT = Path(__file__).resolve().parents[1]


def rounded_box(size, radius, subdivisions=7):
    half = np.asarray(size) / 2 - radius
    points = []
    for signs in itertools.product((-1, 1), repeat=3):
        for i in range(subdivisions + 1):
            a = math.pi * i / (2 * subdivisions)
            for j in range(subdivisions + 1):
                b = math.pi * j / (2 * subdivisions)
                direction = np.array([math.cos(a)*math.cos(b), math.cos(a)*math.sin(b), math.sin(a)])
                points.append(np.asarray(signs) * (half + radius * direction))
    points = np.unique(np.round(points, 12), axis=0)
    hull = ConvexHull(points)
    triangles = []
    for ids, plane in zip(hull.simplices, hull.equations):
        triangle = points[ids].copy()
        if np.dot(np.cross(triangle[1]-triangle[0], triangle[2]-triangle[0]), plane[:3]) < 0:
            triangle = triangle[[0, 2, 1]]
        triangles.append(triangle)
    return np.asarray(triangles)


def stl(path, triangles):
    with path.open('wb') as out:
        out.write(b'io_gazebo SI metre mesh'.ljust(80, b'\0'))
        out.write(struct.pack('<I', len(triangles)))
        for triangle in triangles:
            normal = np.cross(triangle[1]-triangle[0], triangle[2]-triangle[0])
            normal /= np.linalg.norm(normal)
            out.write(struct.pack('<12fH', *normal, *triangle.flatten(), 0))


def read_stl(path):
    data = path.read_bytes()
    count = struct.unpack_from('<I', data, 80)[0]
    return np.array([struct.unpack_from('<12fH', data, 84+50*i)[3:12] for i in range(count)]).reshape(-1, 3, 3)


def collision_parts(size, radius):
    """Exact rounded box as 3 core boxes, 12 edge cylinders and 8 spheres."""
    size = np.asarray(size)
    core = size - 2*radius
    parts = []
    for axis in range(3):
        dimensions = core.copy(); dimensions[axis] = size[axis]
        parts.append(('box', [0., 0., 0.], dimensions.tolist(), [0., 0., 0.]))
    for axis in range(3):
        others = [a for a in range(3) if a != axis]
        rpy = [0., math.pi/2, 0.] if axis == 0 else ([math.pi/2, 0., 0.] if axis == 1 else [0., 0., 0.])
        for signs in itertools.product((-1, 1), repeat=2):
            center = [0., 0., 0.]
            for other, sign in zip(others, signs):
                center[other] = sign * core[other]/2
            parts.append(('cylinder', center, [radius, float(core[axis])], rpy))
    for signs in itertools.product((-1, 1), repeat=3):
        parts.append(('sphere', (np.array(signs)*core/2).tolist(), [radius], [0., 0., 0.]))
    return parts


def write_collision(size, radius):
    ET.register_namespace('xacro', 'http://www.ros.org/wiki/xacro')
    robot = ET.Element('robot')
    macro = ET.SubElement(robot, '{http://www.ros.org/wiki/xacro}macro', name='breaker_grip_collision', params='x')
    for kind, center, dimensions, rpy in collision_parts(size, radius):
        col = ET.SubElement(macro, 'collision')
        ET.SubElement(col, 'origin', xyz='${x + %.12g} %.12g %.12g' % tuple(center), rpy=' '.join(map(str, rpy)))
        geom = ET.SubElement(col, 'geometry')
        if kind == 'box': ET.SubElement(geom, kind, size=' '.join(map(str, dimensions)))
        elif kind == 'cylinder': ET.SubElement(geom, kind, radius=str(dimensions[0]), length=str(dimensions[1]))
        else: ET.SubElement(geom, kind, radius=str(dimensions[0]))
    ET.ElementTree(robot).write(str(ROOT/'urdf/grip_collision.xacro'), encoding='utf-8', xml_declaration=True)


def inertia(config):
    # Three non-overlapping approximate mass components: grip and two hub/arms.
    m = config['handle_mass']; r = config['grip_radius']; spacing = config['pole_width']
    components = [(0.65*m, np.array([r, 0., 0.]), np.array([config['grip_depth'], config['grip_width'], config['grip_height']]))]
    for sign in (-1, 1):
        components.append((0.175*m, np.array([0.003, sign*spacing/2, 0.]), np.array([0.018, config['arm_width'], 0.0124])))
    center = sum(mass*pos for mass, pos, size in components)/m
    tensor = np.zeros((3, 3))
    for mass, pos, size in components:
        x,y,z = size; d = pos-center
        tensor += np.diag(mass*np.array([y*y+z*z, x*x+z*z, x*x+y*y])/12)
        tensor += mass*(np.dot(d,d)*np.eye(3)-np.outer(d,d))
    return {'mass': m, 'center': center.tolist(), 'ixx': float(tensor[0,0]), 'iyy': float(tensor[1,1]), 'izz': float(tensor[2,2]), 'pivot_iyy': float(tensor[1,1]+m*(center[0]**2+center[2]**2))}








def main():
    c = yaml.safe_load((ROOT/'config/breaker.yaml').read_text())
    for d in ['meshes', 'materials/textures', 'urdf']:
        (ROOT/d).mkdir(parents=True, exist_ok=True)
    shapes = {'upper_shell': ([c['body_depth'], c['pole_width']-0.0002, c['label_height']], 0.00065),
              'grip': ([c['grip_depth'], c['grip_width'], c['grip_height']], c['grip_round']),
              'arm': ([0.013, c['arm_width'], 0.0032], 0.0007),
              'rib': ([0.00065, c['grip_width']-0.004, 0.00055], 0.0002)}
    for name, (size, radius) in shapes.items(): stl(ROOT/'meshes'/(name+'.stl'), rounded_box(size, radius))
    write_collision(shapes['grip'][0], c['grip_round'])
    # Remove obsolete product decals when regenerating an existing checkout.
    for side in ('left', 'right'):
        for relative in (f'materials/textures/face_{side}.png', f'meshes/face_{side}.dae'):
            (ROOT/relative).unlink(missing_ok=True)
    (ROOT/'config/inertia.yaml').write_text(yaml.safe_dump(inertia(c)))
    (ROOT/'meshes/manifest.json').write_text(json.dumps({'geometry': c, 'inertia': inertia(c)}, indent=2)+'\n')
    print('Generated SI meshes, rounded collision and inertia; housing has no product decals.')


if __name__ == '__main__': main()
