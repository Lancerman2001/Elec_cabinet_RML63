#!/usr/bin/env python3
"""Render the actual URDF visuals and joint transforms, using numpy/matplotlib.

Run from any directory: python3 scripts/render_preview.py
No Gazebo, ROS, CAD library, or hand-drawn substitute geometry is required.
"""
import argparse
import math
from pathlib import Path
import xml.etree.ElementTree as ET

import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt
import numpy as np
from mpl_toolkits.mplot3d.art3d import Poly3DCollection
from matplotlib.ticker import MaxNLocator

ROOT = Path(__file__).resolve().parents[1]


def numbers(value):
    return np.array([float(v) for v in value.split()])


def axis_rotation(axis, angle):
    axis = np.asarray(axis, dtype=float)
    axis /= np.linalg.norm(axis)
    x, y, z = axis
    cross = np.array([[0, -z, y], [z, 0, -x], [-y, x, 0]])
    return (np.eye(3) * math.cos(angle) +
            (1 - math.cos(angle)) * np.outer(axis, axis) + math.sin(angle) * cross)


def origin(element):
    transform = np.eye(4)
    if element is not None:
        transform[:3, 3] = numbers(element.get('xyz', '0 0 0'))
        r, p, y = numbers(element.get('rpy', '0 0 0'))
        transform[:3, :3] = (axis_rotation([0, 0, 1], y) @
                             axis_rotation([0, 1, 0], p) @ axis_rotation([1, 0, 0], r))
    return transform


def forward_kinematics(robot, positions):
    joints = list(robot.findall('joint'))
    children = {j.find('child').get('link') for j in joints}
    frames = {link.get('name'): np.eye(4) for link in robot.findall('link')
              if link.get('name') not in children}
    values = dict(positions)
    for joint in joints:
        mimic = joint.find('mimic')
        if mimic is not None and joint.get('name') not in values:
            values[joint.get('name')] = (values.get(mimic.get('joint'), 0) *
                                         float(mimic.get('multiplier', '1')) +
                                         float(mimic.get('offset', '0')))
    while joints:
        pending = []
        for joint in joints:
            parent = joint.find('parent').get('link')
            if parent not in frames:
                pending.append(joint)
                continue
            transform = origin(joint.find('origin'))
            movement = np.eye(4)
            q = values.get(joint.get('name'), 0)
            axis = joint.find('axis')
            axis = numbers(axis.get('xyz')) if axis is not None else np.array([1., 0., 0.])
            if joint.get('type') in ('continuous', 'revolute'):
                movement[:3, :3] = axis_rotation(axis, q)
            elif joint.get('type') == 'prismatic':
                movement[:3, 3] = axis / np.linalg.norm(axis) * q
            frames[joint.find('child').get('link')] = frames[parent] @ transform @ movement
        if len(pending) == len(joints):
            raise ValueError('URDF joint tree is disconnected or cyclic')
        joints = pending
    base_inverse = np.linalg.inv(frames['yaban_base_link'])
    return {name: base_inverse @ transform for name, transform in frames.items()}


def geometry_faces(geometry):
    box = geometry.find('box')
    if box is not None:
        x, y, z = numbers(box.get('size')) / 2
        v = np.array([[-x,-y,-z], [x,-y,-z], [x,y,-z], [-x,y,-z],
                      [-x,-y,z], [x,-y,z], [x,y,z], [-x,y,z]])
        return [v[list(f)] for f in [(0,3,2,1), (4,5,6,7), (0,1,5,4),
                                     (1,2,6,5), (2,3,7,6), (3,0,4,7)]]
    cylinder = geometry.find('cylinder')
    if cylinder is not None:
        radius = float(cylinder.get('radius'))
        half = float(cylinder.get('length')) / 2
        theta = np.linspace(0, 2*np.pi, 49)[:-1]
        bottom = np.column_stack((radius*np.cos(theta), radius*np.sin(theta), np.full(48, -half)))
        top = bottom.copy()
        top[:, 2] = half
        return [bottom[::-1], top] + [np.array([bottom[i], bottom[(i+1)%48],
                                               top[(i+1)%48], top[i]]) for i in range(48)]
    mesh = geometry.find('mesh')
    if mesh is not None:
        filename = mesh.get('filename')
        if filename.startswith('package://yaban_gazebo/'):
            path = ROOT / filename.removeprefix('package://yaban_gazebo/')
        else:
            path = Path(filename.removeprefix('file://'))
        vertices = [numbers(line.strip()[7:]) for line in path.read_text().splitlines()
                    if line.strip().startswith('vertex ')]
        if not vertices or len(vertices) % 3:
            raise ValueError('Expected valid ASCII STL: ' + str(path))
        scale = numbers(mesh.get('scale', '1 1 1'))
        faces = np.array(vertices).reshape(-1, 3, 3) * scale
        if path.name in ('base_bridge.stl', 'blade.stl'):
            # These generated meshes comprise adjoining closed trapezoidal cells.
            # Hide their internal X-normal end caps for the painter renderer;
            # all exterior surfaces, hole walls and notch walls remain intact.
            xmin, xmax = faces[:, :, 0].min(), faces[:, :, 0].max()
            flat_x = np.ptp(faces[:, :, 0], axis=1) < 1e-12
            internal_x = ((faces[:, 0, 0] > xmin+1e-12) &
                          (faces[:, 0, 0] < xmax-1e-12))
            faces = faces[~(flat_x & internal_x)]
        return list(faces)
    raise ValueError('Unsupported visual geometry: ' + ET.tostring(geometry).decode())


def model_faces(robot, positions):
    frames = forward_kinematics(robot, positions)
    materials = {m.get('name'): numbers(m.find('color').get('rgba'))
                 for m in robot.findall('material') if m.find('color') is not None}
    all_faces, colors = [], []
    lamp = np.array([-0.35, -0.55, 0.76])
    lamp /= np.linalg.norm(lamp)
    for link in robot.findall('link'):
        for visual in link.findall('visual'):
            transform = frames[link.get('name')] @ origin(visual.find('origin'))
            material = visual.find('material')
            color = np.array([0.7, 0.7, 0.7, 1.0])
            if material is not None:
                inline = material.find('color')
                color = numbers(inline.get('rgba')) if inline is not None else materials.get(material.get('name'), color)
            for face in geometry_faces(visual.find('geometry')):
                face = (face @ transform[:3, :3].T + transform[:3, 3]) * 1000
                normal = np.cross(face[1]-face[0], face[2]-face[0])
                length = np.linalg.norm(normal)
                if length < 1e-12:
                    continue
                brightness = 0.57 + 0.43 * max(0, np.dot(normal / length, lamp))
                all_faces.append(face)
                colors.append(np.r_[np.clip(color[:3]*brightness, 0, 1), color[3]])
    return all_faces, colors


def draw(ax, faces, colors, title, elevation, azimuth, ortho=None):
    collection = Poly3DCollection(faces, facecolors=colors, edgecolors='none',
                                  linewidths=0, antialiased=False, zsort='average')
    ax.add_collection3d(collection)
    points = np.concatenate(faces)
    low, high = points.min(axis=0), points.max(axis=0)
    center, size = (low+high)/2, high-low
    size = np.maximum(size*1.13, 18)
    for setter, c, s in zip([ax.set_xlim, ax.set_ylim, ax.set_zlim], center, size):
        setter(c-s/2, c+s/2)
    ax.set_box_aspect(size)
    ax.set_proj_type('ortho')
    ax.view_init(elev=elevation, azim=azimuth)
    ax.set_title(title, fontsize=15, pad=4, loc='left', color='#203038')
    ax.set_xlabel('X (mm)', labelpad=1)
    ax.set_ylabel('Y (mm)', labelpad=1)
    ax.set_zlabel('Z (mm)', labelpad=1)
    for axis in (ax.xaxis, ax.yaxis, ax.zaxis):
        axis.pane.fill = False
        axis.pane.set_edgecolor('#d7dfe3')
        axis.set_tick_params(labelsize=8, pad=0)
        axis.set_major_locator(MaxNLocator(nbins=5))
        axis._axinfo['grid']['color'] = '#e2e7ea'
    if ortho == 'front':
        ax.set_yticks([])
        ax.set_ylabel('')
    elif ortho == 'top':
        ax.set_zticks([])
        ax.set_zlabel('')
    elif ortho == 'side':
        ax.set_xticks([])
        ax.set_xlabel('')


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--urdf', type=Path, default=ROOT/'urdf/yaban.urdf')
    parser.add_argument('--output', type=Path, default=ROOT/'docs/model_preview.png')
    args = parser.parse_args()
    robot = ET.parse(args.urdf).getroot()
    closed = model_faces(robot, {})
    opened = model_faces(robot, {'yaban_blade_joint': np.pi/2, 'yaban_left_knob_joint': np.pi})
    plt.rcParams.update({'font.family': 'DejaVu Sans', 'font.size': 10})
    fig = plt.figure(figsize=(16, 12), facecolor='white')
    grid = fig.add_gridspec(2, 6, left=0.025, right=0.975, top=0.91, bottom=0.065,
                           hspace=0.1, wspace=0.12, height_ratios=[1.1, 1])
    draw(fig.add_subplot(grid[0, :3], projection='3d'), *closed,
         '01  CLOSED / engaged', 27, -65)
    draw(fig.add_subplot(grid[0, 3:], projection='3d'), *opened,
         '02  OPEN / blade 90 degrees', 39, -62)
    draw(fig.add_subplot(grid[1, :2], projection='3d'), *closed,
         '03  Front (X-Z)', 0, -90, 'front')
    draw(fig.add_subplot(grid[1, 2:4], projection='3d'), *closed,
         '04  Top (X-Y)', 90, -90, 'top')
    draw(fig.add_subplot(grid[1, 4:], projection='3d'), *closed,
         '05  Side (Y-Z)', 0, 0, 'side')
    fig.suptitle('YABAN  /  cabinet protection link', x=0.035, y=0.975,
                 ha='left', fontsize=24, weight='bold', color='#203038')
    fig.text(0.036, 0.937, 'Actual URDF visual geometry  |  dimensions in mm  |  base mounting plane: Z = 0',
             fontsize=12, color='#56666e')
    fig.text(0.036, 0.028, 'Open state: left knob +180 degrees, axial lift +0.35 mm; blade rotates about the right terminal.\n'
             'Ivory: insulation and knobs. Grey: blade, studs, washers and nuts. Rear hardware uses reference dimensions.',
             fontsize=10, color='#56666e', linespacing=1.6)
    args.output.parent.mkdir(parents=True, exist_ok=True)
    fig.savefig(args.output, dpi=170, facecolor='white')
    plt.close(fig)
    print(args.output)


if __name__ == '__main__':
    main()
