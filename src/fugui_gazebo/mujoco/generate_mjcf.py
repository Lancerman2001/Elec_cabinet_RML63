#!/usr/bin/env python3
"""Convert the checked-in URDF to MJCF, retaining SI geometry and joint frames.

Only the Python standard library is needed. STL meshes and the nameplate PNG
are shared with Gazebo; the DAE nameplate becomes an explicitly UV-mapped mesh.
"""
import hashlib
import json
import math
from pathlib import Path
import xml.etree.ElementTree as ET
from xml.dom import minidom

OUT = Path(__file__).resolve().parent
ROOT = OUT.parent


def numbers(text): return [float(v) for v in text.split()]
def fmt(values): return ' '.join('%.12g' % v for v in values)


def quaternion(rpy):
    r, p, y = [v/2 for v in rpy]
    cr, sr, cp, sp, cy, sy = math.cos(r), math.sin(r), math.cos(p), math.sin(p), math.cos(y), math.sin(y)
    return [cr*cp*cy+sr*sp*sy, sr*cp*cy-cr*sp*sy, cr*sp*cy+sr*cp*sy, cr*cp*sy-sr*sp*cy]


def origin(element):
    if element is None: return {'pos': '0 0 0', 'quat': '1 0 0 0'}
    return {'pos': element.get('xyz', '0 0 0'),
            'quat': fmt(quaternion(numbers(element.get('rpy', '0 0 0'))))}


def nameplate(asset):
    dae = ET.parse(ROOT/'meshes/nameplate.dae')
    ns = {'c': 'http://www.collada.org/2005/11/COLLADASchema'}
    front = numbers(dae.find('.//c:float_array[@id="v-array"]', ns).text)
    uv = numbers(dae.find('.//c:float_array[@id="uv-array"]', ns).text)
    # MuJoCo PNG texture V origin is opposite to the Collada convention.
    uv[1::2] = [1-v for v in uv[1::2]]
    back = list(front)
    back[0::3] = [x-.00002 for x in back[0::3]]
    faces = [0,2,1, 0,3,2, 4,5,6, 4,6,7,
             0,1,5, 0,5,4, 1,2,6, 1,6,5, 2,3,7, 2,7,6, 3,0,4, 3,4,7]
    ET.SubElement(asset, 'mesh', name='nameplate', vertex=fmt(front+back),
                  face=fmt(faces), texcoord=fmt(uv+uv))
    ET.SubElement(asset, 'texture', name='nameplate_texture', type='2d', file='nameplate.png')
    ET.SubElement(asset, 'material', name='nameplate_material', texture='nameplate_texture',
                  rgba='1 1 1 1', specular='.05', shininess='.1')


def camera(world, name, pos, target):
    forward = [t-p for p, t in zip(pos, target)]
    norm = math.sqrt(sum(v*v for v in forward)); forward = [v/norm for v in forward]
    right = [forward[1], -forward[0], 0]
    norm = math.hypot(*right[:2]); right = [v/norm for v in right]
    up = [right[1]*forward[2]-right[2]*forward[1],
          right[2]*forward[0]-right[0]*forward[2], right[0]*forward[1]-right[1]*forward[0]]
    ET.SubElement(world, 'camera', name=name, pos=fmt(pos), xyaxes=fmt(right+up), fovy='32')


def generate():
    cfg = json.loads((ROOT/'config/button.yaml').read_text())
    source = ROOT/'urdf/fugui.urdf'
    urdf = ET.parse(source).getroot()
    links = {link.get('name'): link for link in urdf.findall('link')}
    children = {name: [] for name in links}
    for joint in urdf.findall('joint'): children[joint.find('parent').get('link')].append(joint)
    colors = {g.get('reference'): g.find('visual/material/diffuse').text for g in urdf.findall('gazebo')
              if g.find('visual/material/diffuse') is not None}
    root = ET.Element('mujoco', model='protection_signal_reset_button')
    root.append(ET.Comment('Generated from ../urdf/fugui.urdf; metres, kilograms, seconds.'))
    ET.SubElement(root, 'compiler', angle='radian', meshdir='../meshes', texturedir='../materials/textures',
                  autolimits='true', inertiafromgeom='false', fusestatic='false')
    ET.SubElement(root, 'option', timestep='.001', integrator='implicitfast', gravity='0 0 -9.81', iterations='100')
    ET.SubElement(root, 'statistic', center='.010 0 .12', extent='.13')
    visual = ET.SubElement(root, 'visual')
    ET.SubElement(visual, 'global', offwidth='1000', offheight='1000', azimuth='148', elevation='-20')
    ET.SubElement(visual, 'headlight', ambient='.35 .35 .35', diffuse='.5 .5 .5', specular='.15 .15 .15')
    # znear scales by model extent: 1.3 mm gives enough depth precision for
    # the URDF's 20 micrometre decorative face separation in the live viewer.
    ET.SubElement(visual, 'map', znear='.01', zfar='20')
    ET.SubElement(visual, 'quality', shadowsize='2048', offsamples='4')
    default = ET.SubElement(root, 'default')
    vis = ET.SubElement(default, 'default', {'class': 'visual'})
    ET.SubElement(vis, 'geom', contype='0', conaffinity='0', group='2', density='0')
    col = ET.SubElement(default, 'default', {'class': 'collision'})
    ET.SubElement(col, 'geom', contype='1', conaffinity='1', group='3', density='0',
                  rgba='.9 .3 .1 0', friction='.7 .005 .0001', solref='.004 1', solimp='.95 .99 .0001')
    asset = ET.SubElement(root, 'asset')
    for name in sorted({Path(m.get('filename')).name for m in urdf.findall('.//visual/geometry/mesh')}):
        if name == 'nameplate.dae': nameplate(asset)
        else: ET.SubElement(asset, 'mesh', name=Path(name).stem, file=name)
    ET.SubElement(asset, 'material', name='fugui_lamp', rgba=colors['fugui_button_link'],
                  emission='0', specular='.3', shininess='.4')
    world = ET.SubElement(root, 'worldbody')
    ET.SubElement(world, 'light', name='key', pos='.2 -.1 .4', dir='-.7 .3 -.7', directional='true',
                  diffuse='.6 .6 .6', specular='.15 .15 .15')
    ET.SubElement(world, 'light', name='fill', pos='.1 .2 .2', dir='-.6 -.6 .2', directional='true',
                  diffuse='.2 .2 .2', castshadow='false')
    camera(world, 'front', [.18, 0, .12], [.007, 0, .12])
    camera(world, 'oblique', [.155, -.1, .177], [.010, 0, .12])
    camera(world, 'side', [.015, -.18, .12], [.015, 0, .12])

    def add_geom(body, element, link, index, collision=False):
        kind = list(element.find('geometry'))[0]
        attrs = origin(element.find('origin'))
        attrs.update(name=link+('_collision_' if collision else '_visual_')+str(index),
                     **{'class': 'collision' if collision else 'visual'})
        if not collision: attrs['rgba'] = colors.get(link, '1 1 1 1')
        if kind.tag == 'box': attrs.update(type='box', size=fmt([v/2 for v in numbers(kind.get('size'))]))
        elif kind.tag == 'cylinder': attrs.update(type='cylinder', size=fmt([float(kind.get('radius')), float(kind.get('length'))/2]))
        elif kind.tag == 'sphere': attrs.update(type='sphere', size=kind.get('radius'))
        elif kind.tag == 'mesh':
            name = Path(kind.get('filename')).stem
            attrs.update(type='mesh', mesh=name)
            if name == 'nameplate': attrs.update(material='nameplate_material', rgba='1 1 1 1')
        else: raise ValueError('Unsupported geometry: '+kind.tag)
        if link == 'fugui_button_link' and not collision:
            attrs.pop('rgba'); attrs['material'] = 'fugui_lamp'
        ET.SubElement(body, 'geom', attrs)

    def add_body(parent, joint):
        name = joint.find('child').get('link'); link = links[name]
        body = ET.SubElement(parent, 'body', name=name, **origin(joint.find('origin')))
        inertia = link.find('inertial')
        if inertia is not None:
            transform = origin(inertia.find('origin'))
            if transform.pop('quat') != '1 0 0 0': raise ValueError('Rotated inertia not supported by this converter')
            tensor = inertia.find('inertia')
            ET.SubElement(body, 'inertial', mass=inertia.find('mass').get('value'),
                          fullinertia=' '.join(tensor.get(k, '0') for k in ('ixx', 'iyy', 'izz', 'ixy', 'ixz', 'iyz')), **transform)
        if joint.get('type') == 'prismatic':
            limit = joint.find('limit')
            ET.SubElement(body, 'joint', name=joint.get('name'), type='slide', axis=joint.find('axis').get('xyz'),
                          range=limit.get('lower')+' '+limit.get('upper'), stiffness=str(cfg['stiffness']), springref='0',
                          damping=str(cfg['damping']), solreflimit='.002 1', solimplimit='.99 .999 .00001')
        elif joint.get('type') != 'fixed': raise ValueError('Unsupported joint type: '+joint.get('type'))
        for collision in (False, True):
            for index, element in enumerate(link.findall('collision' if collision else 'visual')):
                add_geom(body, element, name, index, collision)
        for child in children[name]: add_body(body, child)

    for joint in children['world']: add_body(world, joint)
    actuator = ET.SubElement(root, 'actuator')
    ET.SubElement(actuator, 'motor', name='fugui_press_force', joint='fugui_press_joint',
                  gear='1', ctrlrange='-10 10', forcerange='-10 10')
    sensor = ET.SubElement(root, 'sensor')
    ET.SubElement(sensor, 'jointpos', name='press_depth', joint='fugui_press_joint')
    ET.SubElement(sensor, 'jointvel', name='press_velocity', joint='fugui_press_joint')
    custom = ET.SubElement(root, 'custom')
    for name, data in {'valid_depth_range': fmt([cfg['valid_min'], cfg['valid_max']]),
                       'default_depth': str(cfg['default_depth']), 'rearm_depth': str(cfg['rearm_depth']),
                       'lamp_duration': str(cfg['lamp_duration']),
                       'numerical_tolerance': str(cfg['numerical_tolerance']), 'reference_speed': '.025',
                       'servo_kp': '400', 'servo_kd': '3', 'action_timeout': '3'}.items():
        ET.SubElement(custom, 'numeric', name=name, data=data)
    ET.SubElement(ET.SubElement(root, 'keyframe'), 'key', name='rest', qpos='0', ctrl='0')
    xml = minidom.parseString(ET.tostring(root)).toprettyxml(indent='  ')
    (OUT/'fugui.xml').write_text(xml)
    import mujoco_compat
    mujoco_compat.write_compatible_model(OUT/'fugui.xml')
    sources = [source, ROOT/'config/button.yaml', *sorted((ROOT/'meshes').glob('*.stl')), ROOT/'meshes/nameplate.dae', ROOT/'materials/textures/nameplate.png']
    manifest = {'source_sha256': {str(p.relative_to(ROOT)): hashlib.sha256(p.read_bytes()).hexdigest() for p in sources},
                'model': 'fugui.xml', 'units': 'metres, kilograms, seconds', 'joint': 'fugui_press_joint',
                'actuator': 'fugui_press_force', 'valid_depth_mm': [cfg['valid_min']*1000, cfg['valid_max']*1000],
                'nominal_depth_mm': cfg['default_depth']*1000, 'lamp_seconds': cfg['lamp_duration']}
    (OUT/'manifest.json').write_text(json.dumps(manifest, indent=2)+'\n')
    print('Generated', OUT/'fugui.xml')


if __name__ == '__main__': generate()
