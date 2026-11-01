#!/usr/bin/env python3
"""Rebuild MJCF and OBJ assets from the checked-in URDF/DAE using stdlib only."""
import hashlib
import json
import math
from pathlib import Path
import xml.etree.ElementTree as ET

OUT = Path(__file__).resolve().parent
ROOT = OUT.parent
NS = {'c': 'http://www.collada.org/2005/11/COLLADASchema'}


def nums(s): return [float(v) for v in s.split()]
def fmt(v): return ' '.join('%.12g' % x for x in v)


def origin(node):
    if node is None: return {'pos': '0 0 0'}
    r, p, y = [v/2 for v in nums(node.get('rpy', '0 0 0'))]
    cr, sr, cp, sp, cy, sy = math.cos(r), math.sin(r), math.cos(p), math.sin(p), math.cos(y), math.sin(y)
    return {'pos': node.get('xyz', '0 0 0'), 'quat': fmt([
        cr*cp*cy+sr*sp*sy, sr*cp*cy-cr*sp*sy, cr*sp*cy+sr*cp*sy, cr*cp*sy-sr*sp*cy])}


def dae_groups(path, asset):
    doc = ET.parse(path)
    groups = []
    for geo in doc.findall('.//c:geometry', NS):
        name = geo.get('id').removesuffix('_geo')
        flat = nums(geo.find('.//c:source[@id="'+name+'_positions"]/c:float_array', NS).text)
        points = [flat[i:i+3] for i in range(0, len(flat), 3)]
        color = doc.find('.//c:effect[@id="'+name+'_effect"]//c:diffuse/c:color', NS)
        if name == 'nameplate':
            # One continuous quad, closed behind its front face. Explicit UVs
            # keep the text upright and avoid the old fan-triangle glyph seams.
            vertices = list(dict.fromkeys(tuple(v) for v in points))
            front = vertices[:]
            vertices += [(x-.00002, y, z) for x, y, z in front]
            faces = [0,1,2, 0,2,3, 4,6,5, 4,7,6,
                     0,4,5, 0,5,1, 1,5,6, 1,6,2, 2,6,7, 2,7,3, 3,7,4, 3,4,0]
            w = max(v[1] for v in front)-min(v[1] for v in front)
            h = max(v[2] for v in front)-min(v[2] for v in front)
            uv = [(y/w+.5, .5-z/h) for x,y,z in vertices]
            ET.SubElement(asset, 'mesh', name=name, vertex=fmt(sum((list(v) for v in vertices), [])),
                          face=fmt(faces), texcoord=fmt(sum((list(v) for v in uv), [])))
            ET.SubElement(asset, 'texture', name='nameplate_texture', type='2d', file='../materials/textures/nameplate.png')
            ET.SubElement(asset, 'material', name=name+'_material', texture='nameplate_texture',
                          rgba='1 1 1 1', specular='.12', shininess='.25')
        else:
            if name == 'pointer':
                a,b,c = points
                aa,bb,cc = [[x-.00002,y,z] for x,y,z in points]
                points = [a,b,c, aa,cc,bb, a,aa,bb, a,bb,b, b,bb,cc, b,cc,c, c,cc,aa, c,aa,a]
            lines = ['# metres; converted from '+path.name]
            lines += ['v '+fmt(v) for v in points]
            lines += ['f %d %d %d' % (i+1,i+2,i+3) for i in range(0,len(points),3)]
            (OUT/'meshes'/f'{name}.obj').write_text('\n'.join(lines)+'\n')
            ET.SubElement(asset, 'mesh', name=name, file=f'meshes/{name}.obj')
            ET.SubElement(asset, 'material', name=name+'_material', rgba=color.text,
                          specular='.35' if name in ('shaft','screw') else '.22', shininess='.35')
        groups.append(name)
    return groups


def camera(world, name, pos, target, fovy, up=(0,0,1)):
    forward = [t-p for p,t in zip(pos,target)]
    def unit(v):
        n = math.sqrt(sum(x*x for x in v)); return [x/n for x in v]
    def cross(a,b): return [a[1]*b[2]-a[2]*b[1], a[2]*b[0]-a[0]*b[2], a[0]*b[1]-a[1]*b[0]]
    forward = unit(forward); right = unit(cross(forward,up)); up = cross(right,forward)
    ET.SubElement(world,'camera',name=name,pos=fmt(pos),xyaxes=fmt(right+up),fovy=str(fovy))


def generate():
    (OUT/'meshes').mkdir(exist_ok=True)
    source = ROOT/'urdf/xuanniu.urdf'
    urdf = ET.parse(source).getroot()
    links = {v.get('name'):v for v in urdf.findall('link')}
    children = {name:[] for name in links}
    for joint in urdf.findall('joint'): children[joint.find('parent').get('link')].append(joint)
    root = ET.Element('mujoco',model='xuanniu_selector_switch')
    root.append(ET.Comment('Generated from URDF; metres, kilograms, seconds. Use xuanniu_mujoco.py for detent control.'))
    ET.SubElement(root,'compiler',angle='radian',autolimits='true',inertiafromgeom='false',fusestatic='false')
    ET.SubElement(root,'option',timestep='.001',integrator='implicitfast',gravity='0 0 -9.81',iterations='100')
    ET.SubElement(root,'statistic',center='.015 0 .12',extent='.1')
    visual=ET.SubElement(root,'visual')
    ET.SubElement(visual,'global',offwidth='1000',offheight='1000')
    # Near/far scale with model extent. 30 mm near plane retains micrometre
    # screw/slot depth separation even in the distant front inspection camera.
    ET.SubElement(visual,'map',znear='.3',zfar='30')
    ET.SubElement(visual,'quality',offsamples='4',shadowsize='2048')
    ET.SubElement(visual,'headlight',ambient='.3 .3 .3',diffuse='.4 .4 .4',specular='.1 .1 .1')
    default=ET.SubElement(root,'default')
    vis=ET.SubElement(default,'default',{'class':'visual'})
    ET.SubElement(vis,'geom',contype='0',conaffinity='0',group='2',density='0')
    col=ET.SubElement(default,'default',{'class':'collision'})
    ET.SubElement(col,'geom',contype='1',conaffinity='1',group='3',density='0',rgba='1 .3 0 0',
                  friction='.7 .005 .0001',solref='.004 1')
    asset=ET.SubElement(root,'asset')
    ET.SubElement(asset,'texture',name='background',type='skybox',builtin='gradient',
                  rgb1='.12 .16 .20',rgb2='.12 .16 .20',width='512',height='3072')
    ET.SubElement(asset,'mesh',name='frame',file='../meshes/frame.stl')
    ET.SubElement(asset,'material',name='frame_material',rgba='.045 .050 .060 1',specular='.25',shininess='.35')
    groups={'frame':['frame'], 'nameplate':dae_groups(ROOT/'meshes/nameplate.dae',asset),
            'rotor':dae_groups(ROOT/'meshes/rotor.dae',asset)}
    world=ET.SubElement(root,'worldbody')
    ET.SubElement(world,'light',name='key',pos='.2 -.1 .4',dir='-.7 .3 -.7',directional='true',diffuse='.3 .3 .3',castshadow='false')
    ET.SubElement(world,'light',name='fill',pos='.1 .2 .2',dir='-.6 -.6 .2',directional='true',diffuse='.12 .12 .12',castshadow='false')
    camera(world,'front',[1.5,0,.12],[.007,0,.12],2.65)
    camera(world,'side',[.016,-.75,.12],[.016,0,.12],5.3)
    camera(world,'top',[.016,0,.87],[.016,0,.12],5.3,up=(-1,0,0))
    camera(world,'oblique',[.145,-.105,.18],[.015,0,.12],24)

    def add_body(parent,joint):
        name=joint.find('child').get('link'); link=links[name]
        body=ET.SubElement(parent,'body',name=name,**origin(joint.find('origin')))
        inertia=link.find('inertial')
        if inertia is not None:
            transform=origin(inertia.find('origin'))
            if transform.pop('quat','1 0 0 0')!='1 0 0 0': raise ValueError('Rotated inertia unsupported')
            tensor=inertia.find('inertia')
            ET.SubElement(body,'inertial',mass=inertia.find('mass').get('value'),
                          fullinertia=' '.join(tensor.get(k,'0') for k in ('ixx','iyy','izz','ixy','ixz','iyz')),**transform)
        if joint.get('type')=='revolute':
            limit=joint.find('limit')
            ET.SubElement(body,'joint',name=joint.get('name'),type='hinge',axis=joint.find('axis').get('xyz'),
                          range=limit.get('lower')+' '+limit.get('upper'),damping='.00032',
                          solreflimit='.002 1',solimplimit='.99 .999 .00001',margin='0')
        elif joint.get('type')!='fixed': raise ValueError('Unsupported joint type')
        for v in link.findall('visual'):
            stem=Path(v.find('geometry/mesh').get('filename')).stem
            for group in groups[stem]:
                ET.SubElement(body,'geom',name=group+'_visual',type='mesh',mesh=group,
                              material=group+'_material',**{'class':'visual'},**origin(v.find('origin')))
        for i,c in enumerate(link.findall('collision')):
            kind=list(c.find('geometry'))[0]; attrs=origin(c.find('origin'))
            if kind.tag=='box': attrs.update(type='box',size=fmt([x/2 for x in nums(kind.get('size'))]))
            elif kind.tag=='cylinder': attrs.update(type='cylinder',size=fmt([float(kind.get('radius')),float(kind.get('length'))/2]))
            else: raise ValueError('Unsupported collision')
            ET.SubElement(body,'geom',name=name+'_collision_'+str(i),**{'class':'collision'},**attrs)
        for child in children[name]: add_body(body,child)
    for joint in children['world']: add_body(world,joint)
    actuator=ET.SubElement(root,'actuator')
    ET.SubElement(actuator,'motor',name='xuanniu_torque',joint='xuanniu_selector_joint',gear='1',ctrlrange='-.04 .04',forcerange='-.04 .04')
    sensors=ET.SubElement(root,'sensor')
    ET.SubElement(sensors,'jointpos',name='selector_angle',joint='xuanniu_selector_joint')
    ET.SubElement(sensors,'jointvel',name='selector_velocity',joint='xuanniu_selector_joint')
    custom=ET.SubElement(root,'custom')
    for name,value in {'target_speed':math.pi/2,'position_gain':.035,'velocity_gain':.00032,
                       'detent_torque':.0015,'drag_timeout':.5,'action_timeout':3}.items():
        ET.SubElement(custom,'numeric',name=name,data=str(value))
    ET.SubElement(ET.SubElement(root,'keyframe'),'key',name='stop',qpos='0',ctrl='0')
    ET.indent(root,space='  '); ET.ElementTree(root).write(OUT/'xuanniu.xml',encoding='utf-8',xml_declaration=True)
    sources=[source,ROOT/'config/model.yaml',ROOT/'meshes/frame.stl',ROOT/'meshes/nameplate.dae',ROOT/'meshes/rotor.dae',ROOT/'materials/textures/nameplate.png']
    manifest={'model':'xuanniu.xml','units':'metres, kilograms, seconds',
              'source_sha256':{str(p.relative_to(ROOT)):hashlib.sha256(p.read_bytes()).hexdigest() for p in sources}}
    (OUT/'manifest.json').write_text(json.dumps(manifest,indent=2)+'\n')
    print('Generated',OUT/'xuanniu.xml')


if __name__=='__main__': generate()
