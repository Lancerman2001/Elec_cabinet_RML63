#!/usr/bin/env python3
"""Convert the checked-in breaker URDF to MJCF using only the standard library.

All link frames, visual geometry, collision primitives and inertia come from
the URDF. Binary STL assets are shared with Gazebo. The breaker housing has
no product decals; cabinet identification nameplates are added separately.
"""
from pathlib import Path
import hashlib
import json
import math
import xml.etree.ElementTree as ET
from xml.dom import minidom

ROOT = Path(__file__).resolve().parents[1]
OUT = Path(__file__).resolve().parent


def numbers(value):
    return [float(v) for v in value.split()]


def fmt(values):
    return ' '.join('%.12g' % v for v in values)


def quat(rpy):
    r,p,y = [v/2 for v in rpy]
    cr,sr,cp,sp,cy,sy = math.cos(r),math.sin(r),math.cos(p),math.sin(p),math.cos(y),math.sin(y)
    return [cr*cp*cy+sr*sp*sy, sr*cp*cy-cr*sp*sy, cr*sp*cy+sr*cp*sy, cr*cp*sy-sr*sp*cy]


def origin(element):
    if element is None: return {'pos':'0 0 0','quat':'1 0 0 0'}
    return {'pos': element.get('xyz','0 0 0'), 'quat':fmt(quat(numbers(element.get('rpy','0 0 0'))))}




def camera(world,name,pos,target,fovy=32):
    forward=[t-p for p,t in zip(pos,target)]; length=math.sqrt(sum(v*v for v in forward))
    f=[v/length for v in forward]
    right=[f[1],-f[0],0]; length=math.hypot(right[0],right[1]); right=[v/length for v in right]
    up=[right[1]*f[2]-right[2]*f[1],right[2]*f[0]-right[0]*f[2],right[0]*f[1]-right[1]*f[0]]
    ET.SubElement(world,'camera',name=name,pos=fmt(pos),xyaxes=fmt(right+up),fovy=str(fovy))


def generate():
    source=ROOT/'urdf/breaker.urdf'; urdf=ET.parse(source).getroot()
    manifest=json.loads((ROOT/'meshes/manifest.json').read_text())
    c=manifest['geometry']; half=math.radians(c['half_angle_deg'])
    links={link.get('name'):link for link in urdf.findall('link')}
    children={name:[] for name in links}
    for joint in urdf.findall('joint'):
        children[joint.find('parent').get('link')].append(joint)
    colors={}
    for tag in urdf.findall('gazebo'):
        color=tag.find('visual/material/diffuse')
        if color is not None: colors[tag.get('reference')]=color.text
    root=ET.Element('mujoco',model='cabinet_circuit_breaker')
    root.append(ET.Comment('Generated from ../urdf/breaker.urdf. Units: m, kg, radians.'))
    ET.SubElement(root,'compiler',angle='radian',meshdir='../meshes',
                  autolimits='true',inertiafromgeom='false',fusestatic='false')
    ET.SubElement(root,'option',timestep='0.001',integrator='implicitfast',gravity='0 0 -9.81',iterations='100')
    ET.SubElement(root,'statistic',center='0.015 0 0.12',extent='0.13')
    visual=ET.SubElement(root,'visual')
    ET.SubElement(visual,'global',offwidth='1100',offheight='1200',azimuth='-35',elevation='-20')
    ET.SubElement(visual,'headlight',ambient='0.35 0.35 0.35',diffuse='0.55 0.55 0.55',specular='0.12 0.12 0.12')
    ET.SubElement(visual,'map',znear='0.001',zfar='20')
    ET.SubElement(visual,'quality',shadowsize='2048',offsamples='4')
    default=ET.SubElement(root,'default')
    vis=ET.SubElement(default,'default',{'class':'visual'})
    ET.SubElement(vis,'geom',contype='0',conaffinity='0',group='2',density='0')
    col=ET.SubElement(default,'default',{'class':'collision'})
    ET.SubElement(col,'geom',contype='1',conaffinity='1',group='3',density='0',
                  rgba='0.9 0.3 0.1 0.0',friction='0.65 0.005 0.0001',solref='0.004 1',solimp='0.95 0.99 0.0001')
    asset=ET.SubElement(root,'asset')
    for path in sorted({Path(m.get('filename')).name for m in urdf.findall('.//visual/geometry/mesh')}):
        mesh=ROOT/'meshes'/path
        ET.SubElement(asset,'mesh',name=mesh.stem,file=mesh.name)
    world=ET.SubElement(root,'worldbody')
    ET.SubElement(world,'light',name='key',pos='0.2 -0.1 0.4',dir='-0.5 0.2 -0.8',
                  directional='true',diffuse='0.65 0.65 0.65',specular='0.12 0.12 0.12')
    ET.SubElement(world,'light',name='fill',pos='0.1 0.2 0.2',dir='-0.6 -0.6 0.2',
                  directional='true',diffuse='0.2 0.2 0.2',castshadow='false')
    camera(world,'front',[.12,0,.12],[.018,0,.12])
    camera(world,'oblique',[.105,-.076,.162],[.014,0,.12],36)
    camera(world,'side',[.017,-.125,.12],[.017,0,.12])

    def add_geom(body,element,linkname,index,collision=False):
        g=element.find('geometry'); kind=list(g)[0]; attrib=origin(element.find('origin'))
        attrib.update(name=linkname+('_collision_' if collision else '_visual_')+str(index),
                      **{'class':'collision' if collision else 'visual'})
        if not collision: attrib['rgba']=colors.get(linkname,'0.875 0.875 0.835 1')
        if linkname=='breaker_panel_link' and not collision: attrib['group']='2'
        if kind.tag=='box': attrib.update(type='box',size=fmt([v/2 for v in numbers(kind.get('size'))]))
        elif kind.tag=='cylinder': attrib.update(type='cylinder',size=fmt([float(kind.get('radius')),float(kind.get('length'))/2]))
        elif kind.tag=='sphere': attrib.update(type='sphere',size=kind.get('radius'))
        elif kind.tag=='mesh':
            name=Path(kind.get('filename')).stem
            attrib.update(type='mesh',mesh=name)
        else: raise ValueError('Unsupported URDF geometry: '+kind.tag)
        ET.SubElement(body,'geom',attrib)

    def add_body(parent,joint):
        name=joint.find('child').get('link'); link=links[name]
        if name in ['breaker_grip_center','breaker_push_up_contact','breaker_push_down_contact']:
            ET.SubElement(parent,'site',name=name,size='0.0008',rgba='0 0 0 0',group='5',**origin(joint.find('origin')))
            return
        transform=origin(joint.find('origin'))
        if joint.get('type')=='revolute':
            # ref=+40 deg + matching body orientation make default qpos0 OFF,
            # while actual joint coordinates still match Gazebo (-40 ON/+40 OFF).
            if transform['quat']!='1 0 0 0': raise ValueError('Unexpected toggle origin rotation')
            transform['quat']=fmt(quat([0,half,0]))
        body=ET.SubElement(parent,'body',name=name,**transform)
        inertia=link.find('inertial')
        if inertia is not None:
            i=inertia.find('inertia')
            a=origin(inertia.find('origin')); a.pop('quat')
            ET.SubElement(body,'inertial',mass=inertia.find('mass').get('value'),
                          fullinertia=' '.join(i.get(k,'0') for k in ['ixx','iyy','izz','ixy','ixz','iyz']),**a)
        if joint.get('type')=='revolute':
            limit=joint.find('limit')
            ET.SubElement(body,'joint',name=joint.get('name'),type='hinge',axis=joint.find('axis').get('xyz'),
                          range=limit.get('lower')+' '+limit.get('upper'),ref=str(half),
                          damping='0.0008',solreflimit='0.012 1',solimplimit='0.99 0.999 0.0001')
        elif joint.get('type')!='fixed': raise ValueError('Unexpected extra movable joint')
        for collision in (False,True):
            for i,e in enumerate(link.findall('collision' if collision else 'visual')): add_geom(body,e,name,i,collision)
        for child in children[name]: add_body(body,child)

    for joint in children['world']: add_body(world,joint)
    actuator=ET.SubElement(root,'actuator')
    ET.SubElement(actuator,'position',name='breaker_position',joint='breaker_toggle_joint',
                  kp=str(c['demo_kp']),kv='0.0016',ctrlrange=fmt([-half,half]),
                  forcerange=fmt([-c['demo_torque_limit'],c['demo_torque_limit']]))
    sensor=ET.SubElement(root,'sensor')
    ET.SubElement(sensor,'jointpos',name='toggle_angle',joint='breaker_toggle_joint')
    ET.SubElement(sensor,'jointvel',name='toggle_velocity',joint='breaker_toggle_joint')
    key=ET.SubElement(root,'keyframe')
    for name,q in [('off',half),('on',-half)]: ET.SubElement(key,'key',name=name,qpos=fmt([q]),ctrl=fmt([q]))
    custom=ET.SubElement(root,'custom')
    ET.SubElement(custom,'numeric',name='target_speed',data=str(c['target_speed']))
    ET.SubElement(custom,'numeric',name='hysteresis',data=str(math.radians(c['hysteresis_deg'])))
    ET.SubElement(custom,'numeric',name='action_timeout',data=str(c['action_timeout']))
    OUT.mkdir(exist_ok=True)
    xml=minidom.parseString(ET.tostring(root)).toprettyxml(indent='  ')
    (OUT/'breaker.xml').write_text(xml)
    sources=[source, ROOT/'meshes/manifest.json', *sorted((ROOT/'meshes').glob('*.stl'))]
    metadata={'source_sha256':{str(p.relative_to(ROOT)):hashlib.sha256(p.read_bytes()).hexdigest() for p in sources},
              'model':'breaker.xml','joint':'breaker_toggle_joint','actuator':'breaker_position','units':'metres, kilograms, radians',
              'off_angle':half,'on_angle':-half,'geometry':c}
    (OUT/'manifest.json').write_text(json.dumps(metadata,indent=2)+'\n')
    print('Generated',OUT/'breaker.xml')


if __name__=='__main__': generate()
