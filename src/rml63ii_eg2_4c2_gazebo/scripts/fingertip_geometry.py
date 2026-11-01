"""Shared split pressing tip geometry, in metres; generated STL is not a CAD edit.

The original CAD jaws close at local x=+/-17.7 mm, y=6 mm, z=28.5 mm.
Aluminium-equivalent density is an explicit simulation assumption, not measured.
"""
import math
from pathlib import Path
import struct
import xml.etree.ElementTree as ET

RADIUS = .0035
LENGTH = .006
DENSITY = 2700.
SEGMENTS = 64
# Finite contact-patch torsion. The flat inner face is 7 x 6 mm; its
# uniform-pressure mean radius is approximately 2.5 mm (mu=1). This is a
# geometry-based equivalent, not a measured coefficient for the hardware.
TORSIONAL_FRICTION = .0025
SIDES = (('left', '4C2_Link5', 1), ('right', '4C2_Link6', -1))


def values(seq):
    return ' '.join(f'{x:.12g}' for x in seq)


def properties(sign):
    mass = math.pi*RADIUS**2*LENGTH*DENSITY/2
    c = 4*RADIUS/(3*math.pi)
    com = [-sign*c, 0, LENGTH/2]
    inertia = [mass*(RADIUS**2/4+LENGTH**2/12),
               mass*(RADIUS**2/4-c*c+LENGTH**2/12),
               mass*(RADIUS**2/2-c*c)]
    return mass, com, inertia


def generate_meshes(package):
    """Identical convex half-cylinder surfaces for Gazebo and MuJoCo."""
    for side, _, sign in SIDES:
        start = math.pi/2 if sign == 1 else -math.pi/2
        arc = [(RADIUS*math.cos(start+i*math.pi/SEGMENTS),
                RADIUS*math.sin(start+i*math.pi/SEGMENTS)) for i in range(SEGMENTS+1)]
        ring = [(0., 0.), *arc]
        faces = []
        for i in range(1, len(ring)-1):
            faces.append([(0.,0.,0.), (*ring[i+1],0.), (*ring[i],0.)])
            faces.append([(0.,0.,LENGTH), (*ring[i],LENGTH), (*ring[i+1],LENGTH)])
        for a,b in zip(ring, ring[1:]+ring[:1]):
            faces += [[(*a,0.), (*b,0.), (*b,LENGTH)], [(*a,0.), (*b,LENGTH), (*a,LENGTH)]]
        path = Path(package)/'assets/gripper_meshes'/f'finger_tip_{side}.stl'
        path.parent.mkdir(parents=True, exist_ok=True)
        with path.open('wb') as out:
            out.write(b'Split pressing tip: diameter 7 mm, length 6 mm'.ljust(80,b'\0'))
            out.write(struct.pack('<I',len(faces)))
            for face in faces:
                a,b,c=face
                u=[b[i]-a[i] for i in range(3)]; v=[c[i]-a[i] for i in range(3)]
                n=[u[1]*v[2]-u[2]*v[1],u[2]*v[0]-u[0]*v[2],u[0]*v[1]-u[1]*v[0]]
                norm=math.sqrt(sum(x*x for x in n)); n=[x/norm for x in n]
                out.write(struct.pack('<12fH',*n,*a,*b,*c,0))


def add_mjcf(root):
    for side, jaw, sign in SIDES:
        ET.SubElement(root.find('asset'),'mesh',name=f'finger_tip_{side}',
                      file=f'../assets/gripper_meshes/finger_tip_{side}.stl')
        body=ET.SubElement(root.find(f'.//body[@name="{jaw}"]'),'body',
                           name=f'{side}_press_tip_link',pos=values([sign*.0177,.006,.0285]))
        mass,com,inertia=properties(sign)
        ET.SubElement(body,'inertial',mass=str(mass),pos=values(com),diaginertia=values(inertia))
        ET.SubElement(body,'geom',name=f'{side}_finger_tip',type='mesh',mesh=f'finger_tip_{side}',
                      contype='1',conaffinity='1',rgba='.75294 .75294 .75294 1',
                      friction=f'1 {TORSIONAL_FRICTION} .0001',condim='4',solref='.004 1',
                      solimp='.98 .995 .0002 .5 2',priority='1',group='0')
        ET.SubElement(body,'site',name=f'{side}_press_tip',pos=values([0,0,LENGTH]),size='.0005',group='3')


def add_urdf(root):
    ET.SubElement(root, 'link', name='press_tip_tcp')
    tcp = ET.SubElement(root, 'joint', name='press_tip_tcp_mount', type='fixed')
    ET.SubElement(tcp, 'parent', link='4C2_baselink')
    ET.SubElement(tcp, 'child', link='press_tip_tcp')
    ET.SubElement(tcp, 'origin', xyz=values([0,0,.114591+.0285+LENGTH]), rpy='0 0 0')
    for side,jaw,sign in SIDES:
        name=f'{side}_press_tip_link'
        link=ET.SubElement(root,'link',name=name)
        inertial=ET.SubElement(link,'inertial');mass,com,inertia=properties(sign)
        ET.SubElement(inertial,'origin',xyz=values(com),rpy='0 0 0')
        ET.SubElement(inertial,'mass',value=str(mass))
        ET.SubElement(inertial,'inertia',ixx=str(inertia[0]),iyy=str(inertia[1]),izz=str(inertia[2]),ixy='0',ixz='0',iyz='0')
        for kind in ['visual','collision']:
            item=ET.SubElement(link,kind,name=f'{side}_finger_tip_{kind}')
            ET.SubElement(ET.SubElement(item,'geometry'),'mesh',
                          filename=f'package://rml63ii_eg2_4c2_gazebo/assets/gripper_meshes/finger_tip_{side}.stl')
            if kind=='visual':
                ET.SubElement(ET.SubElement(item,'material',name='press_tip_silver'),'color',rgba='.75294 .75294 .75294 1')
        joint=ET.SubElement(root,'joint',name=f'{side}_press_tip_mount',type='fixed')
        ET.SubElement(joint,'parent',link=jaw);ET.SubElement(joint,'child',link=name)
        ET.SubElement(joint,'origin',xyz=values([sign*.0177,.006,.0285]),rpy='0 0 0')
        g=ET.SubElement(root,'gazebo',reference=name)
        ET.SubElement(g,'material').text='Gazebo/Grey'
        ET.SubElement(g,'selfCollide').text='false'
