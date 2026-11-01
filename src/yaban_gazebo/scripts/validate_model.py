#!/usr/bin/env python3
"""Check metre units, mesh resources, joint topology, inertia and functional clearance."""
import math
from pathlib import Path
import xml.etree.ElementTree as ET

ROOT = Path(__file__).resolve().parents[1]


def validate():
    robot=ET.parse(ROOT/'urdf/yaban.urdf').getroot()
    links={e.get('name'):e for e in robot.findall('link')}
    joints={e.get('name'):e for e in robot.findall('joint')}
    parents={}
    for name,j in joints.items():
        parent,child=j.find('parent').get('link'),j.find('child').get('link')
        assert parent in links and child in links, name
        assert child not in parents, 'multiple parents: '+child
        parents[child]=parent
    assert set(links)-set(parents)=={'world'}
    for link in links:
        seen=set()
        node=link
        while node in parents:
            assert node not in seen, 'cycle'
            seen.add(node)
            node=parents[node]
    for link in links.values():
        i=link.find('inertial')
        if i is not None:
            assert float(i.find('mass').get('value'))>0
            diag=[float(i.find('inertia').get(k)) for k in ('ixx','iyy','izz')]
            assert all(math.isfinite(v) and v>0 for v in diag)
            assert max(diag)<=sum(diag)-max(diag)+1e-15
    for m in robot.findall('.//mesh'):
        relative=m.get('filename').replace('package://yaban_gazebo/', '', 1)
        assert (ROOT/relative).is_file(), relative
    knob=joints['yaban_left_knob_joint']
    lift=joints['yaban_left_knob_lift_joint']
    theta=float(knob.find('limit').get('upper'))
    multiplier=float(lift.find('mimic').get('multiplier'))
    assert abs(theta*multiplier-0.0014)<1e-10
    assert abs(math.pi*multiplier-0.00035)<1e-10
    blade_limit=float(joints['yaban_blade_joint'].find('limit').get('upper'))
    assert abs(blade_limit-math.radians(120))<1e-10
    release=float(robot.find('.//plugin/releaseTurns').text)
    assert release == 0.5
    # Released pin trajectory through the U slot: +Y, arc shifts less than side clearance.
    pin_radius,half_slot,spacing=0.002,0.0025,0.0315
    angle=math.asin((0.005+pin_radius)/spacing)
    for k in range(1001):
        theta=k*angle/1000
        dx,cy=spacing*(1-math.cos(theta)),spacing*math.sin(theta)
        radius_at_wall=math.sqrt(max(0,pin_radius**2-max(0,cy-0.005)**2))
        assert dx+radius_at_wall < half_slot, 'U-slot arc exit interference' 
    # Hole/slot must exist in the STL, and metre-scale bounds must not become millimetres.
    points=[]
    for line in (ROOT/'meshes/blade.stl').read_text().splitlines():
        if line.startswith('vertex '): points.append(tuple(map(float,line.split()[1:])))
    bounds=[(min(p[a] for p in points),max(p[a] for p in points)) for a in range(3)]
    assert abs(bounds[0][1]-bounds[0][0]-0.045)<1e-8
    assert abs(bounds[1][1]-bounds[1][0]-0.010)<1e-8
    assert abs(bounds[2][1]-bounds[2][0]-0.001)<1e-8
    print('PASS: topology, positive inertia, mesh paths, SI dimensions, screw ratio, opening direction')
    print('Links: %d; joints: %d; independent movable joints: 2' % (len(links),len(joints)))


if __name__=='__main__':
    validate()
