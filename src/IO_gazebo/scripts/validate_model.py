#!/usr/bin/python3
"""Validate the actual exported robot and swept visual envelope against the housing."""
import importlib.util
import json
import math
from pathlib import Path
import xml.etree.ElementTree as ET
import numpy as np
import yaml

ROOT = Path(__file__).resolve().parents[1]


def load_generator():
    spec = importlib.util.spec_from_file_location('generate_assets', ROOT/'scripts/generate_assets.py')
    module = importlib.util.module_from_spec(spec); spec.loader.exec_module(module)
    return module


def inside_housing(points, c, margin=0.00001):
    """Exact core collision solids used by base_link; negative penetration margin."""
    x,y,z = points.T; pw=c['pole_width']; wall=(pw-c['arm_width'])/2-0.0002
    in_box = lambda cx,cy,cz,sx,sy,sz: ((abs(x-cx)<sx/2-margin)&(abs(y-cy)<sy/2-margin)&(abs(z-cz)<sz/2-margin))
    mask = in_box(c['body_depth']/2,0,c['body_height']/2-c['label_height']/2,c['body_depth'],2*pw,c['label_height'])
    mask |= in_box(.004,0,-c['label_height']/2,.008,2*pw,c['body_height']-c['label_height'])
    mask |= in_box(c['body_depth']/2,0,-c['body_height']/2+.0008,c['body_depth'],2*pw,.0016)
    for cy in [pw-wall/2, wall/2+.0001, -wall/2-.0001, -pw+wall/2]:
        mask |= in_box(.014,cy,-c['label_height']/2,.012,wall,c['body_height']-c['label_height']-.001)
        mask |= ((x-c['pivot_x'])**2+(z-c['pivot_z'])**2 < (c['guard_radius']-margin)**2)&(abs(y-cy)<wall/2-margin)
    return mask


def validate():
    robot = ET.parse(ROOT/'urdf/breaker.urdf').getroot()
    c = yaml.safe_load((ROOT/'config/breaker.yaml').read_text()); gen=load_generator()
    links = {link.get('name'):link for link in robot.findall('link')}
    parents = {}
    for j in robot.findall('joint'):
        parent=j.find('parent').get('link'); child=j.find('child').get('link')
        assert parent in links and child in links and child not in parents
        parents[child]=parent
    assert set(links)-set(parents)=={'world'}
    for name in links:
        visited=set()
        while name in parents:
            assert name not in visited, 'Cycle in URDF'; visited.add(name); name=parents[name]
    movable=[j for j in robot.findall('joint') if j.get('type')!='fixed']
    assert len(movable)==1 and movable[0].get('type')=='revolute'
    assert movable[0].find('axis').get('xyz')=='0 1 0'
    for link in links.values():
        inertia=link.find('inertial')
        if inertia is None: continue
        assert float(inertia.find('mass').get('value'))>0
        values=[float(inertia.find('inertia').get(key)) for key in ['ixx','iyy','izz']]
        assert all(np.isfinite(values)) and min(values)>0
        assert max(values)<=sum(values)-max(values)+1e-15
    for mesh in robot.findall('.//mesh'):
        assert (ROOT/mesh.get('filename').replace('package://io_gazebo/','')).is_file()
    manifest=json.loads((ROOT/'meshes/manifest.json').read_text())
    assert manifest['geometry']==c, 'Regenerate assets after editing parameters'
    grip=gen.read_stl(ROOT/'meshes/grip.stl').reshape(-1,3)
    assert np.allclose(np.ptp(grip,axis=0),[c['grip_depth'],c['grip_width'],c['grip_height']],atol=1e-8)
    # Sweep both mesh vertices AND triangle centroids to cover wide inner faces.
    mesh=gen.read_stl(ROOT/'meshes/grip.stl')
    grip=np.concatenate([mesh.reshape(-1,3), mesh.mean(axis=1)]); grip[:,0]+=c['grip_radius']
    arm=gen.read_stl(ROOT/'meshes/arm.stl').reshape(-1,3)
    arm[:,0]+=.005
    pieces=[grip]
    for sign in [-1,1]:
        a=arm.copy(); a[:,1]+=sign*c['pole_width']/2; pieces.append(a)
        theta=np.linspace(0,2*math.pi,180)
        for offset in [-c['arm_width']/2,0,c['arm_width']/2]:
            pieces.append(np.column_stack([c['hub_radius']*np.cos(theta),np.full_like(theta,sign*c['pole_width']/2+offset),c['hub_radius']*np.sin(theta)]))
    points=np.concatenate(pieces)
    for deg in np.linspace(-c['half_angle_deg'],c['half_angle_deg'],321):
        q=math.radians(deg); rotation=np.array([[math.cos(q),0,math.sin(q)],[0,1,0],[-math.sin(q),0,math.cos(q)]])
        moved=points@rotation.T+np.array([c['pivot_x'],0,c['pivot_z']])
        colliding=inside_housing(moved,c)
        assert not colliding.any(), 'Housing intersection at %.2f deg: %s mm' % (deg, moved[colliding][0]*1000)
    assert c['pole_width']==.0175 and c['body_height']==.045 and c['label_height']==.024
    # The analytic rounded collision is a superset of the faceted visual, error <0.5 mm.
    raw=gen.read_stl(ROOT/'meshes/grip.stl'); size=np.array([c['grip_depth'],c['grip_width'],c['grip_height']])
    core=size/2-c['grip_round']; centers=raw.mean(axis=1)
    distances=np.linalg.norm(np.maximum(abs(centers)-core,0),axis=1)
    error=c['grip_round']-distances
    assert error.min()>-1e-7 and error.max()<.0005
    report={'independent_joints':1,'swept_angles':321,'grip_collision_max_surface_error_mm':float(error.max()*1000),'dimensions_mm':[35,45,20]}
    print(json.dumps(report,indent=2)); return report


if __name__=='__main__': validate()
