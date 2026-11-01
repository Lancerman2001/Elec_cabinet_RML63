#!/usr/bin/python3
"""Reproducible metre-scale CAD meshes and baked Chinese label; no CAD runtime."""
import math
from pathlib import Path
import struct
import subprocess
import xml.etree.ElementTree as ET
import numpy as np
import yaml
from PIL import Image, ImageDraw, ImageFont

ROOT = Path(__file__).resolve().parents[1]
if not (ROOT/'config/model.yaml').exists():
    import rospkg
    ROOT = Path(rospkg.RosPack().get_path('xuanniu_gazebo'))


def rounded(width, height, radius, segments=12):
    points = []
    for cy, cz, start in ((width/2-radius, height/2-radius, 0),
                          (-width/2+radius, height/2-radius, 90),
                          (-width/2+radius, -height/2+radius, 180),
                          (width/2-radius, -height/2+radius, 270)):
        for i in range(segments+1):
            a = math.radians(start+90*i/segments)
            points.append((cy+radius*math.cos(a), cz+radius*math.sin(a)))
    return points


def extrude(contour, x0, x1, bevel=0):
    contour = np.array(contour)
    inset = contour * ((np.abs(contour).max(axis=0)-bevel)/np.abs(contour).max(axis=0))
    rings = [(x0, inset), (x0+bevel, contour), (x1-bevel, contour), (x1, inset)] if bevel else [(x0, contour), (x1, contour)]
    triangles = []
    for x, points, flip in ((x0, rings[0][1], True), (x1, rings[-1][1], False)):
        for i, yz in enumerate(points):
            tri = [(x, 0, 0), (x, *yz), (x, *points[(i+1) % len(points)])]
            triangles.append(tri[::-1] if flip else tri)
    for (x, p), (xx, q) in zip(rings, rings[1:]):
        for i in range(len(p)):
            j = (i+1) % len(p)
            a, b, c, d = (x, *p[i]), (x, *p[j]), (xx, *q[i]), (xx, *q[j])
            triangles.extend([(a,b,c), (b,d,c)])
    return triangles


def lathe(profile, n=96):
    triangles = []
    for (x,r), (xx,rr) in zip(profile, profile[1:]+profile[:1]):
        for i in range(n):
            a,b = i*2*math.pi/n, (i+1)*2*math.pi/n
            p,q = (x,r*math.cos(a),r*math.sin(a)), (xx,rr*math.cos(a),rr*math.sin(a))
            s,t = (x,r*math.cos(b),r*math.sin(b)), (xx,rr*math.cos(b),rr*math.sin(b))
            triangles.extend([(p,q,s),(s,q,t)])
    # Profile winds from the front along the outer surface toward the rear.
    return [t for t in triangles if np.linalg.norm(np.cross(np.subtract(t[1],t[0]),np.subtract(t[2],t[0]))) > 1e-14]


def recessed_frame(d):
    """Closed frame with a real 0.5 mm recess, avoiding coplanar label/backing."""
    outer=rounded(d['frame_width'],d['frame_height'],d['frame_radius'])
    inner=rounded(d['plate_width'],d['plate_height'],.0012)
    depth=d['frame_depth']; triangles=[]
    rings=[(0,outer),(depth,outer),(depth,inner),(depth-.0005,inner)]
    for (x,p),(xx,q) in zip(rings,rings[1:]):
        for i in range(len(p)):
            j=(i+1)%len(p)
            a,b,c,e=(x,*p[i]),(x,*p[j]),(xx,*q[i]),(xx,*q[j])
            triangles.extend([(a,b,c),(b,e,c)])
    for x,points,reverse in [(0,outer,True),(depth-.0005,inner,False)]:
        for i,p in enumerate(points):
            triangle=[(x,0,0),(x,*p),(x,*points[(i+1)%len(points)])]
            triangles.append(triangle[::-1] if reverse else triangle)
    return triangles


def normal(t):
    v = np.cross(np.subtract(t[1],t[0]),np.subtract(t[2],t[0]))
    return v/np.linalg.norm(v)


def stl(path, triangles):
    with path.open('wb') as f:
        f.write(b'xuanniu_gazebo; metres'.ljust(80,b'\0'))
        f.write(struct.pack('<I',len(triangles)))
        for t in triangles:
            f.write(struct.pack('<12fH',*normal(t),*np.array(t).flatten(),0))


def dae(path, groups, texture=None):
    """Flatten triangles with per-face normals; front-facing label UV is never mirrored."""
    root=ET.Element('COLLADA',xmlns='http://www.collada.org/2005/11/COLLADASchema',version='1.4.1')
    asset=ET.SubElement(root,'asset')
    ET.SubElement(asset,'created').text='2026-09-12T00:00:00Z'
    ET.SubElement(asset,'modified').text='2026-09-12T00:00:00Z'
    ET.SubElement(asset,'unit',name='meter',meter='1'); ET.SubElement(asset,'up_axis').text='Z_UP'
    if texture:
        images=ET.SubElement(root,'library_images'); im=ET.SubElement(images,'image',id='label_image')
        ET.SubElement(im,'init_from').text='../materials/textures/nameplate.png'
    effects=ET.SubElement(root,'library_effects'); materials=ET.SubElement(root,'library_materials')
    geometries=ET.SubElement(root,'library_geometries')
    scenes=ET.SubElement(root,'library_visual_scenes'); scene=ET.SubElement(scenes,'visual_scene',id='Scene')
    for name, triangles, color in groups:
        effect=ET.SubElement(effects,'effect',id=name+'_effect'); profile=ET.SubElement(effect,'profile_COMMON')
        if texture:
            surface=ET.SubElement(ET.SubElement(profile,'newparam',sid='surface'),'surface',type='2D')
            ET.SubElement(surface,'init_from').text='label_image'
            sampler=ET.SubElement(ET.SubElement(profile,'newparam',sid='sampler'),'sampler2D')
            ET.SubElement(sampler,'source').text='surface'
        phong=ET.SubElement(ET.SubElement(profile,'technique',sid='common'),'phong')
        ET.SubElement(ET.SubElement(phong,'ambient'),'color').text=' '.join(map(str,color))
        diffuse=ET.SubElement(phong,'diffuse')
        if texture: ET.SubElement(diffuse,'texture',texture='sampler',texcoord='UVMap')
        else: ET.SubElement(diffuse,'color').text=' '.join(map(str,color))
        ET.SubElement(ET.SubElement(phong,'specular'),'color').text='.3 .3 .3 1'
        ET.SubElement(ET.SubElement(phong,'shininess'),'float').text='35'
        mat=ET.SubElement(materials,'material',id=name+'_mat'); ET.SubElement(mat,'instance_effect',url='#'+name+'_effect')
        mesh=ET.SubElement(ET.SubElement(geometries,'geometry',id=name+'_geo'),'mesh')
        def source(sid, data, params):
            s=ET.SubElement(mesh,'source',id=sid)
            ET.SubElement(s,'float_array',id=sid+'_array',count=str(len(data))).text=' '.join('%.9g'%v for v in data)
            ac=ET.SubElement(ET.SubElement(s,'technique_common'),'accessor',source='#'+sid+'_array',count=str(len(data)//len(params)),stride=str(len(params)))
            for p in params: ET.SubElement(ac,'param',name=p,type='float')
        points=np.array(triangles).reshape(-1,3)
        source(name+'_positions',points.flatten(),'XYZ')
        source(name+'_normals',np.repeat([normal(t) for t in triangles],3,axis=0).flatten(),'XYZ')
        if texture: source(name+'_uv',np.array([(y/texture[0]+.5,z/texture[1]+.5) for x,y,z in points]).flatten(),'ST')
        verts=ET.SubElement(mesh,'vertices',id=name+'_verts'); ET.SubElement(verts,'input',semantic='POSITION',source='#'+name+'_positions')
        tris=ET.SubElement(mesh,'triangles',count=str(len(triangles)),material=name+'_symbol')
        ET.SubElement(tris,'input',semantic='VERTEX',source='#'+name+'_verts',offset='0')
        ET.SubElement(tris,'input',semantic='NORMAL',source='#'+name+'_normals',offset='1')
        if texture: ET.SubElement(tris,'input',semantic='TEXCOORD',source='#'+name+'_uv',offset='2',set='0')
        ET.SubElement(tris,'p').text=' '.join(str(i) for i in range(len(points)) for _ in range(3 if texture else 2))
        node=ET.SubElement(scene,'node',id=name+'_node'); inst=ET.SubElement(node,'instance_geometry',url='#'+name+'_geo')
        binding=ET.SubElement(ET.SubElement(ET.SubElement(inst,'bind_material'),'technique_common'),'instance_material',symbol=name+'_symbol',target='#'+name+'_mat')
        if texture: ET.SubElement(binding,'bind_vertex_input',semantic='UVMap',input_semantic='TEXCOORD',input_set='0')
    ET.SubElement(ET.SubElement(root,'scene'),'instance_visual_scene',url='#Scene')
    ET.ElementTree(root).write(str(path),encoding='utf-8',xml_declaration=True)


def main():
    d=yaml.safe_load((ROOT/'config/model.yaml').read_text())
    mesh=ROOT/'meshes'; mesh.mkdir(exist_ok=True)
    stl(mesh/'frame.stl',recessed_frame(d))
    shaft_end=d['shaft_length']; hub_end=shaft_end+d['hub_depth']; front=hub_end+d['handle_depth']
    shaft_radius=d['shaft_diameter']/2; hub_radius=d['hub_diameter']/2
    shaft=lathe([(shaft_end,shaft_radius),(.00005,shaft_radius),(.00005,0),(shaft_end,0)])
    hub=lathe([(hub_end,0),(hub_end,hub_radius-.0004),(hub_end-.0004,hub_radius),(shaft_end+.0004,hub_radius),(shaft_end,hub_radius-.0004),(shaft_end,0)])
    t,r,c,h=d['handle_tip_width']/2,d['handle_root_width']/2,d['handle_center_width']/2,d['handle_length']/2
    contour=[(t,h),(-t,h),(-r,c+.001),(-c+.0005,c),(-c,c-.001),(-c,-c+.001),(-c+.0005,-c),(-r,-c-.001),(-t,-h),(t,-h),(r,-c-.001),(c-.0005,-c),(c,-c+.001),(c,c-.001),(c-.0005,c),(r,c+.001)]
    handle=extrude(contour,hub_end,front,.0003)
    sr=d['screw_diameter']/2
    screw=lathe([(front+.00002,0),(front+.00002,sr-.0004),(front-.0002,sr),(front-.0005,sr),(front-.0005,0)])
    cross=extrude(rounded(.0034,.00055,.00015),front+.000025,front+.000035)+extrude(rounded(.00055,.0034,.00015),front+.000025,front+.000035)
    marker=[[(front+.00004,-.0012,h-.0055),(front+.00004,.0012,h-.0055),(front+.00004,0,h-.0018)]]
    dae(mesh/'rotor.dae',[('shaft',shaft,(.48,.51,.54,1)),('hub',hub,(.065,.070,.080,1)),('handle',handle,(.052,.060,.074,1)),('screw',screw,(.58,.61,.64,1)),('slot',cross,(.04,.05,.06,1)),('pointer',marker,(.90,.88,.77,1))])
    w,h=1000,1024
    rng=np.random.RandomState(12)
    brush=rng.normal(0,2.2,(1,w))+rng.normal(0,.4,(h,w))
    shade=178+9*np.sin(np.linspace(0,2.5*math.pi,w))[None,:]+brush
    rgb=np.clip(np.stack([shade+1,shade,shade-4],axis=2),0,255).astype(np.uint8)
    im=Image.fromarray(rgb); draw=ImageDraw.Draw(im)
    bundled=ROOT/'fonts/SelectorCJK.otf'
    font_path=str(bundled) if bundled.exists() else subprocess.check_output(['fc-match','-f','%{file}','Noto Sans CJK SC:lang=zh'],text=True).strip()
    for text,y,z in [('手动',-.015,.0173),('停',0,.0218),('自动',.015,.0173)]:
        font=ImageFont.truetype(font_path,57)
        if hasattr(draw,'textbbox'):
            bx=draw.textbbox((0,0),text,font=font)
        else:  # Ubuntu 20.04 ships Pillow 7, before ImageDraw.textbbox.
            width,height=draw.textsize(text,font=font)
            bx=(0,font.getoffset(text)[1],width,height)
        px=(y/d['plate_width']+.5)*w; py=(.5-z/d['plate_height'])*h
        draw.text((px-(bx[2]-bx[0])/2,py-(bx[3]-bx[1])/2-bx[1]),text,font=font,fill='#171b1c')
    im.save(ROOT/'materials/textures/nameplate.png')
    face=d['frame_depth']-.00005
    # A single UV-mapped quad avoids fan-triangle texture seams in Gazebo's
    # Collada renderer. The recessed frame masks its corners to a rounded shape.
    pw,ph=d['plate_width']/2,d['plate_height']/2
    corners=[(face,pw,-ph),(face,pw,ph),(face,-pw,ph),(face,-pw,-ph)]
    tris=[[corners[0],corners[1],corners[2]],[corners[0],corners[2],corners[3]]]
    dae(mesh/'nameplate.dae',[('nameplate',tris,(.7,.7,.7,1))],(d['plate_width'],d['plate_height']))
    print('Generated frame, multi-material rotor and Chinese nameplate at 1:1 metre scale.')


if __name__=='__main__': main()
