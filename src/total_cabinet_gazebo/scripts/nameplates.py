"""Dimensioned component nameplates shared by the MJCF and URDF builders."""
from pathlib import Path
import hashlib
import json
import os
import xml.etree.ElementTree as ET

import numpy as np
from PIL import Image, ImageDraw, ImageFont


def numbers(values):
    return ' '.join(f'{float(v):.12g}' for v in values)


def validate_font(spec):
    if os.environ.get('CABINET_REUSE_NAMEPLATES') == '1':
        return
    path = Path(os.environ.get('CABINET_NAMEPLATE_FONT', spec['font']['file']))
    if not path.is_file():
        raise FileNotFoundError('宋体字体不存在；请用 CABINET_NAMEPLATE_FONT 指定 simsun.ttc：'+str(path))
    family = ImageFont.truetype(str(path), 200, index=spec['font'].get('index',0)).getname()
    if family[0] not in ('SimSun', '宋体'):
        raise ValueError('铭牌要求宋体，实际字体为 '+str(family))


def visual_bounds(model, data, instance):
    """Actual visible geometry in the declared initial assembly pose."""
    import mujoco_compat as mj
    lower, upper = [], []
    for gid in range(model.ngeom):
        name = model.geom(gid).name or ''
        if not name.startswith(instance+'__') or name.endswith('__red_lamp'):
            continue
        if model.geom_group[gid] == 3 or model.geom_rgba[gid, 3] == 0:
            continue
        pos = data.geom_xpos[gid]
        rotation = data.geom_xmat[gid].reshape(3, 3)
        kind, size = model.geom_type[gid], model.geom_size[gid]
        if kind == mj.mjtGeom.mjGEOM_MESH:
            mesh = model.geom_dataid[gid]
            start, count = model.mesh_vertadr[mesh], model.mesh_vertnum[mesh]
            vertices = model.mesh_vert[start:start+count] @ rotation.T + pos
            lower.append(vertices.min(axis=0)); upper.append(vertices.max(axis=0))
            continue
        if kind == mj.mjtGeom.mjGEOM_BOX:
            half = abs(rotation) @ size
        elif kind == mj.mjtGeom.mjGEOM_SPHERE:
            half = np.full(3, size[0])
        elif kind == mj.mjtGeom.mjGEOM_CYLINDER:
            half = np.sqrt(np.sum(rotation[:, :2]**2, axis=1))*size[0] + abs(rotation[:, 2])*size[1]
        elif kind == mj.mjtGeom.mjGEOM_CAPSULE:
            half = size[0] + abs(rotation[:, 2])*size[1]
        else:
            raise ValueError(f'Unsupported component visual: {name}, type {kind}')
        lower.append(pos-half); upper.append(pos+half)
    if not lower:
        raise ValueError('Missing component visuals: '+instance)
    return np.min(lower, axis=0), np.max(upper, axis=0)


def assign_codes(items):
    """Front view: +Y points right, +Z points up; IDs retain their old order."""
    prefixes = {'yaban': '1LP', 'selector': 'QK', 'breaker': '1CLP', 'reset': 'FA'}
    for kind, prefix in prefixes.items():
        group = [item for item in items if item['kind'] == kind]
        if kind == 'yaban':
            group.sort(key=lambda item: (-item['pos'][2], item['pos'][1]))
        elif kind == 'reset':
            group.sort(key=lambda item: -item['pos'][2])
        else:
            group.sort(key=lambda item: item['pos'][1])
        for index, item in enumerate(group, 1):
            item['nameplate'] = {'code': f'{index}{prefix}' if kind in ('selector', 'reset') else f'{prefix}{index}'}


def texture(root, code, width, height, font_config):
    """Render literal identifiers; each plate has its own readable asset."""
    directory = root/'assets/nameplates'
    directory.mkdir(parents=True, exist_ok=True)
    if os.environ.get('CABINET_REUSE_NAMEPLATES') == '1':
        saved = directory/(code+'.png')
        expected = (round(width*20000), round(height*20000))
        with Image.open(saved) as image:
            if image.size != expected:
                raise ValueError(f'Nameplate {code}: expected {expected}, got {image.size}')
        return
    font_path = Path(os.environ.get('CABINET_NAMEPLATE_FONT', font_config['file']))
    if not font_path.is_file():
        raise FileNotFoundError('宋体字体不存在；请用 CABINET_NAMEPLATE_FONT 指定 simsun.ttc：'+str(font_path))
    font_index = font_config.get('index', 0)
    scale = 20000  # 20 pixels/mm, identical physical font height on every plate.
    w, h = round(width*scale), round(height*scale)
    im = Image.new('RGB', (w, h), '#080808')
    draw = ImageDraw.Draw(im)
    border = round(.0007*scale)
    draw.rectangle((border, border, w-border-1, h-border-1), fill='#ffffff')
    size = round(.010*scale)
    while True:
        font = ImageFont.truetype(str(font_path), size, index=font_index)
        if font.getname()[0] not in ('SimSun', '宋体'):
            raise ValueError('铭牌要求宋体，实际字体为 '+str(font.getname()))
        box = draw.textbbox((0, 0), code, font=font)
        tw, th = box[2]-box[0], box[3]-box[1]
        if tw <= w-2*border-40 and th <= h-2*border-30:
            break
        size -= 1
    draw.text(((w-tw)/2-box[0], (h-th)/2-box[1]), code, font=font, fill='#080808')
    im.save(directory/(code+'.png'))


def face_geometry(width, height, thickness):
    y, z, x = width/2, height/2, thickness/2
    front = [[x, y, -z], [x, -y, -z], [x, -y, z], [x, y, z]]
    back = [[x-.00002, yy, zz] for _, yy, zz in front]
    faces = [0,2,1, 0,3,2, 4,5,6, 4,6,7,
             0,1,5, 0,5,4, 1,2,6, 1,6,5, 2,3,7, 2,7,6, 3,0,4, 3,4,7]
    return front+back, faces


def add_mjcf(root, cfg, items, model, data, base, asset):
    spec = cfg['nameplate']
    width, height, thickness = (spec[key] for key in ('width', 'height', 'thickness'))
    assign_codes(items)
    vertices, faces = face_geometry(width, height, thickness)
    ET.SubElement(asset, 'mesh', name='component_nameplate_face', vertex=numbers(np.ravel(vertices)),
                  face=numbers(faces), texcoord='1 1 0 1 0 0 1 0 1 1 0 1 0 0 1 0')
    ET.SubElement(asset, 'material', name='component_nameplate_black', rgba='.03 .03 .03 1', specular='.1')
    for item in items:
        low, high = visual_bounds(model, data, item['name'])
        plate = item['nameplate']; code = plate['code']; prefix = 'nameplate__'+code
        center = [cfg['panel_x']+thickness/2, (low[1]+high[1])/2, low[2]-spec['center_below_component']]
        plate.update(center_m=center, size_xyz_m=[thickness, width, height],
                     component_bounds_m=[low.tolist(), high.tolist()],
                     boundary_reference='visible assembly at initial_open, excluding indicator lamp',
                     texture='assets/nameplates/'+code+'.png')
        texture(root, code, width, height, spec['font'])
        ET.SubElement(asset, 'texture', name=prefix+'_texture', type='2d', file='../'+plate['texture'])
        ET.SubElement(asset, 'material', name=prefix+'_print', texture=prefix+'_texture',
                      rgba='1 1 1 1', specular='.05', shininess='.1')
        body = ET.SubElement(base, 'body', name=prefix, pos=numbers(center))
        mass = .003
        inertia = mass/12*np.array([width**2+height**2, thickness**2+height**2, thickness**2+width**2])
        ET.SubElement(body, 'inertial', pos='0 0 0', mass=str(mass), diaginertia=numbers(inertia))
        ET.SubElement(body, 'geom', name=prefix+'_back', type='box', pos='-.00001 0 0',
                      size=numbers([thickness/2-.00001,width/2,height/2]),
                      material='component_nameplate_black', contype='0', conaffinity='0', mass='0', group='2')
        ET.SubElement(body, 'geom', name=prefix+'_face', type='mesh', mesh='component_nameplate_face',
                      material=prefix+'_print', contype='0', conaffinity='0', mass='0', group='2')
        ET.SubElement(body, 'geom', name=prefix+'_collision', type='box',
                      size=numbers([thickness/2,width/2,height/2]),
                      contype='1', conaffinity='1', mass='0', rgba='0 0 0 0', group='3')
    if os.environ.get('CABINET_REUSE_NAMEPLATES') != '1':
        font_path = Path(os.environ.get('CABINET_NAMEPLATE_FONT', spec['font']['file']))
        provenance = dict(family='SimSun', style='Regular', index=spec['font'].get('index',0),
                          source_path=str(font_path), sha256=hashlib.sha256(font_path.read_bytes()).hexdigest())
        (root/'assets/nameplates/font_provenance.json').write_text(json.dumps(provenance,indent=2)+'\n')


def write_dae(path, width, height, thickness, code):
    """Collada uses bottom-origin V, whereas inline MJCF uses top-origin V."""
    vertices, _ = face_geometry(width, height, thickness)
    ns = 'http://www.collada.org/2005/11/COLLADASchema'
    ET.register_namespace('', ns)
    def element(parent, tag, **attributes):
        return ET.SubElement(parent, '{'+ns+'}'+tag, **attributes)
    tree = ET.Element('{'+ns+'}COLLADA', version='1.4.1')
    info = element(tree, 'asset'); element(info, 'unit', name='meter', meter='1'); element(info, 'up_axis').text = 'Z_UP'
    img = element(element(tree, 'library_images'), 'image', id='label')
    element(img, 'init_from').text = code+'.png'
    profile = element(element(element(tree, 'library_effects'), 'effect', id='effect'), 'profile_COMMON')
    surface = element(element(profile, 'newparam', sid='surface'), 'surface', type='2D')
    element(surface, 'init_from').text = 'label'
    sampler = element(element(profile, 'newparam', sid='sampler'), 'sampler2D')
    element(sampler, 'source').text = 'surface'
    phong = element(element(profile, 'technique', sid='common'), 'phong')
    element(element(phong, 'diffuse'), 'texture', texture='sampler', texcoord='UVMap')
    material = element(element(tree, 'library_materials'), 'material', id='material')
    element(material, 'instance_effect', url='#effect')
    mesh = element(element(element(tree, 'library_geometries'), 'geometry', id='geometry'), 'mesh')
    for name, values, stride, params in [('positions',np.ravel(vertices[:4]),3,'XYZ'), ('uv',[1,0,0,0,0,1,1,1],2,'ST')]:
        source = element(mesh, 'source', id=name)
        element(source, 'float_array', id=name+'-array', count=str(len(values))).text = numbers(values)
        accessor = element(element(source, 'technique_common'), 'accessor', source='#'+name+'-array', count='4', stride=str(stride))
        for param in params:element(accessor, 'param', name=param, type='float')
    element(element(mesh, 'vertices', id='verts'), 'input', semantic='POSITION', source='#positions')
    triangles = element(mesh, 'triangles', count='2', material='print')
    element(triangles, 'input', semantic='VERTEX', source='#verts', offset='0')
    element(triangles, 'input', semantic='TEXCOORD', source='#uv', offset='1', set='0')
    element(triangles, 'p').text = '0 0 2 2 1 1 0 0 3 3 2 2'
    scene = element(element(tree, 'library_visual_scenes'), 'visual_scene', id='Scene')
    geometry = element(element(scene, 'node', id='plate'), 'instance_geometry', url='#geometry')
    bound = element(element(element(geometry, 'bind_material'), 'technique_common'), 'instance_material', symbol='print', target='#material')
    element(bound, 'bind_vertex_input', semantic='UVMap', input_semantic='TEXCOORD', input_set='0')
    element(element(tree, 'scene'), 'instance_visual_scene', url='#Scene')
    ET.indent(tree, space='  '); ET.ElementTree(tree).write(path, encoding='utf-8', xml_declaration=True)


def add_urdf(root, cfg, items, robot):
    for item in items:
        plate = item['nameplate']; code = plate['code']; name = 'nameplate__'+code
        thickness, width, height = plate['size_xyz_m']
        dae = root/'assets/nameplates'/f'{code}.dae'
        write_dae(dae, width, height, thickness, code)
        link = ET.SubElement(robot, 'link', name=name)
        inertial = ET.SubElement(link, 'inertial')
        ET.SubElement(inertial, 'mass', value='.003')
        diagonal = .003/12*np.array([width**2+height**2, thickness**2+height**2, thickness**2+width**2])
        ET.SubElement(inertial, 'inertia', ixx=str(diagonal[0]), iyy=str(diagonal[1]), izz=str(diagonal[2]),
                      ixy='0', ixz='0', iyz='0')
        back = ET.SubElement(link, 'visual', name=name+'_back')
        ET.SubElement(back, 'origin', xyz='-.00001 0 0', rpy='0 0 0')
        ET.SubElement(ET.SubElement(back, 'geometry'), 'box', size=numbers([thickness-.00002,width,height]))
        ET.SubElement(ET.SubElement(back, 'material', name='component_nameplate_black'), 'color', rgba='.03 .03 .03 1')
        front = ET.SubElement(link, 'visual', name=name+'_face')
        ET.SubElement(ET.SubElement(front, 'geometry'), 'mesh', filename='package://total_cabinet_gazebo/'+dae.relative_to(root).as_posix())
        collision = ET.SubElement(link, 'collision', name=name+'_collision')
        ET.SubElement(ET.SubElement(collision, 'geometry'), 'box', size=numbers([thickness,width,height]))
        joint = ET.SubElement(robot, 'joint', name=name+'_mount', type='fixed')
        ET.SubElement(joint, 'parent', link='cabinet_base_link'); ET.SubElement(joint, 'child', link=name)
        ET.SubElement(joint, 'origin', xyz=numbers(plate['center_m']), rpy='0 0 0')
