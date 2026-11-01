#!/usr/bin/env python3
"""Reproducibly assemble existing URDF/MJCF assets; never modify source packages."""
from __future__ import annotations

import hashlib
import json
import math
from pathlib import Path
import shutil
import xml.etree.ElementTree as ET
import nameplates

ROOT = Path(__file__).resolve().parents[1]
SRC = ROOT.parent
SOURCES = {
    'yaban': ('yaban_gazebo', 'yaban_mujoco.xml', 'yaban.urdf', 'yaban_mount_link'),
    'selector': ('xuanniu_gazebo', 'xuanniu.xml', 'xuanniu.urdf', 'xuanniu_mount_link'),
    'breaker': ('IO_gazebo', 'breaker.xml', 'breaker.urdf', 'breaker_mount_link'),
    'reset': ('fugui_gazebo', 'fugui.xml', 'fugui.urdf', 'fugui_mount_link'),
}


def numbers(values):
    return ' '.join(f'{v:.12g}' for v in values)


def write_xml(path, tree):
    path.parent.mkdir(parents=True, exist_ok=True)
    ET.indent(tree, space='  ')
    ET.ElementTree(tree).write(path, encoding='utf-8', xml_declaration=True)
    if tree.tag == 'mujoco':
        import mujoco_compat
        mujoco_compat.write_compatible_model(path)


def instances(cfg):
    result = []
    def add(kind, name, u, v):
        result.append(dict(kind=kind, name=name, u=u, v=v,
                           pos=[cfg['panel_x'], cfg['left_y']+u, cfg['bottom_z']+v],
                           rpy=[0, math.pi/2 if kind == 'yaban' else 0, 0]))
    p = cfg['yaban']
    for r in range(p['rows']):
        for c in range(p['columns']):
            add('yaban', f'yaban_r{r+1}_c{c+1}', p['left']+c*p['column_pitch'], p['bottom']+r*p['row_pitch'])
            result[-1]['row_from_top'] = p['rows']-r
    p = cfg['selector']
    for c in range(p['count']):
        add('selector', f'selector_{c+1:02}', p['left']+c*p['pitch'], p['bottom'])
    p = cfg['breaker']
    if p.get('alignment') == 'center':
        first = (cfg['width']-(p['count']-1)*p['pitch'])/2
    else:
        first = cfg['width']-p['right']-(p['count']-1)*p['pitch']
    for c in range(p['count']):
        add('breaker', f'breaker_{c+1:02}', first+c*p['pitch'], p['bottom'])
    for c, v in enumerate(cfg['reset']['bottom']):
        add('reset', f'reset_{c+1:02}', cfg['width']-cfg['reset']['right'], v)
    return result


def plate_color(cfg, inst):
    """Visual row order is top-down; existing instance IDs remain bottom-up."""
    colors = cfg['yaban'].get('row_colors_top_to_bottom')
    if colors is None:
        return None
    if len(colors) != cfg['yaban']['rows']:
        raise ValueError('压板配色必须按从上到下逐排指定；null表示保持源模型原色')
    color = colors[inst['row_from_top']-1]
    if color is not None and (len(color) != 4 or not all(math.isfinite(x) and 0 <= x <= 1 for x in color)):
        raise ValueError('压板颜色必须为0～1范围的RGBA四元组')
    return color


def panel_tiles(cfg, items):
    """A closed 20 mm panel with 48 square clearance holes covered by pillars."""
    tiles = [(cfg['left_y'], cfg['left_y']+cfg['width'], cfg['bottom_z'], cfg['bottom_z']+cfg['height'])]
    for inst in items:
        if inst['kind'] != 'yaban':
            continue
        for dz in (-.01575, .01575):
            y, z = inst['pos'][1], inst['pos'][2]+dz
            # 8.4 mm hole clears the OD8 sleeve; the OD13.5 front pillar hides it.
            a, b, c, d = y-.0042, y+.0042, z-.0042, z+.0042
            new = []
            for y0, y1, z0, z1 in tiles:
                ay, by, cz, dz1 = max(a, y0), min(b, y1), max(c, z0), min(d, z1)
                if ay >= by or cz >= dz1:
                    new.append((y0, y1, z0, z1))
                    continue
                for tile in [(y0, ay, z0, z1), (by, y1, z0, z1),
                             (ay, by, z0, cz), (ay, by, dz1, z1)]:
                    if tile[1]-tile[0] > 1e-9 and tile[3]-tile[2] > 1e-9:
                        new.append(tile)
            tiles = new
    return [(numbers([cfg['panel_x']-.01, (a+b)/2, (c+d)/2]),
             numbers([.01, (b-a)/2, (d-c)/2])) for a, b, c, d in tiles]


class Builder:
    def __init__(self):
        self.hashes = {}

    def record(self, path):
        self.hashes[str(path.relative_to(SRC))] = hashlib.sha256(path.read_bytes()).hexdigest()

    def parse(self, path):
        self.record(path)
        return ET.parse(path).getroot()

    def resource(self, path, package):
        path = path.resolve()
        self.record(path)
        dest = ROOT/'assets'/package/path.relative_to(SRC/package)
        dest.parent.mkdir(parents=True, exist_ok=True)
        shutil.copy2(path, dest)
        # DAE images have relative paths into the same original package layout.
        if path.suffix.lower() == '.dae':
            for node in ET.parse(path).iter():
                if node.tag.endswith('init_from') and node.text:
                    image = (path.parent/node.text.strip()).resolve()
                    if image.is_file():
                        self.resource(image, package)
        return dest.relative_to(ROOT).as_posix()

    def mjcf(self, cfg, items, tiles):
        source = SRC/'electrical_cabinet_gazebo/mujoco/electrical_cabinet.xml'
        model = self.parse(source)
        model.set('model', 'total_electrical_cabinet_glass_door')
        if model.find("asset/material[@name='door_glass']") is None:
            raise ValueError('The canonical cabinet template must have a transparent door')
        compiler = model.find('compiler')
        compiler.attrib.update(autolimits='true', fusestatic='false', inertiafromgeom='auto')
        model.find('option').attrib.update(timestep='0.001', iterations='100')
        ET.SubElement(model, 'statistic', center='.25 0 1.13', extent='2.8')
        visual = ET.SubElement(model, 'visual')
        ET.SubElement(visual, 'global', offwidth='1400', offheight='1600')
        ET.SubElement(visual, 'map', znear='.001', zfar='30')
        ET.SubElement(visual, 'headlight', ambient='.25 .25 .25', diffuse='.35 .35 .35', specular='.15 .15 .15')
        default = model.find('default')
        # Cabinet defaults must not leak into independently authored components.
        cabdef = ET.Element('default', {'class': 'cabinet'})
        for child in list(default):
            default.remove(child); cabdef.append(child)
        default.append(cabdef)
        world = model.find('worldbody')
        world.find("light[@name='key_light']").set('diffuse', '.65 .65 .65')
        world.find("light[@name='fill_light']").set('diffuse', '.25 .25 .25')
        for light in world.findall('light'):
            light.set('directional', 'true')
            # Inspection lighting: avoid cabinet-scale shadow-map aliasing on
            # millimetre-size hardware when switching to close-up cameras.
            light.set('castshadow', 'false')
        base = world.find("body[@name='cabinet_base_link']")
        base.set('childclass', 'cabinet')
        world.find("geom[@name='floor']").set('class', 'cabinet')
        base.remove(base.find("geom[@name='recess_back']"))
        # Visible mounting panel uses the cabinet's grey, like the reference photo.
        model.find("asset/material[@name='cavity_gray']").set('rgba', '.30 .32 .34 1')
        for i, (pos, size) in enumerate(tiles):
            ET.SubElement(base, 'geom', name=f'mount_panel_{i:03}', type='box', pos=pos, size=size, material='cavity_gray')
        asset = model.find('asset')
        sections = {tag: model.find(tag) for tag in ('contact', 'sensor')}
        for tag in ('actuator', 'custom', 'equality'):
            sections[tag] = ET.SubElement(model, tag)

        for inst in items:
            kind, prefix = inst['kind'], inst['name']+'__'
            pkg, filename, _, _ = SOURCES[kind]
            srcpath = SRC/pkg/'mujoco'/filename
            part = self.parse(srcpath)
            if kind == 'yaban' and (color := plate_color(cfg, inst)) is not None:
                # Colour insulation and the blade coating, not metal fasteners.
                part.find("asset/material[@name='ivory']").set('rgba', numbers(color))
                ET.SubElement(part.find('asset'), 'material', name='blade_coating',
                              rgba=numbers(color), specular='.18', shininess='.35')
                part.find(".//geom[@name='blade_visual']").set('material', 'blade_coating')
            pc = part.find('compiler')
            # Assets are shared by instances, but all materials remain independent.
            shared_names = {}
            for a in part.find('asset'):
                name = a.get('name')
                shared_names[name] = (prefix if a.tag == 'material' else kind+'__')+name
                if a.get('file'):
                    directory = pc.get('meshdir' if a.tag == 'mesh' else 'texturedir', '')
                    path = srcpath.parent/directory/a.get('file')
                    a.set('file', '../'+self.resource(path, pkg))
            names = {e.get('name'): prefix+e.get('name') for e in part.iter() if e.get('name')}
            names.update(shared_names)
            classes = {e.get('class'): prefix+e.get('class') for e in part.iter('default') if e.get('class')}
            refs = {'name', 'body', 'body1', 'body2', 'joint', 'joint1', 'joint2', 'geom', 'geom1', 'geom2',
                    'site', 'site1', 'site2', 'mesh', 'material', 'texture', 'target', 'tendon', 'actuator', 'objname'}
            for e in part.iter():
                for attr, value in list(e.attrib.items()):
                    if attr in refs and value in names:
                        e.set(attr, names[value])
                    elif attr in ('class', 'childclass') and value in classes:
                        e.set(attr, classes[value])
            local_default = part.find('default')
            local_default.set('class', prefix+'default')
            default.append(local_default)
            for a in part.find('asset'):
                if a.tag == 'texture' and a.get('type') == 'skybox':
                    continue
                if not any(x.tag == a.tag and x.get('name') == a.get('name') for x in asset):
                    asset.append(a)
            body = part.find('worldbody/body')
            body.set('pos', numbers(inst['pos']))
            body.set('quat', numbers([math.cos(math.pi/4), 0, math.sin(math.pi/4), 0]) if kind == 'yaban' else '1 0 0 0')
            body.set('childclass', prefix+'default')
            if kind == 'breaker':
                body.remove(body.find(f"body[@name='{prefix}breaker_panel_link']"))
            base.append(body)
            for tag, section in sections.items():
                other = part.find(tag)
                if other is not None:
                    section.extend(list(other))
            if kind == 'yaban':
                # A single radial stripe makes rotation visible on the
                # otherwise ten-fold symmetric knob. It has no mass/contact.
                knob = body.find(f".//body[@name='{prefix}left_knob']")
                ET.SubElement(knob, 'geom', name=prefix+'knob_rotation_mark',
                              type='box', pos='0.0025 0 0.01402',
                              size='0.002 0.00045 0.00002', rgba='.03 .12 .85 1',
                              contype='0', conaffinity='0', mass='0', group='2')
                # Retain contact geometry, but omit visible grasp-marker spheres.
                for site in body.iter('site'):
                    site.set('rgba', '0 0 0 0')
                for geom in body.iter('geom'):
                    if geom.get('name', '').endswith('_collision'):
                        geom.attrib.pop('material', None)
                        geom.set('rgba', '0 0 0 0')
                        geom.set('group', '3')
                # Legacy thread geometry uses two solid cylinders (stud/nut).
                # They represent a screw pair, not colliding solids.
                ET.SubElement(sections['contact'], 'exclude',
                              body1=prefix+'left_terminal', body2=prefix+'left_knob')
                # The original demonstration prescribes these three coordinates.
                # Host pins only this instance; no global qvel clearing.
            if kind == 'breaker':
                self.lamp_mjcf(base, asset, cfg, inst)
        self.cameras(world)
        for tag in ('actuator', 'custom', 'equality'):
            if len(sections[tag]) == 0:
                model.remove(sections[tag])
        write_xml(ROOT/'mujoco/total_cabinet_glass_door.xml', model)
        # Compile first to obtain actual qpos/ctrl order; do not assume tree order.
        import mujoco_compat as mujoco
        m = mujoco.load_model(str(ROOT/'mujoco/total_cabinet_glass_door.xml'))
        d = mujoco.MjData(m)
        initial = {'door_hinge_joint': math.pi/2, 'handle_turn_joint': math.pi/2, 'handle_pop_joint': -math.pi/3, 'button_press_joint': 0}
        for inst in items:
            if inst['kind'] == 'breaker':
                initial[inst['name']+'__breaker_toggle_joint'] = math.radians(40)
                d.ctrl[m.actuator(inst['name']+'__breaker_position').id] = math.radians(40)
        for name, q in initial.items():
            d.qpos[m.jnt_qposadr[m.joint(name).id]] = q
        mujoco.mj_forward(m, d)
        nameplates.add_mjcf(ROOT, cfg, items, m, d, base, asset)
        # Keep stable material ordering in the compiled transparent models.
        for name in ('door_panel_collision_hidden', 'door_glass'):
            material = asset.find(f"material[@name='{name}']")
            asset.remove(material)
            asset.append(material)
        keyframe = ET.SubElement(model, 'keyframe')
        ET.SubElement(keyframe, 'key', name='initial_open', qpos=numbers(d.qpos), ctrl=numbers(d.ctrl))
        write_xml(ROOT/'mujoco/total_cabinet_glass_door.xml', model)
        return initial

    @staticmethod
    def lamp_mjcf(base, asset, cfg, inst):
        lamp = cfg['lamp']; name = inst['name']+'__red_lamp'
        ET.SubElement(asset, 'material', name=name, rgba='.15 .008 .005 1', emission='0', specular='.35', shininess='.5')
        x, y, z = inst['pos']
        ET.SubElement(base, 'geom', name=name, type='cylinder', material=name,
                      pos=numbers([x+lamp['thickness']/2, y, z+lamp['height_above_breaker']]),
                      quat='.707106781187 0 .707106781187 0', size=numbers([lamp['radius'], lamp['thickness']/2]))

    @staticmethod
    def cameras(world):
        import numpy as np
        views = {
            'front': ([3.9, .13, 1.18], [.20, .13, 1.18], 43),
            'oblique': ([3, -2.5, 2.25], [.25, .0, 1.13], 40),
            'plates': ([.42, -.38, .56], [.21, -.225, .505], 24),
            'breakers': ([.92, 0, 1.09], [.21, 0, 1.03], 37),
            'controls': ([1.0, .02, 1.40], [.21, .02, 1.38], 47),
        }
        for name, (pos, target, fov) in views.items():
            z = np.array(pos)-target; z /= np.linalg.norm(z)
            x = np.cross([0, 0, 1], z); x /= np.linalg.norm(x)
            y = np.cross(z, x)
            ET.SubElement(world, 'camera', name=name, pos=numbers(pos), xyaxes=numbers([*x, *y]), fovy=str(fov))

    def urdf(self, cfg, items, tiles):
        model = self.parse(SRC/'electrical_cabinet_gazebo/urdf/electrical_cabinet.urdf')
        model.set('name', 'total_electrical_cabinet')
        for e in list(model):
            if e.tag not in ('link', 'joint', 'material'):
                model.remove(e)
        # Legacy Gazebo cabinet used blue handles and omitted some collisions.
        # Keep its link/joint tree, align visible geometry and limits to current MJCF.
        cabinet = self.parse(SRC/'electrical_cabinet_gazebo/mujoco/electrical_cabinet.xml')
        colors = {m.get('name'): m.get('rgba') for m in cabinet.findall('asset/material')}
        for link in model.findall('link'):
            for e in list(link):
                if e.tag in ('visual', 'collision'): link.remove(e)
        for body in cabinet.findall('.//body'):
            link = model.find(f"link[@name='{body.get('name')}']")
            if link is None: continue
            for geom in body.findall('geom'):
                if geom.get('name') == 'recess_back': continue
                size = list(map(float, geom.get('size').split()))
                material = geom.get('material')
                rgba = geom.get('rgba', colors.get(material, '1 1 1 1'))
                tags = []
                if float(rgba.split()[3]) > 0:
                    tags.append('visual')
                # Glass/frame geoms are visual only. Hidden original panels
                # retain their existing collision shape, without a visual.
                if int(geom.get('contype', '1')) or int(geom.get('conaffinity', '1')):
                    tags.append('collision')
                for tag in tags:
                    target = link
                    if geom.get('name') == 'door_glass_panel' and tag == 'visual':
                        target = model.find("link[@name='door_glass_visual_link']")
                    e = ET.SubElement(target, tag, name=geom.get('name')+'_'+tag)
                    ET.SubElement(e, 'origin', xyz=geom.get('pos', '0 0 0'), rpy='0 0 0')
                    shape = ET.SubElement(e, 'geometry')
                    if geom.get('type') == 'box': ET.SubElement(shape, 'box', size=numbers([2*v for v in size]))
                    else: raise ValueError('Unexpected cabinet primitive')
                    if tag == 'visual':
                        ET.SubElement(ET.SubElement(e, 'material', name=material), 'color', rgba=rgba)
            for joint in body.findall('joint'):
                uj = model.find(f"joint[@name='{joint.get('name')}']")
                lo, hi = joint.get('range').split()
                uj.find('limit').attrib.update(lower=lo, upper=hi)
        base = model.find("link[@name='cabinet_base_link']")
        for e in list(base):
            if e.get('name', '').startswith('door_recess_back'):
                base.remove(e)
        for i, (pos, half) in enumerate(tiles):
            for tag in ('visual', 'collision'):
                e = ET.SubElement(base, tag, name=f'mount_panel_{i:03}_{tag}')
                ET.SubElement(e, 'origin', xyz=pos, rpy='0 0 0')
                ET.SubElement(ET.SubElement(e, 'geometry'), 'box', size=numbers([float(v)*2 for v in half.split()]))
                if tag == 'visual':
                    mat = ET.SubElement(e, 'material', name='mount_panel_gray')
                    ET.SubElement(mat, 'color', rgba='.30 .32 .34 1')
        for inst in items:
            kind, prefix = inst['kind'], inst['name']+'__'
            pkg, _, filename, mount = SOURCES[kind]
            part = self.parse(SRC/pkg/'urdf'/filename)
            if kind == 'yaban' and (color := plate_color(cfg, inst)) is not None:
                part.find("material[@name='yaban_ivory']/color").set('rgba', numbers(color))
                coating = ET.SubElement(part, 'material', name='yaban_blade_coating')
                ET.SubElement(coating, 'color', rgba=numbers(color))
                for mat in part.findall("link[@name='yaban_blade_link']/visual/material"):
                    mat.set('name', 'yaban_blade_coating')
            for g in part.findall('gazebo'):
                color = g.find('visual/material/diffuse')
                link = part.find(f"link[@name='{g.get('reference')}']")
                if color is not None and link is not None:
                    for visual in link.findall('visual'):
                        if visual.find('material') is None:
                            ET.SubElement(ET.SubElement(visual, 'material', name=link.get('name')+'_color'), 'color', rgba=color.text.strip())
            remove = {'world'} | ({'breaker_panel_link'} if kind == 'breaker' else set())
            for e in list(part):
                if e.tag not in ('link', 'joint', 'material') or e.get('name') in remove:
                    part.remove(e)
                elif e.tag == 'joint' and (e.find('parent').get('link') in remove or e.find('child').get('link') in remove):
                    part.remove(e)
            for e in part.iter():
                for attr in ('name', 'link', 'joint'):
                    if e.get(attr):
                        e.set(attr, prefix+e.get(attr))
                for attr in ('filename',):
                    if e.get(attr):
                        uri = e.get(attr)
                        if uri.startswith('package://'):
                            package, relative = uri[len('package://'):].split('/', 1)
                            if package == 'io_gazebo':
                                package = 'IO_gazebo'
                            res = self.resource(SRC/package/relative, package)
                            e.set(attr, 'package://total_cabinet_gazebo/'+res)
                        else:
                            raise ValueError(f'Unsupported resource URI: {uri}')
            model.extend(list(part))
            joint = ET.SubElement(model, 'joint', name=prefix+'cabinet_mount', type='fixed')
            ET.SubElement(joint, 'parent', link='cabinet_base_link')
            ET.SubElement(joint, 'child', link=prefix+mount)
            ET.SubElement(joint, 'origin', xyz=numbers(inst['pos']), rpy=numbers(inst['rpy']))
            if kind == 'breaker':
                lamp = cfg['lamp']; name=prefix+'red_lamp'
                link = ET.SubElement(model, 'link', name=name)
                for tag in ('visual', 'collision'):
                    e = ET.SubElement(link, tag)
                    ET.SubElement(e, 'origin', xyz='0 0 0', rpy=numbers([0, math.pi/2, 0]))
                    ET.SubElement(ET.SubElement(e, 'geometry'), 'cylinder', radius=str(lamp['radius']), length=str(lamp['thickness']))
                    if tag == 'visual':
                        ET.SubElement(ET.SubElement(e, 'material', name=name), 'color', rgba='.15 .008 .005 1')
                joint = ET.SubElement(model, 'joint', name=name+'_mount', type='fixed')
                ET.SubElement(joint, 'parent', link='cabinet_base_link')
                ET.SubElement(joint, 'child', link=name)
                x,y,z=inst['pos']
                ET.SubElement(joint, 'origin', xyz=numbers([x+lamp['thickness']/2,y,z+lamp['height_above_breaker']]), rpy='0 0 0')
        nameplates.add_urdf(ROOT, cfg, items, model)
        write_xml(ROOT/'urdf/total_cabinet_glass.urdf', model)

    def protocols(self):
        for pkg, filename, target in [
            ('yaban_gazebo', 'mujoco/yaban_mujoco_protocol.py', 'yaban_protocol.py'),
            ('electrical_cabinet_gazebo', 'mujoco/door_close_protocol.py', 'door_protocol.py'),
            ('fugui_gazebo', 'mujoco/tk_runtime.py', 'tk_runtime.py'),
            ('IO_gazebo', 'mujoco/image_utils.py', 'image_utils.py'),
        ]:
            src = SRC/pkg/filename
            self.record(src)
            shutil.copy2(src, ROOT/'mujoco'/target)

    def run(self):
        cfg = json.loads((ROOT/'config/layout.json').read_text())
        nameplates.validate_font(cfg['nameplate'])
        self.record(ROOT/'config/layout.json')
        self.record(Path(__file__))
        self.record(Path(nameplates.__file__))
        items = instances(cfg); tiles = panel_tiles(cfg, items)
        initial = self.mjcf(cfg, items, tiles)
        self.urdf(cfg, items, tiles)
        for path in sorted((ROOT/'assets/nameplates').iterdir()):
            self.record(path)
        self.protocols()
        self.resource(SRC/'xuanniu_gazebo/LICENSE', 'xuanniu_gazebo')
        for pkg, relative in [('IO_gazebo', 'mujoco/breaker_backend.py'),
                              ('fugui_gazebo', 'mujoco/fugui_backend.py'),
                              ('xuanniu_gazebo', 'mujoco/selector_backend.py'),
                              ('yaban_gazebo', 'mujoco/yaban_mujoco.py'),
                              ('electrical_cabinet_gazebo', 'mujoco/view_electrical_cabinet.py')]:
            self.record(SRC/pkg/relative)
        # URDF names differ from legacy MJCF only for pressure-plate joints.
        for i in items:
            if i['kind'] == 'yaban':
                for j in ('left_knob_joint', 'left_knob_lift_joint', 'blade_joint'):
                    initial[i['name']+'__yaban_'+j] = 0
            elif i['kind'] == 'selector':
                initial[i['name']+'__xuanniu_selector_joint'] = 0
            elif i['kind'] == 'reset':
                initial[i['name']+'__fugui_press_joint'] = 0
        (ROOT/'config/initial_joint_positions.json').write_text(json.dumps(initial, indent=2)+'\n')
        (ROOT/'manifest.json').write_text(json.dumps(dict(source_sha256=self.hashes, instances=items, panel_tiles=len(tiles)), indent=2)+'\n')
        print(f'Generated URDF + MJCF: {len(items)} components, 10 lamps, {len(tiles)} panel tiles.')
        from build_gazebo import build
        build()


if __name__ == '__main__':
    Builder().run()
