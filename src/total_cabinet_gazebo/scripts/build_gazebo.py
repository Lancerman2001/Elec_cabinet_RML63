#!/usr/bin/env python3
"""Add Gazebo Classic extensions without changing the portable URDF / MJCF."""
import copy
import hashlib
import json
from pathlib import Path
import xml.etree.ElementTree as ET

ROOT = Path(__file__).resolve().parents[1]
SOURCES = {
    'yaban': ('yaban_gazebo', 'yaban.urdf'),
    'selector': ('xuanniu_gazebo', 'xuanniu.urdf'),
    'breaker': ('IO_gazebo', 'breaker.urdf'),
    'reset': ('fugui_gazebo', 'fugui.urdf'),
}


def indent(node, level=0):
    # ROS Noetic's system Python 3.8 does not have ET.indent.
    if len(node):
        node.text = '\n' + '  ' * (level + 1)
        for child in node:
            indent(child, level + 1)
        child.tail = '\n' + '  ' * level
    node.tail = '\n' + '  ' * level


def build():
    path = ROOT / 'urdf/total_cabinet_glass.urdf'
    robot = ET.parse(path).getroot()
    items = json.loads((ROOT / 'manifest.json').read_text())['instances']
    interaction = json.loads((ROOT/'config/interaction.json').read_text())
    sources = {str(path.relative_to(ROOT)): hashlib.sha256(path.read_bytes()).hexdigest()}
    cabinet_path = ROOT.parent / 'electrical_cabinet_gazebo/urdf/electrical_cabinet.urdf'
    cabinet = ET.parse(cabinet_path).getroot()
    sources[str(cabinet_path.relative_to(ROOT.parent))] = hashlib.sha256(cabinet_path.read_bytes()).hexdigest()

    # The original Gazebo door protocol uses a half-turn cam for 6 mm button
    # travel, not the MJCF slider limits. Restore its matched geometry + joints.
    for tag, names in [('joint', ('button_press_joint', 'handle_pop_joint',
                                  'handle_turn_joint', 'door_hinge_joint')),
                       ('link', ('lock_button_link', 'button_contact_sensor_link'))]:
        for name in names:
            old = robot.find("%s[@name='%s']" % (tag, name))
            replacement = copy.deepcopy(cabinet.find("%s[@name='%s']" % (tag, name)))
            if name == 'lock_button_link':
                # Keep the assembled cabinet's visual finish.
                replacement.find('visual').remove(replacement.find('visual/material'))
                replacement.find('visual').append(copy.deepcopy(old.find('visual/material')))
            robot.remove(old)
            robot.append(replacement)

    # The door plugin makes the base kinematic, anchoring it without freezing
    # the component joints. Do not also constrain that kinematic body to world.

    existing = {e.get('name') for e in robot if e.tag in ('link', 'joint')}
    for inst in items:
        package, filename = SOURCES[inst['kind']]
        source_path = ROOT.parent / package / 'urdf' / filename
        sources[str(source_path.relative_to(ROOT.parent))] = hashlib.sha256(source_path.read_bytes()).hexdigest()
        source = ET.parse(source_path).getroot()
        prefix = inst['name'] + '__'
        if inst['kind'] == 'reset':
            # Retain a dynamic mounting body between the ODE slider and the
            # kinematically held cabinet instead of lumping the two together.
            g = ET.SubElement(robot, 'gazebo', reference=prefix + 'cabinet_mount')
            ET.SubElement(g, 'preserveFixedJoint').text = 'true'
        for extension in source.findall('gazebo'):
            g = copy.deepcopy(extension)
            ref = g.get('reference')
            if ref:
                if prefix + ref not in existing:
                    continue  # e.g. the removed standalone breaker mounting panel
                g.set('reference', prefix + ref)
                for child in list(g):
                    if child.tag in ('visual', 'material'):
                        g.remove(child)  # rebuilt below from current row colors
            for plugin in g.findall('plugin'):
                plugin.set('name', prefix + plugin.get('name'))
                for node in plugin:
                    if node.tag in ('jointPrefix', 'prefix', 'jointName'):
                        node.text = prefix + node.text.strip()
                    elif node.tag == 'robotNamespace':
                        node.text = '/total_cabinet/' + inst['name']
                    elif node.tag == 'demoControl':
                        node.text = 'true'
                if inst['kind'] == 'selector' and plugin.get('filename') == 'libxuanniu_selector_plugin.so':
                    for tag,key in [('velocityGain','damping'),('detentTorque','detent_torque')]:
                        node=plugin.find(tag)
                        if node is None:node=ET.SubElement(plugin,tag)
                        node.text=str(interaction['selector'][key])
            robot.append(g)

    for extension in cabinet.findall('gazebo'):
        g = copy.deepcopy(extension)
        if g.get('reference'):
            for child in list(g):
                if child.tag in ('visual', 'material'):
                    g.remove(child)
            if g.get('reference') != 'door_glass_visual_link' or len(g):
                robot.append(g)
        else:
            for plugin in g.findall('plugin'):
                if plugin.get('filename') == 'libelectrical_cabinet_kinematic_plugin.so':
                    ET.SubElement(robot, 'gazebo').append(plugin)

    # Gazebo Classic ignores URDF RGBA. Its visual material extension applies
    # to every visual in a link. Split ONLY mixed-color links into fixed visual
    # groups; the SDF converter lumps them back, adding no dynamic bodies.
    palette = {m.get('name'): m.find('color').get('rgba')
               for m in robot.findall('material') if m.find('color') is not None}
    for link in list(robot.findall('link')):
        groups = {}
        for visual in link.findall('visual'):
            # Gazebo 11 only applies its link-level material extension to the
            # default (unnamed) URDF visuals. Keep collision/sensor names intact.
            visual.attrib.pop('name', None)
            mat = visual.find('material')
            rgba = None
            if mat is not None:
                color = mat.find('color')
                rgba = color.get('rgba') if color is not None else palette.get(mat.get('name'))
            groups.setdefault(rgba, []).append(visual)
        for index, (rgba, visuals) in enumerate(groups.items()):
            target = link
            if len(groups) > 1:
                # Do not start with the parent link's name: sdformat's lump
                # name heuristic would then skip remapping its material.
                name = 'finish_%02d__' % index + link.get('name')
                target = ET.SubElement(robot, 'link', name=name)
                mount = ET.SubElement(robot, 'joint', name=name + '_mount', type='fixed')
                ET.SubElement(mount, 'parent', link=link.get('name'))
                ET.SubElement(mount, 'child', link=name)
                for visual in visuals:
                    link.remove(visual)
                    target.append(visual)
            if rgba is not None:
                g = ET.SubElement(robot, 'gazebo', reference=target.get('name'))
                material = ET.SubElement(ET.SubElement(g, 'visual'), 'material')
                ET.SubElement(material, 'ambient').text = rgba
                ET.SubElement(material, 'diffuse').text = rgba
                ET.SubElement(material, 'specular').text = '.15 .15 .15 1'
                alpha = float(rgba.split()[3])
                if alpha < 1:
                    visual_extension = g.find('visual')
                    visual_extension.remove(material)
                    ET.SubElement(visual_extension, 'transparency').text = f'{1-alpha:g}'
                    ET.SubElement(visual_extension, 'cast_shadows').text = 'false'
                    material.find('specular').text = '0.5 0.5 0.5 1'
                    visual_extension.append(material)

    bridge = ET.SubElement(ET.SubElement(robot, 'gazebo'), 'plugin',
                           name='cabinet_state_and_lamps', filename='libcabinet_state_plugin.so')
    ET.SubElement(bridge, 'robotNamespace').text = '/total_cabinet'
    for joint in robot.findall('joint'):
        if joint.get('type') != 'fixed':
            ET.SubElement(bridge, 'joint').text = joint.get('name')
    for inst in items:
        if inst['kind'] == 'breaker':
            ET.SubElement(bridge, 'breaker').text = inst['name']
    indent(robot)
    output = ROOT / 'urdf/total_cabinet_glass.gazebo.urdf'
    ET.ElementTree(robot).write(output, encoding='utf-8', xml_declaration=True)
    (ROOT / 'config/gazebo_sources.json').write_text(json.dumps(sources, indent=2) + '\n')
    print('Generated %s (%d independent component plugins)' % (output, len(items)))


if __name__ == '__main__':
    build()
