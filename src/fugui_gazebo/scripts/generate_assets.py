#!/usr/bin/python3
"""Generate metre-scale CAD meshes and the Chinese nameplate reproducibly."""
import math
from pathlib import Path
import struct
import subprocess
from PIL import Image, ImageDraw, ImageFont

ROOT = Path(__file__).resolve().parents[1]


def stl(name, triangles):
    import numpy as np
    with (ROOT / 'meshes' / (name + '.stl')).open('wb') as out:
        out.write(b'fugui_gazebo; units=metres'.ljust(80, b'\0'))
        out.write(struct.pack('<I', len(triangles)))
        for triangle in triangles:
            points = np.array(triangle)
            normal = np.cross(points[1] - points[0], points[2] - points[0])
            normal /= np.linalg.norm(normal)
            out.write(struct.pack('<12fH', *normal, *points.flatten(), 0))


def rounded_plate(name, width, height, depth, radius):
    # Extruded rounded rectangle: X is thickness, Y width, Z height.
    contour = []
    for cy, cz, start in [(width/2-radius, height/2-radius, 0),
                          (-width/2+radius, height/2-radius, 90),
                          (-width/2+radius, -height/2+radius, 180),
                          (width/2-radius, -height/2+radius, 270)]:
        for i in range(17):
            a = math.radians(start + i*90/16)
            contour.append((cy+radius*math.cos(a), cz+radius*math.sin(a)))
    triangles = []
    for i, (y, z) in enumerate(contour):
        yy, zz = contour[(i+1) % len(contour)]
        a, b = (-depth/2, y, z), (-depth/2, yy, zz)
        c, d = (depth/2, y, z), (depth/2, yy, zz)
        triangles.extend([[(depth/2, 0, 0), c, d], [(-depth/2, 0, 0), b, a],
                          [a, b, c], [b, d, c]])
    stl(name, triangles)


def lathe(name, profile, n=128):
    """Closed X/radius profile, ordered counterclockwise in the X/r plane."""
    triangles = []
    for j, (x, r) in enumerate(profile):
        xx, rr = profile[(j+1) % len(profile)]
        for i in range(n):
            a, b = i*2*math.pi/n, (i+1)*2*math.pi/n
            p = (x, r*math.cos(a), r*math.sin(a))
            q = (xx, rr*math.cos(a), rr*math.sin(a))
            s = (x, r*math.cos(b), r*math.sin(b))
            t = (xx, rr*math.cos(b), rr*math.sin(b))
            triangles.extend([[p, q, s], [s, q, t]])
    stl(name, triangles)


def label():
    name = '保护信号复归按钮'
    font_path = subprocess.check_output(
        ['fc-match', '-f', '%{file}', 'Noto Serif CJK SC:lang=zh'], text=True).strip()
    font = ImageFont.truetype(font_path, 106)
    im = Image.new('RGB', (1400, 360), '#f2f1e9')
    draw = ImageDraw.Draw(im)
    width, height = draw.textsize(name, font=font)
    draw.text(((1400-width)/2, (360-height)/2), name, font=font, fill='#202521')
    im.save(ROOT / 'materials/textures/nameplate.png')
    # Front face points +X; U increases toward screen-right (+Y).
    (ROOT / 'meshes/nameplate.dae').write_text('''<?xml version="1.0"?>
<COLLADA xmlns="http://www.collada.org/2005/11/COLLADASchema" version="1.4.1">
<asset><created>2026-09-11T00:00:00Z</created><modified>2026-09-11T00:00:00Z</modified><unit name="meter" meter="1"/><up_axis>Z_UP</up_axis></asset>
<library_images><image id="label"><init_from>../materials/textures/nameplate.png</init_from></image></library_images>
<library_effects><effect id="effect"><profile_COMMON><newparam sid="surface"><surface type="2D"><init_from>label</init_from></surface></newparam><newparam sid="sampler"><sampler2D><source>surface</source></sampler2D></newparam><technique sid="common"><phong><ambient><color>0.7 0.7 0.7 1</color></ambient><diffuse><texture texture="sampler" texcoord="UVMap"/></diffuse></phong></technique></profile_COMMON></effect></library_effects>
<library_materials><material id="material"><instance_effect url="#effect"/></material></library_materials>
<library_geometries><geometry id="geometry"><mesh>
<source id="vertices"><float_array id="v-array" count="12">0 0.023 -0.006 0 -0.023 -0.006 0 -0.023 0.006 0 0.023 0.006</float_array><technique_common><accessor source="#v-array" count="4" stride="3"><param name="X" type="float"/><param name="Y" type="float"/><param name="Z" type="float"/></accessor></technique_common></source>
<source id="uv"><float_array id="uv-array" count="8">1 0 0 0 0 1 1 1</float_array><technique_common><accessor source="#uv-array" count="4" stride="2"><param name="S" type="float"/><param name="T" type="float"/></accessor></technique_common></source>
<source id="normals"><float_array id="n-array" count="3">1 0 0</float_array><technique_common><accessor source="#n-array" count="1" stride="3"><param name="X" type="float"/><param name="Y" type="float"/><param name="Z" type="float"/></accessor></technique_common></source>
<vertices id="verts"><input semantic="POSITION" source="#vertices"/></vertices>
<triangles count="2" material="labelmat"><input semantic="VERTEX" source="#verts" offset="0"/><input semantic="TEXCOORD" source="#uv" offset="1" set="0"/><input semantic="NORMAL" source="#normals" offset="2"/><p>0 0 0 2 2 0 1 1 0 0 0 0 3 3 0 2 2 0</p></triangles>
</mesh></geometry></library_geometries>
<library_visual_scenes><visual_scene id="Scene"><node id="plate"><instance_geometry url="#geometry"><bind_material><technique_common><instance_material symbol="labelmat" target="#material"><bind_vertex_input semantic="UVMap" input_semantic="TEXCOORD" input_set="0"/></instance_material></technique_common></bind_material></instance_geometry></node></visual_scene></library_visual_scenes><scene><instance_visual_scene url="#Scene"/></scene></COLLADA>
''', encoding='utf-8')


def main():
    for directory in ('meshes', 'materials/textures'):
        (ROOT / directory).mkdir(parents=True, exist_ok=True)
    rounded_plate('frame', .070, .075, .007, .006)
    rounded_plate('insert', .047, .047, .0003, .002)
    rounded_plate('label_recess', .048, .014, .0003, .0015)
    lathe('rear_sleeve', [(0, .012), (.008, .012), (.008, .014), (0, .014)])
    lathe('guard_ring', [(0, .012), (.007, .012), (.007, .0133),
                         (.0063, .014), (.0004, .014), (0, .0136)])
    label()
    print('Generated 70 x 75 x 7 mm frame, hollow 28/24 mm rings and Chinese nameplate.')


if __name__ == '__main__':
    main()
