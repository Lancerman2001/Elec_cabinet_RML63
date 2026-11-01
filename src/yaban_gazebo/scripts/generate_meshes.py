#!/usr/bin/env python3
"""Deterministic metre-scale STL geometry. No CAD or pip dependency required."""
import math
from pathlib import Path

ROOT = Path(__file__).resolve().parents[1]


def write_stl(name, triangles):
    path = ROOT / 'meshes' / (name + '.stl')
    path.parent.mkdir(parents=True, exist_ok=True)
    with path.open('w') as out:
        out.write('solid ' + name + '\n')
        for a, b, c in triangles:
            u, v = [b[i]-a[i] for i in range(3)], [c[i]-a[i] for i in range(3)]
            n = [u[1]*v[2]-u[2]*v[1], u[2]*v[0]-u[0]*v[2], u[0]*v[1]-u[1]*v[0]]
            length = math.sqrt(sum(t*t for t in n))
            if length < 1e-14:
                continue
            out.write('facet normal ' + ' '.join(str(t/length) for t in n) + '\nouter loop\n')
            for p in (a, b, c):
                out.write('vertex ' + ' '.join('%.10g' % (t*0.001) for t in p) + '\n')
            out.write('endloop\nendfacet\n')
        out.write('endsolid ' + name + '\n')
    write_obj(name, triangles)


def write_obj(name, triangles):
    """Write the same metre-scale triangles as OBJ for MuJoCo.

    MuJoCo accepts binary STL but deliberately rejects the human-readable STL
    files above.  Keeping this conversion alongside the source geometry makes
    the Gazebo and MJCF assets deterministic and dimensionally identical.
    """
    path = ROOT / 'mujoco' / 'assets' / (name + '.obj')
    path.parent.mkdir(parents=True, exist_ok=True)
    with path.open('w') as out:
        out.write('# Generated from yaban_gazebo/scripts/generate_meshes.py\n')
        index = 1
        for triangle in triangles:
            a, b, c = triangle
            u, v = [b[i]-a[i] for i in range(3)], [c[i]-a[i] for i in range(3)]
            normal = [u[1]*v[2]-u[2]*v[1], u[2]*v[0]-u[0]*v[2], u[0]*v[1]-u[1]*v[0]]
            if math.sqrt(sum(component*component for component in normal)) < 1e-14:
                continue
            for point in triangle:
                out.write('v ' + ' '.join('%.10g' % (coordinate*0.001) for coordinate in point) + '\n')
            out.write('f {} {} {}\n'.format(index, index + 1, index + 2))
            index += 3


def slab(x0, x1, lo0, lo1, hi0, hi1, thickness):
    """Closed trapezoidal prism, in mm; degenerate end faces are discarded."""
    p = [(x0, lo0), (x1, lo1), (x1, hi1), (x0, hi0)]
    v = [(x, y, z) for z in (-thickness/2, thickness/2) for x, y in p]
    ids = [(0,2,1),(0,3,2),(4,5,6),(4,6,7)]
    for i in range(4):
        j=(i+1)%4
        ids += [(i,j,j+4),(i,j+4,i+4)]
    return [tuple(v[i] for i in f) for f in ids]


def cut_plate(name, xmin, xmax, width, thickness, hole_x, hole_r, notch=None, round_end=False):
    # Each strip is a closed convex cell: holes remain genuinely empty.
    cuts = [xmin, xmax, hole_x-hole_r, hole_x+hole_r]
    if notch:
        cuts += [notch[0]-notch[1], notch[0]+notch[1]]
    if round_end:
        cuts += [xmax-width/2]
    cuts = sorted(set(cuts))
    tris = []
    for start, end in zip(cuts, cuts[1:]):
        if start < xmin or end > xmax:
            continue
        n=max(1, math.ceil((end-start)/0.12))
        for k in range(n):
            a, b = start+(end-start)*k/n, start+(end-start)*(k+1)/n
            mid=(a+b)/2
            def edge(x):
                if round_end and x > xmax-width/2:
                    return math.sqrt(max(0, (width/2)**2-(x-(xmax-width/2))**2))
                return width/2
            ea, eb = edge(a), edge(b)
            def circle(x, cx, radius):
                return math.sqrt(max(0, radius**2-(x-cx)**2))
            if abs(mid-hole_x) < hole_r:
                ha,hb=circle(a,hole_x,hole_r),circle(b,hole_x,hole_r)
                tris += slab(a,b,-ea,-eb,-ha,-hb,thickness)
                tris += slab(a,b,ha,hb,ea,eb,thickness)
            elif notch and abs(mid-notch[0]) < notch[1]:
                ha,hb=circle(a,*notch),circle(b,*notch)
                tris += slab(a,b,-ea,-eb,-ha,-hb,thickness)
            else:
                tris += slab(a,b,-ea,-eb,ea,eb,thickness)
    write_stl(name, tris)


def ring(name, outer, inner, height, segments=64):
    tris=[]
    for i in range(segments):
        angles=[i*2*math.pi/segments,(i+1)*2*math.pi/segments]
        v=[(r*math.cos(a),r*math.sin(a),z) for z in (-height/2,height/2)
           for r in (inner,outer) for a in angles]
        for face in [(0,1,3),(0,3,2),(4,6,7),(4,7,5),
                     (2,3,7),(2,7,6),(0,4,5),(0,5,1)]:
            tris.append(tuple(v[j] for j in face))
    write_stl(name,tris)


if __name__ == '__main__':
    # Blade coordinates: origin is RIGHT terminal; closed blade extends toward -X.
    cut_plate('blade', -38.25, 6.75, 10, 1, 0, 2.3, notch=(-31.5,2.5), round_end=True)
    cut_plate('base_bridge', -15.75, 15.75, 12.5, 3, 0, 2.1)
    ring('washer',5,2.2,1)
    ring('nut_m4',4.04145,2,2.8,segments=6)
    print('Generated blade, perforated base bridge, washers and M4 nuts (metres).')
