#!/usr/bin/env python3
"""Maintain transparent door visuals without changing URDF physics."""
import argparse
import copy
import importlib.util
from pathlib import Path
import xml.etree.ElementTree as ET

_layout_spec = importlib.util.spec_from_file_location(
    "glass_door_layout", Path(__file__).with_name("build_glass_door_scene.py"))
_layout = importlib.util.module_from_spec(_layout_spec)
_layout_spec.loader.exec_module(_layout)
SIDE_MARGIN = _layout.SIDE_MARGIN
VERTICAL_MARGIN = _layout.VERTICAL_MARGIN


def numbers(values):
    return " ".join(f"{value:.12g}" for value in values)


def bounds(visual):
    origin = visual.find("origin")
    box = visual.find("geometry/box")
    if origin is None or box is None or any(float(x) for x in origin.get("rpy", "0 0 0").split()):
        raise ValueError("Expected the existing axis-aligned box door panels")
    center = list(map(float, origin.get("xyz").split()))
    half = [float(value) / 2 for value in box.get("size").split()]
    return ([p - h for p, h in zip(center, half)],
            [p + h for p, h in zip(center, half)])


def apply_glass_door(root):
    """Change only door visuals; preserve collisions, mass and moving joints."""
    if root.find("link[@name='door_glass_visual_link']") is not None:
        return root
    door = root.find("link[@name='door_link']")
    layers = [root.find("link[@name='finish_00__door_link']"),
              root.find("link[@name='finish_01__door_link']")]
    if door is None:
        raise ValueError("Expected a cabinet URDF with door_link")
    if any(link is None for link in layers):
        layers = [door]
    panels = [visual for link in layers for visual in link.findall("visual")]
    if len(panels) != 5:
        raise ValueError("Expected the five original core and skin panel visuals")
    core_low, core_high = bounds(panels[0])
    window_low = (core_low[1] + SIDE_MARGIN, core_low[2] + VERTICAL_MARGIN)
    window_high = (core_high[1] - SIDE_MARGIN, core_high[2] - VERTICAL_MARGIN)
    if any(high <= low for low, high in zip(window_low, window_high)):
        raise ValueError("Door panel is too small for the glass margins")

    # Finish links already carry the Gazebo gray materials. Retain their
    # transforms and material bindings, replacing each solid visual by the
    # parts outside the centered window. Handle/cavity finish layers stay as is.
    for link in layers:
        for panel in list(link.findall("visual")):
            low, high = bounds(panel)
            link.remove(panel)
            for index, (y0, y1, z0, z1) in enumerate(
                    _layout.frame_rectangles(low, high, window_low, window_high)):
                visual = copy.deepcopy(panel)
                # Gazebo Classic maps per-link material extensions to its
                # generated link-prefixed visual names after fixed-joint lumping.
                visual.attrib.pop("name", None)
                visual.find("origin").set("xyz", numbers(((low[0] + high[0]) / 2,
                                                          (y0 + y1) / 2, (z0 + z1) / 2)))
                visual.find("geometry/box").set("size", numbers((high[0] - low[0], y1 - y0, z1 - z0)))
                link.append(visual)

    x0 = min(bounds(panel)[0][0] for panel in panels)
    x1 = max(bounds(panel)[1][0] for panel in panels)
    wy0, wz0 = window_low
    wy1, wz1 = window_high
    glass_link = ET.SubElement(root, "link", name="door_glass_visual_link")
    glass = ET.SubElement(glass_link, "visual")
    ET.SubElement(glass, "origin", xyz=numbers(((x0 + x1) / 2, (wy0 + wy1) / 2, (wz0 + wz1) / 2)), rpy="0 0 0")
    ET.SubElement(ET.SubElement(glass, "geometry"), "box", size=numbers((x1 - x0, wy1 - wy0, wz1 - wz0)))
    ET.SubElement(ET.SubElement(glass, "material", name="door_glass"), "color", rgba="0.82 0.93 0.98 0.18")
    joint = ET.SubElement(root, "joint", name="door_glass_visual_mount", type="fixed")
    ET.SubElement(joint, "parent", link="door_link")
    ET.SubElement(joint, "child", link="door_glass_visual_link")
    extension = ET.SubElement(root, "gazebo", reference="door_glass_visual_link")
    sdf_visual = ET.SubElement(extension, "visual")
    ET.SubElement(sdf_visual, "transparency").text = "0.82"
    ET.SubElement(sdf_visual, "cast_shadows").text = "false"
    material = ET.SubElement(sdf_visual, "material")
    ET.SubElement(material, "ambient").text = "0.82 0.93 0.98 0.18"
    ET.SubElement(material, "diffuse").text = "0.82 0.93 0.98 0.18"
    ET.SubElement(material, "specular").text = "0.5 0.5 0.5 1"
    return root


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    package = Path(__file__).resolve().parents[1]
    parser.add_argument("--source", type=Path, default=package.parent / "electrical_cabinet_gazebo/urdf/electrical_cabinet.urdf")
    parser.add_argument("--output", type=Path)
    args = parser.parse_args()
    args.output = args.output or args.source
    root = apply_glass_door(ET.parse(args.source).getroot())
    ET.indent(root, space="  ")
    args.output.parent.mkdir(parents=True, exist_ok=True)
    ET.ElementTree(root).write(args.output, encoding="utf-8", xml_declaration=True)
    print("Built", args.output)


if __name__ == "__main__":
    main()
