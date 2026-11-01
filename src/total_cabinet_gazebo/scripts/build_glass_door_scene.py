#!/usr/bin/env python3
"""Transparent cabinet door geometry shared by the model generators.

Original door geoms remain invisible collision shapes. The frame and glass
are visual only, preserving contact geometry, named surfaces and inertia.
"""
from __future__ import annotations

import argparse
import copy
from pathlib import Path
import xml.etree.ElementTree as ET


ROOT = Path(__file__).resolve().parents[1]
DEFAULT_SOURCE = ROOT.parent / "electrical_cabinet_gazebo/mujoco/electrical_cabinet.xml"
DEFAULT_OUTPUT = DEFAULT_SOURCE
SIDE_MARGIN = 0.095
VERTICAL_MARGIN = 0.077
PANEL_NAMES = (
    "door_core", "door_skin_bottom", "door_skin_top",
    "door_skin_left", "door_skin_right",
)


def numbers(values):
    return " ".join(f"{value:.12g}" for value in values)


def bounds(geom):
    center = list(map(float, geom.get("pos").split()))
    half = list(map(float, geom.get("size").split()))
    return ([p - h for p, h in zip(center, half)],
            [p + h for p, h in zip(center, half)])


def frame_rectangles(low, high, window_low, window_high):
    """Subtract the centered window from one original Y/Z panel rectangle."""
    y0, z0 = low[1:]
    y1, z1 = high[1:]
    wy0, wz0 = window_low
    wy1, wz1 = window_high
    # Full-height left/right strips, then bottom/top inside the window width.
    middle_y0, middle_y1 = max(y0, wy0), min(y1, wy1)
    candidates = (
        (y0, min(y1, wy0), z0, z1),
        (max(y0, wy1), y1, z0, z1),
        (middle_y0, middle_y1, z0, min(z1, wz0)),
        (middle_y0, middle_y1, max(z0, wz1), z1),
    )
    return [rect for rect in candidates
            if rect[1] - rect[0] > 1e-12 and rect[3] - rect[2] > 1e-12]


def build(source=DEFAULT_SOURCE, output=DEFAULT_OUTPUT):
    source, output = Path(source).resolve(), Path(output).resolve()
    if source.parent != output.parent:
        raise ValueError("Keep the output beside its source to preserve relative asset paths")
    root = ET.parse(source).getroot()
    door = root.find(".//body[@name='door_link']")
    asset = root.find("asset")
    if door is None or asset is None:
        raise ValueError("Expected the complete robot/cabinet MJCF scene")
    if asset.find("material[@name='door_glass']") is not None:
        if source != output:
            ET.ElementTree(root).write(output, encoding="utf-8", xml_declaration=True)
        return output
    panels = [door.find(f"geom[@name='{name}']") for name in PANEL_NAMES]
    if any(geom is None or geom.get("type") != "box" for geom in panels):
        raise ValueError("Expected all five original box door panels")
    if any(any(key in geom.attrib for key in ("quat", "euler", "axisangle", "xyaxes", "zaxis"))
           for geom in panels):
        raise ValueError("Expected door panels aligned with door_link axes")

    core_low, core_high = bounds(panels[0])
    window_low = (core_low[1] + SIDE_MARGIN, core_low[2] + VERTICAL_MARGIN)
    window_high = (core_high[1] - SIDE_MARGIN, core_high[2] - VERTICAL_MARGIN)
    if any(hi <= lo for lo, hi in zip(window_low, window_high)):
        raise ValueError("Door dimensions are too small for the requested glass margins")
    ET.SubElement(asset, "material", name="door_panel_collision_hidden", rgba="0 0 0 0")
    ET.SubElement(asset, "material", name="door_glass", rgba="0.82 0.93 0.98 0.18",
                  specular="0.5", shininess="0.8", reflectance="0")

    # Keep the existing collision shapes (including the handle recess) and
    # their names. Zero alpha hides them even when collision group 3 is shown.
    # Explicit door inertia is left untouched; visuals add no mass/contact.
    visuals = []
    for panel in panels:
        low, high = bounds(panel)
        for index, (y0, y1, z0, z1) in enumerate(
                frame_rectangles(low, high, window_low, window_high)):
            visual = copy.deepcopy(panel)
            visual.set("name", f"{panel.get('name')}_frame_visual_{index}")
            visual.set("pos", numbers(((low[0] + high[0]) / 2, (y0 + y1) / 2, (z0 + z1) / 2)))
            visual.set("size", numbers(((high[0] - low[0]) / 2, (y1 - y0) / 2, (z1 - z0) / 2)))
            visual.attrib.update(contype="0", conaffinity="0", group="1", density="0")
            visuals.append(visual)
        panel.set("material", "door_panel_collision_hidden")
        panel.set("rgba", "0 0 0 0")
        panel.set("group", "3")

    # One glass visual spans the original 20 mm panel envelope. This keeps
    # the external door dimensions without layering two translucent panels.
    x0 = min(bounds(panel)[0][0] for panel in panels)
    x1 = max(bounds(panel)[1][0] for panel in panels)
    wy0, wz0 = window_low
    wy1, wz1 = window_high
    glass = ET.Element("geom", name="door_glass_panel", type="box",
                       pos=numbers(((x0 + x1) / 2, (wy0 + wy1) / 2, (wz0 + wz1) / 2)),
                       size=numbers(((x1 - x0) / 2, (wy1 - wy0) / 2, (wz1 - wz0) / 2)),
                       material="door_glass", contype="0", conaffinity="0", group="1", density="0")
    insert_at = list(door).index(panels[-1]) + 1
    for visual in visuals + [glass]:
        door.insert(insert_at, visual)
        insert_at += 1
    root.set("model", root.get("model", "robot_cabinet") + "_glass_door")
    root.insert(0, ET.Comment(
        " Generated by scripts/build_glass_door_scene.py. Glass margins: "
        "left/right 95 mm, top/bottom 77 mm. Original collision geometry and inertia retained. "))
    ET.indent(root, space="  ")
    ET.ElementTree(root).write(output, encoding="utf-8", xml_declaration=True)
    print(f"Built {output}")
    print(f"Glass: {(wy1 - wy0) * 1000:g} x {(wz1 - wz0) * 1000:g} mm; "
          f"left/right margin {SIDE_MARGIN * 1000:g} mm; "
          f"top/bottom margin {VERTICAL_MARGIN * 1000:g} mm")
    return output


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--source", type=Path, default=DEFAULT_SOURCE)
    parser.add_argument("--output", type=Path, default=DEFAULT_OUTPUT)
    args = parser.parse_args()
    build(args.source, args.output)


if __name__ == "__main__":
    main()
