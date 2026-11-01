#!/usr/bin/env python3
"""Build the passive full-cabinet/mobile-robot MuJoCo scene from its source models.

Run with the MuJoCo Python selected by tools/mujoco_python.sh. Source models are read only; all mesh
and texture paths in the generated scene remain relative to its location.
"""
from __future__ import annotations

import argparse
import ast
import copy
import json
import math
import os
from pathlib import Path
import xml.etree.ElementTree as ET

import mujoco_compat as mujoco
import numpy as np


ROOT = Path(__file__).resolve().parents[1]
ROBOT_ROOT = ROOT.parent / "rml63ii_eg2_4c2_gazebo"
DEFAULT_CABINET = ROOT / "mujoco" / "total_cabinet_glass_door.xml"
DEFAULT_ROBOT = ROBOT_ROOT / "model_mjcf" / "rml63ii_eg2_4c2_mobile_stage1.xml"
DEFAULT_OUTPUT = ROOT / "mujoco" / "robot_cabinet_scene_glass_door.xml"
ARM_JOINTS = tuple(f"joint_{index}" for index in range(1, 7))
ROBOT_ACTUATORS = tuple(f"act_{name}" for name in ARM_JOINTS) + ("act_gripper",)


def _numbers(values):
    return " ".join(f"{float(value):.15g}" for value in values)


def _section(root, tag):
    section = root.find(tag)
    return section if section is not None else ET.SubElement(root, tag)


def _relocate_resources(root, source_dir, output_dir):
    compiler = root.find("compiler")
    settings = compiler.attrib if compiler is not None else {}
    for item in root.iter():
        filename = item.get("file")
        if not filename:
            continue
        subdir = settings.get("assetdir", "")
        if item.tag == "mesh":
            subdir = settings.get("meshdir", subdir)
        elif item.tag == "texture":
            subdir = settings.get("texturedir", subdir)
        resource = (source_dir / subdir / filename).resolve()
        if not resource.is_file():
            raise FileNotFoundError(resource)
        item.set("file", os.path.relpath(resource, output_dir))
    if compiler is not None:
        for name in ("assetdir", "meshdir", "texturedir"):
            compiler.attrib.pop(name, None)


def _merge_section(root, source, tag):
    incoming = source.find(tag)
    if incoming is None:
        return
    target = _section(root, tag)
    names = {item.get("name"): item for item in target if item.get("name")}
    for item in incoming:
        name = item.get("name")
        if name in names:
            if ET.tostring(item) != ET.tostring(names[name]):
                raise ValueError(f"conflicting {tag} entry: {name}")
            continue
        target.append(copy.deepcopy(item))


def _merge_robot(root, robot):
    # Cabinet defaults are named, so they cannot leak into the robot. Scope
    # any future robot global defaults to its own top-level bodies as well.
    defaults = robot.find("default")
    if defaults is not None:
        scoped = copy.deepcopy(defaults)
        scoped.set("class", "robot_interaction_defaults")
        _section(root, "default").append(scoped)
        for body in robot.findall("worldbody/body"):
            if body.get("childclass") is None:
                body.set("childclass", "robot_interaction_defaults")

    for tag in ("asset", "contact", "equality", "tendon", "sensor", "custom", "actuator"):
        _merge_section(root, robot, tag)

    worldbody = _section(root, "worldbody")
    names = {(item.tag, item.get("name")) for item in worldbody if item.get("name")}
    has_floor = any(item.tag == "geom" and item.get("type") == "plane" for item in worldbody)
    for item in robot.findall("worldbody/*"):
        if item.tag == "geom" and item.get("type") == "plane" and has_floor:
            continue
        key = (item.tag, item.get("name"))
        if item.get("name") and key in names:
            if item.tag in ("camera", "light"):
                continue
            raise ValueError(f"duplicate world object: {key}")
        worldbody.append(copy.deepcopy(item))
        names.add(key)

    # Use the existing cabinet's 1 ms timestep and integrator, while retaining
    # the robot's contact solver settings where the cabinet has no override.
    option = _section(root, "option")
    if robot.find("option") is not None:
        for name, value in robot.find("option").attrib.items():
            if name not in option.attrib:
                option.set(name, value)

    # A fixed scene overview; RGB-D sensing belongs to the wrist RealSense.
    if root.find(".//camera[@name='global_default']") is None:
        ET.SubElement(worldbody, "camera", name="global_default", pos="3.20 -3.00 2.80",
                      xyaxes="0.732 0.682 0 -0.294 0.316 0.901",
                      fovy="53.7", resolution="640 480")


def _add_plate_constraints(root):
    parameters = json.loads((ROOT / "config/interaction.json").read_text())["yaban"]
    pitch = float(parameters["screw_pitch"])
    if not math.isfinite(pitch) or pitch <= 0:
        raise ValueError("pressure-plate screw_pitch must be finite and positive")
    joints = {joint.get("name") for joint in root.findall(".//body/joint")}
    equality = _section(root, "equality")
    suffix = "left_knob_lift_joint"
    for lift in sorted(name for name in joints if name and name.startswith("yaban_") and name.endswith(suffix)):
        prefix = lift[:-len(suffix)]
        knob, blade = prefix + "left_knob_joint", prefix + "blade_joint"
        if knob not in joints or blade not in joints:
            raise ValueError(f"incomplete pressure-plate joint set: {prefix}")
        for name, first, second, polynomial in (
            (prefix + "thread_coupling", lift, knob, _numbers([0, pitch / (2 * math.pi), 0, 0, 0])),
            (prefix + "blade_lock", blade, None, "0 0 0 0 0"),
        ):
            if equality.find(f"*[@name='{name}']") is not None:
                raise ValueError(f"duplicate pressure-plate constraint: {name}")
            attributes = dict(name=name, joint1=first, polycoef=polynomial,
                              active="true", solref=_numbers(parameters["constraint_solref"]),
                              solimp=_numbers(parameters["constraint_solimp"]))
            if second is not None:
                attributes["joint2"] = second
                attributes['solimp'] = _numbers(parameters['thread_solimp'])
            ET.SubElement(equality, "joint", attributes)


def _joint_width(model, index):
    joint_type = model.jnt_type[index]
    if joint_type == mujoco.mjtJoint.mjJNT_FREE:
        return 7
    if joint_type == mujoco.mjtJoint.mjJNT_BALL:
        return 4
    return 1


def _arm_start_position():
    # Read the existing literal without importing the environment or creating
    # a second MuJoCo simulation during asset generation.
    source = ROBOT_ROOT / "mujoco" / "environment.py"
    for node in ast.parse(source.read_text(encoding="utf-8")).body:
        if isinstance(node, ast.Assign) and any(
            isinstance(target, ast.Name) and target.id == "HOME_JOINTS"
            for target in node.targets
        ):
            return np.asarray(ast.literal_eval(node.value), dtype=float)
    raise ValueError(f"missing HOME_JOINTS in {source}")


def _compile(root, output_dir):
    absolute = copy.deepcopy(root)
    assets = mujoco.normalize_tree(absolute, output_dir)
    return mujoco.MjModel.from_xml_string(ET.tostring(absolute, encoding="unicode"), assets)


def _add_initial_keyframe(root, cabinet_path, output_dir):
    cabinet = mujoco.load_model(str(cabinet_path))
    initial_id = mujoco.mj_name2id(cabinet, mujoco.mjtObj.mjOBJ_KEY, "initial_open")
    source_qpos = cabinet.key_qpos[initial_id] if initial_id >= 0 else cabinet.qpos0
    model = _compile(root, output_dir)
    data = mujoco.MjData(model)
    for index in range(cabinet.njnt):
        name = mujoco.mj_id2name(cabinet, mujoco.mjtObj.mjOBJ_JOINT, index)
        if name is None:
            raise ValueError("cabinet joints must be named for reproducible initialization")
        target_id = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_JOINT, name)
        width = _joint_width(cabinet, index)
        source_adr, target_adr = cabinet.jnt_qposadr[index], model.jnt_qposadr[target_id]
        data.qpos[target_adr:target_adr + width] = source_qpos[source_adr:source_adr + width]

    for name, value in (("door_hinge_joint", math.pi / 2),
                        ("button_press_joint", 0),
                        ("handle_turn_joint", math.pi / 2),
                        ("handle_pop_joint", -math.pi / 3)):
        data.qpos[model.joint(name).qposadr[0]] = value
    for name, value in zip(ARM_JOINTS, _arm_start_position()):
        data.qpos[model.joint(name).qposadr[0]] = value
        data.ctrl[model.actuator(f"act_{name}").id] = value
    for index in range(1, 7):
        data.qpos[model.joint(f"gripper_joint{index}").qposadr[0]] = -.01 if index in (3, 4) else .01
    data.ctrl[model.actuator("act_gripper").id] = .01
    ET.SubElement(_section(root, "keyframe"), "key", name="initial_open",
                  qpos=_numbers(data.qpos), ctrl=_numbers(data.ctrl),
                  mpos=_numbers(data.mocap_pos.ravel()), mquat=_numbers(data.mocap_quat.ravel()))


def build(cabinet_xml=DEFAULT_CABINET, robot_xml=DEFAULT_ROBOT, output=DEFAULT_OUTPUT):
    cabinet_path, robot_path, output = (Path(value).resolve() for value in (cabinet_xml, robot_xml, output))
    if output in (cabinet_path, robot_path):
        raise ValueError("output must not overwrite either source model")
    root, robot = (ET.parse(path).getroot() for path in (cabinet_path, robot_path))
    if root.tag != "mujoco" or robot.tag != "mujoco":
        raise ValueError("both inputs must be MJCF models")
    if robot.find(".//body[@name='cabinet_base_link']") is not None:
        raise ValueError("robot input must be the standalone mobile model, without the old cabinet")
    _relocate_resources(root, cabinet_path.parent, output.parent)
    _relocate_resources(robot, robot_path.parent, output.parent)
    for section in list(root):
        if section.tag in ("actuator", "keyframe"):
            root.remove(section)
    root.set("model", "rml63ii_total_cabinet_contact_glass_door")
    if root.find("asset/material[@name='door_glass']") is None:
        raise ValueError('The source cabinet must have a transparent door')
    _merge_robot(root, robot)
    asset = root.find('asset')
    for name in ('door_panel_collision_hidden', 'door_glass'):
        material = asset.find(f"material[@name='{name}']")
        asset.remove(material)
        asset.append(material)
    for name in ("mobile_table", "cart_lock_anchor"):
        body = root.find(f".//body[@name='{name}']")
        if body is None:
            raise ValueError(f"robot model is missing {name}")
        body.set("pos", "1.40 -0.35 0")
        body.set("quat", "0 0 0 1")
    _add_plate_constraints(root)
    _add_initial_keyframe(root, cabinet_path, output.parent)
    model = _compile(root, output.parent)
    names = {mujoco.mj_id2name(model, mujoco.mjtObj.mjOBJ_ACTUATOR, index) for index in range(model.nu)}
    if names != set(ROBOT_ACTUATORS) or model.nu != 7:
        raise ValueError("combined scene must contain only the seven robot actuators")
    output.parent.mkdir(parents=True, exist_ok=True)
    ET.indent(root, space="  ")
    ET.ElementTree(root).write(output, encoding="utf-8", xml_declaration=True)
    mujoco.write_compatible_model(output)
    print(f"Built {output} (nq={model.nq}, nv={model.nv}, nu={model.nu}, ngeom={model.ngeom})")
    return output


def main(argv=None):
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--cabinet-xml", type=Path, default=DEFAULT_CABINET)
    parser.add_argument("--robot-xml", type=Path, default=DEFAULT_ROBOT)
    parser.add_argument("--output", type=Path, default=DEFAULT_OUTPUT)
    args = parser.parse_args(argv)
    build(args.cabinet_xml, args.robot_xml, args.output)


if __name__ == "__main__":
    main()
