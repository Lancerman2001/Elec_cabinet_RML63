#!/usr/bin/env python3
"""Rebuild ROS 2 plugin and control descriptions from the extracted models."""
import ast
import copy
from pathlib import Path
import xml.etree.ElementTree as ET

ROOT = Path(__file__).resolve().parents[1]
ROBOT = ROOT.parent / "rml63ii_eg2_4c2_gazebo"


def ros_namespace(plugin, namespace):
    ros = plugin.find("ros")
    if ros is None:
        ros = ET.SubElement(plugin, "ros")
    ns = ros.find("namespace")
    if ns is None:
        ns = ET.SubElement(ros, "namespace")
    ns.text = namespace
    # gazebo_ros2_control Humble can put its namespace into context-global
    # arguments. A node-local remap takes precedence, independent of spawn order.
    override = next((item for item in ros.findall("remapping")
                     if (item.text or "").startswith("__ns:=")), None)
    if override is None:
        override = ET.SubElement(ros, "remapping")
    override.text = "__ns:=" + namespace
    return ros


def write(root, path):
    path.parent.mkdir(parents=True, exist_ok=True)
    ET.indent(root, space="  ")
    ET.ElementTree(root).write(path, encoding="utf-8", xml_declaration=True)


def cabinet():
    root = ET.parse(ROOT / "urdf/total_cabinet_glass.gazebo.urdf").getroot()
    for plugin in root.iter("plugin"):
        namespace = plugin.findtext("robotNamespace", "/electrical_cabinet")
        ros = ros_namespace(plugin, namespace)
        if plugin.get("filename") == "libgazebo_ros_bumper.so":
            for child in list(plugin):
                if child.tag in ("robotNamespace", "frameName", "bumperTopicName"):
                    plugin.remove(child)
            ET.SubElement(ros, "remapping").text = "bumper_states:=button_contacts"
            ET.SubElement(plugin, "frame_name").text = "lock_button_link"
        if plugin.get("filename") == "libelectrical_cabinet_kinematic_plugin.so":
            old = plugin.find("initial_pose")
            if old is None:
                old = ET.SubElement(plugin, "initial_pose")
            old.text = "open"
    write(root, ROOT / "urdf/total_cabinet_glass.ros2.urdf")


def arm_start():
    module = ast.parse((ROBOT / "mujoco/environment.py").read_text())
    for node in module.body:
        if isinstance(node, ast.Assign) and any(getattr(t, "id", "") == "HOME_JOINTS" for t in node.targets):
            return list(ast.literal_eval(node.value))
    raise ValueError("Missing arm initial configuration")


def robot(variant):
    filename = "RML63II-6F-V-EG2-4C2" + ("" if variant == "arm" else "-table-" + variant) + ".gazebo.urdf"
    root = ET.parse(ROBOT / "model_gazebo" / filename).getroot()
    mimic = {}
    for element in list(root):
        if element.tag == "transmission":
            root.remove(element)
        if element.tag == "gazebo":
            for plugin in list(element.findall("plugin")):
                library = plugin.get("filename", "")
                if "mimic_joint" in library:
                    mimic[plugin.findtext("mimicJoint")] = (plugin.findtext("joint"), plugin.findtext("multiplier"))
                if "mimic_joint" in library or library == "libgazebo_ros_control.so":
                    element.remove(plugin)
    for plugin in root.iter("plugin"):
        if plugin.get("filename") == "libgazebo_ros_openni_kinect.so":
            plugin.attrib.update(name="d435_ros2_camera", filename="libgazebo_ros_camera.so")
            for child in list(plugin):
                plugin.remove(child)
            ros = ros_namespace(plugin, "/camera")
            for remap in ("camera/image_raw:=color/image_raw", "camera/camera_info:=color/camera_info",
                          "camera/depth/image_raw:=depth/image_rect_raw", "camera/depth/camera_info:=depth/camera_info",
                          "camera/points:=depth/points"):
                ET.SubElement(ros, "remapping").text = remap
            ET.SubElement(plugin, "camera_name").text = "camera"
            ET.SubElement(plugin, "frame_name").text = "arm/camera_depth_optical_frame"
            ET.SubElement(plugin, "min_depth").text = "0.1"
            ET.SubElement(plugin, "max_depth").text = "10.0"
    control = ET.SubElement(root, "ros2_control", name="GazeboSystem", type="system")
    ET.SubElement(ET.SubElement(control, "hardware"), "plugin").text = "gazebo_ros2_control/GazeboSystem"
    initial = dict(zip([f"joint_{i}" for i in range(1, 7)], arm_start()))
    initial["gripper_joint1"] = .01
    for joint in root.findall("joint"):
        name = joint.get("name")
        if joint.get("type") == "fixed":
            continue
        item = ET.SubElement(control, "joint", name=name)
        if name in initial or name in mimic:
            ET.SubElement(item, "command_interface", name="position")
        state = ET.SubElement(item, "state_interface", name="position")
        if name in initial:
            ET.SubElement(state, "param", name="initial_value").text = str(initial[name])
        if name in mimic:
            leader, multiplier = mimic[name]
            old = joint.find("mimic")
            if old is None:
                old = ET.SubElement(joint, "mimic")
            old.attrib.update(joint=leader, multiplier=multiplier, offset="0")
            ET.SubElement(item, "param", name="mimic").text = leader
            ET.SubElement(item, "param", name="multiplier").text = multiplier
            ET.SubElement(state, "param", name="initial_value").text = str(.01 * float(multiplier))
        ET.SubElement(item, "state_interface", name="velocity")
        ET.SubElement(item, "state_interface", name="effort")
    plugin = ET.SubElement(ET.SubElement(root, "gazebo"), "plugin",
                          name="gazebo_ros2_control", filename="libgazebo_ros2_control.so")
    ros_namespace(plugin, "/arm")
    ET.SubElement(plugin, "robot_param").text = "robot_description"
    ET.SubElement(plugin, "robot_param_node").text = "/arm/robot_state_publisher"
    ET.SubElement(plugin, "parameters").text = "@ROBOT_SHARE@/config/ros2_controllers.yaml"
    write(root, ROBOT / "urdf" / ("robot_" + variant + ".ros2.urdf"))


def main():
    cabinet()
    for variant in ("locked", "passive", "arm"):
        robot(variant)
    print("Generated cabinet and robot ROS 2 descriptions")


if __name__ == "__main__":
    main()
