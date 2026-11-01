#!/usr/bin/env python3
"""生成 RML63II 标准 URDF 的 Gazebo 专用派生模型。

标准 URDF 永不被修改。本脚本将其运动学和资源复制到 ROS/Gazebo 模型中，
然后添加 ros_control、夹爪 mimic 插件和真实 Gazebo RGB-D 传感器。脚本会
生成仅机械臂版本，以及由可复用移动工作台组件组成的锁定/被动组合版本。
"""

from __future__ import print_function

import argparse
import copy
import hashlib
import os
import sys
import xml.etree.ElementTree as ET
from pathlib import Path


# ===== 模块级常量与辅助定义 =====
ARM_JOINTS = ("joint_1", "joint_2", "joint_3", "joint_4", "joint_5", "joint_6")
SOURCE_GRIPPER_JOINT = "4C2_Joint1"
SIM_GRIPPER_JOINT = "gripper_joint1"
CAMERA_ROLL_JOINT = "camera_rojoint"
CAMERA_SENSOR_LINK = "camera_sensor_link"
# CAD 的 camera_link 网格是薄的镜头/前面板。其局部 x 范围约为
# [-0.072695, 0.007695] m，因此光学中心位于 -0.0325 m；朝外表面法线为 +Z。
# Gazebo 相机传感器沿 +X 方向观察。在此处应用物理相机滚转，以完成 Gazebo
# 姿态测试；RViz 配置显示原始图像，因此不会重复应用该变换。
CAMERA_LENS_CENTER_X = -0.0325
CAMERA_SENSOR_RPY = "1.5708 -1.5708 0"
POSITION_INTERFACE = "hardware_interface/PositionJointInterface"
MIMIC_PLUGIN = "libroboticsgroup_upatras_gazebo_mimic_joint_plugin.so"

# Gazebo 专用工具法兰插入 link_6 与夹爪之间。link_6 的圆形安装端直径约
# 57 mm，因此沿 link_6 局部 +Z 方向使用 80 mm 圆柱形间隔件。由于未提供
# 法兰材料，质量采用明确的近似值。
FLANGE_LINK = "gripper_flange"
FLANGE_JOINT = "gripper_flange_joint"
FLANGE_LENGTH = 0.08
FLANGE_RADIUS = 0.0285
FLANGE_MASS = 0.30

# Gazebo 专用安装修正。EG2-4C2 CAD 组件相对于期望的 Gazebo 垂直轴展示姿态
# 偏航了 90 度。将修正作用于唯一的固定安装关节，使整个夹爪作为刚性组件
# 一起旋转；机械臂关节、手指开合轴、相机、标准 URDF 和 MJCF 源文件保持不变。
GRIPPER_MOUNT_YAW = 1.5707963267948966

# 源 URDF 的 effort 字段是制造商给出的执行器限制。没有重力前馈的 Gazebo
# 位置控制器可能仅为保持 CAD 姿态就耗尽全部力矩预算，尤其是 J4-J6。饱和
# 会使机械臂在两个较大的力矩之间交替，看起来像具有弹性。MJCF 保留源限制，
# 但为 Gazebo 专用位置接口提供保守的保持预算；这不代表真实机器人的连续
# 力矩额定值。
GAZEBO_ARM_EFFORT_LIMITS = {
    "joint_1": 120.0,
    "joint_2": 120.0,
    "joint_3": 60.0,
    "joint_4": 60.0,
    "joint_5": 30.0,
    "joint_6": 20.0,
}

GAZEBO_ARM_JOINT_DAMPING = {
    "joint_1": 2.0,
    "joint_2": 4.0,
    "joint_3": 2.0,
    "joint_4": 2.0,
    "joint_5": 0.75,
    "joint_6": 0.5,
}

GAZEBO_NO_GRAVITY_LINKS = {
    "camera_rolink",
    "camera_link",
    CAMERA_SENSOR_LINK,
    FLANGE_LINK,
    "4C2_baselink",
    "4C2_Link1",
    "4C2_Link2",
    "4C2_Link3",
    "4C2_Link4",
    "4C2_Link5",
    "4C2_Link6",
}

GRIPPER_RENAMES = {
    "4C2_Joint1": "gripper_joint1",
    "4C2_Joint2": "gripper_joint2",
    "4C2_Joint3": "gripper_joint3",
    "4C2_Joint4": "gripper_joint4",
    "4C2_Joint5": "gripper_joint5",
    "4C2_Joint6": "gripper_joint6",
}

# 源夹爪是非常轻的 CAD 组件，没有关节动力学参数。为 Gazebo 专用验证模型
# 添加少量物理阻尼，避免 mimic 关节在硬限位处抖动。这些值有意不复制到
# 标准 URDF 或 MJCF 模型。
GAZEBO_GRIPPER_JOINT_DAMPING = {
    "gripper_joint1": 0.002,
    "gripper_joint2": 0.002,
    "gripper_joint3": 0.002,
    "gripper_joint4": 0.002,
    "gripper_joint5": 0.002,
    "gripper_joint6": 0.002,
}

MIMIC_JOINTS = (
    ("4C2_Joint5", 1.0),
    ("4C2_Joint2", 1.0),
    ("4C2_Joint3", -1.0),
    ("4C2_Joint4", -1.0),
    ("4C2_Joint6", 1.0),
)

PACKAGE_NAME = "rml63ii_eg2_4c2_gazebo"
TABLE_ROOT_LINK = "table_base_link"
ARM_MOUNT_JOINT = "rml63ii_table_mount_joint"
ARM_MOUNT_XYZ = "-0.140 0 0.700"
TABLE_JOINTS = tuple(
    "{}_{}_joint".format(corner, joint_type)
    for corner in ("front_left", "front_right", "rear_left", "rear_right")
    for joint_type in ("caster_swivel", "wheel")
)


# 函数功能：向 XML 节点添加文本子元素。
def _add_text(parent, tag, value):
    child = ET.SubElement(parent, tag)
    child.text = str(value)
    return child


# 函数功能：为 XML 树添加统一缩进。
def _indent(element, level=0):
    indent = "\n" + level * "  "
    if len(element):
        if not element.text or not element.text.strip():
            element.text = indent + "  "
        for child in element:
            _indent(child, level + 1)
        if not element[-1].tail or not element[-1].tail.strip():
            element[-1].tail = indent
    if level and (not element.tail or not element.tail.strip()):
        element.tail = indent


# 函数功能：将模型网格路径改为软件包路径。
def _replace_mesh_paths(root, package_root):
    arm_meshes = package_root / "assets" / "arm_meshes"
    gripper_meshes = package_root / "assets" / "gripper_meshes"
    roots = {"RML63II-6F-V": arm_meshes, "EG2-4C2": gripper_meshes}
    for mesh in root.iter("mesh"):
        filename = mesh.get("filename", "")
        if not filename.startswith("package://"):
            continue
        package_and_path = filename[len("package://") :]
        package_name, separator, relative_path = package_and_path.partition("/")
        if not separator or package_name not in roots:
            raise ValueError("unsupported mesh URI: {}".format(filename))
        source_mesh = roots[package_name] / Path(relative_path).name
        if not source_mesh.is_file():
            raise ValueError("mesh does not exist: {}".format(source_mesh))
        if package_name == "RML63II-6F-V":
            rel = "assets/arm_meshes/{}".format(Path(relative_path).name)
        else:
            rel = "assets/gripper_meshes/{}".format(Path(relative_path).name)
        mesh.set("filename", "package://{}/{}".format(PACKAGE_NAME, rel))


# 函数功能：修正相机滚转关节姿态。
def _fix_camera_roll_joint(root):
    for joint in root.findall("joint"):
        if joint.get("name") != CAMERA_ROLL_JOINT:
            continue
        joint.set("type", "fixed")
        for tag in ("axis", "limit", "dynamics", "mimic", "calibration", "safety_controller"):
            for child in list(joint.findall(tag)):
                joint.remove(child)
        return
    raise ValueError("missing {}".format(CAMERA_ROLL_JOINT))


# 函数功能：将夹爪安装关节旋转到目标方向。
def _rotate_gripper_mount(root):
    """在 Gazebo 中将完整夹爪组件绕 Z 轴偏航 +90 度。

    该修正有意施加在机械臂到夹爪的固定关节上，不改变任何手指关节轴或
    mimic 符号。安装关节的源姿态角为零，因此生成的固定变换明确表示
    绕其 Z 轴旋转。
    """
    for joint in root.findall("joint"):
        if joint.get("name") != "gripper_mount_joint":
            continue
        origin = joint.find("origin")
        if origin is None:
            origin = ET.SubElement(joint, "origin", {"xyz": "0 0 0"})
        origin.set("rpy", "0 0 {}".format(GRIPPER_MOUNT_YAW))
        return
    raise ValueError("missing gripper_mount_joint")


# 函数功能：在腕部与夹爪之间添加固定法兰。
def _add_gripper_flange(root):
    """在夹爪安装关节之前插入固定圆柱形间隔法兰。

    标准 URDF 保持不变。现有安装关节的父链接从 link_6 改为新法兰，关节
    原点移动到法兰远端。后续 Gazebo 安装旋转步骤只改变安装关节的偏航角，
    因此法兰仍与腕部同轴。
    """
    link_names = {link.get("name") for link in root.findall("link")}
    if FLANGE_LINK in link_names or any(
        joint.get("name") == FLANGE_JOINT for joint in root.findall("joint")
    ):
        raise ValueError("flange already exists in input URDF")

    mount = next(
        (joint for joint in root.findall("joint") if joint.get("name") == "gripper_mount_joint"),
        None,
    )
    if mount is None:
        raise ValueError("missing gripper_mount_joint")
    parent = mount.find("parent")
    child = mount.find("child")
    if parent is None or child is None or parent.get("link") != "link_6":
        raise ValueError("gripper_mount_joint must be attached to link_6")

    flange = ET.Element("link", {"name": FLANGE_LINK})
    inertial = ET.SubElement(flange, "inertial")
    ET.SubElement(inertial, "origin", {"xyz": "0 0 0.04", "rpy": "0 0 0"})
    ET.SubElement(inertial, "mass", {"value": str(FLANGE_MASS)})
    # 计算圆柱体绕中心的惯性，圆柱轴线沿 Z 方向。
    iz = 0.5 * FLANGE_MASS * FLANGE_RADIUS * FLANGE_RADIUS
    ixy = FLANGE_MASS * (
        3.0 * FLANGE_RADIUS * FLANGE_RADIUS + FLANGE_LENGTH * FLANGE_LENGTH
    ) / 12.0
    ET.SubElement(
        inertial,
        "inertia",
        {
            "ixx": str(ixy),
            "ixy": "0",
            "ixz": "0",
            "iyy": str(ixy),
            "iyz": "0",
            "izz": str(iz),
        },
    )
    for tag in ("visual", "collision"):
        geometry_parent = ET.SubElement(flange, tag)
        ET.SubElement(
            geometry_parent,
            "origin",
            {"xyz": "0 0 0.04", "rpy": "0 0 0"},
        )
        geometry = ET.SubElement(geometry_parent, "geometry")
        ET.SubElement(
            geometry,
            "cylinder",
            {"radius": str(FLANGE_RADIUS), "length": str(FLANGE_LENGTH)},
        )
        if tag == "visual":
            material = ET.SubElement(geometry_parent, "material", {"name": "flange_gray"})
            ET.SubElement(material, "color", {"rgba": "0.32 0.34 0.38 1"})

    flange_joint = ET.Element("joint", {"name": FLANGE_JOINT, "type": "fixed"})
    ET.SubElement(flange_joint, "origin", {"xyz": "0 0 0", "rpy": "0 0 0"})
    ET.SubElement(flange_joint, "parent", {"link": "link_6"})
    ET.SubElement(flange_joint, "child", {"link": FLANGE_LINK})

    parent.set("link", FLANGE_LINK)
    origin = mount.find("origin")
    if origin is None:
        origin = ET.SubElement(mount, "origin")
    origin.set("xyz", "0 0 {}".format(FLANGE_LENGTH))

    root.append(flange)
    root.append(flange_joint)


# 函数功能：统一夹爪关节命名。
def _rename_gripper_joints(root):
    for joint in root.findall("joint"):
        old_name = joint.get("name")
        if old_name in GRIPPER_RENAMES:
            joint.set("name", GRIPPER_RENAMES[old_name])
        mimic = joint.find("mimic")
        if mimic is not None and mimic.get("joint") in GRIPPER_RENAMES:
            mimic.set("joint", GRIPPER_RENAMES[mimic.get("joint")])
    for transmission in root.findall("transmission"):
        joint = transmission.find("joint")
        if joint is not None and joint.get("name") in GRIPPER_RENAMES:
            joint.set("name", GRIPPER_RENAMES[joint.get("name")])


# 函数功能：归一化 Gazebo 派生模型的关节范围。
def _normalise_gazebo_joint_limits(root):
    """为 ROS/Gazebo 控制派生模型使用单圈腕部关节范围。

    源模型为 joint_6 设置了约 +/-2*pi 的旋转范围，Gazebo 可能因此把零位
    表示为 2*pi；对于非连续关节，JointTrajectoryController 会正确地将
    接近零位的命令视为另一个位置。实际机械臂的常用范围是一整圈，因此
    Gazebo 专用派生模型使用常规的 +/-pi 范围。标准 URDF 和 MuJoCo 转换
    有意保持不变。
    """
    for joint in root.findall("joint"):
        if joint.get("name") in GAZEBO_ARM_EFFORT_LIMITS:
            limit = joint.find("limit")
            if limit is None:
                limit = ET.SubElement(joint, "limit")
            limit.set("effort", str(GAZEBO_ARM_EFFORT_LIMITS[joint.get("name")]))
        if joint.get("name") != "joint_6":
            continue
        limit = joint.find("limit")
        if limit is None:
            limit = ET.SubElement(joint, "limit")
        limit.set("lower", "-3.141592653589793")
        limit.set("upper", "3.141592653589793")
        return
    raise ValueError("missing joint_6")


# 函数功能：为 Gazebo 机械臂关节添加阻尼。
def _add_gazebo_joint_damping(root):
    """为 Gazebo 专用控制派生模型添加物理关节阻尼。"""
    joints = {joint.get("name"): joint for joint in root.findall("joint")}
    for name, damping in GAZEBO_ARM_JOINT_DAMPING.items():
        joint = joints.get(name)
        if joint is None:
            raise ValueError("missing {}".format(name))
        dynamics = joint.find("dynamics")
        if dynamics is None:
            dynamics = ET.SubElement(joint, "dynamics")
        dynamics.set("damping", str(damping))


# 函数功能：为 Gazebo 夹爪关节添加阻尼。
def _add_gripper_joint_damping(root):
    """为 Gazebo 夹爪的六个关节添加阻尼。"""
    joints = {joint.get("name"): joint for joint in root.findall("joint")}
    for name, damping in GAZEBO_GRIPPER_JOINT_DAMPING.items():
        joint = joints.get(name)
        if joint is None:
            raise ValueError("missing {}".format(name))
        dynamics = joint.find("dynamics")
        if dynamics is None:
            dynamics = ET.SubElement(joint, "dynamics")
        dynamics.set("damping", str(damping))


# 函数功能：为指定关节添加 transmission 配置。
def _add_transmission(root, joint_name):
    transmission = ET.Element("transmission", {"name": joint_name + "_transmission"})
    _add_text(transmission, "type", "transmission_interface/SimpleTransmission")
    joint = ET.SubElement(transmission, "joint", {"name": joint_name})
    _add_text(joint, "hardwareInterface", POSITION_INTERFACE)
    actuator = ET.SubElement(transmission, "actuator", {"name": joint_name + "_actuator"})
    _add_text(actuator, "hardwareInterface", POSITION_INTERFACE)
    _add_text(actuator, "mechanicalReduction", "1")
    root.append(transmission)


# 函数功能：统一各关节的 transmission 配置。
def _normalise_transmissions(root):
    existing = set()
    for transmission in root.findall("transmission"):
        for interface in transmission.iter("hardwareInterface"):
            interface.text = POSITION_INTERFACE
        joint = transmission.find("joint")
        if joint is not None and joint.get("name"):
            existing.add(joint.get("name"))
    for joint_name in ARM_JOINTS:
        if joint_name not in existing:
            _add_transmission(root, joint_name)
    if SIM_GRIPPER_JOINT not in existing:
        _add_transmission(root, SIM_GRIPPER_JOINT)


# 函数功能：向模型添加固定坐标系链接。
def _add_fixed_frame(root, name, parent, rpy="0 0 0", xyz="0 0 0"):
    ET.SubElement(root, "link", {"name": name})
    joint = ET.SubElement(root, "joint", {"name": name + "_joint", "type": "fixed"})
    ET.SubElement(joint, "origin", {"xyz": xyz, "rpy": rpy})
    ET.SubElement(joint, "parent", {"link": parent})
    ET.SubElement(joint, "child", {"link": name})


# 函数功能：添加相机光学坐标系。
def _add_camera_frames(root):
    link_names = {link.get("name") for link in root.findall("link")}
    if CAMERA_SENSOR_LINK in link_names or "camera_color_optical_frame" in link_names:
        return
    # 保持 CAD 可视链接不变，但在真实镜头中心为模拟传感器建立独立坐标系。
    # 传感器坐标系的 +X 轴旋转到 camera_link 的 +Z 镜头法线方向。
    _add_fixed_frame(
        root,
        CAMERA_SENSOR_LINK,
        "camera_link",
        rpy=CAMERA_SENSOR_RPY,
        xyz="{} 0 0".format(CAMERA_LENS_CENTER_X),
    )
    _add_fixed_frame(root, "camera_color_frame", CAMERA_SENSOR_LINK)
    _add_fixed_frame(
        root,
        "camera_color_optical_frame",
        "camera_color_frame",
        rpy="-1.5708 0 -1.5708",
    )
    _add_fixed_frame(root, "camera_depth_frame", CAMERA_SENSOR_LINK)
    _add_fixed_frame(
        root,
        "camera_depth_optical_frame",
        "camera_depth_frame",
        rpy="-1.5708 0 -1.5708",
    )


# 函数功能：为模型添加世界固定关节。
def _add_world_anchor(root, child_link="base_link", joint_name="rml63ii_fixed_to_world"):
    if any(link.get("name") == "world" for link in root.findall("link")):
        raise ValueError("world link already exists")
    ET.SubElement(root, "link", {"name": "world"})
    joint = ET.SubElement(root, "joint", {"name": joint_name, "type": "fixed"})
    ET.SubElement(joint, "parent", {"link": "world"})
    ET.SubElement(joint, "child", {"link": child_link})


# 函数功能：将可复用工作台组件合并到机器人模型。
def _merge_table_component(root, table_source):
    """将可复用的工作台关节树合并到 Gazebo 机器人派生模型中。"""
    table_root = ET.parse(str(table_source)).getroot()
    if table_root.tag != "robot":
        raise ValueError("table source root element must be <robot>")

    existing_names = {
        tag: {item.get("name") for item in root.findall(tag) if item.get("name")}
        for tag in ("link", "joint", "transmission")
    }
    for tag in ("link", "joint", "transmission"):
        table_names = [
            item.get("name") for item in table_root.findall(tag) if item.get("name")
        ]
        duplicates = existing_names[tag].intersection(table_names)
        if duplicates:
            raise ValueError(
                "duplicate {} names in table component: {}".format(
                    tag, sorted(duplicates)
                )
            )
        if len(table_names) != len(set(table_names)):
            raise ValueError("duplicate {} names inside table component".format(tag))

    table_links = {link.get("name") for link in table_root.findall("link")}
    table_joints = {joint.get("name") for joint in table_root.findall("joint")}
    if TABLE_ROOT_LINK not in table_links:
        raise ValueError("table source is missing {}".format(TABLE_ROOT_LINK))
    missing_joints = set(TABLE_JOINTS) - table_joints
    if missing_joints:
        raise ValueError("table source is missing joints: {}".format(sorted(missing_joints)))

    for child in table_root:
        root.append(copy.deepcopy(child))


# 函数功能：添加工作台到机械臂的安装结构。
def _add_table_arm_mount(root):
    joint = ET.SubElement(root, "joint", {"name": ARM_MOUNT_JOINT, "type": "fixed"})
    ET.SubElement(joint, "origin", {"xyz": ARM_MOUNT_XYZ, "rpy": "0 0 0"})
    ET.SubElement(joint, "parent", {"link": TABLE_ROOT_LINK})
    ET.SubElement(joint, "child", {"link": "base_link"})


# 函数功能：添加 Gazebo 专用碰撞和重力设置。
def _add_auxiliary_gazebo_settings(root, enable_external_collisions=False):
    # 常规 Gazebo 位置控制派生模型用于接口验证，不用于力矩辨识。导出的 CAD
    # 碰撞外壳可能在零位姿态互相穿透并向关节注入力，因此旧版派生模型禁用
    # 这些接触。阶段 1 工作区派生模型启用与外部模型接触所需的物理碰撞外壳，
    # 同时明确禁用机械臂链接之间的自碰撞。
    auxiliary_links = {
        "link_1",
        "link_2",
        "link_3",
        "link_4",
        "link_5",
        "link_6",
        "camera_rolink",
        "camera_link",
        FLANGE_LINK,
        "4C2_baselink",
        "4C2_Link1",
        "4C2_Link2",
        "4C2_Link3",
        "4C2_Link4",
        "4C2_Link5",
        "4C2_Link6",
    }
    for link_name in auxiliary_links:
        gazebo = ET.SubElement(root, "gazebo", {"reference": link_name})
        if link_name in GAZEBO_NO_GRAVITY_LINKS:
            _add_text(gazebo, "gravity", "false")
        link = next((item for item in root.findall("link") if item.get("name") == link_name), None)
        if link is None:
            continue
        if enable_external_collisions:
            _add_text(gazebo, "selfCollide", "false")
            for collision in link.findall("collision"):
                surface = collision.find("surface")
                if surface is None:
                    continue
                contact = surface.find("contact")
                if contact is None:
                    continue
                bitmask = contact.find("collide_bitmask")
                if bitmask is not None:
                    contact.remove(bitmask)
            continue

        for collision in link.findall("collision"):
            surface = collision.find("surface")
            if surface is None:
                surface = ET.SubElement(collision, "surface")
            contact = surface.find("contact")
            if contact is None:
                contact = ET.SubElement(surface, "contact")
            bitmask = contact.find("collide_bitmask")
            if bitmask is None:
                bitmask = ET.SubElement(contact, "collide_bitmask")
            bitmask.text = "0"


# 函数功能：添加 ROS 控制器接口配置。
def _add_ros_control(root):
    gazebo = ET.SubElement(root, "gazebo")
    plugin = ET.SubElement(
        gazebo,
        "plugin",
        {"name": "gazebo_ros_control", "filename": "libgazebo_ros_control.so"},
    )
    _add_text(plugin, "robotNamespace", "/arm")
    _add_text(plugin, "robotSimType", "gazebo_ros_control/DefaultRobotHWSim")
    _add_text(plugin, "legacyModeNS", "true")


# 函数功能：添加夹爪 mimic 跟随插件。
def _add_mimic_plugins(root):
    for destination, multiplier in MIMIC_JOINTS:
        gazebo = ET.SubElement(root, "gazebo")
        plugin = ET.SubElement(
            gazebo,
            "plugin",
            {"name": "mimic_{}".format(destination), "filename": MIMIC_PLUGIN},
        )
        _add_text(plugin, "joint", SIM_GRIPPER_JOINT)
        _add_text(plugin, "mimicJoint", GRIPPER_RENAMES[destination])
        _add_text(plugin, "multiplier", multiplier)
        _add_text(plugin, "offset", 0.0)
        # 保持插件的常规位置跟随模式。PID 模式对该 CAD 夹爪很小的惯性较敏感，
        # 可能使跟随链不稳定。端点余量和关节阻尼由下面的桥接器/图形界面处理。
        _add_text(plugin, "sensitiveness", 0.001)


# 函数功能：添加 D435 风格 RGB-D 传感器。
def _add_d435_sensor(root):
    gazebo = ET.SubElement(root, "gazebo", {"reference": CAMERA_SENSOR_LINK})
    sensor = ET.SubElement(gazebo, "sensor", {"name": "d435_rgbd_sensor", "type": "depth"})
    _add_text(sensor, "always_on", "true")
    _add_text(sensor, "update_rate", "30")
    _add_text(sensor, "visualize", "false")
    camera = ET.SubElement(sensor, "camera")
    _add_text(camera, "horizontal_fov", "0.9666")
    image = ET.SubElement(camera, "image")
    _add_text(image, "width", "640")
    _add_text(image, "height", "480")
    _add_text(image, "format", "R8G8B8")
    clip = ET.SubElement(camera, "clip")
    _add_text(clip, "near", "0.1")
    _add_text(clip, "far", "10.0")
    noise = ET.SubElement(camera, "noise")
    _add_text(noise, "type", "gaussian")
    _add_text(noise, "mean", "0.0")
    _add_text(noise, "stddev", "0.0")
    depth_camera = ET.SubElement(camera, "depth_camera")
    # 在 Gazebo 11 中包含 `points` 会使 DepthCamera::PostRender 输出 RGB 点云
    # 缓冲区而不是深度缓冲区。OpenNI 封装器接收深度帧并同时生成 ROS 深度图
    # 和 ROS 点云，因此这里只请求 `depth`。（ROS 点云话题仍由封装器完整提供。）
    _add_text(depth_camera, "output", "depth")

    plugin = ET.SubElement(
        sensor,
        "plugin",
        {"name": "d435_openni_ros", "filename": "libgazebo_ros_openni_kinect.so"},
    )
    _add_text(plugin, "baseline", "0.05")
    _add_text(plugin, "alwaysOn", "true")
    _add_text(plugin, "updateRate", "0.0")
    _add_text(plugin, "cameraName", "camera")
    _add_text(plugin, "imageTopicName", "/camera/color/image_raw")
    _add_text(plugin, "cameraInfoTopicName", "/camera/color/camera_info")
    _add_text(plugin, "depthImageTopicName", "/camera/depth/image_rect_raw")
    _add_text(plugin, "depthImageCameraInfoTopicName", "/camera/depth/camera_info")
    _add_text(plugin, "pointCloudTopicName", "/camera/depth/points")
    _add_text(plugin, "frameName", "camera_color_optical_frame")
    _add_text(plugin, "pointCloudCutoff", "0.1")
    _add_text(plugin, "pointCloudCutoffMax", "10.0")
    _add_text(plugin, "useDepth16UC1Format", "true")
    for tag in ("distortionK1", "distortionK2", "distortionK3", "distortionT1", "distortionT2"):
        _add_text(plugin, tag, "0.0")
    _add_text(plugin, "hackBaseline", "0.05")


# 函数功能：构建非锚定 Gazebo 工作台组合模型。
def _build_unanchored_gazebo_urdf(source, package_root, enable_external_collisions=False):
    from fingertip_geometry import generate_meshes, add_urdf
    generate_meshes(package_root)
    tree = ET.parse(str(source))
    root = tree.getroot()
    if root.tag != "robot":
        raise ValueError("root element must be <robot>")
    joint_names = {joint.get("name") for joint in root.findall("joint")}
    missing = set(ARM_JOINTS) | {SOURCE_GRIPPER_JOINT, CAMERA_ROLL_JOINT}
    if missing - joint_names:
        raise ValueError("missing joints: {}".format(sorted(missing - joint_names)))

    _replace_mesh_paths(root, package_root)
    _add_gripper_flange(root)
    _fix_camera_roll_joint(root)
    _rotate_gripper_mount(root)
    _rename_gripper_joints(root)
    _normalise_gazebo_joint_limits(root)
    _add_gazebo_joint_damping(root)
    _add_gripper_joint_damping(root)
    add_urdf(root)
    _normalise_transmissions(root)
    _add_camera_frames(root)
    _add_auxiliary_gazebo_settings(root, enable_external_collisions=enable_external_collisions)
    _add_ros_control(root)
    _add_mimic_plugins(root)
    _add_d435_sensor(root)
    return tree


# 函数功能：构建兼容旧接口的 Gazebo 机械臂模型。
def build_gazebo_urdf(source, package_root):
    """构建保持向后兼容且固定在世界坐标系中的仅机械臂模型。"""
    tree = _build_unanchored_gazebo_urdf(source, package_root)
    root = tree.getroot()
    _add_world_anchor(root)
    _indent(root)
    return tree


# 函数功能：构建工作台与机械臂组合模型。
def build_table_gazebo_urdf(
    source, table_source, package_root, locked, enable_external_collisions=False
):
    """构建一个刚性工作台与机械臂组合实体，并按需固定到世界坐标系。"""
    tree = _build_unanchored_gazebo_urdf(
        source,
        package_root,
        enable_external_collisions=enable_external_collisions,
    )
    root = tree.getroot()
    root.set("name", "RML63II-6F-V-EG2-4C2-mobile-table")
    _merge_table_component(root, table_source)
    _add_table_arm_mount(root)
    if locked:
        _add_world_anchor(
            root,
            child_link=TABLE_ROOT_LINK,
            joint_name="mobile_table_fixed_to_world",
        )
    _indent(root)
    return tree


# 函数功能：初始化参数、节点和主运行流程。
def main():
    parser = argparse.ArgumentParser(description=__doc__)
    here = Path(__file__).resolve().parents[1]
    parser.add_argument("--source", type=Path, default=here / "model_source" / "RML63II-6F-V-EG2-4C2.urdf")
    parser.add_argument("--package-root", type=Path, default=here)
    parser.add_argument("--output", type=Path, default=here / "model_gazebo" / "RML63II-6F-V-EG2-4C2.gazebo.urdf")
    parser.add_argument("--table-source", type=Path, default=here / "urdf" / "mobile_table.urdf")
    parser.add_argument(
        "--locked-table-output",
        type=Path,
        default=here / "model_gazebo" / "RML63II-6F-V-EG2-4C2-table-locked.gazebo.urdf",
    )
    parser.add_argument(
        "--passive-table-output",
        type=Path,
        default=here / "model_gazebo" / "RML63II-6F-V-EG2-4C2-table-passive.gazebo.urdf",
    )
    args = parser.parse_args()
    try:
        outputs = (
            (args.output, build_gazebo_urdf(args.source, args.package_root)),
            (
                args.locked_table_output,
                build_table_gazebo_urdf(
                    args.source, args.table_source, args.package_root, locked=True
                ),
            ),
            (
                args.passive_table_output,
                build_table_gazebo_urdf(
                    args.source, args.table_source, args.package_root, locked=False
                ),
            ),
        )
        for output, tree in outputs:
            output.parent.mkdir(parents=True, exist_ok=True)
            tree.write(str(output), encoding="utf-8", xml_declaration=True)
            print("wrote {}".format(output))
        digest = hashlib.sha256(args.source.read_bytes()).hexdigest()
        print("source_sha256 {}".format(digest))
        return 0
    except (OSError, ET.ParseError, ValueError) as exc:
        print("prepare_gazebo_urdf: {}".format(exc), file=sys.stderr)
        return 2


if __name__ == "__main__":
    sys.exit(main())
