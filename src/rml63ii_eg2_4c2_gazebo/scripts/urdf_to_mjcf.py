#!/usr/bin/env python3
"""将 RML63II 标准 URDF 转换为 MuJoCo 模型和场景。

转换过程有意分为两个阶段：先由 MuJoCo 解析清理后的 URDF，再由本脚本写入
MuJoCo 专用执行器、等式约束、相机和接触设置。Gazebo 派生模型永远不会
作为输入。
"""

from __future__ import annotations

import argparse
import math
import os
import sys
import tempfile
import xml.etree.ElementTree as ET
from pathlib import Path
from fingertip_geometry import generate_meshes, add_mjcf


# ===== 模块级常量与辅助定义 =====
ARM_JOINTS = ("joint_1", "joint_2", "joint_3", "joint_4", "joint_5", "joint_6")
SOURCE_GRIPPER_JOINT = "4C2_Joint1"
ACTIVE_GRIPPER_JOINT = "gripper_joint1"
CAMERA_ROLL_JOINT = "camera_rojoint"
CAMERA_LINK = "camera_link"
# 与 Gazebo 派生模型中的 camera_sensor_link 保持一致。camera_link 的局部
# +Z 是镜头朝外方向，光学中心位于前面板局部 x=-32.5 mm 处。
CAMERA_LENS_CENTER_X = -0.0325
CAMERA_HORIZONTAL_FOV = 0.9666
CAMERA_RESOLUTION = (640, 480)
FLANGE_LINK = "gripper_flange"
FLANGE_JOINT = "gripper_flange_joint"
FLANGE_LENGTH = 0.08
FLANGE_RADIUS = 0.0285
FLANGE_MASS = 0.30
# Gazebo 派生模型将整个夹爪组件绕 link_6 的安装轴偏航 +90 度。MuJoCo
# 必须使用同一固定安装变换；机械臂和相机位姿保持源模型不变。
GRIPPER_MOUNT_YAW = 1.5707963267948966
GRIPPER_RENAMES = {
    "4C2_Joint1": "gripper_joint1",
    "4C2_Joint2": "gripper_joint2",
    "4C2_Joint3": "gripper_joint3",
    "4C2_Joint4": "gripper_joint4",
    "4C2_Joint5": "gripper_joint5",
    "4C2_Joint6": "gripper_joint6",
}
MIMIC_JOINTS = (
    ("gripper_joint5", 1.0),
    ("gripper_joint2", 1.0),
    ("gripper_joint3", -1.0),
    ("gripper_joint4", -1.0),
    ("gripper_joint6", 1.0),
)

# 保持 MuJoCo 场景与 Gazebo 工作区的几何体一致。工作台本身没有 free joint，
# 因此在训练场景中是固定在世界上的静态设施。这相当于 Gazebo 默认的
# ``table_locked:=true`` 模式：策略执行期间，机械臂反作用力不能平移或掀翻工作台。
TABLE_LENGTH = 0.600
TABLE_WIDTH = 0.400
TABLETOP_THICKNESS = 0.030
TABLETOP_TOP_Z = 0.700
TABLETOP_CENTER_Z = TABLETOP_TOP_Z - TABLETOP_THICKNESS / 2.0
ARM_MOUNT_XYZ = "-0.140 0 0.700"
TABLE_BODY_MASS = 240.0
TABLE_BODY_COM = "0 0 0.32"
TABLE_BODY_INERTIA = "13.000 17.000 10.400"
# MuJoCo 阶段 1 的初始位置只要求与电气柜保持正间隙；小车初始绕竖直轴
# 旋转 180 度，使机械臂朝向符合实际实验中的反向停放姿态。
STAGE1_TABLE_POS = "1.40 -0.35 0"
STAGE1_TABLE_QUAT = "0 0 0 1"
CASTER_PREFIXES = (
    ("front_left", 0.255, 0.155),
    ("front_right", 0.255, -0.155),
    ("rear_left", -0.255, 0.155),
    ("rear_right", -0.255, -0.155),
)



# 函数功能：在腕部与夹爪之间添加固定法兰。
def _add_gripper_flange(root):
    """添加与 Gazebo 派生模型相同的固定 80 mm 间隔法兰。

    该操作在 MuJoCo 导入前作用于清理后的临时 URDF；标准源文件永远不会被
    修改。MJCF 保持源夹爪方向，只增加轴向间隔法兰。
    """
    if any(link.get("name") == FLANGE_LINK for link in root.findall("link")):
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
    visual = ET.SubElement(flange, "visual")
    ET.SubElement(visual, "origin", {"xyz": "0 0 0.04", "rpy": "0 0 0"})
    visual_geometry = ET.SubElement(visual, "geometry")
    ET.SubElement(
        visual_geometry,
        "cylinder",
        {"radius": str(FLANGE_RADIUS), "length": str(FLANGE_LENGTH)},
    )
    material = ET.SubElement(visual, "material", {"name": "flange_gray"})
    ET.SubElement(material, "color", {"rgba": "0.32 0.34 0.38 1"})
    collision = ET.SubElement(flange, "collision")
    ET.SubElement(collision, "origin", {"xyz": "0 0 0.04", "rpy": "0 0 0"})
    collision_geometry = ET.SubElement(collision, "geometry")
    ET.SubElement(
        collision_geometry,
        "cylinder",
        {"radius": str(FLANGE_RADIUS), "length": str(FLANGE_LENGTH)},
    )

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


# 函数功能：同步 Gazebo 的夹爪安装偏航修正。
def _rotate_gripper_mount(root):
    for joint in root.findall("joint"):
        if joint.get("name") != "gripper_mount_joint":
            continue
        origin = joint.find("origin")
        if origin is None:
            origin = ET.SubElement(joint, "origin", {"xyz": "0 0 0"})
        origin.set("rpy", "0 0 {}".format(GRIPPER_MOUNT_YAW))
        return
    raise ValueError("missing gripper_mount_joint")


# 函数功能：清理源 URDF，添加法兰并移除 Gazebo/ROS 专用配置。
def _clean_urdf(source, package_root):
    tree = ET.parse(str(source))
    root = tree.getroot()
    if root.tag != "robot":
        raise ValueError("source root must be <robot>")

    # 只同步 Gazebo 对夹爪安装关节的 +90 度修正；机械臂与相机链不改动。
    _rotate_gripper_mount(root)
    _add_gripper_flange(root)

    for child in list(root):
        if child.tag in ("transmission", "gazebo"):
            root.remove(child)

    # 某些 CAD 导出的惯性在打印精度下不是正定的（相机链接特别小）。MuJoCo
    # 文档中的 URDF 扩展可以在导入时平衡这些值；生成的 MJCF 包含修正后的惯性，
    # 而源 URDF 保持不变。
    mujoco_extension = root.find("mujoco")
    if mujoco_extension is None:
        mujoco_extension = ET.SubElement(root, "mujoco")
    compiler = mujoco_extension.find("compiler")
    if compiler is None:
        compiler = ET.SubElement(mujoco_extension, "compiler")
    compiler.set("balanceinertia", "true")
    compiler.set("discardvisual", "false")
    # 保留固定 CAD 链接对应的命名 MJCF body。这样可以检查 16 个链接的源模型到
    # MJCF 映射，并为数据集工具保留稳定的相机/夹爪 body 名称；固定关节仍不会
    # 增加自由度。
    compiler.set("fusestatic", "false")

    for mesh in root.iter("mesh"):
        filename = mesh.get("filename", "")
        if not filename.startswith("package://"):
            continue
        package_and_path = filename[len("package://") :]
        package, separator, relative = package_and_path.partition("/")
        if not separator:
            raise ValueError("invalid mesh URI: {}".format(filename))
        if package == "RML63II-6F-V":
            asset_path = Path("assets") / "arm_meshes" / Path(relative).name
        elif package == "EG2-4C2":
            asset_path = Path("assets") / "gripper_meshes" / Path(relative).name
        else:
            raise ValueError("unknown mesh package: {}".format(package))
        mesh_path = package_root / asset_path
        if not mesh_path.is_file():
            raise ValueError("missing mesh: {}".format(mesh_path))
        mesh.set("filename", asset_path.as_posix())

    for joint in root.findall("joint"):
        if joint.get("name") == CAMERA_ROLL_JOINT:
            joint.set("type", "fixed")
            for tag in ("axis", "limit", "dynamics", "mimic", "calibration", "safety_controller"):
                for item in list(joint.findall(tag)):
                    joint.remove(item)
        old_name = joint.get("name")
        if old_name in GRIPPER_RENAMES:
            joint.set("name", GRIPPER_RENAMES[old_name])
        mimic = joint.find("mimic")
        if mimic is not None:
    # 下面的 MuJoCo equality 约束是权威的 mimic 实现。移除这个仅供 ROS 使用的
    # 标签，可以避免依赖导入器特有的 mimic 行为。
            joint.remove(mimic)

    return tree


# 函数功能：按名称查找转换结果中的 MJCF body。
def _body_by_name(root, names):
    for body in root.iter("body"):
        if body.get("name") in names:
            return body
    return None


# 函数功能：将转换结果中的网格路径改写为软件包相对路径。
def _rewrite_mesh_paths(root, package_root):
    for mesh in root.iter("mesh"):
        filename = mesh.get("file")
        if not filename:
            continue
        name = Path(filename).name
        if (package_root / "assets" / "gripper_meshes" / name).is_file():
            mesh.set("file", "../assets/gripper_meshes/{}".format(name))
        else:
            mesh.set("file", "../assets/arm_meshes/{}".format(name))


# 函数功能：为机械臂和夹爪创建 MuJoCo 位置执行器。
def _add_actuators(root):
    actuator = root.find("actuator")
    if actuator is not None:
        root.remove(actuator)
    actuator = ET.SubElement(root, "actuator")
    # 这些是仿真位置环限制，不是 ROS/Gazebo transmission 的 effort。CAD 惯性
    # 包含相机和夹爪负载，因此在 MuJoCo 重力下，源产品目录中的 effort 不足以
    # 保持有用的训练姿态。关节范围仍使用源范围；只提高执行器权限以稳定执行策略。
    limits = {
        "joint_1": (-3.14, 3.14, 200.0),
        "joint_2": (-3.14, 3.14, 200.0),
        "joint_3": (-3.14, 3.14, 150.0),
        "joint_4": (-3.14, 3.14, 150.0),
        "joint_5": (-3.14, 3.14, 80.0),
        "joint_6": (-6.28, 6.28, 80.0),
    }
    for joint_name in ARM_JOINTS:
        lower, upper, effort = limits[joint_name]
        ET.SubElement(
            actuator,
            "position",
            {
                "name": "act_{}".format(joint_name),
                "joint": joint_name,
                "kp": "200",
                "kv": "8",
                "ctrlrange": "{} {}".format(lower, upper),
                "forcerange": "{} {}".format(-effort, effort),
            },
        )
    ET.SubElement(
        actuator,
        "position",
        {
            "name": "act_gripper",
            "joint": ACTIVE_GRIPPER_JOINT,
            "kp": "5",
            "ctrlrange": "0 0.82",
            "forcerange": "-1 1",
        },
    )


# 函数功能：根据夹爪映射创建 MuJoCo mimic 等式约束。
def _add_equalities(root):
    equality = root.find("equality")
    if equality is not None:
        root.remove(equality)
    equality = ET.SubElement(root, "equality")
    for destination, multiplier in MIMIC_JOINTS:
        ET.SubElement(
            equality,
            "joint",
            {
                "name": "mimic_{}".format(destination),
                "joint1": ACTIVE_GRIPPER_JOINT,
                "joint2": destination,
                "polycoef": "0 {} 0 0 0".format(multiplier),
                "solref": "0.002 1",
                "solimp": "0.9999 0.9999 0.0001 0.5 2",
            },
        )


# 函数功能：为腕部相机创建 RGB-D 相机配置。
def _add_contact_stability(root):
    """Explicit finger contact surfaces and passive damping (SI units).

    CAD link inertias and actuation limits are retained. Only the distal
    collision hull is split into a knuckle, shank and contact pad; the visible
    mesh is untouched. Pad inner faces follow the CAD x=+/-0.0177 surface.
    """
    for joint in root.findall(".//body/joint"):
        if joint.get("name", "").startswith("gripper_joint"):
            # Small CAD link inertias otherwise oscillate when the pad ends
            # press on the panel. This dissipates motion without extra torque.
            joint.set("damping", "0.05")
            # Reflected transmission inertia (simulation equivalent, not a
            # measured motor specification). Conditions the very light CAD
            # linkage so finite-impedance mimic constraints transmit load.
            joint.set('armature', '0.0001')
            joint.set('solreflimit', '0.004 1')
            joint.set('solimplimit', '0.99 0.999 0.0001')
    for side, body_name, sign in (("left", "4C2_Link5", 1), ("right", "4C2_Link6", -1)):
        body = _body_by_name(root, (body_name,))
        for geom in list(body.findall("geom")):
            if geom.get("contype") != "0":
                body.remove(geom)
        for part, x, z, sx, sz in (("knuckle", 0, 0, .005, .005),
                                  ("shank", .0065, .013, .0065, .012),
                                  ("pad", .01385, .025, .00385, .0035)):
            attrs = dict(name=f"{side}_finger_{part}", type="box",
                         pos=f"{sign*x} 0.006 {z}", size=f"{sx} 0.009 {sz}",
                         contype="1", conaffinity="1", group="3", rgba="0.2 0.6 0.8 0.25")
            if part == "pad":
                attrs.update(friction="1.0 0.001 0.0001", condim="4",
                             solref="0.004 1", solimp="0.9 0.95 0.001 0.5 2", priority="1")
            ET.SubElement(body, "geom", attrs)
        ET.SubElement(body, "site", name=f"{side}_finger_contact", type="sphere",
                      pos=f"{sign*.0177} 0.006 0.025", size="0.001", group="3")


def _add_cameras(root):
    # fusestatic=false 会保留 camera_link body，因此相机应直接使用 Gazebo
    # camera_sensor_link 相对于 camera_link 的局部位姿。相机修正只负责保持
    # 已验证的相机视角；夹爪的相对安装角由 _rotate_gripper_mount 单独处理。
    body = _body_by_name(root, (CAMERA_LINK,))
    if body is None:
        raise ValueError("converted MJCF has no camera_link body")

    # 清除任何旧生成结果，使后处理函数对已有 MJCF 也保持幂等。
    for parent in root.iter():
        for camera in list(parent.findall("camera")):
            if camera.get("name", "").startswith("d435_"):
                parent.remove(camera)

    camera_pos = "{} 0 0".format(CAMERA_LENS_CENTER_X)
    # Gazebo 相机沿传感器局部 +X 看、+Z 朝上；该传感器 frame 相对
    # camera_link 的 RPY 为 (+pi/2, -pi/2, 0)。MuJoCo 相机沿自身 -Z 看、
    # +Y 朝上。合成两套轴约定后，相对 camera_link 恰好是绕 +X 旋转 pi，
    # 即 MuJoCo [w, x, y, z] 四元数 (0, 1, 0, 0)。最终视线沿
    # camera_link/+Z，也就是 link_6 的夹爪方向。
    camera_quat = "0 1 0 0"
    width, height = CAMERA_RESOLUTION
    vertical_fov = math.degrees(
        2.0
        * math.atan(
            math.tan(CAMERA_HORIZONTAL_FOV / 2.0)
            * float(height)
            / float(width)
        )
    )
    camera_attributes = {
        "pos": camera_pos,
        "quat": camera_quat,
        "fovy": "{:.6f}".format(vertical_fov),
        "resolution": "{} {}".format(width, height),
    }
    for name in ("d435_rgb", "d435_depth"):
        attributes = dict(camera_attributes)
        attributes["name"] = name
        ET.SubElement(body, "camera", attributes)


# 函数功能：添加机械臂、夹爪及结构性相邻 body 的碰撞排除规则。
def _add_contacts(root):
    contact = root.find("contact")
    if contact is None:
        contact = ET.SubElement(root, "contact")
    existing = {(item.get("body1"), item.get("body2")) for item in contact.findall("exclude")}
    gripper_links = ("4C2_baselink", "4C2_Link1", "4C2_Link2", "4C2_Link3", "4C2_Link4", "4C2_Link5", "4C2_Link6", "left_press_tip_link", "right_press_tip_link")
    body_names = {body.get("name") for body in root.iter("body")}
    # URDF 导入器可能将固定 body 合并到 link_6。这里只为编译后仍存在的 body
    # 添加排除项；指尖/物体接触仍为后续抓取任务保留。
    active_links = [name for name in gripper_links if name in body_names]
    for index, body1 in enumerate(active_links):
        for body2 in active_links[index + 1 :]:
            if (body1, body2) not in existing and (body2, body1) not in existing:
                ET.SubElement(contact, "exclude", {"body1": body1, "body2": body2})

    # 相邻 CAD 碰撞网格通常会在旋转关节接口处重叠。排除这些结构性碰撞对可以
    # 避免人为的自接触冲量，同时保留非相邻机械臂和夹爪/物体接触。
    structural_pairs = (
        ("base_link", "link_1"),
        ("link_1", "link_2"),
        ("link_2", "link_3"),
        ("link_3", "link_4"),
        ("link_4", "link_5"),
        ("link_5", "link_6"),
        ("link_6", "camera_rolink"),
        ("camera_rolink", "camera_link"),
        ("link_6", FLANGE_LINK),
        (FLANGE_LINK, "4C2_baselink"),
    )
    for body1, body2 in structural_pairs:
        if body1 in body_names and body2 in body_names:
            if (body1, body2) not in existing and (body2, body1) not in existing:
                ET.SubElement(contact, "exclude", {"body1": body1, "body2": body2})
    if "link_6" in body_names:
        for gripper_link in active_links:
            if ("link_6", gripper_link) not in existing and (gripper_link, "link_6") not in existing:
                ET.SubElement(contact, "exclude", {"body1": "link_6", "body2": gripper_link})


# 函数功能：在静态模式下将机械臂安装到固定移动工作台 body。
def _add_static_mobile_table(root):
    """用固定的 700 mm 移动工作台几何体包裹转换后的机械臂。

    没有 free/slide/hinge 关节的 MuJoCo body 会焊接到父 body。将
    ``mobile_table`` 作为机械臂父 body，既能使场景获得与 Gazebo 锁定版本
    相同的刚性树，又能避免生成重合的独立世界固定机械臂和工作台。四个脚轮
    在该固定运行模式下仅作为可视/接触几何体；脚轮被动动力学仍是 Gazebo
    专用布置模式。
    """
    worldbody = root.find("worldbody")
    if worldbody is None:
        raise ValueError("converted MJCF has no worldbody")
    base_link = next(
        (
            body
            for body in list(worldbody)
            if body.tag == "body" and body.get("name") == "base_link"
        ),
        None,
    )
    if base_link is None:
        raise ValueError("converted MJCF has no root base_link body")

    worldbody.remove(base_link)
    table = ET.SubElement(worldbody, "body", {"name": "mobile_table"})
    ET.SubElement(
        table,
        "geom",
        {
            "name": "mobile_table_top",
            "type": "box",
            "pos": "0 0 {:.3f}".format(TABLETOP_CENTER_Z),
            "size": "{:.3f} {:.3f} {:.3f}".format(
                TABLE_LENGTH / 2.0, TABLE_WIDTH / 2.0, TABLETOP_THICKNESS / 2.0
            ),
            "friction": "1.2 0.005 0.0001",
            "rgba": "0.33 0.37 0.40 1",
        },
    )

    # 框架立柱从 z=0.100 的脚轮支撑延伸到 z=0.670 的台面下表面。这里有意只
    # 使用基本几何体，使生成场景除机械臂外不依赖外部网格。
    for x_label, x in (("front", 0.255), ("rear", -0.255)):
        for y_label, y in (("left", 0.155), ("right", -0.155)):
            corner = "{}_{}".format(x_label, y_label)
            ET.SubElement(
                table,
                "geom",
                {
                    "name": "mobile_table_{}_leg".format(corner),
                    "type": "box",
                    "pos": "{:.3f} {:.3f} 0.385".format(x, y),
                    "size": "0.015 0.015 0.285",
                    "rgba": "0.22 0.26 0.29 1",
                },
            )
            ET.SubElement(
                table,
                "geom",
                {
                    "name": "mobile_table_{}_caster_yoke".format(corner),
                    "type": "box",
                    "pos": "{:.3f} {:.3f} 0.105".format(x, y),
                    "size": "0.030 0.030 0.025",
                    "rgba": "0.16 0.18 0.20 1",
                },
            )
            ET.SubElement(
                table,
                "geom",
                {
                    "name": "mobile_table_{}_wheel".format(corner),
                    "type": "cylinder",
                    "pos": "{:.3f} {:.3f} 0.050".format(x, y),
                    "euler": "1.570796 0 0",
                    "size": "0.050 0.0125",
                    "friction": "1.2 0.005 0.0001",
                    "rgba": "0.06 0.06 0.07 1",
                },
            )

    # 两根下部框架横梁让简单的小车轮廓更容易在 MuJoCo 查看器中观察，同时为
    # 场景道具增加可用的碰撞几何体。
    ET.SubElement(
        table,
        "geom",
        {
            "name": "mobile_table_lower_x_rail",
            "type": "box",
            "pos": "0 0.155 0.150",
            "size": "0.270 0.012 0.012",
            "rgba": "0.22 0.26 0.29 1",
        },
    )
    ET.SubElement(
        table,
        "geom",
        {
            "name": "mobile_table_lower_y_rail",
            "type": "box",
            "pos": "0.255 0 0.150",
            "size": "0.012 0.170 0.012",
            "rgba": "0.22 0.26 0.29 1",
        },
    )

    base_link.set("pos", ARM_MOUNT_XYZ)
    table.append(base_link)


# 函数功能：为机械臂末端添加可标定的 TCP site。
def _add_dynamic_mobile_table(root, table_pos=None, table_quat=None):
    """用物理上可移动的脚轮工作台包裹转换后的机械臂。

    固定模型有意使用一个焊接工作台 body，以保证机械臂运行稳定。此变体
    保留相同的几何体和机械臂安装位置，但为工作台提供 free base，并根据
    ``urdf/mobile_table.urdf`` 重建四组旋转/滚动脚轮链。不添加工作台执行器；
    调用方通过环境接口中的接触或 ``data.xfrc_applied`` 移动工作台。
    """
    worldbody = root.find("worldbody")
    if worldbody is None:
        raise ValueError("converted MJCF has no worldbody")
    base_link = next(
        (
            body
            for body in list(worldbody)
            if body.tag == "body" and body.get("name") == "base_link"
        ),
        None,
    )
    if base_link is None:
        raise ValueError("converted MJCF has no root base_link body")

    worldbody.remove(base_link)
    table_attributes = {"name": "mobile_table"}
    if table_pos is not None:
        table_attributes["pos"] = table_pos
    if table_quat is not None:
        table_attributes["quat"] = table_quat
    table = ET.SubElement(worldbody, "body", table_attributes)
    ET.SubElement(table, "freejoint", {"name": "mobile_table_freejoint"})
    ET.SubElement(
        table,
        "inertial",
        {
            "pos": TABLE_BODY_COM,
            "mass": str(TABLE_BODY_MASS),
            "diaginertia": TABLE_BODY_INERTIA,
        },
    )

    # 函数功能：向工作台 body 添加带统一默认属性的几何体。
    def add_geom(name, geom_type, pos, size, **attributes):
        values = {
            "name": name,
            "type": geom_type,
            "pos": pos,
            "size": size,
            "density": "0",
        }
        values.update(attributes)
        return ET.SubElement(table, "geom", values)

    add_geom(
        "mobile_table_top",
        "box",
        "0 0 {:.3f}".format(TABLETOP_CENTER_Z),
        "{:.3f} {:.3f} {:.3f}".format(
            TABLE_LENGTH / 2.0, TABLE_WIDTH / 2.0, TABLETOP_THICKNESS / 2.0
        ),
        friction="1.2 0.005 0.0001",
        rgba="0.33 0.37 0.40 1",
    )
    for x_label, x in (("front", 0.255), ("rear", -0.255)):
        for y_label, y in (("left", 0.155), ("right", -0.155)):
            corner = "{}_{}".format(x_label, y_label)
            add_geom(
                "mobile_table_{}_leg".format(corner),
                "box",
                "{:.3f} {:.3f} 0.385".format(x, y),
                "0.015 0.015 0.285",
                rgba="0.22 0.26 0.29 1",
            )

    add_geom(
        "mobile_table_lower_x_rail",
        "box",
        "0 0.155 0.150",
        "0.270 0.012 0.012",
        rgba="0.22 0.26 0.29 1",
    )
    add_geom(
        "mobile_table_lower_y_rail",
        "box",
        "0.255 0 0.150",
        "0.012 0.170 0.012",
        rgba="0.22 0.26 0.29 1",
    )

    for prefix, x, y in CASTER_PREFIXES:
        caster = ET.SubElement(
            table,
            "body",
            {
                "name": "{}_caster_link".format(prefix),
                "pos": "{:.3f} {:.3f} 0.110".format(x, y),
            },
        )
        ET.SubElement(
            caster,
            "inertial",
            {
                "pos": "0.020 0 -0.030",
                "mass": "0.2",
                "diaginertia": "0.00010 0.00015 0.00010",
            },
        )
        ET.SubElement(
            caster,
            "joint",
            {
                "name": "{}_caster_swivel_joint".format(prefix),
                "type": "hinge",
                "axis": "0 0 1",
                "damping": "0.50",
                "frictionloss": "0.50",
                # Retain bearing resistance, but avoid sub-threshold creep of
                # a free cart under a few newtons of door contact force.
                "solreffriction": "0.005 1",
                "solimpfriction": "0.999 0.999 0.001",
            },
        )
        ET.SubElement(
            caster,
            "geom",
            {
                "name": "{}_caster_yoke".format(prefix),
                "type": "box",
                "pos": "0.020 0 -0.030",
                "size": "0.030 0.030 0.025",
                "density": "0",
                "rgba": "0.16 0.18 0.20 1",
            },
        )

        wheel = ET.SubElement(
            caster,
            "body",
            {
                "name": "{}_wheel_link".format(prefix),
                "pos": "0.030 0 -0.060",
            },
        )
        ET.SubElement(
            wheel,
            "inertial",
            {
                "pos": "0 0 0",
                "mass": "0.3",
                "diaginertia": "0.000203125 0.000375 0.000203125",
            },
        )
        ET.SubElement(
            wheel,
            "joint",
            {
                "name": "{}_wheel_joint".format(prefix),
                "type": "hinge",
                "axis": "0 1 0",
                "damping": "0.20",
                "frictionloss": "0.20",
                "solreffriction": "0.005 1",
                "solimpfriction": "0.999 0.999 0.001",
            },
        )
        ET.SubElement(
            wheel,
            "geom",
            {
                "name": "{}_wheel".format(prefix),
                "type": "cylinder",
                "euler": "1.570796 0 0",
                "size": "0.050 0.0125",
                "density": "0",
                "friction": "0.8 0.005 0.0001",
                "rgba": "0.06 0.06 0.07 1",
            },
        )

    # 机械臂底座安装在台面上。其结构碰撞与工作台本体保持分离；源机械臂的接触
    # 排除规则继续用于防止 CAD 自接触冲量。
    base_link.set("pos", ARM_MOUNT_XYZ)
    table.append(base_link)

def _enable_external_collisions(root):
    """启用阶段 1 机器人导出模型中的所有非可视碰撞几何体。

    常规 MuJoCo 导出有意不显式写出碰撞属性，以保持现有默认值不变。组合
    工作区还包含电气柜模型，而电气柜独立演示的默认设置会禁用大多数几何体。
    显式启用机器人的碰撞几何体，可以避免电气柜默认值泄漏到组合后的工作台
    和机械臂模型中。
    """
    for geom in root.iter("geom"):
        if geom.get("contype") is None:
            geom.set("contype", "1")
        if geom.get("conaffinity") is None:
            geom.set("conaffinity", "1")

def _add_stage2_tcp(root):
    """添加可配置的 TCP 标定 site。

    该 site 有意放置在 ``4C2_baselink`` 原点。``RML63IIEnv`` 在加载时将用户
    提供的局部平移/方向标定应用到此 site，因此无需修改机器人几何体。
    """
    if any(site.get("name") == "stage2_tcp" for site in root.iter("site")):
        return
    gripper_base = next(
        (body for body in root.iter("body") if body.get("name") == "4C2_baselink"),
        None,
    )
    if gripper_base is None:
        raise ValueError("converted MJCF has no 4C2_baselink for stage2 TCP")
    ET.SubElement(
        gripper_base,
        "site",
        {
            "name": "stage2_tcp",
            "type": "sphere",
            "pos": "0 0 0",
            "size": "0.008",
            "rgba": "0.10 0.85 0.95 0.85",
            "group": "3",
        },
    )


# 函数功能：对 MuJoCo 导出结果应用执行器、约束、相机和场景后处理。
def _postprocess_mjcf(
    xml_path,
    output,
    package_root,
    dynamic_table=False,
    table_pos=None,
    table_quat=None,
    stage1_collision=False,
    model_name=None,
):
    tree = ET.parse(str(xml_path))
    root = tree.getroot()
    if root.tag != "mujoco":
        raise ValueError("MuJoCo export root must be <mujoco>")
    root.set(
        "model",
        model_name
        or ("rml63ii_eg2_4c2_mobile" if dynamic_table else "rml63ii_eg2_4c2"),
    )
    option = root.find("option")
    if option is None:
        option = ET.Element("option")
        root.insert(0, option)
    option.set("timestep", "0.002")
    option.set("integrator", "implicitfast")
    option.set("gravity", "0 0 -9.81")
    option.set("iterations", "100")
    option.set("tolerance", "1e-10")
    # Keep the normal pad compliance, but resist tangential creep on the
    # narrow handle face. Validated with the light-door physical profile.
    option.set("cone", "elliptic")
    option.set("impratio", "10")
    _rewrite_mesh_paths(root, package_root)
    _add_actuators(root)
    _add_equalities(root)
    _add_contact_stability(root)
    # 2.3's URDF importer does not propagate effort limits as newer versions
    # do. Retain the CAD joint limits in addition to actuator force limits.
    source_limits = {j.get('name'): float(j.find('limit').get('effort'))
                     for j in ET.parse(package_root/'model_source/RML63II-6F-V-EG2-4C2.urdf').findall('joint')
                     if j.find('limit') is not None and j.find('limit').get('effort')}
    for joint in root.findall('.//body/joint'):
        name = joint.get('name','')
        source_name = name.replace('gripper_joint', '4C2_Joint')
        if source_name in source_limits:
            effort = source_limits[source_name]
            joint.set('actuatorforcerange', f'{-effort:g} {effort:g}')
    add_mjcf(root)
    _add_cameras(root)
    _add_contacts(root)
    if dynamic_table:
        _add_dynamic_mobile_table(root, table_pos=table_pos, table_quat=table_quat)
        table = _body_by_name(root, ("mobile_table",))
        ET.SubElement(root.find("worldbody"), "body", name="cart_lock_anchor", mocap="true",
                      pos=table.get("pos", "0 0 0"), quat=table.get("quat", "1 0 0 0"))
        ET.SubElement(root.find("equality"), "weld", name="cart_lock_weld",
                      body1="mobile_table", body2="cart_lock_anchor", active="false",
                      relpose="0 0 0 1 0 0 0", solref="0.004 1", solimp="0.999 0.999 0.001")
    else:
        _add_static_mobile_table(root)
    _add_stage2_tcp(root)
    if stage1_collision:
        _enable_external_collisions(root)
    _indent(root)
    output.parent.mkdir(parents=True, exist_ok=True)
    root.find('compiler').set('autolimits', 'true')
    tree.write(str(output), encoding="utf-8", xml_declaration=True)
    import mujoco_compat
    mujoco_compat.write_compatible_model(output)


# 函数功能：生成包含机械臂、地面与光源的 MuJoCo 场景文件。
def _write_scene(output, model_file, model_name):
    scene = ET.Element("mujoco", {"model": model_name})
    ET.SubElement(scene, "include", {"file": model_file})
    option = ET.SubElement(scene, "option", {"timestep": "0.002", "integrator": "implicitfast"})
    visual = ET.SubElement(scene, "visual")
    ET.SubElement(visual, "global", {"azimuth": "120", "elevation": "-20"})
    worldbody = ET.SubElement(scene, "worldbody")
    ET.SubElement(worldbody, "light", {"name": "key", "pos": "2 -2 3", "dir": "-1 1 -1", "castshadow": "true"})
    ET.SubElement(worldbody, "geom", {"name": "floor", "type": "plane", "size": "3 3 0.1", "rgba": "0.18 0.18 0.18 1"})
    _indent(scene)
    output.parent.mkdir(parents=True, exist_ok=True)
    ET.ElementTree(scene).write(str(output), encoding="utf-8", xml_declaration=True)


def convert(source, package_root, output, scene_output):
    """Generate fixed and passive-cart robot models for both scene entries."""
    import mujoco_compat as mujoco

    package_root = Path(package_root).resolve()
    output, scene_output = Path(output), Path(scene_output)
    mobile_output = output.with_name("rml63ii_eg2_4c2_mobile_stage1.xml")
    generate_meshes(package_root)
    with tempfile.TemporaryDirectory(prefix="arm_urdf_to_mjcf_") as temp_dir:
        temp_root = Path(temp_dir)
        (temp_root / "assets").symlink_to(package_root / "assets", target_is_directory=True)
        # The URDF importer resolves mesh basenames beside the temporary URDF.
        for mesh_path in sorted((package_root / "assets").glob("**/*.STL")):
            link = temp_root / mesh_path.name
            if link.exists():
                raise ValueError(f"duplicate mesh basename: {mesh_path.name}")
            link.symlink_to(mesh_path)
        cleaned = _clean_urdf(source, package_root)
        normalized_urdf = temp_root / "input.urdf"
        cleaned.write(normalized_urdf, encoding="utf-8", xml_declaration=True)
        model = mujoco.load_model(normalized_urdf)
        initial_xml = temp_root / "initial.xml"
        mujoco.mj_saveLastXML(str(initial_xml), model)
        _postprocess_mjcf(initial_xml, output, package_root)
        _postprocess_mjcf(initial_xml, mobile_output, package_root, dynamic_table=True,
                          table_pos=STAGE1_TABLE_POS, table_quat=STAGE1_TABLE_QUAT,
                          stage1_collision=True, model_name="rml63ii_eg2_4c2_mobile_stage1")
    _write_scene(scene_output, os.path.relpath(output, scene_output.parent), "rml63ii_d435_scene")
    mujoco.load_model(mobile_output)
    mujoco.load_model(output)
    mujoco.load_model(scene_output)


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    here = Path(__file__).resolve().parents[1]
    parser.add_argument("--source", type=Path, default=here / "model_source/RML63II-6F-V-EG2-4C2.urdf")
    parser.add_argument("--package-root", type=Path, default=here)
    parser.add_argument("--output", type=Path, default=here / "model_mjcf/rml63ii_eg2_4c2.xml")
    parser.add_argument("--scene-output", type=Path, default=here / "model_mjcf/scene.xml")
    args = parser.parse_args()
    # Keep conversion imports local to this extracted package.
    sys.path.insert(0, str(here / "mujoco"))
    try:
        convert(args.source, args.package_root, args.output, args.scene_output)
        print(f"wrote {args.output}")
        print(f"wrote {args.scene_output}")
        return 0
    except (ImportError, OSError, ET.ParseError, RuntimeError, ValueError) as exc:
        print(f"urdf_to_mjcf: {exc}", file=sys.stderr)
        return 2


if __name__ == "__main__":
    sys.exit(main())
