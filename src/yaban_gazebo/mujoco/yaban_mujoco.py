#!/usr/bin/env python3
"""A small, ROS-independent MuJoCo backend for the pressure-plate MJCF."""

from __future__ import annotations

import pathlib
from typing import Optional

try:
    from yaban_mujoco_protocol import Pose, bounded_pose
except ImportError:  # pragma: no cover - package-style import in external tests
    from .yaban_mujoco_protocol import Pose, bounded_pose


MODEL_PATH = pathlib.Path(__file__).with_name("yaban_mujoco.xml")
JOINT_NAMES = ("left_knob_joint", "left_knob_lift_joint", "blade_joint")


class MuJoCoUnavailable(RuntimeError):
    """Raised with an actionable command instead of a raw ImportError."""


def require_mujoco():
    try:
        import mujoco
    except ImportError as exc:  # pragma: no cover - depends on local installation
        raise MuJoCoUnavailable(
            "未找到 MuJoCo。请运行：python3 -m pip install mujoco==3.3.7"
        ) from exc
    return mujoco


class YabanMuJoCoBackend:
    """Loads MJCF and keeps the screw lift kinematically tied to knob rotation."""

    def __init__(self, model_path: Optional[pathlib.Path] = None):
        self.mujoco = require_mujoco()
        self.model_path = pathlib.Path(model_path or MODEL_PATH).resolve()
        self.model = self.mujoco.MjModel.from_xml_path(str(self.model_path))
        self.data = self.mujoco.MjData(self.model)
        self.joint_ids = {}
        self.qpos_addresses = {}
        for name in JOINT_NAMES:
            joint_id = self.mujoco.mj_name2id(self.model, self.mujoco.mjtObj.mjOBJ_JOINT, name)
            if joint_id < 0:
                raise RuntimeError("MuJoCo model is missing joint {}".format(name))
            self.joint_ids[name] = joint_id
            self.qpos_addresses[name] = self.model.jnt_qposadr[joint_id]
        self.set_pose(Pose())

    def pose(self) -> Pose:
        return Pose(
            knob=float(self.data.qpos[self.qpos_addresses["left_knob_joint"]]),
            blade=float(self.data.qpos[self.qpos_addresses["blade_joint"]]),
        )

    def lift(self) -> float:
        return float(self.data.qpos[self.qpos_addresses["left_knob_lift_joint"]])

    def set_pose(self, pose: Pose) -> Pose:
        """Set the two task coordinates and derive the third qpos from screw pitch."""
        pose = bounded_pose(pose)
        self.data.qpos[self.qpos_addresses["left_knob_joint"]] = pose.knob
        self.data.qpos[self.qpos_addresses["left_knob_lift_joint"]] = pose.lift
        self.data.qpos[self.qpos_addresses["blade_joint"]] = pose.blade
        self.data.qvel[:] = 0.0
        self.mujoco.mj_forward(self.model, self.data)
        return pose

    def reset_keyframe(self, key_name: str = "initial") -> Pose:
        key_id = self.mujoco.mj_name2id(self.model, self.mujoco.mjtObj.mjOBJ_KEY, key_name)
        if key_id < 0:
            raise ValueError("MuJoCo model is missing keyframe {}".format(key_name))
        self.mujoco.mj_resetDataKeyframe(self.model, self.data, key_id)
        self.mujoco.mj_forward(self.model, self.data)
        # Keep the controller's screw relation authoritative if a keyframe is edited.
        return self.set_pose(self.pose())
