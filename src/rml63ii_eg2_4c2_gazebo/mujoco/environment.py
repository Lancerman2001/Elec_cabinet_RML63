"""Small, ROS-free control API for the RML63II arm and EG2-4C2 gripper.

Joint commands are radians. The seventh action value is gripper closedness:
0 means open and 1 means closed. Observations contain measured simulation state.
"""
from __future__ import annotations

import math
from pathlib import Path

import numpy as np

try:
    from . import mujoco_compat as mj
except ImportError:
    import mujoco_compat as mj


ARM_JOINTS = tuple(f"joint_{index}" for index in range(1, 7))
GRIPPER_JOINTS = (
    ("gripper_joint1", 1), ("gripper_joint5", 1), ("gripper_joint2", 1),
    ("gripper_joint3", -1), ("gripper_joint4", -1), ("gripper_joint6", 1),
)
HOME_JOINTS = (-0.605, 0.085, 0.986, 1.294, 1.585, 0.420)
GRIPPER_MAX_ANGLE = 0.82
HOME_GRIPPER = 1.0 - 0.01 / GRIPPER_MAX_ANGLE


def _vector(value, size, name):
    result = np.asarray(value, dtype=float)
    if result.shape != (size,) or not np.isfinite(result).all():
        raise ValueError(f"{name} must contain {size} finite numbers")
    return result


def _closedness(value):
    result = float(value)
    if not math.isfinite(result):
        raise ValueError("gripper must be finite")
    return float(np.clip(result, 0.0, 1.0))


def _rpy_matrix(value):
    roll, pitch, yaw = value
    cr, sr, cp, sp, cy, sy = (
        math.cos(roll), math.sin(roll), math.cos(pitch), math.sin(pitch),
        math.cos(yaw), math.sin(yaw),
    )
    return np.array([
        [cy * cp, cy * sp * sr - sy * cr, cy * sp * cr + sy * sr],
        [sy * cp, sy * sp * sr + cy * cr, sy * sp * cr - cy * sr],
        [-sp, cp * sr, cp * cr],
    ])


class ArmEnv:
    """Seven-action position control, Cartesian jogging, and wrist rendering.

    ``reset()`` and ``step()`` return an observation dictionary. ``step(action)``
    accepts six absolute joint targets plus normalized gripper closedness;
    ``step()`` without an action advances the current targets by ``control_dt``.
    The TCP defaults to the gripper mounting origin (``stage2_tcp``). Set
    ``tcp_offset`` to a calibrated local offset when controlling a tool tip.
    """

    def __init__(self, xml_path=None, control_dt=0.02, gravity_compensation=True,
                 tcp_site="stage2_tcp", tcp_offset=None):
        package = Path(__file__).resolve().parents[1]
        self.xml_path = Path(xml_path or package / "model_mjcf/scene.xml").resolve()
        self.model = mj.load_model(self.xml_path)
        self.data = mj.MjData(self.model)
        self._ik_data = mj.MjData(self.model)
        self._gravity_data = mj.MjData(self.model)
        self.control_dt = float(control_dt)
        if not math.isfinite(self.control_dt) or self.control_dt < self.model.opt.timestep:
            raise ValueError("control_dt must be finite and at least the physics timestep")
        self.gravity_compensation = bool(gravity_compensation)
        self._joint_ids = np.array([self._id(mj.mjtObj.mjOBJ_JOINT, n) for n in ARM_JOINTS])
        self._qpos = self.model.jnt_qposadr[self._joint_ids]
        self._dofs = self.model.jnt_dofadr[self._joint_ids]
        self._actuators = np.array([
            self._id(mj.mjtObj.mjOBJ_ACTUATOR, "act_" + n) for n in ARM_JOINTS
        ])
        self._gripper_actuator = self._id(mj.mjtObj.mjOBJ_ACTUATOR, "act_gripper")
        self._gripper_qpos = self.model.jnt_qposadr[
            self._id(mj.mjtObj.mjOBJ_JOINT, "gripper_joint1")
        ]
        self.joint_limits = np.column_stack((
            np.maximum(self.model.jnt_range[self._joint_ids, 0],
                       self.model.actuator_ctrlrange[self._actuators, 0]),
            np.minimum(self.model.jnt_range[self._joint_ids, 1],
                       self.model.actuator_ctrlrange[self._actuators, 1]),
        ))
        self.tcp_site_id = self._id(mj.mjtObj.mjOBJ_SITE, tcp_site)
        if tcp_offset is not None:
            self.model.site_pos[self.tcp_site_id] = _vector(tcp_offset, 3, "tcp_offset")
        self._renderers = {}
        self.reset()

    def _id(self, kind, name):
        result = mj.mj_name2id(self.model, kind, name)
        if result < 0:
            raise ValueError(f"model is missing {name!r}")
        return result

    def set_joint_targets(self, joints, gripper=None):
        self.joint_target = np.clip(_vector(joints, 6, "joints"),
                                    self.joint_limits[:, 0], self.joint_limits[:, 1])
        if gripper is not None:
            self.gripper_target = _closedness(gripper)

    def set_gripper(self, closedness):
        self.gripper_target = _closedness(closedness)

    def reset(self, joints=HOME_JOINTS, gripper=HOME_GRIPPER):
        """Reset physics and place the robot at the requested starting pose."""
        self._reset_physics()
        self.set_joint_targets(joints, gripper)
        self.data.qpos[self._qpos] = self.joint_target
        angle = (1.0 - self.gripper_target) * GRIPPER_MAX_ANGLE
        for name, multiplier in GRIPPER_JOINTS:
            joint = self._id(mj.mjtObj.mjOBJ_JOINT, name)
            self.data.qpos[self.model.jnt_qposadr[joint]] = multiplier * angle
        self._reference = np.r_[self.joint_target, angle]
        self._remainder = 0.0
        self.data.ctrl[self._actuators] = self.joint_target
        self.data.ctrl[self._gripper_actuator] = angle
        mj.mj_forward(self.model, self.data)
        return self.observation()

    def _reset_physics(self):
        mj.mj_resetData(self.model, self.data)

    def step(self, action=None):
        """Advance one control interval; limit target speed to 0.8 rad/s."""
        if action is not None:
            action = _vector(action, 7, "action")
            self.set_joint_targets(action[:6], action[6])
        self._remainder += self.control_dt
        dt = float(self.model.opt.timestep)
        substeps = int((self._remainder + 1e-12) / dt)
        self._remainder -= substeps * dt
        target = np.r_[self.joint_target, (1.0 - self.gripper_target) * GRIPPER_MAX_ANGLE]
        for _ in range(substeps):
            self._before_physics_step()
            self._reference += np.clip(target - self._reference, -0.8 * dt, 0.8 * dt)
            self.data.ctrl[self._actuators] = self._reference[:6]
            self.data.ctrl[self._gripper_actuator] = self._reference[6]
            if self.gravity_compensation:
                gd = self._gravity_data
                gd.qpos[:] = self.data.qpos
                gd.qvel[:] = 0.0
                mj.mj_kinematics(self.model, gd)
                mj.mj_comPos(self.model, gd)
                mj.mj_comVel(self.model, gd)
                gravity = np.zeros(self.model.nv)
                mj.mj_rne(self.model, gd, 0, gravity)
                # Feedforward stays inside actuator force limits.
                self.model.actuator_biasprm[self._actuators, 0] = gravity[self._dofs]
            else:
                self.model.actuator_biasprm[self._actuators, 0] = 0.0
            mj.mj_step(self.model, self.data)
            self._after_physics_step()
        self._refresh_geometry(self.data)
        if not np.isfinite(self.data.qpos).all():
            raise RuntimeError("simulation state became non-finite")
        return self.observation()

    def _before_physics_step(self):
        pass

    def _after_physics_step(self):
        pass

    def _refresh_geometry(self, data):
        mj.mj_forward(self.model, data)

    def observation(self):
        quaternion = np.zeros(4)
        mj.mju_mat2Quat(quaternion, self.data.site_xmat[self.tcp_site_id])
        return {
            "time": float(self.data.time),
            "joint_position": self.data.qpos[self._qpos].copy(),
            "joint_velocity": self.data.qvel[self._dofs].copy(),
            "joint_target": self.joint_target.copy(),
            "gripper": float(np.clip(1.0 - self.data.qpos[self._gripper_qpos] /
                                     GRIPPER_MAX_ANGLE, 0.0, 1.0)),
            "gripper_target": self.gripper_target,
            "tcp_position": self.data.site_xpos[self.tcp_site_id].copy(),
            "tcp_quaternion": quaternion,
        }

    def jog_cartesian(self, delta_xyz, delta_rpy=(0.0, 0.0, 0.0)):
        """Request a world-frame TCP increment in meters and radians.

        Damped least-squares IK preserves orientation for translation-only
        commands. Failed requests leave the previous joint target unchanged.
        This is a kinematic jog interface; it does not plan around collisions.
        """
        translation = _vector(delta_xyz, 3, "delta_xyz")
        rotation = _rpy_matrix(_vector(delta_rpy, 3, "delta_rpy"))
        data = self._ik_data
        data.qpos[:] = self.data.qpos
        data.qvel[:] = 0.0
        q = self.joint_target.copy()
        data.qpos[self._qpos] = q
        self._refresh_ik_geometry(data)
        position_goal = data.site_xpos[self.tcp_site_id].copy() + translation
        rotation_goal = rotation @ data.site_xmat[self.tcp_site_id].reshape(3, 3)
        jacp, jacr = np.zeros((3, self.model.nv)), np.zeros((3, self.model.nv))
        position_error = orientation_error = math.inf
        for _ in range(100):
            data.qpos[self._qpos] = q
            self._refresh_ik_geometry(data)
            quaternion, angular_error = np.zeros(4), np.zeros(3)
            rotation_error = rotation_goal @ data.site_xmat[self.tcp_site_id].reshape(3, 3).T
            mj.mju_mat2Quat(quaternion, rotation_error.reshape(9))
            if quaternion[0] < 0.0:
                quaternion *= -1.0
            mj.mju_quat2Vel(angular_error, quaternion, 1.0)
            error = np.r_[position_goal - data.site_xpos[self.tcp_site_id], angular_error]
            position_error, orientation_error = np.linalg.norm(error[:3]), np.linalg.norm(error[3:])
            if position_error <= 0.0005 and orientation_error <= 0.0088:
                self.set_joint_targets(q)
                return {"accepted": True, "position_error": float(position_error),
                        "orientation_error": float(orientation_error)}
            mj.mj_jacSite(self.model, data, jacp, jacr, self.tcp_site_id)
            jacobian = np.vstack((jacp[:, self._dofs], jacr[:, self._dofs]))
            delta = jacobian.T @ np.linalg.solve(
                jacobian @ jacobian.T + 0.04 ** 2 * np.eye(6), error)
            q = np.clip(q + np.clip(delta, -0.2, 0.2),
                        self.joint_limits[:, 0], self.joint_limits[:, 1])
        return {"accepted": False, "position_error": float(position_error),
                "orientation_error": float(orientation_error)}

    def _refresh_ik_geometry(self, data):
        mj.mj_kinematics(self.model, data)
        mj.mj_comPos(self.model, data)

    def render(self, camera="d435_rgb", depth=None, width=640, height=480):
        """Return RGB uint8 (H,W,3) or optical-axis depth float32 (H,W) in meters.

        Named ``*_depth`` cameras default to depth; other cameras default to
        RGB. An explicit ``depth=True/False`` overrides this selection. Depth
        is linear engine output, including the far clipping plane background,
        without display normalization or a physical D435 noise model.
        """
        self._id(mj.mjtObj.mjOBJ_CAMERA, camera)
        if depth is None:
            depth = camera.endswith("_depth")
        key = (int(width), int(height))
        if key not in self._renderers:
            self._renderers[key] = mj.Renderer(self.model, height=key[1], width=key[0])
        renderer = self._renderers[key]
        if depth:
            renderer.enable_depth_rendering()
        else:
            renderer.disable_depth_rendering()
        renderer.update_scene(self.data, camera=camera)
        return renderer.render().copy()

    def close(self):
        for renderer in self._renderers.values():
            renderer.close()
        self._renderers.clear()

    def __enter__(self):
        return self

    def __exit__(self, *_):
        self.close()


RML63IIEnv = ArmEnv
