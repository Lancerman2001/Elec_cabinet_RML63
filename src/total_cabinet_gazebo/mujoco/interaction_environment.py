"""Robot control and the original complete-cabinet passive mechanisms.

The scene, component coefficients, latch logic and electrical indicators are
retained from cabinet_sim_ws. Control is deliberately limited to robot commands
and explicit setup operations; this module has no recording or policy code.
"""
from __future__ import annotations

import math
from pathlib import Path
import sys

import numpy as np

ROOT = Path(__file__).resolve().parents[1]
ROBOT_RUNTIME = ROOT.parent / "rml63ii_eg2_4c2_gazebo/mujoco"
if str(ROBOT_RUNTIME) not in sys.path:
    sys.path.insert(0, str(ROBOT_RUNTIME))

from environment import ArmEnv, HOME_JOINTS, HOME_GRIPPER, mj
from cabinet_interaction import CABINET_JOINTS, DOOR_OPEN, HANDLE_TURNED, HANDLE_POPPED
from interaction_backend import TotalCabinetInteraction


class TotalCabinetEnv(ArmEnv):
    """ArmEnv plus all 40 components, cabinet door and movable table.

    ``step`` has the same seven-value action contract as ArmEnv. Cabinet joints
    respond to robot contacts and the original springs/interlocks. Door and cart
    placement methods are explicit setup operations, without advancing time.
    """

    def __init__(self, xml_path=None, **kwargs):
        self.cabinet_interaction = None
        self.paused = False
        self._cart_locked = True
        self._cart_pose = None
        super().__init__(xml_path or ROOT / "mujoco/robot_cabinet_scene_glass_door.xml", **kwargs)
        self.mujoco = mj
        cart_joint = self._id(mj.mjtObj.mjOBJ_JOINT, "mobile_table_freejoint")
        self._cart_qpos = int(self.model.jnt_qposadr[cart_joint])
        self._cart_dof = int(self.model.jnt_dofadr[cart_joint])
        self._cart_weld = self._id(mj.mjtObj.mjOBJ_EQUALITY, "cart_lock_weld")
        anchor = self._id(mj.mjtObj.mjOBJ_BODY, "cart_lock_anchor")
        self._cart_mocap = int(self.model.body_mocapid[anchor])
        self.cabinet_interaction = TotalCabinetInteraction(mj, self.model, self.data)
        self._preview_data = mj.MjData(self.model)
        self._mobile_geoms = self._descendant_geoms("mobile_table")
        self._cabinet_geoms = self._descendant_geoms("cabinet_base_link")
        self.reset()

    def _reset_physics(self):
        key = mj.mj_name2id(self.model, mj.mjtObj.mjOBJ_KEY, "initial_open")
        if key < 0:
            raise ValueError("complete cabinet scene is missing initial_open keyframe")
        mj.mj_resetDataKeyframe(self.model, self.data, key)

    def reset(self, joints=HOME_JOINTS, gripper=HOME_GRIPPER, door_state="open"):
        if door_state not in ("open", "closed", "door_returned", "latched_handle_out"):
            raise ValueError("invalid cabinet door reset state")
        super().reset(joints, gripper)
        if self.cabinet_interaction is not None:
            self.cabinet_interaction.reset(door_state)
            self._cart_pose = self.data.qpos[self._cart_qpos:self._cart_qpos + 7].copy()
            self._cart_locked = True
            self._pin_cart()
            mj.mj_forward(self.model, self.data)
            self.cabinet_interaction.post_step()
        return self.observation()

    def _descendant_geoms(self, name):
        ancestor = self._id(mj.mjtObj.mjOBJ_BODY, name)
        result = set()
        for geom, initial_body in enumerate(self.model.geom_bodyid):
            body = int(initial_body)
            while body > 0 and body != ancestor:
                body = int(self.model.body_parentid[body])
            if body == ancestor:
                result.add(geom)
        return result

    def _pin_cart(self):
        if self._cart_pose is None:
            return
        mj.equality_active(self.model, self.data)[self._cart_weld] = self._cart_locked
        if self._cart_locked:
            self.data.mocap_pos[self._cart_mocap] = self._cart_pose[:3]
            self.data.mocap_quat[self._cart_mocap] = self._cart_pose[3:]
            self.data.qpos[self._cart_qpos:self._cart_qpos + 7] = self._cart_pose
            self.data.qvel[self._cart_dof:self._cart_dof + 6] = 0.0

    def _before_physics_step(self):
        self._pin_cart()
        self.cabinet_interaction.pre_step()

    def _after_physics_step(self):
        self._pin_cart()
        self._refresh_geometry(self.data)
        self.cabinet_interaction.post_step()

    def _refresh_geometry(self, data):
        mj.mj_kinematics(self.model, data)
        mj.mj_comPos(self.model, data)
        mj.mj_comVel(self.model, data)

    def step(self, action=None):
        return self.observation() if self.paused else super().step(action)

    def set_paused(self, paused):
        self.paused = bool(paused)
        return self.paused

    def set_cart_fixed(self, fixed=True):
        self._cart_locked = bool(fixed)
        if self._cart_locked:
            self._cart_pose = self.data.qpos[self._cart_qpos:self._cart_qpos + 7].copy()
        self._pin_cart()
        mj.mj_forward(self.model, self.data)
        return self.table_state()

    def table_state(self):
        pose = self.data.qpos[self._cart_qpos:self._cart_qpos + 7].copy()
        yaw = 2.0 * math.atan2(float(pose[6]), float(pose[3]))
        return {"pose": pose, "x": float(pose[0]), "y": float(pose[1]), "yaw": yaw,
                "cart_fixed": self._cart_locked}

    def _placement_collision(self, preview, first_geoms, second_geoms):
        mj.mj_forward(self.model, preview)
        for contact in preview.contact:
            first, second = int(contact.geom1), int(contact.geom2)
            if contact.dist < -0.001 and (
                (first in first_geoms and second in second_geoms)
                or (second in first_geoms and first in second_geoms)
            ):
                return {"accepted": False, "reason": "setup pose intersects robot or cabinet",
                        "penetration_m": -float(contact.dist)}
        return None

    def set_table_pose(self, x, y, yaw):
        values = np.asarray([x, y, yaw], dtype=float)
        if not np.isfinite(values).all():
            raise ValueError("table pose must be finite")
        x, y, yaw = values
        if abs(x) > 2.5 or abs(y) > 2.5:
            raise ValueError("table pose must stay within the scene floor")
        pose = np.array([x, y, 0.0, math.cos(yaw / 2), 0.0, 0.0, math.sin(yaw / 2)])
        probe = self._preview_data
        mj.mj_copyData(probe, self.model, self.data)
        probe.qpos[self._cart_qpos:self._cart_qpos + 7] = pose
        rejection = self._placement_collision(probe, self._mobile_geoms, self._cabinet_geoms)
        if rejection is not None:
            return rejection
        self.data.qpos[self._cart_qpos:self._cart_qpos + 7] = pose
        self.data.qvel[self._cart_dof:self._cart_dof + 6] = 0.0
        self._cart_pose = pose
        self._pin_cart()
        mj.mj_forward(self.model, self.data)
        return {"accepted": True, **self.table_state()}

    def set_door_state(self, state):
        if state not in ("open", "closed"):
            raise ValueError("door state must be open or closed")
        door = self.cabinet_interaction.door
        probe = self._preview_data
        mj.mj_copyData(probe, self.model, self.data)
        values = (DOOR_OPEN, 0.0, HANDLE_TURNED, HANDLE_POPPED) if state == "open" else (0.0,) * 4
        for name, value in zip(CABINET_JOINTS, values):
            probe.qpos[door.qpos_addresses[name]] = value
        door_geoms = set(door.door_geoms) | set(door.handle_geoms) | set(door.button_geoms)
        rejection = self._placement_collision(probe, door_geoms, self._mobile_geoms)
        if rejection is not None:
            return rejection
        door.reset(state)
        door_dofs = list(door.dof_addresses.values())
        self.data.qfrc_applied[door_dofs] = 0.0
        self.data.qacc_warmstart[door_dofs] = 0.0
        for body in range(self.model.nbody):
            ancestor = body
            while ancestor > 0 and ancestor != door.door_body_id:
                ancestor = int(self.model.body_parentid[ancestor])
            if ancestor == door.door_body_id:
                self.data.xfrc_applied[body] = 0.0
        mj.mj_forward(self.model, self.data)
        door.post_step()
        return {"accepted": True, "state": door.state_name()}

    def component_state(self):
        return self.cabinet_interaction.component_state()

    def component_contacts(self):
        return self.cabinet_interaction.component_contacts()

    def observation(self):
        result = super().observation()
        if self.cabinet_interaction is not None:
            result.update(door=self.cabinet_interaction.door.snapshot(),
                          components=self.component_state(),
                          component_contacts=self.component_contacts(),
                          table=self.table_state(), paused=self.paused)
        return result
