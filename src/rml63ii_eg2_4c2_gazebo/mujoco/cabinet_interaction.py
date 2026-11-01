"""Contact-driven MuJoCo interaction model for the electrical cabinet.

The cabinet is intentionally controlled with generalized spring forces rather
than position writes.  The robot therefore remains a real collision body and
the door can either move under the spring torque or stop against the robot.
This module has no ROS dependency and is shared by the environment and the
stage-3 viewer.
"""

from __future__ import annotations

import math
from dataclasses import dataclass

import numpy as np


DOOR_JOINT = "door_hinge_joint"
BUTTON_JOINT = "button_press_joint"
HANDLE_TURN_JOINT = "handle_turn_joint"
HANDLE_POP_JOINT = "handle_pop_joint"
CABINET_JOINTS = (DOOR_JOINT, BUTTON_JOINT, HANDLE_TURN_JOINT, HANDLE_POP_JOINT)

DOOR_OPEN = math.pi / 2.0
HANDLE_TURNED = math.pi / 2.0
HANDLE_POPPED = -math.pi / 3.0


@dataclass
class CabinetInteractionConfig:
    """Tunable first-pass physical parameters in SI units."""

    button_trigger_depth: float = 0.002
    button_trigger_force: float = 0.5
    button_return_stiffness: float = 50.0
    button_return_damping: float = 0.5
    button_force_limit: float = 5.0

    handle_pop_target: float = HANDLE_POPPED
    handle_pop_trigger_angle: float = math.radians(-45.0)
    handle_pop_stiffness: float = 0.8
    handle_pop_damping: float = 0.08
    handle_pop_force_limit: float = 0.5

    handle_turn_trigger_angle: float = math.radians(80.0)
    handle_turn_lock_stiffness: float = 4.0
    handle_turn_lock_damping: float = 0.12
    handle_turn_force_limit: float = 8.0

    door_open_target: float = DOOR_OPEN
    # Light spring-assisted cabinet door: at zero angle the released spring
    # produces 2.36 Nm (~3.4 N at the outer edge), not the previous 28.3 Nm.
    # Damping acts implicitly in the hinge, independently of surface friction.
    door_open_stiffness: float = 1.5
    door_open_damping: float = 3.0
    door_open_force_limit: float = 3.0
    door_release_duration: float = 0.3
    door_latch_stiffness: float = 50.0
    door_latch_damping: float = 2.0
    door_latch_force_limit: float = 60.0

    door_closed_tolerance: float = math.radians(5.0)
    handle_latch_tolerance: float = math.radians(5.0)
    event_contact_epsilon: float = 1.0e-9

    def __post_init__(self):
        for name, value in vars(self).items():
            if not math.isfinite(float(value)):
                raise ValueError("cabinet interaction parameter {} must be finite".format(name))
        if self.button_trigger_depth <= 0.0:
            raise ValueError("button_trigger_depth must be positive")
        if self.button_trigger_force < 0.0:
            raise ValueError("button_trigger_force must be non-negative")
        if self.door_closed_tolerance <= 0.0 or self.handle_latch_tolerance <= 0.0:
            raise ValueError("cabinet angle tolerances must be positive")
        for name, value in vars(self).items():
            if any(word in name for word in ("stiffness", "damping", "force_limit", "duration")) and value < 0:
                raise ValueError("{} must be non-negative".format(name))


class CabinetInteraction:
    """Stateful spring, latch, and contact logic for one MuJoCo cabinet."""

    VALID_RESET_STATES = (
        "closed",
        "open",
        "door_returned",
        "latched_handle_out",
    )

    def __init__(self, mujoco, model, data, config=None):
        self.mujoco = mujoco
        self.model = model
        self.data = data
        self.config = config or CabinetInteractionConfig()
        self.joint_ids = {
            name: mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_JOINT, name)
            for name in CABINET_JOINTS
        }
        if any(identifier < 0 for identifier in self.joint_ids.values()):
            raise RuntimeError("MuJoCo model is missing cabinet interaction joints")
        self.qpos_addresses = {
            name: int(model.jnt_qposadr[joint_id])
            for name, joint_id in self.joint_ids.items()
        }
        self.dof_addresses = {
            name: int(model.jnt_dofadr[joint_id])
            for name, joint_id in self.joint_ids.items()
        }
        # Native damping is integrated implicitly and must not also be added
        # to qfrc_applied. Keep configuration as the single source of truth.
        model.dof_damping[self.dof_addresses[BUTTON_JOINT]] = self.config.button_return_damping
        model.dof_damping[self.dof_addresses[HANDLE_POP_JOINT]] = self.config.handle_pop_damping

        self.cabinet_body_id = mujoco.mj_name2id(
            model, mujoco.mjtObj.mjOBJ_BODY, "cabinet_base_link"
        )
        self.arm_body_id = mujoco.mj_name2id(
            model, mujoco.mjtObj.mjOBJ_BODY, "base_link"
        )
        self.button_sensor_id = mujoco.mj_name2id(
            model, mujoco.mjtObj.mjOBJ_SENSOR, "button_contact_sensor"
        )
        self.door_body_id = mujoco.mj_name2id(
            model, mujoco.mjtObj.mjOBJ_BODY, "door_link"
        )
        self.handle_body_id = mujoco.mj_name2id(
            model, mujoco.mjtObj.mjOBJ_BODY, "door_handle_link"
        )
        self.button_body_id = mujoco.mj_name2id(
            model, mujoco.mjtObj.mjOBJ_BODY, "lock_button_link"
        )

        self.robot_geoms = self._descendant_geoms(self.arm_body_id)
        self.door_geoms = self._descendant_geoms(self.door_body_id)
        self.handle_geoms = self._descendant_geoms(self.handle_body_id)
        self.button_geoms = self._descendant_geoms(self.button_body_id)
        self._last_event = "reset"
        self._event_counter = 0
        self._button_triggered = False
        self._handle_popped = False
        self._handle_has_reached_pop = False
        self._door_unlocked = False
        self._door_latched = True
        self._door_blocked = False
        self._contact_flags = {"button": False, "handle": False, "door": False}
        self._spring_torque = np.zeros(4, dtype=np.float64)
        self.reset("closed")

    def _is_descendant(self, body_id, ancestor_id):
        if body_id < 0 or ancestor_id < 0:
            return False
        while body_id > 0:
            if body_id == ancestor_id:
                return True
            body_id = int(self.model.body_parentid[body_id])
        return body_id == ancestor_id

    def _descendant_geoms(self, ancestor_id):
        if ancestor_id < 0:
            return set()
        return {
            index
            for index in range(self.model.ngeom)
            if self.model.geom_contype[index]
            and self.model.geom_conaffinity[index]
            and self._is_descendant(int(self.model.geom_bodyid[index]), ancestor_id)
        }

    def _joint_value(self, name):
        return float(self.data.qpos[self.qpos_addresses[name]])

    def _joint_velocity(self, name):
        return float(self.data.qvel[self.dof_addresses[name]])

    def _set_joint_value(self, name, value):
        self.data.qpos[self.qpos_addresses[name]] = float(value)

    def _set_joint_velocity(self, name, value=0.0):
        self.data.qvel[self.dof_addresses[name]] = float(value)

    def _body_has_robot_contact(self, geoms):
        if not geoms or not self.robot_geoms:
            return False
        for index in range(int(self.data.ncon)):
            contact = self.data.contact[index]
            first = int(contact.geom1)
            second = int(contact.geom2)
            if (first in geoms and second in self.robot_geoms) or (
                second in geoms and first in self.robot_geoms
            ):
                if float(contact.dist) <= self.config.event_contact_epsilon:
                    return True
        return False

    def _contacts(self):
        flags = {
            "button": self._body_has_robot_contact(self.button_geoms),
            "handle": self._body_has_robot_contact(self.handle_geoms),
            "door": self._body_has_robot_contact(self.door_geoms),
        }
        return flags

    def _button_sensor_value(self):
        if self.button_sensor_id < 0:
            return 0.0
        return float(self.data.sensordata[self.button_sensor_id])

    def _emit(self, event):
        if event:
            self._last_event = str(event)
            self._event_counter += 1

    def reset(self, state="closed"):
        state = str(state).lower().strip()
        if state not in self.VALID_RESET_STATES:
            raise ValueError(
                "cabinet_state must be one of {}".format(self.VALID_RESET_STATES)
            )
        poses = {
            "closed": (0.0, 0.0, 0.0, 0.0),
            "open": (DOOR_OPEN, 0.0, HANDLE_TURNED, HANDLE_POPPED),
            "door_returned": (0.0, 0.0, HANDLE_TURNED, HANDLE_POPPED),
            "latched_handle_out": (0.0, 0.0, 0.0, HANDLE_POPPED),
        }
        for name, value in zip(CABINET_JOINTS, poses[state]):
            self._set_joint_value(name, value)
            self._set_joint_velocity(name, 0.0)
        self._button_triggered = state != "closed"
        self._handle_popped = state != "closed"
        self._handle_has_reached_pop = state != "closed"
        self._door_unlocked = state in ("open", "door_returned")
        self._door_latched = state in ("closed", "latched_handle_out")
        self._door_release_time = float(self.data.time) if self._door_unlocked else None
        self._door_blocked = False
        self._last_event = "reset_{}".format(state)
        self._event_counter += 1
        self._spring_torque[:] = 0.0
        self._contact_flags = {"button": False, "handle": False, "door": False}

    def _spring(self, q, velocity, target, stiffness, damping, force_limit):
        value = stiffness * (target - q) - damping * velocity
        return float(np.clip(value, -force_limit, force_limit))

    def _update_state(self):
        cfg = self.config
        button = self._joint_value(BUTTON_JOINT)
        door = self._joint_value(DOOR_JOINT)
        turn = self._joint_value(HANDLE_TURN_JOINT)
        pop = self._joint_value(HANDLE_POP_JOINT)
        self._contact_flags = self._contacts()

        button_pressed = self._contact_flags["button"] and (
            -button >= cfg.button_trigger_depth
            or self._button_sensor_value() >= cfg.button_trigger_force
        )
        if (
            not self._button_triggered
            and button_pressed
            and door <= cfg.door_closed_tolerance
            and abs(turn) <= cfg.handle_latch_tolerance
        ):
            self._button_triggered = True
            self._handle_popped = True
            self._emit("button_pressed_handle_pop_triggered")

        # Turning is accepted only after the pop axis has physically reached
        # its trigger angle.  This prevents a same-step turn command from
        # bypassing the spring-driven handle pop.
        if (
            self._handle_popped
            and pop <= cfg.handle_pop_trigger_angle
            and turn >= cfg.handle_turn_trigger_angle
        ):
            if not self._door_unlocked:
                self._emit("handle_turned_door_open_triggered")
                self._door_release_time = float(self.data.time)
            self._door_unlocked = True
            self._door_latched = False

        # A door can be caught only after it is physically near closed and the
        # turn handle has returned.  If it is closed while the handle remains
        # turned, the open spring remains active and pushes it out again.
        if (
            self._door_unlocked
            and door <= cfg.door_closed_tolerance
            and turn <= cfg.handle_latch_tolerance
        ):
            if not self._door_latched:
                self._emit("door_caught_handle_returned")
            self._door_latched = True

        # The outward handle spring stays active until the robot physically
        # pushes the handle back into the lock after the door is caught.
        if (
            self._handle_popped
            and self._handle_has_reached_pop
            and self._door_latched
            and door <= cfg.door_closed_tolerance
            and turn <= cfg.handle_latch_tolerance
            and pop >= -cfg.handle_latch_tolerance
        ):
            self._handle_popped = False
            self._handle_has_reached_pop = False
            self._button_triggered = False
            self._door_unlocked = False
            self._emit("handle_latched_closed")

        self._door_blocked = bool(
            self._contact_flags["door"]
            and not self._door_latched
            and abs(self._joint_velocity(DOOR_JOINT)) < 0.05
        )
        if self._handle_popped and pop <= cfg.handle_pop_trigger_angle:
            self._handle_has_reached_pop = True

    def pre_step(self):
        """Update triggers and apply one substep of passive generalized forces."""
        self._update_state()
        cfg = self.config
        door = self._joint_value(DOOR_JOINT)
        button = self._joint_value(BUTTON_JOINT)
        turn = self._joint_value(HANDLE_TURN_JOINT)
        pop = self._joint_value(HANDLE_POP_JOINT)

        forces = {
            DOOR_JOINT: 0.0,
            BUTTON_JOINT: self._spring(
                button,
                self._joint_velocity(BUTTON_JOINT),
                0.0,
                cfg.button_return_stiffness,
                0.0,
                cfg.button_force_limit,
            ),
            HANDLE_TURN_JOINT: 0.0,
            HANDLE_POP_JOINT: 0.0,
        }
        if not self._handle_popped:
            forces[HANDLE_POP_JOINT] = self._spring(
                pop,
                self._joint_velocity(HANDLE_POP_JOINT),
                0.0,
                cfg.handle_pop_stiffness,
                0.0,
                cfg.handle_pop_force_limit,
            )
        else:
            # The popped handle has no preferred turn angle. Native MJCF
            # joint friction holds it against gravity at intermediate angles,
            # and implicit joint damping dissipates motion in both directions.
            # A 0/90-degree target spring would oppose opening/returning the
            # handle and snap it back as soon as the gripper loses contact.
            forces[HANDLE_POP_JOINT] = self._spring(
                pop,
                self._joint_velocity(HANDLE_POP_JOINT),
                cfg.handle_pop_target,
                cfg.handle_pop_stiffness,
                0.0,
                cfg.handle_pop_force_limit,
            )

        if not self._handle_popped:
            forces[HANDLE_TURN_JOINT] = self._spring(
                turn,
                self._joint_velocity(HANDLE_TURN_JOINT),
                0.0,
                cfg.handle_turn_lock_stiffness,
                cfg.handle_turn_lock_damping,
                cfg.handle_turn_force_limit,
            )

        if self._door_latched or not self._door_unlocked:
            self.model.dof_damping[self.dof_addresses[DOOR_JOINT]] = cfg.door_latch_damping
            forces[DOOR_JOINT] = self._spring(
                door,
                self._joint_velocity(DOOR_JOINT),
                0.0,
                cfg.door_latch_stiffness,
                0.0,
                cfg.door_latch_force_limit,
            )
        else:
            self.model.dof_damping[self.dof_addresses[DOOR_JOINT]] = cfg.door_open_damping
            elapsed = max(0.0, float(self.data.time) - float(self._door_release_time or 0.0))
            progress = 1.0 if cfg.door_release_duration == 0 else min(1.0, elapsed/cfg.door_release_duration)
            release = progress*progress*(3.0-2.0*progress)
            forces[DOOR_JOINT] = self._spring(
                door,
                self._joint_velocity(DOOR_JOINT),
                cfg.door_open_target,
                cfg.door_open_stiffness * release,
                0.0,
                cfg.door_open_force_limit,
            )

        for name, force in forces.items():
            self.data.qfrc_applied[self.dof_addresses[name]] = force
        self._spring_torque[:] = [
            forces[DOOR_JOINT],
            forces[BUTTON_JOINT],
            forces[HANDLE_TURN_JOINT],
            forces[HANDLE_POP_JOINT],
        ]

    def post_step(self):
        self._update_state()

    def snapshot(self):
        self._update_state()
        values = np.asarray(
            [self._joint_value(name) for name in CABINET_JOINTS], dtype=np.float64
        )
        return {
            "state": self.state_name(),
            "joint_position": values.copy(),
            "button_triggered": bool(self._button_triggered),
            "handle_popped": bool(self._handle_popped),
            "handle_has_reached_pop": bool(self._handle_has_reached_pop),
            "door_unlocked": bool(self._door_unlocked),
            "door_latched": bool(self._door_latched),
            "door_blocked": bool(self._door_blocked),
            "contact_count": int(self._cabinet_contact_count()),
            "button_contact": bool(self._contact_flags["button"]),
            "handle_contact": bool(self._contact_flags["handle"]),
            "door_contact": bool(self._contact_flags["door"]),
            "spring_torque": self._spring_torque.copy(),
            "interaction_event": self._last_event,
            "interaction_event_count": int(self._event_counter),
        }

    def _cabinet_contact_count(self):
        cabinet_geoms = self.door_geoms | self.handle_geoms | self.button_geoms
        count = 0
        for index in range(int(self.data.ncon)):
            contact = self.data.contact[index]
            if int(contact.geom1) in cabinet_geoms or int(contact.geom2) in cabinet_geoms:
                if float(contact.dist) <= self.config.event_contact_epsilon:
                    count += 1
        return count

    def state_name(self):
        door = self._joint_value(DOOR_JOINT)
        turn = self._joint_value(HANDLE_TURN_JOINT)
        pop = self._joint_value(HANDLE_POP_JOINT)
        cfg = self.config
        if self._door_latched and not self._handle_popped and door <= cfg.door_closed_tolerance:
            return "locked_closed"
        if self._door_unlocked and door > cfg.door_closed_tolerance:
            return "open" if door >= cfg.door_open_target - cfg.door_closed_tolerance else "opening"
        if (
            self._door_latched
            and self._handle_popped
            and self._handle_has_reached_pop
            and door <= cfg.door_closed_tolerance
        ):
            return "closed_handle_out"
        if self._handle_popped and turn >= cfg.handle_turn_trigger_angle:
            return "unlatched_closed" if door <= cfg.door_closed_tolerance else "opening"
        if self._handle_popped or self._button_triggered or pop < -cfg.handle_latch_tolerance:
            return "handle_popped"
        return "locked_closed"
