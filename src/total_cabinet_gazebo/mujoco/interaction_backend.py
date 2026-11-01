"""Passive mechanisms for the robot and complete-cabinet MuJoCo scene.

The host owns mj_step. This module never prescribes cabinet motion: positions
and velocities are assigned only by reset; runtime changes are mechanism
forces, screw/latch constraints, limit stops, and measured electrical states.
The original cabinet_backend remains the separate demonstration entry point.
"""
from __future__ import annotations

import json
import math
from pathlib import Path

import numpy as np

from cabinet_interaction import CabinetInteraction, CabinetInteractionConfig


ROOT = Path(__file__).resolve().parents[1]
CONFIG_PATH = ROOT / "config/interaction.json"
MODES = {"AUTO": -math.pi / 4, "STOP": 0.0, "MANUAL": math.pi / 4}
JOINTS = {
    "yaban": ("left_knob_joint", "left_knob_lift_joint", "blade_joint"),
    "breaker": ("breaker_toggle_joint",),
    "selector": ("xuanniu_selector_joint",),
    "reset": ("fugui_press_joint",),
}


class TotalCabinetInteraction:
    """CabinetInteraction-compatible host with all 40 passive components.

    ``config`` retains the old door-config meaning (dataclass or dict). The
    component coefficients live in config/interaction.json. Public state and
    contact mappings use the unchanged manifest instance names.
    """

    def __init__(self, mujoco, model, data, config=None):
        self.mujoco, self.model, self.data = mujoco, model, data
        self.model_revision = 0
        if isinstance(config, dict):
            config = CabinetInteractionConfig(**config)
        self.parameters = json.loads(CONFIG_PATH.read_text())
        button = json.loads((ROOT.parent / 'fugui_gazebo/config/button.yaml').read_text())
        button.update(self.parameters['reset'])
        self.parameters['reset'] = button
        self._validate_parameters()
        self.door = CabinetInteraction(mujoco, model, data, config)
        instances = json.loads((ROOT / "manifest.json").read_text())["instances"]
        self._parts = {}
        self._geom_owner = {}
        self._contact_state = {}
        geom_names = [model.geom(i).name or "" for i in range(model.ngeom)]
        for item in instances:
            name, kind = item["name"], item["kind"]
            prefix = name + "__"
            ids = np.asarray([model.joint(prefix + j).id for j in JOINTS[kind]], dtype=int)
            part = dict(name=name, kind=kind, prefix=prefix, ids=ids,
                        qa=model.jnt_qposadr[ids].copy(), va=model.jnt_dofadr[ids].copy(),
                        initial_range=model.jnt_range[ids].copy())
            if kind == "yaban":
                part["thread"] = model.equality(prefix + "thread_coupling").id
                part["lock"] = model.equality(prefix + "blade_lock").id
                expected = self.parameters[kind]["screw_pitch"] / (2 * math.pi)
                if not math.isclose(model.eq_data[part["thread"], 1], expected, rel_tol=1e-8):
                    raise ValueError(f"{name}: generated screw pitch differs from interaction.json")
            elif kind == "breaker":
                part["lamp"] = model.material(prefix + "red_lamp").id
                part["indicators"] = [i for i, n in enumerate(geom_names)
                                      if n.startswith(prefix + "breaker_indicator_link_visual_")]
            elif kind == "reset":
                part["lamp"] = model.material(prefix + "fugui_lamp").id
                if not np.allclose(model.jnt_range[ids[0]], [0, button['travel_max']], atol=1e-9, rtol=0):
                    raise ValueError(f'{name}: button travel differs from button.yaml; rebuild models')
            for i, geom_name in enumerate(geom_names):
                if geom_name.startswith(prefix) and (model.geom_contype[i] or model.geom_conaffinity[i]):
                    self._geom_owner[i] = name
            self._parts[name] = part
            self._apply_parameters(part)
        component_joints = {int(j) for p in self._parts.values() for j in p["ids"]}
        joint_transmissions = {int(mujoco.mjtTrn.mjTRN_JOINT), int(mujoco.mjtTrn.mjTRN_JOINTINPARENT)}
        for i in range(model.nu):
            if int(model.actuator_trntype[i]) in joint_transmissions and int(model.actuator_trnid[i, 0]) in component_joints:
                raise ValueError("Passive cabinet scene must not contain component actuators")
        self._wrench = np.zeros(6)
        self.reset("open")

    def __getattr__(self, name):
        # Existing environment code also reads door joint-address dictionaries
        # and robot-contact geometry sets directly.
        door = self.__dict__.get("door")
        if door is None:
            raise AttributeError(name)
        return getattr(door, name)

    def _validate_parameters(self):
        for kind in JOINTS:
            for name, value in self.parameters[kind].items():
                values = np.asarray(value, dtype=float)
                if not np.isfinite(values).all() or np.any(values < 0):
                    raise ValueError(f"{kind}.{name} must be finite and non-negative")
        plate = self.parameters["yaban"]
        if not (0 < plate["closed_tolerance"] < plate["leave_closed_tolerance"]):
            raise ValueError("Pressure-plate closed tolerances must define a hysteresis interval")
        if not (0 < plate["knob_hysteresis"] < plate["release_angle"] < 4 * math.pi):
            raise ValueError("Invalid pressure-plate release angle or hysteresis")
        if plate["screw_pitch"] <= 0 or self.parameters["breaker"]["detent_angle"] <= 0:
            raise ValueError("Screw pitch and breaker detent angle must be positive")
        button = self.parameters["reset"]
        if not (0 <= button["rearm_depth"] < button["valid_min"] <= button["default_depth"] <= button["valid_max"] <= button['travel_max']):
            raise ValueError("Invalid reset-button trigger depths")

    def _apply_parameters(self, part):
        cfg = self.parameters[part["kind"]]
        ids, va = part["ids"], part["va"]
        if part["kind"] == "yaban":
            self.model.dof_damping[va] = cfg["joint_damping"]
            self.model.dof_frictionloss[va[0]] = cfg["knob_frictionloss"]
            self.model.dof_frictionloss[va[2]] = cfg["blade_frictionloss"]
            self.model.jnt_solref[ids] = cfg["limit_solref"]
            self.model.jnt_solimp[ids] = cfg["limit_solimp"]
            for eq in (part["thread"], part["lock"]):
                self.model.eq_solref[eq] = cfg["constraint_solref"]
                self.model.eq_solimp[eq] = cfg["constraint_solimp"]
            self.model.eq_solimp[part['thread']] = cfg['thread_solimp']
        else:
            self.model.dof_damping[va] = cfg["damping"]
            if part["kind"] == "reset":
                self.model.jnt_stiffness[ids[0]] = cfg["stiffness"]
                self.model.qpos_spring[part["qa"][0]] = 0.0

    def configure_selector(self, parameters):
        """Update passive selector coefficients during scene setup."""
        if parameters is None:
            return
        allowed = self.parameters['selector']
        if not isinstance(parameters, dict) or set(parameters) - set(allowed):
            raise ValueError('Unknown selector mechanism parameters')
        values = {key: float(value) for key, value in parameters.items()}
        if not all(math.isfinite(value) and value >= 0 for value in values.values()):
            raise ValueError('Selector parameters must be finite and non-negative')
        allowed.update(values)
        for part in self._parts.values():
            if part['kind'] == 'selector':
                self._apply_parameters(part)

    def reset(self, state="open"):
        """Reset this cabinet only; the robot state belongs to the host."""
        self.door.reset(state)
        self.data.qfrc_applied[list(self.door.dof_addresses.values())] = 0.0
        for part in self._parts.values():
            self.model.jnt_range[part["ids"]] = part["initial_range"]
            self.data.qpos[part["qa"]] = 0.0
            self.data.qvel[part["va"]] = 0.0
            self.data.qfrc_applied[part["va"]] = 0.0
            kind = part["kind"]
            if kind == "yaban":
                part.update(locked=True, tightening_blocked=False)
                self.mujoco.equality_active(self.model, self.data)[part["thread"]] = True
                self.mujoco.equality_active(self.model, self.data)[part["lock"]] = True
            elif kind == "breaker":
                self.data.qpos[part["qa"][0]] = part["initial_range"][0, 1]
                part["powered"] = False
                self._update_lamp(part)
            elif kind == "selector":
                part.update(mode="STOP", settled=self.parameters["selector"]["settle_seconds"],
                            settling_mode="STOP")
            else:
                part.update(count=0, armed=True, led_on=False, led_until=-1.0)
                self._update_lamp(part)
        self._last_post_time = float(self.data.time)
        self._contact_state = {name: self._empty_contact() for name in self._parts}

    def _update_plate(self, part):
        cfg = self.parameters["yaban"]
        knob, _, blade = self.data.qpos[part["qa"]]
        if part["locked"] and knob >= cfg["release_angle"]:
            part["locked"] = False
        elif (not part["locked"] and abs(blade) <= cfg["closed_tolerance"]
              and knob < cfg["release_angle"] - cfg["knob_hysteresis"]):
            part["locked"] = True
        # Elastic deflection of a locked blade must never create a pi-radian
        # lower stop under a still-tight knob and thereby unwind that knob.
        if part["locked"]:
            part["tightening_blocked"] = False
        elif abs(blade) > cfg["leave_closed_tolerance"]:
            part["tightening_blocked"] = True
        elif abs(blade) <= cfg["closed_tolerance"]:
            part["tightening_blocked"] = False
        self.mujoco.equality_active(self.model, self.data)[part["thread"]] = True
        self.mujoco.equality_active(self.model, self.data)[part["lock"]] = part["locked"]
        lower = cfg["release_angle"] if part["tightening_blocked"] else part["initial_range"][0, 0]
        if self.model.jnt_range[part["ids"][0], 0] != lower:
            self.model.jnt_range[part["ids"][0], 0] = lower
            self.model_revision += 1

    def pre_step(self):
        """Apply mechanical forces once before the host's mj_step."""
        self.door.pre_step()
        for part in self._parts.values():
            kind = part["kind"]
            cfg = self.parameters[kind]
            self.data.qfrc_applied[part["va"]] = 0.0
            if kind == "yaban":
                self._update_plate(part)
            elif kind == "breaker":
                q = float(self.data.qpos[part["qa"][0]])
                self.data.qfrc_applied[part["va"][0]] = cfg["detent_torque"] * math.sin(math.pi * q / cfg["detent_angle"])
            elif kind == "selector":
                q = float(self.data.qpos[part["qa"][0]])
                self.data.qfrc_applied[part["va"][0]] = -cfg["detent_torque"] * math.sin(8 * q)
            # Reset buttons use their native, implicitly damped joint spring.

    def post_step(self):
        """Read actual joints for interlocks, electrical states and indicators."""
        self.door.post_step()
        now = float(self.data.time)
        dt = max(0.0, now - self._last_post_time)
        self._last_post_time = now
        for part in self._parts.values():
            kind, cfg = part["kind"], self.parameters[part["kind"]]
            q = float(self.data.qpos[part["qa"][0]])
            if kind == "yaban":
                self._update_plate(part)
            elif kind == "breaker":
                previous = part["powered"]
                if q < -cfg["hysteresis"]:
                    part["powered"] = True
                elif q > cfg["hysteresis"]:
                    part["powered"] = False
                if part["powered"] != previous:
                    self._update_lamp(part)
            elif kind == "selector":
                nearest = min(MODES, key=lambda mode: abs(q - MODES[mode]))
                v = float(self.data.qvel[part["va"][0]])
                if nearest != part["settling_mode"]:
                    part["settled"] = 0.0
                    part["settling_mode"] = nearest
                if abs(q - MODES[nearest]) < cfg["angle_tolerance"] and abs(v) < cfg["velocity_tolerance"]:
                    part["settled"] += dt
                else:
                    part["settled"] = 0.0
                part["mode"] = nearest if part["settled"] >= cfg["settle_seconds"] else "TRANSITION"
            else:
                if q <= cfg["rearm_depth"]:
                    part["armed"] = True
                if part["armed"] and cfg["valid_min"] - cfg["numerical_tolerance"] <= q <= cfg["valid_max"] + cfg["numerical_tolerance"]:
                    part["armed"] = False
                    part["count"] += 1
                    part["led_until"] = now + cfg["lamp_duration"]
                led_on = now < part["led_until"]
                if led_on != part["led_on"]:
                    part["led_on"] = led_on
                    self._update_lamp(part)
        self._refresh_contacts()

    def _update_lamp(self, part):
        self.model_revision += 1
        lamp = part["lamp"]
        if part["kind"] == "breaker":
            on = part["powered"]
            self.model.mat_rgba[lamp] = [1, .018, .008, 1] if on else [.15, .008, .005, 1]
            for geom in part["indicators"]:
                self.model.geom_rgba[geom] = [.85, .055, .025, 1] if on else [.015, .32, .18, 1]
        else:
            on = part["led_on"]
            self.model.mat_rgba[lamp] = [.03, 1, .13, 1] if on else [.005, .12, .045, 1]
        self.model.mat_emission[lamp] = .8 if on else 0.0

    @staticmethod
    def _empty_contact():
        return dict(robot_contact=False, contact_count=0, normal_force=0.0)

    def _refresh_contacts(self):
        self._contact_state = {name: self._empty_contact() for name in self._parts}
        for index in range(int(self.data.ncon)):
            contact = self.data.contact[index]
            first, second = int(contact.geom1), int(contact.geom2)
            if first in self.door.robot_geoms:
                name = self._geom_owner.get(second)
            elif second in self.door.robot_geoms:
                name = self._geom_owner.get(first)
            else:
                continue
            if name is None or contact.dist > self.door.config.event_contact_epsilon:
                continue
            item = self._contact_state[name]
            item["robot_contact"] = True
            item["contact_count"] += 1
            self.mujoco.mj_contactForce(self.model, self.data, index, self._wrench)
            item["normal_force"] += max(0.0, float(self._wrench[0]))

    def component_state(self):
        result = {}
        for name, part in self._parts.items():
            q = self.data.qpos[part["qa"]]
            state = dict(kind=part["kind"])
            if part["kind"] == "yaban":
                knob, lift, blade = map(float, q)
                cfg = self.parameters["yaban"]
                closed = abs(blade) <= cfg["closed_tolerance"]
                status = "locked" if part["locked"] else "inserted_released" if closed else "withdrawn"
                state.update(knob=knob, lift=lift, blade=blade, locked=part["locked"],
                             tightening_blocked=part["tightening_blocked"], status=status,
                             thread_error=lift - knob * cfg["screw_pitch"] / (2 * math.pi),
                             knob_deg=math.degrees(knob), blade_deg=math.degrees(blade),
                             lift_mm=lift*1000,
                             withdrawal_complete=not part['locked'] and blade>=math.radians(88))
            elif part["kind"] == "breaker":
                state.update(angle=float(q[0]), powered=part["powered"])
            elif part["kind"] == "selector":
                state.update(angle=float(q[0]), mode=part["mode"])
            else:
                state.update(depth_mm=float(q[0]) * 1000, count=part["count"],
                             armed=part["armed"], led_on=part["led_on"])
            result[name] = state
        return result

    def component_contacts(self):
        return {name: item.copy() for name, item in self._contact_state.items()}

    def snapshot(self):
        result = self.door.snapshot()
        result["components"] = self.component_state()
        result["component_contacts"] = self.component_contacts()
        return result
