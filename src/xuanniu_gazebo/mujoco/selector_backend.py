"""MuJoCo physics and three-detent controller, independent of ROS and Qt."""
from dataclasses import dataclass
import math
from pathlib import Path
import sys
import time

import mujoco
import numpy as np

ROOT = Path(__file__).resolve().parent.parent
sys.path.insert(0, str(ROOT/'src'))
from xuanniu_gazebo.logic import HALF, MODES, clamp_angle, snap


@dataclass(frozen=True)
class Status:
    time: float
    angle_rad: float
    velocity_rad_s: float
    torque_nm: float
    target_angle_rad: float
    mode: str
    dragging: bool
    moving: bool
    error: str
    paused: bool


class SelectorController:
    """Single-owner API. Callers must serialize commands, stepping and data reads.

    Only reset writes qpos/qvel. All ordinary actions use a bounded motor torque.
    Dead-man timeout uses a monotonic wall clock, settling uses simulation time.
    """
    def __init__(self, model, data, prefix='xuanniu_', clock=time.monotonic):
        self.clock = clock
        self.model = model; self.data = data
        self.joint = self.model.joint(prefix+'selector_joint').id
        self.qadr = int(self.model.jnt_qposadr[self.joint])
        self.vadr = int(self.model.jnt_dofadr[self.joint])
        self.motor = self.model.actuator(prefix+'torque').id
        self.speed = self.parameter('target_speed')
        self.kp = self.parameter('position_gain')
        self.kd = self.parameter('velocity_gain')
        self.detent = self.parameter('detent_torque')
        self.drag_timeout = self.parameter('drag_timeout')
        self.action_timeout = self.parameter('action_timeout')
        self.torque_min, self.torque_max = self.model.actuator_ctrlrange[self.motor]
        self.model.dof_damping[self.vadr] = self.kd  # implicit damping, once only
        self._reset_state()

    def parameter(self, name):
        return float(self.model.numeric(name).data[0])

    @property
    def angle(self): return float(self.data.qpos[self.qadr])

    @property
    def velocity(self): return float(self.data.qvel[self.vadr])

    def reset(self):
        # Reset this component only; do not reset the cabinet or other joints.
        self.data.qpos[self.qadr] = 0; self.data.qvel[self.vadr] = 0
        self.data.ctrl[self.motor] = 0
        mujoco.mj_forward(self.model, self.data)
        self._reset_state()

    def _reset_state(self):
        self.target = self.reference = 0.0
        self.dragging = self.active = self.paused = False
        self.error = ''
        self.stable_since = float(self.data.time)-.1
        self.mode = 'STOP'
        self.started = 0.0
        self.last_drag = self.clock()

    def command(self, angle, dragging=False):
        if self.paused or self.error == 'NONFINITE_PHYSICS':
            if self.paused: self.error = 'SIMULATION_NOT_READY'
            return False
        if not math.isfinite(angle) or abs(angle) > HALF + 1e-12:
            self.error = 'INVALID_ANGLE'
            return False
        if not self.active: self.reference = clamp_angle(self.angle)
        self.target = clamp_angle(angle) if dragging else snap(angle)
        self.dragging = bool(dragging); self.active = True
        self.last_drag = self.clock(); self.started = float(self.data.time)
        self.stable_since = None; self.mode = 'TRANSITION'; self.error = ''
        return True

    def end_drag(self):
        """Lost focus, lost stream or close: choose the measured position."""
        if self.dragging:
            self.dragging = False; self.target = snap(self.angle)
            self.reference = clamp_angle(self.angle); self.active = True
            self.started = float(self.data.time); self.stable_since = None

    def set_paused(self, paused):
        self.end_drag()
        self.paused = bool(paused)
        if not paused and self.error == 'SIMULATION_NOT_READY': self.error = ''

    def step(self, count=1):
        for _ in range(count):
            if not self.before_step(): return
            mujoco.mj_step(self.model, self.data)
            self.after_step()

    def before_step(self):
        """Cabinet host calls this before its single shared mj_step()."""
        if self.paused or self.error == 'NONFINITE_PHYSICS': return False
        if not math.isfinite(self.angle) or not math.isfinite(self.velocity):
            self.error = 'NONFINITE_PHYSICS'; self.active = self.dragging = False
            self.data.ctrl[self.motor] = 0
            return False
        if self.dragging and self.clock()-self.last_drag > self.drag_timeout:
            self.end_drag()
        dt = float(self.model.opt.timestep)
        if self.active:
            delta = max(-self.speed*dt, min(self.speed*dt, self.target-self.reference))
            self.reference += delta
            torque = self.kp*(self.reference-self.angle)
        else:
            torque = -self.detent*math.sin(8*self.angle)
        self.data.ctrl[self.motor] = max(self.torque_min, min(self.torque_max, torque))
        return True

    def after_step(self):
        """Cabinet host calls this after its shared mj_step()."""
        self._classify()

    def _classify(self):
        nearest = snap(self.angle)
        goal = self.target if self.active else nearest
        stable = not self.dragging and abs(self.angle-goal)<math.radians(1) and abs(self.velocity)<.05
        if stable:
            if self.stable_since is None: self.stable_since = float(self.data.time)
            if self.data.time-self.stable_since >= .1:
                self.mode = next(key for key,value in MODES.items() if abs(value-goal)<1e-10)
                self.active = False; self.target = goal
        else:
            self.stable_since = None; self.mode = 'TRANSITION'
        if self.active and not self.dragging and self.data.time-self.started > self.action_timeout:
            self.active = False; self.target = nearest; self.error = 'TARGET_TIMEOUT'

    def status(self):
        return Status(float(self.data.time),self.angle,self.velocity,
                      float(self.data.qfrc_actuator[self.vadr]),self.target,self.mode,
                      self.dragging,self.active or self.mode=='TRANSITION',self.error,self.paused)


class SelectorSimulation(SelectorController):
    """Standalone demo owns a model; cabinets can use SelectorController directly."""
    def __init__(self, model_path=None, clock=time.monotonic):
        model = mujoco.MjModel.from_xml_path(str(model_path or ROOT/'mujoco/xuanniu.xml'))
        data = mujoco.MjData(model)
        super().__init__(model,data,clock=clock)
        self.reset()

    def reset(self):
        mujoco.mj_resetDataKeyframe(self.model,self.data,self.model.key('stop').id)
        super().reset()
