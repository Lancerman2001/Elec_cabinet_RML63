"""One simulation owner; per-instance controllers adapted from the source demos.

Breakers, selectors and reset buttons use the existing bounded actuators.
Door/pressure-plate motions preserve the source kinematic demonstration semantics.
No component controller calls mj_step or resets another component's state.
"""
from __future__ import annotations

from collections import deque
import json
import math
from pathlib import Path
import numpy as np
import mujoco_compat as mujoco

try:
    from . import yaban_protocol as yp, door_protocol as dp
except ImportError:
    import yaban_protocol as yp
    import door_protocol as dp

ROOT = Path(__file__).resolve().parents[1]
MODEL_PATH = ROOT/'mujoco/total_cabinet_glass_door.xml'
MODES = {'MANUAL': math.pi/4, 'STOP': 0., 'AUTO': -math.pi/4}


class Component:
    def __init__(self, host, name, joint, actuator):
        self.host, self.m, self.d = host, host.model, host.data
        self.name, self.prefix = name, name+'__'
        self.jid = self.m.joint(self.prefix+joint).id
        self.qa = int(self.m.jnt_qposadr[self.jid])
        self.va = int(self.m.jnt_dofadr[self.jid])
        self.aid = self.m.actuator(self.prefix+actuator).id
        self.error = ''

    @property
    def q(self): return float(self.d.qpos[self.qa])

    @property
    def v(self): return float(self.d.qvel[self.va])

    def numeric(self, name): return self.m.numeric(self.prefix+name).data.copy()

    def ready(self):
        if self.host.paused: raise RuntimeError('仿真已暂停')
        if self.error: raise RuntimeError(self.error)


class Breaker(Component):
    def __init__(self, host, name):
        super().__init__(host, name, 'breaker_toggle_joint', 'breaker_position')
        self.on, self.off = map(float, self.m.jnt_range[self.jid])
        self.speed = float(self.numeric('target_speed')[0])
        self.hysteresis = float(self.numeric('hysteresis')[0])
        self.timeout = float(self.numeric('action_timeout')[0])
        self.indicators = [i for i in range(self.m.ngeom) if self.m.geom(i).name.startswith(self.prefix+'breaker_indicator_link_visual_')]
        self.lamp = self.m.material(self.prefix+'red_lamp').id
        self.powered = self.busy = False
        self.target = self.reference = self.off
        self.elapsed = self.settled = 0.
        self.update_lamp()

    def set_power(self, powered):
        self.ready()
        if not isinstance(powered, bool): raise ValueError('上电状态必须为bool')
        if self.busy: raise RuntimeError('空气开关正在拨动')
        self.target = self.on if powered else self.off
        self.reference = self.q; self.elapsed = self.settled = 0.; self.busy = True

    def before_step(self):
        if self.busy:
            self.reference += float(np.clip(self.target-self.reference, -self.speed*self.host.dt, self.speed*self.host.dt))
        self.d.ctrl[self.aid] = self.reference

    def update_lamp(self):
        self.m.mat_rgba[self.lamp] = [1., .018, .008, 1] if self.powered else [.15, .008, .005, 1]
        self.m.mat_emission[self.lamp] = .8 if self.powered else 0
        for i in self.indicators:
            self.m.geom_rgba[i] = [.85, .055, .025, 1] if self.powered else [.015, .32, .18, 1]

    def after_step(self):
        old = self.powered
        if self.q < -self.hysteresis: self.powered = True
        elif self.q > self.hysteresis: self.powered = False
        if old != self.powered: self.update_lamp()
        if self.busy:
            self.elapsed += self.host.dt
            self.settled = self.settled+self.host.dt if abs(self.q-self.target)<math.radians(1) and abs(self.v)<.08 else 0
            if self.settled >= .08: self.busy = False; self.reference = self.target
            elif self.elapsed > self.timeout:
                self.cancel(); self.error = '空气开关动作超时'

    def cancel(self):
        self.busy = False; self.reference = self.target = float(np.clip(self.q, self.on, self.off))

    def state(self):
        return dict(angle=self.q, powered=self.powered, busy=self.busy, error=self.error)


class Selector(Component):
    def __init__(self, host, name):
        super().__init__(host, name, 'xuanniu_selector_joint', 'xuanniu_torque')
        self.speed = float(self.numeric('target_speed')[0])
        self.kp = float(self.numeric('position_gain')[0])
        self.detent = float(self.numeric('detent_torque')[0])
        self.timeout = float(self.numeric('action_timeout')[0])
        self.target = self.reference = 0.; self.busy = False; self.mode = 'STOP'
        self.elapsed = self.settled = 0.

    def command(self, mode):
        self.ready()
        if mode not in MODES: raise ValueError('转换开关挡位必须为 MANUAL / STOP / AUTO')
        if self.busy: raise RuntimeError('转换开关正在旋转')
        self.target = MODES[mode]; self.reference = self.q
        self.busy = True; self.mode = 'TRANSITION'; self.elapsed = self.settled = 0.

    def before_step(self):
        if self.busy:
            self.reference += float(np.clip(self.target-self.reference, -self.speed*self.host.dt, self.speed*self.host.dt))
            torque = self.kp*(self.reference-self.q)
        else:
            torque = -self.detent*math.sin(8*self.q)
        self.d.ctrl[self.aid] = np.clip(torque, *self.m.actuator_ctrlrange[self.aid])

    def after_step(self):
        goal = self.target if self.busy else min(MODES.values(), key=lambda q: abs(q-self.q))
        self.settled = self.settled+self.host.dt if abs(self.q-goal)<math.radians(1) and abs(self.v)<.05 else 0
        if self.settled >= .1:
            self.mode = next(k for k, q in MODES.items() if abs(q-goal)<1e-10); self.busy = False
        else: self.mode = 'TRANSITION'
        if self.busy:
            self.elapsed += self.host.dt
            if self.elapsed > self.timeout:
                self.busy = False; self.error = '转换开关动作超时'

    def state(self):
        return dict(angle=self.q, mode=self.mode, busy=self.busy, error=self.error)


class ResetButton(Component):
    def __init__(self, host, name):
        super().__init__(host, name, 'fugui_press_joint', 'fugui_press_force')
        self.lamp = self.m.material(self.prefix+'fugui_lamp').id
        self.valid_min, self.valid_max = self.numeric('valid_depth_range')
        self.rearm = float(self.numeric('rearm_depth')[0])
        self.pulse = float(self.numeric('lamp_duration')[0])
        self.tolerance = float(self.numeric('numerical_tolerance')[0])
        self.speed = float(self.numeric('reference_speed')[0])
        self.kp = float(self.numeric('servo_kp')[0]); self.kd = float(self.numeric('servo_kd')[0])
        self.timeout = float(self.numeric('action_timeout')[0])
        self.spring = float(self.m.jnt_stiffness[self.jid]); self.damping = float(self.m.dof_damping[self.va])
        self.active = self.led_on = False; self.armed = True; self.count = 0
        self.reference = self.target = 0.; self.hold_start = None; self.led_until = -1.
        self.update_lamp()

    def press(self, depth_mm=None, hold_seconds=.35, auto_release=True):
        self.ready()
        if depth_mm is None: depth_mm = float(self.numeric('default_depth')[0])*1000
        maximum = float(self.m.jnt_range[self.jid, 1])*1000
        if not math.isfinite(depth_mm) or not 0 <= depth_mm <= maximum: raise ValueError(f'按压深度必须为0～{maximum:g}mm')
        if not math.isfinite(hold_seconds) or not 0 <= hold_seconds <= 10: raise ValueError('保持时间必须为0～10秒')
        if not isinstance(auto_release, bool): raise ValueError('auto_release必须为bool')
        if self.active or self.q > self.rearm: raise RuntimeError('请先释放并等待按钮回位')
        self.target = depth_mm*.001; self.reference = self.q
        self.hold_seconds = hold_seconds; self.auto_release = auto_release
        self.hold_start = None; self.started = self.d.time; self.active = True

    def release(self):
        self.active = False; self.hold_start = None; self.d.ctrl[self.aid] = 0

    def before_step(self):
        force = 0.
        if self.active:
            self.reference += float(np.clip(self.target-self.reference, -self.speed*self.host.dt, self.speed*self.host.dt))
            force = self.kp*(self.reference-self.q)-self.kd*self.v+self.spring*self.q+self.damping*self.v
        self.d.ctrl[self.aid] = np.clip(force, *self.m.actuator_ctrlrange[self.aid])

    def update_lamp(self):
        self.m.mat_rgba[self.lamp] = [.03, 1, .13, 1] if self.led_on else [.005, .12, .045, 1]
        self.m.mat_emission[self.lamp] = .8 if self.led_on else 0

    def after_step(self):
        now = float(self.d.time)
        if self.q <= self.rearm: self.armed = True
        if self.armed and self.valid_min-self.tolerance <= self.q <= self.valid_max+self.tolerance:
            self.armed = False; self.count += 1; self.led_until = now+self.pulse
        old = self.led_on; self.led_on = now < self.led_until
        if old != self.led_on: self.update_lamp()
        if self.active:
            if self.hold_start is None and abs(self.q-self.target)<.00002 and abs(self.v)<.002:
                self.hold_start = now
            if self.hold_start is None and now-self.started > self.timeout:
                self.release(); self.error = '复归按钮动作超时'
            if self.active and self.auto_release and self.hold_start is not None and now-self.hold_start >= self.hold_seconds:
                self.release()

    def state(self):
        return dict(depth_mm=self.q*1000, count=self.count, led_on=self.led_on, busy=self.active, error=self.error)


class PrescribedMotion:
    """Only the listed DOFs are prescribed, as in the existing standalone demos."""
    def __init__(self, host, joints):
        self.host, self.m, self.d = host, host.model, host.data
        self.qa = np.array([self.m.jnt_qposadr[self.m.joint(j).id] for j in joints], dtype=int)
        self.va = np.array([self.m.jnt_dofadr[self.m.joint(j).id] for j in joints], dtype=int)
        self.values = self.d.qpos[self.qa].copy()
        self.segments = deque(); self.elapsed = 0.; self.error = ''

    @property
    def busy(self): return bool(self.segments)

    def enqueue(self, segments):
        if self.host.paused: raise RuntimeError('仿真已暂停')
        if self.busy: raise RuntimeError('当前动作尚未完成')
        self.segments = deque((duration, np.array(target, dtype=float)) for duration, target in segments)
        self.start = self.d.qpos[self.qa].copy(); self.elapsed = 0.

    def advance(self):
        if self.segments:
            duration, target = self.segments[0]
            self.elapsed += self.host.dt
            alpha = min(1., self.elapsed/duration)
            # Smooth start and stop, timed by simulation time rather than wall clock.
            blend = alpha*alpha*(3-2*alpha)
            self.values = self.start+(target-self.start)*blend
            if alpha >= 1:
                self.segments.popleft(); self.start = target.copy(); self.elapsed = 0.
                if not self.segments: self.finished()

    def pin(self):
        self.d.qpos[self.qa] = self.values
        self.d.qvel[self.va] = 0

    def finished(self): pass


class PressurePlate(PrescribedMotion):
    def __init__(self, host, name):
        self.name = name
        super().__init__(host, [name+'__'+j for j in ('left_knob_joint', 'left_knob_lift_joint', 'blade_joint')])

    def pose(self):
        return yp.Pose(float(self.d.qpos[self.qa[0]]), float(self.d.qpos[self.qa[2]]))

    def command(self, action):
        if action not in yp.allowed_actions(self.pose(), self.busy):
            raise RuntimeError(yp.action_error(action, self.pose(), self.busy))
        segments = []
        for step in yp.make_plan(action, self.pose()):
            pose = step.target
            segments.append((.8, [pose.knob, pose.lift, pose.blade]))
        self.enqueue(segments)

    def state(self):
        p = self.pose()
        return dict(knob=p.knob, blade=p.blade, lift=p.lift, busy=self.busy,
                    status=yp.state_text(p), allowed=sorted(yp.allowed_actions(p, self.busy)), error=self.error)


class Door(PrescribedMotion):
    def __init__(self, host):
        super().__init__(host, ['button_press_joint', 'handle_pop_joint', 'handle_turn_joint', 'door_hinge_joint'])
        self.protocol = dp.DoorCloseStateMachine()
        self.protocol.seed_pose(self.pose()); self.commit = None

    def pose(self): return dp.Pose.from_tuple(self.d.qpos[self.qa])

    def command(self, action):
        if self.host.paused: raise RuntimeError('仿真已暂停')
        if self.busy: raise RuntimeError('柜门正在运动')
        states = {'open': dp.DoorState.CLOSED, 'door_return': dp.DoorState.OPEN,
                  'handle_return': dp.DoorState.DOOR_RETURNED, 'handle_latch': dp.DoorState.LATCHED_HANDLE_OUT}
        if action not in states: raise ValueError('未知柜门操作')
        if self.protocol.state != states[action]: raise RuntimeError('柜门操作顺序错误')
        actual = self.pose()
        checks = {'open': self.protocol.is_closed_pose,
                  'door_return': self.protocol.is_open_pose,
                  'handle_return': lambda p: self.protocol.door_return_checks(p) == 0,
                  'handle_latch': lambda p: self.protocol.handle_return_checks(p) == 0}
        if not checks[action](actual): raise RuntimeError('实际柜门姿态不满足操作条件')
        self.commit = None
        if action == 'open':
            self.enqueue([(.20, [-.006, 0, 0, 0]), (.25, [0, 0, 0, 0]),
                          (.60, [0, -math.pi/3, 0, 0]), (.60, [0, -math.pi/3, math.pi/2, 0]),
                          (1.20, dp.open_pose().as_tuple())])
        else:
            targets = {'door_return': (1.20, dp.door_returned_pose(), dp.DoorCommand.COMMIT_DOOR_RETURN),
                       'handle_return': (.60, dp.latched_handle_out_pose(), dp.DoorCommand.COMMIT_HANDLE_RETURN),
                       'handle_latch': (.60, dp.closed_pose(), dp.DoorCommand.COMMIT_HANDLE_LATCH)}
            if action == 'door_return':
                self.protocol.handle_command(dp.DoorCommand.START_CLOSE, actual)
            duration, target, self.commit = targets[action]
            self.enqueue([(duration, target.as_tuple())])

    def finished(self):
        if self.commit is None:
            self.protocol.seed_pose(dp.Pose.from_tuple(self.values))
        else:
            result = self.protocol.handle_command(self.commit, dp.Pose.from_tuple(self.values))
            if result.result != dp.CommandResult.SUCCEEDED:
                self.error = result.detail

    def state(self):
        return dict(pose=list(self.pose().as_tuple()), status=self.protocol.state.name, busy=self.busy, error=self.error)


class CabinetSimulation:
    """Public host API. Call step/commands from one thread; viewer access needs its lock."""
    def __init__(self, model_path=MODEL_PATH):
        self.model = mujoco.load_model(str(model_path))
        self.data = mujoco.MjData(self.model)
        self.dt = float(self.model.opt.timestep)
        self.layout = json.loads((ROOT/'config/layout.json').read_text())
        self.instances = json.loads((ROOT/'manifest.json').read_text())['instances']
        self.reset()

    def reset(self):
        mujoco.mj_resetDataKeyframe(self.model, self.data, self.model.key('initial_open').id)
        self.paused = False
        self.components = {}
        classes = {'breaker': Breaker, 'selector': Selector, 'reset': ResetButton, 'yaban': PressurePlate}
        for item in self.instances:
            self.components[item['name']] = classes[item['kind']](self, item['name'])
        self.door = Door(self)
        self.motions = [self.door]+[x for x in self.components.values() if isinstance(x, PressurePlate)]
        self.dynamic = [x for x in self.components.values() if isinstance(x, Component)]
        mujoco.mj_forward(self.model, self.data)

    def command(self, instance, action, **kwargs):
        if self.paused and action != 'release': raise RuntimeError('仿真已暂停')
        if instance == 'door': return self.door.command(action)
        c = self.components[instance]
        if isinstance(c, Breaker):
            if action not in ('on', 'off'): raise ValueError('空气开关操作必须为on/off')
            return c.set_power(action == 'on')
        if isinstance(c, ResetButton):
            if action == 'press': return c.press(**kwargs)
            if action == 'release': return c.release()
            raise ValueError('复归按钮操作必须为press/release')
        return c.command(action)

    def set_paused(self, paused):
        self.paused = bool(paused)
        if self.paused:
            for c in self.components.values():
                if isinstance(c, ResetButton): c.release()

    def step(self, count=1, *, refresh=True):
        """Advance physics; optionally defer derived geometry refresh to display time.

        Controller feedback uses qpos/qvel. mj_step already computes the full
        dynamics at each step, so a second mj_forward is needed only when the
        caller consumes post-step transforms/contacts, not at every millisecond.
        """
        if self.paused: return
        for _ in range(count):
            for c in self.motions: c.advance(); c.pin()
            for c in self.dynamic: c.before_step()
            mujoco.mj_step(self.model, self.data)
            if not (np.isfinite(self.data.qpos).all() and np.isfinite(self.data.qvel).all()):
                self.set_paused(True); self.data.ctrl[:] = 0
                raise RuntimeError('MuJoCo出现非有限状态，请复位')
            for c in self.motions: c.pin()
            for c in self.dynamic: c.after_step()
        if count and refresh:
            mujoco.mj_forward(self.model, self.data)

    def run_for(self, seconds):
        if not math.isfinite(seconds) or seconds < 0: raise ValueError('时间必须为非负有限值')
        self.step(round(seconds/self.dt))

    def state(self):
        return dict(time=float(self.data.time), paused=self.paused, door=self.door.state(),
                    components={name: c.state() for name, c in self.components.items()})
