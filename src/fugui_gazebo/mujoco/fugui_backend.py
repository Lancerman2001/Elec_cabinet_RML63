"""Physical MuJoCo pushbutton; no ROS, Tk or viewer dependencies."""
from dataclasses import dataclass
import math
from pathlib import Path

MODEL_PATH = Path(__file__).with_name('fugui.xml')


@dataclass(frozen=True)
class State:
    depth_mm: float
    velocity: float
    target_mm: float
    led_on: bool
    valid: bool
    armed: bool
    active: bool
    reset_count: int
    phase: str
    sim_time: float
    paused: bool


class FuguiBackend:
    def __init__(self, model_path=MODEL_PATH):
        try: import mujoco
        except ImportError as exc:
            raise RuntimeError('请先执行 conda activate ros，再运行 MuJoCo 程序。') from exc
        self.mj = mujoco
        self.model = mujoco.MjModel.from_xml_path(str(Path(model_path).resolve()))
        self.data = mujoco.MjData(self.model)
        self.joint = self.model.joint('fugui_press_joint').id
        self.qadr = int(self.model.jnt_qposadr[self.joint]); self.vadr = int(self.model.jnt_dofadr[self.joint])
        self.actuator = self.model.actuator('fugui_press_force').id
        self.lamp = self.model.material('fugui_lamp').id
        self.dt = float(self.model.opt.timestep)
        self.lower, self.upper = map(float, self.model.jnt_range[self.joint])
        self.valid_min, self.valid_max = self.numeric('valid_depth_range')
        self.default_depth_mm = self.numeric('default_depth')[0]*1000
        self.rearm_depth = self.numeric('rearm_depth')[0]
        self.pulse = self.numeric('lamp_duration')[0]
        self.tolerance = self.numeric('numerical_tolerance')[0]
        self.speed = self.numeric('reference_speed')[0]
        self.kp = self.numeric('servo_kp')[0]; self.kd = self.numeric('servo_kd')[0]
        self.timeout = self.numeric('action_timeout')[0]
        self.spring = float(self.model.jnt_stiffness[self.joint])
        self.damping = float(self.model.dof_damping[self.vadr])
        self.force_limit = float(self.model.actuator_ctrlrange[self.actuator, 1])
        self.reset()

    def numeric(self, name):
        idx = self.model.numeric(name).id; adr = self.model.numeric_adr[idx]
        return list(map(float, self.model.numeric_data[adr:adr+self.model.numeric_size[idx]]))

    def reset(self):
        self.mj.mj_resetDataKeyframe(self.model, self.data, self.model.key('rest').id)
        self.active = False; self.paused = False; self.auto_release = True
        self.armed = True; self.valid = False; self.led_on = False; self.reset_count = 0
        self.reference = 0.; self.target = 0.; self.hold_start = None; self.hold_seconds = .35
        self.action_start = 0.; self.led_until = -1.; self.last_event_time = None; self.error = ''
        self.mj.mj_forward(self.model, self.data); self._lamp()

    def press(self, depth_mm=None, hold_seconds=.35, auto_release=True):
        if depth_mm is None:
            depth_mm = self.default_depth_mm
        if not math.isfinite(depth_mm) or not self.lower*1000 <= depth_mm <= self.upper*1000:
            raise ValueError(f'目标按入深度必须为 {self.lower*1000:g}–{self.upper*1000:g} mm 的有限数值。')
        if not math.isfinite(hold_seconds) or not 0 <= hold_seconds <= 10:
            raise ValueError('保持时间必须为 0–10 秒的有限数值。')
        if not isinstance(auto_release, bool): raise ValueError('auto_release 必须为 bool。')
        if self.paused: raise RuntimeError('仿真已暂停，请恢复后按压。')
        if self.active or self.data.qpos[self.qadr] > self.rearm_depth or self.error:
            raise RuntimeError('请先释放按钮，等待回位后再按压。')
        self.target = depth_mm*.001; self.reference = float(self.data.qpos[self.qadr])
        self.hold_seconds = hold_seconds; self.auto_release = auto_release
        self.hold_start = None; self.action_start = float(self.data.time); self.active = True

    def release(self):
        self.active = False; self.hold_start = None; self.data.ctrl[self.actuator] = 0

    def set_paused(self, paused):
        if paused: self.release()
        self.paused = bool(paused)

    def _lamp(self):
        self.model.mat_rgba[self.lamp] = [.03, 1., .13, 1] if self.led_on else [.005, .12, .045, 1]
        self.model.mat_emission[self.lamp] = .8 if self.led_on else 0.

    def step(self, count=1):
        if self.paused: return
        for _ in range(count):
            q = float(self.data.qpos[self.qadr]); v = float(self.data.qvel[self.vadr])
            if self.active:
                self.reference += max(-self.speed*self.dt, min(self.speed*self.dt, self.target-self.reference))
                # Compensate the MJCF spring/damper during demonstration so net
                # force is kp*(reference-q)-kd*v. Passive spring remains on release.
                force = self.kp*(self.reference-q)-self.kd*v+self.spring*q+self.damping*v
                self.data.ctrl[self.actuator] = max(-self.force_limit, min(self.force_limit, force))
            else: self.data.ctrl[self.actuator] = 0
            self.mj.mj_step(self.model, self.data)
            q = float(self.data.qpos[self.qadr]); v = float(self.data.qvel[self.vadr]); now = float(self.data.time)
            if not (math.isfinite(q) and math.isfinite(v)):
                self.release(); self.error = 'NONFINITE_PHYSICS'; self.led_on = False; self._lamp()
                raise RuntimeError('MuJoCo 物理状态异常，请初始化。')
            self.valid = self.valid_min-self.tolerance <= q <= self.valid_max+self.tolerance
            if q <= self.rearm_depth: self.armed = True
            if self.valid and self.armed:
                self.armed = False; self.reset_count += 1; self.led_until = now+self.pulse; self.last_event_time = now
            old_lamp = self.led_on; self.led_on = now < self.led_until
            if self.led_on != old_lamp: self._lamp()
            if self.active:
                if self.hold_start is None and abs(q-self.target) <= .00002 and abs(v) < .002:
                    self.hold_start = now
                if self.hold_start is None and now-self.action_start > self.timeout:
                    self.release(); self.error = 'TARGET_TIMEOUT'
                if self.active and self.auto_release and self.hold_start is not None and now-self.hold_start >= self.hold_seconds:
                    self.release()
            if not self.active and q <= self.rearm_depth: self.error = ''

    def state(self):
        q = float(self.data.qpos[self.qadr]); v = float(self.data.qvel[self.vadr])
        phase = ('PRESSING' if self.hold_start is None else 'HOLDING') if self.active else ('RETURNING' if q > .0001 else 'IDLE')
        return State(q*1000, v, self.target*1000, self.led_on, self.valid, self.armed,
                     self.active, self.reset_count, self.error or phase, float(self.data.time), self.paused)
