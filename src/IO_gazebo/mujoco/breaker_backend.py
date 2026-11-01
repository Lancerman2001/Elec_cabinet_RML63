"""ROS-independent single-joint MuJoCo demonstration backend."""
from dataclasses import dataclass
import math
from pathlib import Path
import numpy as np

MODEL_PATH=Path(__file__).with_name('breaker.xml')


@dataclass(frozen=True)
class State:
    angle: float
    velocity: float
    powered: bool
    busy: bool
    status: str
    sim_time: float


class BreakerBackend:
    def __init__(self,model_path=MODEL_PATH,show_panel=True):
        try: import mujoco
        except ImportError as exc:
            raise RuntimeError('请先执行 conda activate ros，再运行本程序。') from exc
        self.mj=mujoco
        self.model=mujoco.MjModel.from_xml_path(str(Path(model_path).resolve()))
        self.data=mujoco.MjData(self.model)
        self.joint=self.model.joint('breaker_toggle_joint').id
        self.qadr=int(self.model.jnt_qposadr[self.joint]); self.vadr=int(self.model.jnt_dofadr[self.joint])
        self.actuator=self.model.actuator('breaker_position').id
        self.on_angle,self.off_angle=map(float,self.model.jnt_range[self.joint])
        self.dt=float(self.model.opt.timestep)
        self.speed=self.numeric('target_speed'); self.hysteresis=self.numeric('hysteresis')
        self.timeout=self.numeric('action_timeout')
        self.indicators=[i for i in range(self.model.ngeom) if self.model.geom(i).name.startswith('breaker_indicator_link_visual_')]
        if not show_panel:
            for i in range(self.model.ngeom):
                if self.model.geom(i).name.startswith('breaker_panel_link_visual_'): self.model.geom_rgba[i,3]=0
        self.reset()

    def numeric(self,name):
        index=self.model.numeric(name).id
        return float(self.model.numeric_data[self.model.numeric_adr[index]])

    def reset(self):
        self.mj.mj_resetDataKeyframe(self.model,self.data,self.model.key('off').id)
        self.busy=False; self.powered=False; self.error=''; self.target=self.off_angle
        self.reference=self.off_angle; self.elapsed=0.; self.settled=0.
        self.mj.mj_forward(self.model,self.data); self.update_indicator()

    def set_power(self,powered):
        if not isinstance(powered,bool): raise ValueError('set_power expects a bool')
        if self.busy: raise RuntimeError('正在拨动，请等待当前动作完成。')
        self.target=self.on_angle if powered else self.off_angle
        self.reference=float(self.data.qpos[self.qadr]); self.elapsed=0.; self.settled=0.; self.error=''
        self.busy=True

    def cancel(self):
        self.busy=False
        self.reference=float(np.clip(self.data.qpos[self.qadr],self.on_angle,self.off_angle))
        self.target=self.reference; self.data.ctrl[self.actuator]=self.reference

    def update_indicator(self):
        rgba=[.85,.055,.025,1] if self.powered else [.015,.32,.18,1]
        for i in self.indicators: self.model.geom_rgba[i]=rgba

    def step(self,count=1):
        for _ in range(count):
            if self.busy:
                self.reference+=float(np.clip(self.target-self.reference,-self.speed*self.dt,self.speed*self.dt))
                self.data.ctrl[self.actuator]=self.reference
            self.mj.mj_step(self.model,self.data)
            q=float(self.data.qpos[self.qadr]); v=float(self.data.qvel[self.vadr])
            if not (math.isfinite(q) and math.isfinite(v)):
                self.busy=False; self.error='仿真状态异常，请复位。'; raise RuntimeError(self.error)
            old=self.powered
            if q < -self.hysteresis: self.powered=True
            elif q > self.hysteresis: self.powered=False
            if old!=self.powered: self.update_indicator()
            if self.busy:
                self.elapsed+=self.dt
                self.settled=self.settled+self.dt if abs(q-self.target)<math.radians(1) and abs(v)<.08 else 0.
                if self.settled>=.08:
                    self.busy=False; self.data.ctrl[self.actuator]=self.target
                elif self.elapsed>self.timeout:
                    self.cancel(); self.error='动作超时，目标已取消；请复位。'

    def state(self):
        q=float(self.data.qpos[self.qadr]); v=float(self.data.qvel[self.vadr])
        if self.error: status=self.error
        elif self.busy: status='正在拨动'
        elif abs(q-(self.on_angle if self.powered else self.off_angle))<math.radians(2):
            status='已到通电端位' if self.powered else '已到断电端位'
        else: status='动作已取消'
        return State(q,v,self.powered,self.busy,status,float(self.data.time))
