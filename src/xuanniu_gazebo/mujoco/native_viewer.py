"""Full native Simulate UI with a selector-aware physics loop.

The native binding is isolated here and validated against MuJoCo 3.12.x.
Unlike launch_passive, Simulate owns the scene, UI, model edits and history;
there is no Python viewer.sync() or second copy of the rendered model.
"""
import atexit
import threading
import time

import glfw
import mujoco
import mujoco.viewer  # Initializes the native binding's GLFW library handle.
from mujoco import _simulate

from runtime import SimulationConnection
from selector_backend import ROOT, SelectorSimulation
from native_input import NativeInput

# Native Simulate uses these UI slowdown percentages (same table as viewer.py).
SPEEDS=mujoco.viewer.PERCENT_REALTIME


class NativeViewer(SimulationConnection):
    def __init__(self, simulation, model_path=None, camera='free'):
        super().__init__(simulation,start=False)
        self.model_path=str(model_path or ROOT/'mujoco/xuanniu.xml')
        self.camera_name=camera
        self.camera=mujoco.MjvCamera(); self.option=mujoco.MjvOption(); self.perturb=mujoco.MjvPerturb()
        # True selects managed Simulate: enables native Simulation, timing,
        # stepping, history, profiler and sensor features.
        self.viewer=_simulate.Simulate(self.camera,self.option,self.perturb,None,True,None)
        self.lock=self.viewer.lock()
        self.viewer.ui0_enable=True; self.viewer.ui1_enable=True
        self.failure=None
        self._last_running=True; self._last_speed=1.0
        self.received=0.0  # Do not enable remote actions before the scene loads.
        self.max_lock_wait=0.0; self.max_step_batch=0.0
        self.input=NativeInput()
        self._requested_run=None

    def set_camera(self):
        if self.camera_name=='free':
            mujoco.mjv_defaultFreeCamera(self.simulation.model,self.camera)
            self.camera.lookat[:]=[.015,0,.12]
            self.camera.distance=.13; self.camera.azimuth=-35; self.camera.elevation=-20
        else:
            self.camera.type=mujoco.mjtCamera.mjCAMERA_FIXED
            self.camera.fixedcamid=self.simulation.model.camera(self.camera_name).id

    def apply_pause(self, paused):
        desired=not paused
        if self._requested_run==desired: return
        self._requested_run=desired
        if bool(self.viewer.run)!=desired: self.input.key(32)
        if paused: self.simulation.set_paused(True)
        self.reanchor()

    def apply_reset(self):
        super().apply_reset()
        self.apply_pause(False)
        self.viewer.speed_changed=True
        self._last_time=float(self.simulation.data.time)

    def _load(self, path, initial=False):
        if not initial:
            # A failed or incompatible drop leaves the current model intact.
            replacement=SelectorSimulation(path)
        else: replacement=self.simulation
        self.viewer.load(replacement.model,replacement.data,path)
        self.simulation=replacement; self.model_path=path
        self.input.find_window()  # Resolve once, outside the interactive model lock.
        with self.viewer.lock():
            self.set_camera(); self.reanchor()
            self._last_time=float(replacement.data.time)
            self._last_running=bool(self.viewer.run)
            replacement.set_paused(not self._last_running)
            self.publish(); self.viewer.load_error=''

    def _run(self):
        try:
            self._load(self.model_path,initial=True)
            while not self.done.is_set() and not self.viewer.exitrequest:
                path=None
                if self.viewer.droploadrequest:
                    self.viewer.droploadrequest=0; path=self.viewer.dropfilename
                elif self.viewer.uiloadrequest:
                    self.viewer.uiloadrequest_decrement(); path=self.model_path
                if path:
                    try: self._load(path)
                    except Exception as error:
                        self.viewer.load_error=str(error); self.viewer.load_message_clear()
                waiting=time.monotonic()
                with self.viewer.lock():
                    acquired=time.monotonic()
                    self.max_lock_wait=max(self.max_lock_wait,acquired-waiting)
                    sim=self.simulation
                    running=bool(self.viewer.run)
                    if sim.data.time<self._last_time:
                        # Native Reset/keyframe/history edits cancel old targets.
                        sim._reset_state()
                        if abs(sim.angle)>.001:
                            from xuanniu_gazebo.logic import snap
                            sim.target=snap(sim.angle); sim.mode='TRANSITION'; sim.stable_since=None
                        sim.set_paused(not running); self.reanchor()
                    if running!=self._last_running:
                        sim.set_paused(not running); self.reanchor()
                        self._last_running=running
                    if self._requested_run is not None and running==self._requested_run:
                        self._requested_run=None
                        sim.set_paused(not running); self.reanchor()
                    speed=SPEEDS[self.viewer.real_time_index]/100
                    if speed!=self._last_speed or self.viewer.speed_changed:
                        self.reanchor(); self._last_speed=speed; self.viewer.speed_changed=False
                    commanded=self.drain_commands()
                    if self.advance(speed): self.viewer.add_to_history()
                    if sim.paused:
                        mujoco.mj_forward(sim.model,sim.data)
                        sim.after_step()
                        self._last_time=float(sim.data.time)
                    elapsed=time.monotonic()-self._anchor_wall
                    advanced=sim.data.time-self._anchor_sim
                    if advanced>.001: self.viewer.measured_slowdown=elapsed/advanced
                    if commanded or time.monotonic()-self.received>=.02: self.publish()
                    self.max_step_batch=max(self.max_step_batch,time.monotonic()-acquired)
                self.done.wait(.001)
        except BaseException as error:
            self.failure=error
        finally:
            self.done.set(); self.viewer.exit()

    def run(self, duration=0):
        if not glfw.init(): raise RuntimeError('GLFW could not initialize the display')
        timer=None
        try:
            atexit.register(self.viewer.exit)
            self.start()
            if duration:
                timer=threading.Timer(duration,self.viewer.exit); timer.start()
            # The official C++ Simulate render loop runs on the main thread.
            self.viewer.render_loop()
        finally:
            if timer: timer.cancel()
            self.close(); atexit.unregister(self.viewer.exit)
            self.viewer.destroy(); self.input.close(); glfw.terminate()
        if self.failure: raise self.failure

    def close(self):
        self.done.set(); self.viewer.exit()
        if self.thread: self.thread.join(timeout=3)
