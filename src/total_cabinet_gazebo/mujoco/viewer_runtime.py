"""Free-camera presets and a physics worker independent of the Tk event loop."""
from __future__ import annotations

from contextlib import nullcontext
import queue
import threading
import time

import mujoco_compat as mujoco

# Positions here are targets and viewing directions, not fixed MJCF cameras.
# MuJoCo's azimuth=180 places the observer on the cabinet's +X side.
CAMERAS = {
    'front': ((.2, 0, 1.13), 3.5, 180., 0.),
    'oblique': ((.25, 0, 1.13), 3.6, 140., -15.),
    'plates': ((.215, -.21, .505), .24, 145., -12.),
    'breakers': ((.215, 0, 1.035), .66, 180., -5.),
    'controls': ((.215, .02, 1.39), .85, 180., -3.),
}


def set_free_camera(camera, preset='front'):
    lookat, distance, azimuth, elevation = CAMERAS[preset]
    camera.type = mujoco.mjtCamera.mjCAMERA_FREE
    camera.fixedcamid = -1; camera.trackbodyid = -1
    camera.lookat[:] = lookat
    camera.distance = distance
    camera.azimuth = azimuth; camera.elevation = elevation


class NativeControls:
    """Isolate the MuJoCo 3.12 native run/speed adapter.

    Handle exposes no public run/speed API. The installed viewer's _Simulate
    fields are also used by its own physics loop; feature detection permits
    other MuJoCo versions to retain the independent panel controls.
    """
    def __init__(self, viewer):
        self.viewer = viewer
        handle = self.handle()
        self.last_run = bool(handle.run) if handle is not None and hasattr(handle, 'run') else None

    def handle(self):
        getter = getattr(self.viewer, '_get_sim', None) or getattr(self.viewer, '_sim', None)
        return getter() if getter else None

    @property
    def supported(self):
        handle = self.handle()
        return handle is not None and hasattr(handle, 'run')

    def set_paused(self, paused):
        # run is read-only in the 3.12 Python binding. A panel command changes
        # the host, while subsequent native run changes are observed as events.
        handle = self.handle()
        if handle is not None and hasattr(handle, 'run'): self.last_run = bool(handle.run)

    def paused(self, fallback):
        handle = self.handle()
        if handle is not None and hasattr(handle, 'run'):
            run = bool(handle.run)
            if run != self.last_run:
                self.last_run = run
                # The panel may have toggled the host independently since the
                # last native event. Space must toggle that actual host state,
                # not reapply an out-of-date native run indicator.
                return not fallback
        return fallback

    def speed(self):
        handle = self.handle()
        if handle is not None and hasattr(handle, 'real_time_index'):
            from mujoco.viewer import PERCENT_REALTIME
            index = max(0, min(len(PERCENT_REALTIME)-1, handle.real_time_index))
            return PERCENT_REALTIME[index]/100.
        return 1.


class SimulationWorker(threading.Thread):
    """Own commands and physics; release the viewer lock after <=4 physics steps.

    Tk only reads snapshots. Full viewer sync runs at 60Hz, panel snapshots at
    10Hz, and the 1kHz physics clock is driven by accumulated wall time.
    """
    def __init__(self, sim, viewer=None):
        super().__init__(name='cabinet-physics', daemon=True)
        self.sim, self.viewer = sim, viewer
        self.native = NativeControls(viewer)
        self.commands = queue.Queue()
        self.updates = queue.Queue(maxsize=1)
        self.stop_event = threading.Event()
        self.message = '已启动：自由视角，柜门开启90°'
        self.running = True
        self.command_count = 0
        self.metrics = dict(physics_steps=0, sync_count=0, lock_max_ms=0., lock_wait_max_ms=0., sync_max_ms=0.,
                            command_latency_max_ms=0., slow_commands=[], measured_realtime=0.)
        self.started_wall = time.monotonic()
        self.last_rate_wall = self.started_wall; self.last_rate_sim = self.sim.data.time
        self.native.set_paused(False)
        self.publish()

    def lock(self): return self.viewer.lock() if self.viewer is not None else nullcontext()

    def submit(self, instance, action):
        self.commands.put((instance, action, time.monotonic()))

    def publish(self):
        update = dict(state=self.sim.state(), message=self.message, running=self.running,
                      metrics=dict(self.metrics), command_count=self.command_count)
        try: self.updates.get_nowait()
        except queue.Empty: pass
        self.updates.put_nowait(update)

    def dispatch(self, instance, action):
        if instance == 'host':
            if action == 'reset':
                self.sim.reset(); self.native.set_paused(False)
            elif action == 'pause':
                self.sim.set_paused(not self.sim.paused); self.native.set_paused(self.sim.paused)
            else: raise ValueError('未知整柜操作')
        elif instance == 'camera':
            if self.viewer is not None: set_free_camera(self.viewer.cam, action)
        else:
            self.sim.command(instance, action)
        self.message = f'{instance}：{action}'

    def accept_commands(self):
        changed = False
        while True:
            try: instance, action, submitted = self.commands.get_nowait()
            except queue.Empty: return changed
            latency_ms = (time.monotonic()-submitted)*1000
            self.metrics['command_latency_max_ms'] = max(self.metrics['command_latency_max_ms'],latency_ms)
            if latency_ms > 100:
                self.metrics['slow_commands'] = (self.metrics['slow_commands'] + [dict(
                    instance=instance,action=action,latency_ms=latency_ms,
                    wall_since_start=time.monotonic()-self.started_wall)])[-10:]
            try: self.dispatch(instance, action)
            except (ValueError, RuntimeError, KeyError) as exc:
                self.message = str(exc)
                print(f'Cabinet command rejected: {exc}', flush=True)
            self.command_count += 1; changed = True

    def synchronize(self):
        if self.viewer is None: return
        # sync locks internally; do not hold the rendering mutex around Tk work.
        before = self.sim.data.time
        t = time.monotonic()
        self.viewer.sync()
        self.metrics['sync_max_ms'] = max(self.metrics['sync_max_ms'],(time.monotonic()-t)*1000)
        self.metrics['sync_count'] += 1
        # Native reset/backspace rewinds data; rebuild controller bookkeeping,
        # including lamp states and the declared initial-open pose.
        if self.sim.data.time < before-1e-9:
            with self.lock():
                paused = self.native.paused(self.sim.paused)
                self.sim.reset(); self.sim.set_paused(paused)
                self.message = 'MuJoCo复位：已恢复默认开门状态'

    def run(self):
        previous = time.monotonic(); accumulator = 0.
        next_sync = previous; next_publish = previous
        try:
            while not self.stop_event.is_set() and (self.viewer is None or self.viewer.is_running()):
                now = time.monotonic()
                elapsed = max(0., now-previous); previous = now
                with self.lock():
                    lock_start = time.monotonic()
                    self.metrics['lock_wait_max_ms'] = max(self.metrics['lock_wait_max_ms'],(lock_start-now)*1000)
                    paused = self.native.paused(self.sim.paused)
                    if paused != self.sim.paused: self.sim.set_paused(paused)
                    changed = self.accept_commands()
                    if self.sim.paused:
                        accumulator = 0.
                    else:
                        accumulator = min(.05, accumulator+elapsed*self.native.speed())
                        steps = min(4, int(accumulator/self.sim.dt))
                        if steps:
                            self.sim.step(steps, refresh=False)
                            accumulator -= steps*self.sim.dt
                            self.metrics['physics_steps'] += steps
                    sync_due = now >= next_sync
                    if sync_due:
                        mujoco.mj_forward(self.sim.model, self.sim.data)
                    lock_ms=(time.monotonic()-lock_start)*1000
                    self.metrics['lock_max_ms']=max(self.metrics['lock_max_ms'],lock_ms)
                    if now-self.last_rate_wall >= .5:
                        self.metrics['measured_realtime'] = (self.sim.data.time-self.last_rate_sim)/(now-self.last_rate_wall)
                        self.last_rate_wall=now;self.last_rate_sim=self.sim.data.time
                    if changed or now >= next_publish:
                        self.publish(); next_publish = now+.1
                if sync_due:
                    self.synchronize(); next_sync = time.monotonic()+1/60
                # Yield every cycle, including catch-up; never monopolize UI lock.
                self.stop_event.wait(.0005 if accumulator >= self.sim.dt else .001)
        except Exception as exc:
            self.message = '仿真停止：'+str(exc)
        finally:
            with self.lock():
                self.sim.set_paused(True)
                self.running = False; self.publish()
