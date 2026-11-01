"""Physics owner and immutable feedback snapshots, shared by both launch modes."""
from collections import deque
import threading
import time
import mujoco


class SimulationConnection:
    def __init__(self, simulation, start=True):
        self.simulation=simulation
        self.lock=threading.RLock()
        self.done=threading.Event()
        self._sample_lock=threading.Lock()
        self._queue_lock=threading.Lock()
        self._commands=deque()
        self.sample=simulation.status(); self.received=time.monotonic()
        self.thread=None
        self._anchor_wall=time.monotonic(); self._anchor_sim=float(simulation.data.time)
        self._last_time=float(simulation.data.time)
        if start: self.start()

    def start(self):
        self.thread=threading.Thread(target=self._run,name='selector-physics',daemon=True)
        self.thread.start()

    def enqueue(self, callback):
        with self._queue_lock:
            # A local producer cannot grow an unbounded queue while the viewer
            # is loading. Old timestamped commands will be rejected on receipt.
            if len(self._commands)<256: self._commands.append(callback)

    def drain_commands(self):
        with self._queue_lock:
            commands=list(self._commands); self._commands.clear()
        for callback in commands: callback()
        return bool(commands)

    def publish(self):
        state=self.simulation.status(); now=time.monotonic()
        with self._sample_lock: self.sample=state; self.received=now

    def feedback(self):
        # Network/GUI never wait for a rendering or MuJoCo model lock.
        with self._sample_lock: return self.sample,self.received

    def command_ready(self):
        state,received=self.feedback()
        return not self.done.is_set() and not state.paused and time.monotonic()-received<.5

    def send(self, angle, dragging=False):
        self.enqueue(lambda:self.simulation.command(angle,dragging))

    def apply_pause(self, paused):
        self.simulation.set_paused(paused)
        self.reanchor()

    def apply_reset(self):
        self.simulation.reset(); self.reanchor()

    def pause(self, paused): self.enqueue(lambda:self.apply_pause(paused))
    def reset(self): self.enqueue(self.apply_reset)

    def reanchor(self):
        self._anchor_wall=time.monotonic(); self._anchor_sim=float(self.simulation.data.time)

    def advance(self, speed=1.0):
        sim=self.simulation; now=time.monotonic()
        if sim.paused:
            self.reanchor(); return False
        if sim.data.time<self._last_time:
            self.reanchor()
        target=self._anchor_sim+(now-self._anchor_wall)*speed
        budget=now+.004; stepped=False
        while sim.data.time<target and time.monotonic()<budget:
            if not sim.before_step(): break
            mujoco.mj_step(sim.model,sim.data); sim.after_step(); stepped=True
        self._last_time=float(sim.data.time)
        return stepped

    def _run(self):
        self.reanchor()
        while not self.done.wait(.001):
            with self.lock:
                commanded=self.drain_commands(); self.advance()
                if commanded or time.monotonic()-self.received>=.02: self.publish()

    def close(self):
        self.done.set()
        if self.thread: self.thread.join(timeout=3)
