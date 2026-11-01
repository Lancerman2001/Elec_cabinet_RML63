#!/usr/bin/env python3
"""MuJoCo viewer and Chinese pushbutton control panel; run in conda env ros."""
from __future__ import annotations
import argparse
import json
from contextlib import nullcontext
from dataclasses import dataclass
import queue
import threading
import time
from tk_runtime import prepare_tk
prepare_tk()  # Must precede tkinter: Conda's bitmap-only Tk cannot render CJK.
import tkinter as tk
from tkinter import font as tkfont
from fugui_backend import FuguiBackend, MODEL_PATH, State


def configure_camera(camera):
    # MuJoCo azimuth describes the viewing direction, not the camera position.
    # 148 degrees places the observer on the button's +X front side.
    camera.lookat[:] = [.010, 0, .12]
    camera.distance = .19; camera.azimuth = 148; camera.elevation = -20


@dataclass(frozen=True)
class Update:
    state: State | None
    running: bool
    ack: int
    error: str = ''


class Worker(threading.Thread):
    """Sole owner of physics; commands are ordered and snapshots are immutable."""
    def __init__(self, use_viewer=True, model_path=MODEL_PATH):
        super().__init__(name='fugui-mujoco', daemon=True)
        self.use_viewer = use_viewer; self.model_path = model_path
        self.commands = queue.Queue(); self.updates = queue.Queue(maxsize=1)
        self.stop_event = threading.Event(); self.ready = threading.Event()
        self.command_lock = threading.Lock(); self.sequence = 0; self.ack = 0
        self.backend = None; self.viewer = None; self.error = ''

    def submit(self, kind, *args):
        with self.command_lock:
            self.sequence += 1; self.commands.put((self.sequence, kind, args)); return self.sequence

    def emit(self, running=True):
        update = Update(self.backend.state() if self.backend else None, running, self.ack, self.error)
        try: self.updates.get_nowait()
        except queue.Empty: pass
        self.updates.put_nowait(update)

    def accept_commands(self):
        while True:
            try: sequence, kind, args = self.commands.get_nowait()
            except queue.Empty: return
            try:
                if kind == 'press': self.backend.press(*args)
                elif kind == 'release': self.backend.release()
                elif kind == 'pause': self.backend.set_paused(not self.backend.paused)
                elif kind == 'reset': self.backend.reset()
                elif kind == 'view':
                    if self.viewer is not None:
                        self.viewer.cam.type = self.backend.mj.mjtCamera.mjCAMERA_FIXED
                        self.viewer.cam.fixedcamid = self.backend.model.camera(args[0]).id
                else: raise ValueError('未知操作：'+kind)
                self.error = ''
            except (ValueError, RuntimeError) as exc: self.error = str(exc)
            self.ack = sequence

    def loop(self, viewer):
        self.viewer = viewer; self.ready.set()
        previous = time.monotonic(); accumulator = 0.; last_emit = 0.
        while not self.stop_event.is_set() and (viewer is None or viewer.is_running()):
            now = time.monotonic(); elapsed = min(.05, max(0., now-previous)); previous = now
            with viewer.lock() if viewer is not None else nullcontext():
                self.accept_commands()
                if self.backend.paused: accumulator = 0.
                else:
                    accumulator += elapsed; steps = int(accumulator/self.backend.dt)
                    if steps: self.backend.step(steps); accumulator -= steps*self.backend.dt
                if now-last_emit >= .025: self.emit(); last_emit = now
            if viewer is not None: viewer.sync()
            self.stop_event.wait(.005)

    def run(self):
        try:
            self.backend = FuguiBackend(self.model_path)
            if self.use_viewer:
                import mujoco.viewer
                with mujoco.viewer.launch_passive(self.backend.model, self.backend.data,
                        key_callback=lambda key: self.submit('pause') if key == 32 else None,
                        show_left_ui=False, show_right_ui=False) as viewer:
                    with viewer.lock():
                        configure_camera(viewer.cam)
                        viewer.opt.geomgroup[3] = 0
                    self.loop(viewer)
            else: self.loop(None)
        except Exception as exc: self.error = str(exc)
        finally:
            if self.backend: self.backend.release()
            self.viewer = None; self.ready.set(); self.emit(running=False)


class FuguiGUI:
    def __init__(self, root=None, use_viewer=True, model_path=MODEL_PATH):
        self.root = root or tk.Tk(); self.closed = False; self.held = False
        self.latest = Update(None, True, 0); self.pending = 0; self.received = 0.; self.previous_phase = None
        self.worker = Worker(use_viewer, model_path)
        self.root.title('Signal Reset Button - MuJoCo Control')
        self.root.geometry('570x790'); self.root.resizable(False, False); self.root.configure(bg='#eef2f0')
        available = set(tkfont.families(self.root))
        self.font_family = next((f for f in ('Noto Sans CJK SC', 'Microsoft YaHei', 'WenQuanYi Micro Hei',
            'WenQuanYi Zen Hei', 'Noto Serif CJK SC', 'SimHei') if f in available),
            tkfont.nametofont('TkDefaultFont').actual('family'))
        for name in ('TkDefaultFont', 'TkTextFont', 'TkMenuFont', 'TkHeadingFont', 'TkCaptionFont'):
            tkfont.nametofont(name).configure(family=self.font_family, size=11)
        self.root.option_add('*Font', 'TkDefaultFont')
        self.button_config = json.loads((MODEL_PATH.parent.parent/'config/button.yaml').read_text())
        self.connection = tk.StringVar(value='正在启动 MuJoCo…'); self.depth = tk.DoubleVar(value=self.button_config['default_depth']*1000)
        self.depth_text = tk.StringVar(value='实际按入  0.00 mm'); self.state_text = tk.StringVar(value='等待仿真')
        self.count_text = tk.StringVar(value='有效复归  0 次'); self.message = tk.StringVar(value='点击单次复归，按钮按入后自动回弹。')
        tk.Label(self.root, text='保护信号复归按钮', font=(self.font_family, 20, 'bold'), bg='#eef2f0', fg='#183c30').pack(pady=(18, 2))
        tk.Label(self.root, textvariable=self.connection, bg='#eef2f0', fg='#607168').pack()
        self.canvas = tk.Canvas(self.root, width=510, height=270, bg='#eef2f0', highlightthickness=0); self.canvas.pack(pady=8)
        tk.Label(self.root, textvariable=self.state_text, bg='#eef2f0', fg='#1c5f44', font=(self.font_family, 14, 'bold')).pack()
        row = tk.Frame(self.root, bg='#eef2f0'); row.pack(pady=8)
        tk.Label(row, textvariable=self.depth_text, bg='#eef2f0').pack(side='left', padx=18)
        tk.Label(row, textvariable=self.count_text, bg='#eef2f0').pack(side='left', padx=18)
        control = tk.Frame(self.root, bg='white', padx=18, pady=10); control.pack(fill='x', padx=24)
        tk.Label(control, text='目标按入深度（mm）', bg='white').pack(anchor='w')
        self.slider = tk.Scale(control, from_=0, to=self.button_config['travel_max']*1000, resolution=.1, variable=self.depth,
                               orient='horizontal', length=465, bg='white', highlightthickness=0); self.slider.pack()
        cfg = self.button_config
        tk.Label(control, text=f"有效区间 {cfg['valid_min']*1000:g}–{cfg['valid_max']*1000:g} mm  ·  默认 {cfg['default_depth']*1000:g} mm  ·  亮灯 {cfg['lamp_duration']:g} 秒", bg='white', fg='#64746b', font=(self.font_family, 10)).pack()
        actions = tk.Frame(self.root, bg='#eef2f0'); actions.pack(pady=(14, 8))
        self.press_button = tk.Button(actions, text='单次复归', bg='#13734c', fg='white',
            activebackground='#219363', activeforeground='white', width=12, command=self.press)
        self.press_button.pack(side='left', padx=5)
        self.hold_button = tk.Button(actions, text='按住测试', width=10); self.hold_button.pack(side='left', padx=5)
        self.release_button = tk.Button(actions, text='释放', width=8, command=self.release); self.release_button.pack(side='left', padx=5)
        self.hold_button.bind('<ButtonPress-1>', self.hold)
        self.root.bind('<ButtonRelease-1>', self.release_hold, add='+')
        self.root.bind('<FocusOut>', lambda _: self.root.after(20, self.check_focus), add='+')
        row = tk.Frame(self.root, bg='#eef2f0'); row.pack(pady=(0, 8))
        self.pause_button = tk.Button(row, text='暂停仿真', width=13, command=self.toggle_pause); self.pause_button.pack(side='left', padx=5)
        self.reset_button = tk.Button(row, text='初始化 / 清零', width=15, command=lambda: self.submit('reset')); self.reset_button.pack(side='left', padx=5)
        views = tk.Frame(self.root, bg='#eef2f0'); views.pack(pady=(0, 8)); self.view_buttons = []
        if use_viewer:
            for name, text in [('front', '正视'), ('oblique', '斜视'), ('side', '侧视')]:
                button = tk.Button(views, text=text, width=8, command=lambda view=name: self.submit('view', view))
                button.pack(side='left', padx=4); self.view_buttons.append(button)
        tk.Label(self.root, textvariable=self.message, wraplength=510, bg='#eef2f0', fg='#52685c', font=(self.font_family, 10)).pack(padx=20)
        self.root.protocol('WM_DELETE_WINDOW', self.close)
        self.worker.start(); self._tick()

    def submit(self, kind, *args):
        if self.closed: return
        self.pending = self.worker.submit(kind, *args)
        for control in (self.press_button, self.hold_button, self.slider): control.configure(state='disabled')

    def press(self):
        if str(self.press_button['state']) == 'disabled': return
        self.submit('press', self.depth.get(), .35, True); self.message.set('正在执行单次按压…')

    def hold(self, _event=None):
        if str(self.hold_button['state']) != 'disabled':
            self.held = True; self.submit('press', self.depth.get(), 0., False)
            self.message.set('保持按压中；松开鼠标后回弹。')
        return 'break'

    def release(self):
        self.held = False; self.submit('release'); self.message.set('已释放，按钮将弹簧回位。')

    def release_hold(self, _event=None):
        if self.held: self.release()

    def check_focus(self):
        if not self.closed and self.held and self.root.focus_displayof() is None: self.release()

    def toggle_pause(self):
        self.held = False; self.submit('pause')

    def _draw(self, depth, lit):
        c = self.canvas; c.delete('all')
        c.create_rectangle(45, 10, 255, 258, fill='#bdb68e', outline='#a59e78', width=3)
        c.create_rectangle(62, 29, 238, 67, fill='#f3f1e8', outline='#9d9674')
        c.create_text(150, 48, text='保护信号复归按钮', font=(self.font_family, 11))
        c.create_oval(92, 110, 208, 226, fill='#cdd2ce', outline='#818c86', width=3)
        c.create_oval(100, 118, 200, 218, fill='#26f45c' if lit else '#063c28', outline='#203e30', width=3)
        c.create_text(150, 244, text='正视状态', fill='#4b5140', font=(self.font_family, 9))
        c.create_text(386, 24, text='按压行程示意', fill='#4a6357', font=(self.font_family, 11))
        c.create_rectangle(320, 65, 338, 203, fill='#bdb68e', outline='#9c987e')
        c.create_rectangle(338, 113, 457, 126, fill='#afb9b3', outline='')
        c.create_rectangle(338, 174, 457, 187, fill='#afb9b3', outline='')
        face = 457-max(0, min(self.button_config['travel_max']*1000, depth))*9
        c.create_rectangle(face-12, 127, face, 173, fill='#26f45c' if lit else '#063c28', outline='#183f2c')
        c.create_line(462, 216, 390, 216, arrow='last', fill='#446353', width=2)
        c.create_text(394, 243, text='← 向内按入 %.2f mm' % depth, fill='#4a6357', font=(self.font_family, 10))

    def _tick(self):
        if self.closed: return
        try:
            self.latest = self.worker.updates.get_nowait(); self.received = time.monotonic()
        except queue.Empty: pass
        update = self.latest; state = update.state
        fresh = bool(self.received) and time.monotonic()-self.received < .75
        connected = fresh and update.running and state is not None
        pending = update.ack < self.pending
        ready = connected and not state.paused and not state.active and state.depth_mm < .5 and not pending
        for control in (self.press_button, self.hold_button, self.slider): control.configure(state='normal' if ready else 'disabled')
        for control in (self.release_button, self.pause_button, self.reset_button, *self.view_buttons):
            control.configure(state='normal' if connected else 'disabled')
        self.pause_button.configure(text='恢复仿真' if state and state.paused else '暂停仿真')
        self.connection.set(('MuJoCo · 已暂停' if state.paused else 'MuJoCo · 实时物理仿真') if connected else ('MuJoCo 已停止' if not update.running else '正在连接 MuJoCo…'))
        if not fresh and self.held: self.release()
        if state:
            self.depth_text.set('实际按入  %.2f mm' % max(0, state.depth_mm)); self.count_text.set('有效复归  %d 次' % state.reset_count)
            phases = {'IDLE': '待机 · 墨绿色灭灯', 'PRESSING': '正在按入', 'RETURNING': '弹簧回弹中',
                      'HOLDING': '保持按压 · 本次已复归，灯已熄灭' if not state.armed else '保持按压 · 未达到有效行程',
                      'TARGET_TIMEOUT': '未到达目标，请释放后重试'}
            self.state_text.set('仿真已暂停' if state.paused else ('复归有效 · 亮绿色' if state.led_on else phases.get(state.phase, state.phase)))
            if state.phase == 'IDLE' and self.previous_phase not in (None, 'IDLE'):
                self.message.set('按钮已回位，可再次按压。')
            self.previous_phase = state.phase
            self._draw(state.depth_mm, state.led_on)
        else: self._draw(0, False)
        if update.error: self.message.set(update.error)
        elif not update.running: self.message.set('三维窗口已关闭。关闭此面板后可重新启动。')
        self.root.after(25, self._tick)

    def close(self):
        if self.closed: return
        self.closed = True; self.worker.stop_event.set(); self.worker.join(timeout=3)
        self.root.destroy()

    def run(self):
        try: self.root.mainloop()
        finally: self.close()


def main():
    parser = argparse.ArgumentParser(description='MuJoCo signal reset button GUI (conda activate ros).')
    parser.add_argument('--no-viewer', action='store_true', help='Run physics and control panel without the 3D window.')
    parser.add_argument('--model', default=str(MODEL_PATH), help='Path to fugui MJCF XML.')
    args = parser.parse_args()
    FuguiGUI(use_viewer=not args.no_viewer, model_path=args.model).run()


if __name__ == '__main__': main()
