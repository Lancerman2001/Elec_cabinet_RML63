#!/usr/bin/env python3
"""Cabinet controls and MuJoCo viewer, with keyboard and headless entrypoints.

English labels keep the panel readable with the bundled Python's bitmap Tk.
Original protocol action strings remain unchanged in the backend API.
"""
from __future__ import annotations

import argparse
import json
import math
import queue
import threading
import time
from pathlib import Path

from cabinet_backend import CabinetSimulation
from viewer_runtime import NativeControls, SimulationWorker, set_free_camera
BUTTON_CONFIG=json.loads((Path(__file__).resolve().parents[2]/'fugui_gazebo/config/button.yaml').read_text())

KEYS = {
    'O': ('door', 'open'), '1': ('door', 'door_return'),
    '2': ('door', 'handle_return'), '3': ('door', 'handle_latch'),
    'L': ('yaban_r1_c1', '松旋钮'), 'W': ('yaban_r1_c1', '压板退'),
    'I': ('yaban_r1_c1', '压板投'), 'T': ('yaban_r1_c1', '紧旋钮'),
    'Y': ('yaban_r1_c1', '初始化'),
    'A': ('selector_01', 'AUTO'), 'S': ('selector_01', 'STOP'), 'D': ('selector_01', 'MANUAL'),
    'N': ('breaker_10', 'on'), 'F': ('breaker_10', 'off'),
    'P': ('reset_01', 'press'), 'R': ('reset_01', 'release'),
    ' ': ('host', 'pause'), '0': ('host', 'reset'),
    '4': ('camera', 'front'), '5': ('camera', 'oblique'),
    '6': ('camera', 'plates'), '7': ('camera', 'breakers'), '8': ('camera', 'controls'),
}
HELP = '''Door: O open; 1 return door; 2 return handle; 3 latch handle
Plate: L loosen knob; W withdraw; I insert; T tighten knob; Y initialize
Selector: A automatic; S stop; D manual
Breaker: N on; F off. Reset button: P press once; R release
Space pause/resume; 0 reset cabinet. Cameras: 4 front; 5 oblique; 6 plates; 7 breakers; 8 controls.
Select any component instance in the panel. Mouse: left rotate, right pan, wheel zoom.
Tab toggles native panels; F1 shows native shortcuts.'''

ACTION_LABELS = {'初始化': 'initialize', '松旋钮': 'loosen knob', '压板退': 'withdraw plate',
                 '压板投': 'insert plate', '紧旋钮': 'tighten knob'}


def display_message(message):
    known = {'已启动：自由视角，柜门开启90°': 'Ready: free camera, door open 90 deg',
             'MuJoCo复位：已恢复默认开门状态': 'Native reset: restored the open cabinet'}
    if message in known:
        return known[message]
    result = message.replace('：', ': ')
    for original, english in ACTION_LABELS.items():
        result = result.replace(original, english)
    return result if result.isascii() else 'Command rejected or simulation stopped; see terminal for details'


class Application:
    def __init__(self, use_viewer=True, use_panel=True):
        self.sim = CabinetSimulation()
        self.commands = queue.Queue()
        self.viewer = None; self.root = None; self.running = True
        self.viewer_threads = []
        self.message = 'Ready: door open 90 deg, all breakers off'
        self.buttons = []
        self.selections = {}
        self.selected_instances = {}
        if use_panel:
            from tk_runtime import prepare_tk
            prepare_tk()
            self.build_panel()
        if use_viewer:
            import mujoco_compat
            threads_before = set(threading.enumerate())
            self.viewer = mujoco_compat.launch_passive(self.sim.model, self.sim.data,
                                                       key_callback=self.key, show_left_ui=True, show_right_ui=True)
            # MuJoCo 3.12 Handle.close requests exit without joining its daemon
            # render thread. Retain the newly created threads until native GL
            # teardown completes, before Python/GLFW interpreter finalization.
            self.viewer_threads = [t for t in threading.enumerate() if t not in threads_before]
            with self.viewer.lock():
                self.camera('front')
                self.viewer.opt.geomgroup[3] = 0  # retain physics, hide duplicate collision drawings
        self.worker = SimulationWorker(self.sim, self.viewer)
        self.worker.commands = self.commands
        self.latest = self.worker.updates.get_nowait()
        self.worker.start()
        print(HELP, flush=True)

    def key(self, code):
        try: key = chr(code).upper()
        except ValueError: return
        # Space already toggles the native viewer run flag; avoid double-toggle.
        if key == ' ' and self.viewer is not None and NativeControls(self.viewer).supported:
            return
        if key in KEYS: self.submit(*KEYS[key])

    def camera(self, name):
        if self.viewer is not None:
            set_free_camera(self.viewer.cam, name)

    def submit(self, instance, action):
        instance = self.selected_instances.get(instance, instance)
        self.commands.put((instance, action, time.monotonic()))

    def dispatch(self, instance, action):
        self.submit(instance, action)

    def tick(self):
        if not self.running: return
        changed = False
        while True:
            try: self.latest = self.worker.updates.get_nowait(); changed = True
            except queue.Empty: break
        if changed:
            self.message = display_message(self.latest['message']); self.refresh()
        if not self.latest['running'] or (self.viewer is not None and not self.viewer.is_running()):
            self.close(); return
        if self.root is not None and self.running: self.root.after(33, self.tick)

    def build_panel(self):
        import tkinter as tk
        from tkinter import font, ttk
        self.root = root = tk.Tk()
        root.title('Electrical Cabinet - MuJoCo Control')
        root.geometry('820x720'); root.minsize(800, 680)
        families = set(font.families(root))
        family = next((x for x in ('Noto Sans CJK SC', 'WenQuanYi Zen Hei', 'Microsoft YaHei') if x in families), 'sans')
        font.nametofont('TkDefaultFont').configure(family=family, size=11)
        root.option_add('*Font', (family, 11))
        outer = tk.Frame(root, padx=18, pady=14); outer.pack(fill='both', expand=True)
        tk.Label(outer, text='Complete electrical cabinet - controls', font=(family, 18, 'bold')).pack(anchor='w')
        tk.Label(outer, text='24 plates | 3 selectors | 10 breakers | 3 reset buttons | 10 indicator lamps',
                 fg='#465566').pack(anchor='w', pady=(3, 12))
        self.labels = {}
        groups = [
            ('door', 'Door: operate in sequence', [('Open O', 'open'), ('Return door 1', 'door_return'), ('Return handle 2', 'handle_return'), ('Latch handle 3', 'handle_latch')]),
            ('yaban_r1_c1', 'Pressure plate: select any of 24 instances', [('Initialize Y', '初始化'), ('Loosen L', '松旋钮'), ('Withdraw W', '压板退'), ('Insert I', '压板投'), ('Tighten T', '紧旋钮')]),
            ('selector_01', 'Selector: select any of 3 instances', [('Auto A', 'AUTO'), ('Stop S', 'STOP'), ('Manual D', 'MANUAL')]),
            ('breaker_10', 'Breaker: select any of 10 instances', [('On N', 'on'), ('Off F', 'off')]),
            ('reset_01', f"Reset button: {BUTTON_CONFIG['default_depth']*1000:g} mm, hold 0.35 s", [('Press P', 'press'), ('Release R', 'release')]),
        ]
        for instance, title, actions in groups:
            frame = tk.LabelFrame(outer, text=title, padx=9, pady=5); frame.pack(fill='x', pady=4)
            if instance != 'door':
                kind = {'yaban': 'yaban', 'selector': 'selector', 'breaker': 'breaker', 'reset': 'reset'}[instance.split('_')[0]]
                names = [item['name'] for item in self.sim.instances if item['kind'] == kind]
                selected = tk.StringVar(value=instance)
                self.selections[instance] = selected
                self.selected_instances[instance] = instance
                choice = ttk.Combobox(frame, textvariable=selected, values=names, state='readonly', width=28)
                choice.pack(anchor='w', pady=(0, 4))
                choice.bind('<<ComboboxSelected>>',
                            lambda _event, key=instance, value=selected:
                            self.selected_instances.__setitem__(key, value.get()))
            row = tk.Frame(frame); row.pack(fill='x')
            for label, action in actions:
                button = tk.Button(row, text=label, command=lambda i=instance, a=action: self.submit(i, a))
                button.pack(side='left', padx=(0, 5))
                self.buttons.append((instance, action, button))
            self.labels[instance] = tk.StringVar()
            tk.Label(frame, textvariable=self.labels[instance], anchor='w', fg='#294d66').pack(fill='x')
        row = tk.Frame(outer); row.pack(fill='x', pady=8)
        for text, instance, action in [('Pause/resume', 'host', 'pause'), ('Reset all', 'host', 'reset'),
                                       ('Front', 'camera', 'front'), ('Oblique', 'camera', 'oblique'),
                                       ('Plates', 'camera', 'plates'), ('Breakers', 'camera', 'breakers')]:
            tk.Button(row, text=text, command=lambda i=instance,a=action: self.submit(i,a)).pack(side='left', padx=3)
        self.status = tk.StringVar(value=self.message)
        tk.Label(outer, textvariable=self.status, anchor='w', wraplength=660).pack(fill='x')
        root.protocol('WM_DELETE_WINDOW', self.close)

    def refresh(self):
        if self.root is None: return
        s = self.latest['state']
        ds = s['door']
        self.labels['door'].set(f"{ds['status']} | door {math.degrees(ds['pose'][3]):.1f} deg"+(' | moving' if ds['busy'] else ''))
        for name in self.labels:
            if name == 'door': continue
            selected = self.selections[name].get() if name in self.selections else name
            c = s['components'][selected]
            if name.startswith('yaban'):
                status = 'withdrawn' if abs(c['blade']) > .001 else 'inserted'
                text = f"{status} | knob {math.degrees(c['knob']):.1f} deg | blade {math.degrees(c['blade']):.1f} deg"
            elif name.startswith('selector'):
                text = f"{c['mode']} | {math.degrees(c['angle']):.1f} deg"
            elif name.startswith('breaker'):
                text = ('ON / red lamp on' if c['powered'] else 'OFF / red lamp off')+f" | {math.degrees(c['angle']):.1f} deg"
            else:
                text = f"Travel {c['depth_mm']:.2f} mm | count {c['count']} | "+('green LED on' if c['led_on'] else 'green LED off')
            self.labels[name].set(text+(' | moving' if c['busy'] else '')+(' | error' if c['error'] else ''))
        for instance, action, button in self.buttons:
            selected = self.selections[instance].get() if instance in self.selections else instance
            c = ds if instance == 'door' else s['components'][selected]
            enabled = not s['paused'] and not c['busy']
            if instance.startswith('yaban'): enabled &= action in c['allowed']
            if instance == 'door':
                enabled &= ds['status'] == {'open':'CLOSED','door_return':'OPEN','handle_return':'DOOR_RETURNED','handle_latch':'LATCHED_HANDLE_OUT'}[action]
            if action == 'release': enabled = True
            button.configure(state='normal' if enabled else 'disabled')
        rate = self.latest['metrics']['measured_realtime']
        self.status.set(f"Simulation {s['time']:.2f}s | {'paused' if s['paused'] else f'{rate:.2f}x realtime'} | {self.message}")

    def close(self):
        if not self.running: return
        self.running = False
        self.worker.stop_event.set()
        self.worker.join(timeout=3.)
        if self.viewer is not None: self.viewer.close()
        for thread in self.viewer_threads:
            thread.join(timeout=3.)
        if self.root is not None: self.root.destroy()

    def run(self, seconds=None):
        if self.root is not None:
            if seconds is not None: self.root.after(round(seconds*1000), self.close)
            self.root.after(0, self.tick); self.root.mainloop()
        else:
            until = time.monotonic()+seconds if seconds is not None else math.inf
            try:
                while self.running and time.monotonic()<until:
                    self.tick(); time.sleep(.01)
            finally: self.close()


def main():
    p = argparse.ArgumentParser(description=__doc__)
    p.add_argument('--keyboard-only', action='store_true', help='viewer and keyboard only')
    p.add_argument('--no-viewer', action='store_true', help='control panel only')
    p.add_argument('--headless', action='store_true', help='run without windows and output state JSON')
    p.add_argument('--seconds', type=float, help='exit after the requested duration')
    args = p.parse_args()
    if args.seconds is not None and (not math.isfinite(args.seconds) or args.seconds <= 0): p.error('--seconds must be positive and finite')
    if args.no_viewer and args.keyboard_only: p.error('--no-viewer and --keyboard-only cannot be combined')
    if args.headless:
        s=CabinetSimulation(); s.run_for(args.seconds or 1.)
        print(json.dumps(s.state(), ensure_ascii=False, indent=2)); return
    import sys
    import mujoco_compat as mujoco
    print(f'Python: {sys.executable}\nMuJoCo: {mujoco.__version__}', flush=True)
    app = Application(not args.no_viewer, not args.keyboard_only)
    app.run(args.seconds)


if __name__ == '__main__': main()
