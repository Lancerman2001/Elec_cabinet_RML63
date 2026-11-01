#!/usr/bin/env python3
"""Complete cabinet and arm viewer with basic control and setup panels."""
from __future__ import annotations

import argparse
from contextlib import nullcontext
import json
import math
from pathlib import Path
import time

import numpy as np

from interaction_environment import TotalCabinetEnv
from environment import HOME_JOINTS, HOME_GRIPPER, mj
from arm_control import ControlPanel
from camera_depth import DepthPreview
from viewer_runtime import NativeControls, set_free_camera


VIEW_NAMES = ("scene", "front", "oblique", "plates", "breakers", "controls",
              "d435_rgb", "d435_depth", "global_default")


def json_safe(value):
    if isinstance(value, np.ndarray):
        return value.tolist()
    if isinstance(value, np.generic):
        return value.item()
    if isinstance(value, dict):
        return {name: json_safe(item) for name, item in value.items()}
    if isinstance(value, (list, tuple)):
        return [json_safe(item) for item in value]
    return value


def set_view(viewer, name, env):
    if name == "scene":
        viewer.cam.type = mj.mjtCamera.mjCAMERA_FREE
        viewer.cam.fixedcamid = -1
        viewer.cam.lookat[:] = (0.7, -0.1, 1.1)
        viewer.cam.distance = 3.6
        viewer.cam.azimuth, viewer.cam.elevation = 145.0, -15.0
    elif name in ("front", "oblique", "plates", "breakers", "controls"):
        set_free_camera(viewer.cam, name)
    else:
        viewer.cam.type = mj.mjtCamera.mjCAMERA_FIXED
        viewer.cam.fixedcamid = env._id(mj.mjtObj.mjOBJ_CAMERA, name)


class CabinetPanel(ControlPanel):
    def __init__(self, env, viewer=None, view="scene"):
        from tk_runtime import prepare_tk
        prepare_tk()
        import tkinter as tk
        from tkinter import ttk

        root = tk.Tk()
        tabs = ttk.Notebook(root)
        cabinet, arm = ttk.Frame(tabs, padding=10), ttk.Frame(tabs, padding=10)
        tabs.add(cabinet, text="Cabinet and scene")
        tabs.add(arm, text="Arm and gripper")
        tabs.pack(fill="both", expand=True)
        super().__init__(env, root=root, parent=arm)
        self.viewer = viewer
        self.root.title("Complete Electrical Cabinet + RML63II - MuJoCo")
        self.root.geometry("860x720")
        ttk.Label(cabinet, text="Complete electrical cabinet", font=("sans", 18, "bold")).pack(anchor="w")
        ttk.Label(cabinet, text="24 pressure plates | 3 selectors | 10 breakers | 3 reset buttons | 10 indicator lamps").pack(anchor="w", pady=5)
        actions = ttk.Frame(cabinet)
        actions.pack(fill="x", pady=5)
        for text, command in (
            ("Pause / resume", lambda: env.set_paused(not env.paused)),
            ("Reset scene", self.reset),
            ("Set door open", lambda: self.door_setup("open")),
            ("Set door closed", lambda: self.door_setup("closed")),
        ):
            ttk.Button(actions, text=text, command=command).pack(side="left", padx=2)
        ttk.Label(cabinet, text="Door and table placement buttons set the scene instantly.").pack(anchor="w")
        self.notice = tk.StringVar(value="Robot contacts drive the cabinet mechanisms.")
        ttk.Label(cabinet, textvariable=self.notice, wraplength=800).pack(anchor="w", pady=4)

        setup = ttk.LabelFrame(cabinet, text="Table placement", padding=6)
        setup.pack(fill="x", pady=5)
        table = env.table_state()
        self.table_values = [tk.DoubleVar(value=value) for value in
                             (table["x"], table["y"], math.degrees(table["yaw"]))]
        for label, value in zip(("X (m)", "Y (m)", "Yaw (deg)"), self.table_values):
            ttk.Label(setup, text=label).pack(side="left", padx=3)
            ttk.Entry(setup, textvariable=value, width=9).pack(side="left")
        ttk.Button(setup, text="Place table", command=self.table_setup).pack(side="left", padx=5)
        self.cart_fixed = tk.BooleanVar(value=True)
        ttk.Checkbutton(setup, text="Lock table", variable=self.cart_fixed,
                        command=self.set_cart_fixed).pack(side="left")

        viewrow = ttk.Frame(cabinet)
        viewrow.pack(fill="x", pady=4)
        ttk.Label(viewrow, text="Viewer camera").pack(side="left")
        self.view = tk.StringVar(value=view)
        camera = ttk.Combobox(viewrow, textvariable=self.view, state="readonly", width=20,
                              values=VIEW_NAMES)
        camera.pack(side="left", padx=6)
        camera.bind("<<ComboboxSelected>>", self.change_view)
        ttk.Label(cabinet, text="Direct cabinet operations: ./run_cabinet_mujoco.sh", font=("monospace", 10, "bold")).pack(anchor="w", pady=(8, 1))
        ttk.Label(cabinet, text="That entry opens the original cabinet demo with door, plate, selector, breaker and reset controls.",
                  wraplength=800).pack(anchor="w")
        frame = ttk.Frame(cabinet)
        frame.pack(fill="both", expand=True, pady=8)
        self.components = ttk.Treeview(frame, columns=("kind", "state"), show="tree headings", height=13)
        self.components.heading("#0", text="Component")
        self.components.heading("kind", text="Kind")
        self.components.heading("state", text="Measured state")
        self.components.column("#0", width=170)
        self.components.column("kind", width=85)
        self.components.column("state", width=490)
        scrollbar = ttk.Scrollbar(frame, orient="vertical", command=self.components.yview)
        self.components.configure(yscrollcommand=scrollbar.set)
        self.components.pack(side="left", fill="both", expand=True)
        scrollbar.pack(side="right", fill="y")
        for name in env.component_state():
            self.components.insert("", "end", iid=name, text=name)
        self.scene_status = tk.StringVar()
        ttk.Label(cabinet, textvariable=self.scene_status, wraplength=800).pack(anchor="w")
        self._cabinet_refresh_at = 0.0

    def _data_lock(self):
        return self.viewer.lock() if self.viewer is not None else nullcontext()

    def set_cart_fixed(self):
        with self._data_lock():
            self.env.set_cart_fixed(self.cart_fixed.get())

    def door_setup(self, state):
        try:
            with self._data_lock():
                result = self.env.set_door_state(state)
            self.notice.set("Door setup: " + json.dumps(json_safe(result)))
        except (ValueError, RuntimeError) as exc:
            self.notice.set(str(exc))

    def table_setup(self):
        try:
            x, y, yaw = (value.get() for value in self.table_values)
            with self._data_lock():
                result = self.env.set_table_pose(x, y, math.radians(yaw))
            self.notice.set("Table placement accepted" if result["accepted"] else result["reason"])
        except (ValueError, RuntimeError, self.tk.TclError) as exc:
            self.notice.set(str(exc))

    def change_view(self, _event=None):
        if self.viewer is not None:
            with self.viewer.lock():
                set_view(self.viewer, self.view.get(), self.env)

    def reset(self):
        with self._data_lock():
            super().reset()
        if hasattr(self, "cart_fixed"):
            self.cart_fixed.set(True)
            table = self.env.table_state()
            for value, measured in zip(self.table_values,
                                       (table["x"], table["y"], math.degrees(table["yaw"]))):
                value.set(measured)

    def refresh(self):
        if time.monotonic() >= self._cabinet_refresh_at:
            observation = self.env.observation()
            for name, state in observation["components"].items():
                kind = state["kind"]
                if kind == "yaban":
                    text = f"{state['status']} | knob {state['knob_deg']:.1f} deg | blade {state['blade_deg']:.1f} deg"
                elif kind == "breaker":
                    text = f"{'ON / lamp on' if state['powered'] else 'OFF / lamp off'} | {math.degrees(state['angle']):.1f} deg"
                elif kind == "selector":
                    text = f"{state['mode']} | {math.degrees(state['angle']):.1f} deg"
                else:
                    text = f"{state['depth_mm']:.2f} mm | count {state['count']} | LED {'on' if state['led_on'] else 'off'}"
                self.components.item(name, values=(kind, text))
            self.scene_status.set("Time {:.2f}s | {} | Door {} | TCP {}".format(
                observation["time"], "paused" if self.env.paused else "running",
                observation["door"]["state"], np.round(observation["tcp_position"], 3)))
            self._cabinet_refresh_at = time.monotonic() + 0.15
        super().refresh()


def main(argv=None):
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--scene", type=Path, help="default: robot_cabinet_scene_glass_door.xml")
    parser.add_argument("--headless", action="store_true")
    parser.add_argument("--seconds", type=float, help="headless: simulation duration; GUI: wall-clock duration")
    parser.add_argument("--joints", type=float, nargs=6, default=HOME_JOINTS, help="initial joints in radians")
    parser.add_argument("--gripper", type=float, default=HOME_GRIPPER, help="0=open, 1=closed")
    parser.add_argument("--door-state", choices=("open", "closed"), default="open")
    parser.add_argument("--no-panel", "--keyboard-only", dest="no_panel", action="store_true")
    parser.add_argument("--no-viewer", action="store_true")
    parser.add_argument("--view", "--camera", dest="view", default="scene",
                        choices=VIEW_NAMES)
    args = parser.parse_args(argv)
    if args.seconds is not None and (not math.isfinite(args.seconds) or args.seconds <= 0.0):
        parser.error("--seconds must be positive and finite")
    if not math.isfinite(args.gripper) or not 0 <= args.gripper <= 1:
        parser.error("--gripper must be between 0 and 1")
    if args.no_panel and args.no_viewer and not args.headless:
        parser.error("use --headless to run without a panel or viewer")
    with TotalCabinetEnv(args.scene) as env:
        env.reset(args.joints, args.gripper, door_state=args.door_state)
        if args.headless:
            duration = args.seconds or 1.0
            while env.data.time + 1e-12 < duration:
                env.step()
        else:
            viewer_context = nullcontext(None) if args.no_viewer else mj.launch_passive(env.model, env.data)
            panel = None
            depth_preview = None
            try:
                with viewer_context as viewer:
                    if viewer is not None:
                        set_view(viewer, args.view, env)
                    panel = None if args.no_panel else CabinetPanel(env, viewer, view=args.view)
                    depth_preview = DepthPreview(env, parent=panel.root if panel is not None else None)
                    native = NativeControls(viewer) if viewer is not None else None
                    until = time.monotonic() + args.seconds if args.seconds else math.inf
                    while time.monotonic() < until and (viewer is None or viewer.is_running()) and (panel is None or panel.alive):
                        started = time.monotonic()
                        if panel is not None:
                            panel.refresh()
                            if not panel.alive:
                                break
                        if native is not None:
                            env.set_paused(native.paused(env.paused))
                        lock = viewer.lock() if viewer is not None else nullcontext()
                        with lock:
                            env.step()
                        if viewer is not None:
                            before_sync = float(env.data.time)
                            viewer.sync()
                            if env.data.time + 1e-12 < before_sync:
                                # The native viewer reset changes MjData only;
                                # rebuild Python references and mechanism state.
                                paused = env.paused
                                if panel is not None:
                                    panel.reset()
                                else:
                                    with viewer.lock():
                                        env.reset()
                                env.set_paused(paused)
                        depth_preview.update(viewer, camera=panel.view.get() if panel is not None else args.view)
                        time.sleep(max(0.0, env.control_dt - (time.monotonic() - started)))
            finally:
                if depth_preview is not None:
                    depth_preview.close()
                if panel is not None:
                    panel.close()
        print(json.dumps(json_safe(env.observation()), ensure_ascii=False, indent=2))
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
