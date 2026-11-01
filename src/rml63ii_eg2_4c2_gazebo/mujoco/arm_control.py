#!/usr/bin/env python3
"""MuJoCo arm viewer, joint/gripper panel, and headless entry point."""
from __future__ import annotations

import argparse
import json
import math
from pathlib import Path
import time

import numpy as np

try:
    from .environment import ArmEnv, HOME_JOINTS, HOME_GRIPPER, mj
    from .camera_depth import DepthPreview
except ImportError:
    from environment import ArmEnv, HOME_JOINTS, HOME_GRIPPER, mj
    from camera_depth import DepthPreview


class ControlPanel:
    def __init__(self, env, root=None, parent=None):
        import tkinter as tk
        from tkinter import ttk

        self.env, self.tk = env, tk
        self.alive = True
        self.root = root if root is not None else tk.Tk()
        self.container = parent if parent is not None else self.root
        self.root.title("RML63II / EG2-4C2 - MuJoCo")
        self.root.protocol("WM_DELETE_WINDOW", self.close)
        self.values, self.measured = [], []
        for index, limits in enumerate(env.joint_limits):
            row = ttk.Frame(self.container, padding=4)
            row.pack(fill="x")
            ttk.Label(row, text=f"J{index + 1}", width=4).pack(side="left")
            value = tk.DoubleVar(value=math.degrees(env.joint_target[index]))
            tk.Scale(row, variable=value, from_=math.degrees(limits[0]),
                     to=math.degrees(limits[1]), resolution=0.1,
                     orient="horizontal", length=300).pack(side="left")
            measured = tk.StringVar()
            ttk.Label(row, textvariable=measured, width=16).pack(side="left")
            self.values.append(value)
            self.measured.append(measured)
        row = ttk.Frame(self.container, padding=6)
        row.pack()
        for title, callback in [
            ("Send joints", self.send_joints), ("Sync measured", self.sync),
            ("Open", lambda: self.env.set_gripper(0.0)),
            ("Close", lambda: self.env.set_gripper(1.0)), ("Reset", self.reset),
        ]:
            ttk.Button(row, text=title, command=callback).pack(side="left", padx=2)
        ttk.Label(self.container, text="TCP jog: world frame, 5 mm / 0.02 rad").pack()
        jog = ttk.Frame(self.container, padding=5)
        jog.pack()
        for index, axis in enumerate(("X", "Y", "Z", "Roll", "Pitch", "Yaw")):
            for sign in (-1, 1):
                ttk.Button(jog, text=axis + ("+" if sign > 0 else "-"), width=7,
                           command=lambda i=index, s=sign: self.jog(i, s)).grid(
                               row=index // 3, column=(index % 3) * 2 + (sign > 0), padx=2, pady=2)
        self.status = tk.StringVar()
        ttk.Label(self.container, textvariable=self.status, padding=6).pack()
        self._refresh_at = 0.0

    def send_joints(self):
        self.env.set_joint_targets([math.radians(value.get()) for value in self.values])

    def sync(self):
        for value, measured in zip(self.values, self.env.observation()["joint_position"]):
            value.set(math.degrees(measured))

    def reset(self):
        self.env.reset()
        self.sync()

    def jog(self, index, sign):
        delta = np.zeros(6)
        delta[index] = sign * (0.005 if index < 3 else 0.02)
        result = self.env.jog_cartesian(delta[:3], delta[3:])
        if result["accepted"]:
            for value, target in zip(self.values, self.env.joint_target):
                value.set(math.degrees(target))
        else:
            self.status.set("TCP request could not be reached")

    def refresh(self):
        if time.monotonic() >= self._refresh_at:
            observation = self.env.observation()
            for label, measured in zip(self.measured, observation["joint_position"]):
                label.set(f"Actual {math.degrees(measured):.1f} deg")
            self.status.set("Time {:.2f}s | Gripper {:.2f} (0=open, 1=closed)".format(
                observation["time"], observation["gripper"]))
            self._refresh_at = time.monotonic() + 0.1
        self.root.update()

    def close(self):
        if self.alive:
            self.alive = False
            self.root.destroy()


def _json_state(observation):
    return json.dumps({name: value.tolist() if isinstance(value, np.ndarray) else value
                       for name, value in observation.items()}, indent=2)


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--scene", type=Path, help="MJCF scene; defaults to model_mjcf/scene.xml")
    parser.add_argument("--headless", action="store_true", help="run without a viewer or GL context")
    parser.add_argument("--seconds", type=float, help="duration in simulation seconds (headless default: 5)")
    parser.add_argument("--joints", type=float, nargs=6, default=HOME_JOINTS,
                        metavar=("J1", "J2", "J3", "J4", "J5", "J6"), help="initial joints in radians")
    parser.add_argument("--gripper", type=float, default=HOME_GRIPPER, help="initial closedness: 0=open, 1=closed")
    parser.add_argument("--no-panel", action="store_true", help="show only the MuJoCo viewer")
    parser.add_argument("--camera", default="free", choices=("free", "d435_rgb", "d435_depth"))
    args = parser.parse_args()
    if args.seconds is not None and (not math.isfinite(args.seconds) or args.seconds <= 0.0):
        parser.error("--seconds must be positive and finite")
    if not math.isfinite(args.gripper) or not 0.0 <= args.gripper <= 1.0:
        parser.error("--gripper must be between 0 and 1")
    with ArmEnv(args.scene) as env:
        env.reset(args.joints, args.gripper)
        duration = args.seconds if args.seconds is not None else (5.0 if args.headless else math.inf)
        if args.headless:
            while env.data.time + 1e-12 < duration:
                env.step()
        else:
            panel = None if args.no_panel else ControlPanel(env)
            depth_preview = DepthPreview(env, parent=panel.root if panel is not None else None)
            try:
                with mj.launch_passive(env.model, env.data) as viewer:
                    viewer.cam.lookat[:] = (-0.1, 0.0, 1.0)
                    viewer.cam.distance = 2.3
                    viewer.cam.azimuth = 135.0
                    viewer.cam.elevation = -25.0
                    if args.camera != "free":
                        viewer.cam.type = mj.mjtCamera.mjCAMERA_FIXED
                        viewer.cam.fixedcamid = env._id(mj.mjtObj.mjOBJ_CAMERA, args.camera)
                    elapsed = 0.0
                    while viewer.is_running() and (panel is None or panel.alive) and elapsed < duration:
                        started = time.monotonic()
                        if panel is not None:
                            panel.refresh()
                            if not panel.alive:
                                break
                        with viewer.lock():
                            env.step()
                        elapsed += env.control_dt
                        viewer.sync()
                        depth_preview.update(viewer)
                        time.sleep(max(0.0, env.control_dt - (time.monotonic() - started)))
            finally:
                depth_preview.close()
                if panel is not None:
                    panel.close()
        print(_json_state(env.observation()))
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
