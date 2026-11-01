"""Metric depth preview, independent of the native viewer's RGB camera view."""
from __future__ import annotations

from contextlib import nullcontext
import math
import time

import numpy as np

try:
    from .environment import mj
except ImportError:
    from environment import mj


def depth_to_grayscale(depth, near=0.1, far=5.0):
    """Map meters to near-white / far-black pixels; leave raw depth untouched.

    The fixed display range avoids stretching against the far clipping plane.
    Nonfinite and nonpositive values are black. This is a visualization range,
    not a calibrated D435 operating range or a sensor validity mask.
    """
    if not math.isfinite(near) or not math.isfinite(far) or not 0 < near < far:
        raise ValueError("depth display range must satisfy 0 < near < far")
    values = np.asarray(depth)
    if values.ndim != 2:
        raise ValueError("expected a two-dimensional metric depth image")
    valid = np.isfinite(values) & (values > 0)
    values = np.where(valid, values, far)
    brightness = np.clip((far - values) / (far - near), 0, 1)
    return np.rint(brightness * 255).astype(np.uint8)


class DepthPreview:
    """Show true depth when a named *_depth camera is selected.

    Read the native camera selection as well as the application's selector.
    Tk and the offscreen renderer stay on the application thread. A panel's
    existing Tk root is reused; viewer-only mode creates a hidden root lazily.
    """
    def __init__(self, env, parent=None):
        self.env, self.parent = env, parent
        self.root = self.window = self.image_label = self.status = None
        self.photo = None
        self.camera = self.dismissed_camera = None
        self.depth_meters = None
        self._next_frame = 0.0

    def _selected_camera(self, viewer, fallback):
        if viewer is None:
            name = fallback
        else:
            with viewer.lock():
                camera = viewer.cam
                if camera.type != mj.mjtCamera.mjCAMERA_FIXED:
                    return None
                index = camera.fixedcamid
                if not 0 <= index < self.env.model.ncam:
                    return None
                name = mj.mj_id2name(self.env.model, mj.mjtObj.mjOBJ_CAMERA, index)
        return name if name and name.endswith("_depth") else None

    def _show(self):
        if self.window is None:
            import tkinter as tk
            from tkinter import ttk

            self.root = self.parent
            if self.root is None:
                self.root = tk.Tk()
                self.root.withdraw()
            self.window = tk.Toplevel(self.root)
            self.window.protocol("WM_DELETE_WINDOW", self.dismiss)
            self.status = tk.StringVar(master=self.window)
            ttk.Label(self.window, textvariable=self.status, padding=6).pack()
            self.image_label = ttk.Label(self.window)
            self.image_label.pack()
            ttk.Label(self.window, text="Depth display: 0.10–5.00 m | near = white, far = black",
                      padding=6).pack()
        self.window.title(f"{self.camera} — metric depth")
        self.window.deiconify()

    def dismiss(self):
        self.dismissed_camera = self.camera
        self.window.withdraw()

    def update(self, viewer=None, camera=None):
        selected = self._selected_camera(viewer, camera)
        if selected != self.camera:
            self.camera = selected
            self.dismissed_camera = None
            self.depth_meters = None
            self._next_frame = 0.0
            if selected is None and self.window is not None:
                self.window.withdraw()
        if selected is not None and selected != self.dismissed_camera:
            self._show()
            now = time.monotonic()
            if now >= self._next_frame:
                import tkinter as tk

                try:
                    with viewer.lock() if viewer is not None else nullcontext():
                        self.depth_meters = self.env.render(camera=selected, depth=True)
                    pixels = depth_to_grayscale(self.depth_meters)
                    height, width = pixels.shape
                    ppm = f"P5\n{width} {height}\n255\n".encode() + pixels.tobytes()
                    self.photo = tk.PhotoImage(master=self.window, data=ppm, format="PPM")
                    self.image_label.configure(image=self.photo)
                    center = float(self.depth_meters[height // 2, width // 2])
                    distance = f"{center:.3f} m" if math.isfinite(center) and 0 < center < 5 else "outside display range"
                    self.status.set(f"{selected} | {width} × {height} | center: {distance}")
                    self._next_frame = time.monotonic() + 0.2
                except (RuntimeError, ValueError) as exc:
                    # Report a graphics failure rather than silently showing RGB.
                    self.status.set(f"Depth rendering failed: {exc}")
                    self._next_frame = time.monotonic() + 2.0
        if self.root is not None and self.parent is None:
            self.root.update()

    def close(self):
        if self.root is not None:
            import tkinter as tk

            try:
                if self.window is not None:
                    self.window.destroy()
                if self.parent is None:
                    self.root.destroy()
            except tk.TclError:
                # Closing the control panel may already have destroyed children.
                pass
        self.root = self.window = self.image_label = self.status = self.photo = None
