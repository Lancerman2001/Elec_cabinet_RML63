#!/usr/bin/env python3
"""Standalone five-button MuJoCo control panel for the electrical-cabinet pressure plate."""

from __future__ import annotations

import argparse
from dataclasses import dataclass
import math
import queue
import threading
import time
import tkinter as tk
from tkinter import font as tkfont

try:
    from yaban_mujoco import YabanMuJoCoBackend
    from yaban_mujoco_protocol import (
        ACTIONS, INITIALIZE, INSERT, LOOSEN, TIGHTEN, WITHDRAW,
        Pose, action_error, allowed_actions, make_plan, next_instruction, state_text,
    )
except ImportError:  # pragma: no cover - package-style import in external tests
    from .yaban_mujoco import YabanMuJoCoBackend
    from .yaban_mujoco_protocol import (
        ACTIONS, INITIALIZE, INSERT, LOOSEN, TIGHTEN, WITHDRAW,
        Pose, action_error, allowed_actions, make_plan, next_instruction, state_text,
    )


BUTTON_HELP = {
    INITIALIZE: "恢复压片 0°、旋钮 0°",
    LOOSEN: "左旋钮逆时针 180°",
    WITHDRAW: "压片逆时针 90°",
    INSERT: "压片顺时针 90°，止于 0°",
    TIGHTEN: "左旋钮顺时针 180°，止于 0°",
}


@dataclass(frozen=True)
class Update:
    pose: Pose
    busy: bool
    status: str
    error: str = ""
    running: bool = True


def _move_towards(value: float, target: float, speed: float, dt: float) -> float:
    distance = target - value
    if abs(distance) <= speed * dt:
        return target
    return value + math.copysign(speed * dt, distance)


class MuJoCoMotionWorker(threading.Thread):
    """Owns MuJoCo data and viewer so Tk only exchanges immutable state messages."""

    knob_speed = math.pi       # rad/s: 180 degrees in one second
    blade_speed = math.pi / 2  # rad/s: 90 degrees in one second

    def __init__(self, use_viewer: bool = True):
        super().__init__(name="yaban-mujoco", daemon=True)
        self.use_viewer = use_viewer
        self.commands: queue.Queue[str] = queue.Queue()
        self.updates: queue.Queue[Update] = queue.Queue(maxsize=4)
        self.stop_event = threading.Event()
        self.ready_event = threading.Event()
        self.startup_error = ""
        self._backend = None
        self._busy = False
        self._status = "正在加载 MuJoCo 模型…"
        self._error = ""
        self._steps = []
        self._active_step = None
        self._last_emit = 0.0

    def submit(self, action: str) -> None:
        self.commands.put(action)

    def stop(self) -> None:
        self.stop_event.set()

    def _emit(self, force: bool = False, running: bool = True) -> None:
        now = time.monotonic()
        if not force and now - self._last_emit < 0.05:
            return
        self._last_emit = now
        pose = self._backend.pose() if self._backend else Pose(math.nan, math.nan)
        update = Update(pose, self._busy, self._status, self._error, running)
        try:
            while True:
                self.updates.get_nowait()
        except queue.Empty:
            pass
        try:
            self.updates.put_nowait(update)
        except queue.Full:  # pragma: no cover - drained immediately above
            pass

    def _accept_commands(self) -> None:
        while True:
            try:
                action = self.commands.get_nowait()
            except queue.Empty:
                return
            pose = self._backend.pose()
            if action not in allowed_actions(pose, self._busy):
                self._error = action_error(action, pose, self._busy)
                self._status = "操作未执行"
                self._emit(force=True)
                continue
            self._steps = make_plan(action, pose)
            self._active_step = None
            self._busy = bool(self._steps)
            self._error = ""
            self._status = "正在执行：{}".format(action) if self._busy else "初始化完成：已处于投状态并锁紧。"
            self._emit(force=True)

    def _advance_motion(self, dt: float) -> None:
        if not self._busy:
            return
        if self._active_step is None:
            self._active_step = self._steps.pop(0)
            self._status = self._active_step.description
        current = self._backend.pose()
        target = self._active_step.target
        next_pose = Pose(
            _move_towards(current.knob, target.knob, self.knob_speed, dt),
            _move_towards(current.blade, target.blade, self.blade_speed, dt),
        )
        self._backend.set_pose(next_pose)
        if next_pose == target:
            self._active_step = None
            if not self._steps:
                self._busy = False
                self._status = "操作完成；MuJoCo 实际关节位置已确认。"

    def _run_loop(self, viewer) -> None:
        last_time = time.monotonic()
        while not self.stop_event.is_set() and (viewer is None or viewer.is_running()):
            now = time.monotonic()
            dt = min(0.05, max(0.001, now - last_time))
            last_time = now
            self._accept_commands()
            self._advance_motion(dt)
            self._backend.mujoco.mj_forward(self._backend.model, self._backend.data)
            if viewer is not None:
                viewer.sync()
            self._emit()
            time.sleep(0.01)

    def run(self) -> None:
        try:
            self._backend = YabanMuJoCoBackend()
            self._status = "MuJoCo 已连接：初始化为投状态、旋钮锁紧。"
            self.ready_event.set()
            self._emit(force=True)
            if self.use_viewer:
                viewer_module = __import__("mujoco.viewer", fromlist=["launch_passive"])
                with viewer_module.launch_passive(
                    self._backend.model, self._backend.data, show_left_ui=True, show_right_ui=False
                ) as viewer:
                    self._run_loop(viewer)
            else:
                self._run_loop(None)
        except Exception as exc:  # keep the Tk window responsive and show the actual issue
            self.startup_error = str(exc)
            self._error = self.startup_error
            self._status = "MuJoCo 未启动"
            self.ready_event.set()
            self._emit(force=True, running=False)
        finally:
            self.ready_event.set()
            if self._backend:
                self._status = "MuJoCo 控制已停止"
                self._emit(force=True, running=False)


def choose_font_family(root: tk.Tk) -> str:
    available = set(tkfont.families(root))
    for candidate in ("Noto Sans CJK SC", "Noto Sans CJK", "WenQuanYi Zen Hei", "Microsoft YaHei", "SimHei"):
        if candidate in available:
            return candidate
    return tkfont.nametofont("TkDefaultFont").actual("family")


class YabanMuJoCoPanel:
    def __init__(self, root: tk.Tk, use_viewer: bool):
        self.root = root
        self.root.title("YABAN - MuJoCo Control")
        self.root.geometry("880x540")
        self.root.minsize(760, 500)
        self.root.configure(bg="#eef2f7")
        self.font_family = choose_font_family(root)
        self.worker = MuJoCoMotionWorker(use_viewer=use_viewer)
        self.latest = Update(Pose(math.nan, math.nan), False, "正在启动 MuJoCo…")
        self.buttons: dict[str, tk.Button] = {}
        self._build()
        self.root.protocol("WM_DELETE_WINDOW", self.close)
        self.worker.start()
        self.root.after(50, self._poll)

    def _font(self, size: int, weight: str = "normal"):
        return (self.font_family, size, weight)

    def _build(self) -> None:
        outer = tk.Frame(self.root, bg="#eef2f7", padx=30, pady=24)
        outer.pack(fill="both", expand=True)
        tk.Label(outer, text="压板投退控制 · MuJoCo", bg="#eef2f7", fg="#193654", font=self._font(24, "bold")).pack(anchor="w")
        tk.Label(outer, text="先松旋钮，再拨压片；先合压片，再紧旋钮。", bg="#eef2f7", fg="#58718c", font=self._font(11)).pack(anchor="w", pady=(5, 16))

        self.state_var = tk.StringVar(value="等待模型反馈")
        self.status_var = tk.StringVar(value="正在启动 MuJoCo…")
        self.error_var = tk.StringVar(value="")
        tk.Label(outer, textvariable=self.state_var, bg="#eef2f7", fg="#193654", font=self._font(18, "bold")).pack(anchor="w")

        metrics = tk.Frame(outer, bg="#eef2f7")
        metrics.pack(fill="x", pady=(12, 18))
        self.knob_var = tk.StringVar(value="-- °")
        self.blade_var = tk.StringVar(value="-- °")
        self.lift_var = tk.StringVar(value="-- mm")
        for label, variable in (("左旋钮角度", self.knob_var), ("压片角度 / 0–120°", self.blade_var), ("左旋钮升程", self.lift_var)):
            card = tk.Frame(metrics, bg="white", bd=1, relief="solid", padx=18, pady=10)
            card.pack(side="left", fill="x", expand=True, padx=(0, 8))
            tk.Label(card, text=label, bg="white", fg="#58718c", font=self._font(10)).pack(anchor="w")
            tk.Label(card, textvariable=variable, bg="white", fg="#193654", font=self._font(18, "bold")).pack(anchor="w", pady=(4, 0))

        content = tk.Frame(outer, bg="#eef2f7")
        content.pack(fill="both", expand=True)
        controls = tk.Frame(content, bg="#eef2f7")
        controls.pack(side="left", fill="y")
        for action in ACTIONS:
            row = tk.Frame(controls, bg="#eef2f7")
            row.pack(fill="x", pady=4)
            button = tk.Button(row, text=action, width=10, command=lambda value=action: self.worker.submit(value),
                               font=self._font(12, "bold"), fg="#193654", bg="#d9e7f6", activebackground="#bed7ef", relief="solid", bd=1)
            button.pack(side="left")
            self.buttons[action] = button
            tk.Label(row, text=BUTTON_HELP[action], bg="#eef2f7", fg="#58718c", font=self._font(10)).pack(side="left", padx=12)

        right = tk.Frame(content, bg="white", bd=1, relief="solid", padx=18, pady=16)
        right.pack(side="left", fill="both", expand=True, padx=(24, 0))
        tk.Label(right, text="压片位置示意", bg="white", fg="#193654", font=self._font(15, "bold")).pack(anchor="w")
        self.canvas = tk.Canvas(right, bg="white", highlightthickness=0, height=175)
        self.canvas.pack(fill="both", expand=True, pady=(8, 4))
        tk.Label(right, textvariable=self.status_var, bg="white", fg="#355c7d", justify="left", anchor="w", wraplength=390, font=self._font(10)).pack(fill="x")
        tk.Label(right, textvariable=self.error_var, bg="white", fg="#b33a3a", justify="left", anchor="w", wraplength=390, font=self._font(10)).pack(fill="x", pady=(4, 0))

    def _draw(self, blade: float) -> None:
        self.canvas.delete("all")
        width = max(320, self.canvas.winfo_width())
        height = max(150, self.canvas.winfo_height())
        pivot = (width - 70, height // 2 + 20)
        length = min(190, width - 130)
        angle = math.pi - blade
        end = (pivot[0] + length * math.cos(angle), pivot[1] - length * math.sin(angle))
        self.canvas.create_line(45, pivot[1], pivot[0], pivot[1], fill="#a8b8c7", width=12)
        self.canvas.create_line(pivot[0], pivot[1], end[0], end[1], fill="#919da8", width=7)
        self.canvas.create_oval(pivot[0] - 12, pivot[1] - 12, pivot[0] + 12, pivot[1] + 12, fill="#f1f0d5", outline="#849071", width=2)
        self.canvas.create_rectangle(end[0] - 9, end[1] - 9, end[0] + 9, end[1] + 9, fill="#f1f0d5", outline="#9b9271", width=2)
        self.canvas.create_text(pivot[0], pivot[1] - 28, text="右端转轴", fill="#193654", font=self._font(10))
        self.canvas.create_text(end[0], end[1] - 24, text="白色拨柄", fill="#193654", font=self._font(10))
        self.canvas.create_text(width // 2, height - 14, text="{:.1f}°".format(math.degrees(blade)), fill="#168c65", font=self._font(14, "bold"))

    def _poll(self) -> None:
        try:
            while True:
                self.latest = self.worker.updates.get_nowait()
        except queue.Empty:
            pass
        pose = self.latest.pose
        if math.isfinite(pose.knob) and math.isfinite(pose.blade):
            self.state_var.set(state_text(pose))
            self.knob_var.set("{:.2f}°".format(math.degrees(pose.knob)))
            self.blade_var.set("{:.2f}°".format(math.degrees(pose.blade)))
            self.lift_var.set("{:.3f} mm".format(pose.lift * 1000.0))
            self._draw(pose.blade)
            allowed = allowed_actions(pose, self.latest.busy) if self.latest.running else frozenset()
        else:
            allowed = frozenset()
        for action, button in self.buttons.items():
            button.configure(state="normal" if action in allowed else "disabled")
        status = self.latest.status
        if not self.latest.busy and not self.latest.error and math.isfinite(pose.knob):
            status = next_instruction(pose)
        self.status_var.set(status)
        self.error_var.set(self.latest.error)
        if self.root.winfo_exists():
            self.root.after(50, self._poll)

    def close(self) -> None:
        self.worker.stop()
        self.worker.join(timeout=1.0)
        self.root.destroy()


def main() -> int:
    parser = argparse.ArgumentParser(description="Standalone MuJoCo pressure-plate controller")
    parser.add_argument("--no-viewer", action="store_true", help="run control logic without opening the MuJoCo 3D viewer")
    args = parser.parse_args()
    try:
        root = tk.Tk()
    except tk.TclError as exc:
        raise SystemExit("无法打开 Tk 窗口：{}".format(exc))
    panel = YabanMuJoCoPanel(root, use_viewer=not args.no_viewer)
    root.mainloop()
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
