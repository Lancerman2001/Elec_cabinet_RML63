#!/usr/bin/python3
"""One ROS 2/Tk owner for arm, cabinet components, and collapsible door controls."""
from pathlib import Path
import signal
import sys
import threading
import tkinter as tk
from tkinter import ttk

import rclpy
from rclpy.executors import MultiThreadedExecutor, ExternalShutdownException
from rclpy.node import Node
from ament_index_python.packages import get_package_share_directory

from gazebo_control import Panel as CabinetPanel
from door_control import ElectricalCabinetDemo as DoorPanel

# Source execution and the installed ROS executable share the same component.
_source_arm = Path(__file__).resolve().parents[2] / "rml63ii_eg2_4c2_gazebo/scripts"
_arm_scripts = _source_arm if _source_arm.is_dir() else (
    Path(get_package_share_directory("rml63ii_eg2_4c2_gazebo")) / "scripts")
sys.path.insert(0, str(_arm_scripts))
from arm_control import Panel as ArmPanel


class UnifiedControl:
    """Build all enabled panels in one window and run one ROS executor thread.

    ROS must be initialized before constructing this application. Embedding
    panels leaves context/node destruction to this owner. Collapsing the door
    area only changes layout; its subscriptions and state machine keep running.
    """

    def __init__(self, with_robot=True, with_cabinet=True):
        if not rclpy.ok():
            raise RuntimeError("Initialize ROS before creating the control window")
        if not (with_robot or with_cabinet):
            raise ValueError("Enable at least the robot or the cabinet controls")
        self._closed = False
        self.arm_panel = self.cabinet_panel = self.door_panel = None
        self.executor = MultiThreadedExecutor(num_threads=3)
        self.root = tk.Tk()
        self.root.title("Robot and Transparent Cabinet - Gazebo Controls")
        self.root.option_add('*Font', (DoorPanel._select_ui_font(), 10))
        self.root.protocol("WM_DELETE_WINDOW", self.close)
        screen_width = self.root.winfo_screenwidth()
        screen_height = self.root.winfo_screenheight()
        width = min(1100 if with_robot and with_cabinet else 650, screen_width - 60)
        height = min(680, screen_height - 100)
        self.root.geometry(f"{width}x{height}")
        self.root.minsize(min(640, width), min(450, height))

        shell = ttk.Frame(self.root)
        shell.pack(fill="both", expand=True)
        self.canvas = tk.Canvas(shell, highlightthickness=0)
        scrollbar = ttk.Scrollbar(shell, orient="vertical", command=self.canvas.yview)
        self.canvas.configure(yscrollcommand=scrollbar.set)
        scrollbar.pack(side="right", fill="y")
        self.canvas.pack(side="left", fill="both", expand=True)
        self.content = ttk.Frame(self.canvas, padding=10)
        self._canvas_window = self.canvas.create_window((0, 0), window=self.content, anchor="nw")
        self.content.bind("<Configure>", self._update_scrollregion)
        self.canvas.bind("<Configure>", self._resize_content)
        self.root.bind("<MouseWheel>", self._wheel)
        self.root.bind("<Button-4>", lambda _: self.canvas.yview_scroll(-3, "units"))
        self.root.bind("<Button-5>", lambda _: self.canvas.yview_scroll(3, "units"))

        title = ttk.Label(self.content, text="机械臂与透明门电柜 · Gazebo 控制", font=(DoorPanel._select_ui_font(), 14, "bold"))
        title.grid(row=0, column=0, columnspan=2, sticky="w", pady=(0, 10))
        column = 0
        if with_robot:
            arm_frame = ttk.LabelFrame(self.content, text="机械臂 / 夹爪", padding=7)
            arm_frame.grid(row=1, column=column, sticky="new", padx=(0, 7 if with_cabinet else 0))
            self.arm_panel = ArmPanel(root=self.root, parent=arm_frame)
            self.executor.add_node(self.arm_panel)
            self.content.columnconfigure(column, weight=1)
            column += 1
        if with_cabinet:
            cabinet_frame = ttk.LabelFrame(self.content, text="柜内元件", padding=2)
            cabinet_frame.grid(row=1, column=column, sticky="new")
            self.cabinet_panel = CabinetPanel(root=self.root, parent=cabinet_frame, executor=self.executor)
            self.content.columnconfigure(column, weight=1)

        self.door_expanded = tk.BooleanVar(master=self.root, value=False)
        self.door_toggle = self.door_frame = None
        if with_cabinet:
            self.door_toggle = ttk.Button(self.content, text="+ 展开柜门开关控制", command=self.toggle_door)
            self.door_toggle.grid(row=2, column=0, columnspan=2, sticky="ew", pady=(12, 0))
            self.door_frame = ttk.LabelFrame(self.content, text="柜门开关控制", padding=5)
            self.door_frame.grid(row=3, column=0, columnspan=2, sticky="ew", pady=(5, 0))
            self.door_panel = DoorPanel(root=self.root, parent=self.door_frame, executor=self.executor)
            self.door_frame.grid_remove()

        self.ros_thread = threading.Thread(target=self._spin, name="unified-controls-ros", daemon=True)
        self.ros_thread.start()
        self.root.after(100, self._check_context)

    def _update_scrollregion(self, _event=None):
        self.canvas.configure(scrollregion=self.canvas.bbox("all"))

    def _resize_content(self, event):
        self.canvas.itemconfigure(self._canvas_window, width=event.width)

    def _wheel(self, event):
        self.canvas.yview_scroll(-1 if event.delta > 0 else 1, "units")

    def toggle_door(self):
        if self.door_frame is None or self._closed:
            return
        expanded = not self.door_expanded.get()
        self.door_expanded.set(expanded)
        if expanded:
            self.door_frame.grid()
        else:
            self.door_frame.grid_remove()
        self.door_toggle.configure(text="− 收起柜门开关控制" if expanded else "+ 展开柜门开关控制")
        self.root.update_idletasks()
        self._update_scrollregion()
        self.canvas.yview_moveto(1.0 if expanded else 0.0)

    def _spin(self):
        try:
            self.executor.spin()
        except ExternalShutdownException:
            pass

    def _check_context(self):
        if self._closed:
            return
        if not rclpy.ok():
            self.close()
            return
        self.root.after(100, self._check_context)

    def close(self):
        if self._closed:
            return
        self._closed = True
        # Stop Tk animation/command producers while ROS still services any
        # already-issued request, then terminate the single executor.
        for panel in (self.door_panel, self.cabinet_panel, self.arm_panel):
            if panel is not None:
                panel.dispose()
        # Tcl timers belong to this window. Cancel them before destroying its
        # interpreter so a later control window cannot see stale Tk callbacks.
        for timer in self.root.tk.call("after", "info"):
            self.root.after_cancel(timer)
        self.executor.shutdown(timeout_sec=3)
        rclpy.try_shutdown()
        self.ros_thread.join(timeout=3)
        for node in (
            self.arm_panel,
            self.cabinet_panel.node if self.cabinet_panel else None,
            self.door_panel.node if self.door_panel else None,
        ):
            if node is not None:
                node.destroy_node()
        self.root.destroy()

    def run(self):
        self.root.mainloop()


def main():
    rclpy.init()
    config = Node("simulation_controls")
    with_robot = config.declare_parameter("with_robot", True).value
    with_cabinet = config.declare_parameter("with_cabinet", True).value
    config.destroy_node()
    application = UnifiedControl(with_robot=with_robot, with_cabinet=with_cabinet)
    stop_requested = threading.Event()
    signal.signal(signal.SIGINT, lambda *_: stop_requested.set())
    signal.signal(signal.SIGTERM, lambda *_: stop_requested.set())

    def poll_stop():
        if application._closed:
            return
        if stop_requested.is_set():
            application.close()
        else:
            application.root.after(100, poll_stop)

    application.root.after(100, poll_stop)
    try:
        application.run()
    finally:
        application.close()


if __name__ == "__main__":
    main()
