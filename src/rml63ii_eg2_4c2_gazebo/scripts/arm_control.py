#!/usr/bin/python3
"""ROS 2 joint and gripper panel using FollowJointTrajectory actions."""
import math
import queue
import threading
import time
import tkinter as tk
from tkinter import ttk

import rclpy
from rclpy.action import ActionClient
from rclpy.node import Node
from rclpy.parameter import Parameter
from control_msgs.action import FollowJointTrajectory
from sensor_msgs.msg import JointState
from trajectory_msgs.msg import JointTrajectoryPoint
from builtin_interfaces.msg import Duration


class Panel(Node):
    def __init__(self, root=None, parent=None):
        super().__init__("arm_controls", parameter_overrides=[Parameter('use_sim_time', value=True)])
        self._owns_root = root is None
        self._closed = False
        self.root = root if root is not None else tk.Tk()
        self.parent = parent if parent is not None else self.root
        if self._owns_root:
            self.root.title("RML63II / EG2-4C2 - ROS 2 Humble")
            self.root.option_add('*Font', '"Noto Sans CJK SC" 10')
            self.root.protocol("WM_DELETE_WINDOW", self.close)
        self.events = queue.Queue()
        self.latest = {}
        self.last_feedback_wall = 0.0
        self.initialized = False
        self.arm = ActionClient(self, FollowJointTrajectory, "/arm/arm_joint_controller/follow_joint_trajectory")
        self.gripper = ActionClient(self, FollowJointTrajectory, "/arm/gripper_controller/follow_joint_trajectory")
        self.create_subscription(JointState, "/arm/joint_states", self.feedback, 10)
        self.values = []
        self.actual = []
        for i, limit in enumerate([178, 130, 135, 178, 128, 360]):
            row = ttk.Frame(self.parent, padding=5)
            row.pack(fill="x")
            ttk.Label(row, text=f"J{i+1}", width=4).pack(side="left")
            value = tk.DoubleVar()
            tk.Scale(row, variable=value, from_=-limit, to=limit, resolution=.1,
                     orient="horizontal", length=250).pack(side="left")
            measured = tk.StringVar(value="等待反馈")
            ttk.Label(row, textvariable=measured, width=12).pack(side="left")
            self.values.append(value)
            self.actual.append(measured)
        buttons = ttk.Frame(self.parent, padding=8)
        buttons.pack()
        for label, command in [("发送关节目标", self.send_arm), ("同步当前姿态", self.sync),
                               ("张开夹爪", lambda: self.send_gripper(.81)),
                               ("闭合夹爪", lambda: self.send_gripper(.01))]:
            ttk.Button(buttons, text=label, command=command).pack(side="left", padx=3)
        self.notice = tk.StringVar(value="等待控制器与关节反馈…")
        ttk.Label(self.parent, textvariable=self.notice, padding=8).pack()
        self.root.after(100, self.refresh)

    def feedback(self, message):
        self.events.put(("state", dict(zip(message.name, message.position))))

    def sync(self):
        for i, value in enumerate(self.values):
            value.set(math.degrees(self.latest.get(f"joint_{i+1}", 0.)))

    def send(self, client, names, positions):
        if not client.server_is_ready():
            self.notice.set("控制器尚未就绪")
            return
        goal = FollowJointTrajectory.Goal()
        goal.trajectory.joint_names = names
        goal.trajectory.points = [JointTrajectoryPoint(positions=positions, time_from_start=Duration(sec=2))]
        future = client.send_goal_async(goal)
        def completed(result):
            if self._closed:
                return
            try:
                self.events.put(("notice", f"执行完成，状态码 {result.result().result.error_code}"))
            except Exception as error:
                self.events.put(("notice", f"控制器结果不可用：{error}"))
        def accepted(result):
            if self._closed:
                return
            try:
                handle = result.result()
                if not handle.accepted:
                    self.events.put(("notice", "目标被控制器拒绝"))
                    return
                handle.get_result_async().add_done_callback(completed)
            except Exception as error:
                self.events.put(("notice", f"控制器请求失败：{error}"))
        future.add_done_callback(accepted)
        self.notice.set("已发送；以实际关节反馈为准")

    def send_arm(self):
        self.send(self.arm, [f"joint_{i+1}" for i in range(6)], [math.radians(v.get()) for v in self.values])

    def send_gripper(self, angle):
        self.send(self.gripper, ["gripper_joint1"], [angle])

    def refresh(self):
        if self._closed:
            return
        if not rclpy.ok():
            if self._owns_root:
                self.close()
            return
        while not self.events.empty():
            kind, value = self.events.get_nowait()
            if kind == "state":
                self.latest = value
                self.last_feedback_wall = time.monotonic()
                if not self.initialized:
                    self.sync()
                    self.initialized = True
                for i, label in enumerate(self.actual):
                    label.set(f"实测 {math.degrees(value.get(f'joint_{i+1}', 0.)):.1f}°")
            else:
                self.notice.set(value)
        if (self.notice.get() == "等待控制器与关节反馈…"
                and time.monotonic() - self.last_feedback_wall < .8
                and self.arm.server_is_ready() and self.gripper.server_is_ready()):
            self.notice.set("控制器已就绪，已连接机械臂关节反馈")
        self.root.after(100, self.refresh)

    def dispose(self):
        self._closed = True

    def close(self):
        if self._closed:
            return
        self.dispose()
        if self._owns_root:
            rclpy.try_shutdown()
            self.root.destroy()


def main():
    rclpy.init()
    panel = Panel()
    def spin():
        try:
            rclpy.spin(panel)
        except rclpy.executors.ExternalShutdownException:
            pass
    worker = threading.Thread(target=spin, daemon=True)
    worker.start()
    try:
        panel.root.mainloop()
    finally:
        rclpy.try_shutdown()
        worker.join(timeout=2)
        panel.destroy_node()


if __name__ == "__main__":
    main()
