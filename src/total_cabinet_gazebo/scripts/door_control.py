#!/usr/bin/env python3
"""电气柜开门和三阶段关门的交互式演示程序。

图形界面向电气柜 Gazebo 运动学插件发布平滑的关节目标位置。关门判断和
恢复动作由该插件负责，并通过带类型的 ROS 消息报告。按钮接触传感器也
可以触发与手动按钮相同的开门流程。
"""

from __future__ import print_function

import math
import queue
import subprocess
import threading
import time

import rclpy
from rclpy.node import Node
from rclpy.parameter import Parameter
from rclpy.qos import QoSProfile, DurabilityPolicy
import tkinter as tk
import tkinter.font as tkfont
from tkinter import messagebox

from electrical_cabinet_gazebo.msg import DoorCloseCommand, DoorCloseStatus
from gazebo_msgs.msg import ContactsState
from sensor_msgs.msg import JointState
from std_msgs.msg import Float64MultiArray


# ===== 模块级常量与辅助定义 =====
JOINTS = (
    "button_press_joint",
    "handle_pop_joint",
    "handle_turn_joint",
    "door_hinge_joint",
)

# URDF 使用半圈凸轮表示按钮，因为 Gazebo Classic 会对原始零范围棱柱关节
# 返回 NaN；按钮中心在 BUTTON_PRESSED 状态下仍会准确沿 -X 方向移动 6 mm。
BUTTON_PRESSED = math.pi
HANDLE_POPPED = -math.pi / 3.0
HANDLE_TURNED = math.pi / 2.0
DOOR_OPEN = math.pi / 2.0

CLOSED = (0.0, 0.0, 0.0, 0.0)
POPPED = (0.0, HANDLE_POPPED, 0.0, 0.0)
TURNED = (0.0, HANDLE_POPPED, HANDLE_TURNED, 0.0)
OPEN = (0.0, HANDLE_POPPED, HANDLE_TURNED, DOOR_OPEN)
DOOR_RETURNED = TURNED
LATCHED_HANDLE_OUT = (0.0, HANDLE_POPPED, 0.0, 0.0)

PLUGIN_STATE_NAMES = {
    DoorCloseStatus.STATE_UNKNOWN: "UNKNOWN",
    DoorCloseStatus.STATE_CLOSED: "CLOSED",
    DoorCloseStatus.STATE_OPEN: "OPEN",
    DoorCloseStatus.STATE_CLOSING_DOOR: "CLOSING_DOOR",
    DoorCloseStatus.STATE_DOOR_RETURNED: "DOOR_RETURNED",
    DoorCloseStatus.STATE_LATCHED_HANDLE_OUT: "LATCHED_HANDLE_OUT",
    DoorCloseStatus.STATE_RECOVERING_OPEN: "RECOVERING_OPEN",
    DoorCloseStatus.STATE_RECOVERING_HANDLE_OUT: "RECOVERING_HANDLE_OUT",
    DoorCloseStatus.STATE_FORCE_RESETTING: "FORCE_RESETTING",
}


# 类功能：封装 ElectricalCabinetDemo 相关的状态、数据和操作。
class ElectricalCabinetDemo(object):
    """封装 Tk 事件循环、Gazebo 运动命令和接触状态机。"""

    # 函数功能：初始化对象成员和运行状态。
    def param(self, name, default):
        return self.node.declare_parameter(name.lstrip('~'), default).value

    def __init__(self, root=None, parent=None, executor=None):
        self._owns_root = root is None
        if not rclpy.ok():
            if not self._owns_root:
                raise RuntimeError("Initialize ROS before creating an embedded door panel")
            rclpy.init()
        self.node = Node("electrical_cabinet_demo", parameter_overrides=[Parameter('use_sim_time', value=True)])

        self.command_topic = self.param(
            "~command_topic", "/electrical_cabinet/kinematic_command"
        )
        self.contact_topic = self.param(
            "~contact_topic", "/electrical_cabinet/button_contacts"
        )
        self.joint_state_topic = self.param(
            "~joint_state_topic", "/total_cabinet/joint_states"
        )
        self.close_command_topic = self.param(
            "~close_command_topic", "/electrical_cabinet/door_close/command"
        )
        self.close_status_topic = self.param(
            "~close_status_topic", "/electrical_cabinet/door_close/status"
        )

        self.command_pub = self.node.create_publisher(Float64MultiArray, self.command_topic, 1)
        self.close_command_pub = self.node.create_publisher(DoorCloseCommand, self.close_command_topic, 10)
        self._state_lock = threading.Lock()
        self._state = "UNKNOWN"
        self._actual = dict(zip(JOINTS, CLOSED))
        self._commanded = tuple(CLOSED)
        self._motion_generation = 0
        self._next_request_id = 1
        self._pending_close_results = {}
        self._close_status_seen = False
        self._contact_active = False
        self._contact_armed = True
        self._closed = False
        self._controller_ready = False
        self._ui_events = queue.Queue()

        self.root = root if root is not None else tk.Tk()
        self.parent = parent if parent is not None else self.root
        self.ui_font_family = self._select_ui_font()
        self._configure_ui_fonts()
        # 窗口管理器标题保留 ASCII；Tk 控件会使用下面选择的中日韩字体，
        # 因此界面正文仍可正常显示中文。
        if self._owns_root:
            self.root.title("Electrical Cabinet Demo")
            self.root.resizable(False, False)
            self.root.protocol("WM_DELETE_WINDOW", self.close)
        self._build_ui()

        self.joint_sub = self.node.create_subscription(JointState, self.joint_state_topic, self._joint_state_cb, 1)
        self.contact_sub = self.node.create_subscription(ContactsState, self.contact_topic, self._contact_cb, 1)
        self.close_status_sub = self.node.create_subscription(DoorCloseStatus, self.close_status_topic, self._close_status_cb,
            QoSProfile(depth=10, durability=DurabilityPolicy.TRANSIENT_LOCAL))

        self._set_status("正在连接 Gazebo 门特性插件和按钮接触传感器...", "#174a7e")
        self.root.after(100, self._refresh_ui)
        self.root.after(500, self._check_connections)
        self.root.after(20, self._drain_ui_events)

        # 保持 ROS 2 回调继续运行，同时不阻塞 Tk 事件循环。
        self._ros_thread = None
        if executor is not None:
            executor.add_node(self.node)
        else:
            self._ros_thread = threading.Thread(target=self._spin, daemon=True)
            self._ros_thread.start()

    # 函数功能：选择支持中文字符的界面字体。
    @staticmethod
    def _select_ui_font():
        """选择包含中日韩字符的字体，避免依赖 Tk 的字体回退。"""
        for candidate in (
            "Noto Sans CJK SC",
            "Noto Sans CJK TC",
            "WenQuanYi Zen Hei",
        ):
            try:
                output = subprocess.check_output(
                    ["fc-match", "-f", "%{family}\n", candidate],
                    stderr=subprocess.STDOUT,
                    universal_newlines=True,
                    timeout=2.0,
                )
                if output.strip().split(",", 1)[0].strip() == candidate:
                    return candidate
            except (OSError, subprocess.SubprocessError):
                pass
        try:
            output = subprocess.check_output(
                ["fc-match", "-f", "%{family}\n", ":lang=zh"],
                stderr=subprocess.STDOUT,
                universal_newlines=True,
                timeout=2.0,
            )
            family = output.strip().split(",", 1)[0].strip()
            if family:
                return family
        except (OSError, subprocess.SubprocessError):
            pass
        return "Noto Sans CJK SC"

    # 函数功能：配置界面控件使用选定字体。
    def _configure_ui_fonts(self):
        """让标签和按钮使用支持中日韩字符的字体。"""
        for named_font in (
            "TkDefaultFont",
            "TkTextFont",
            "TkMenuFont",
            "TkHeadingFont",
            "TkCaptionFont",
            "TkSmallCaptionFont",
            "TkTooltipFont",
            "TkFixedFont",
        ):
            try:
                tkfont.nametofont(named_font).configure(family=self.ui_font_family)
            except tk.TclError:
                pass

    # 函数功能：创建演示界面的控件和布局。
    def _build_ui(self):
        outer = tk.Frame(self.parent, padx=12, pady=10)
        outer.pack(fill=tk.BOTH, expand=True)

        tk.Label(
            outer,
            text="柜门开关控制",
            font=(self.ui_font_family, 16, "bold"),
        ).grid(row=0, column=0, columnspan=3, sticky="w")
        tk.Label(
            outer,
            text="开门沿用原流程；关门依次执行门回转、门把手回转、把手回扣",
            fg="#555555",
        ).grid(row=1, column=0, columnspan=3, sticky="w", pady=(3, 14))

        self.state_value = tk.StringVar(value="正在识别 Gazebo 初始姿态")
        self.contact_value = tk.StringVar(value="未检测到按钮接触")
        self.target_value = tk.StringVar(value="等待门状态")
        self.status_value = tk.StringVar(value="等待 Gazebo...")

        tk.Label(outer, text="当前状态：", width=12, anchor="w").grid(
            row=2, column=0, sticky="w"
        )
        tk.Label(outer, textvariable=self.state_value, width=50, anchor="w").grid(
            row=2, column=1, columnspan=2, sticky="w"
        )
        tk.Label(outer, text="接触检测：", width=12, anchor="w").grid(
            row=3, column=0, sticky="w"
        )
        tk.Label(outer, textvariable=self.contact_value, width=50, anchor="w").grid(
            row=3, column=1, columnspan=2, sticky="w"
        )
        tk.Label(outer, text="当前指令：", width=12, anchor="w").grid(
            row=4, column=0, sticky="w"
        )
        tk.Label(outer, textvariable=self.target_value, width=50, anchor="w").grid(
            row=4, column=1, columnspan=2, sticky="w"
        )

        open_buttons = tk.Frame(outer)
        open_buttons.grid(
            row=5, column=0, columnspan=3, sticky="we", pady=(14, 5)
        )
        self.button_action = tk.Button(
            open_buttons, text="按钮", width=14, height=2, command=self.press_button
        )
        self.button_action.pack(side=tk.LEFT, padx=(0, 8))
        self.turn_action = tk.Button(
            open_buttons, text="转把手并开门", width=14, height=2,
            command=self.turn_handle
        )
        self.turn_action.pack(side=tk.LEFT, padx=8)
        self.reset_action = tk.Button(
            open_buttons, text="强制复位（调试）", width=16, height=2,
            command=self.reset
        )
        self.reset_action.pack(side=tk.LEFT, padx=(8, 0))

        close_buttons = tk.Frame(outer)
        close_buttons.grid(
            row=6, column=0, columnspan=3, sticky="we", pady=(5, 14)
        )
        self.door_return_action = tk.Button(
            close_buttons, text="1. 门回转", width=14, height=2,
            command=self.return_door
        )
        self.door_return_action.pack(side=tk.LEFT, padx=(0, 8))
        self.handle_return_action = tk.Button(
            close_buttons, text="2. 门把手回转", width=14, height=2,
            command=self.return_handle
        )
        self.handle_return_action.pack(side=tk.LEFT, padx=8)
        self.handle_latch_action = tk.Button(
            close_buttons, text="3. 把手回扣", width=16, height=2,
            command=self.latch_handle
        )
        self.handle_latch_action.pack(side=tk.LEFT, padx=(8, 0))

        self.status_label = tk.Label(
            outer,
            textvariable=self.status_value,
            width=68,
            anchor="w",
            justify=tk.LEFT,
            wraplength=520,
        )
        self.status_label.grid(row=7, column=0, columnspan=3, sticky="w")

    # 函数功能：接收关节状态并刷新实际位置。
    def _joint_state_cb(self, message):
        values = dict(zip(message.name, message.position))
        with self._state_lock:
            for joint in JOINTS:
                if joint in values:
                    self._actual[joint] = values[joint]

    # 函数功能：接收关门状态并安排界面更新。
    def _close_status_cb(self, message):
        callback = None
        with self._state_lock:
            self._close_status_seen = True
            if (
                message.request_id
                and message.result
                in (
                    DoorCloseStatus.RESULT_ACCEPTED,
                    DoorCloseStatus.RESULT_SUCCEEDED,
                    DoorCloseStatus.RESULT_FAILED,
                    DoorCloseStatus.RESULT_REJECTED,
                )
            ):
                callback = self._pending_close_results.pop(
                    message.request_id, None
                )
        self._ui_call(self._apply_close_status, message, callback)

    # 函数功能：将关门状态应用到界面和本地状态。
    def _apply_close_status(self, message, callback):
        plugin_state = PLUGIN_STATE_NAMES.get(message.state, "UNKNOWN")
        with self._state_lock:
            local_state = self._state

        # 旧版开门动画会短暂经过非标准姿态（按钮按下、把手弹出等）。这些
        # request_id=0/RESULT_NONE 的被动观测是预期现象，不能覆盖界面的本地
        # 开门流程状态；否则瞬时的 UNKNOWN/CLOSED 消息可能在用户完成“按钮”
        # 操作后立即禁用下一个按钮。
        passive_observation = (
            message.request_id == 0
            and message.result == DoorCloseStatus.RESULT_NONE
        )
        opening_sequence_states = {
            "BUTTON_SEQUENCE",
            "OPENING",
            "POPPED",
            "FORCE_RESETTING",
            "WAITING_CLOSE_START",
            "CLOSING_DOOR",
            "WAITING_DOOR_COMMIT",
            "RETURNING_HANDLE",
            "WAITING_HANDLE_COMMIT",
            "LATCHING_HANDLE",
            "WAITING_LATCH_COMMIT",
        }
        if passive_observation and local_state in opening_sequence_states:
            return

        state_text = {
            "UNKNOWN": "门处于非标准姿态或正在执行原开门流程",
            "CLOSED": "已关闭，可按按钮解锁",
            "OPEN": "柜门已完整打开 90°，可开始三阶段关门",
            "CLOSING_DOOR": "关门第 1 步：门回转",
            "DOOR_RETURNED": "门已回到柜体，等待门把手回转",
            "LATCHED_HANDLE_OUT": "门已扣上，把手仍外倾 60°",
            "RECOVERING_OPEN": "关门条件不满足，正在恢复完整打开状态",
            "RECOVERING_HANDLE_OUT": "回扣未转满，正在恢复把手外倾 60°",
            "FORCE_RESETTING": "正在执行调试用强制复位",
        }[plugin_state]
        target_text = {
            "UNKNOWN": "等待标准打开或关闭姿态",
            "CLOSED": "门、把手均为 0°",
            "OPEN": "门 +90°；把手转动 +90°、外倾 -60°",
            "CLOSING_DOOR": "仅允许门铰链 +90° → 0°",
            "DOOR_RETURNED": "仅允许门把手 +90° → 0°",
            "LATCHED_HANDLE_OUT": "仅允许把手外倾 -60° → 0°",
            "RECOVERING_OPEN": "先恢复把手，再将门打开至 +90°",
            "RECOVERING_HANDLE_OUT": "把手退回 -60°，门保持扣住",
            "FORCE_RESETTING": "调试旁路：全部关节回到 0°",
        }[plugin_state]

        with self._state_lock:
            canonical_positions = {
                "CLOSED": CLOSED,
                "OPEN": OPEN,
                "CLOSING_DOOR": OPEN,
                "DOOR_RETURNED": DOOR_RETURNED,
                "LATCHED_HANDLE_OUT": LATCHED_HANDLE_OUT,
            }
            if plugin_state in canonical_positions:
                self._commanded = tuple(canonical_positions[plugin_state])
        opening_busy = local_state in ("BUTTON_SEQUENCE", "OPENING", "POPPED")
        if plugin_state != "UNKNOWN" or not opening_busy:
            self._set_state(plugin_state, state_text)
            self._set_target_text(target_text)

        if message.result == DoorCloseStatus.RESULT_FAILED:
            self._set_status(message.detail, "#a33a00")
        elif message.result == DoorCloseStatus.RESULT_REJECTED:
            self._set_status("关门命令被拒绝：" + message.detail, "#a33a00")
        elif message.result == DoorCloseStatus.RESULT_SUCCEEDED:
            self._set_status(message.detail, "#176b3a")
        elif message.result == DoorCloseStatus.RESULT_ACCEPTED:
            self._set_status(message.detail, "#174a7e")

        if callback is not None:
            callback(message)

    # 函数功能：发送关门协议命令并登记回调。
    def _send_close_command(self, command, callback=None):
        if not self._close_interface_is_connected():
            self._set_status("Gazebo 关门特性接口未连接，命令未发送。", "#a33a00")
            return None
        with self._state_lock:
            request_id = self._next_request_id
            self._next_request_id += 1
            if callback is not None:
                self._pending_close_results[request_id] = callback
        self.close_command_pub.publish(
            DoorCloseCommand(request_id=request_id, command=command)
        )
        return request_id

    # 函数功能：接收按钮接触消息并触发外部按钮流程。
    def _contact_cb(self, message):
        external_contact = self._contains_external_button_contact(message)

        with self._state_lock:
            previous = self._contact_active
            self._contact_active = external_contact
            if not external_contact:
                self._contact_armed = True
            should_trigger = (
                external_contact
                and not previous
                and self._contact_armed
                and self._state == "CLOSED"
            )
            if should_trigger:
                self._contact_armed = False

        if external_contact:
            self._ui_call(self._set_contact_status, "检测到外部物体接触按钮")
        else:
            self._ui_call(self._set_contact_status, "按钮已释放")

        if should_trigger:
            self._ui_call(self._press_button_from_contact)

    # 函数功能：判断接触消息是否来自外部按钮碰撞体。
    @staticmethod
    def _contains_external_button_contact(message):
        for contact in message.states:
            names = (contact.collision1_name, contact.collision2_name)
            button_names = [
                name for name in names if "button_contact_pad_collision" in name
            ]
            if not button_names:
                continue
            other_names = [name for name in names if name not in button_names]
            if any(
                name
                and not name.startswith("electrical_cabinet::")
                and not name.startswith("electrical_cabinet/")
                for name in other_names
            ):
                return True
        return False

    # 函数功能：执行按钮按压和把手弹出动作。
    def press_button(self, source="Demo 按键"):
        with self._state_lock:
            state = self._state
        if state != "CLOSED":
            self._set_status("当前不是关闭待解锁状态，未重复触发按钮。", "#a33a00")
            return
        if not self._controller_is_connected():
            self._set_status("Gazebo 运动插件未连接，按钮动作未执行。", "#a33a00")
            return

        self._set_state("BUTTON_SEQUENCE", "正在执行：按钮按入、复位并弹出把手")
        self._set_target_text("按钮按入 6 mm → 复位 → 把手弹出 60°")
        self._run_waypoints(
            [
                (0.20, (BUTTON_PRESSED, 0.0, 0.0, 0.0)),
                (0.25, CLOSED),
                (0.60, POPPED),
            ],
            expected_state="BUTTON_SEQUENCE",
            done=lambda success, text: self._button_done(success, text, source),
        )

    # 函数功能：处理按钮动作完成结果。
    def _button_done(self, success, text, source):
        if success:
            self._set_state("POPPED", "把手已弹出，可点击“转把手”")
            self._set_target_text("按钮释放；把手弹出 60°")
            self._set_status("按钮动作完成（来源：%s）。" % source, "#176b3a")
        else:
            self._set_state("CLOSED", "按钮动作未完成，柜门保持关闭")
            self._set_target_text("关闭位置")
            self._set_status("按钮动作未完成：%s" % text, "#a33a00")

    # 函数功能：处理接触传感器触发的按钮动作。
    def _press_button_from_contact(self):
        self._set_contact_status("外部接触触发按钮动作")
        self.press_button(source="接触检测")

    # 函数功能：执行把手旋转动作。
    def turn_handle(self):
        with self._state_lock:
            state = self._state
        if state != "POPPED":
            message = "请先按下“按钮”，使把手弹出。"
            self._set_status("操作顺序错误：" + message, "#a33a00")
            messagebox.showerror("操作顺序错误", message, parent=self.root)
            return
        if not self._controller_is_connected():
            self._set_status("Gazebo 运动插件未连接，转把手动作未执行。", "#a33a00")
            return

        self._set_state(
            "OPENING", "正在执行：把手保持外倾 60°并旋转 90°，随后自动开门"
        )
        self._set_target_text(
            "把手保持外倾 60°并旋转 90° → 柜门绕右侧铰链打开 90°"
        )
        self._run_waypoints(
            [
                (0.60, TURNED),
                (1.20, OPEN),
            ],
            expected_state="OPENING",
            done=self._opening_done,
        )

    # 函数功能：处理开门动作完成结果。
    def _opening_done(self, success, text):
        if success:
            self._set_state("OPEN", "柜门已打开 90°")
            self._set_target_text(
                "把手保持外倾 60°并旋转 90°；柜门打开 90°"
            )
            self._set_status("开门动作完成。", "#176b3a")
        else:
            self._set_state("POPPED", "开门动作未完成，把手保持弹出")
            self._set_target_text("把手弹出 60°")
            self._set_status("开门动作未完成：%s" % text, "#a33a00")

    # 函数功能：启动柜门回位阶段。
    def return_door(self):
        with self._state_lock:
            state = self._state
        if state != "OPEN":
            self._show_close_order_error("请先将柜门完整打开到 90°。")
            return
        if not self._controller_is_connected():
            self._set_status("Gazebo 运动插件未连接，门回转未执行。", "#a33a00")
            return

        self._set_state("WAITING_CLOSE_START", "正在请求开始三阶段关门")
        self._set_target_text("等待 Gazebo 门特性插件确认打开姿态")
        request_id = self._send_close_command(
            DoorCloseCommand.START_CLOSE, self._close_start_result
        )
        if request_id is None:
            self._set_state("OPEN", "柜门已完整打开 90°")

    # 函数功能：处理关门开始命令的返回结果。
    def _close_start_result(self, message):
        if message.result != DoorCloseStatus.RESULT_ACCEPTED:
            return
        self._set_state("CLOSING_DOOR", "关门第 1 步：门沿铰链回转 90°")
        self._set_target_text("门铰链 +90° → 0°；把手保持打开姿态")
        self._run_waypoints(
            [(1.20, DOOR_RETURNED)],
            expected_state="CLOSING_DOOR",
            done=self._door_return_motion_done,
        )

    # 函数功能：处理柜门回位运动结果。
    def _door_return_motion_done(self, success, text):
        if not success:
            self._set_status("门回转动画未完成：%s" % text, "#a33a00")
            return
        self._submit_after_joint_feedback(
            DoorCloseCommand.COMMIT_DOOR_RETURN,
            DOOR_RETURNED,
            "WAITING_DOOR_COMMIT",
            "正在提交门回转结果",
            "插件按实际角度检查门是否回转满 90°",
        )

    # 函数功能：启动把手旋回阶段。
    def return_handle(self):
        with self._state_lock:
            state = self._state
        if state != "DOOR_RETURNED":
            self._show_close_order_error("请先完成第 1 步门回转。")
            return
        if not self._controller_is_connected():
            self._set_status("Gazebo 运动插件未连接，门把手回转未执行。", "#a33a00")
            return

        self._set_state("RETURNING_HANDLE", "关门第 2 步：门把手顺时针回转 90°")
        self._set_target_text("门把手 +90° → 0°；门保持在 0°")
        self._run_waypoints(
            [(0.60, LATCHED_HANDLE_OUT)],
            expected_state="RETURNING_HANDLE",
            done=self._handle_return_motion_done,
        )

    # 函数功能：处理把手旋回运动结果。
    def _handle_return_motion_done(self, success, text):
        if not success:
            self._set_status("门把手回转动画未完成：%s" % text, "#a33a00")
            return
        self._submit_after_joint_feedback(
            DoorCloseCommand.COMMIT_HANDLE_RETURN,
            LATCHED_HANDLE_OUT,
            "WAITING_HANDLE_COMMIT",
            "正在提交门把手回转结果",
            "插件同时检查门 0° 与门把手回转 0°",
        )

    # 函数功能：启动把手锁止阶段。
    def latch_handle(self):
        with self._state_lock:
            state = self._state
        if state != "LATCHED_HANDLE_OUT":
            self._show_close_order_error("请先完成门回转和门把手回转，使门扣上。")
            return
        if not self._controller_is_connected():
            self._set_status("Gazebo 运动插件未连接，把手回扣未执行。", "#a33a00")
            return

        self._set_state("LATCHING_HANDLE", "关门第 3 步：把手朝门锁回扣 60°")
        self._set_target_text("把手外倾 -60° → 0°，贴紧门锁")
        self._run_waypoints(
            [(0.60, CLOSED)],
            expected_state="LATCHING_HANDLE",
            done=self._handle_latch_motion_done,
        )

    # 函数功能：处理把手锁止运动结果。
    def _handle_latch_motion_done(self, success, text):
        if not success:
            self._set_status("把手回扣动画未完成：%s" % text, "#a33a00")
            return
        self._submit_after_joint_feedback(
            DoorCloseCommand.COMMIT_HANDLE_LATCH,
            CLOSED,
            "WAITING_LATCH_COMMIT",
            "正在提交把手回扣结果",
            "插件检查把手是否回扣满 60°",
        )

    # 函数功能：等待关节反馈到位后提交关门阶段。
    def _submit_after_joint_feedback(
        self, command, target, waiting_state, state_text, target_text
    ):
        self._set_state(waiting_state, state_text)
        self._set_target_text(target_text)
        deadline = time.monotonic() + 0.75
        with self._state_lock:
            generation = self._motion_generation

        # 函数功能：轮询 ROS 状态并刷新界面。
        def poll():
            with self._state_lock:
                valid = (
                    generation == self._motion_generation
                    and self._state == waiting_state
                )
                actual = tuple(self._actual[joint] for joint in JOINTS)
            if not valid:
                return
            feedback_ready = all(
                abs(value - expected) <= math.radians(0.5)
                for value, expected in zip(actual, target)
            )
            if feedback_ready or time.monotonic() >= deadline:
                self._send_close_command(command)
                return
            self.root.after(20, poll)

        poll()

    # 函数功能：显示关门阶段顺序错误提示。
    def _show_close_order_error(self, message):
        self._set_status("关门顺序错误：" + message, "#a33a00")
        messagebox.showerror("关门顺序错误", message, parent=self.root)

    # 函数功能：将当前环境或控制状态恢复到初始状态。
    def reset(self):
        if not self._close_interface_is_connected():
            self._set_status("强制复位未发送：Gazebo 门特性接口未连接。", "#a33a00")
            return

        with self._state_lock:
            self._motion_generation += 1
            self._pending_close_results.clear()
        self._set_state("FORCE_RESETTING", "正在执行调试用强制复位")
        self._set_target_text("旁路正常关门协议，全部关节平滑回到 0°")
        self._send_close_command(DoorCloseCommand.FORCE_CLOSED)

    # 函数功能：插值并发布关节目标路径点。
    def _run_waypoints(self, waypoint_specs, expected_state, done):
        """插值计算并发布一系列关节位置路径点。"""
        with self._state_lock:
            self._motion_generation += 1
            generation = self._motion_generation
            start = tuple(self._commanded)

        # 函数功能：推进到下一条动画路径点。
        def next_waypoint(index, segment_start):
            with self._state_lock:
                valid = (
                    generation == self._motion_generation
                    and self._state == expected_state
                )
            if not valid:
                return
            if index >= len(waypoint_specs):
                done(True, "动作完成")
                return
            duration, target = waypoint_specs[index]
            target = tuple(target)
            self._animate_segment(
                segment_start,
                target,
                duration,
                generation,
                lambda: next_waypoint(index + 1, target),
            )

        next_waypoint(0, start)

    # 函数功能：执行一段关节目标动画。
    def _animate_segment(self, start, target, duration, generation, on_done):
        segment_started = time.monotonic()

        # 函数功能：按定时周期推进当前动画。
        def tick():
            if self._closed or not rclpy.ok():
                return
            with self._state_lock:
                if generation != self._motion_generation:
                    return
            alpha = min(
                1.0,
                (time.monotonic() - segment_started) / max(duration, 0.001),
            )
            position = tuple(
                a + (b - a) * alpha for a, b in zip(start, target)
            )
            self._publish_position(position)
            if alpha < 1.0:
                self.root.after(40, tick)
            else:
                on_done()

        tick()

    # 函数功能：发布关节位置目标。
    def _publish_position(self, position):
        values = tuple(float(value) for value in position)
        with self._state_lock:
            self._commanded = values
        self.command_pub.publish(Float64MultiArray(data=list(values)))

    # 函数功能：检查关节控制器是否已连接。
    def _controller_is_connected(self):
        return self.command_pub.get_subscription_count() > 0

    # 函数功能：检查关门协议接口是否已连接。
    def _close_interface_is_connected(self):
        return (
            self.close_command_pub.get_subscription_count() > 0
            and self.node.count_publishers(self.close_status_topic) > 0
        )

    # 函数功能：检查演示所需的 ROS 连接。
    def _check_connections(self):
        if self._closed:
            return
        controller_connected = (
            self._controller_is_connected()
            and self._close_interface_is_connected()
            and self._close_status_seen
        )
        if controller_connected and not self._controller_ready:
            self._controller_ready = True
            self._set_status("已就绪：已连接 Gazebo 运动与门特性接口。", "#176b3a")
        elif not controller_connected:
            self._controller_ready = False
            self._set_status("等待 Gazebo 运动与门特性接口连接...", "#174a7e")

        if self.node.count_publishers(self.contact_topic) == 0:
            self._set_contact_status("等待 Gazebo 按钮接触传感器")
        self.root.after(1000, self._check_connections)

    # 函数功能：刷新界面实际值、目标值和状态。
    def _refresh_ui(self):
        if not rclpy.ok():
            self.close()
            return
        if self._closed:
            return
        with self._state_lock:
            state = self._state
        motion_connected = self._controller_is_connected()
        close_connected = self._close_interface_is_connected()
        self.button_action.configure(
            state=tk.NORMAL if motion_connected and state == "CLOSED" else tk.DISABLED
        )
        self.turn_action.configure(
            state=tk.NORMAL if motion_connected and state == "POPPED" else tk.DISABLED
        )
        self.door_return_action.configure(
            state=tk.NORMAL
            if motion_connected and close_connected and state == "OPEN"
            else tk.DISABLED
        )
        self.handle_return_action.configure(
            state=tk.NORMAL
            if motion_connected and close_connected and state == "DOOR_RETURNED"
            else tk.DISABLED
        )
        self.handle_latch_action.configure(
            state=tk.NORMAL
            if motion_connected
            and close_connected
            and state == "LATCHED_HANDLE_OUT"
            else tk.DISABLED
        )
        self.reset_action.configure(
            state=tk.NORMAL if close_connected else tk.DISABLED
        )
        self.root.after(200, self._refresh_ui)

    # 函数功能：更新界面演示状态。
    def _set_state(self, state, text):
        with self._state_lock:
            self._state = state
        self._update_state_text(text)

    # 函数功能：更新状态文本和按钮可用性。
    def _update_state_text(self, text):
        self._ui_call(self.state_value.set, text)

    # 函数功能：更新按钮接触状态显示。
    def _set_contact_status(self, text):
        self.contact_value.set(text)

    # 函数功能：更新目标姿态文本。
    def _set_target_text(self, text):
        self._ui_call(self.target_value.set, text)

    # 函数功能：更新界面状态提示。
    def _set_status(self, text, color):
        self.status_value.set(text)
        self.status_label.configure(fg=color)

    # 函数功能：在线程安全方式调用界面函数。
    def _ui_call(self, callback, *args):
        if self._closed:
            return
        # Only the main thread may access Tcl, including root.after().
        self._ui_events.put((callback, args))

    def _drain_ui_events(self):
        if self._closed:
            return
        while not self._ui_events.empty():
            callback, args = self._ui_events.get_nowait()
            callback(*args)
        self.root.after(20, self._drain_ui_events)

    # 函数功能：释放当前对象占用的资源。
    def _spin(self):
        try:
            rclpy.spin(self.node)
        except rclpy.executors.ExternalShutdownException:
            pass

    def dispose(self):
        if self._closed:
            return
        self._closed = True
        with self._state_lock:
            self._motion_generation += 1
            self._pending_close_results.clear()

    def close(self):
        if self._closed:
            return
        self.dispose()
        if self._owns_root:
            rclpy.try_shutdown()
            if self._ros_thread is not None:
                self._ros_thread.join(timeout=3)
            self.root.destroy()
            self.node.destroy_node()

    # 函数功能：运行当前模块的主循环或测试流程。
    def run(self):
        self.root.mainloop()


if __name__ == "__main__":
    try:
        ElectricalCabinetDemo().run()
    except (KeyboardInterrupt, rclpy.executors.ExternalShutdownException):
        pass
