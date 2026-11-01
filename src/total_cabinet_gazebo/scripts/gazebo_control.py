#!/usr/bin/env python3
"""ROS-only control panel; selectable instances, no MuJoCo or physics loop."""
import json
import math
import queue
import threading
import time
import tkinter as tk
from tkinter import ttk
from concurrent.futures import ThreadPoolExecutor
from pathlib import Path

import rclpy
from rclpy.node import Node
from rclpy.parameter import Parameter
from sensor_msgs.msg import JointState
from std_msgs.msg import UInt8MultiArray, String
from std_srvs.srv import SetBool, Trigger
from fugui_gazebo.srv import Press
from xuanniu_gazebo.msg import SwitchCommand
from ament_index_python.packages import get_package_share_directory


class Panel:
    def __init__(self, root=None, parent=None, executor=None):
        button=json.loads((Path(get_package_share_directory('fugui_gazebo'))/'config/button.yaml').read_text())
        self.reset_depth_mm=1000*button['default_depth']
        self.node = Node("cabinet_controls", parameter_overrides=[Parameter('use_sim_time', value=True)])
        self._owns_root = root is None
        self._closed = False
        self.ros_thread = None
        self.root = root if root is not None else tk.Tk()
        self.parent = parent if parent is not None else self.root
        if self._owns_root:
            self.root.title('Total Cabinet - Gazebo controls')
            self.root.option_add('*Font', '"Noto Sans CJK SC" 10')
            self.root.protocol('WM_DELETE_WINDOW', self.close)
        self.lock = threading.Lock()
        self.positions, self.lights, self.last_wall = {}, [], 0
        self.statuses = {}
        self.events = queue.Queue()
        self.worker = ThreadPoolExecutor(max_workers=1)
        self.pending = False
        self.publishers = {}
        self.selections, self.labels = {}, {}
        self.subscribers = [
            self.node.create_subscription(JointState, '/total_cabinet/joint_states', self.joints, 1),
            self.node.create_subscription(UInt8MultiArray, '/total_cabinet/red_lamps', self.lamps, 1),
        ]
        if self._owns_root:
            ttk.Label(self.parent, text='整柜 Gazebo · 选择任意元件操作', padding=10).pack()
        self.row('yaban', '压板（行号从下向上）',
                 ['yaban_r%d_c%d' % (r, c) for r in range(1, 4) for c in range(1, 9)],
                 [('松旋钮', 'loosen'), ('压板退', 'withdraw'), ('压板投', 'insert'), ('紧旋钮', 'tighten')])
        self.row('selector', '转换开关', ['selector_%02d' % i for i in range(1, 4)],
                 [('自动 −45°', 'auto'), ('停止 0°', 'stop'), ('手动 +45°', 'manual')])
        self.row('breaker', '空气开关 / 对应红灯', ['breaker_%02d' % i for i in range(1, 11)],
                 [('上电', 'on'), ('断电', 'off')])
        self.selections['breaker'].set('breaker_10')
        self.row('reset', '复归按钮', ['reset_%02d' % i for i in range(1, 4)],
                 [(f'按压 {self.reset_depth_mm:g} mm', 'press'), ('释放', 'release')])
        self.notice = tk.StringVar(value='正在连接 Gazebo…')
        ttk.Label(self.parent, textvariable=self.notice, wraplength=450, padding=6).pack(fill='x')
        if self._owns_root:
            ttk.Label(self.parent, text='压板须先松再退、先投再紧。\n暂停 / 恢复请使用 Gazebo 播放按钮。', padding=10).pack()
        self.root.after(100, self.refresh)
        if executor is not None:
            executor.add_node(self.node)
        else:
            self.ros_thread = threading.Thread(target=self.spin, daemon=True)
            self.ros_thread.start()

    def row(self, kind, title, instances, actions):
        frame = ttk.LabelFrame(self.parent, text=title, padding=8)
        frame.pack(fill='x', padx=10, pady=5)
        selected = tk.StringVar(value=instances[0])
        self.selections[kind] = selected
        ttk.Combobox(frame, textvariable=selected, values=instances, state='readonly', width=20).pack(side='top', anchor='w')
        buttons = ttk.Frame(frame)
        buttons.pack(anchor='w', pady=5)
        for label, action in actions:
            ttk.Button(buttons, text=label, command=lambda k=kind, a=action: self.submit(k, a)).pack(side='left', padx=2)
        self.labels[kind] = tk.StringVar(value='尚无反馈')
        ttk.Label(frame, textvariable=self.labels[kind], wraplength=450).pack(anchor='w')
        for name in instances:
            if kind in ('breaker', 'reset'):
                self.subscribers.append(self.node.create_subscription(String, '/total_cabinet/' + name + '/status',
                    lambda msg, n=name: self.status(n, msg.data), 1))
            if kind in ('yaban', 'selector'):
                msgtype = JointState if kind == 'yaban' else SwitchCommand
                self.publishers[name] = self.node.create_publisher(msgtype, '/total_cabinet/' + name + '/command', 1)

    def joints(self, msg):
        with self.lock:
            self.positions = dict(zip(msg.name, msg.position))
            self.last_wall = time.monotonic()

    def lamps(self, msg):
        with self.lock:
            self.lights = list(msg.data)

    def status(self, name, text):
        with self.lock:
            self.statuses[name] = text

    def submit(self, kind, action):
        if self._closed:
            return
        if self.pending:
            self.notice.set('上一条请求正在发送，请稍候。')
            return
        name = self.selections[kind].get()
        with self.lock:
            fresh = time.monotonic() - self.last_wall < .8
        if not fresh:
            self.notice.set('没有正在推进的关节反馈：请启动仿真或取消暂停。')
            return
        self.pending = True
        self.notice.set(name + '：发送请求…')
        self.worker.submit(self.command, kind, name, action)

    def call(self, service_type, name, request):
        client = self.node.create_client(service_type, name)
        try:
            deadline = time.monotonic() + 2.0
            while not client.wait_for_service(timeout_sec=.1):
                if self._closed:
                    raise RuntimeError('控制界面已关闭')
                if time.monotonic() >= deadline:
                    raise RuntimeError('服务未连接：' + name)
            future = client.call_async(request)
            completed = threading.Event()
            future.add_done_callback(lambda _: completed.set())
            deadline = time.monotonic() + 5.0
            while not completed.wait(.1):
                if self._closed:
                    raise RuntimeError('控制界面已关闭')
                if time.monotonic() >= deadline:
                    raise RuntimeError('服务响应超时：' + name)
            return future.result()
        finally:
            self.node.destroy_client(client)

    def command(self, kind, name, action):
        try:
            ns = '/total_cabinet/' + name
            if kind == 'breaker':
                result = self.call(SetBool, ns + '/set_power', SetBool.Request(data=action == 'on'))
            elif kind == 'reset':
                service = '/press' if action == 'press' else '/release'
                if action == 'press':
                    result = self.call(Press, ns + service, Press.Request(depth_mm=self.reset_depth_mm, hold_seconds=.35, auto_release=True))
                else:
                    result = self.call(Trigger, ns + service, Trigger.Request())
            else:
                publisher = self.publishers[name]
                if not publisher.get_subscription_count():
                    raise RuntimeError('元件控制插件未连接')
                if kind == 'selector':
                    msg = SwitchCommand()
                    msg.header.stamp = self.node.get_clock().now().to_msg()
                    msg.target_angle_rad = {'auto': -math.pi/4, 'stop': 0., 'manual': math.pi/4}[action]
                    msg.dragging = False
                else:
                    prefix = name + '__yaban_'
                    with self.lock:
                        knob = self.positions.get(prefix + 'left_knob_joint', 0.)
                        blade = self.positions.get(prefix + 'blade_joint', 0.)
                    if action == 'withdraw' and knob < math.pi - .02:
                        raise RuntimeError('先松旋钮至少 180°，等待完成后再退出压板')
                    if action == 'tighten' and abs(blade) > .02:
                        raise RuntimeError('先投入压板，等待完成后再紧旋钮')
                    joint, target = {
                        'loosen': ('left_knob_joint', math.pi),
                        'tighten': ('left_knob_joint', 0.),
                        'withdraw': ('blade_joint', math.pi/2),
                        'insert': ('blade_joint', 0.),
                    }[action]
                    msg = JointState(name=[prefix + joint], position=[target])
                    msg.header.stamp = self.node.get_clock().now().to_msg()
                publisher.publish(msg)
                self.events.put(name + '：已发送，完成情况以实际位置反馈为准。')
                return
            self.events.put(name + ('：已接受；' if result.success else '：未接受；') + result.message)
        except Exception as exc:
            self.events.put(name + '：' + str(exc))

    def refresh(self):
        if self._closed:
            return
        if not rclpy.ok():
            if self._owns_root:
                self.close()
            return
        while not self.events.empty():
            self.notice.set(self.events.get_nowait())
            self.pending = False
        with self.lock:
            p, lights, statuses = dict(self.positions), list(self.lights), dict(self.statuses)
            fresh = time.monotonic() - self.last_wall < .8
        if fresh and self.notice.get() == '正在连接 Gazebo…':
            self.notice.set('已连接 Gazebo，可选择元件操作。')
        for kind, selection in self.selections.items():
            name = selection.get()
            if not fresh:
                text = '反馈已暂停 / 未连接'
            elif kind == 'yaban':
                prefix = name + '__yaban_'
                text = '旋钮 %.1f° · 压片 %.1f° · 升程 %.3f mm' % (
                    math.degrees(p.get(prefix+'left_knob_joint', 0)),
                    math.degrees(p.get(prefix+'blade_joint', 0)),
                    1000*p.get(prefix+'left_knob_lift_joint', 0))
            elif kind == 'selector':
                text = '实际角度 %.1f°' % math.degrees(p.get(name+'__xuanniu_selector_joint', 0))
            elif kind == 'breaker':
                i = int(name[-2:])-1
                text = '实际角度 %.1f° · 红灯 %s · %s' % (
                    math.degrees(p.get(name+'__breaker_toggle_joint', 0)),
                    ('亮' if lights[i] else '灭') if i < len(lights) else '未知', statuses.get(name, ''))
            else:
                try:
                    state = json.loads(statuses.get(name, '{}'))
                except ValueError:
                    state = {}
                text = '深度 %.2f mm · 复归次数 %s · 绿灯 %s' % (
                    1000*p.get(name+'__fugui_press_joint', 0), state.get('reset_count', '?'),
                    '亮' if state.get('led_on') else '灭')
            self.labels[kind].set(text)
        self.root.after(100, self.refresh)

    def spin(self):
        try:
            rclpy.spin(self.node)
        except rclpy.executors.ExternalShutdownException:
            pass

    def dispose(self):
        if self._closed:
            return
        self._closed = True
        self.worker.shutdown(wait=True, cancel_futures=True)

    def close(self):
        if self._closed:
            return
        self.dispose()
        if self._owns_root:
            rclpy.try_shutdown()
            if self.ros_thread is not None:
                self.ros_thread.join(timeout=3)
            self.root.destroy()
            self.node.destroy_node()


if __name__ == '__main__':
    rclpy.init()
    Panel().root.mainloop()
