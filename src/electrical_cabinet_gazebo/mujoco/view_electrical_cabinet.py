#!/usr/bin/env python3
"""电气柜门演示的交互式 MuJoCo 查看器。

查看器保留原有的独立键盘操作，并在不引入 ROS 依赖的情况下复现 Gazebo
的三阶段关门协议：

  B：按下按钮、释放按钮并弹出把手
  T：旋转弹出的把手并打开柜门
  1：返回柜门并提交第 1 阶段
  2：返回把手旋转角并提交第 2 阶段
  3：把手回位并提交第 3 阶段
  F：强制关闭并复位（R 保留为兼容性复位快捷键）
"""

from __future__ import annotations

import argparse
import pathlib
import threading
import time

import mujoco
import mujoco.viewer

# ===== 模块级常量与辅助定义 =====
try:
    from door_close_protocol import (
        CommandResult,
        Decision,
        DoorCloseStateMachine,
        DoorCommand,
        DoorState,
        MotionRequest,
        Pose,
        closed_pose,
        door_returned_pose,
        latched_handle_out_pose,
        open_pose,
    )
except ImportError:  # pragma: no cover - 测试运行器中的包式导入
    from .door_close_protocol import (
        CommandResult,
        Decision,
        DoorCloseStateMachine,
        DoorCommand,
        DoorState,
        MotionRequest,
        Pose,
        closed_pose,
        door_returned_pose,
        latched_handle_out_pose,
        open_pose,
    )


MODEL_PATH = pathlib.Path(__file__).with_name("electrical_cabinet.xml")

JOINTS = (
    "button_press_joint",
    "handle_pop_joint",
    "handle_turn_joint",
    "door_hinge_joint",
)

BUTTON_PRESSED = -0.006
HANDLE_POPPED = -1.0471975512
HANDLE_TURNED = 1.57079632679
DOOR_OPEN = 1.57079632679

CLOSED = closed_pose().as_tuple()
POPPED = (0.0, HANDLE_POPPED, 0.0, 0.0)
TURNED = door_returned_pose().as_tuple()
OPEN = open_pose().as_tuple()
DOOR_RETURNED = TURNED
LATCHED_HANDLE_OUT = latched_handle_out_pose().as_tuple()

POSES = {
    "closed": CLOSED,
    "popped": POPPED,
    "turned": TURNED,
    "open": OPEN,
    "door_returned": DOOR_RETURNED,
    "latched_handle_out": LATCHED_HANDLE_OUT,
}


# 函数功能：建立关节名称到 MuJoCo 编号的映射。
def joint_id_map(model):
    result = {}
    for name in JOINTS:
        joint_id = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_JOINT, name)
        if joint_id < 0:
            raise RuntimeError("MuJoCo model is missing joint {}".format(name))
        result[name] = joint_id
    return result


# 函数功能：将目标关节值写入 MuJoCo 状态。
def set_joint_values(model, data, joint_ids, values):
    for name, value in zip(JOINTS, values):
        data.qpos[model.jnt_qposadr[joint_ids[name]]] = value
    mujoco.mj_forward(model, data)


# 函数功能：读取 MuJoCo 中的实际关节值。
def current_joint_values(model, data, joint_ids):
    return tuple(
        float(data.qpos[model.jnt_qposadr[joint_ids[name]]])
        for name in JOINTS
    )


# 函数功能：根据调试姿态名称生成初始状态。
def initial_state(pose_name):
    return {
        "closed": "CLOSED",
        "popped": "POPPED",
        "turned": "TURNED",
        "open": "OPEN",
        "door_returned": "DOOR_RETURNED",
        "latched_handle_out": "LATCHED_HANDLE_OUT",
    }[pose_name]


# 函数功能：将状态枚举转换为可显示名称。
def state_name(state):
    return state.name if isinstance(state, DoorState) else str(state)


# 函数功能：格式化关节姿态以便输出。
def format_pose(pose):
    values = pose.as_tuple() if isinstance(pose, Pose) else tuple(pose)
    return "button={:.4f}, pop={:.4f}, turn={:.4f}, door={:.4f}".format(
        *values
    )


# 函数功能：初始化参数、节点和主运行流程。
def main():
    parser = argparse.ArgumentParser()
    parser.add_argument(
        "--pose",
        choices=sorted(POSES),
        default="closed",
        help="initial inspection pose (default: closed)",
    )
    args = parser.parse_args()

    model = mujoco.MjModel.from_xml_path(str(MODEL_PATH))
    data = mujoco.MjData(model)
    joint_ids = joint_id_map(model)
    set_joint_values(model, data, joint_ids, POSES[args.pose])

    touch_sensor_id = mujoco.mj_name2id(
        model, mujoco.mjtObj.mjOBJ_SENSOR, "button_contact_sensor"
    )
    if touch_sensor_id < 0:
        raise RuntimeError("MuJoCo model is missing button_contact_sensor")
    button_geom_id = mujoco.mj_name2id(
        model, mujoco.mjtObj.mjOBJ_GEOM, "button"
    )
    if button_geom_id < 0:
        raise RuntimeError("MuJoCo model is missing button geometry")

    protocol = DoorCloseStateMachine()
    protocol.seed_pose(Pose.from_tuple(POSES[args.pose]))

    action_lock = threading.Lock()
    requested_actions = []

    # 函数功能：接收查看器按键并排队相应动作。
    def key_callback(keycode):
        try:
            key = chr(keycode).lower()
        except (TypeError, ValueError):
            return
        if key in ("b", "t", "r", "f", "1", "2", "3"):
            with action_lock:
                requested_actions.append(key)

    print("MuJoCo electrical cabinet viewer")
    print("model: {}".format(MODEL_PATH))
    print("initial pose: {}".format(args.pose))
    print("操作: B=按钮, T=转把手并开门, 1=门回转, 2=把手回转, 3=把手回扣")
    print("复位: F=强制复位, R=兼容性复位别名")
    print("流程: 关闭 -> 按钮 -> 转把手并开门 -> 1 -> 2 -> 3")
    print("容差: +/-2 degrees; 接触传感器: button_contact_sensor")
    print("Use the mouse to orbit/zoom the camera; close the viewer window to exit.")

    display_state = initial_state(args.pose)
    motion = None
    touch_armed = True
    next_request_id = 1

    # 函数功能：向终端输出动作提示。
    def announce(message):
        print("[MuJoCo] {}".format(message), flush=True)

    # 函数功能：生成下一个协议请求编号。
    def new_request_id():
        nonlocal next_request_id
        request_id = next_request_id
        next_request_id += 1
        return request_id

    # 函数功能：发布协议状态和实际姿态。
    def publish_status(request_id, command, decision, actual=None):
        if actual is None:
            with viewer.lock():
                actual = current_joint_values(model, data, joint_ids)
        command_name = "NONE"
        try:
            command_name = DoorCommand(int(command)).name
        except (TypeError, ValueError):
            if command:
                command_name = str(command)
        announce(
            "status request_id={} command={} state={} result={} "
            "failed_checks={} detail={} [{}]".format(
                request_id,
                command_name,
                state_name(decision.state),
                decision.result.name,
                decision.failed_checks,
                decision.detail,
                format_pose(actual),
            )
        )

    # 函数功能：启动内部运动并登记完成回调。
    def begin_motion(name, segments, on_complete):
        nonlocal motion, display_state
        if motion is not None:
            announce("当前动作进行中，未覆盖原动作。")
            return False
        with viewer.lock():
            start = current_joint_values(model, data, joint_ids)
        motion = {
            "name": name,
            "segments": tuple(
                (float(duration), tuple(target))
                for duration, target in segments
            ),
            "index": 0,
            "start": start,
            "started": time.monotonic(),
            "on_complete": on_complete,
        }
        display_state = name
        announce("开始{}。".format(name))
        return True

    # 函数功能：根据失败结果生成恢复运动分段。
    def recovery_segments(request):
        with viewer.lock():
            actual = current_joint_values(model, data, joint_ids)
        current = Pose.from_tuple(actual)
        if request == MotionRequest.RECOVER_OPEN:
            handle_restored = Pose(
                button=0.0,
                handle_pop=HANDLE_POPPED,
                handle_turn=HANDLE_TURNED,
                door_hinge=current.door_hinge,
            ).as_tuple()
            return (
                (0.60, handle_restored),
                (1.20, OPEN),
            )
        if request == MotionRequest.RECOVER_HANDLE_OUT:
            return ((0.60, LATCHED_HANDLE_OUT),)
        if request == MotionRequest.FORCE_CLOSED:
            return ((0.90, CLOSED),)
        return ()

    # 函数功能：完成内部运动并发布决策。
    def finish_internal(request_id, command, actual):
        nonlocal display_state
        decision = protocol.complete_internal_motion()
        target_by_state = {
            DoorState.OPEN: OPEN,
            DoorState.LATCHED_HANDLE_OUT: LATCHED_HANDLE_OUT,
            DoorState.CLOSED: CLOSED,
        }
        target = target_by_state.get(decision.state)
        if target is not None:
            with viewer.lock():
                set_joint_values(model, data, joint_ids, target)
                actual = current_joint_values(model, data, joint_ids)
        display_state = state_name(decision.state)
        publish_status(request_id, command, decision, actual)

    # 函数功能：启动协议要求的恢复运动。
    def begin_recovery(request_id, command, decision):
        nonlocal display_state
        segments = recovery_segments(decision.motion)
        if not segments:
            announce("恢复请求缺少有效轨迹。")
            return
        display_state = state_name(decision.state)
        begin_motion(
            display_state,
            segments,
            lambda actual: finish_internal(request_id, command, actual),
        )

    # 函数功能：提交关门阶段并处理结果。
    def commit_stage(request_id, command, actual):
        nonlocal display_state
        pose = Pose.from_tuple(actual)
        decision = protocol.handle_command(command, pose)
        publish_status(request_id, command, decision, actual)
        if decision.result == CommandResult.SUCCEEDED:
            target_by_state = {
                DoorState.DOOR_RETURNED: DOOR_RETURNED,
                DoorState.LATCHED_HANDLE_OUT: LATCHED_HANDLE_OUT,
                DoorState.CLOSED: CLOSED,
            }
            target = target_by_state.get(decision.state)
            if target is not None:
                with viewer.lock():
                    set_joint_values(model, data, joint_ids, target)
            display_state = state_name(decision.state)
        elif decision.motion != MotionRequest.NONE:
            begin_recovery(request_id, command, decision)

    # 函数功能：启动指定关门阶段的目标运动。
    def start_close_stage(command, target, duration, expected_state):
        nonlocal display_state
        with viewer.lock():
            actual = current_joint_values(model, data, joint_ids)
        protocol.observe_pose(Pose.from_tuple(actual))
        request_id = new_request_id()
        if command == DoorCommand.COMMIT_DOOR_RETURN:
            decision = protocol.handle_command(
                DoorCommand.START_CLOSE, Pose.from_tuple(actual)
            )
            publish_status(request_id, DoorCommand.START_CLOSE, decision, actual)
            if decision.result != CommandResult.ACCEPTED:
                return
        display_state = expected_state
        begin_motion(
            expected_state,
            ((duration, target),),
            lambda final_actual: commit_stage(request_id, command, final_actual),
        )

    # 函数功能：执行调试用的强制关闭和姿态复位。
    def force_reset():
        nonlocal motion, display_state
        if motion is not None:
            announce("收到强制复位，取消当前动作并取得控制权。")
            motion = None
        with viewer.lock():
            actual = current_joint_values(model, data, joint_ids)
        request_id = new_request_id()
        decision = protocol.handle_command(
            DoorCommand.FORCE_CLOSED, Pose.from_tuple(actual)
        )
        publish_status(request_id, DoorCommand.FORCE_CLOSED, decision, actual)
        display_state = state_name(decision.state)
        begin_motion(
            "FORCE_RESETTING",
            recovery_segments(MotionRequest.FORCE_CLOSED),
            lambda final_actual: finish_internal(
                request_id, DoorCommand.FORCE_CLOSED, final_actual
            ),
        )

    # 函数功能：完成开门运动并更新显示状态。
    def finish_opening():
        nonlocal display_state
        with viewer.lock():
            actual = current_joint_values(model, data, joint_ids)
        protocol.observe_pose(Pose.from_tuple(actual))
        display_state = "OPEN"
        publish_status(
            0,
            0,
            Decision(protocol.state, CommandResult.NONE, detail="开门动作完成。"),
            actual,
        )

    # 函数功能：完成按钮动作和把手弹出流程。
    def finish_button_sequence():
        nonlocal display_state
        display_state = "POPPED"
        announce("按钮动作完成，把手已弹出 60°；可按 T。")

    # 函数功能：分发键盘动作并协调当前运动。
    def process_action(action):
        nonlocal motion, display_state

        if action in ("f", "r"):
            force_reset()
            return

        if motion is not None:
            announce("当前动作进行中，请等待或按 F/R 复位。")
            return

        if action == "b":
            if display_state != "CLOSED":
                announce("操作顺序错误：请先复位到关闭状态。")
                return
            begin_motion(
                "BUTTON_SEQUENCE",
                (
                    (0.20, (BUTTON_PRESSED, 0.0, 0.0, 0.0)),
                    (0.25, CLOSED),
                    (0.60, POPPED),
                ),
                lambda actual: finish_button_sequence(),
            )
            return

        if action == "t":
            if display_state != "POPPED":
                announce("操作顺序错误：请先按 B，使把手弹出。")
                return
            begin_motion(
                "OPENING",
                ((0.60, TURNED), (1.20, OPEN)),
                lambda actual: finish_opening(),
            )
            return

        if action == "1":
            if display_state != "OPEN":
                announce("关门顺序错误：请先将柜门完整打开到 90°。")
                return
            start_close_stage(
                DoorCommand.COMMIT_DOOR_RETURN,
                DOOR_RETURNED,
                1.20,
                "CLOSING_DOOR",
            )
            return

        if action == "2":
            if display_state != "DOOR_RETURNED":
                announce("关门顺序错误：请先完成第 1 步门回转。")
                return
            start_close_stage(
                DoorCommand.COMMIT_HANDLE_RETURN,
                LATCHED_HANDLE_OUT,
                0.60,
                "DOOR_RETURNED",
            )
            return

        if action == "3":
            if display_state != "LATCHED_HANDLE_OUT":
                announce("关门顺序错误：请先完成第 2 步门把手回转。")
                return
            start_close_stage(
                DoorCommand.COMMIT_HANDLE_LATCH,
                CLOSED,
                0.60,
                "LATCHED_HANDLE_OUT",
            )

    with mujoco.viewer.launch_passive(
        model, data, key_callback=key_callback
    ) as viewer:
        viewer.cam.lookat[:] = [0.0, 0.0, 1.13]
        viewer.cam.distance = 3.1
        viewer.cam.azimuth = 120.0
        viewer.cam.elevation = -8.0

        publish_status(
            0,
            0,
            Decision(protocol.state, CommandResult.NONE, detail="MuJoCo 初始姿态已加载。"),
            POSES[args.pose],
        )

        while viewer.is_running():
            with action_lock:
                actions = list(requested_actions)
                requested_actions[:] = []
            for action in actions:
                process_action(action)

            if motion is not None:
                duration, target = motion["segments"][motion["index"]]
                elapsed = time.monotonic() - motion["started"]
                alpha = min(1.0, elapsed / max(duration, 1e-6))
                values = tuple(
                    start + (goal - start) * alpha
                    for start, goal in zip(motion["start"], target)
                )
                with viewer.lock():
                    set_joint_values(model, data, joint_ids, values)
                if alpha >= 1.0:
                    finished = motion
                    if finished["index"] + 1 < len(finished["segments"]):
                        motion["index"] += 1
                        motion["start"] = tuple(target)
                        motion["started"] = time.monotonic()
                    else:
                        motion = None
                        with viewer.lock():
                            final_actual = current_joint_values(model, data, joint_ids)
                        finished["on_complete"](final_actual)

            # 按照 Gazebo 的方式对按钮接触执行边沿触发判断。这里同时检查接触
            # 列表和触摸传感器，因为 mj_forward 可能在动力学步产生触摸力之前
            # 就已经暴露接触信息。
            contact_active = bool(data.sensordata[touch_sensor_id] > 1e-9)
            for contact_index in range(data.ncon):
                contact = data.contact[contact_index]
                if contact.geom1 == button_geom_id or contact.geom2 == button_geom_id:
                    contact_active = True
                    break
            if not contact_active:
                touch_armed = True
            elif touch_armed and display_state == "CLOSED" and motion is None:
                touch_armed = False
                with action_lock:
                    requested_actions.append("b")

            viewer.sync()
            time.sleep(0.02)


if __name__ == "__main__":
    main()
