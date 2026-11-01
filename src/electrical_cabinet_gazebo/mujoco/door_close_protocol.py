"""电气柜关门协议的 MuJoCo 侧实现。

本模块不依赖 MuJoCo 或 ROS。交互式查看器使用这里的状态检查和恢复决策，
以便与 Gazebo 插件保持一致；单元测试也可以直接使用普通元组测试协议。
"""

from __future__ import annotations

import math
from dataclasses import dataclass
from enum import Enum, IntEnum
from typing import Optional, Tuple


# ===== 模块级常量与辅助定义 =====
PI = math.pi
DOOR_OPEN = PI / 2.0
HANDLE_TURNED = PI / 2.0
HANDLE_POPPED = -PI / 3.0
ANGLE_TOLERANCE = math.radians(2.0)


# 类功能：封装 DoorState 相关的状态、数据和操作。
class DoorState(IntEnum):
    UNKNOWN = 0
    CLOSED = 1
    OPEN = 2
    CLOSING_DOOR = 3
    DOOR_RETURNED = 4
    LATCHED_HANDLE_OUT = 5
    RECOVERING_OPEN = 6
    RECOVERING_HANDLE_OUT = 7
    FORCE_RESETTING = 8


# 类功能：封装 DoorCommand 相关的状态、数据和操作。
class DoorCommand(IntEnum):
    START_CLOSE = 1
    COMMIT_DOOR_RETURN = 2
    COMMIT_HANDLE_RETURN = 3
    COMMIT_HANDLE_LATCH = 4
    FORCE_CLOSED = 5


# 类功能：封装 CommandResult 相关的状态、数据和操作。
class CommandResult(IntEnum):
    NONE = 0
    ACCEPTED = 1
    SUCCEEDED = 2
    FAILED = 3
    REJECTED = 4


# 类功能：封装 MotionRequest 相关的状态、数据和操作。
class MotionRequest(Enum):
    NONE = "none"
    RECOVER_OPEN = "recover_open"
    RECOVER_HANDLE_OUT = "recover_handle_out"
    FORCE_CLOSED = "force_closed"


CHECK_NONE = 0
CHECK_DOOR_HINGE = 1
CHECK_HANDLE_TURN = 2
CHECK_HANDLE_POP = 4


# 类功能：封装 Pose 相关的状态、数据和操作。
@dataclass(frozen=True)
class Pose:
    """关节值顺序为 [button, handle_pop, handle_turn, door_hinge]。"""

    button: float = 0.0
    handle_pop: float = 0.0
    handle_turn: float = 0.0
    door_hinge: float = 0.0

    # 函数功能：将姿态转换为约定顺序的关节值元组。
    def as_tuple(self) -> Tuple[float, float, float, float]:
        return (
            self.button,
            self.handle_pop,
            self.handle_turn,
            self.door_hinge,
        )

    # 函数功能：从关节值序列构造姿态对象。
    @classmethod
    def from_tuple(cls, values) -> "Pose":
        button, handle_pop, handle_turn, door_hinge = values
        return cls(float(button), float(handle_pop), float(handle_turn), float(door_hinge))


# 类功能：封装 Decision 相关的状态、数据和操作。
@dataclass(frozen=True)
class Decision:
    state: DoorState
    result: CommandResult
    motion: MotionRequest = MotionRequest.NONE
    failed_checks: int = CHECK_NONE
    detail: str = ""


# 函数功能：返回电气柜完全关闭时的标准姿态。
def closed_pose() -> Pose:
    return Pose()


# 函数功能：返回电气柜完全打开时的标准姿态。
def open_pose() -> Pose:
    return Pose(handle_pop=HANDLE_POPPED, handle_turn=HANDLE_TURNED, door_hinge=DOOR_OPEN)


# 函数功能：返回柜门回位但把手仍外伸的中间姿态。
def door_returned_pose() -> Pose:
    return Pose(handle_pop=HANDLE_POPPED, handle_turn=HANDLE_TURNED)


# 函数功能：返回柜门锁住但把手仍外伸的中间姿态。
def latched_handle_out_pose() -> Pose:
    return Pose(handle_pop=HANDLE_POPPED)


# 类功能：封装 DoorCloseStateMachine 相关的状态、数据和操作。
class DoorCloseStateMachine:
    """封装 MuJoCo 查看器所有操作共用的状态检查和状态转换。"""

    # 函数功能：初始化对象成员和运行状态。
    def __init__(self):
        self._state = DoorState.UNKNOWN

    # 函数功能：读取当前协议或环境状态。
    @property
    def state(self) -> DoorState:
        return self._state

    # 函数功能：判断关节值是否在角度容差内。
    @staticmethod
    def is_near(value: float, target: float) -> bool:
        return math.isfinite(value) and abs(value - target) <= ANGLE_TOLERANCE + 1e-12

    # 函数功能：检查姿态值是否全部为有限数。
    @staticmethod
    def is_finite(pose: Pose) -> bool:
        return all(math.isfinite(value) for value in pose.as_tuple())

    # 函数功能：判断姿态是否符合完全打开条件。
    @classmethod
    def is_open_pose(cls, pose: Pose) -> bool:
        return cls.is_finite(pose) and all(
            (
                cls.is_near(pose.button, 0.0),
                cls.is_near(pose.handle_pop, HANDLE_POPPED),
                cls.is_near(pose.handle_turn, HANDLE_TURNED),
                cls.is_near(pose.door_hinge, DOOR_OPEN),
            )
        )

    # 函数功能：判断姿态是否符合完全关闭条件。
    @classmethod
    def is_closed_pose(cls, pose: Pose) -> bool:
        return cls.is_finite(pose) and all(
            (
                cls.is_near(pose.button, 0.0),
                cls.is_near(pose.handle_pop, 0.0),
                cls.is_near(pose.handle_turn, 0.0),
                cls.is_near(pose.door_hinge, 0.0),
            )
        )

    # 函数功能：检查柜门回位阶段的关节条件。
    @classmethod
    def door_return_checks(cls, pose: Pose) -> int:
        if not cls.is_finite(pose):
            return CHECK_DOOR_HINGE | CHECK_HANDLE_TURN | CHECK_HANDLE_POP
        return (
            (CHECK_NONE if cls.is_near(pose.door_hinge, 0.0) else CHECK_DOOR_HINGE)
            | (CHECK_NONE if cls.is_near(pose.handle_turn, HANDLE_TURNED) else CHECK_HANDLE_TURN)
            | (CHECK_NONE if cls.is_near(pose.handle_pop, HANDLE_POPPED) else CHECK_HANDLE_POP)
        )

    # 函数功能：检查把手旋回阶段的关节条件。
    @classmethod
    def handle_return_checks(cls, pose: Pose) -> int:
        if not cls.is_finite(pose):
            return CHECK_DOOR_HINGE | CHECK_HANDLE_TURN | CHECK_HANDLE_POP
        return (
            (CHECK_NONE if cls.is_near(pose.door_hinge, 0.0) else CHECK_DOOR_HINGE)
            | (CHECK_NONE if cls.is_near(pose.handle_turn, 0.0) else CHECK_HANDLE_TURN)
            | (CHECK_NONE if cls.is_near(pose.handle_pop, HANDLE_POPPED) else CHECK_HANDLE_POP)
        )

    # 函数功能：检查把手锁止阶段的关节条件。
    @classmethod
    def handle_latch_checks(cls, pose: Pose) -> int:
        if not cls.is_finite(pose):
            return CHECK_DOOR_HINGE | CHECK_HANDLE_TURN | CHECK_HANDLE_POP
        return (
            (CHECK_NONE if cls.is_near(pose.door_hinge, 0.0) else CHECK_DOOR_HINGE)
            | (CHECK_NONE if cls.is_near(pose.handle_turn, 0.0) else CHECK_HANDLE_TURN)
            | (CHECK_NONE if cls.is_near(pose.handle_pop, 0.0) else CHECK_HANDLE_POP)
        )

    # 函数功能：根据实际姿态更新协议状态。
    def observe_pose(self, pose: Pose) -> bool:
        if self._state in (
            DoorState.CLOSING_DOOR,
            DoorState.DOOR_RETURNED,
            DoorState.LATCHED_HANDLE_OUT,
            DoorState.RECOVERING_OPEN,
            DoorState.RECOVERING_HANDLE_OUT,
            DoorState.FORCE_RESETTING,
        ):
            return False
        observed = DoorState.UNKNOWN
        if self.is_closed_pose(pose):
            observed = DoorState.CLOSED
        elif self.is_open_pose(pose):
            observed = DoorState.OPEN
        if observed == self._state:
            return False
        self._state = observed
        return True

    # 函数功能：设置调试姿态并返回对应状态。
    def seed_pose(self, pose: Pose) -> DoorState:
        """设置用户明确指定的调试姿态，包括中间阶段姿态。"""
        if self.is_closed_pose(pose):
            self._state = DoorState.CLOSED
        elif self.is_open_pose(pose):
            self._state = DoorState.OPEN
        elif self.door_return_checks(pose) == CHECK_NONE:
            self._state = DoorState.DOOR_RETURNED
        elif self.handle_return_checks(pose) == CHECK_NONE:
            self._state = DoorState.LATCHED_HANDLE_OUT
        else:
            self._state = DoorState.UNKNOWN
        return self._state

    # 函数功能：将关门命令转换为阶段等级。
    @staticmethod
    def _command_rank(command: DoorCommand) -> int:
        return {
            DoorCommand.COMMIT_DOOR_RETURN: 1,
            DoorCommand.COMMIT_HANDLE_RETURN: 2,
            DoorCommand.COMMIT_HANDLE_LATCH: 3,
        }.get(command, 0)

    # 函数功能：计算当前状态对应的下一阶段等级。
    @staticmethod
    def _expected_rank(state: DoorState) -> int:
        return {
            DoorState.CLOSING_DOOR: 1,
            DoorState.DOOR_RETURNED: 2,
            DoorState.LATCHED_HANDLE_OUT: 3,
        }.get(state, 0)

    # 函数功能：判断状态是否属于关门协议阶段。
    @staticmethod
    def _is_protocol_state(state: DoorState) -> bool:
        return state in (
            DoorState.CLOSING_DOOR,
            DoorState.DOOR_RETURNED,
            DoorState.LATCHED_HANDLE_OUT,
        )

    # 函数功能：生成拒绝当前命令的决策结果。
    def _reject(self, detail: str) -> Decision:
        return Decision(self._state, CommandResult.REJECTED, detail=detail)

    # 函数功能：记录失败检查项并请求恢复到打开姿态。
    def _fail_open(self, checks: int, detail: str) -> Decision:
        self._state = DoorState.RECOVERING_OPEN
        return Decision(
            self._state,
            CommandResult.FAILED,
            motion=MotionRequest.RECOVER_OPEN,
            failed_checks=checks,
            detail=detail,
        )

    # 函数功能：校验命令并推进关门状态机。
    def handle_command(self, command_value: int, pose: Pose) -> Decision:
        try:
            command = DoorCommand(int(command_value))
        except (TypeError, ValueError):
            return self._reject("未知关门命令。")

        if command == DoorCommand.FORCE_CLOSED:
            self._state = DoorState.FORCE_RESETTING
            return Decision(
                self._state,
                CommandResult.ACCEPTED,
                motion=MotionRequest.FORCE_CLOSED,
                detail="已接受强制复位。",
            )

        if command == DoorCommand.START_CLOSE:
            if self._state != DoorState.OPEN or not self.is_open_pose(pose):
                return self._reject("柜门不在完整打开位，不能开始关门。")
            self._state = DoorState.CLOSING_DOOR
            return Decision(self._state, CommandResult.ACCEPTED, detail="已开始门回转阶段。")

        rank = self._command_rank(command)
        if rank == 0:
            return self._reject("未知关门命令。")
        if not self._is_protocol_state(self._state):
            return self._reject("当前状态不接受该阶段提交。")

        expected = self._expected_rank(self._state)
        if rank != expected:
            if rank > expected:
                checks = CHECK_NONE
                if rank == 2:
                    checks = self.handle_return_checks(pose)
                elif rank == 3:
                    checks = self.handle_latch_checks(pose)
                if checks == CHECK_NONE:
                    checks = CHECK_DOOR_HINGE | CHECK_HANDLE_TURN
                return self._fail_open(checks, "关门阶段被越级提交，柜门将恢复打开。")
            return self._reject("该阶段已经完成，拒绝重复或逆序提交。")

        if self._state == DoorState.CLOSING_DOOR:
            checks = self.door_return_checks(pose)
            if checks:
                return self._fail_open(checks, "门回转未满足锁扣条件，柜门将恢复打开。")
            self._state = DoorState.DOOR_RETURNED
            return Decision(self._state, CommandResult.SUCCEEDED, detail="门已回转到柜体，可进行门把手回转。")

        if self._state == DoorState.DOOR_RETURNED:
            checks = self.handle_return_checks(pose)
            if checks:
                return self._fail_open(checks, "门或门把手回转未满足锁扣条件，柜门将恢复打开。")
            self._state = DoorState.LATCHED_HANDLE_OUT
            return Decision(self._state, CommandResult.SUCCEEDED, detail="柜门已扣上，把手保持外倾 60 度。")

        checks = self.handle_latch_checks(pose)
        if checks:
            self._state = DoorState.RECOVERING_HANDLE_OUT
            return Decision(
                self._state,
                CommandResult.FAILED,
                motion=MotionRequest.RECOVER_HANDLE_OUT,
                failed_checks=checks,
                detail="把手回扣未转满 60 度，将退回完整外倾位置。",
            )
        self._state = DoorState.CLOSED
        return Decision(self._state, CommandResult.SUCCEEDED, detail="把手已贴紧门锁，关门完成。")

    # 函数功能：完成内部运动并确定恢复后的状态。
    def complete_internal_motion(self) -> Decision:
        if self._state == DoorState.RECOVERING_OPEN:
            self._state = DoorState.OPEN
            return Decision(self._state, CommandResult.FAILED, detail="恢复完成，柜门已回到完整打开状态。")
        if self._state == DoorState.RECOVERING_HANDLE_OUT:
            self._state = DoorState.LATCHED_HANDLE_OUT
            return Decision(self._state, CommandResult.FAILED, detail="把手已退回外倾 60 度，可重新回扣。")
        if self._state == DoorState.FORCE_RESETTING:
            self._state = DoorState.CLOSED
            return Decision(self._state, CommandResult.SUCCEEDED, detail="强制复位完成，柜门已回到关闭零位。")
        return Decision(self._state, CommandResult.REJECTED, detail="当前没有待完成的内部动作。")
