#!/usr/bin/env python3
"""Pure-Python pressure-plate interlock shared by the MuJoCo controller and tests.

The model has three qpos values, but lift is derived from the left knob angle:
0.7 mm per complete rotation.  A normal release uses +pi rad and lifts 0.35 mm.
"""

from __future__ import annotations

from dataclasses import dataclass
import math


INITIALIZE = "初始化"
LOOSEN = "松旋钮"
WITHDRAW = "压板退"
INSERT = "压板投"
TIGHTEN = "紧旋钮"
ACTIONS = (INITIALIZE, LOOSEN, WITHDRAW, INSERT, TIGHTEN)

RELEASE_ANGLE = math.pi
KNOB_UPPER = 4.0 * math.pi
BLADE_UPPER = math.radians(120.0)
SCREW_PITCH = 0.0007
POSITION_TOLERANCE = 0.001


@dataclass(frozen=True)
class Pose:
    """Kinematic state in radians; lift is a calculated metres value."""

    knob: float = 0.0
    blade: float = 0.0

    @property
    def lift(self) -> float:
        return self.knob * SCREW_PITCH / (2.0 * math.pi)


@dataclass(frozen=True)
class Step:
    target: Pose
    description: str


def finite_pose(pose: Pose) -> bool:
    return math.isfinite(pose.knob) and math.isfinite(pose.blade)


def bounded_pose(pose: Pose) -> Pose:
    if not finite_pose(pose):
        raise ValueError("关节值必须是有限数字")
    return Pose(
        knob=min(KNOB_UPPER, max(0.0, pose.knob)),
        blade=min(BLADE_UPPER, max(0.0, pose.blade)),
    )


def is_closed(pose: Pose) -> bool:
    return abs(pose.blade) <= POSITION_TOLERANCE


def is_released(pose: Pose) -> bool:
    return pose.knob >= RELEASE_ANGLE - POSITION_TOLERANCE


def state_text(pose: Pose) -> str:
    if not finite_pose(pose):
        return "等待模型反馈"
    if is_closed(pose):
        if pose.knob <= POSITION_TOLERANCE:
            return "投 · 已锁紧"
        if is_released(pose):
            return "投位 · 旋钮已松开"
        return "投位 · 旋钮未完全锁紧"
    if is_released(pose):
        return "退 · 压片已拨开"
    return "压片未闭合 · 需初始化恢复"


def next_instruction(pose: Pose) -> str:
    if not finite_pose(pose):
        return "等待模型反馈。"
    if is_closed(pose):
        if is_released(pose):
            return "下一步：压板退；或紧旋钮恢复锁紧。"
        if pose.knob > POSITION_TOLERANCE:
            return "下一步：继续紧旋钮；退操作前需先松旋钮。"
        return "下一步：松旋钮 → 压板退。"
    if is_released(pose):
        return "下一步：压板投 → 紧旋钮。压片闭合前禁止锁紧。"
    return "当前角度不满足互锁，请初始化恢复投状态。"


def allowed_actions(pose: Pose, busy: bool = False) -> frozenset[str]:
    """Return only actions permitted by the measured model pose."""
    if busy or not finite_pose(pose):
        return frozenset()
    pose = bounded_pose(pose)
    result = {INITIALIZE}
    if is_closed(pose):
        if not is_released(pose):
            result.add(LOOSEN)
        else:
            result.add(WITHDRAW)
        if pose.knob > POSITION_TOLERANCE:
            result.add(TIGHTEN)
    elif is_released(pose):
        result.add(INSERT)
    return frozenset(result)


def make_plan(action: str, measured: Pose) -> list[Step]:
    """Build one sequential operation from a valid measured pose.

    Callers must first check ``action in allowed_actions(measured)``.  Every
    resulting target is clamped to the physical 0--120 degree blade limit.
    """
    pose = bounded_pose(measured)
    if action == LOOSEN:
        return [Step(Pose(min(KNOB_UPPER, pose.knob + math.pi), 0.0), "左旋钮逆时针旋转 180°")]
    if action == WITHDRAW:
        return [Step(Pose(pose.knob, min(BLADE_UPPER, pose.blade + math.pi / 2.0)), "压片逆时针旋转 90°")]
    if action == INSERT:
        return [Step(Pose(pose.knob, max(0.0, pose.blade - math.pi / 2.0)), "压片顺时针旋转，最多 90°")]
    if action == TIGHTEN:
        return [Step(Pose(max(0.0, pose.knob - math.pi), 0.0), "左旋钮顺时针旋转，最多 180°")]
    if action == INITIALIZE:
        steps: list[Step] = []
        knob, blade = pose.knob, pose.blade
        if not is_closed(pose):
            if not is_released(pose):
                knob = RELEASE_ANGLE
                steps.append(Step(Pose(knob, blade), "初始化 1/3：松开左旋钮"))
            blade = 0.0
            steps.append(Step(Pose(knob, blade), "初始化 2/3：将压片转回 0°"))
        if knob > POSITION_TOLERANCE:
            steps.append(Step(Pose(0.0, 0.0), "初始化 3/3：锁紧左旋钮"))
        return steps
    raise ValueError("未知操作：{}".format(action))


def action_error(action: str, pose: Pose, busy: bool = False) -> str:
    if action not in ACTIONS:
        return "未知操作：{}".format(action)
    if not finite_pose(pose):
        return "等待有效模型反馈"
    if busy:
        return "当前动作尚未完成"
    return "当前压板状态不允许此操作"
