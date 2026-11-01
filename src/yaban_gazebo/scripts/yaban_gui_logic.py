#!/usr/bin/python3
"""ROS/Tk-independent feedback checks and interlocked operation sequencing."""

from dataclasses import dataclass
import math


INITIALIZE = "初始化"
LOOSEN = "松旋钮"
WITHDRAW = "压板退"
INSERT = "压板投"
TIGHTEN = "紧旋钮"
ACTIONS = (INITIALIZE, LOOSEN, WITHDRAW, INSERT, TIGHTEN)
RELEASE_ANGLE = math.pi
BLADE_UPPER = math.radians(120.0)
KNOB_UPPER = 4.0 * math.pi
POSITION_TOLERANCE = 0.001
# Match the command interface's one-moving-joint tolerance; a looser closed
# classification could unintentionally command both the blade and knob at once.
STATE_TOLERANCE = POSITION_TOLERANCE
RELEASE_TOLERANCE = 0.001
FRESHNESS_SECONDS = 1.0


@dataclass(frozen=True)
class Feedback:
    knob: float = math.nan
    blade: float = math.nan
    lift: float = math.nan
    received_at: float = 0.0
    sequence: int = 0
    connected: bool = False
    demo_enabled: object = None
    error: str = ""


@dataclass(frozen=True)
class Step:
    knob: float
    blade: float
    description: str


def unavailable_reason(feedback, now):
    if feedback.error:
        return feedback.error
    if not feedback.connected:
        return "等待 Gazebo 命令连接"
    if feedback.demo_enabled is False:
        return "被动交互模式：请以 demo_control:=true 启动"
    if feedback.demo_enabled is not True:
        return "等待 Gazebo 控制模式确认"
    if not all(math.isfinite(v) for v in (feedback.knob, feedback.blade, feedback.lift)):
        return "等待有效关节反馈"
    if feedback.sequence <= 0 or not 0.0 <= now - feedback.received_at <= FRESHNESS_SECONDS:
        return "关节反馈超时：检查 Gazebo 是否暂停或断开"
    if not (-STATE_TOLERANCE <= feedback.knob <= KNOB_UPPER + STATE_TOLERANCE
            and -STATE_TOLERANCE <= feedback.blade <= BLADE_UPPER + STATE_TOLERANCE
            and -1e-6 <= feedback.lift <= 0.001401):
        return "关节反馈超出模型范围，请检查模型参数"
    return ""


def closed(feedback):
    return abs(feedback.blade) <= STATE_TOLERANCE


def released(feedback):
    return feedback.knob >= RELEASE_ANGLE - RELEASE_TOLERANCE


def allowed_actions(feedback, now, busy=False):
    if busy or unavailable_reason(feedback, now):
        return frozenset()
    result = {INITIALIZE}
    if closed(feedback):
        if not released(feedback):
            result.add(LOOSEN)
        else:
            result.add(WITHDRAW)
        if feedback.knob > STATE_TOLERANCE:
            result.add(TIGHTEN)
    elif released(feedback):
        result.add(INSERT)
    return frozenset(result)


def state_text(feedback):
    if not all(math.isfinite(v) for v in (feedback.knob, feedback.blade)):
        return "等待模型反馈"
    if closed(feedback):
        if abs(feedback.knob) <= STATE_TOLERANCE:
            return "投 · 已锁紧"
        if released(feedback):
            return "投位 · 旋钮已松开"
        return "投位 · 旋钮未完全锁紧"
    if released(feedback):
        return "退 · 压片已拨开"
    return "压片未闭合 · 需初始化恢复"


def next_instruction(feedback):
    if not all(math.isfinite(v) for v in (feedback.knob, feedback.blade)):
        return "连接后按实测状态启用操作。"
    if closed(feedback):
        if released(feedback):
            return "可压板退；或紧旋钮，恢复锁紧。"
        if feedback.knob > STATE_TOLERANCE:
            return "可继续紧旋钮；退操作前需先松旋钮。"
        return "下一步：松旋钮 → 压板退。"
    if released(feedback):
        return "下一步：压板投 → 紧旋钮。压片闭合前禁止锁紧。"
    return "当前角度不满足操作互锁，请初始化恢复投状态。"


def make_plan(action, feedback):
    """Call only after allowed_actions validates the actual feedback."""
    knob = min(KNOB_UPPER, max(0.0, feedback.knob))
    blade = min(BLADE_UPPER, max(0.0, feedback.blade))
    if action == LOOSEN:
        return [Step(min(KNOB_UPPER, knob + math.pi), 0.0, "左旋钮逆时针旋转 180°")]
    if action == WITHDRAW:
        return [Step(knob, min(BLADE_UPPER, blade + math.pi / 2.0), "压片逆时针旋转 90°")]
    if action == INSERT:
        return [Step(knob, max(0.0, blade - math.pi / 2.0), "压片顺时针旋转，最多 90°")]
    if action == TIGHTEN:
        return [Step(max(0.0, knob - math.pi), 0.0, "左旋钮顺时针旋转，最多 180°")]
    if action == INITIALIZE:
        steps = []
        if not closed(feedback):
            if not released(feedback):
                knob = RELEASE_ANGLE
                steps.append(Step(knob, blade, "初始化 1/3：松开左旋钮"))
            steps.append(Step(knob, 0.0, "初始化 2/3：将压片转回 0°"))
        if knob > STATE_TOLERANCE:
            steps.append(Step(0.0, 0.0, "初始化 3/3：锁紧左旋钮"))
        return steps
    raise ValueError("未知操作：" + str(action))


class Sequencer:
    """One operation at a time; targets complete only on newer, fresh feedback.

    tick returns a Step to publish, or None. The caller must consume
    cancel_requested after an abort and publish the Gazebo cancel message.
    All methods are called from the GUI main thread.
    """

    def __init__(self, timeout=12.0):
        if not math.isfinite(timeout) or timeout <= 0.0:
            raise ValueError("每步超时必须是有限正数")
        self.timeout = timeout
        self.busy = False
        self.status = "连接后可直接操作；初始化将恢复投状态。"
        self.error = ""
        self.cancel_requested = False
        self._remaining = []
        self._current = None
        self._action = ""
        self._started_at = 0.0
        self._last_sent_at = 0.0
        self._start_sequence = 0

    def request(self, action, feedback, now):
        if action not in allowed_actions(feedback, now, self.busy):
            reason = unavailable_reason(feedback, now)
            if not reason:
                reason = "当前动作尚未完成" if self.busy else "当前压板状态不允许此操作"
            raise ValueError(reason)
        self._remaining = make_plan(action, feedback)
        self._current = None
        self._action = action
        self.error = ""
        self.busy = bool(self._remaining)
        self.status = "正在执行：" + action if self.busy else "初始化完成：已处于投状态并锁紧。"

    def abort(self, reason):
        if self.busy:
            self.cancel_requested = True
        self.busy = False
        self._current = None
        self._remaining = []
        self.error = reason
        self.status = "操作已中止；请检查反馈后重新操作。"

    def tick(self, feedback, now):
        if not self.busy:
            return None
        reason = unavailable_reason(feedback, now)
        if reason:
            self.abort(reason)
            return None
        if self._current is not None:
            reached = (abs(feedback.knob - self._current.knob) <= POSITION_TOLERANCE
                       and abs(feedback.blade - self._current.blade) <= POSITION_TOLERANCE)
            if feedback.sequence > self._start_sequence and reached:
                self._current = None
                if not self._remaining:
                    self.busy = False
                    self.status = self._action + "完成；已由 Gazebo 实测反馈确认。"
                    return None
            elif now - self._started_at >= self.timeout:
                self.abort(self._current.description + "超时，已取消运动目标。")
                return None
        if self._current is None:
            self._current = self._remaining.pop(0)
            self._started_at = now
            self._start_sequence = feedback.sequence
            self._last_sent_at = now
            self.status = self._current.description
            return self._current
        if now - self._last_sent_at >= 0.2:
            self._last_sent_at = now
            return self._current
        return None
