// 文件功能：实现电气柜关门状态机的姿态判断、阶段提交和恢复决策。
#include "electrical_cabinet_gazebo/door_close_state_machine.hpp"

#include <cmath>

namespace electrical_cabinet_gazebo {

namespace {

// 将单个关节的偏差转换为对应的失败检查位。
std::uint8_t CheckNear(double value, double target, std::uint8_t bit) {
  return DoorCloseStateMachine::IsNear(value, target) ? kCheckNone : bit;
}

}  // 匿名命名空间结束

// 以下函数返回关门协议各阶段使用的标准关节姿态。
JointPositions ClosedPositions() { return JointPositions{}; }

JointPositions OpenPositions() {
  JointPositions positions;
  positions.handle_pop = kHandlePopped;
  positions.handle_turn = kHandleTurned;
  positions.door_hinge = kDoorOpen;
  return positions;
}

JointPositions DoorReturnedPositions() {
  JointPositions positions = OpenPositions();
  positions.door_hinge = 0.0;
  return positions;
}

JointPositions LatchedHandleOutPositions() {
  JointPositions positions = DoorReturnedPositions();
  positions.handle_turn = 0.0;
  return positions;
}

bool DoorCloseStateMachine::IsNear(double value, double target) {
  return std::isfinite(value) &&
         std::abs(value - target) <= kAngleTolerance + 1e-12;
}

// 以下检查函数负责识别标准姿态并生成阶段失败位掩码。
bool DoorCloseStateMachine::IsFinite(const JointPositions& positions) {
  return std::isfinite(positions.button) &&
         std::isfinite(positions.handle_pop) &&
         std::isfinite(positions.handle_turn) &&
         std::isfinite(positions.door_hinge);
}

bool DoorCloseStateMachine::IsOpenPose(const JointPositions& positions) {
  return IsFinite(positions) && IsNear(positions.button, 0.0) &&
         IsNear(positions.handle_pop, kHandlePopped) &&
         IsNear(positions.handle_turn, kHandleTurned) &&
         IsNear(positions.door_hinge, kDoorOpen);
}

bool DoorCloseStateMachine::IsClosedPose(const JointPositions& positions) {
  return IsFinite(positions) && IsNear(positions.button, 0.0) &&
         IsNear(positions.handle_pop, 0.0) &&
         IsNear(positions.handle_turn, 0.0) &&
         IsNear(positions.door_hinge, 0.0);
}

std::uint8_t DoorCloseStateMachine::DoorReturnChecks(
    const JointPositions& positions) {
  if (!IsFinite(positions)) {
    return kCheckDoorHinge | kCheckHandleTurn | kCheckHandlePop;
  }
  return CheckNear(positions.door_hinge, 0.0, kCheckDoorHinge) |
         CheckNear(positions.handle_turn, kHandleTurned, kCheckHandleTurn) |
         CheckNear(positions.handle_pop, kHandlePopped, kCheckHandlePop);
}

std::uint8_t DoorCloseStateMachine::HandleReturnChecks(
    const JointPositions& positions) {
  if (!IsFinite(positions)) {
    return kCheckDoorHinge | kCheckHandleTurn | kCheckHandlePop;
  }
  return CheckNear(positions.door_hinge, 0.0, kCheckDoorHinge) |
         CheckNear(positions.handle_turn, 0.0, kCheckHandleTurn) |
         CheckNear(positions.handle_pop, kHandlePopped, kCheckHandlePop);
}

std::uint8_t DoorCloseStateMachine::HandleLatchChecks(
    const JointPositions& positions) {
  if (!IsFinite(positions)) {
    return kCheckDoorHinge | kCheckHandleTurn | kCheckHandlePop;
  }
  return CheckNear(positions.door_hinge, 0.0, kCheckDoorHinge) |
         CheckNear(positions.handle_turn, 0.0, kCheckHandleTurn) |
         CheckNear(positions.handle_pop, 0.0, kCheckHandlePop);
}

bool DoorCloseStateMachine::IsProtocolState(DoorState state) {
  return state == DoorState::kClosingDoor ||
         state == DoorState::kDoorReturned ||
         state == DoorState::kLatchedHandleOut;
}

// 将状态和外部命令映射为协议阶段序号，用于防止越级或逆序提交。
int DoorCloseStateMachine::ExpectedCommitRank(DoorState state) {
  switch (state) {
    case DoorState::kClosingDoor:
      return 1;
    case DoorState::kDoorReturned:
      return 2;
    case DoorState::kLatchedHandleOut:
      return 3;
    default:
      return 0;
  }
}

int DoorCloseStateMachine::CommandRank(std::uint8_t command) {
  switch (static_cast<DoorCommand>(command)) {
    case DoorCommand::kCommitDoorReturn:
      return 1;
    case DoorCommand::kCommitHandleReturn:
      return 2;
    case DoorCommand::kCommitHandleLatch:
      return 3;
    default:
      return 0;
  }
}

bool DoorCloseStateMachine::ObservePose(const JointPositions& positions) {
  if (IsProtocolState(state_) || state_ == DoorState::kRecoveringOpen ||
      state_ == DoorState::kRecoveringHandleOut ||
      state_ == DoorState::kForceResetting) {
    return false;
  }

  DoorState observed = DoorState::kUnknown;
  if (IsClosedPose(positions)) {
    observed = DoorState::kClosed;
  } else if (IsOpenPose(positions)) {
    observed = DoorState::kOpen;
  }
  if (observed == state_) {
    return false;
  }
  state_ = observed;
  return true;
}

// 构造拒绝或恢复决策，统一填充当前状态和错误详情。
StateDecision DoorCloseStateMachine::Reject(const std::string& detail) const {
  StateDecision decision;
  decision.state = state_;
  decision.result = CommandResult::kRejected;
  decision.detail = detail;
  return decision;
}

StateDecision DoorCloseStateMachine::FailToOpen(std::uint8_t checks,
                                                 const std::string& detail) {
  state_ = DoorState::kRecoveringOpen;
  StateDecision decision;
  decision.state = state_;
  decision.result = CommandResult::kFailed;
  decision.motion = MotionRequest::kRecoverOpen;
  decision.failed_checks = checks;
  decision.detail = detail;
  return decision;
}

StateDecision DoorCloseStateMachine::HandleCommand(
    std::uint8_t command, const JointPositions& positions) {
  if (command == static_cast<std::uint8_t>(DoorCommand::kForceClosed)) {
    state_ = DoorState::kForceResetting;
    StateDecision decision;
    decision.state = state_;
    decision.result = CommandResult::kAccepted;
    decision.motion = MotionRequest::kForceClosed;
    decision.detail = "已接受强制复位。";
    return decision;
  }

  if (command == static_cast<std::uint8_t>(DoorCommand::kStartClose)) {
    if (state_ != DoorState::kOpen || !IsOpenPose(positions)) {
      return Reject("柜门不在完整打开位，不能开始关门。");
    }
    state_ = DoorState::kClosingDoor;
    StateDecision decision;
    decision.state = state_;
    decision.result = CommandResult::kAccepted;
    decision.detail = "已开始门回转阶段。";
    return decision;
  }

  const int command_rank = CommandRank(command);
  if (command_rank == 0) {
    return Reject("未知关门命令。");
  }

  if (!IsProtocolState(state_)) {
    return Reject("当前状态不接受该阶段提交。");
  }

  const int expected_rank = ExpectedCommitRank(state_);
  if (command_rank != expected_rank) {
    if (command_rank > expected_rank) {
      std::uint8_t checks = kCheckNone;
      if (command_rank == 2) {
        checks = HandleReturnChecks(positions);
      } else if (command_rank == 3) {
        checks = HandleLatchChecks(positions);
      }
      if (checks == kCheckNone) {
        checks = kCheckDoorHinge | kCheckHandleTurn;
      }
      return FailToOpen(checks, "关门阶段被越级提交，柜门将恢复打开。");
    }
    return Reject("该阶段已经完成，拒绝重复或逆序提交。");
  }

  if (state_ == DoorState::kClosingDoor) {
    const std::uint8_t checks = DoorReturnChecks(positions);
    if (checks != kCheckNone) {
      return FailToOpen(checks, "门回转未满足锁扣条件，柜门将恢复打开。");
    }
    state_ = DoorState::kDoorReturned;
    StateDecision decision;
    decision.state = state_;
    decision.result = CommandResult::kSucceeded;
    decision.detail = "门已回转到柜体，可进行门把手回转。";
    return decision;
  }

  if (state_ == DoorState::kDoorReturned) {
    const std::uint8_t checks = HandleReturnChecks(positions);
    if (checks != kCheckNone) {
      return FailToOpen(checks, "门或门把手回转未满足锁扣条件，柜门将恢复打开。");
    }
    state_ = DoorState::kLatchedHandleOut;
    StateDecision decision;
    decision.state = state_;
    decision.result = CommandResult::kSucceeded;
    decision.detail = "柜门已扣上，把手保持外倾 60 度。";
    return decision;
  }

  const std::uint8_t checks = HandleLatchChecks(positions);
  if (checks != kCheckNone) {
    state_ = DoorState::kRecoveringHandleOut;
    StateDecision decision;
    decision.state = state_;
    decision.result = CommandResult::kFailed;
    decision.motion = MotionRequest::kRecoverHandleOut;
    decision.failed_checks = checks;
    decision.detail = "把手回扣未转满 60 度，将退回完整外倾位置。";
    return decision;
  }

  state_ = DoorState::kClosed;
  StateDecision decision;
  decision.state = state_;
  decision.result = CommandResult::kSucceeded;
  decision.detail = "把手已贴紧门锁，关门完成。";
  return decision;
}

// 完成插件执行的恢复或强制复位动作，并回写状态机状态。
StateDecision DoorCloseStateMachine::CompleteInternalMotion() {
  StateDecision decision;
  switch (state_) {
    case DoorState::kRecoveringOpen:
      state_ = DoorState::kOpen;
      decision.result = CommandResult::kFailed;
      decision.detail = "恢复完成，柜门已回到完整打开状态。";
      break;
    case DoorState::kRecoveringHandleOut:
      state_ = DoorState::kLatchedHandleOut;
      decision.result = CommandResult::kFailed;
      decision.detail = "把手已退回外倾 60 度，可重新回扣。";
      break;
    case DoorState::kForceResetting:
      state_ = DoorState::kClosed;
      decision.result = CommandResult::kSucceeded;
      decision.detail = "强制复位完成，柜门已回到关闭零位。";
      break;
    default:
      return Reject("当前没有待完成的内部动作。");
  }
  decision.state = state_;
  return decision;
}

}  // 电气柜命名空间结束
