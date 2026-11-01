// 文件功能：声明电气柜三阶段关门状态机及其关节姿态判断接口。
#pragma once

#include <cstdint>
#include <string>

namespace electrical_cabinet_gazebo {

// 关门协议使用的圆周率、标准姿态和角度容差。
constexpr double kPi = 3.14159265358979323846;
constexpr double kDoorOpen = kPi / 2.0;
constexpr double kHandleTurned = kPi / 2.0;
constexpr double kHandlePopped = -kPi / 3.0;
constexpr double kAngleTolerance = 2.0 * kPi / 180.0;

// 关门状态机的状态，包括正常阶段、恢复阶段和强制复位阶段。
enum class DoorState : std::uint8_t {
  kUnknown = 0,
  kClosed = 1,
  kOpen = 2,
  kClosingDoor = 3,
  kDoorReturned = 4,
  kLatchedHandleOut = 5,
  kRecoveringOpen = 6,
  kRecoveringHandleOut = 7,
  kForceResetting = 8,
};

// 外部可提交的关门协议命令。
enum class DoorCommand : std::uint8_t {
  kStartClose = 1,
  kCommitDoorReturn = 2,
  kCommitHandleReturn = 3,
  kCommitHandleLatch = 4,
  kForceClosed = 5,
};

// 命令处理结果。
enum class CommandResult : std::uint8_t {
  kNone = 0,
  kAccepted = 1,
  kSucceeded = 2,
  kFailed = 3,
  kRejected = 4,
};

// 阶段检查失败项的位掩码。
enum FailedCheck : std::uint8_t {
  kCheckNone = 0,
  kCheckDoorHinge = 1,
  kCheckHandleTurn = 2,
  kCheckHandlePop = 4,
};

// 状态机请求插件执行的内部恢复动作。
enum class MotionRequest {
  kNone,
  kRecoverOpen,
  kRecoverHandleOut,
  kForceClosed,
};

// 电气柜按钮、把手和柜门关节的联合姿态。
struct JointPositions {
  double button{0.0};
  double handle_pop{0.0};
  double handle_turn{0.0};
  double door_hinge{0.0};
};

// 状态机对一次命令或内部动作给出的决策结果。
struct StateDecision {
  DoorState state{DoorState::kUnknown};
  CommandResult result{CommandResult::kNone};
  MotionRequest motion{MotionRequest::kNone};
  std::uint8_t failed_checks{kCheckNone};
  std::string detail;
};

// 实现三阶段关门协议、阶段校验和失败恢复。
class DoorCloseStateMachine {
 public:
  // 返回当前协议状态。
  DoorState state() const { return state_; }

  // 在没有进行关门/恢复流程时，识别标准姿态，使以打开姿态生成的模型和
  // 现有开门演示都能被正确识别。
  bool ObservePose(const JointPositions& positions);

  // 校验并处理一条关门协议命令。
  StateDecision HandleCommand(std::uint8_t command,
                              const JointPositions& positions);
  // 完成插件请求的恢复或强制复位动作。
  StateDecision CompleteInternalMotion();

  // 判断数值是否在协议角度容差内。
  static bool IsNear(double value, double target);
  // 判断姿态中的所有关节值是否有限。
  static bool IsFinite(const JointPositions& positions);
  // 判断姿态是否为完整打开姿态。
  static bool IsOpenPose(const JointPositions& positions);
  // 判断姿态是否为完整关闭姿态。
  static bool IsClosedPose(const JointPositions& positions);
  // 检查柜门回位阶段的关节条件。
  static std::uint8_t DoorReturnChecks(const JointPositions& positions);
  // 检查把手旋回阶段的关节条件。
  static std::uint8_t HandleReturnChecks(const JointPositions& positions);
  // 检查把手锁止阶段的关节条件。
  static std::uint8_t HandleLatchChecks(const JointPositions& positions);

 private:
  static bool IsProtocolState(DoorState state);
  static int ExpectedCommitRank(DoorState state);
  static int CommandRank(std::uint8_t command);
  StateDecision Reject(const std::string& detail) const;
  StateDecision FailToOpen(std::uint8_t checks, const std::string& detail);

  DoorState state_{DoorState::kUnknown};
};

JointPositions ClosedPositions();
JointPositions OpenPositions();
JointPositions DoorReturnedPositions();
JointPositions LatchedHandleOutPositions();

}  // 电气柜命名空间结束
