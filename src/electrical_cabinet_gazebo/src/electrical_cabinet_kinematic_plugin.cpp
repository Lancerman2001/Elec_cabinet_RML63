#include <cabinet_sim_support/transport.hpp>
// 文件功能：实现 Gazebo 电气柜运动学插件，负责关节目标控制、状态发布和关门协议。
#include <algorithm>
#include <array>
#include <cmath>
#include <cstdint>
#include <deque>
#include <functional>
#include <limits>
#include <mutex>
#include <string>
#include <utility>

#include <electrical_cabinet_gazebo/msg/door_close_command.hpp>
#include <electrical_cabinet_gazebo/msg/door_close_status.hpp>
#include <gazebo/common/Events.hh>
#include <gazebo/gazebo.hh>
#include <gazebo/physics/Joint.hh>
#include <gazebo/physics/Link.hh>
#include <gazebo/physics/Model.hh>
#include <gazebo/physics/World.hh>
#include <gazebo/physics/physics.hh>
#include <std_msgs/msg/float64_multi_array.hpp>

#include "electrical_cabinet_gazebo/door_close_state_machine.hpp"

namespace electrical_cabinet_gazebo {

namespace {

// ROS 消息中的协议值必须与本地状态机枚举保持一致。
using CommandMessage = electrical_cabinet_gazebo::msg::DoorCloseCommand;
using StatusMessage = electrical_cabinet_gazebo::msg::DoorCloseStatus;

static_assert(CommandMessage::START_CLOSE ==
                  static_cast<std::uint8_t>(DoorCommand::kStartClose),
              "Door close command values must match the state machine");
static_assert(CommandMessage::FORCE_CLOSED ==
                  static_cast<std::uint8_t>(DoorCommand::kForceClosed),
              "Door close command values must match the state machine");
static_assert(StatusMessage::STATE_FORCE_RESETTING ==
                  static_cast<std::uint8_t>(DoorState::kForceResetting),
              "Door state values must match the status message");

enum JointIndex : std::size_t {
  kButtonIndex = 0,
  kHandlePopIndex = 1,
  kHandleTurnIndex = 2,
  kDoorHingeIndex = 3,
  kJointCount = 4,
};

// 在消息数组和状态机姿态结构之间转换。
std::array<double, kJointCount> ToArray(const JointPositions& positions) {
  return {{positions.button, positions.handle_pop, positions.handle_turn,
           positions.door_hinge}};
}

JointPositions FromArray(const std::array<double, kJointCount>& values) {
  JointPositions positions;
  positions.button = values[kButtonIndex];
  positions.handle_pop = values[kHandlePopIndex];
  positions.handle_turn = values[kHandleTurnIndex];
  positions.door_hinge = values[kDoorHingeIndex];
  return positions;
}

}  // 匿名命名空间结束

// Gazebo 运动学插件：接收关节/关门命令，推进内部运动并发布状态。
class ElectricalCabinetKinematicPlugin : public gazebo::ModelPlugin {
 public:
  // 构造函数不执行 ROS/Gazebo 初始化，初始化统一在 Load 中完成。
  ElectricalCabinetKinematicPlugin() = default;
  void Reset() override {
    std::lock_guard<std::mutex> lock(input_mutex_);
    initial_applied_ = false;
    initial_status_published_ = false;
    has_applied_position_.fill(false);
    close_commands_.clear();
    motion_segments_.clear();
    raw_sequence_ = consumed_raw_sequence_ = 0;
    canonical_hold_active_ = false;
    state_machine_ = DoorCloseStateMachine();
  }
  // 释放更新连接、订阅器和状态发布器。
  ~ElectricalCabinetKinematicPlugin() override {
    update_connection_.reset();
    command_sub_.reset();
    close_command_sub_.reset();
    status_pub_.reset();
  }

  // 初始化关节、ROS 接口、运动学 link 和 Gazebo 更新回调。
  void Load(gazebo::physics::ModelPtr model, sdf::ElementPtr sdf) override {
    model_ = std::move(model);
    initial_open_ = !sdf->HasElement("initial_pose") || sdf->Get<std::string>("initial_pose") == "open";

    if (!rclcpp::ok()) { gzerr << "Initialize ROS 2 with gazebo_ros before loading plugins\n"; return; }

    node_ = cabinet_sim::node(sdf, "/electrical_cabinet");
    command_sub_ = cabinet_sim::subscribe(node_, 
        "kinematic_command", 1,
        &ElectricalCabinetKinematicPlugin::CommandCallback, this);
    close_command_sub_ = cabinet_sim::subscribe(node_, 
        "door_close/command", 20,
        &ElectricalCabinetKinematicPlugin::CloseCommandCallback, this);
    status_pub_ = cabinet_sim::advertise<StatusMessage>(node_, "door_close/status", 10, true);

    const std::array<std::string, kJointCount> names = {
        "button_press_joint", "handle_pop_joint", "handle_turn_joint",
        "door_hinge_joint"};
    const std::array<double, kJointCount> fallback_lower = {
        {-0.0001, kHandlePopped, -0.0001, -0.0001}};
    const std::array<double, kJointCount> fallback_upper = {
        {kPi, 0.0001, kHandleTurned, kDoorOpen}};

    all_joints_available_ = true;
    for (std::size_t i = 0; i < names.size(); ++i) {
      joints_[i] = model_->GetJoint(names[i]);
      lower_limits_[i] = fallback_lower[i];
      upper_limits_[i] = fallback_upper[i];
      if (!joints_[i]) {
        all_joints_available_ = false;
        CAB_ERROR_STREAM("Cannot find cabinet joint: " << names[i]);
        continue;
      }
      const double lower = joints_[i]->LowerLimit(0);
      const double upper = joints_[i]->UpperLimit(0);
      if (std::isfinite(lower) && std::isfinite(upper) && lower <= upper) {
        lower_limits_[i] = lower;
        upper_limits_[i] = upper;
      }
    }

    // 该模型仍用于可视化和交互。运动学链接保留按钮接触传感器，同时避免
    // 物理约束与明确命令的柜门和锁具姿态相互冲突。
    const std::array<std::string, 8> kinematic_links = {
        "cabinet_base_link", "door_hinge_link", "door_link",
        "lock_cavity_link", "lock_frame_link", "lock_button_link",
        "button_contact_sensor_link", "handle_pop_link"};
    for (const std::string& name : kinematic_links) {
      gazebo::physics::LinkPtr link = model_->GetLink(name);
      if (link) {
        link->SetGravityMode(false);
        link->SetSelfCollide(false);
        link->SetKinematic(true);
      }
    }
    if (gazebo::physics::LinkPtr handle = model_->GetLink("door_handle_link")) {
      handle->SetGravityMode(false);
      handle->SetSelfCollide(false);
      handle->SetKinematic(true);
    }

    update_connection_ = gazebo::event::Events::ConnectWorldUpdateBegin(
        std::bind(&ElectricalCabinetKinematicPlugin::OnUpdate, this));
    CAB_INFO("Electrical cabinet kinematic and three-stage close plugin is ready");
  }

 private:
  // 一段内部恢复运动的起点、终点和持续时间。
  struct MotionSegment {
    JointPositions start;
    JointPositions target;
    double duration{0.0};
  };

  // 接收底层四关节目标并缓存为下一次更新使用的快照。
  void CommandCallback(const std_msgs::msg::Float64MultiArray::ConstSharedPtr& message) {
    if (message->data.size() < kJointCount) {
      CAB_WARN_THROTTLE(2.0,
                        "Cabinet kinematic command needs four joint values");
      return;
    }

    std::array<double, kJointCount> values{};
    for (std::size_t i = 0; i < values.size(); ++i) {
      if (!std::isfinite(message->data[i])) {
        CAB_WARN_THROTTLE(
            2.0, "Cabinet kinematic command contains a non-finite value");
        return;
      }
      values[i] = message->data[i];
    }

    std::lock_guard<std::mutex> lock(input_mutex_);
    raw_targets_ = values;
    ++raw_sequence_;
  }

  // 缓存关门协议命令，统一在 Gazebo 更新线程中处理。
  void CloseCommandCallback(const CommandMessage::ConstSharedPtr& message) {
    std::lock_guard<std::mutex> lock(input_mutex_);
    close_commands_.push_back(*message);
  }

  // 读取 Gazebo 当前四个受控关节的位置。
  JointPositions ReadPositions() const {
    JointPositions positions;
    if (!all_joints_available_) {
      const double invalid = std::numeric_limits<double>::quiet_NaN();
      positions.button = invalid;
      positions.handle_pop = invalid;
      positions.handle_turn = invalid;
      positions.door_hinge = invalid;
      return positions;
    }
    positions.button = joints_[kButtonIndex]->Position(0);
    positions.handle_pop = joints_[kHandlePopIndex]->Position(0);
    positions.handle_turn = joints_[kHandleTurnIndex]->Position(0);
    positions.door_hinge = joints_[kDoorHingeIndex]->Position(0);
    return positions;
  }

  // 将外部目标限制在模型关节的实际范围内。
  JointPositions ClampToLimits(const JointPositions& positions) const {
    std::array<double, kJointCount> values = ToArray(positions);
    for (std::size_t i = 0; i < values.size(); ++i) {
      values[i] = std::max(lower_limits_[i],
                           std::min(upper_limits_[i], values[i]));
    }
    return FromArray(values);
  }

  // 按父子顺序将关节目标写入 Gazebo，并避免重复写入相同目标。
  void ApplyPositions(const JointPositions& requested, bool force = false) {
    const std::array<double, kJointCount> values =
        ToArray(ClampToLimits(requested));

    // 先更新父关节，再更新子关节。这样即使 Gazebo 合并固定关节，也能保持
    // Rx(turn) * Ry(pop) 的把手组合变换。
    const std::array<std::size_t, kJointCount> update_order = {
        {kDoorHingeIndex, kHandleTurnIndex, kHandlePopIndex, kButtonIndex}};
    for (const std::size_t i : update_order) {
      if (joints_[i] &&
          (force || !has_applied_position_[i] ||
           std::abs(values[i] - applied_targets_[i]) > 1e-9)) {
        joints_[i]->SetPosition(0, values[i], false);
        applied_targets_[i] = values[i];
        has_applied_position_[i] = true;
      }
    }
  }

  // 根据当前关门阶段屏蔽不应由外部命令改变的关节。
  JointPositions GateExternalTargets(const JointPositions& requested) const {
    JointPositions gated = requested;
    switch (state_machine_.state()) {
      case DoorState::kClosingDoor:
        gated.button = 0.0;
        gated.handle_pop = kHandlePopped;
        gated.handle_turn = kHandleTurned;
        break;
      case DoorState::kDoorReturned:
        gated.button = 0.0;
        gated.handle_pop = kHandlePopped;
        gated.door_hinge = 0.0;
        break;
      case DoorState::kLatchedHandleOut:
        gated.button = 0.0;
        gated.handle_turn = 0.0;
        gated.door_hinge = 0.0;
        break;
      default:
        break;
    }
    return ClampToLimits(gated);
  }

  // 将成功阶段归一化到标准端点并开启端点保持。
  void NormalizeSuccessfulCheckpoint(DoorState state) {
    switch (state) {
      case DoorState::kDoorReturned:
        ApplyPositions(DoorReturnedPositions());
        break;
      case DoorState::kLatchedHandleOut:
        ApplyPositions(LatchedHandleOutPositions());
        break;
      case DoorState::kClosed:
        ApplyPositions(ClosedPositions());
        break;
      default:
        break;
    }
    BeginCanonicalHold(state);
  }

  // 启动标准阶段端点保持，等待 Gazebo 物理步真正应用目标姿态。
  void BeginCanonicalHold(DoorState state) {
    switch (state) {
      case DoorState::kClosed:
        canonical_hold_target_ = ClosedPositions();
        break;
      case DoorState::kOpen:
        canonical_hold_target_ = OpenPositions();
        break;
      case DoorState::kDoorReturned:
        canonical_hold_target_ = DoorReturnedPositions();
        break;
      case DoorState::kLatchedHandleOut:
        canonical_hold_target_ = LatchedHandleOutPositions();
        break;
      default:
        canonical_hold_active_ = false;
        return;
    }
    canonical_hold_active_ = true;
    // Gazebo 会在下一次物理步应用 SetPosition。持续确认目标端点直到该物理步
    // 可观测，避免把单帧插值残留误判为 UNKNOWN。
  }

  bool CanonicalHoldReached() const {
    const JointPositions actual = ReadPositions();
    return DoorCloseStateMachine::IsNear(actual.button,
                                          canonical_hold_target_.button) &&
           DoorCloseStateMachine::IsNear(actual.handle_pop,
                                          canonical_hold_target_.handle_pop) &&
           DoorCloseStateMachine::IsNear(actual.handle_turn,
                                          canonical_hold_target_.handle_turn) &&
           DoorCloseStateMachine::IsNear(actual.door_hinge,
                                          canonical_hold_target_.door_hinge);
  }

  // 读取当前仿真时间，供内部插值运动计算使用。
  double SimulationTime() const {
    if (!model_ || !model_->GetWorld()) {
      return 0.0;
    }
    return model_->GetWorld()->SimTime().Double();
  }

  // 根据状态机决策创建恢复或强制复位的插值运动段。
  void BeginInternalMotion(MotionRequest request,
                           const JointPositions& current,
                           const CommandMessage& command,
                           const StateDecision& decision) {
    motion_segments_.clear();
    motion_request_id_ = command.request_id;
    motion_command_ = command.command;
    motion_failed_checks_ = decision.failed_checks;

    if (request == MotionRequest::kRecoverOpen) {
      JointPositions handle_restored = current;
      handle_restored.button = 0.0;
      handle_restored.handle_pop = kHandlePopped;
      handle_restored.handle_turn = kHandleTurned;
      motion_segments_.push_back({current, handle_restored, 0.60});
      motion_segments_.push_back(
          {handle_restored, OpenPositions(), 1.20});
    } else if (request == MotionRequest::kRecoverHandleOut) {
      motion_segments_.push_back(
          {current, LatchedHandleOutPositions(), 0.60});
    } else if (request == MotionRequest::kForceClosed) {
      motion_segments_.push_back({current, ClosedPositions(), 0.90});
    }

    motion_segment_started_ = SimulationTime();
  }

  bool IsInternalMotionActive() const { return !motion_segments_.empty(); }

  // 推进恢复/复位插值段，完成后通知状态机结束内部动作。
  void AdvanceInternalMotion() {
    if (motion_segments_.empty()) {
      return;
    }

    MotionSegment& segment = motion_segments_.front();
    double now = SimulationTime();
    if (now < motion_segment_started_) {
      motion_segment_started_ = now;
    }
    const double elapsed = now - motion_segment_started_;
    const double alpha = std::max(
        0.0, std::min(1.0, elapsed / std::max(segment.duration, 1e-6)));

    const std::array<double, kJointCount> start = ToArray(segment.start);
    const std::array<double, kJointCount> target = ToArray(segment.target);
    std::array<double, kJointCount> values{};
    for (std::size_t i = 0; i < values.size(); ++i) {
      values[i] = start[i] + (target[i] - start[i]) * alpha;
    }
    ApplyPositions(FromArray(values));

    if (alpha < 1.0) {
      return;
    }

    ApplyPositions(segment.target);
    motion_segments_.pop_front();
    if (!motion_segments_.empty()) {
      motion_segment_started_ = now;
      return;
    }

    StateDecision completed = state_machine_.CompleteInternalMotion();
    completed.failed_checks = motion_failed_checks_;
    BeginCanonicalHold(completed.state);
    PublishStatus(motion_request_id_, motion_command_, completed);
    motion_request_id_ = 0;
    motion_command_ = 0;
    motion_failed_checks_ = kCheckNone;
  }

  // 发布一次关门协议状态及当前关节位置快照。
  void PublishStatus(std::uint32_t request_id, std::uint8_t command,
                     const StateDecision& decision,
                     const JointPositions* position_snapshot = nullptr) {
    StatusMessage message;
    message.header.stamp = cabinet_sim::stamp(model_->GetWorld()->SimTime().Double());
    message.request_id = request_id;
    message.state = static_cast<std::uint8_t>(decision.state);
    message.last_command = command;
    message.result = static_cast<std::uint8_t>(decision.result);
    message.failed_checks = decision.failed_checks;
    message.detail = decision.detail;

    const JointPositions actual =
        position_snapshot ? *position_snapshot : ReadPositions();
    message.door_hinge_position = actual.door_hinge;
    message.handle_turn_position = actual.handle_turn;
    message.handle_pop_position = actual.handle_pop;
    status_pub_.publish(message);
    initial_status_published_ = true;
  }

  // 发布由实际关节姿态识别出的被动状态变化。
  void PublishObservedState(const std::string& detail) {
    StateDecision decision;
    decision.state = state_machine_.state();
    decision.result = CommandResult::kNone;
    decision.detail = detail;
    PublishStatus(0, 0, decision);
  }

  // 调用状态机处理一条关门命令，并根据决策启动后续动作。
  void ProcessCloseCommand(const CommandMessage& command) {
    const JointPositions actual = ReadPositions();
    const StateDecision decision =
        state_machine_.HandleCommand(command.command, actual);

    if (decision.result == CommandResult::kSucceeded) {
      NormalizeSuccessfulCheckpoint(decision.state);
    }
    if (decision.motion != MotionRequest::kNone) {
      BeginInternalMotion(decision.motion, actual, command, decision);
    }
    // 即使成功检查点刚被归一化到精确的标准端点，也要报告本次决策采样到的
    // 实际关节位置。
    PublishStatus(command.request_id, command.command, decision, &actual);
  }

  // Gazebo 更新回调：处理最新目标、关门命令、内部运动和状态发布。
  void OnUpdate() {
    if (!initial_applied_) {
      if (initial_open_) {
        ApplyPositions(FromArray({{0.0, kHandlePopped, kHandleTurned, kDoorOpen}}), true);
      }
      initial_applied_ = true;
    }
    std::array<double, kJointCount> raw_targets{};
    std::uint64_t raw_sequence = 0;
    std::deque<CommandMessage> close_commands;
    {
      std::lock_guard<std::mutex> lock(input_mutex_);
      raw_targets = raw_targets_;
      raw_sequence = raw_sequence_;
      close_commands.swap(close_commands_);
    }

    // FORCE_CLOSED 可以抢占恢复过程。内部运动期间收到的其他命令仍由状态机
    // 按正常流程处理并拒绝。
    auto force = std::find_if(
        close_commands.begin(), close_commands.end(),
        [](const CommandMessage& command) {
          return command.command == CommandMessage::FORCE_CLOSED;
        });
    if (force != close_commands.end()) {
      motion_segments_.clear();
      ProcessCloseCommand(*force);
      close_commands.erase(force);
    }

    const bool has_new_raw_command = raw_sequence != consumed_raw_sequence_;
    if (IsInternalMotionActive()) {
      consumed_raw_sequence_ = raw_sequence;
      for (const CommandMessage& command : close_commands) {
        ProcessCloseCommand(command);
      }
      AdvanceInternalMotion();
      return;
    }

    // 在评估阶段提交之前先应用最新的底层位置样本。这样先发布位置、随后
    // 立即发布提交命令也是安全的。
    if (has_new_raw_command) {
      canonical_hold_active_ = false;
      ApplyPositions(GateExternalTargets(FromArray(raw_targets)));
      consumed_raw_sequence_ = raw_sequence;
    }

    if (canonical_hold_active_) {
      ApplyPositions(canonical_hold_target_, true);
      if (CanonicalHoldReached()) {
        canonical_hold_active_ = false;
      }
    }

    if (!canonical_hold_active_ && state_machine_.ObservePose(ReadPositions())) {
      PublishObservedState("已根据 Gazebo 实际关节位置更新门状态。");
    } else if (!initial_status_published_) {
      PublishObservedState("正在等待可识别的电柜姿态。");
    }

    for (const CommandMessage& command : close_commands) {
      ProcessCloseCommand(command);
    }
  }

  gazebo_ros::Node::SharedPtr node_;
  bool initial_open_ = true;
  bool initial_applied_ = false;
  gazebo::physics::ModelPtr model_;
  std::array<gazebo::physics::JointPtr, kJointCount> joints_{};
  std::array<double, kJointCount> lower_limits_{};
  std::array<double, kJointCount> upper_limits_{};
  std::array<double, kJointCount> raw_targets_{{0.0, 0.0, 0.0, 0.0}};
  std::array<double, kJointCount> applied_targets_{{0.0, 0.0, 0.0, 0.0}};
  std::array<bool, kJointCount> has_applied_position_{{false, false, false,
                                                       false}};
  std::uint64_t raw_sequence_{0};
  std::uint64_t consumed_raw_sequence_{0};
  std::deque<CommandMessage> close_commands_;
  std::mutex input_mutex_;

  DoorCloseStateMachine state_machine_;
  std::deque<MotionSegment> motion_segments_;
  double motion_segment_started_{0.0};
  std::uint32_t motion_request_id_{0};
  std::uint8_t motion_command_{0};
  std::uint8_t motion_failed_checks_{kCheckNone};
  bool all_joints_available_{false};
  bool initial_status_published_{false};
  bool canonical_hold_active_{false};
  JointPositions canonical_hold_target_;

  gazebo::event::ConnectionPtr update_connection_;
  rclcpp::SubscriptionBase::SharedPtr command_sub_;
  rclcpp::SubscriptionBase::SharedPtr close_command_sub_;
  cabinet_sim::Publisher status_pub_;
};

GZ_REGISTER_MODEL_PLUGIN(ElectricalCabinetKinematicPlugin)

}  // 电气柜命名空间结束
