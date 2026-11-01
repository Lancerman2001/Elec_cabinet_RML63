#include <cabinet_sim_support/transport.hpp>
// Simplified screw kinematics and an interlock, not a thread-contact simulator.
#include <algorithm>
#include <cmath>
#include <functional>
#include <memory>
#include <string>

#include <gazebo/common/Events.hh>
#include <gazebo/common/Plugin.hh>
#include <gazebo/common/UpdateInfo.hh>
#include <gazebo/physics/physics.hh>
#include <sensor_msgs/msg/joint_state.hpp>
#include <std_msgs/msg/bool.hpp>
#include <std_msgs/msg/empty.hpp>

namespace gazebo
{
class YabanInteractionPlugin : public ModelPlugin
{
 public:
  ~YabanInteractionPlugin() override
  {
    update_connection_.reset();
    command_subscriber_.reset();
    cancel_subscriber_.reset();
    
    
    
  }

  void Load(physics::ModelPtr model, sdf::ElementPtr sdf) override
  {
    model_ = model;
    prefix_ = Parameter<std::string>(sdf, "jointPrefix", "yaban_");
    const auto robot_namespace =
        Parameter<std::string>(sdf, "robotNamespace", "/yaban");
    demo_control_ = Parameter<bool>(sdf, "demoControl", false);
    screw_pitch_ = Parameter<double>(sdf, "screwPitch", 0.0007);
    release_angle_ = Parameter<double>(sdf, "releaseTurns", 0.5) * kTwoPi;
    knob_speed_ = Parameter<double>(sdf, "knobSpeed", kTwoPi / 2.0);
    blade_speed_ = Parameter<double>(sdf, "bladeSpeed", kTwoPi / 8.0);

    knob_ = model_->GetJoint(prefix_ + "left_knob_joint");
    lift_ = model_->GetJoint(prefix_ + "left_knob_lift_joint");
    blade_ = model_->GetJoint(prefix_ + "blade_joint");
    if (!knob_ || !lift_ || !blade_)
    {
      gzerr << "YabanInteractionPlugin: missing joints with prefix '"
            << prefix_ << "'.\n";
      return;
    }
    // During runtime model insertion, ODE's cached limits may not be initialized
    // yet. Read the declared limits from SDF, including the multi-turn angle.
    knob_limit_ = DeclaredUpperLimit(knob_);
    blade_limit_ = DeclaredUpperLimit(blade_);
    const double lift_limit = DeclaredUpperLimit(lift_);
    if (!PositiveFinite(screw_pitch_) || !PositiveFinite(release_angle_) ||
        !PositiveFinite(knob_speed_) || !PositiveFinite(blade_speed_) ||
        !PositiveFinite(knob_limit_) || !PositiveFinite(blade_limit_) ||
        release_angle_ > knob_limit_ ||
        !PositiveFinite(lift_limit) ||
        // URDF-to-SDF conversion can round the multi-turn angle to 6 digits.
        screw_pitch_ * knob_limit_ / kTwoPi > lift_limit + 1e-7)
    {
      gzerr << "YabanInteractionPlugin: invalid parameters; plugin disabled. "
            << "pitch=" << screw_pitch_ << ", release=" << release_angle_
            << ", knobSpeed=" << knob_speed_ << ", bladeSpeed=" << blade_speed_
            << ", knobLimit=" << knob_limit_ << ", bladeLimit=" << blade_limit_
            << ", liftLimit=" << lift_limit << "\n";
      return;
    }

    if (!rclcpp::ok()) { gzerr << "Initialize ROS 2 with gazebo_ros before loading plugins\n"; return; }
    node_ = cabinet_sim::node(sdf, robot_namespace);
    // The Gazebo update thread owns both commands and physics mutations.
    
    command_subscriber_ = cabinet_sim::subscribe(node_, 
        "command", 1, &YabanInteractionPlugin::Command, this);
    cancel_subscriber_ = cabinet_sim::subscribe(node_, 
        "cancel", 1, &YabanInteractionPlugin::Cancel, this);
    state_publisher_ = cabinet_sim::advertise<sensor_msgs::msg::JointState>(node_, "joint_states", 1);
    engaged_publisher_ = cabinet_sim::advertise<std_msgs::msg::Bool>(node_, "engaged", 1, true);
    mode_publisher_ = cabinet_sim::advertise<std_msgs::msg::Bool>(node_, "demo_control_enabled", 1, true);
    std_msgs::msg::Bool mode;
    mode.data = demo_control_;
    mode_publisher_.publish(mode);
    update_connection_ = event::Events::ConnectWorldUpdateBegin(
        std::bind(&YabanInteractionPlugin::Update, this, std::placeholders::_1));
    CAB_INFO_STREAM_NAMED("yaban", "Yaban plugin ready in " << robot_namespace
                          << "; demoControl=" << (demo_control_ ? "true" : "false"));
  }

  void Reset() override
  {
    std::lock_guard<std::recursive_mutex> guard(mutex_);
    
    initialized_ = false;
    last_publish_time_ = -1.0;
  }

 private:
  template <typename T>
  static T Parameter(const sdf::ElementPtr &sdf, const std::string &name,
                     const T &fallback)
  {
    return sdf->HasElement(name) ? sdf->Get<T>(name) : fallback;
  }

  static bool PositiveFinite(double value)
  {
    return std::isfinite(value) && value > 0.0;
  }

  static double DeclaredUpperLimit(const physics::JointPtr &joint)
  {
    const auto joint_sdf = joint->GetSDF();
    if (joint_sdf && joint_sdf->HasElement("axis"))
    {
      const auto axis = joint_sdf->GetElement("axis");
      if (axis->HasElement("limit"))
        return axis->GetElement("limit")->Get<double>("upper");
    }
    return joint->UpperLimit(0);
  }

  static double Clamp(double value, double upper)
  {
    return std::max(0.0, std::min(value, upper));
  }

  static double Advance(double current, double target, double maximum_step)
  {
    return current + std::max(-maximum_step, std::min(target - current, maximum_step));
  }

  static void SetPosition(const physics::JointPtr &joint, double position)
  {
    joint->SetPosition(0, position, true);
    joint->SetVelocity(0, 0.0);
  }

  void Cancel(const std_msgs::msg::Empty::ConstSharedPtr &)
  {
    std::lock_guard<std::recursive_mutex> guard(mutex_);
    // A cancelled / timed-out command must not restart when Gazebo is unpaused.
    knob_target_ = Clamp(knob_->Position(0), knob_limit_);
    blade_target_ = Clamp(blade_->Position(0), blade_limit_);
  }

  void Command(const sensor_msgs::msg::JointState::ConstSharedPtr &message)
  {
    std::lock_guard<std::recursive_mutex> guard(mutex_);
    if (!demo_control_)
    {
      CAB_WARN_THROTTLE_NAMED(5.0, "yaban",
          "Ignoring command: launch with demo_control:=true for scripted motion.");
      return;
    }
    if (message->name.empty() || message->name.size() != message->position.size())
    {
      CAB_WARN_NAMED("yaban", "Rejected command: names and positions must have equal nonzero lengths.");
      return;
    }
    double knob_target = knob_target_;
    double blade_target = blade_target_;
    bool saw_knob = false;
    bool saw_blade = false;
    for (std::size_t i = 0; i < message->name.size(); ++i)
    {
      if (!std::isfinite(message->position[i]))
      {
        CAB_WARN_NAMED("yaban", "Rejected command: nonfinite joint position.");
        return;
      }
      if (message->name[i] == knob_->GetName() && !saw_knob)
      {
        if (message->position[i] < -1e-5 || message->position[i] > knob_limit_ + 1e-5)
        {
          CAB_WARN_NAMED("yaban", "Rejected knob target outside joint limits.");
          return;
        }
        knob_target = Clamp(message->position[i], knob_limit_);
        saw_knob = true;
      }
      else if (message->name[i] == blade_->GetName() && !saw_blade)
      {
        if (message->position[i] < -1e-5 || message->position[i] > blade_limit_ + 1e-5)
        {
          CAB_WARN_NAMED("yaban", "Rejected blade target outside 0..120 degree limits.");
          return;
        }
        blade_target = Clamp(message->position[i], blade_limit_);
        saw_blade = true;
      }
      else
      {
        CAB_WARN_NAMED("yaban", "Rejected command: unknown, duplicate or dependent joint '%s'.",
                       message->name[i].c_str());
        return;
      }
    }
    const double knob_position = knob_->Position(0);
    const double blade_position = blade_->Position(0);
    const bool blade_moves = std::abs(blade_target - blade_position) > kPositionTolerance;
    const bool knob_moves = std::abs(knob_target - knob_position) > kPositionTolerance;
    // Reject immediately instead of caching an unsafe target for later execution.
    // Every stage must be reached in measured joint feedback before the next stage.
    if ((blade_moves && knob_position < release_angle_ - kPositionTolerance) ||
        (knob_target < knob_position - kPositionTolerance &&
         blade_position > kClosedTolerance) ||
        (blade_moves && knob_moves))
    {
      CAB_WARN_THROTTLE_NAMED(1.0, "yaban",
          "Rejected out-of-sequence command: loosen >=180 deg, move blade, close blade, tighten.");
      return;
    }
    knob_target_ = knob_target;
    blade_target_ = blade_target;
  }

  void Update(const common::UpdateInfo &info)
  {
    std::lock_guard<std::recursive_mutex> guard(mutex_);
    const double now = info.simTime.Double();
    double knob_position = knob_->Position(0);
    double blade_position = blade_->Position(0);
    if (!std::isfinite(knob_position) || !std::isfinite(blade_position))
    {
      CAB_ERROR_THROTTLE_NAMED(2.0, "yaban", "Nonfinite physics state; resetting moving joints.");
      knob_position = 0.0;
      blade_position = 0.0;
      SetPosition(knob_, 0.0);
      SetPosition(blade_, 0.0);
      initialized_ = false;
    }
    if (!initialized_ || now < last_update_time_)
    {
      knob_target_ = Clamp(knob_position, knob_limit_);
      blade_target_ = Clamp(blade_position, blade_limit_);
      previous_knob_ = knob_position;
      previous_blade_ = blade_position;
      last_update_time_ = now;
      last_publish_time_ = -1.0;
      initialized_ = true;
    }
    
    const double dt = std::max(0.0, now - last_update_time_);
    const double control_dt = std::min(dt, 0.02);
    double next_knob = Clamp(knob_position, knob_limit_);
    double next_blade = Clamp(blade_position, blade_limit_);
    if (demo_control_)
    {
      next_knob = Advance(next_knob, knob_target_, knob_speed_ * control_dt);
      next_blade = Advance(next_blade, blade_target_, blade_speed_ * control_dt);
    }

    // An already open blade blocks tightening; a closed, clamped blade blocks opening.
    if (next_knob < release_angle_ - kPositionTolerance)
    {
      if (previous_blade_ > kClosedTolerance)
        next_knob = release_angle_;
      else
        next_blade = 0.0;
    }
    if (demo_control_ || std::abs(next_knob - knob_position) > 1e-12)
      SetPosition(knob_, next_knob);
    if (demo_control_ || std::abs(next_blade - blade_position) > 1e-12)
      SetPosition(blade_, next_blade);

    // Gazebo Classic does not enforce URDF mimic; impose the M4 pitch explicitly.
    const double actual_knob = knob_->Position(0);
    const double actual_blade = blade_->Position(0);
    const double desired_lift = screw_pitch_ * Clamp(actual_knob, knob_limit_) / kTwoPi;
    // Avoid needless slider pose updates while stationary (ODE's slider Anchor
    // helper also logs a warning on every SetPosition call in Gazebo Classic).
    if (std::abs(desired_lift - lift_->Position(0)) > 1e-9)
      SetPosition(lift_, desired_lift);
    else
      lift_->SetVelocity(0, 0.0);
    if (last_publish_time_ < 0.0 || now - last_publish_time_ >= 0.02 - 1e-9)
    {
      sensor_msgs::msg::JointState state;
      state.header.stamp = cabinet_sim::stamp(std::max(0.0, now));
      state.name = {knob_->GetName(), lift_->GetName(), blade_->GetName()};
      state.position = {actual_knob, lift_->Position(0), actual_blade};
      const double knob_velocity = dt > 0.0 ? (actual_knob - previous_knob_) / dt : 0.0;
      const double blade_velocity = dt > 0.0 ? (actual_blade - previous_blade_) / dt : 0.0;
      state.velocity = {knob_velocity, knob_velocity * screw_pitch_ / kTwoPi, blade_velocity};
      state_publisher_.publish(state);
      std_msgs::msg::Bool engaged;
      engaged.data = std::abs(actual_blade) <= kClosedTolerance &&
                     std::abs(actual_knob) <= kTightTolerance;
      engaged_publisher_.publish(engaged);
      last_publish_time_ = now;
    }
    previous_knob_ = actual_knob;
    previous_blade_ = actual_blade;
    last_update_time_ = now;
  }

  static constexpr double kTwoPi = 6.2831853071795864769;
  static constexpr double kClosedTolerance = 0.0017453292519943296;  // 0.1 degree
  static constexpr double kTightTolerance = 0.0017453292519943296;
  static constexpr double kPositionTolerance = 0.001;  // Includes SDF rounding / solver noise.
  physics::ModelPtr model_;
  physics::JointPtr knob_, lift_, blade_;
  event::ConnectionPtr update_connection_;
  std::recursive_mutex mutex_;
  gazebo_ros::Node::SharedPtr node_;
  
  rclcpp::SubscriptionBase::SharedPtr command_subscriber_, cancel_subscriber_;
  cabinet_sim::Publisher state_publisher_, engaged_publisher_, mode_publisher_;
  std::string prefix_;
  bool demo_control_ = false;
  bool initialized_ = false;
  double screw_pitch_ = 0.0007;
  double release_angle_ = kTwoPi / 2.0;
  double knob_speed_ = kTwoPi / 2.0;
  double blade_speed_ = kTwoPi / 8.0;
  double knob_limit_ = 2.0 * kTwoPi;
  double blade_limit_ = kTwoPi / 3.0;
  double knob_target_ = 0.0, blade_target_ = 0.0;
  double previous_knob_ = 0.0, previous_blade_ = 0.0;
  double last_update_time_ = 0.0, last_publish_time_ = -1.0;
};

GZ_REGISTER_MODEL_PLUGIN(YabanInteractionPlugin)
}  // namespace gazebo
