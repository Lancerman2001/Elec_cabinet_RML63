#include <cabinet_sim_support/transport.hpp>
#include <algorithm>
#include <cmath>
#include <functional>
#include <memory>
#include <mutex>
#include <string>
#include <gazebo/common/Plugin.hh>
#include <gazebo/common/Events.hh>
#include <gazebo/physics/physics.hh>
#include <gazebo/transport/transport.hh>
#include <gazebo/msgs/msgs.hh>
#include <sensor_msgs/msg/joint_state.hpp>
#include <std_msgs/msg/bool.hpp>
#include <std_msgs/msg/string.hpp>
#include <std_srvs/srv/set_bool.hpp>
#include <std_srvs/srv/trigger.hpp>

namespace gazebo {
class BreakerInteractionPlugin : public ModelPlugin {
 public:
  ~BreakerInteractionPlugin() override {
    connection_.reset();
    
    
     
  }

  void Load(physics::ModelPtr model, sdf::ElementPtr sdf) override {
    model_ = model; world_ = model_->GetWorld();
    prefix_ = Param<std::string>(sdf, "jointPrefix", "breaker_");
    const auto ns = Param<std::string>(sdf, "robotNamespace", "/breaker");
    demo_ = Param<bool>(sdf, "demoControl", true);
    initial_demo_ = demo_;
    half_ = Param<double>(sdf, "halfAngle", 0.698131700798);
    hysteresis_ = Param<double>(sdf, "hysteresis", 0.174532925199);
    peak_ = Param<double>(sdf, "peakTorque", 0.015);
    damping_ = Param<double>(sdf, "damping", 0.0008);
    inertia_ = Param<double>(sdf, "pivotInertia", 1e-6);
    kp_ = Param<double>(sdf, "demoKp", 0.08);
    kd_ = Param<double>(sdf, "demoKd", 0.0008);
    demo_limit_ = Param<double>(sdf, "demoTorqueLimit", 0.04);
    effort_limit_ = Param<double>(sdf, "effortLimit", 0.06);
    target_speed_ = Param<double>(sdf, "targetSpeed", 1.5);
    timeout_ = Param<double>(sdf, "actionTimeout", 5.0);
    joint_ = model_->GetJoint(prefix_ + "toggle_joint");
    indicator_ = model_->GetLink(prefix_ + "indicator_link");
    for (double x : {half_, hysteresis_, peak_, damping_, inertia_, kp_, kd_,
                     demo_limit_, effort_limit_, target_speed_, timeout_}) {
      if (!std::isfinite(x) || x <= 0) {
        gzerr << "Breaker: invalid non-positive/non-finite parameter; plugin disabled.\n";
        return;
      }
    }
    if (!joint_ || !indicator_ || hysteresis_ >= half_) {
      gzerr << "Breaker: missing toggle/indicator or invalid hysteresis; plugin disabled.\n";
      return;
    }
    const auto limits = joint_->GetSDF()->GetElement("axis")->GetElement("limit");
    if (std::abs(limits->Get<double>("lower") + half_) > 1e-5 ||
        std::abs(limits->Get<double>("upper") - half_) > 1e-5) {
      gzerr << "Breaker: URDF limits and plugin halfAngle disagree.\n"; return;
    }
    if (!rclcpp::ok()) { gzerr << "Initialize ROS 2 with gazebo_ros before loading plugins\n"; return; }
    node_ = cabinet_sim::node(sdf, ns); 
    states_ = cabinet_sim::advertise<sensor_msgs::msg::JointState>(node_, "joint_states", 1);
    power_pub_ = cabinet_sim::advertise<std_msgs::msg::Bool>(node_, "power_state", 1, true);
    mode_pub_ = cabinet_sim::advertise<std_msgs::msg::Bool>(node_, "demo_control_enabled", 1, true);
    busy_pub_ = cabinet_sim::advertise<std_msgs::msg::Bool>(node_, "busy", 1, true);
    status_pub_ = cabinet_sim::advertise<std_msgs::msg::String>(node_, "status", 1, true);
    power_srv_ = cabinet_sim::service<std_srvs::srv::SetBool>(node_, "set_power", &BreakerInteractionPlugin::SetPower, this);
    mode_srv_ = cabinet_sim::service<std_srvs::srv::SetBool>(node_, "set_demo_mode", &BreakerInteractionPlugin::SetMode, this);
    reset_srv_ = cabinet_sim::service<std_srvs::srv::Trigger>(node_, "reset", &BreakerInteractionPlugin::ResetRequest, this);
    cancel_srv_ = cabinet_sim::service<std_srvs::srv::Trigger>(node_, "cancel", &BreakerInteractionPlugin::Cancel, this);
    transport_.reset(new transport::Node()); transport_->Init(world_->Name());
    visual_pub_ = transport_->Advertise<msgs::Visual>("~/visual");
    
    connection_ = event::Events::ConnectWorldUpdateBegin(
        std::bind(&BreakerInteractionPlugin::Update, this, std::placeholders::_1));
    CAB_INFO_STREAM("Circuit breaker ready: " << ns << "; up=ON, down=OFF; torque driven.");
  }

  void Reset() override {
    std::lock_guard<std::mutex> lock(mutex_);
    initialized_ = false; active_ = false; reset_pending_ = false;
    demo_ = initial_demo_; fault_.clear(); last_publish_ = -1; last_visual_ = -1;
  }

 private:
  template <class T> static T Param(const sdf::ElementPtr &sdf, const std::string &key, const T &fallback) {
    return sdf->HasElement(key) ? sdf->Get<T>(key) : fallback;
  }
  static double Clamp(double x, double lo, double hi) { return std::max(lo, std::min(x, hi)); }
  bool Ready(std::string &message) const {
    if (!initialized_) { message = "Model is initializing"; return false; }
    if (world_->IsPaused() || (cabinet_sim::SteadyTime::now() - last_update_wall_).toSec() > 0.3) {
      message = "Simulation is paused or feedback is stale"; return false;
    }
    if (!fault_.empty() && fault_ != "TIMEOUT") { message = fault_; return false; }
    return true;
  }
  bool SetPower(std_srvs::srv::SetBool::Request &request, std_srvs::srv::SetBool::Response &response) {
    std::lock_guard<std::mutex> lock(mutex_);
    if (!Ready(response.message)) return true;
    if (!demo_) { response.message = "Passive contact mode: demonstration commands disabled"; return true; }
    if (active_ || reset_pending_) { response.message = "An action is already running"; return true; }
    goal_ = request.data ? -half_ : half_; reference_ = position_;
    active_ = true; fault_.clear(); settled_ = 0;
    // Motion advances in simulation time, which can be much slower than wall
    // time with llvmpipe. Keep the wall-clock watchdog only in Ready().
    action_start_ = world_->SimTime().Double();
    response.success = true; response.message = "Accepted; completion is reported by actual joint feedback";
    return true;
  }
  bool SetMode(std_srvs::srv::SetBool::Request &request, std_srvs::srv::SetBool::Response &response) {
    std::lock_guard<std::mutex> lock(mutex_);
    // No physics mutations in ROS callbacks, including when Gazebo is paused.
    demo_ = request.data; active_ = false; reset_pending_ = false; fault_.clear();
    std_msgs::msg::Bool message; message.data = demo_; mode_pub_.publish(message);
    response.success = true; response.message = demo_ ? "Demonstration enabled; no pending target" : "Passive contact enabled; previous target cancelled";
    return true;
  }
  bool ResetRequest(std_srvs::srv::Trigger::Request &, std_srvs::srv::Trigger::Response &response) {
    std::lock_guard<std::mutex> lock(mutex_);
    if (!Ready(response.message)) return true;
    if (!demo_) { response.message = "Reset disabled in passive contact mode"; return true; }
    reset_pending_ = true; active_ = false; fault_.clear();
    response.success = true; response.message = "Reset to OFF queued"; return true;
  }
  bool Cancel(std_srvs::srv::Trigger::Request &, std_srvs::srv::Trigger::Response &response) {
    std::lock_guard<std::mutex> lock(mutex_);
    active_ = false; reset_pending_ = false;
    response.success = true; response.message = "Demonstration target cancelled; passive detent remains"; return true;
  }
  void ResetPose() {
    joint_->SetPosition(0, half_, true); joint_->SetVelocity(0, 0);
    position_ = half_; velocity_ = 0; powered_ = false; active_ = false;
    reset_pending_ = false; initialized_ = true; settled_ = 0; fault_.clear();
  }
  void Update(const common::UpdateInfo &info) {
    std::lock_guard<std::mutex> lock(mutex_);
    const double now = info.simTime.Double();
    const double dt = Clamp(now-last_sim_, 0.00001, 0.01); last_sim_ = now;
    last_update_wall_ = cabinet_sim::SteadyTime::now();
    if (!initialized_ || reset_pending_) ResetPose();
    position_ = joint_->Position(0); velocity_ = joint_->GetVelocity(0);
    if (!std::isfinite(position_) || !std::isfinite(velocity_)) {
      active_ = false; fault_ = "NONFINITE_PHYSICS"; Publish(now); return;
    }
    if (position_ < -hysteresis_) powered_ = true;
    else if (position_ > hysteresis_) powered_ = false;
    double demo_torque = 0, b = damping_;
    if (active_ && demo_) {
      if (now-action_start_ > timeout_) {
        active_ = false; fault_ = "TIMEOUT";
      } else {
        reference_ += Clamp(goal_-reference_, -target_speed_*dt, target_speed_*dt);
        demo_torque = Clamp(kp_*(reference_-position_), -demo_limit_, demo_limit_);
        b += kd_;
        if (std::abs(position_-goal_) < 0.0174533 && std::abs(velocity_) < 0.08)
          settled_ += dt;
        else settled_ = 0;
        if (settled_ >= 0.08) { active_ = false; demo_torque = 0; b = damping_; }
      }
    }
    // Cubic double-well torque: stable +/-halfAngle, unstable zero; maximum = peakTorque.
    const double u = Clamp(position_/half_, -1.1, 1.1);
    const double detent = peak_ * 2.598076211353316 * u * (1-u*u);
    // Regularize only the damping coefficient for millimetre-scale inertias.
    // Scaling the spring torque here would incorrectly reduce the static detent
    // barrier against an external contact force (peakTorque must stay physical).
    const double effective_damping = b / (1+b*dt/inertia_);
    applied_ = Clamp(detent + demo_torque - effective_damping*velocity_, -effort_limit_, effort_limit_);
    joint_->SetForce(0, applied_);
    if (now-last_publish_ >= 0.0199 || now < last_publish_) { Publish(now); last_publish_ = now; }
    if (now-last_visual_ >= 0.25 || now < last_visual_) { UpdateIndicator(); last_visual_ = now; }
  }
  void Publish(double now) {
    sensor_msgs::msg::JointState state; state.header.stamp = cabinet_sim::stamp(now);
    state.name = {prefix_+"toggle_joint"}; state.position = {position_}; state.velocity = {velocity_}; state.effort = {applied_}; states_.publish(state);
    std_msgs::msg::Bool boolean; boolean.data = powered_; power_pub_.publish(boolean);
    boolean.data = demo_; mode_pub_.publish(boolean); boolean.data = active_; busy_pub_.publish(boolean);
    std_msgs::msg::String status;
    if (!fault_.empty()) status.data = fault_;
    else if (active_) status.data = "MOVING";
    else if (std::abs(velocity_) > 0.08 || std::abs(std::abs(position_)-half_) > 0.0349066) status.data = "SETTLING";
    else status.data = powered_ ? "ON" : "OFF";
    status_pub_.publish(status);
  }
  void UpdateIndicator() {
    const ignition::math::Color color = powered_ ? ignition::math::Color(0.85,0.055,0.025,1) : ignition::math::Color(0.015,0.32,0.18,1);
    for (const auto &entry : indicator_->Visuals()) {
      msgs::Visual visual = entry.second;
      msgs::Set(visual.mutable_material()->mutable_ambient(), color);
      msgs::Set(visual.mutable_material()->mutable_diffuse(), color);
      visual_pub_->Publish(visual);
    }
  }
  physics::ModelPtr model_; physics::WorldPtr world_; physics::JointPtr joint_; physics::LinkPtr indicator_;
  event::ConnectionPtr connection_; transport::NodePtr transport_; transport::PublisherPtr visual_pub_;
  gazebo_ros::Node::SharedPtr node_;  
  cabinet_sim::Publisher states_, power_pub_, mode_pub_, busy_pub_, status_pub_;
  rclcpp::ServiceBase::SharedPtr power_srv_, mode_srv_, reset_srv_, cancel_srv_;
  std::mutex mutex_; std::string prefix_, fault_;
  bool demo_ = true, initial_demo_ = true, initialized_ = false, reset_pending_ = false, active_ = false, powered_ = false;
  double half_, hysteresis_, peak_, damping_, inertia_, kp_, kd_, demo_limit_, effort_limit_, target_speed_, timeout_;
  double position_ = 0, velocity_ = 0, applied_ = 0, goal_ = 0, reference_ = 0, settled_ = 0;
  double last_publish_ = -1, last_visual_ = -1, last_sim_ = 0;
  double action_start_ = 0;
  cabinet_sim::SteadyTime last_update_wall_;
};
GZ_REGISTER_MODEL_PLUGIN(BreakerInteractionPlugin)
}  // namespace gazebo
