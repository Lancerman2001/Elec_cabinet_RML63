#include <cabinet_sim_support/transport.hpp>
#include <algorithm>
#include <cmath>
#include <functional>
#include <iomanip>
#include <memory>
#include <mutex>
#include <sstream>
#include <vector>
#include <gazebo/common/Plugin.hh>
#include <gazebo/common/Events.hh>
#include <gazebo/physics/physics.hh>
#include <gazebo/transport/transport.hh>
#include <gazebo/msgs/msgs.hh>
#include <sensor_msgs/msg/joint_state.hpp>
#include <std_msgs/msg/bool.hpp>
#include <std_msgs/msg/empty.hpp>
#include <std_msgs/msg/string.hpp>
#include <std_msgs/msg/u_int32.hpp>
#include <std_srvs/srv/trigger.hpp>
#include <fugui_gazebo/srv/press.hpp>

namespace gazebo {
class FuguiButtonPlugin : public ModelPlugin {
 public:
  ~FuguiButtonPlugin() override {
    update_.reset();
    
    
     
  }

  void Load(physics::ModelPtr model, sdf::ElementPtr sdf) override {
    model_ = model; world_ = model_->GetWorld();
    prefix_ = sdf->HasElement("prefix") ? sdf->Get<std::string>("prefix") : "fugui_";
    const std::string ns = sdf->HasElement("robotNamespace") ? sdf->Get<std::string>("robotNamespace") : "/fugui";
    joint_ = model_->GetJoint(prefix_ + "press_joint");
    cap_ = model_->GetLink(prefix_ + "button_link");
    if (!joint_ || !cap_) { gzerr << "Fugui: missing press joint or button link\n"; return; }
    max_depth_ = joint_->UpperLimit(0);
    for (const auto &entry : std::vector<std::pair<std::string, double*>>{
         {"valid_min", &valid_min_}, {"valid_max", &valid_max_},
         {"rearm_depth", &rearm_depth_}, {"lamp_duration", &lamp_duration_},
         {"numerical_tolerance", &tolerance_}, {"stiffness", &stiffness_}, {"damping", &damping_}}) {
      if (!sdf->HasElement(entry.first)) { gzerr << "Fugui: missing " << entry.first << "; rebuild models\n"; return; }
      *entry.second = sdf->Get<double>(entry.first);
      if (!std::isfinite(*entry.second) || *entry.second < 0) { gzerr << "Fugui: invalid parameter\n"; return; }
    }
    if (std::abs(joint_->LowerLimit(0)) > 1e-9 || !std::isfinite(max_depth_) ||
        !(rearm_depth_ < valid_min_ && valid_min_ <= valid_max_ && valid_max_ <= max_depth_)) {
      gzerr << "Fugui: invalid joint/trigger limits\n"; return;
    }
    if (!rclcpp::ok()) { gzerr << "Initialize ROS 2 with gazebo_ros before loading plugins\n"; return; }
    node_ = cabinet_sim::node(sdf, ns); 
    joint_pub_ = cabinet_sim::advertise<sensor_msgs::msg::JointState>(node_, "joint_states", 1);
    led_pub_ = cabinet_sim::advertise<std_msgs::msg::Bool>(node_, "led_on", 1, true);
    count_pub_ = cabinet_sim::advertise<std_msgs::msg::UInt32>(node_, "reset_count", 1, true);
    status_pub_ = cabinet_sim::advertise<std_msgs::msg::String>(node_, "status", 1, true);
    event_pub_ = cabinet_sim::advertise<std_msgs::msg::Empty>(node_, "reset_event", 10);
    press_srv_ = cabinet_sim::service<fugui_gazebo::srv::Press>(node_, "press", &FuguiButtonPlugin::Press, this);
    release_srv_ = cabinet_sim::service<std_srvs::srv::Trigger>(node_, "release", &FuguiButtonPlugin::Release, this);
    transport_.reset(new transport::Node()); transport_->Init(world_->Name());
    visual_pub_ = transport_->Advertise<msgs::Visual>("~/visual");
    
    update_ = event::Events::ConnectWorldUpdateBegin(std::bind(&FuguiButtonPlugin::Update, this, std::placeholders::_1));
    CAB_INFO_STREAM("Fugui ready at " << ns << ": " << valid_min_*1000 << ".." << valid_max_*1000 << " mm valid, spring return.");
  }

  void Reset() override {
    std::lock_guard<std::mutex> lock(mutex_);
    initialized_ = false; active_ = false; armed_ = true; led_ = false; integral_force_ = 0;
    count_ = 0; led_until_ = -1; hold_start_ = -1; fault_.clear();
    last_publish_ = -1; last_visual_ = -1;
  }

 private:
  static double Clamp(double x, double lo, double hi) { return std::max(lo, std::min(x, hi)); }
  bool Press(fugui_gazebo::srv::Press::Request &req, fugui_gazebo::srv::Press::Response &res) {
    std::lock_guard<std::mutex> lock(mutex_);
    if (!std::isfinite(req.depth_mm) || req.depth_mm < 0 || req.depth_mm > max_depth_*1000 ||
        !std::isfinite(req.hold_seconds) || req.hold_seconds < 0 || req.hold_seconds > 10) {
      res.message = "Depth must be finite in [0," + std::to_string(max_depth_*1000) + "] mm; hold time must be finite in [0,10] s"; return true;
    }
    if (!initialized_ || world_->IsPaused() || (cabinet_sim::SteadyTime::now()-last_wall_).toSec() > .4) {
      res.message = "Simulation is initializing, paused or disconnected"; return true;
    }
    if (active_ || position_ > rearm_depth_ || !fault_.empty()) {
      res.message = "Release the button and wait for spring return before the next press"; return true;
    }
    target_ = req.depth_mm*.001; reference_ = position_;
    integral_force_ = 0;
    auto_release_ = req.auto_release; hold_seconds_ = req.hold_seconds;
    hold_start_ = -1; action_start_ = last_sim_; active_ = true;
    res.success = true; res.message = "Accepted; actual joint travel determines reset validity";
    return true;
  }
  bool Release(std_srvs::srv::Trigger::Request &, std_srvs::srv::Trigger::Response &res) {
    // Safe while paused; the next physics update applies the return spring.
    std::lock_guard<std::mutex> lock(mutex_);
    active_ = false; hold_start_ = -1; integral_force_ = 0;
    res.success = true; res.message = "Released; return spring enabled"; return true;
  }
  void Update(const common::UpdateInfo &info) {
    std::lock_guard<std::mutex> lock(mutex_);
    const double now = info.simTime.Double();
    const double dt = Clamp(now-last_sim_, 0.00001, .01);
    last_sim_ = now; last_wall_ = cabinet_sim::SteadyTime::now();
    if (!initialized_) {
      joint_->SetPosition(0, 0, true); joint_->SetVelocity(0, 0);
      initialized_ = true;
    }
    position_ = joint_->Position(0); velocity_ = joint_->GetVelocity(0);
    if (!std::isfinite(position_) || !std::isfinite(velocity_)) {
      fault_ = "NONFINITE_PHYSICS"; active_ = false; led_ = false; applied_ = 0;
      joint_->SetForce(0, 0); Publish(now); UpdateVisual(); return;
    }
    const bool old_led = led_;
    // 1 micrometre numerical tolerance at the two inclusive limits.
    valid_ = position_ >= valid_min_-tolerance_ && position_ <= valid_max_+tolerance_;
    if (position_ <= rearm_depth_) armed_ = true;
    bool event = false;
    if (valid_ && armed_) {
      armed_ = false; led_until_ = now+lamp_duration_; ++count_; event = true;
      event_pub_.publish(std_msgs::msg::Empty());
    }
    led_ = now < led_until_;
    if (active_) {
      reference_ += Clamp(target_-reference_, -.025*dt, .025*dt);
      if (hold_start_ < 0 && std::abs(position_-target_) <= .00002 && std::abs(velocity_) < .002)
        hold_start_ = now;
      if (hold_start_ < 0 && now-action_start_ > 3.0) {
        active_ = false; fault_ = "TARGET_TIMEOUT";
      }
      if (auto_release_ && hold_start_ >= 0 && now-hold_start_ >= hold_seconds_) active_ = false;
    }
    if (!active_ && position_ <= rearm_depth_) fault_.clear();
    // Physical force control only. No teleporting during normal operation.
    // Mass=10 g, demonstration stiffness=400 N/m, return spring=120 N/m.
    // A bounded integral force compensates the static ODE slider/contact
    // load. Pure PD had a ~0.26 mm steady error on the upper button, which
    // could make an exact 3 mm request miss the inclusive trigger threshold.
    // This is actuator control, not a change to the return spring or a pose
    // assignment; a blocked target still times out and the force is bounded.
    if (!active_) integral_force_ = 0;
    else if (std::abs(reference_-target_) <= tolerance_ && std::abs(velocity_) < .002)
      integral_force_ = Clamp(integral_force_ + 4000*(reference_-position_)*dt, -1, 1);
    applied_ = active_ ? 400*(reference_-position_)-3.0*velocity_+integral_force_
                       : -stiffness_*position_-damping_*velocity_;
    applied_ = Clamp(applied_, -10, 10);
    joint_->SetForce(0, applied_);
    if (now-last_publish_ >= .0199 || old_led != led_ || event || last_publish_ < 0) {
      Publish(now); last_publish_ = now;
    }
    // Re-send materials so late clients/cameras also acquire the current lamp state.
    if (now-last_visual_ >= .1 || old_led != led_ || last_visual_ < 0) {
      UpdateVisual(); last_visual_ = now;
    }
  }
  void Publish(double now) {
    sensor_msgs::msg::JointState joint; joint.header.stamp = cabinet_sim::stamp(now);
    joint.name = {prefix_+"press_joint"}; joint.position = {position_};
    joint.velocity = {velocity_}; joint.effort = {applied_}; joint_pub_.publish(joint);
    std_msgs::msg::Bool led; led.data = led_; led_pub_.publish(led);
    std_msgs::msg::UInt32 count; count.data = count_; count_pub_.publish(count);
    std::string phase = active_ ? (hold_start_ < 0 ? "PRESSING" : "HOLDING") : (position_ > .0001 ? "RETURNING" : "IDLE");
    if (!fault_.empty()) phase = fault_;
    std::ostringstream s; s << std::boolalpha << std::setprecision(10)
      << "{\"sim_time\":" << now << ",\"depth_mm\":" << position_*1000
      << ",\"target_mm\":" << target_*1000 << ",\"led_on\":" << led_
      << ",\"valid\":" << valid_ << ",\"armed\":" << armed_
      << ",\"active\":" << active_ << ",\"reset_count\":" << count_
      << ",\"phase\":\"" << phase << "\"}";
    std_msgs::msg::String status; status.data = s.str(); status_pub_.publish(status);
  }
  void UpdateVisual() {
    const ignition::math::Color color = led_ ? ignition::math::Color(.03, 1.0, .13, 1)
                                           : ignition::math::Color(.005, .12, .045, 1);
    for (const auto &entry : cap_->Visuals()) {
      msgs::Visual visual = entry.second;
      auto material = visual.mutable_material();
      msgs::Set(material->mutable_ambient(), color); msgs::Set(material->mutable_diffuse(), color);
      msgs::Set(material->mutable_emissive(), led_ ? ignition::math::Color(.02, .8, .06, 1)
                                                  : ignition::math::Color(0, 0, 0, 1));
      visual_pub_->Publish(visual);
    }
  }
  physics::ModelPtr model_; physics::WorldPtr world_; physics::JointPtr joint_; physics::LinkPtr cap_;
  event::ConnectionPtr update_; transport::NodePtr transport_; transport::PublisherPtr visual_pub_;
  gazebo_ros::Node::SharedPtr node_;  
  cabinet_sim::Publisher joint_pub_, led_pub_, count_pub_, status_pub_, event_pub_;
  rclcpp::ServiceBase::SharedPtr press_srv_, release_srv_; std::mutex mutex_;
  std::string prefix_, fault_;
  bool initialized_ = false, active_ = false, auto_release_ = true;
  bool armed_ = true, valid_ = false, led_ = false;
  unsigned int count_ = 0;
  double position_ = 0, velocity_ = 0, applied_ = 0, reference_ = 0, target_ = 0;
  double max_depth_ = 0, valid_min_ = 0, valid_max_ = 0, rearm_depth_ = 0;
  double integral_force_ = 0;
  double lamp_duration_ = 0, tolerance_ = 0, stiffness_ = 0, damping_ = 0;
  double hold_seconds_ = .35, hold_start_ = -1, action_start_ = 0, led_until_ = -1;
  double last_sim_ = 0, last_publish_ = -1, last_visual_ = -1;
  cabinet_sim::SteadyTime last_wall_;
};
GZ_REGISTER_MODEL_PLUGIN(FuguiButtonPlugin)
}  // namespace gazebo
