#include <cabinet_sim_support/transport.hpp>
#include <algorithm>
#include <cmath>
#include <functional>
#include <memory>
#include <mutex>
#include <string>
#include <gazebo/common/Events.hh>
#include <gazebo/common/Plugin.hh>
#include <gazebo/physics/physics.hh>
#include <sensor_msgs/msg/joint_state.hpp>
#include <xuanniu_gazebo/msg/switch_command.hpp>
#include <xuanniu_gazebo/msg/switch_status.hpp>

namespace gazebo {
class SelectorPlugin : public ModelPlugin {
 public:
  ~SelectorPlugin() override {
    update_.reset(); pause_connection_.reset();
    
    
     
  }
  void Load(physics::ModelPtr model, sdf::ElementPtr sdf) override {
    world_=model->GetWorld();
    name_=Param<std::string>(sdf,"jointName","xuanniu_selector_joint");
    joint_=model->GetJoint(name_);
    if (!joint_ || std::abs(joint_->LowerLimit(0)+half_)>1e-6 || std::abs(joint_->UpperLimit(0)-half_)>1e-6) {
      gzerr << "Selector requires a revolute joint limited to +/-pi/4.\n"; return;
    }
    speed_=Param<double>(sdf,"targetSpeed",M_PI/2);
    kp_=Param<double>(sdf,"positionGain",.035); kd_=Param<double>(sdf,"velocityGain",.00032);
    detent_=Param<double>(sdf,"detentTorque",.0015);
    for (double p : {speed_,kp_,kd_,detent_}) if (!std::isfinite(p) || p<=0) {
      gzerr << "Selector controller parameters must be positive and finite.\n"; return;
    }
    if (!rclcpp::ok()) { gzerr << "Initialize ROS 2 with gazebo_ros before loading plugins\n"; return; }
    node_ = cabinet_sim::node(sdf, Param<std::string>(sdf,"robotNamespace","/xuanniu"));
    
    command_=cabinet_sim::subscribe(node_, "command",1,&SelectorPlugin::Command,this);
    status_pub_=cabinet_sim::advertise<xuanniu_gazebo::msg::SwitchStatus>(node_, "status",1);
    joints_pub_=cabinet_sim::advertise<sensor_msgs::msg::JointState>(node_, "joint_states",1);
    
    update_=event::Events::ConnectWorldUpdateBegin(std::bind(&SelectorPlugin::Update,this,std::placeholders::_1));
    pause_connection_=event::Events::ConnectPause(std::bind(&SelectorPlugin::Paused,this,std::placeholders::_1));
    CAB_INFO_STREAM("Selector ready: " << node_->get_namespace() << "; front view left=MANUAL (+45 deg), centre=STOP, right=AUTO (-45 deg).");
  }
  void Reset() override {
    std::lock_guard<std::mutex> lock(mutex_);
    initialized_=false; pending_=false; dragging_=false; active_=false; pause_pending_=false;
    stable_=0; last_publish_=-1; last_stamp_=rclcpp::Time(0, 0, RCL_ROS_TIME); error_.clear();
  }
 private:
  template<class T> static T Param(const sdf::ElementPtr &sdf,const std::string &key,const T &fallback) {
    return sdf->HasElement(key)?sdf->Get<T>(key):fallback;
  }
  static double Clamp(double x,double lo,double hi) { return std::max(lo,std::min(x,hi)); }
  static double Snap(double a) { return a>M_PI/8+1e-12?M_PI/4:(a<-M_PI/8-1e-12?-M_PI/4:0); }
  void Paused(bool paused) {
    if (!paused) return;
    std::lock_guard<std::mutex> lock(mutex_);
    pause_pending_=true;
  }
  void Command(const xuanniu_gazebo::msg::SwitchCommand::ConstSharedPtr &msg) {
    std::lock_guard<std::mutex> lock(mutex_);
    if (!initialized_ || world_->IsPaused() || (cabinet_sim::SteadyTime::now()-last_wall_).toSec()>.5) {
      error_="SIMULATION_NOT_READY"; pending_=false; return;
    }
    if (!std::isfinite(msg->target_angle_rad) || std::abs(msg->target_angle_rad)>half_+1e-10) {
      error_="INVALID_ANGLE"; return;
    }
    const double stamp=rclcpp::Time(msg->header.stamp).seconds();
    if ((rclcpp::Time(msg->header.stamp).nanoseconds()==0) || stamp>last_sim_+.1 || last_sim_-stamp>.5 || rclcpp::Time(msg->header.stamp)<last_stamp_) {
      error_="STALE_COMMAND"; return;
    }
    if (fatal_) { error_="NONFINITE_PHYSICS"; return; }
    last_stamp_=msg->header.stamp; next_=*msg; pending_=true;
    command_wall_=cabinet_sim::SteadyTime::now(); error_.clear();
  }
  void Select(double a) { target_=Snap(a); reference_=position_; active_=true; dragging_=false; stable_=0; action_start_=last_sim_; }
  void Update(const common::UpdateInfo &info) {
    std::lock_guard<std::mutex> lock(mutex_);
    const double now=info.simTime.Double();
    const double dt=Clamp(now-last_sim_,0,.01);
    const auto wall=cabinet_sim::SteadyTime::now();
    const bool gap=initialized_ && (wall-last_wall_).toSec()>.5;
    last_sim_=now; last_wall_=wall;
    if (!initialized_) {
      joint_->SetPosition(0,0,true); joint_->SetVelocity(0,0);
      position_=velocity_=target_=reference_=stable_=0; active_=dragging_=pending_=fatal_=false;
      initialized_=true; error_.clear();
    }
    position_=joint_->Position(0); velocity_=joint_->GetVelocity(0);
    if (!std::isfinite(position_) || !std::isfinite(velocity_)) {
      fatal_=true; active_=dragging_=pending_=false; error_="NONFINITE_PHYSICS";
      joint_->SetForce(0,0); Publish(now,0); return;
    }
    if (gap || pause_pending_) { pending_=false; if (dragging_) Select(position_); pause_pending_=false; }
    if (pending_) {
      if (next_.dragging) {
        if (!dragging_) reference_=position_;
        dragging_=active_=true; target_=Clamp(next_.target_angle_rad,-half_,half_); stable_=0;
      } else Select(next_.target_angle_rad);
      pending_=false;
    }
    if (dragging_ && (wall-command_wall_).toSec()>.5) Select(position_);
    if (active_) reference_+=Clamp(target_-reference_,-speed_*dt,speed_*dt);
    const double nearest=Snap(position_);
    const bool settled=std::abs(position_-nearest)<M_PI/180 && std::abs(velocity_)<.05;
    stable_=settled?stable_+dt:0;
    if (active_ && !dragging_) {
      if (std::abs(position_-target_)<M_PI/180 && stable_>=.1) active_=false;
      else if (now-action_start_>3) { active_=false; error_="TARGET_TIMEOUT"; target_=nearest; }
    }
    // During commands use a damped servo; idle has three physical potential wells.
    // -sin(8*q) has stable equilibria at -pi/4, 0, +pi/4, barriers at +/-pi/8.
    double torque=active_?kp_*(reference_-position_)-kd_*velocity_:-detent_*std::sin(8*position_)-kd_*velocity_;
    torque=Clamp(torque,-.04,.04); joint_->SetForce(0,torque);
    if (now-last_publish_>=.0199 || last_publish_<0 || now<last_publish_) { Publish(now,torque); last_publish_=now; }
  }
  void Publish(double now,double torque) {
    sensor_msgs::msg::JointState j; j.header.stamp=cabinet_sim::stamp(now); j.name={name_};
    j.position={position_}; j.velocity={velocity_}; j.effort={torque}; joints_pub_.publish(j);
    xuanniu_gazebo::msg::SwitchStatus s; s.header=j.header;
    s.angle_rad=position_; s.velocity_rad_s=velocity_; s.target_angle_rad=target_;
    s.mode="TRANSITION";
    if (!fatal_ && stable_>=.1 && !active_) s.mode=position_>M_PI/8?"MANUAL":(position_<-M_PI/8?"AUTO":"STOP");
    s.dragging=dragging_; s.moving=active_ || s.mode=="TRANSITION"; s.error=error_; status_pub_.publish(s);
  }
  physics::WorldPtr world_; physics::JointPtr joint_; event::ConnectionPtr update_,pause_connection_;
  gazebo_ros::Node::SharedPtr node_;  
  rclcpp::SubscriptionBase::SharedPtr command_; cabinet_sim::Publisher status_pub_,joints_pub_; std::mutex mutex_;
  std::string name_,error_; xuanniu_gazebo::msg::SwitchCommand next_; rclcpp::Time last_stamp_{0, 0, RCL_ROS_TIME};
  cabinet_sim::SteadyTime last_wall_,command_wall_;
  const double half_=M_PI/4;
  double speed_,kp_,kd_,detent_,position_=0,velocity_=0,target_=0,reference_=0,stable_=0;
  double last_sim_=0,last_publish_=-1,action_start_=0;
  bool initialized_=false,pending_=false,dragging_=false,active_=false,fatal_=false,pause_pending_=false;
};
GZ_REGISTER_MODEL_PLUGIN(SelectorPlugin)
}  // namespace gazebo
