#include <cabinet_sim_support/transport.hpp>
// Read-only joint aggregation and ten red indicator visuals. No physics drive.
#include <cmath>
#include <functional>
#include <memory>
#include <string>
#include <vector>
#include <gazebo/gazebo.hh>
#include <gazebo/physics/physics.hh>
#include <gazebo/transport/transport.hh>
#include <gazebo/msgs/msgs.hh>
#include <sensor_msgs/msg/joint_state.hpp>
#include <std_msgs/msg/u_int8_multi_array.hpp>

namespace gazebo {
class CabinetStatePlugin : public ModelPlugin {
 public:
  ~CabinetStatePlugin() override { update_.reset(); }

  void Load(physics::ModelPtr model, sdf::ElementPtr sdf) override {
    model_ = model;
    if (!rclcpp::ok()) { gzerr << "Initialize ROS 2 with gazebo_ros before loading plugins\n"; return; }
    node_ = cabinet_sim::node(sdf, sdf->Get<std::string>("robotNamespace"));
    states_ = cabinet_sim::advertise<sensor_msgs::msg::JointState>(node_, "joint_states", 1);
    lights_ = cabinet_sim::advertise<std_msgs::msg::UInt8MultiArray>(node_, "red_lamps", 1, true);
    for (auto e = sdf->GetElement("joint"); e; e = e->GetNextElement("joint")) {
      const auto name = e->Get<std::string>();
      auto joint = model_->GetJoint(name);
      if (!joint) { gzerr << "Cabinet missing joint: " << name << "\n"; continue; }
      joints_.push_back(joint);
      message_.name.push_back(name);
    }
    message_.position.resize(joints_.size());
    message_.velocity.resize(joints_.size());
    for (auto e = sdf->GetElement("breaker"); e; e = e->GetNextElement("breaker")) {
      Lamp lamp;
      const auto name = e->Get<std::string>();
      lamp.joint = model_->GetJoint(name + "__breaker_toggle_joint");
      // Fixed visual links are normally lumped into cabinet_base_link. Use
      // actual scoped visual messages/IDs, not invented Gazebo entity names.
      for (const auto &link : model_->GetLinks())
        for (const auto &entry : link->Visuals())
          if (entry.second.name().find(name + "__red_lamp") != std::string::npos)
            lamp.visuals.push_back(entry.second);
      if (!lamp.joint || lamp.visuals.empty()) {
        gzerr << "Cabinet missing lamp/joint for " << name << "\n";
      }
      lamps_.push_back(lamp);
    }
    transport_.reset(new transport::Node());
    transport_->Init(model_->GetWorld()->Name());
    visuals_ = transport_->Advertise<msgs::Visual>("~/visual");
    update_ = event::Events::ConnectWorldUpdateBegin(
        std::bind(&CabinetStatePlugin::Update, this, std::placeholders::_1));
    CAB_INFO_STREAM("Cabinet aggregate: " << joints_.size() << " joints, "
                    << lamps_.size() << " red lamps");
  }

  void Reset() override {
    last_state_ = last_visual_ = -1;
    for (auto &lamp : lamps_) lamp.on = false;
  }

 private:
  struct Lamp {
    physics::JointPtr joint;
    std::vector<msgs::Visual> visuals;
    bool on = false;
  };
  void Update(const common::UpdateInfo &info) {
    const double t = info.simTime.Double();
    if (t < last_state_ || t < last_visual_) Reset();
    // Same actual-angle hysteresis as the original breaker plugin, every step.
    for (auto &lamp : lamps_) {
      if (!lamp.joint) continue;
      const double q = lamp.joint->Position(0);
      if (q < -0.174532925199) lamp.on = true;
      else if (q > 0.174532925199) lamp.on = false;
    }
    if (t - last_state_ >= .02 || last_state_ < 0) {
      message_.header.stamp = cabinet_sim::stamp(t);
      for (size_t i = 0; i < joints_.size(); ++i) {
        message_.position[i] = joints_[i]->Position(0);
        message_.velocity[i] = joints_[i]->GetVelocity(0);
      }
      states_.publish(message_);
      last_state_ = t;
    }
    if (t - last_visual_ >= .1 || last_visual_ < 0) {
      std_msgs::msg::UInt8MultiArray lights;
      for (auto &lamp : lamps_) {
        lights.data.push_back(lamp.on ? 1 : 0);
        const ignition::math::Color color = lamp.on ?
            ignition::math::Color(1, .025, .015, 1) :
            ignition::math::Color(.15, .008, .005, 1);
        const ignition::math::Color emission = lamp.on ?
            ignition::math::Color(.8, .012, .008, 1) :
            ignition::math::Color(0, 0, 0, 1);
        for (auto visual : lamp.visuals) {
          msgs::Set(visual.mutable_material()->mutable_ambient(), color);
          msgs::Set(visual.mutable_material()->mutable_diffuse(), color);
          msgs::Set(visual.mutable_material()->mutable_emissive(), emission);
          visuals_->Publish(visual);
        }
      }
      lights_.publish(lights);
      last_visual_ = t;
    }
  }
  physics::ModelPtr model_;
  gazebo_ros::Node::SharedPtr node_;
  std::vector<physics::JointPtr> joints_;
  std::vector<Lamp> lamps_;
  sensor_msgs::msg::JointState message_;
  cabinet_sim::Publisher states_, lights_;
  transport::NodePtr transport_;
  transport::PublisherPtr visuals_;
  event::ConnectionPtr update_;
  double last_state_ = -1, last_visual_ = -1;
};
GZ_REGISTER_MODEL_PLUGIN(CabinetStatePlugin)
}  // namespace gazebo
