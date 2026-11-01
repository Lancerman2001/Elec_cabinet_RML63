#pragma once
#include <chrono>
#include <cmath>
#include <memory>
#include <mutex>
#include <rclcpp/rclcpp.hpp>
#include <gazebo_ros/node.hpp>

namespace cabinet_sim {
inline builtin_interfaces::msg::Time stamp(double seconds) {
  return rclcpp::Time(static_cast<int64_t>(std::llround(seconds * 1e9)), RCL_ROS_TIME);
}
inline rclcpp::Clock& wall_clock() {
  static rclcpp::Clock clock(RCL_STEADY_TIME); return clock;
}
struct SteadyTime {
  double value = 0;
  static SteadyTime now() {
    return {std::chrono::duration<double>(std::chrono::steady_clock::now().time_since_epoch()).count()};
  }
  SteadyTime operator-(const SteadyTime& other) const { return {value - other.value}; }
  double toSec() const { return value; }
};
inline gazebo_ros::Node::SharedPtr node(sdf::ElementPtr sdf, const std::string& fallback) {
  if (!sdf->HasElement("ros")) {
    auto ros = sdf->AddElement("ros");
    ros->AddElement("namespace")->Set(fallback);
  }
  return gazebo_ros::Node::Get(sdf);
}
// A small type-erased publisher keeps the mechanism implementations independent
// of message template declarations; every underlying endpoint is a ROS 2 entity.
class Publisher {
 public:
  Publisher() = default;
  explicit Publisher(rclcpp::PublisherBase::SharedPtr publisher) : publisher_(publisher) {}
  template<class Message> void publish(const Message& message) const {
    auto typed = std::dynamic_pointer_cast<rclcpp::Publisher<Message>>(publisher_);
    if (typed) typed->publish(message);
  }
  void reset() { publisher_.reset(); }
 private:
  rclcpp::PublisherBase::SharedPtr publisher_;
};
template<class Message>
Publisher advertise(const gazebo_ros::Node::SharedPtr& node, const std::string& topic,
                    size_t depth, bool durable = false) {
  auto qos = rclcpp::QoS(depth).reliable();
  if (durable) qos.transient_local();
  return Publisher(node->create_publisher<Message>(topic, qos));
}
template<class Message, class Owner>
rclcpp::SubscriptionBase::SharedPtr subscribe(const gazebo_ros::Node::SharedPtr& node,
    const std::string& topic, size_t depth,
    void (Owner::*method)(const std::shared_ptr<const Message>&), Owner* owner) {
  return node->create_subscription<Message>(topic, rclcpp::QoS(depth).reliable(),
    [owner, method](std::shared_ptr<const Message> message) { (owner->*method)(message); });
}
template<class Service, class Owner>
rclcpp::ServiceBase::SharedPtr service(const gazebo_ros::Node::SharedPtr& node,
    const std::string& name,
    bool (Owner::*method)(typename Service::Request&, typename Service::Response&), Owner* owner) {
  return node->create_service<Service>(name,
    [owner, method](const std::shared_ptr<typename Service::Request> request,
                    std::shared_ptr<typename Service::Response> response) {
      (owner->*method)(*request, *response);
    });
}
}  // namespace cabinet_sim

#define CAB_LOGGER rclcpp::get_logger("cabinet_sim")
#define CAB_INFO(...) RCLCPP_INFO(CAB_LOGGER, __VA_ARGS__)
#define CAB_ERROR(...) RCLCPP_ERROR(CAB_LOGGER, __VA_ARGS__)
#define CAB_WARN(...) RCLCPP_WARN(CAB_LOGGER, __VA_ARGS__)
#define CAB_INFO_STREAM(value) RCLCPP_INFO_STREAM(CAB_LOGGER, value)
#define CAB_ERROR_STREAM(value) RCLCPP_ERROR_STREAM(CAB_LOGGER, value)
#define CAB_WARN_STREAM(value) RCLCPP_WARN_STREAM(CAB_LOGGER, value)
#define CAB_INFO_STREAM_NAMED(name, value) RCLCPP_INFO_STREAM(rclcpp::get_logger(name), value)
#define CAB_WARN_NAMED(name, ...) RCLCPP_WARN(rclcpp::get_logger(name), __VA_ARGS__)
#define CAB_WARN_THROTTLE(seconds, ...) RCLCPP_WARN_THROTTLE(CAB_LOGGER, cabinet_sim::wall_clock(), static_cast<int64_t>((seconds)*1000), __VA_ARGS__)
#define CAB_WARN_THROTTLE_NAMED(seconds, name, ...) RCLCPP_WARN_THROTTLE(rclcpp::get_logger(name), cabinet_sim::wall_clock(), static_cast<int64_t>((seconds)*1000), __VA_ARGS__)
#define CAB_ERROR_THROTTLE_NAMED(seconds, name, ...) RCLCPP_ERROR_THROTTLE(rclcpp::get_logger(name), cabinet_sim::wall_clock(), static_cast<int64_t>((seconds)*1000), __VA_ARGS__)
