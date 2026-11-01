#include <chrono>
#include <iostream>
#include <thread>
#include <gazebo/gazebo_client.hh>
#include <gazebo/transport/transport.hh>
#include <gazebo/msgs/msgs.hh>

// Unlike the generic CLI, publish with an explicit type and wait for a
// transport handshake. The server control topic normally has no publisher.
int main(int argc, char **argv) {
  gazebo::client::setup(argc, argv);
  auto node = gazebo::transport::NodePtr(new gazebo::transport::Node());
  node->Init();
  auto publisher = node->Advertise<gazebo::msgs::ServerControl>("/gazebo/server/control");
  const auto deadline = std::chrono::steady_clock::now() + std::chrono::seconds(5);
  while (!publisher->HasConnections() && std::chrono::steady_clock::now() < deadline)
    std::this_thread::sleep_for(std::chrono::milliseconds(20));
  if (!publisher->HasConnections()) {
    std::cerr << "No Gazebo server-control subscriber\n";
    gazebo::client::shutdown();
    return 1;
  }
  gazebo::msgs::ServerControl message;
  message.set_stop(true);
  publisher->Publish(message, true);
  std::this_thread::sleep_for(std::chrono::milliseconds(200));
  gazebo::client::shutdown();
  return 0;
}
