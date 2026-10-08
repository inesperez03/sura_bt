#pragma once

#include <atomic>
#include <chrono>
#include <memory>
#include <mutex>
#include <string>
#include <thread>

#include "behaviortree_cpp_v3/action_node.h"
#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/string.hpp"

namespace sura_bt
{

class AnalyzePhoto : public BT::StatefulActionNode
{
public:
  AnalyzePhoto(const std::string & name, const BT::NodeConfiguration & config);
  ~AnalyzePhoto() override;
  static BT::PortsList providedPorts();
  static const char * main_description();
  BT::NodeStatus onStart() override;
  BT::NodeStatus onRunning() override;
  void onHalted() override;

private:
  struct RequestState
  {
    std::atomic<bool> cancel{false};
    std::atomic<bool> done{false};
    std::mutex mutex;
    std::string answer;
    std::string error;
  };

  void stopRequest();
  void publishEvent(const std::string & status, const std::string & message);

  rclcpp::Node::SharedPtr node_;
  rclcpp::Publisher<std_msgs::msg::String>::SharedPtr event_pub_;
  std::shared_ptr<RequestState> request_;
  std::thread worker_;
  std::string robot_;
  std::string image_path_;
  std::string prompt_;
};

}  // namespace sura_bt
