#pragma once

#include <mutex>
#include <string>

#include "behaviortree_cpp_v3/action_node.h"
#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/image.hpp"

namespace sura_bt
{

class TakePhoto : public BT::SyncActionNode
{
public:
  TakePhoto(
    const std::string & name,
    const BT::NodeConfiguration & config);

  static BT::PortsList providedPorts();
  static const char * main_description();

  BT::NodeStatus tick() override;

private:
  using ImageMsg = sensor_msgs::msg::Image;

  bool saveImage(
    const ImageMsg & image,
    const std::string & path,
    std::string & error) const;
  std::string makeOutputPath(
    const rclcpp::Node::SharedPtr & ros_node,
    const ImageMsg & image) const;

  mutable std::mutex image_mutex_;
};

}  // namespace sura_bt
