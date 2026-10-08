#pragma once

#include <chrono>
#include <cstdint>
#include <map>
#include <memory>
#include <mutex>
#include <string>
#include <vector>

#include "behaviortree_cpp_v3/action_node.h"
#include "geometry_msgs/msg/point.hpp"
#include "geometry_msgs/msg/point_stamped.hpp"
#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/point_cloud2.hpp"
#include "tf2_ros/buffer.h"
#include "tf2_ros/transform_listener.h"

namespace sura_bt
{

class SeafloorMap
{
public:
  explicit SeafloorMap(double resolution, std::size_t max_cells);
  void addPoints(const std::vector<geometry_msgs::msg::Point> & points);
  std::vector<geometry_msgs::msg::Point> anomalyCentroids(
    double height_threshold, std::size_t min_cluster_points, double neighborhood_radius) const;
  std::size_t size() const {return cells_.size();}

private:
  using Key = std::pair<int64_t, int64_t>;
  struct Cell
  {
    geometry_msgs::msg::Point sum;
    std::size_t count{0};
    uint64_t last_update{0};
  };
  geometry_msgs::msg::Point center(const Cell & cell) const;
  Key keyFor(const geometry_msgs::msg::Point & point) const;

  double resolution_;
  std::size_t max_cells_;
  uint64_t update_sequence_{0};
  std::map<Key, Cell> cells_;
};

class DetectSeafloorAnomalies : public BT::StatefulActionNode
{
public:
  DetectSeafloorAnomalies(const std::string & name, const BT::NodeConfiguration & config);
  static BT::PortsList providedPorts();
  static const char * main_description();
  BT::NodeStatus onStart() override;
  BT::NodeStatus onRunning() override;
  void onHalted() override;

private:
  void cloudCallback(sensor_msgs::msg::PointCloud2::ConstSharedPtr cloud);
  void resetSubscription();

  rclcpp::Node::SharedPtr node_;
  rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr cloud_sub_;
  rclcpp::Publisher<geometry_msgs::msg::PointStamped>::SharedPtr anomaly_pub_;
  std::unique_ptr<tf2_ros::Buffer> tf_buffer_;
  std::unique_ptr<tf2_ros::TransformListener> tf_listener_;
  std::mutex cloud_mutex_;
  sensor_msgs::msg::PointCloud2::ConstSharedPtr latest_cloud_;
  std::chrono::steady_clock::time_point cloud_received_at_;
  std::chrono::steady_clock::time_point last_scan_at_;
  std::unique_ptr<SeafloorMap> map_;
  std::vector<geometry_msgs::msg::Point> published_centroids_;
  double height_threshold_{0.5};
  double scan_timeout_{3.0};
  double map_resolution_{0.25};
  double neighborhood_radius_{1.0};
  std::size_t min_cluster_points_{3};
  std::size_t max_map_cells_{10000};
};

}  // namespace sura_bt
