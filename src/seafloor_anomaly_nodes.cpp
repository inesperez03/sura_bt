#include "sura_bt/seafloor_anomaly_nodes.hpp"

#include <algorithm>
#include <cmath>
#include <set>
#include <stdexcept>
#include <utility>

#include "sensor_msgs/point_cloud2_iterator.hpp"
#include "tf2_geometry_msgs/tf2_geometry_msgs.hpp"

namespace sura_bt
{
namespace
{
double median(std::vector<double> values)
{
  const auto middle = values.begin() + values.size() / 2;
  std::nth_element(values.begin(), middle, values.end());
  const double upper = *middle;
  if (values.size() % 2 != 0) {return upper;}
  std::nth_element(values.begin(), middle - 1, values.end());
  return 0.5 * (upper + *(middle - 1));
}

std::string namespacedTopic(std::string robot_namespace, std::string suffix)
{
  while (!robot_namespace.empty() && robot_namespace.front() == '/') {
    robot_namespace.erase(robot_namespace.begin());
  }
  while (!robot_namespace.empty() && robot_namespace.back() == '/') {
    robot_namespace.pop_back();
  }
  while (!suffix.empty() && suffix.front() == '/') {suffix.erase(suffix.begin());}
  return "/" + robot_namespace + "/" + suffix;
}
}  // namespace

SeafloorMap::SeafloorMap(double resolution, std::size_t max_cells)
: resolution_(resolution), max_cells_(max_cells)
{
  if (!std::isfinite(resolution_) || resolution_ <= 0.0 || max_cells_ == 0) {
    throw std::invalid_argument("Invalid seafloor map limits");
  }
}

SeafloorMap::Key SeafloorMap::keyFor(const geometry_msgs::msg::Point & point) const
{
  return {static_cast<int64_t>(std::floor(point.x / resolution_)),
    static_cast<int64_t>(std::floor(point.y / resolution_))};
}

geometry_msgs::msg::Point SeafloorMap::center(const Cell & cell) const
{
  geometry_msgs::msg::Point point;
  point.x = cell.sum.x / cell.count;
  point.y = cell.sum.y / cell.count;
  point.z = cell.sum.z / cell.count;
  return point;
}

void SeafloorMap::addPoints(const std::vector<geometry_msgs::msg::Point> & points)
{
  for (const auto & point : points) {
    if (!std::isfinite(point.x) || !std::isfinite(point.y) || !std::isfinite(point.z)) {
      continue;
    }
    const auto key = keyFor(point);
    auto & cell = cells_[key];
    cell.sum.x += point.x;
    cell.sum.y += point.y;
    cell.sum.z += point.z;
    ++cell.count;
    cell.last_update = ++update_sequence_;
    if (cells_.size() > max_cells_) {
      const auto oldest = std::min_element(
        cells_.begin(), cells_.end(), [](const auto & a, const auto & b) {
          return a.second.last_update < b.second.last_update;
        });
      cells_.erase(oldest);
    }
  }
}

std::vector<geometry_msgs::msg::Point> SeafloorMap::anomalyCentroids(
  double height_threshold, std::size_t min_cluster_points, double neighborhood_radius) const
{
  std::vector<geometry_msgs::msg::Point> centroids;
  if (!std::isfinite(height_threshold) || height_threshold <= 0.0 ||
    !std::isfinite(neighborhood_radius) || neighborhood_radius <= resolution_ ||
    min_cluster_points < 2)
  {
    return centroids;
  }
  std::map<Key, int> anomalous;
  const int range = static_cast<int>(std::ceil(neighborhood_radius / resolution_));
  const double radius_sq = neighborhood_radius * neighborhood_radius;
  for (const auto & [key, cell] : cells_) {
    const auto point = center(cell);
    std::vector<double> neighbor_heights;
    for (int dx = -range; dx <= range; ++dx) {
      for (int dy = -range; dy <= range; ++dy) {
        if (dx == 0 && dy == 0) {continue;}
        const auto other = cells_.find({key.first + dx, key.second + dy});
        if (other == cells_.end()) {continue;}
        const auto neighbor = center(other->second);
        const double x_distance = neighbor.x - point.x;
        const double y_distance = neighbor.y - point.y;
        if (x_distance * x_distance + y_distance * y_distance <= radius_sq) {
          neighbor_heights.push_back(neighbor.z);
        }
      }
    }
    if (neighbor_heights.size() < 5) {continue;}
    const double difference = point.z - median(std::move(neighbor_heights));
    if (difference >= height_threshold) {anomalous[key] = 1;}
    if (difference <= -height_threshold) {anomalous[key] = -1;}
  }
  std::set<Key> visited;
  for (const auto & [key, sign] : anomalous) {
    if (!visited.insert(key).second) {continue;}
    std::vector<Key> pending{key};
    geometry_msgs::msg::Point sum;
    std::size_t count = 0;
    while (!pending.empty()) {
      const auto current = pending.back();
      pending.pop_back();
      const auto point = center(cells_.at(current));
      sum.x += point.x;
      sum.y += point.y;
      sum.z += point.z;
      ++count;
      for (int dx = -1; dx <= 1; ++dx) {
        for (int dy = -1; dy <= 1; ++dy) {
          if (dx == 0 && dy == 0) {continue;}
          const Key neighbor{current.first + dx, current.second + dy};
          const auto found = anomalous.find(neighbor);
          if (found != anomalous.end() && found->second == sign && visited.insert(neighbor).second) {
            pending.push_back(neighbor);
          }
        }
      }
    }
    if (count < min_cluster_points) {continue;}
    geometry_msgs::msg::Point centroid;
    centroid.x = sum.x / count;
    centroid.y = sum.y / count;
    centroid.z = sum.z / count;
    centroids.push_back(centroid);
  }
  return centroids;
}

DetectSeafloorAnomalies::DetectSeafloorAnomalies(
  const std::string & name, const BT::NodeConfiguration & config)
: BT::StatefulActionNode(name, config)
{
  node_ = config.blackboard->get<rclcpp::Node::SharedPtr>("ros_node");
}

BT::PortsList DetectSeafloorAnomalies::providedPorts()
{
  return {
    BT::InputPort<std::string>("robot_namespace", "Surface robot with a multibeam sensor."),
    BT::InputPort<double>("height_threshold", 0.5,
      "Minimum vertical deviation from nearby mapped cells, in metres."),
    BT::InputPort<int>("min_cluster_points", 3,
      "Minimum number of adjacent anomalous map cells per group."),
    BT::InputPort<double>("scan_timeout", 3.0,
      "Maximum seconds without a usable multibeam scan and world_ned transform."),
    BT::InputPort<double>("map_resolution", 0.25,
      "XY cell size of the internal world_ned map, in metres."),
    BT::InputPort<double>("neighborhood_radius", 1.0,
      "Radius used to estimate nearby seafloor height, in metres."),
    BT::InputPort<int>("max_map_cells", 10000,
      "Maximum number of internal map cells; least recently updated cells are evicted."),
    BT::InputPort<std::string>("points_topic", "sensors/multibeam/points",
      "Multibeam PointCloud2 topic relative to the robot namespace."),
    BT::InputPort<std::string>("output_topic", "actions/seafloor_anomalies",
      "PointStamped topic for anomaly centroids in world_ned, relative to the robot namespace.")
  };
}

const char * DetectSeafloorAnomalies::main_description()
{
  return "Accumulates multibeam points in a bounded world_ned map and detects groups of "
    "neighboring cells with unusual height. Publishes one world_ned centroid per new group; "
    "keeps running until halted. The map remains internal. For surface robots only.";
}

BT::NodeStatus DetectSeafloorAnomalies::onStart()
{
  const auto robot = getInput<std::string>("robot_namespace");
  if (!robot || robot->empty()) {
    RCLCPP_ERROR(node_->get_logger(), "[sura_bt] DetectSeafloorAnomalies requires robot_namespace");
    return BT::NodeStatus::FAILURE;
  }
  std::string family;
  if (!config().blackboard->get("robot_family_" + robot.value(), family) || family != "surface") {
    RCLCPP_ERROR(node_->get_logger(), "[sura_bt] DetectSeafloorAnomalies requires a surface robot");
    return BT::NodeStatus::FAILURE;
  }
  height_threshold_ = getInput<double>("height_threshold").value_or(0.5);
  scan_timeout_ = getInput<double>("scan_timeout").value_or(3.0);
  map_resolution_ = getInput<double>("map_resolution").value_or(0.25);
  neighborhood_radius_ = getInput<double>("neighborhood_radius").value_or(1.0);
  const int min_points = getInput<int>("min_cluster_points").value_or(3);
  const int max_cells = getInput<int>("max_map_cells").value_or(10000);
  if (!std::isfinite(height_threshold_) || height_threshold_ <= 0.0 ||
    !std::isfinite(scan_timeout_) || scan_timeout_ <= 0.0 || min_points < 2 ||
    !std::isfinite(map_resolution_) || map_resolution_ <= 0.0 ||
    !std::isfinite(neighborhood_radius_) || neighborhood_radius_ <= map_resolution_ ||
    max_cells < min_points)
  {
    RCLCPP_ERROR(node_->get_logger(), "[sura_bt] Invalid anomaly detection parameters");
    return BT::NodeStatus::FAILURE;
  }
  min_cluster_points_ = static_cast<std::size_t>(min_points);
  max_map_cells_ = static_cast<std::size_t>(max_cells);
  map_ = std::make_unique<SeafloorMap>(map_resolution_, max_map_cells_);
  published_centroids_.clear();
  const auto input_topic = namespacedTopic(
    robot.value(), getInput<std::string>("points_topic").value_or("sensors/multibeam/points"));
  const auto output_topic = namespacedTopic(
    robot.value(), getInput<std::string>("output_topic").value_or("actions/seafloor_anomalies"));
  tf_buffer_ = std::make_unique<tf2_ros::Buffer>(node_->get_clock());
  tf_listener_ = std::make_unique<tf2_ros::TransformListener>(*tf_buffer_, node_, false);
  anomaly_pub_ = node_->create_publisher<geometry_msgs::msg::PointStamped>(output_topic, 10);
  {
    std::lock_guard<std::mutex> lock(cloud_mutex_);
    latest_cloud_.reset();
  }
  cloud_sub_ = node_->create_subscription<sensor_msgs::msg::PointCloud2>(
    input_topic, rclcpp::SensorDataQoS(),
    [this](sensor_msgs::msg::PointCloud2::ConstSharedPtr cloud) {cloudCallback(cloud);});
  last_scan_at_ = std::chrono::steady_clock::now();
  return BT::NodeStatus::RUNNING;
}

BT::NodeStatus DetectSeafloorAnomalies::onRunning()
{
  if (std::chrono::duration<double>(std::chrono::steady_clock::now() - last_scan_at_).count() >=
    scan_timeout_)
  {
    RCLCPP_ERROR(node_->get_logger(), "[sura_bt] Multibeam scan or TF timed out");
    resetSubscription();
    return BT::NodeStatus::FAILURE;
  }
  sensor_msgs::msg::PointCloud2::ConstSharedPtr cloud;
  {
    std::lock_guard<std::mutex> lock(cloud_mutex_);
    cloud = latest_cloud_;
  }
  if (!cloud) {return BT::NodeStatus::RUNNING;}

  geometry_msgs::msg::TransformStamped transform;
  try {
    transform = tf_buffer_->lookupTransform(
      "world_ned", cloud->header.frame_id, rclcpp::Time(cloud->header.stamp));
  } catch (const tf2::TransformException &) {
    std::lock_guard<std::mutex> lock(cloud_mutex_);
    if (latest_cloud_ == cloud &&
      std::chrono::duration<double>(
        std::chrono::steady_clock::now() - cloud_received_at_).count() > 0.5)
    {
      latest_cloud_.reset();
    }
    return BT::NodeStatus::RUNNING;
  }

  std::vector<geometry_msgs::msg::Point> points;
  try {
    sensor_msgs::PointCloud2ConstIterator<float> x(*cloud, "x");
    sensor_msgs::PointCloud2ConstIterator<float> y(*cloud, "y");
    sensor_msgs::PointCloud2ConstIterator<float> z(*cloud, "z");
    const std::size_t count = static_cast<std::size_t>(cloud->width) * cloud->height;
    points.reserve(count);
    for (std::size_t i = 0; i < count; ++i, ++x, ++y, ++z) {
      if (!std::isfinite(*x) || !std::isfinite(*y) || !std::isfinite(*z)) {continue;}
      geometry_msgs::msg::PointStamped sensor_point;
      sensor_point.header = cloud->header;
      sensor_point.point.x = *x;
      sensor_point.point.y = *y;
      sensor_point.point.z = *z;
      geometry_msgs::msg::PointStamped world_point;
      tf2::doTransform(sensor_point, world_point, transform);
      if (!std::isfinite(world_point.point.x) || !std::isfinite(world_point.point.y) ||
        !std::isfinite(world_point.point.z)) {continue;}
      points.push_back(world_point.point);
    }
  } catch (const std::runtime_error & error) {
    RCLCPP_ERROR(node_->get_logger(), "[sura_bt] Invalid multibeam cloud: %s", error.what());
    resetSubscription();
    return BT::NodeStatus::FAILURE;
  }

  {
    std::lock_guard<std::mutex> lock(cloud_mutex_);
    if (latest_cloud_ == cloud) {latest_cloud_.reset();}
  }
  if (points.empty()) {return BT::NodeStatus::RUNNING;}
  last_scan_at_ = std::chrono::steady_clock::now();
  map_->addPoints(points);
  const auto centroids = map_->anomalyCentroids(
    height_threshold_, min_cluster_points_, neighborhood_radius_);
  for (const auto & point : centroids) {
    const bool already_published = std::any_of(
      published_centroids_.begin(), published_centroids_.end(), [&](const auto & previous) {
        const double dx = point.x - previous.x;
        const double dy = point.y - previous.y;
        return dx * dx + dy * dy <= map_resolution_ * map_resolution_ * 4.0;
      });
    if (already_published) {continue;}
    geometry_msgs::msg::PointStamped output;
    output.header.stamp = cloud->header.stamp;
    output.header.frame_id = "world_ned";
    output.point = point;
    anomaly_pub_->publish(output);
    published_centroids_.push_back(point);
  }
  return BT::NodeStatus::RUNNING;
}

void DetectSeafloorAnomalies::onHalted()
{
  resetSubscription();
}

void DetectSeafloorAnomalies::cloudCallback(sensor_msgs::msg::PointCloud2::ConstSharedPtr cloud)
{
  std::lock_guard<std::mutex> lock(cloud_mutex_);
  if (!latest_cloud_) {
    latest_cloud_ = std::move(cloud);
    cloud_received_at_ = std::chrono::steady_clock::now();
  }
}

void DetectSeafloorAnomalies::resetSubscription()
{
  cloud_sub_.reset();
  tf_listener_.reset();
  tf_buffer_.reset();
  std::lock_guard<std::mutex> lock(cloud_mutex_);
  latest_cloud_.reset();
  map_.reset();
  published_centroids_.clear();
}

}  // namespace sura_bt
