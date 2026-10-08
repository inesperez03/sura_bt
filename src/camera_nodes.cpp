#include "sura_bt/camera_nodes.hpp"
#include "sura_bt/mission_failure_reasons.hpp"

#include <cv_bridge/cv_bridge.h>

#include <chrono>
#include <cstdint>
#include <filesystem>
#include <iomanip>
#include <memory>
#include <sstream>
#include <string>
#include <thread>

#include "opencv2/imgcodecs.hpp"
#include "opencv2/imgproc.hpp"

namespace sura_bt
{
namespace
{

rclcpp::Node::SharedPtr getRosNode(const BT::NodeConfiguration & config)
{
  return config.blackboard->get<rclcpp::Node::SharedPtr>("ros_node");
}

std::string stripSlashes(std::string value)
{
  while (!value.empty() && value.front() == '/') { value.erase(value.begin()); }
  while (!value.empty() && value.back() == '/') { value.pop_back(); }
  return value;
}

std::string requiredRobotNamespace(const BT::TreeNode & node)
{
  const auto robot_namespace = node.getInput<std::string>("robot_namespace");
  if (!robot_namespace || stripSlashes(robot_namespace.value()).empty())
  {
    throw BT::RuntimeError(node.name(), " requires a non-empty robot_namespace input");
  }
  return stripSlashes(robot_namespace.value());
}

std::string namespacedTopic(const std::string & robot_namespace, const std::string & topic)
{
  if (!topic.empty() && topic.front() == '/') { return topic; }
  return "/" + robot_namespace + "/" + stripSlashes(topic);
}

constexpr double PHOTO_TIMEOUT_SEC = 2.0;
constexpr const char * PHOTO_OUTPUT_DIR = "photos";
constexpr const char * PHOTO_FILENAME_PREFIX = "photo";

}  // namespace

bool TakePhoto::saveImage(
  const ImageMsg & image,
  const std::string & path,
  std::string & error) const
{
  const auto pixel_count = static_cast<size_t>(image.width) * static_cast<size_t>(image.height);
  if (image.width == 0 || image.height == 0 || pixel_count == 0)
  {
    error = "image has zero width or height";
    return false;
  }

  const bool mono =
    image.encoding == "mono8" ||
    image.encoding == "8UC1";
  const bool rgb =
    image.encoding == "rgb8" ||
    image.encoding == "bgr8" ||
    image.encoding == "rgba8" ||
    image.encoding == "bgra8";

  if (!mono && !rgb)
  {
    error = "unsupported image encoding '" + image.encoding + "'";
    return false;
  }

  const size_t input_channels = mono ? 1U :
    (image.encoding == "rgba8" || image.encoding == "bgra8" ? 4U : 3U);
  const size_t min_step = static_cast<size_t>(image.width) * input_channels;
  if (image.step < min_step)
  {
    error = "image step is smaller than expected for encoding";
    return false;
  }
  if (image.data.size() < image.step * static_cast<size_t>(image.height))
  {
    error = "image data is smaller than step * height";
    return false;
  }

  try
  {
    cv_bridge::CvImagePtr cv_image = cv_bridge::toCvCopy(image, image.encoding);
    cv::Mat output;
    if (image.encoding == "bgr8" || image.encoding == "mono8" || image.encoding == "8UC1")
    {
      output = cv_image->image;
    }
    else if (image.encoding == "rgb8")
    {
      cv::cvtColor(cv_image->image, output, cv::COLOR_RGB2BGR);
    }
    else if (image.encoding == "rgba8")
    {
      cv::cvtColor(cv_image->image, output, cv::COLOR_RGBA2BGR);
    }
    else if (image.encoding == "bgra8")
    {
      cv::cvtColor(cv_image->image, output, cv::COLOR_BGRA2BGR);
    }
    else
    {
      error = "unsupported image encoding '" + image.encoding + "'";
      return false;
    }

    std::filesystem::create_directories(std::filesystem::path(path).parent_path());
    if (!cv::imwrite(path, output))
    {
      error = "failed to write PNG image";
      return false;
    }
  }
  catch (const cv_bridge::Exception & exception)
  {
    error = exception.what();
    return false;
  }

  return true;
}

std::string TakePhoto::makeOutputPath(
  const rclcpp::Node::SharedPtr & ros_node,
  const ImageMsg & image) const
{
  const auto stamp = image.header.stamp.sec != 0 || image.header.stamp.nanosec != 0 ?
    rclcpp::Time(image.header.stamp) :
    ros_node->now();

  std::stringstream filename;
  filename << PHOTO_FILENAME_PREFIX << "_"
           << stamp.nanoseconds() / 1000000000LL << "_"
           << std::setw(9) << std::setfill('0') << stamp.nanoseconds() % 1000000000LL
           << ".png";

  return std::filesystem::absolute(
    std::filesystem::path(PHOTO_OUTPUT_DIR) / filename.str()).string();
}

TakePhoto::TakePhoto(
  const std::string & name,
  const BT::NodeConfiguration & config)
: BT::SyncActionNode(name, config)
{
}

BT::PortsList TakePhoto::providedPorts()
{
  return {
    BT::InputPort<std::string>("robot_namespace", "Robot namespace whose camera is used."),
    BT::InputPort<std::string>("topic", "Image topic to capture from."),
    BT::OutputPort<std::string>("image_path", "Absolute path of the saved PNG image.")
  };
}

const char * TakePhoto::main_description()
{
  return "Captures an image from a robot camera topic, saves it as PNG, and outputs its "
    "absolute image_path for later analysis.";
}

BT::NodeStatus TakePhoto::tick()
{
  const auto ros_node = getRosNode(config());
  const auto topic = getInput<std::string>("topic");

  if (!topic || topic.value().empty())
  {
    recordMissionFailure(config(), name(), "No camera topic is configured for this capture.");
    RCLCPP_ERROR(
      ros_node->get_logger(),
      "[sura_bt] TakePhoto requires non-empty topic input");
    return BT::NodeStatus::FAILURE;
  }
  const auto resolved_topic = namespacedTopic(requiredRobotNamespace(*this), topic.value());

  std::shared_ptr<ImageMsg> captured_image;
  auto callback_group = ros_node->create_callback_group(
    rclcpp::CallbackGroupType::MutuallyExclusive);
  rclcpp::SubscriptionOptions subscription_options;
  subscription_options.callback_group = callback_group;
  auto image_sub = ros_node->create_subscription<ImageMsg>(
    resolved_topic,
    rclcpp::SensorDataQoS(),
    [this, &captured_image](const ImageMsg::SharedPtr msg)
    {
      std::lock_guard<std::mutex> lock(image_mutex_);
      if (!captured_image)
      {
        captured_image = std::make_shared<ImageMsg>(*msg);
      }
    },
    subscription_options);

  const auto deadline = std::chrono::steady_clock::now() +
    std::chrono::duration<double>(PHOTO_TIMEOUT_SEC);
  while (rclcpp::ok() && std::chrono::steady_clock::now() < deadline)
  {
    {
      std::lock_guard<std::mutex> lock(image_mutex_);
      if (captured_image)
      {
        break;
      }
    }
    rclcpp::spin_some(ros_node);
    std::this_thread::sleep_for(std::chrono::milliseconds(10));
  }

  image_sub.reset();

  if (!captured_image)
  {
    std::ostringstream elapsed;
    elapsed << PHOTO_TIMEOUT_SEC;
    recordMissionFailure(
      config(), name(),
      "No image arrived on topic " + resolved_topic + " within " + elapsed.str() +
      " seconds. Check that the camera is active and publishing images.");
    RCLCPP_ERROR(
      ros_node->get_logger(),
      "[sura_bt] TakePhoto timed out waiting %.3fs for image topic %s",
      PHOTO_TIMEOUT_SEC,
      resolved_topic.c_str());
    return BT::NodeStatus::FAILURE;
  }

  const auto path = makeOutputPath(
    ros_node,
    *captured_image);
  std::string error;
  if (!saveImage(*captured_image, path, error))
  {
    recordMissionFailure(
      config(), name(),
      "The camera published an image, but it could not be saved from " + resolved_topic +
      ": " + error);
    RCLCPP_ERROR(
      ros_node->get_logger(),
      "[sura_bt] TakePhoto failed to save image from %s: %s",
      topic.value().c_str(),
      error.c_str());
    return BT::NodeStatus::FAILURE;
  }

  RCLCPP_INFO(
    ros_node->get_logger(),
    "[sura_bt] Action: take photo. topic=%s path=%s",
    resolved_topic.c_str(),
    path.c_str());

  setOutput("image_path", path);

  return BT::NodeStatus::SUCCESS;
}

}  // namespace sura_bt
