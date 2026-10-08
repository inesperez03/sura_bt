#include <arpa/inet.h>
#include <netinet/in.h>
#include <sys/socket.h>
#include <unistd.h>

#include <atomic>
#include <chrono>
#include <cstdlib>
#include <filesystem>
#include <memory>
#include <string>
#include <thread>
#include <vector>

#include "behaviortree_cpp_v3/bt_factory.h"
#include "gtest/gtest.h"
#include "nlohmann/json.hpp"
#include "opencv2/imgcodecs.hpp"
#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/image.hpp"
#include "std_msgs/msg/string.hpp"
#include "sura_bt/analyze_photo.hpp"
#include "sura_bt/camera_nodes.hpp"
#include "sura_bt/mission_failure_reasons.hpp"

namespace
{
using namespace std::chrono_literals;

class MockVisionServer
{
public:
  MockVisionServer()
  {
    fd_ = ::socket(AF_INET, SOCK_STREAM, 0);
    if (fd_ < 0) {return;}
    sockaddr_in address{};
    address.sin_family = AF_INET;
    address.sin_addr.s_addr = htonl(INADDR_LOOPBACK);
    address.sin_port = 0;
    if (::bind(fd_, reinterpret_cast<sockaddr *>(&address), sizeof(address)) != 0 ||
      ::listen(fd_, 4) != 0)
    {
      ::close(fd_);
      fd_ = -1;
      return;
    }
    socklen_t length = sizeof(address);
    ::getsockname(fd_, reinterpret_cast<sockaddr *>(&address), &length);
    port_ = ntohs(address.sin_port);
    thread_ = std::thread([this]() {serve();});
  }

  ~MockVisionServer()
  {
    if (fd_ >= 0) {
      ::shutdown(fd_, SHUT_RDWR);
      ::close(fd_);
    }
    if (thread_.joinable()) {thread_.join();}
  }

  bool available() const {return fd_ >= 0;}
  int port() const {return port_;}

private:
  void serve()
  {
    while (true) {
      const int client = ::accept(fd_, nullptr, nullptr);
      if (client < 0) {return;}
      std::string request;
      char chunk[4096];
      std::size_t expected = 0;
      while (true) {
        const auto count = ::recv(client, chunk, sizeof(chunk), 0);
        if (count <= 0) {break;}
        request.append(chunk, static_cast<std::size_t>(count));
        const auto header_end = request.find("\r\n\r\n");
        if (header_end == std::string::npos) {continue;}
        if (expected == 0) {
          const auto start = request.find("Content-Length: ");
          if (start != std::string::npos) {
            expected = std::stoul(request.substr(start + 16));
          }
        }
        if (request.size() >= header_end + 4 + expected) {break;}
      }
      std::string prompt;
      try {
        const auto body = nlohmann::json::parse(request.substr(request.find("\r\n\r\n") + 4));
        prompt = body.at("messages").at(1).at("content").at(0).at("text").get<std::string>();
      } catch (const std::exception &) {
        prompt = "invalid request";
      }
      if (prompt == "DELAY") {std::this_thread::sleep_for(300ms);}
      if (prompt == "DELAY_LONG") {std::this_thread::sleep_for(3s);}
      const auto json = nlohmann::json({{"choices", nlohmann::json::array({
        {{"message", {{"content", "Respuesta: " + prompt}}}}
      })}}).dump();
      const std::string response = "HTTP/1.1 200 OK\r\nContent-Type: application/json\r\n"
        "Content-Length: " + std::to_string(json.size()) + "\r\nConnection: close\r\n\r\n" + json;
      ::send(client, response.data(), response.size(), MSG_NOSIGNAL);
      ::close(client);
    }
  }

  int fd_{-1};
  int port_{0};
  std::thread thread_;
};

BT::NodeStatus tickUntilDone(
  BT::Tree & tree, const rclcpp::Node::SharedPtr & node,
  std::chrono::seconds max_duration = 3s)
{
  auto status = tree.tickRoot();
  const auto end = std::chrono::steady_clock::now() + max_duration;
  while (status == BT::NodeStatus::RUNNING && std::chrono::steady_clock::now() < end) {
    rclcpp::spin_some(node);
    std::this_thread::sleep_for(5ms);
    status = tree.tickRoot();
  }
  return status;
}

TEST(AnalyzePhoto, ConnectsTakePhotoToVisionAndPublishesAnswer)
{
  MockVisionServer server;
  if (!server.available()) {GTEST_SKIP() << "Loopback socket unavailable";}
  const std::string endpoint = "http://127.0.0.1:" + std::to_string(server.port()) +
    "/v1/chat/completions";
  ::setenv("SURA_VLM_ENDPOINT", endpoint.c_str(), 1);
  rclcpp::init(0, nullptr);
  auto node = std::make_shared<rclcpp::Node>("analyze_photo_integration_test");
  auto camera = node->create_publisher<sensor_msgs::msg::Image>(
    "/photo_test_robot/camera/image", rclcpp::SensorDataQoS());
  std::vector<std::string> events;
  auto output = node->create_subscription<std_msgs::msg::String>(
    "/sura_bt_runner/vision_answers", rclcpp::QoS(10).reliable().transient_local(),
    [&](std_msgs::msg::String::ConstSharedPtr message) {events.push_back(message->data);});
  auto blackboard = BT::Blackboard::create();
  blackboard->set("ros_node", node);
  BT::BehaviorTreeFactory factory;
  factory.registerNodeType<sura_bt::TakePhoto>("TakePhoto");
  factory.registerNodeType<sura_bt::AnalyzePhoto>("AnalyzePhoto");
  auto tree = factory.createTreeFromText(
    "<root main_tree_to_execute=\"Main\"><BehaviorTree ID=\"Main\"><Sequence>"
    "<TakePhoto robot_namespace=\"photo_test_robot\" topic=\"camera/image\" image_path=\"{photo_path}\"/>"
    "<AnalyzePhoto robot_namespace=\"photo_test_robot\" image_path=\"{photo_path}\" "
    "prompt=\"¿Qué ves?\" answer=\"{photo_answer}\"/>"
    "</Sequence></BehaviorTree></root>", blackboard);
  std::atomic<bool> publishing{true};
  std::thread publisher([&]() {
      sensor_msgs::msg::Image image;
      image.header.frame_id = "camera";
      image.height = 2;
      image.width = 2;
      image.encoding = "rgb8";
      image.step = 6;
      image.data.assign(12, 128);
      while (publishing.load()) {
        image.header.stamp = node->now();
        camera->publish(image);
        std::this_thread::sleep_for(10ms);
      }
    });
  EXPECT_EQ(tickUntilDone(tree, node), BT::NodeStatus::SUCCESS);
  publishing.store(false);
  publisher.join();
  std::string image_path;
  std::string answer;
  ASSERT_TRUE(blackboard->get("photo_path", image_path));
  ASSERT_TRUE(blackboard->get("photo_answer", answer));
  EXPECT_TRUE(std::filesystem::path(image_path).is_absolute());
  EXPECT_TRUE(std::filesystem::is_regular_file(image_path));
  EXPECT_EQ(answer, "Respuesta: ¿Qué ves?");
  for (int i = 0; i < 50 && events.empty(); ++i) {
    rclcpp::spin_some(node);
    std::this_thread::sleep_for(5ms);
  }
  ASSERT_FALSE(events.empty());
  EXPECT_EQ(nlohmann::json::parse(events.back()).at("answer"), answer);
  std::filesystem::remove(image_path);
  tree.haltTree();
  output.reset();
  camera.reset();
  node.reset();
  rclcpp::shutdown();
  ::unsetenv("SURA_VLM_ENDPOINT");
}

TEST(TakePhoto, ReportsMissingCameraImageToMissionProgress)
{
  rclcpp::init(0, nullptr);
  auto node = std::make_shared<rclcpp::Node>("take_photo_failure_reason_test");
  auto reasons = std::make_shared<sura_bt::MissionFailureReasons>();
  auto blackboard = BT::Blackboard::create();
  blackboard->set("ros_node", node);
  blackboard->set("mission_failure_reasons", reasons);
  BT::BehaviorTreeFactory factory;
  factory.registerNodeType<sura_bt::TakePhoto>("TakePhoto");
  auto tree = factory.createTreeFromText(
    "<root main_tree_to_execute=\"Main\"><BehaviorTree ID=\"Main\">"
    "<TakePhoto name=\"capture_timeout\" robot_namespace=\"sura_failure_reason_test\" "
    "topic=\"camera/no_image\" image_path=\"{photo_path}\"/>"
    "</BehaviorTree></root>", blackboard);

  EXPECT_EQ(tree.tickRoot(), BT::NodeStatus::FAILURE);
  const auto reason = reasons->take("capture_timeout");
  EXPECT_NE(reason.find("/sura_failure_reason_test/camera/no_image"), std::string::npos);
  EXPECT_NE(reason.find("2 segundos"), std::string::npos);
  EXPECT_TRUE(reasons->take("capture_timeout").empty());

  tree.haltTree();
  node.reset();
  rclcpp::shutdown();
}

TEST(AnalyzePhoto, DefaultPromptTimeoutAndHalt)
{
  MockVisionServer server;
  if (!server.available()) {GTEST_SKIP() << "Loopback socket unavailable";}
  const std::string endpoint = "http://127.0.0.1:" + std::to_string(server.port()) +
    "/v1/chat/completions";
  ::setenv("SURA_VLM_ENDPOINT", endpoint.c_str(), 1);
  rclcpp::init(0, nullptr);
  auto node = std::make_shared<rclcpp::Node>("analyze_photo_lifecycle_test");
  auto blackboard = BT::Blackboard::create();
  blackboard->set("ros_node", node);
  BT::BehaviorTreeFactory factory;
  factory.registerNodeType<sura_bt::AnalyzePhoto>("AnalyzePhoto");
  const auto path = (std::filesystem::temp_directory_path() / "sura_analyze_test.png").string();
  cv::imwrite(path, cv::Mat(4, 4, CV_8UC3, cv::Scalar(60, 80, 100)));
  const auto xml = [&](const std::string & attributes, const std::string & image_path) {
      return "<root main_tree_to_execute=\"Main\"><BehaviorTree ID=\"Main\">"
             "<AnalyzePhoto robot_namespace=\"robot\" image_path=\"" + image_path + "\" " +
             attributes + "/></BehaviorTree></root>";
    };
  auto missing = factory.createTreeFromText(xml("", "/tmp/missing_photo.png"), blackboard);
  EXPECT_EQ(missing.tickRoot(), BT::NodeStatus::FAILURE);
  auto default_prompt = factory.createTreeFromText(xml("answer=\"{default_answer}\"", path), blackboard);
  EXPECT_EQ(tickUntilDone(default_prompt, node), BT::NodeStatus::SUCCESS);
  std::string answer;
  ASSERT_TRUE(blackboard->get("default_answer", answer));
  EXPECT_NE(answer.find("Describe brevemente"), std::string::npos);
  auto timeout = factory.createTreeFromText(xml("prompt=\"DELAY\" timeout=\"0.05\"", path), blackboard);
  EXPECT_EQ(tickUntilDone(timeout, node), BT::NodeStatus::FAILURE);
  auto halted = factory.createTreeFromText(xml("prompt=\"DELAY\" answer=\"{halted_answer}\"", path), blackboard);
  EXPECT_EQ(halted.tickRoot(), BT::NodeStatus::RUNNING);
  halted.haltTree();
  const auto * halted_value = blackboard->getAny("halted_answer");
  EXPECT_TRUE(!halted_value || halted_value->empty());
  ::setenv("SURA_VLM_ENDPOINT", "http://127.0.0.1:1/v1/chat/completions", 1);
  auto unavailable = factory.createTreeFromText(xml("", path), blackboard);
  EXPECT_EQ(tickUntilDone(unavailable, node), BT::NodeStatus::FAILURE);
  std::filesystem::remove(path);
  node.reset();
  rclcpp::shutdown();
  ::unsetenv("SURA_VLM_ENDPOINT");
}

TEST(AnalyzePhoto, RealLocalQwenSmoke)
{
  const char * image = std::getenv("SURA_VLM_REAL_IMAGE");
  if (!image || !*image) {GTEST_SKIP() << "Set SURA_VLM_REAL_IMAGE for a real model test";}
  ::unsetenv("SURA_VLM_ENDPOINT");
  rclcpp::init(0, nullptr);
  auto node = std::make_shared<rclcpp::Node>("analyze_photo_real_model_test");
  auto blackboard = BT::Blackboard::create();
  blackboard->set("ros_node", node);
  BT::BehaviorTreeFactory factory;
  factory.registerNodeType<sura_bt::AnalyzePhoto>("AnalyzePhoto");
  auto tree = factory.createTreeFromText(
    "<root main_tree_to_execute=\"Main\"><BehaviorTree ID=\"Main\">"
    "<AnalyzePhoto robot_namespace=\"robot\" image_path=\"" + std::string(image) +
    "\" prompt=\"Describe brevemente qué ves en esta foto.\" answer=\"{answer}\"/>"
    "</BehaviorTree></root>", blackboard);
  EXPECT_EQ(tickUntilDone(tree, node, 180s), BT::NodeStatus::SUCCESS);
  std::string answer;
  ASSERT_TRUE(blackboard->get("answer", answer));
  EXPECT_FALSE(answer.empty());
  node.reset();
  rclcpp::shutdown();
}

TEST(AnalyzePhoto, HaltingDoesNotWaitForSlowInference)
{
  MockVisionServer server;
  if (!server.available()) {GTEST_SKIP() << "Loopback socket unavailable";}
  const std::string endpoint = "http://127.0.0.1:" + std::to_string(server.port()) +
    "/v1/chat/completions";
  ::setenv("SURA_VLM_ENDPOINT", endpoint.c_str(), 1);
  rclcpp::init(0, nullptr);
  auto node = std::make_shared<rclcpp::Node>("analyze_photo_cancel_test");
  auto blackboard = BT::Blackboard::create();
  blackboard->set("ros_node", node);
  BT::BehaviorTreeFactory factory;
  factory.registerNodeType<sura_bt::AnalyzePhoto>("AnalyzePhoto");
  const auto path = (std::filesystem::temp_directory_path() / "sura_cancel_test.png").string();
  cv::imwrite(path, cv::Mat(4, 4, CV_8UC3, cv::Scalar(60, 80, 100)));
  auto tree = factory.createTreeFromText(
    "<root main_tree_to_execute=\"Main\"><BehaviorTree ID=\"Main\">"
    "<AnalyzePhoto robot_namespace=\"robot\" image_path=\"" + path +
    "\" prompt=\"DELAY_LONG\" answer=\"{answer}\"/>"
    "</BehaviorTree></root>", blackboard);
  EXPECT_EQ(tree.tickRoot(), BT::NodeStatus::RUNNING);
  std::this_thread::sleep_for(100ms);
  const auto started = std::chrono::steady_clock::now();
  tree.haltTree();
  const auto elapsed = std::chrono::steady_clock::now() - started;
  EXPECT_LT(elapsed, 2500ms);
  const auto * answer = blackboard->getAny("answer");
  EXPECT_TRUE(!answer || answer->empty());
  std::filesystem::remove(path);
  node.reset();
  rclcpp::shutdown();
  ::unsetenv("SURA_VLM_ENDPOINT");
}
}  // namespace
