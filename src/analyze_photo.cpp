#include "sura_bt/analyze_photo.hpp"
#include "sura_bt/mission_failure_reasons.hpp"

#include <curl/curl.h>

#include <algorithm>
#include <cmath>
#include <cstdlib>
#include <filesystem>
#include <fstream>
#include <cstdlib>
#include <mutex>
#include <signal.h>
#include <stdexcept>
#include <sys/types.h>
#include <sys/wait.h>
#include <thread>
#include <unistd.h>
#include <utility>
#include <vector>

#include "ament_index_cpp/get_package_prefix.hpp"
#include "nlohmann/json.hpp"
#include "opencv2/imgcodecs.hpp"
#include "opencv2/imgproc.hpp"

namespace sura_bt
{
namespace
{
constexpr const char * kEndpoint = "http://127.0.0.1:8080/v1/chat/completions";
constexpr const char * kModel = "Qwen3-VL-2B-Instruct";
constexpr const char * kDefaultPrompt = "Briefly describe what appears in this image.";
constexpr std::uintmax_t kMaxFileBytes = 16U * 1024U * 1024U;
constexpr auto kServerStartupTimeout = std::chrono::seconds(180);
std::mutex server_process_mutex;
pid_t owned_server_pid = -1;

void stopOwnedServer()
{
  std::lock_guard<std::mutex> lock(server_process_mutex);
  if (owned_server_pid > 0) {
    kill(owned_server_pid, SIGTERM);
    waitpid(owned_server_pid, nullptr, 0);
    owned_server_pid = -1;
  }
}

std::string localEndpoint()
{
  const char * configured = std::getenv("SURA_VLM_ENDPOINT");
  if (!configured || !*configured) {return kEndpoint;}
  const std::string endpoint(configured);
  if (endpoint.rfind("http://127.0.0.1:", 0) != 0 ||
    endpoint.find("/v1/chat/completions") == std::string::npos)
  {
    throw std::runtime_error("SURA_VLM_ENDPOINT must be a loopback chat endpoint");
  }
  return endpoint;
}

std::pair<std::string, std::string> serverAddress(const std::string & endpoint)
{
  constexpr const char * prefix = "http://127.0.0.1:";
  const auto port_start = std::string(prefix).size();
  const auto port_end = endpoint.find('/', port_start);
  if (endpoint.rfind(prefix, 0) != 0 || port_end == std::string::npos || port_end == port_start) {
    throw std::runtime_error("SURA_VLM_ENDPOINT must include a valid local port.");
  }
  const std::string port = endpoint.substr(port_start, port_end - port_start);
  const auto health_path = endpoint.substr(0, port_end) + "/health";
  return {health_path, port};
}

bool serverReady(const std::string & health_endpoint)
{
  CURL * handle = curl_easy_init();
  if (!handle) {return false;}
  curl_easy_setopt(handle, CURLOPT_URL, health_endpoint.c_str());
  curl_easy_setopt(handle, CURLOPT_CONNECTTIMEOUT_MS, 500L);
  curl_easy_setopt(handle, CURLOPT_TIMEOUT_MS, 1000L);
  curl_easy_setopt(handle, CURLOPT_NOSIGNAL, 1L);
  curl_easy_setopt(handle, CURLOPT_WRITEFUNCTION,
    +[](char *, std::size_t size, std::size_t count, void *) -> std::size_t {
      return size * count;
    });
  const CURLcode result = curl_easy_perform(handle);
  long status = 0;
  curl_easy_getinfo(handle, CURLINFO_RESPONSE_CODE, &status);
  curl_easy_cleanup(handle);
  return result == CURLE_OK && status == 200;
}

void startOwnedServer(const std::string & port)
{
  std::lock_guard<std::mutex> lock(server_process_mutex);
  if (owned_server_pid > 0) {
    const pid_t result = waitpid(owned_server_pid, nullptr, WNOHANG);
    if (result == 0) {return;}
    owned_server_pid = -1;
  }

  const auto prefix = std::filesystem::path(ament_index_cpp::get_package_prefix("sura_bt"));
  const auto script = prefix / "lib" / "sura_bt" / "start_qwen_vl_cpu.sh";
  if (!std::filesystem::is_regular_file(script)) {
    throw std::runtime_error("The Qwen launcher was not found at " + script.string());
  }
  const auto workspace = prefix.parent_path().parent_path().string();
  const pid_t child = fork();
  if (child < 0) {throw std::runtime_error("Could not start the local Qwen process.");}
  if (child == 0) {
    const std::string workspace_env = "SURA_WS_META=" + workspace;
    const std::string port_env = "SURA_VLM_PORT=" + port;
    execl("/usr/bin/env", "env", workspace_env.c_str(), port_env.c_str(),
      script.c_str(), static_cast<char *>(nullptr));
    _exit(127);
  }
  owned_server_pid = child;
  std::atexit(stopOwnedServer);
}

void ensureServerReady(const std::string & endpoint, const std::atomic<bool> & cancel)
{
  const auto address = serverAddress(endpoint);
  if (serverReady(address.first)) {return;}
  startOwnedServer(address.second);

  const auto deadline = std::chrono::steady_clock::now() + kServerStartupTimeout;
  while (std::chrono::steady_clock::now() < deadline) {
    if (cancel.load()) {throw std::runtime_error("Waiting for the image analysis server was cancelled.");}
    if (serverReady(address.first)) {return;}
    {
      std::lock_guard<std::mutex> lock(server_process_mutex);
      if (owned_server_pid > 0 && waitpid(owned_server_pid, nullptr, WNOHANG) == owned_server_pid) {
        owned_server_pid = -1;
        throw std::runtime_error(
                "Qwen could not start. Check that llama-server and the model files are available.");
      }
    }
    std::this_thread::sleep_for(std::chrono::milliseconds(500));
  }
  throw std::runtime_error(
          "Qwen did not become ready within 180 seconds. Check llama-server output and the local model.");
}

std::string encodeBase64(const std::vector<unsigned char> & bytes)
{
  static constexpr char alphabet[] =
    "ABCDEFGHIJKLMNOPQRSTUVWXYZabcdefghijklmnopqrstuvwxyz0123456789+/";
  std::string result;
  result.reserve((bytes.size() + 2) / 3 * 4);
  for (std::size_t i = 0; i < bytes.size(); i += 3) {
    const unsigned first = bytes[i];
    const unsigned second = i + 1 < bytes.size() ? bytes[i + 1] : 0;
    const unsigned third = i + 2 < bytes.size() ? bytes[i + 2] : 0;
    result.push_back(alphabet[first >> 2]);
    result.push_back(alphabet[((first & 3U) << 4) | (second >> 4)]);
    result.push_back(i + 1 < bytes.size() ? alphabet[((second & 15U) << 2) | (third >> 6)] : '=');
    result.push_back(i + 2 < bytes.size() ? alphabet[third & 63U] : '=');
  }
  return result;
}

std::vector<unsigned char> prepareImage(const std::string & path)
{
  namespace fs = std::filesystem;
  if (!fs::is_regular_file(path) || fs::file_size(path) == 0 ||
    fs::file_size(path) > kMaxFileBytes)
  {
    throw std::runtime_error("Image file is missing, empty, or larger than 16 MiB");
  }
  auto image = cv::imread(path, cv::IMREAD_COLOR);
  if (image.empty()) {throw std::runtime_error("Image cannot be decoded");}
  const int longest = std::max(image.cols, image.rows);
  if (longest > 1024) {
    const double scale = 1024.0 / longest;
    cv::resize(image, image, cv::Size(), scale, scale, cv::INTER_AREA);
  }
  std::vector<unsigned char> jpeg;
  if (!cv::imencode(".jpg", image, jpeg, {cv::IMWRITE_JPEG_QUALITY, 85})) {
    throw std::runtime_error("Image cannot be encoded for inference");
  }
  return jpeg;
}

std::size_t appendResponse(char * data, std::size_t size, std::size_t count, void * output)
{
  auto * response = static_cast<std::string *>(output);
  const auto bytes = size * count;
  if (response->size() + bytes > 2U * 1024U * 1024U) {return 0;}
  response->append(data, bytes);
  return bytes;
}

std::string queryModel(
  const std::vector<unsigned char> & jpeg, const std::string & prompt,
  double timeout, const std::atomic<bool> & cancel)
{
  const nlohmann::json request = {
    {"model", kModel},
    {"stream", false},
    {"temperature", 0.1},
    {"max_tokens", 256},
    {"messages", nlohmann::json::array({
      {{"role", "system"}, {"content", "Always answer in English about the image. "
          "If unsure, say so."}},
      {{"role", "user"}, {"content", nlohmann::json::array({
        {{"type", "text"}, {"text", prompt}},
        {{"type", "image_url"}, {"image_url", {{"url", "data:image/jpeg;base64," + encodeBase64(jpeg)}}}}
      })}}
    })}
  };
  const std::string body = request.dump();
  const std::string endpoint = localEndpoint();
  CURL * handle = curl_easy_init();
  if (!handle) {throw std::runtime_error("Cannot initialize HTTP client");}
  curl_slist * headers = curl_slist_append(nullptr, "Content-Type: application/json");
  std::string response;
  curl_easy_setopt(handle, CURLOPT_URL, endpoint.c_str());
  curl_easy_setopt(handle, CURLOPT_HTTPHEADER, headers);
  curl_easy_setopt(handle, CURLOPT_POSTFIELDS, body.c_str());
  curl_easy_setopt(handle, CURLOPT_POSTFIELDSIZE_LARGE, static_cast<curl_off_t>(body.size()));
  curl_easy_setopt(handle, CURLOPT_CONNECTTIMEOUT_MS, 2000L);
  curl_easy_setopt(handle, CURLOPT_TIMEOUT_MS,
    std::max(1L, static_cast<long>(std::ceil(timeout * 1000))));
  curl_easy_setopt(handle, CURLOPT_WRITEFUNCTION, appendResponse);
  curl_easy_setopt(handle, CURLOPT_WRITEDATA, &response);
  curl_easy_setopt(handle, CURLOPT_NOPROGRESS, 0L);
  curl_easy_setopt(handle, CURLOPT_XFERINFOFUNCTION,
    +[](void * state, curl_off_t, curl_off_t, curl_off_t, curl_off_t) -> int {
      return static_cast<const std::atomic<bool> *>(state)->load() ? 1 : 0;
    });
  curl_easy_setopt(handle, CURLOPT_XFERINFODATA, &cancel);
  const CURLcode result = curl_easy_perform(handle);
  long status = 0;
  curl_easy_getinfo(handle, CURLINFO_RESPONSE_CODE, &status);
  curl_slist_free_all(headers);
  curl_easy_cleanup(handle);
  if (cancel.load()) {throw std::runtime_error("Image analysis cancelled");}
  if (result == CURLE_COULDNT_CONNECT) {
    throw std::runtime_error("Could not connect to the image analysis server.");
  }
  if (result == CURLE_OPERATION_TIMEDOUT) {
    throw std::runtime_error("The image analysis server took too long to respond.");
  }
  if (result != CURLE_OK) {
    throw std::runtime_error(
            std::string("Communication with the image analysis server failed: ") +
            curl_easy_strerror(result));
  }
  if (status != 200) {
    throw std::runtime_error(
            "The image analysis server returned HTTP status " + std::to_string(status) + ".");
  }
  try {
    const auto parsed = nlohmann::json::parse(response);
    const auto answer = parsed.at("choices").at(0).at("message").at("content").get<std::string>();
    if (answer.find_first_not_of(" \t\r\n") == std::string::npos) {
      throw std::runtime_error("The image analysis server returned an empty response.");
    }
    return answer;
  } catch (const nlohmann::json::exception &) {
    throw std::runtime_error("The image analysis server returned an invalid response.");
  }
}
}  // namespace

AnalyzePhoto::AnalyzePhoto(const std::string & name, const BT::NodeConfiguration & config)
: BT::StatefulActionNode(name, config)
{
  node_ = config.blackboard->get<rclcpp::Node::SharedPtr>("ros_node");
}

AnalyzePhoto::~AnalyzePhoto()
{
  stopRequest();
}

BT::PortsList AnalyzePhoto::providedPorts()
{
  return {
    BT::InputPort<std::string>("robot_namespace", "Robot whose camera took the photo."),
    BT::InputPort<std::string>("image_path", "Absolute path to the PNG saved by TakePhoto."),
    BT::InputPort<std::string>("prompt", kDefaultPrompt,
      "Question about the image; defaults to a short description."),
    BT::InputPort<double>("timeout", 180.0,
      "Maximum seconds allowed for local image inference."),
    BT::OutputPort<std::string>("answer", "Model answer about the image, set only on success.")
  };
}

const char * AnalyzePhoto::main_description()
{
  return "Analyzes a TakePhoto image with a local CPU Qwen VL model. Answers the prompt, "
    "writes the answer to the BT output, and displays it in Mission Assistant.";
}

BT::NodeStatus AnalyzePhoto::onStart()
{
  stopRequest();
  const auto robot = getInput<std::string>("robot_namespace");
  const auto image = getInput<std::string>("image_path");
  const auto prompt = getInput<std::string>("prompt");
  const double timeout = getInput<double>("timeout").value_or(180.0);
  if (!robot || robot->empty() || !image || image->empty() || !prompt || prompt->empty() ||
    !std::isfinite(timeout) || timeout <= 0.0 || timeout > 3600.0)
  {
    const std::string reason =
      "Could not start image analysis: provide a robot, an image, a question, and a valid timeout.";
    recordMissionFailure(config(), name(), reason);
    RCLCPP_ERROR(node_->get_logger(), "[sura_bt] AnalyzePhoto: %s", reason.c_str());
    return BT::NodeStatus::FAILURE;
  }
  robot_ = robot.value();
  image_path_ = image.value();
  prompt_ = prompt.value();
  std::vector<unsigned char> jpeg;
  try {
    jpeg = prepareImage(image_path_);
  } catch (const std::exception & error) {
    const std::string reason = std::string("Could not prepare the image for analysis: ") + error.what();
    recordMissionFailure(config(), name(), reason);
    RCLCPP_ERROR(node_->get_logger(), "[sura_bt] AnalyzePhoto: %s", reason.c_str());
    return BT::NodeStatus::FAILURE;
  }
  event_pub_ = node_->create_publisher<std_msgs::msg::String>(
    "/sura_bt_runner/vision_answers", rclcpp::QoS(10).reliable().transient_local());
  request_ = std::make_shared<RequestState>();
  const auto state = request_;
  const auto question = prompt_;
  worker_ = std::thread([state, question, jpeg = std::move(jpeg), timeout]() {
      try {
        ensureServerReady(localEndpoint(), state->cancel);
        auto answer = queryModel(jpeg, question, timeout, state->cancel);
        std::lock_guard<std::mutex> lock(state->mutex);
        state->answer = std::move(answer);
      } catch (const std::exception & error) {
        std::lock_guard<std::mutex> lock(state->mutex);
        state->error = error.what();
      }
      state->done.store(true);
    });
  return BT::NodeStatus::RUNNING;
}

BT::NodeStatus AnalyzePhoto::onRunning()
{
  if (!request_ || !request_->done.load()) {return BT::NodeStatus::RUNNING;}
  if (worker_.joinable()) {worker_.join();}
  std::string answer;
  std::string error;
  {
    std::lock_guard<std::mutex> lock(request_->mutex);
    answer = request_->answer;
    error = request_->error;
  }
  request_.reset();
  if (!error.empty()) {
    const std::string reason = std::string("Could not analyze the image. ") + error;
    recordMissionFailure(config(), name(), reason);
    RCLCPP_ERROR(node_->get_logger(), "[sura_bt] AnalyzePhoto: %s", error.c_str());
    publishEvent("error", reason);
    return BT::NodeStatus::FAILURE;
  }
  setOutput("answer", answer);
  publishEvent("success", answer);
  return BT::NodeStatus::SUCCESS;
}

void AnalyzePhoto::onHalted()
{
  stopRequest();
}

void AnalyzePhoto::stopRequest()
{
  if (request_) {request_->cancel.store(true);}
  if (worker_.joinable()) {worker_.join();}
  request_.reset();
}

void AnalyzePhoto::publishEvent(const std::string & status, const std::string & message)
{
  if (!event_pub_) {return;}
  std_msgs::msg::String event;
  event.data = nlohmann::json({
    {"status", status}, {"robot", robot_}, {"image_path", image_path_},
    {"prompt", prompt_}, {"answer", status == "success" ? message : ""},
    {"error", status == "error" ? message : ""}
  }).dump();
  event_pub_->publish(event);
}

}  // namespace sura_bt
