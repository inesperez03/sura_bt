#pragma once

#include <mutex>
#include <string>
#include <unordered_map>
#include <utility>

#include "behaviortree_cpp_v3/action_node.h"

namespace sura_bt
{

// Shared between mission action nodes and MissionProgress. A reason is consumed
// when the corresponding action reports FAILURE.
class MissionFailureReasons
{
public:
  void record(const std::string & action, const std::string & reason)
  {
    std::lock_guard<std::mutex> lock(mutex_);
    reasons_[action] = reason;
  }

  std::string take(const std::string & action)
  {
    std::lock_guard<std::mutex> lock(mutex_);
    const auto found = reasons_.find(action);
    if (found == reasons_.end()) {return {};}
    std::string reason = std::move(found->second);
    reasons_.erase(found);
    return reason;
  }

  void clear()
  {
    std::lock_guard<std::mutex> lock(mutex_);
    reasons_.clear();
  }

private:
  std::mutex mutex_;
  std::unordered_map<std::string, std::string> reasons_;
};

inline void recordMissionFailure(
  const BT::NodeConfiguration & config,
  const std::string & action,
  const std::string & reason)
{
  std::shared_ptr<MissionFailureReasons> reasons;
  if (config.blackboard->get("mission_failure_reasons", reasons) && reasons)
  {
    reasons->record(action, reason);
  }
}

}  // namespace sura_bt
