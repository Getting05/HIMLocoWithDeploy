/**
 * @file policy_profile_manager.h
 * @brief Multi-policy profile loading and lookup for obstacle-course deploy.
 */
#pragma once

#include <memory>
#include <optional>
#include <string>
#include <unordered_map>
#include <vector>

#include "policy_runner.h"
#include "robot_runtime_config.h"

namespace deploy {

struct JoyAxisTrigger {
  int axis = -1;
  float value = 0.0f;
  float tolerance = -1.0f;
};

struct PolicyProfile {
  std::string id;
  std::string name;
  std::string config_file;
  int joy_button = -1;
  std::optional<JoyAxisTrigger> joy_axis;
  RobotRuntimeConfig config;
  std::unique_ptr<PolicyRunner> runner;
};

class PolicyProfileManager {
public:
  PolicyProfileManager() = default;

  void load_from_file(const std::string &profiles_file);
  void load_single_profile(const std::string &config_file);

  bool multi_profile_enabled() const { return multi_profile_enabled_; }
  const std::string &initial_profile_id() const { return initial_profile_id_; }

  PolicyProfile &initial_profile();
  PolicyProfile *profile_by_id(const std::string &profile_id);
  const PolicyProfile *profile_by_id(const std::string &profile_id) const;
  PolicyProfile *profile_for_button(int joy_button);
  const std::vector<PolicyProfile> &profiles() const { return profiles_; }

private:
  void clear();
  void add_profile(PolicyProfile profile);

  bool multi_profile_enabled_ = false;
  std::string initial_profile_id_;
  std::vector<PolicyProfile> profiles_;
  std::unordered_map<std::string, size_t> id_to_index_;
  std::unordered_map<int, std::string> button_to_id_;
};

} // namespace deploy
