/**
 * @file policy_profile_manager.cpp
 * @brief Multi-policy profile loading and validation.
 */

#include "policy_profile_manager.h"

#include <cmath>
#include <filesystem>
#include <stdexcept>
#include <utility>

#include <yaml-cpp/yaml.h>

namespace deploy {
namespace {

void require_node(const YAML::Node &node, const std::string &key,
                  const std::string &source) {
  if (!node[key]) {
    throw std::runtime_error("Missing key '" + key + "' in " + source);
  }
}

void require_existing_file(const std::string &path, const std::string &label) {
  std::error_code ec;
  if (!std::filesystem::exists(path, ec) ||
      !std::filesystem::is_regular_file(path, ec)) {
    throw std::runtime_error(label + " does not exist: " + path);
  }
}

JoyAxisTrigger parse_joy_axis_trigger(const YAML::Node &node,
                                      const std::string &source) {
  require_node(node, "index", source);
  require_node(node, "value", source);

  JoyAxisTrigger trigger;
  trigger.axis = node["index"].as<int>();
  trigger.value = node["value"].as<float>();
  if (node["tolerance"]) {
    trigger.tolerance = node["tolerance"].as<float>();
  }
  return trigger;
}

std::string joy_axis_label(const JoyAxisTrigger &trigger) {
  return "axis " + std::to_string(trigger.axis) + " value " +
         std::to_string(trigger.value);
}

float canonical_joy_axis_value(float value) {
  if (std::fabs(value) > 1.0f) {
    return std::copysign(1.0f, value);
  }
  return value;
}

} // namespace

void PolicyProfileManager::clear() {
  multi_profile_enabled_ = false;
  initial_profile_id_.clear();
  profiles_.clear();
  id_to_index_.clear();
  button_to_id_.clear();
}

void PolicyProfileManager::add_profile(PolicyProfile profile) {
  if (profile.id.empty()) {
    throw std::runtime_error("Policy profile id cannot be empty");
  }
  if (id_to_index_.find(profile.id) != id_to_index_.end()) {
    throw std::runtime_error("Duplicate policy profile id: " + profile.id);
  }
  if (profile.joy_button < -1) {
    throw std::runtime_error("Policy profile '" + profile.id +
                             "' joy_button must be >= -1");
  }
  if (profile.joy_button >= 0 &&
      button_to_id_.find(profile.joy_button) != button_to_id_.end()) {
    throw std::runtime_error(
        "Duplicate policy profile joy_button " +
        std::to_string(profile.joy_button) + " for profiles '" +
        button_to_id_[profile.joy_button] + "' and '" + profile.id + "'");
  }
  if (profile.joy_axis) {
    if (profile.joy_axis->axis < 0) {
      throw std::runtime_error("Policy profile '" + profile.id +
                               "' joy_axis.index must be >= 0");
    }
    if (profile.joy_axis->tolerance < -1.0f) {
      throw std::runtime_error("Policy profile '" + profile.id +
                               "' joy_axis.tolerance must be >= -1");
    }
    for (const auto &existing : profiles_) {
      if (existing.joy_axis &&
          existing.joy_axis->axis == profile.joy_axis->axis &&
          std::fabs(canonical_joy_axis_value(existing.joy_axis->value) -
                    canonical_joy_axis_value(profile.joy_axis->value)) <
              1e-4f) {
        throw std::runtime_error(
            "Duplicate policy profile joy_axis " +
            joy_axis_label(*profile.joy_axis) + " for profiles '" +
            existing.id + "' and '" + profile.id + "'");
      }
    }
  }

  const size_t idx = profiles_.size();
  id_to_index_[profile.id] = idx;
  if (profile.joy_button >= 0) {
    button_to_id_[profile.joy_button] = profile.id;
  }
  profiles_.push_back(std::move(profile));
}

void PolicyProfileManager::load_single_profile(const std::string &config_file) {
  clear();

  PolicyProfile profile;
  profile.id = "single";
  profile.name = "single";
  profile.config_file = config_file;
  profile.config = load_robot_runtime_config(config_file);
  profile.runner =
      std::make_unique<PolicyRunner>(profile.config.policy_path,
                                     profile.config);

  initial_profile_id_ = profile.id;
  add_profile(std::move(profile));
}

void PolicyProfileManager::load_from_file(const std::string &profiles_file) {
  clear();
  require_existing_file(profiles_file, "policy_profiles_file");

  YAML::Node root = YAML::LoadFile(profiles_file);
  require_node(root, "initial_profile", profiles_file);
  require_node(root, "profiles", profiles_file);
  if (!root["profiles"].IsSequence() || root["profiles"].size() == 0) {
    throw std::runtime_error("profiles must be a non-empty list in " +
                             profiles_file);
  }

  initial_profile_id_ = root["initial_profile"].as<std::string>();
  RobotRuntimeConfig base_config;
  std::string base_id;
  bool has_base = false;

  for (const auto &node : root["profiles"]) {
    require_node(node, "id", profiles_file);
    require_node(node, "config_file", profiles_file);
    if (!node["joy_button"] && !node["joy_axis"]) {
      throw std::runtime_error(
          "Each profile in " + profiles_file +
          " must define either joy_button or joy_axis");
    }

    PolicyProfile profile;
    profile.id = node["id"].as<std::string>();
    profile.name = node["name"] ? node["name"].as<std::string>() : profile.id;
    profile.config_file =
        resolve_path_from_file(profiles_file,
                               node["config_file"].as<std::string>());
    if (node["joy_button"]) {
      profile.joy_button = node["joy_button"].as<int>();
    }
    if (node["joy_axis"]) {
      profile.joy_axis =
          parse_joy_axis_trigger(node["joy_axis"], profiles_file);
    }

    require_existing_file(profile.config_file,
                          "config_file for profile '" + profile.id + "'");
    profile.config = load_robot_runtime_config(profile.config_file);
    require_existing_file(profile.config.policy_path,
                          "policy_path for profile '" + profile.id + "'");

    if (!has_base) {
      base_config = profile.config;
      base_id = profile.id;
      has_base = true;
    } else {
      validate_robot_runtime_compatible(base_config, profile.config, base_id,
                                        profile.id);
    }

    profile.runner =
        std::make_unique<PolicyRunner>(profile.config.policy_path,
                                       profile.config);
    add_profile(std::move(profile));
  }

  if (id_to_index_.find(initial_profile_id_) == id_to_index_.end()) {
    throw std::runtime_error("initial_profile '" + initial_profile_id_ +
                             "' is not listed in profiles");
  }
  multi_profile_enabled_ = true;
}

PolicyProfile &PolicyProfileManager::initial_profile() {
  auto *profile = profile_by_id(initial_profile_id_);
  if (!profile) {
    throw std::runtime_error("Initial policy profile is unavailable: " +
                             initial_profile_id_);
  }
  return *profile;
}

PolicyProfile *PolicyProfileManager::profile_by_id(
    const std::string &profile_id) {
  auto it = id_to_index_.find(profile_id);
  if (it == id_to_index_.end()) {
    return nullptr;
  }
  return &profiles_[it->second];
}

const PolicyProfile *PolicyProfileManager::profile_by_id(
    const std::string &profile_id) const {
  auto it = id_to_index_.find(profile_id);
  if (it == id_to_index_.end()) {
    return nullptr;
  }
  return &profiles_[it->second];
}

PolicyProfile *PolicyProfileManager::profile_for_button(int joy_button) {
  auto it = button_to_id_.find(joy_button);
  if (it == button_to_id_.end()) {
    return nullptr;
  }
  return profile_by_id(it->second);
}

} // namespace deploy
