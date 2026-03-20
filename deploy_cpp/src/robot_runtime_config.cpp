#include "robot_runtime_config.h"

#include <stdexcept>

#include <yaml-cpp/yaml.h>

namespace deploy {
namespace {

template <typename T, size_t N>
std::array<T, N> parse_required_array(const YAML::Node &root,
                                      const std::string &key) {
  const YAML::Node n = root[key];
  if (!n || !n.IsSequence() || n.size() != N) {
    throw std::runtime_error("Invalid or missing array key: " + key);
  }
  std::array<T, N> out{};
  for (size_t i = 0; i < N; ++i) {
    out[i] = n[i].as<T>();
  }
  return out;
}

template <typename T, size_t N>
std::array<T, N> parse_scalar_or_array(const YAML::Node &root,
                                       const std::string &key,
                                       T default_value) {
  const YAML::Node n = root[key];
  std::array<T, N> out{};
  out.fill(default_value);
  if (!n) {
    return out;
  }
  if (n.IsSequence()) {
    if (n.size() != N) {
      throw std::runtime_error("Invalid array length for key: " + key);
    }
    for (size_t i = 0; i < N; ++i) {
      out[i] = n[i].as<T>();
    }
    return out;
  }
  const T v = n.as<T>();
  out.fill(v);
  return out;
}

void validate_positive_array(const std::array<float, NUM_JOINTS> &vals,
                             const std::string &name) {
  for (size_t i = 0; i < vals.size(); ++i) {
    if (vals[i] <= 0.0f) {
      throw std::runtime_error(name + " has non-positive value at index " +
                               std::to_string(i));
    }
  }
}

} // namespace

RobotRuntimeConfig default_robot_runtime_config() {
  RobotRuntimeConfig cfg;

  for (int i = 0; i < NUM_JOINTS; ++i) {
    cfg.fixed_kp[i] = cfg.kp_joint;
    cfg.fixed_kd[i] = 3.0f;
    cfg.torque_limits[i] = 23.5f;
    cfg.joint_names[i] = JOINT_NAMES[i];
    cfg.joint_controller_names[i] = std::string(JOINT_NAMES[i]) + "_controller";
    cfg.joint_mapping[i] = i;
    cfg.dof_pos_scale[i] = 1.0f;
    cfg.dof_vel_scale[i] = 0.08f;
    cfg.action_scale[i] = 0.15f;
    cfg.joint_transmission_ratio[i] = GEAR_RATIO;
  }

  // Calves include additional 7/3 reduction from joint side to motor side.
  cfg.joint_transmission_ratio[2] = GEAR_RATIO * (7.0f / 3.0f);
  cfg.joint_transmission_ratio[5] = GEAR_RATIO * (7.0f / 3.0f);
  cfg.joint_transmission_ratio[8] = GEAR_RATIO * (7.0f / 3.0f);
  cfg.joint_transmission_ratio[11] = GEAR_RATIO * (7.0f / 3.0f);

  cfg.robot_name = "mybot";
  cfg.urdf_relpath = "robot/mybot/urdf/mybot.urdf";
  cfg.mujoco_xml_relpath = "robot/mybot/xml/mybot.xml";
  cfg.isaac_xml_relpath = "robot/mybot/xml/mybot.xml";

  return cfg;
}

RobotRuntimeConfig load_robot_runtime_config(const std::string &yaml_file) {
  if (yaml_file.empty()) {
    throw std::runtime_error("robot_config_file is required");
  }

  YAML::Node root = YAML::LoadFile(yaml_file);
  RobotRuntimeConfig cfg = default_robot_runtime_config();

  cfg.num_of_dofs = root["num_of_dofs"].as<int>();
  if (cfg.num_of_dofs != NUM_JOINTS) {
    throw std::runtime_error("num_of_dofs must be 12 for this build");
  }

  cfg.dt = root["dt"].as<float>();
  cfg.decimation = root["decimation"].as<int>();

  cfg.fixed_kp = parse_required_array<float, NUM_JOINTS>(root, "fixed_kp");
  cfg.fixed_kd = parse_required_array<float, NUM_JOINTS>(root, "fixed_kd");
  cfg.torque_limits =
      parse_required_array<float, NUM_JOINTS>(root, "torque_limits");

  cfg.default_dof_pos =
      parse_required_array<float, NUM_JOINTS>(root, "default_dof_pos");
  cfg.standup_target_pos =
      parse_required_array<float, NUM_JOINTS>(root, "standup_target_pos");
  cfg.policy_dof_pos =
      parse_required_array<float, NUM_JOINTS>(root, "policy_dof_pos");

  cfg.joint_pos_lower =
      parse_required_array<float, NUM_JOINTS>(root, "joint_pos_lower");
  cfg.joint_pos_upper =
      parse_required_array<float, NUM_JOINTS>(root, "joint_pos_upper");

  const YAML::Node names = root["joint_names"];
  const YAML::Node ctrl_names = root["joint_controller_names"];
  const YAML::Node mapping = root["joint_mapping"];
  if (!names || !names.IsSequence() || names.size() != NUM_JOINTS) {
    throw std::runtime_error("joint_names must be an array of size 12");
  }
  if (!ctrl_names || !ctrl_names.IsSequence() ||
      ctrl_names.size() != NUM_JOINTS) {
    throw std::runtime_error(
        "joint_controller_names must be an array of size 12");
  }
  if (!mapping || !mapping.IsSequence() || mapping.size() != NUM_JOINTS) {
    throw std::runtime_error("joint_mapping must be an array of size 12");
  }
  for (int i = 0; i < NUM_JOINTS; ++i) {
    cfg.joint_names[i] = names[i].as<std::string>();
    cfg.joint_controller_names[i] = ctrl_names[i].as<std::string>();
    cfg.joint_mapping[i] = mapping[i].as<int>();
  }

  const YAML::Node wheels = root["wheel_indices"];
  cfg.wheel_indices.clear();
  if (wheels && wheels.IsSequence()) {
    for (const auto &w : wheels) {
      cfg.wheel_indices.push_back(w.as<int>());
    }
  }

  cfg.joint_transmission_ratio =
      parse_required_array<float, NUM_JOINTS>(root, "joint_transmission_ratio");
  validate_positive_array(cfg.joint_transmission_ratio,
                          "joint_transmission_ratio");

  cfg.device = root["device"].as<std::string>();
  cfg.port0 = root["port0"].as<std::string>();
  cfg.port1 = root["port1"].as<std::string>();
  cfg.imu_topic = root["imu_topic"].as<std::string>();
  cfg.robot_name = root["robot_name"].as<std::string>();
  cfg.urdf_relpath = root["urdf_relpath"].as<std::string>();
  cfg.mujoco_xml_relpath = root["mujoco_xml_relpath"].as<std::string>();
  cfg.isaac_xml_relpath = root["isaac_xml_relpath"].as<std::string>();
  if (root["policy_path"]) {
    cfg.policy_path = root["policy_path"].as<std::string>();
  }

  cfg.kp_joint = root["kp_joint"].as<float>();
  cfg.kd_joint = root["kd_joint"].as<float>();
  cfg.kd_damp_motor = root["kd_damp_motor"].as<float>();

  cfg.lin_vel_scale = root["lin_vel_scale"].as<float>();
  cfg.ang_vel_scale = root["ang_vel_scale"].as<float>();
  cfg.dof_pos_scale =
      parse_scalar_or_array<float, NUM_JOINTS>(root, "dof_pos_scale", 1.0f);
  cfg.dof_vel_scale =
      parse_scalar_or_array<float, NUM_JOINTS>(root, "dof_vel_scale", 0.08f);
  cfg.action_scale =
      parse_scalar_or_array<float, NUM_JOINTS>(root, "action_scale", 0.15f);

  cfg.cmd_deadband = root["cmd_deadband"].as<float>();
  cfg.control_dt = root["control_dt"].as<float>();
  cfg.standup_duration = root["standup_duration"].as<float>();

  if (root["cmd_vx_min"]) cfg.cmd_vx_min = root["cmd_vx_min"].as<float>();
  if (root["cmd_vx_max"]) cfg.cmd_vx_max = root["cmd_vx_max"].as<float>();
  if (root["cmd_vy_min"]) cfg.cmd_vy_min = root["cmd_vy_min"].as<float>();
  if (root["cmd_vy_max"]) cfg.cmd_vy_max = root["cmd_vy_max"].as<float>();
  if (root["cmd_yaw_min"]) cfg.cmd_yaw_min = root["cmd_yaw_min"].as<float>();
  if (root["cmd_yaw_max"]) cfg.cmd_yaw_max = root["cmd_yaw_max"].as<float>();

  if (root["cmd_vx_step"]) cfg.cmd_vx_step = root["cmd_vx_step"].as<float>();
  if (root["cmd_vy_step"]) cfg.cmd_vy_step = root["cmd_vy_step"].as<float>();
  if (root["cmd_yaw_step"]) {
    cfg.cmd_yaw_step = root["cmd_yaw_step"].as<float>();
  }

  if (root["clip_obs"]) cfg.clip_obs = root["clip_obs"].as<float>();
  if (root["clip_actions"]) {
    cfg.clip_actions = root["clip_actions"].as<float>();
  }
  if (root["hip_reduction"]) {
    cfg.hip_reduction = root["hip_reduction"].as<float>();
  }

  return cfg;
}

} // namespace deploy
