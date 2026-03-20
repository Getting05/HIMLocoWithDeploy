#pragma once

#include <array>
#include <string>
#include <vector>

#include "robot_config.h"

namespace deploy {

struct RobotRuntimeConfig {
  int num_of_dofs = NUM_JOINTS;
  float dt = 0.005f;
  int decimation = 4;

  std::array<float, NUM_JOINTS> fixed_kp{};
  std::array<float, NUM_JOINTS> fixed_kd{};
  std::array<float, NUM_JOINTS> torque_limits{};

  std::array<float, NUM_JOINTS> default_dof_pos = DEFAULT_DOF_POS;
  std::array<float, NUM_JOINTS> standup_target_pos = STANDUP_TARGET_POS;
  std::array<float, NUM_JOINTS> policy_dof_pos = POLICY_DOF_POS;
  std::array<float, NUM_JOINTS> joint_pos_lower = JOINT_POS_LOWER;
  std::array<float, NUM_JOINTS> joint_pos_upper = JOINT_POS_UPPER;

  std::array<std::string, NUM_JOINTS> joint_names{};
  std::array<std::string, NUM_JOINTS> joint_controller_names{};
  std::array<int, NUM_JOINTS> joint_mapping{};
  std::vector<int> wheel_indices;

  // Motor-side: q_motor = q_joint * ratio
  std::array<float, NUM_JOINTS> joint_transmission_ratio{};

  std::string policy_path = "policy/policy.pt";
  std::string device = "cuda:0";
  std::string port0 = "/dev/motor_front";
  std::string port1 = "/dev/motor_rear";
  std::string imu_topic = "/fast_livo2/state6";

  float kp_joint = 80.0f;
  float kd_joint = 2.0f;
  float kd_damp_motor = 0.1f;

  float lin_vel_scale = 2.0f;
  float ang_vel_scale = 0.25f;
  std::array<float, NUM_JOINTS> dof_pos_scale{};
  std::array<float, NUM_JOINTS> dof_vel_scale{};
  std::array<float, NUM_JOINTS> action_scale{};

  std::string robot_name = "mybot";
  std::string urdf_relpath = "robot/mybot/urdf/mybot.urdf";
  std::string mujoco_xml_relpath = "robot/mybot/xml/mybot.xml";
  std::string isaac_xml_relpath = "robot/mybot/xml/mybot.xml";

  float cmd_deadband = 0.05f;
  float control_dt = 0.02f;
  float standup_duration = 2.0f;

  float cmd_vx_min = -1.0f;
  float cmd_vx_max = 1.0f;
  float cmd_vy_min = -1.0f;
  float cmd_vy_max = 1.0f;
  float cmd_yaw_min = -3.14f;
  float cmd_yaw_max = 3.14f;

  float cmd_vx_step = 0.1f;
  float cmd_vy_step = 0.1f;
  float cmd_yaw_step = 0.3f;

  float clip_obs = 100.0f;
  float clip_actions = 100.0f;
  float hip_reduction = 1.0f;

  float motor_kp(int joint_idx) const {
    const float r = joint_transmission_ratio[joint_idx];
    return kp_joint / (r * r);
  }

  float motor_kd(int joint_idx) const {
    const float r = joint_transmission_ratio[joint_idx];
    return kd_joint / (r * r);
  }
};

RobotRuntimeConfig default_robot_runtime_config();
RobotRuntimeConfig load_robot_runtime_config(const std::string &yaml_file);

} // namespace deploy
