/**
 * @file robot_visualizer.cpp
 * @brief RViz visualization implementation.
 */

#include "robot_visualizer.h"

namespace deploy {

RobotVisualizer::RobotVisualizer(
    ros::NodeHandle &nh,
    const std::array<std::string, NUM_JOINTS> &joint_names) {
  // Create JointState publisher
  pub_ = nh.advertise<sensor_msgs::JointState>("/joint_states", 10);

  // Build joint name list matching URDF joint names
  joint_names_.reserve(NUM_JOINTS);
  for (int i = 0; i < NUM_JOINTS; ++i) {
    joint_names_.emplace_back(joint_names[i]);
  }

  ROS_INFO("RobotVisualizer: publishing to /joint_states");
}

void RobotVisualizer::publish_joint_states(
    const std::array<float, NUM_JOINTS> &dof_pos,
    const std::array<float, NUM_JOINTS> &dof_vel) {
  sensor_msgs::JointState msg;
  msg.header.stamp = ros::Time::now();

  msg.name = joint_names_;
  msg.position.resize(NUM_JOINTS);
  msg.velocity.resize(NUM_JOINTS);
  msg.effort.resize(NUM_JOINTS, 0.0);

  for (int i = 0; i < NUM_JOINTS; ++i) {
    msg.position[i] = static_cast<double>(dof_pos[i]);
    msg.velocity[i] = static_cast<double>(dof_vel[i]);
  }

  pub_.publish(msg);
}

} // namespace deploy
