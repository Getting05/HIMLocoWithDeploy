/**
 * @file motor_driver.cpp
 * @brief GO-M8010-6 motor driver implementation.
 */

#include "motor_driver.h"

#include <cmath>
#include <iostream>

#include "serialPort/SerialPort.h"
#include "unitreeMotor/unitreeMotor.h"

namespace deploy {

MotorDriver::MotorDriver(const std::string &port0, const std::string &port1) {
  std::cout << "[MotorDriver] Opening serial ports: " << port0 << ", " << port1
            << std::endl;

  serials_[0] = std::make_unique<SerialPort>(port0);
  serials_[1] = std::make_unique<SerialPort>(port1);

  cmd_ = std::make_unique<MotorCmd>();
  data_ = std::make_unique<MotorData>();
  cmd_->motorType = MotorType::GO_M8010_6;
  data_->motorType = MotorType::GO_M8010_6;
  foc_mode_ = queryMotorMode(MotorType::GO_M8010_6, MotorMode::FOC);

  // Verify gear ratio from SDK
  float sdk_ratio = queryGearRatio(MotorType::GO_M8010_6);
  if (std::abs(sdk_ratio - GEAR_RATIO) > 0.1f) {
    std::cout << "[MotorDriver] WARNING: SDK gear ratio " << sdk_ratio
              << " != config " << GEAR_RATIO << std::endl;
  }

  // Initialize offsets to zero (will be calibrated below)
  motor_offsets_.fill(0.0f);
  dof_pos_ = DEFAULT_DOF_POS; // Initialize to standing pose
  dof_vel_.fill(0.0f);
  motor_temps_.fill(0.0f);
  motor_errors_.fill(0);

  // Calibrate encoder offsets so initial readings = DEFAULT_DOF_POS
  calibrate_offsets();
}

MotorDriver::~MotorDriver() {
  // Serials are cleaned up by unique_ptr
}

// ------------------------------------------------------------------ //
//  Encoder offset calibration                                         //
// ------------------------------------------------------------------ //

void MotorDriver::calibrate_offsets() {
  std::cout << "[MotorDriver] Calibrating encoder offsets..." << std::endl;
  std::cout << "[MotorDriver] Assuming current pose = DEFAULT_DOF_POS"
            << std::endl;

  // Read raw encoder positions by sending zero-torque commands
  // (offsets are currently 0, so data_->q is the raw encoder value)
  // Send a few times to ensure we get valid readings
  constexpr int NUM_READS = 3;
  for (int r = 0; r < NUM_READS; ++r) {
    for (int i = 0; i < NUM_JOINTS; ++i) {
      const auto &mapping = MOTOR_MAP[i];
      float direction = mapping.is_reversed ? -1.0f : 1.0f;

      cmd_->mode = foc_mode_;
      cmd_->id = static_cast<unsigned short>(mapping.motor_id);
      cmd_->q = 0.0f;
      cmd_->dq = 0.0f;
      cmd_->kp = 0.0f;
      cmd_->kd = 0.0f;
      cmd_->tau = 0.0f;

      data_->correct = false;
      serials_[mapping.port_idx]->sendRecv(cmd_.get(), data_.get());

      if (r == NUM_READS - 1 && data_->correct &&
          static_cast<int>(data_->motor_id) == mapping.motor_id) {
        // offset = raw_q - direction * DEFAULT_DOF_POS[i] * GEAR_RATIO
        // So that: direction * (raw_q - offset) / GEAR_RATIO =
        // DEFAULT_DOF_POS[i]
        float raw_q = data_->q;
        motor_offsets_[i] = raw_q - direction * DEFAULT_DOF_POS[i] * GEAR_RATIO;
        dof_pos_[i] = DEFAULT_DOF_POS[i];
        dof_vel_[i] = direction * data_->dq / GEAR_RATIO;

        std::cout << "[MotorDriver] " << JOINT_NAMES[i] << ": raw_q=" << raw_q
                  << " offset=" << motor_offsets_[i]
                  << " -> joint=" << DEFAULT_DOF_POS[i] << " rad" << std::endl;
      } else if (r == NUM_READS - 1) {
        std::cout << "[MotorDriver] WARNING: " << JOINT_NAMES[i] << " (motor "
                  << mapping.motor_id << ") did not reply, using offset=0"
                  << std::endl;
      }
    }
  }

  std::cout << "[MotorDriver] Calibration complete." << std::endl;
}

// ------------------------------------------------------------------ //
//  Low-level: single motor                                            //
// ------------------------------------------------------------------ //

void MotorDriver::send_single(int dof_idx, float q_joint, float dq_joint,
                              float kp, float kd, float tau) {
  const auto &mapping = MOTOR_MAP[dof_idx];
  float direction = mapping.is_reversed ? -1.0f : 1.0f;

  // Convert joint-side → motor-side
  float q_motor = direction * q_joint * GEAR_RATIO + motor_offsets_[dof_idx];
  float dq_motor = direction * dq_joint * GEAR_RATIO;

  cmd_->mode = foc_mode_;
  cmd_->id = static_cast<unsigned short>(mapping.motor_id);
  cmd_->q = q_motor;
  cmd_->dq = dq_motor;
  cmd_->kp = kp;
  cmd_->kd = kd;
  cmd_->tau = tau;

  // Reset correct flag before sendRecv
  data_->correct = false;

  serials_[mapping.port_idx]->sendRecv(cmd_.get(), data_.get());

  // Only update cached values if the motor actually replied
  if (data_->correct && static_cast<int>(data_->motor_id) == mapping.motor_id) {
    dof_pos_[dof_idx] =
        direction * (data_->q - motor_offsets_[dof_idx]) / GEAR_RATIO;
    dof_vel_[dof_idx] = direction * data_->dq / GEAR_RATIO;
    motor_temps_[dof_idx] = static_cast<float>(data_->temp);
    motor_errors_[dof_idx] = data_->merror;
  }
  // else: keep previous cached values for this DOF unchanged
}

// ------------------------------------------------------------------ //
//  High-level commands                                                //
// ------------------------------------------------------------------ //

void MotorDriver::send_commands(
    const std::array<float, NUM_JOINTS> &target_dof_pos, float kp, float kd) {
  for (int i = 0; i < NUM_JOINTS; ++i) {
    send_single(i, target_dof_pos[i], 0.0f, kp, kd, 0.0f);
  }
}

void MotorDriver::send_damping(float kd) {
  for (int i = 0; i < NUM_JOINTS; ++i) {
    send_single(i, 0.0f, 0.0f, 0.0f, kd, 0.0f);
  }
}

void MotorDriver::set_zero_torque() {
  for (int i = 0; i < NUM_JOINTS; ++i) {
    send_single(i, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f);
  }
}

bool MotorDriver::check_errors() const {
  bool has_error = false;
  for (int i = 0; i < NUM_JOINTS; ++i) {
    if (motor_errors_[i] != 0) {
      std::cout << "[MotorDriver] ERROR on " << JOINT_NAMES[i] << " (motor "
                << MOTOR_MAP[i].motor_id << "): code " << motor_errors_[i]
                << std::endl;
      has_error = true;
    }
  }
  return has_error;
}

} // namespace deploy
