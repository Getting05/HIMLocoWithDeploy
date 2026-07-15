/**
 * @file motor_driver.cpp
 * @brief GO-M8010-6 motor driver implementation.
 *
 * 所有电机映射信息 (motor_id, port_idx, is_reversed) 来自
 * RobotRuntimeConfig，由 YAML 配置文件加载。
 *
 * 拔掉 USB 时 SDK 通常只报 "does not reply"/timeout，进程不挂。
 * 重新插上后旧 SerialPort fd 已失效，再 sendRecv 时 tcgetattr 抛
 * IOException 会把进程打死。此处：路径消失时立刻释放旧句柄；路径回来后
 * 重新 open；并用 try/catch 兜底，避免热插拔杀进程。
 */

#include "motor_driver.h"

#include <cmath>
#include <iostream>
#include <sys/stat.h>

#include "serialPort/SerialPort.h"
#include "unitreeMotor/unitreeMotor.h"

namespace deploy {

namespace {
constexpr auto kReopenInterval = std::chrono::milliseconds(500);

bool device_path_present(const std::string &path) {
  struct stat st {};
  return ::stat(path.c_str(), &st) == 0 && S_ISCHR(st.st_mode);
}

void safe_release_serial(std::unique_ptr<SerialPort> &port) {
  if (!port) {
    return;
  }
  try {
    port.reset();
  } catch (...) {
    // 避免析构阶段再抛异常杀进程；宁可泄漏 fd。
    (void)port.release();
  }
}
} // namespace

MotorDriver::MotorDriver(const RobotRuntimeConfig &config,
                         const std::string &port0,
                         const std::string &port1)
    : config_(config) {
  port_paths_[0] = port0;
  port_paths_[1] = port1;

  std::cout << "[MotorDriver] Opening serial ports: " << port0 << ", " << port1
            << std::endl;

  serials_[0] = std::make_unique<SerialPort>(port0);
  serials_[1] = std::make_unique<SerialPort>(port1);
  port_ok_[0] = true;
  port_ok_[1] = true;
  path_present_[0] = true;
  path_present_[1] = true;

  cmd_ = std::make_unique<MotorCmd>();
  data_ = std::make_unique<MotorData>();
  cmd_->motorType = MotorType::GO_M8010_6;
  data_->motorType = MotorType::GO_M8010_6;
  foc_mode_ = queryMotorMode(MotorType::GO_M8010_6, MotorMode::FOC);

  // Verify gear ratio from SDK
  float sdk_ratio = queryGearRatio(MotorType::GO_M8010_6);
  if (std::abs(sdk_ratio - config_.joint_transmission_ratio[0]) > 0.1f) {
    std::cout << "[MotorDriver] WARNING: SDK gear ratio " << sdk_ratio
              << " != config " << config_.joint_transmission_ratio[0]
              << std::endl;
  }

  // Initialize offsets to zero (will be calibrated below)
  motor_offsets_.fill(0.0f);
  dof_pos_ = config_.default_dof_pos; // Initialize to configured pose
  dof_vel_.fill(0.0f);
  dof_tau_.fill(0.0f);
  motor_temps_.fill(0.0f);
  motor_errors_.fill(0);

  // Calibrate encoder offsets so initial readings = default_dof_pos
  calibrate_offsets();
}

MotorDriver::~MotorDriver() {
  safe_release_serial(serials_[0]);
  safe_release_serial(serials_[1]);
}

void MotorDriver::mark_port_failed(int port_idx, const char *reason) {
  if (port_idx < 0 || port_idx > 1) {
    return;
  }
  if (port_ok_[port_idx] || serials_[port_idx]) {
    std::cerr << "[MotorDriver] Serial port " << port_idx << " ("
              << port_paths_[port_idx] << ") unavailable: " << reason
              << " — waiting for replug + reopen" << std::endl;
  }
  port_ok_[port_idx] = false;
  safe_release_serial(serials_[port_idx]);
}

bool MotorDriver::reopen_port(int port_idx) {
  if (port_idx < 0 || port_idx > 1) {
    return false;
  }
  if (!device_path_present(port_paths_[port_idx])) {
    port_ok_[port_idx] = false;
    safe_release_serial(serials_[port_idx]);
    path_present_[port_idx] = false;
    return false;
  }

  safe_release_serial(serials_[port_idx]);
  try {
    serials_[port_idx] = std::make_unique<SerialPort>(port_paths_[port_idx]);
    port_ok_[port_idx] = true;
    path_present_[port_idx] = true;
    std::cout << "[MotorDriver] Serial port " << port_idx
              << " reopened after hotplug: " << port_paths_[port_idx]
              << std::endl;
    return true;
  } catch (const std::exception &e) {
    std::cerr << "[MotorDriver] Reopen port " << port_idx
              << " failed: " << e.what() << std::endl;
    port_ok_[port_idx] = false;
    safe_release_serial(serials_[port_idx]);
    return false;
  } catch (...) {
    port_ok_[port_idx] = false;
    safe_release_serial(serials_[port_idx]);
    return false;
  }
}

bool MotorDriver::ensure_port_ready(int port_idx) {
  if (port_idx < 0 || port_idx > 1) {
    return false;
  }

  const bool present = device_path_present(port_paths_[port_idx]);

  // 拔掉：by-id / tty 节点消失。立刻丢掉旧 fd，否则插回去再用会死。
  if (!present) {
    path_present_[port_idx] = false;
    if (serials_[port_idx] || port_ok_[port_idx]) {
      mark_port_failed(port_idx, "device path disappeared (unplugged)");
    }
    return false;
  }

  // 重新插上：路径回来了，但旧 SerialPort 句柄一定不能复用，必须重新 open。
  const bool need_fresh_open =
      !path_present_[port_idx] || !port_ok_[port_idx] || !serials_[port_idx];

  if (!need_fresh_open) {
    return true;
  }

  const auto now = std::chrono::steady_clock::now();
  if (now - last_reopen_attempt_[port_idx] < kReopenInterval) {
    return false;
  }
  last_reopen_attempt_[port_idx] = now;
  return reopen_port(port_idx);
}

bool MotorDriver::try_reopen_ports() {
  for (int i = 0; i < 2; ++i) {
    (void)ensure_port_ready(i);
  }
  return ports_ok();
}

// ------------------------------------------------------------------ //
//  Encoder offset calibration                                         //
// ------------------------------------------------------------------ //

void MotorDriver::calibrate_offsets() {
  std::cout << "[MotorDriver] Calibrating encoder offsets..." << std::endl;
  std::cout << "[MotorDriver] Assuming current pose = default_dof_pos"
            << std::endl;

  // Read raw encoder positions by sending zero-torque commands
  // (offsets are currently 0, so data_->q is the raw encoder value)
  // Send a few times to ensure we get valid readings
  constexpr int NUM_READS = 3;
  for (int r = 0; r < NUM_READS; ++r) {
    for (int i = 0; i < NUM_JOINTS; ++i) {
      const auto &mapping = config_.motor_map[i];
      float direction = mapping.is_reversed ? -1.0f : 1.0f;

      cmd_->mode = foc_mode_;
      cmd_->id = static_cast<unsigned short>(mapping.motor_id);
      cmd_->q = 0.0f;
      cmd_->dq = 0.0f;
      cmd_->kp = 0.0f;
      cmd_->kd = 0.0f;
      cmd_->tau = 0.0f;

      data_->correct = false;
      try {
        if (!ensure_port_ready(mapping.port_idx)) {
          throw std::runtime_error("serial port not ready");
        }
        serials_[mapping.port_idx]->sendRecv(cmd_.get(), data_.get());
      } catch (const std::exception &e) {
        mark_port_failed(mapping.port_idx, e.what());
        path_present_[mapping.port_idx] = false;
        if (r == NUM_READS - 1) {
          std::cout << "[MotorDriver] WARNING: " << config_.joint_names[i]
                    << " (motor " << mapping.motor_id
                    << ") serial error during calibration" << std::endl;
        }
        continue;
      }

      if (r == NUM_READS - 1 && data_->correct &&
          static_cast<int>(data_->motor_id) == mapping.motor_id) {
        const float ratio = config_.joint_transmission_ratio[i];
        // offset = raw_q - direction * q_joint * ratio
        float raw_q = data_->q;
        motor_offsets_[i] =
            raw_q - direction * config_.default_dof_pos[i] * ratio;
        dof_pos_[i] = config_.default_dof_pos[i];
        dof_vel_[i] = direction * data_->dq / ratio;
        dof_tau_[i] = direction * data_->tau * ratio;

        std::cout << "[MotorDriver] " << config_.joint_names[i] << " (motor "
                  << mapping.motor_id << ")"
                  << ": raw_q=" << raw_q << " offset=" << motor_offsets_[i]
                  << " -> joint=" << config_.default_dof_pos[i] << " rad"
                  << std::endl;
      } else if (r == NUM_READS - 1) {
        std::cout << "[MotorDriver] WARNING: " << config_.joint_names[i]
                  << " (motor " << mapping.motor_id
                  << ") did not reply, using offset=0" << std::endl;
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
  const auto &mapping = config_.motor_map[dof_idx];
  const int port_idx = mapping.port_idx;

  // 热插拔关键：路径没了就释放；插回来必须新 open，禁止用旧 fd。
  if (!ensure_port_ready(port_idx)) {
    return;
  }

  const float ratio = config_.joint_transmission_ratio[dof_idx];
  float direction = mapping.is_reversed ? -1.0f : 1.0f;

  // Convert joint-side → motor-side
  float q_motor = direction * q_joint * ratio + motor_offsets_[dof_idx];
  float dq_motor = direction * dq_joint * ratio;
  float tau_motor = direction * tau / ratio;
  float kp_motor = kp / (ratio * ratio);
  float kd_motor = kd / (ratio * ratio);

  cmd_->mode = foc_mode_;
  cmd_->id = static_cast<unsigned short>(mapping.motor_id);
  cmd_->q = q_motor;
  cmd_->dq = dq_motor;
  cmd_->kp = kp_motor;
  cmd_->kd = kd_motor;
  cmd_->tau = tau_motor;

  // Reset correct flag before sendRecv
  data_->correct = false;

  try {
    serials_[port_idx]->sendRecv(cmd_.get(), data_.get());
  } catch (const std::exception &e) {
    // 插回去瞬间若仍踩到失效 fd，吞掉异常并强制下一轮重开
    mark_port_failed(port_idx, e.what());
    path_present_[port_idx] = false;
    return;
  } catch (...) {
    mark_port_failed(port_idx, "unknown serial error");
    path_present_[port_idx] = false;
    return;
  }

  // Only update cached values if the motor actually replied
  if (data_->correct && static_cast<int>(data_->motor_id) == mapping.motor_id) {
    dof_pos_[dof_idx] = direction * (data_->q - motor_offsets_[dof_idx]) / ratio;
    dof_vel_[dof_idx] = direction * data_->dq / ratio;
    dof_tau_[dof_idx] = direction * data_->tau * ratio;
    motor_temps_[dof_idx] = static_cast<float>(data_->temp);
    motor_errors_[dof_idx] = data_->merror;
  }
  // else: keep previous cached values for this DOF unchanged (don't reply)
}

// ------------------------------------------------------------------ //
//  High-level commands                                                //
// ------------------------------------------------------------------ //

void MotorDriver::send_commands(
    const std::array<float, NUM_JOINTS> &target_dof_pos,
    const std::array<float, NUM_JOINTS> &kp,
    const std::array<float, NUM_JOINTS> &kd) {
  for (int i = 0; i < NUM_JOINTS; ++i) {
    send_single(i, target_dof_pos[i], 0.0f, kp[i], kd[i], 0.0f);
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
      std::cout << "[MotorDriver] ERROR on " << config_.joint_names[i]
                << " (motor " << config_.motor_map[i].motor_id << "): code "
                << motor_errors_[i] << std::endl;
      has_error = true;
    }
  }
  return has_error;
}

} // namespace deploy
