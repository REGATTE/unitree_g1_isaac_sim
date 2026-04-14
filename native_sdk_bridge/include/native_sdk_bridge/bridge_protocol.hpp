#pragma once

#include <array>
#include <cstdint>
#include <string>
#include <vector>

namespace native_sdk_bridge {

struct LowStatePacket {
  std::int32_t tick{0};
  std::vector<double> joint_positions_dds;
  std::vector<double> joint_velocities_dds;
  std::vector<double> joint_efforts_dds;
  std::array<double, 4> imu_quaternion_wxyz{1.0, 0.0, 0.0, 0.0};
  std::array<double, 3> imu_accelerometer_body{0.0, 0.0, 0.0};
  std::array<double, 3> imu_gyroscope_body{0.0, 0.0, 0.0};
};

struct LowCmdPacket {
  std::uint8_t mode_pr{0};
  std::uint8_t mode_machine{0};
  std::vector<double> joint_positions_dds;
  std::vector<double> joint_velocities_dds;
  std::vector<double> joint_torques_dds;
  std::vector<double> joint_kp_dds;
  std::vector<double> joint_kd_dds;
};

bool DecodeLowStatePacket(const std::string& payload, LowStatePacket& packet);
std::string EncodeLowCmdPacket(const LowCmdPacket& packet);

}  // namespace native_sdk_bridge
