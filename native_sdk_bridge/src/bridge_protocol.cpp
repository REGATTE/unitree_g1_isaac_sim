#include "native_sdk_bridge/bridge_protocol.hpp"

#include <iostream>

#include <unitree/common/json/json.hpp>
#include <unitree/common/json/jsonize.hpp>

namespace native_sdk_bridge {

namespace {

struct LowStatePacketJson : public unitree::common::Jsonize {
  std::int32_t tick{0};
  std::vector<double> joint_positions_dds;
  std::vector<double> joint_velocities_dds;
  std::vector<double> joint_efforts_dds;
  std::vector<double> imu_quaternion_wxyz;
  std::vector<double> imu_accelerometer_body;
  std::vector<double> imu_gyroscope_body;

  void toJson(unitree::common::JsonMap&) const override {}

  void fromJson(unitree::common::JsonMap& map) override {
    JN_FROM(map, "tick", tick);
    JN_FROM(map, "joint_positions_dds", joint_positions_dds);
    JN_FROM(map, "joint_velocities_dds", joint_velocities_dds);
    JN_FROM(map, "joint_efforts_dds", joint_efforts_dds);
    JN_FROM(map, "imu_quaternion_wxyz", imu_quaternion_wxyz);
    JN_FROM(map, "imu_accelerometer_body", imu_accelerometer_body);
    JN_FROM(map, "imu_gyroscope_body", imu_gyroscope_body);
  }
};

template <std::size_t N>
bool copy_array(const std::vector<double>& src, std::array<double, N>& dst, const char* label) {
  if (src.size() != N) {
    std::cerr << "invalid native lowstate packet field " << label << ": expected "
              << N << " entries, got " << src.size() << std::endl;
    return false;
  }
  for (std::size_t index = 0; index < N; ++index) {
    dst[index] = src[index];
  }
  return true;
}

}  // namespace

bool DecodeLowStatePacket(const std::string& payload, LowStatePacket& packet) {
  LowStatePacketJson parsed;
  try {
    unitree::common::FromJsonString(payload, parsed);
  } catch (const std::exception& exc) {
    std::cerr << "failed to parse native lowstate packet JSON: " << exc.what() << std::endl;
    return false;
  }

  if (parsed.joint_positions_dds.size() < 29 || parsed.joint_velocities_dds.size() < 29 ||
      parsed.joint_efforts_dds.size() < 29) {
    std::cerr << "native lowstate packet is too short: positions="
              << parsed.joint_positions_dds.size() << " velocities="
              << parsed.joint_velocities_dds.size() << " efforts="
              << parsed.joint_efforts_dds.size() << std::endl;
    return false;
  }

  if (!copy_array(parsed.imu_quaternion_wxyz, packet.imu_quaternion_wxyz, "imu_quaternion_wxyz") ||
      !copy_array(parsed.imu_accelerometer_body, packet.imu_accelerometer_body, "imu_accelerometer_body") ||
      !copy_array(parsed.imu_gyroscope_body, packet.imu_gyroscope_body, "imu_gyroscope_body")) {
    return false;
  }

  packet.tick = parsed.tick;
  packet.joint_positions_dds.assign(parsed.joint_positions_dds.begin(), parsed.joint_positions_dds.begin() + 29);
  packet.joint_velocities_dds.assign(parsed.joint_velocities_dds.begin(), parsed.joint_velocities_dds.begin() + 29);
  packet.joint_efforts_dds.assign(parsed.joint_efforts_dds.begin(), parsed.joint_efforts_dds.begin() + 29);
  return true;
}

}  // namespace native_sdk_bridge
