#include "native_sdk_bridge/lowstate_publisher.hpp"

#include <array>
#include <cstdint>
#include <iostream>
#include <memory>
#include <utility>

#include <unitree/dds_wrapper/common/crc.h>
#include <unitree/idl/hg/IMUState_.hpp>
#include <unitree/idl/hg/LowState_.hpp>
#include <unitree/robot/channel/channel_factory.hpp>
#include <unitree/robot/channel/channel_publisher.hpp>

namespace native_sdk_bridge {

using unitree::robot::ChannelFactory;
using unitree::robot::ChannelPublisher;
using unitree::robot::ChannelPublisherPtr;
using unitree_hg::msg::dds_::IMUState_;
using unitree_hg::msg::dds_::LowState_;

struct LowStatePublisher::Impl {
  int domain_id{0};
  std::string topic;
  std::string udp_host;
  int udp_port{0};
  bool initialized{false};
  ChannelPublisherPtr<LowState_> publisher;
};

namespace {

constexpr std::size_t kBodyJointCount = 29;
constexpr std::size_t kMotorSlotCount = 35;
constexpr int64_t kWriteWaitForReaderMicroseconds = 20000;

LowState_ BuildLowStateMessage(const LowStatePacket& packet) {
  LowState_ message;
  message.version(std::array<uint32_t, 2>{1, 0});
  message.mode_pr(0);
  message.mode_machine(0);
  message.tick(static_cast<uint32_t>(packet.tick));

  IMUState_ imu;
  std::array<float, 4> quat{};
  std::array<float, 3> gyro{};
  std::array<float, 3> accel{};
  for (std::size_t index = 0; index < quat.size(); ++index) {
    quat[index] = static_cast<float>(packet.imu_quaternion_wxyz[index]);
  }
  for (std::size_t index = 0; index < gyro.size(); ++index) {
    gyro[index] = static_cast<float>(packet.imu_gyroscope_body[index]);
    accel[index] = static_cast<float>(packet.imu_accelerometer_body[index]);
  }
  imu.quaternion(quat);
  imu.gyroscope(gyro);
  imu.accelerometer(accel);
  imu.rpy(std::array<float, 3>{0.0f, 0.0f, 0.0f});
  imu.temperature(0);
  message.imu_state(std::move(imu));

  auto& motor_state = message.motor_state();
  for (std::size_t index = 0; index < kBodyJointCount; ++index) {
    motor_state[index].mode(0);
    motor_state[index].q(static_cast<float>(packet.joint_positions_dds[index]));
    motor_state[index].dq(static_cast<float>(packet.joint_velocities_dds[index]));
    motor_state[index].ddq(0.0f);
    motor_state[index].tau_est(static_cast<float>(packet.joint_efforts_dds[index]));
    motor_state[index].temperature(std::array<int16_t, 2>{0, 0});
    motor_state[index].vol(0.0f);
    motor_state[index].sensor(std::array<uint32_t, 2>{0, 0});
    motor_state[index].motorstate(0);
    motor_state[index].reserve(std::array<uint32_t, 4>{0, 0, 0, 0});
  }
  for (std::size_t index = kBodyJointCount; index < kMotorSlotCount; ++index) {
    motor_state[index] = {};
  }

  message.wireless_remote(std::array<uint8_t, 40>{});
  message.reserve(std::array<uint32_t, 4>{0, 0, 0, 0});
  message.crc(0);
  message.crc(crc32_core(reinterpret_cast<uint32_t*>(&message), (sizeof(LowState_) >> 2) - 1));
  return message;
}

}  // namespace

LowStatePublisher::LowStatePublisher() = default;

LowStatePublisher::~LowStatePublisher() = default;

bool LowStatePublisher::initialize(int domain_id, const std::string& topic, const std::string& udp_host, int udp_port) {
  if (impl_ && impl_->initialized) {
    return true;
  }
  impl_ = std::make_unique<Impl>();
  impl_->domain_id = domain_id;
  impl_->topic = topic;
  impl_->udp_host = udp_host;
  impl_->udp_port = udp_port;

  ChannelFactory::Instance()->Init(domain_id, "");
  impl_->publisher = std::make_shared<ChannelPublisher<LowState_>>(topic);
  impl_->publisher->InitChannel();
  impl_->initialized = true;
  std::cerr << "native lowstate publisher initialized (domain_id=" << impl_->domain_id
            << ", topic=" << impl_->topic
            << ", udp_host=" << impl_->udp_host
            << ", udp_port=" << impl_->udp_port
            << ")" << std::endl;
  return true;
}

bool LowStatePublisher::publish(const LowStatePacket& packet) {
  if (!impl_ || !impl_->initialized || !impl_->publisher) {
    return false;
  }
  const LowState_ message = BuildLowStateMessage(packet);
  return impl_->publisher->Write(message, kWriteWaitForReaderMicroseconds);
}

void LowStatePublisher::shutdown() {
  if (!impl_) {
    return;
  }
  if (impl_->publisher) {
    impl_->publisher->CloseChannel();
  }
  impl_.reset();
}

}  // namespace native_sdk_bridge
