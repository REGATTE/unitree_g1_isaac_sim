#include "native_sdk_bridge/lowcmd_subscriber.hpp"

#include <arpa/inet.h>
#include <sys/socket.h>
#include <unistd.h>

#include <cstring>
#include <iostream>
#include <memory>
#include <vector>

#include <unitree/dds_wrapper/common/crc.h>
#include <unitree/idl/hg/LowCmd_.hpp>
#include <unitree/robot/channel/channel_factory.hpp>
#include <unitree/robot/channel/channel_subscriber.hpp>

#include "native_sdk_bridge/bridge_protocol.hpp"

namespace native_sdk_bridge {

using unitree::robot::ChannelFactory;
using unitree::robot::ChannelSubscriber;
using unitree::robot::ChannelSubscriberPtr;
using unitree_hg::msg::dds_::LowCmd_;

struct LowCmdSubscriber::Impl {
  int domain_id{0};
  std::string topic;
  int socket_fd{-1};
  sockaddr_in udp_peer{};
  ChannelSubscriberPtr<LowCmd_> subscriber;
};

namespace {

constexpr std::size_t kBodyJointCount = 29;

bool HasValidOrUnsetCrc(const LowCmd_& message) {
  if (message.crc() == 0) {
    return true;
  }
  const auto expected = crc32_core(
      reinterpret_cast<uint32_t*>(const_cast<LowCmd_*>(&message)),
      (sizeof(LowCmd_) >> 2) - 1);
  return message.crc() == expected;
}

LowCmdPacket BuildLowCmdPacket(const LowCmd_& message) {
  LowCmdPacket packet;
  packet.mode_pr = message.mode_pr();
  packet.mode_machine = message.mode_machine();
  packet.joint_positions_dds.reserve(kBodyJointCount);
  packet.joint_velocities_dds.reserve(kBodyJointCount);
  packet.joint_torques_dds.reserve(kBodyJointCount);
  packet.joint_kp_dds.reserve(kBodyJointCount);
  packet.joint_kd_dds.reserve(kBodyJointCount);

  const auto& motor_cmd = message.motor_cmd();
  for (std::size_t index = 0; index < kBodyJointCount; ++index) {
    packet.joint_positions_dds.push_back(motor_cmd[index].q());
    packet.joint_velocities_dds.push_back(motor_cmd[index].dq());
    packet.joint_torques_dds.push_back(motor_cmd[index].tau());
    packet.joint_kp_dds.push_back(motor_cmd[index].kp());
    packet.joint_kd_dds.push_back(motor_cmd[index].kd());
  }
  return packet;
}

}  // namespace

LowCmdSubscriber::LowCmdSubscriber() = default;

LowCmdSubscriber::~LowCmdSubscriber() = default;

bool LowCmdSubscriber::initialize(int domain_id, const std::string& topic, const std::string& udp_host, int udp_port) {
  if (impl_ && impl_->subscriber) {
    return true;
  }

  auto impl = std::make_unique<Impl>();
  impl->domain_id = domain_id;
  impl->topic = topic;
  impl->socket_fd = socket(AF_INET, SOCK_DGRAM, 0);
  if (impl->socket_fd < 0) {
    std::perror("lowcmd socket");
    return false;
  }

  impl->udp_peer.sin_family = AF_INET;
  impl->udp_peer.sin_port = htons(static_cast<uint16_t>(udp_port));
  if (inet_pton(AF_INET, udp_host.c_str(), &impl->udp_peer.sin_addr) != 1) {
    std::cerr << "invalid lowcmd UDP host: " << udp_host << std::endl;
    close(impl->socket_fd);
    return false;
  }

  ChannelFactory::Instance()->Init(domain_id, "");
  const int socket_fd = impl->socket_fd;
  const sockaddr_in udp_peer = impl->udp_peer;
  impl->subscriber = std::make_shared<ChannelSubscriber<LowCmd_>>(topic);
  impl->subscriber->InitChannel(
      [socket_fd, udp_peer](const void* data) {
        if (data == nullptr) {
          return;
        }
        const auto* message = static_cast<const LowCmd_*>(data);
        if (!HasValidOrUnsetCrc(*message)) {
          std::cerr << "dropped native lowcmd with invalid CRC" << std::endl;
          return;
        }
        const std::string payload = EncodeLowCmdPacket(BuildLowCmdPacket(*message));
        const ssize_t sent = sendto(socket_fd, payload.data(), payload.size(), 0,
                                    reinterpret_cast<const sockaddr*>(&udp_peer), sizeof(udp_peer));
        if (sent < 0) {
          std::perror("lowcmd sendto");
        }
      },
      1);

  impl_ = std::move(impl);
  std::cerr << "native lowcmd subscriber initialized (domain_id=" << domain_id
            << ", topic=" << topic
            << ", udp_host=" << udp_host
            << ", udp_port=" << udp_port
            << ")" << std::endl;
  return true;
}

void LowCmdSubscriber::shutdown() {
  if (!impl_) {
    return;
  }
  if (impl_->subscriber) {
    impl_->subscriber->CloseChannel();
  }
  if (impl_->socket_fd >= 0) {
    close(impl_->socket_fd);
  }
  impl_.reset();
}

}  // namespace native_sdk_bridge
