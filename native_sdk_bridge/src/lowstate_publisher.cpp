#include "native_sdk_bridge/lowstate_publisher.hpp"

#include <iostream>

namespace native_sdk_bridge {

bool LowStatePublisher::initialize(int domain_id, const std::string& topic) {
  domain_id_ = domain_id;
  topic_ = topic;
  initialized_ = true;
  std::cerr << "native lowstate publisher initialized (domain_id=" << domain_id_
            << ", topic=" << topic_ << ")" << std::endl;
  return true;
}

void LowStatePublisher::publish(const LowStatePacket& packet) {
  if (!initialized_) {
    return;
  }
  std::cerr << "native lowstate publish placeholder tick=" << packet.tick
            << " joints=" << packet.joint_positions_dds.size() << std::endl;
}

}  // namespace native_sdk_bridge
