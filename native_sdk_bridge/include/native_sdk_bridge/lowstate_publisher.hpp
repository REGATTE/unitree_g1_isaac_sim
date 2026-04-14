#pragma once

#include <string>

#include "native_sdk_bridge/bridge_protocol.hpp"

namespace native_sdk_bridge {

class LowStatePublisher {
 public:
  bool initialize(int domain_id, const std::string& topic);
  void publish(const LowStatePacket& packet);

 private:
  int domain_id_{0};
  std::string topic_;
  bool initialized_{false};
};

}  // namespace native_sdk_bridge
