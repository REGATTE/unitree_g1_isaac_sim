#pragma once

#include <memory>
#include <string>

#include "native_sdk_bridge/bridge_protocol.hpp"

namespace native_sdk_bridge {

class LowStatePublisher {
 public:
  LowStatePublisher();
  ~LowStatePublisher();

  bool initialize(int domain_id, const std::string& topic, const std::string& udp_host, int udp_port);
  bool publish(const LowStatePacket& packet);
  void shutdown();

 private:
  struct Impl;
  std::unique_ptr<Impl> impl_;
};

}  // namespace native_sdk_bridge
