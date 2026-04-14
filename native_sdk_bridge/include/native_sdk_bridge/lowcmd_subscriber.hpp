#pragma once

#include <memory>
#include <string>

namespace native_sdk_bridge {

class LowCmdSubscriber {
 public:
  LowCmdSubscriber();
  ~LowCmdSubscriber();

  bool initialize(int domain_id, const std::string& topic, const std::string& udp_host, int udp_port);
  void shutdown();

 private:
  struct Impl;
  std::unique_ptr<Impl> impl_;
};

}  // namespace native_sdk_bridge
