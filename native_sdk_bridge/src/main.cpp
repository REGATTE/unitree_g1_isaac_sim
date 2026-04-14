#include <chrono>
#include <cstring>
#include <iostream>
#include <string>
#include <thread>

#include "native_sdk_bridge/lowstate_publisher.hpp"

namespace {

struct Options {
  int domain_id{1};
  std::string lowstate_topic{"rt/lowstate"};
  std::string bind_host{"127.0.0.1"};
  int lowstate_port{35511};
};

Options parse_args(int argc, char* argv[]) {
  Options options;
  for (int index = 1; index < argc; ++index) {
    const std::string arg = argv[index];
    if (arg == "--domain-id" && index + 1 < argc) {
      options.domain_id = std::stoi(argv[++index]);
    } else if (arg == "--lowstate-topic" && index + 1 < argc) {
      options.lowstate_topic = argv[++index];
    } else if (arg == "--bind-host" && index + 1 < argc) {
      options.bind_host = argv[++index];
    } else if (arg == "--lowstate-port" && index + 1 < argc) {
      options.lowstate_port = std::stoi(argv[++index]);
    }
  }
  return options;
}

}  // namespace

int main(int argc, char* argv[]) {
  const Options options = parse_args(argc, argv);

  std::cerr << "native bridge placeholder starting "
            << "(domain_id=" << options.domain_id
            << ", lowstate_topic=" << options.lowstate_topic
            << ", bind_host=" << options.bind_host
            << ", lowstate_port=" << options.lowstate_port
            << ")" << std::endl;

  native_sdk_bridge::LowStatePublisher publisher;
  publisher.initialize(options.domain_id, options.lowstate_topic);

  // Phase 1 scaffold only: keep the process alive so the Python runtime can
  // manage its lifecycle while the real SDK/DDS publish path is filled in.
  while (true) {
    std::this_thread::sleep_for(std::chrono::seconds(1));
  }
  return 0;
}
