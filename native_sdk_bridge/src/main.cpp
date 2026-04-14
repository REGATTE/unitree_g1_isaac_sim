#include <arpa/inet.h>
#include <csignal>
#include <cstring>
#include <errno.h>
#include <iostream>
#include <string>
#include <unistd.h>

#include "native_sdk_bridge/bridge_protocol.hpp"
#include "native_sdk_bridge/lowstate_publisher.hpp"

namespace {

volatile std::sig_atomic_t g_running = 1;

struct Options {
  int domain_id{1};
  std::string lowstate_topic{"rt/lowstate"};
  std::string bind_host{"127.0.0.1"};
  int lowstate_port{35511};
};

void handle_signal(int) {
  g_running = 0;
}

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

int create_udp_socket(const Options& options) {
  const int fd = socket(AF_INET, SOCK_DGRAM, 0);
  if (fd < 0) {
    std::perror("socket");
    return -1;
  }

  sockaddr_in addr{};
  addr.sin_family = AF_INET;
  addr.sin_port = htons(static_cast<uint16_t>(options.lowstate_port));
  if (inet_pton(AF_INET, options.bind_host.c_str(), &addr.sin_addr) != 1) {
    std::cerr << "invalid bind host: " << options.bind_host << std::endl;
    close(fd);
    return -1;
  }

  if (bind(fd, reinterpret_cast<sockaddr*>(&addr), sizeof(addr)) != 0) {
    std::perror("bind");
    close(fd);
    return -1;
  }

  return fd;
}

}  // namespace

int main(int argc, char* argv[]) {
  const Options options = parse_args(argc, argv);
  std::signal(SIGINT, handle_signal);
  std::signal(SIGTERM, handle_signal);

  const int socket_fd = create_udp_socket(options);
  if (socket_fd < 0) {
    return 1;
  }

  std::cerr << "native bridge starting "
            << "(domain_id=" << options.domain_id
            << ", lowstate_topic=" << options.lowstate_topic
            << ", bind_host=" << options.bind_host
            << ", lowstate_port=" << options.lowstate_port
            << ")" << std::endl;

  native_sdk_bridge::LowStatePublisher publisher;
  if (!publisher.initialize(options.domain_id, options.lowstate_topic)) {
    close(socket_fd);
    return 1;
  }

  while (g_running) {
    char buffer[65535];
    sockaddr_in peer{};
    socklen_t peer_len = sizeof(peer);
    const ssize_t received = recvfrom(socket_fd, buffer, sizeof(buffer), 0,
                                      reinterpret_cast<sockaddr*>(&peer), &peer_len);
    if (received < 0) {
      if (errno == EINTR) {
        continue;
      }
      std::perror("recvfrom");
      break;
    }

    native_sdk_bridge::LowStatePacket packet;
    if (!native_sdk_bridge::DecodeLowStatePacket(std::string(buffer, static_cast<std::size_t>(received)), packet)) {
      continue;
    }
    if (!publisher.publish(packet)) {
      std::cerr << "failed to publish native lowstate tick=" << packet.tick << std::endl;
    }
  }

  publisher.shutdown();
  close(socket_fd);
  return 0;
}
