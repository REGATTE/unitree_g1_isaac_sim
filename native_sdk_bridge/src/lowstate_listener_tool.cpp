#include <chrono>
#include <cstdint>
#include <iostream>
#include <mutex>
#include <string>
#include <thread>

#include <unitree/dds_wrapper/common/crc.h>
#include <unitree/idl/hg/LowState_.hpp>
#include <unitree/robot/channel/channel_factory.hpp>
#include <unitree/robot/channel/channel_subscriber.hpp>

namespace {

constexpr int kDefaultDomainId = 1;
constexpr int kBodyJointCount = 29;

struct Options {
  int domain_id{kDefaultDomainId};
  std::string topic{"rt/lowstate"};
  double duration_seconds{5.0};
  int min_messages{1};
  int joint_index{0};
};

struct LatestSample {
  bool available{false};
  bool crc_ok{false};
  std::uint32_t tick{0};
  float q{0.0f};
  float dq{0.0f};
  float tau{0.0f};
  int messages{0};
  int crc_failures{0};
};

Options parse_args(int argc, char** argv) {
  Options options;
  for (int index = 1; index < argc; ++index) {
    const std::string arg = argv[index];
    if (arg == "--domain-id" && index + 1 < argc) {
      options.domain_id = std::stoi(argv[++index]);
    } else if (arg == "--topic" && index + 1 < argc) {
      options.topic = argv[++index];
    } else if (arg == "--duration" && index + 1 < argc) {
      options.duration_seconds = std::stod(argv[++index]);
    } else if (arg == "--min-messages" && index + 1 < argc) {
      options.min_messages = std::stoi(argv[++index]);
    } else if (arg == "--joint-index" && index + 1 < argc) {
      options.joint_index = std::stoi(argv[++index]);
    }
  }
  return options;
}

bool valid_joint_index(int index) {
  return index >= 0 && index < kBodyJointCount;
}

bool crc_ok(unitree_hg::msg::dds_::LowState_& message) {
  return message.crc() == crc32_core(reinterpret_cast<std::uint32_t*>(&message),
                                    (sizeof(unitree_hg::msg::dds_::LowState_) >> 2) - 1);
}

}  // namespace

int main(int argc, char** argv) {
  const Options options = parse_args(argc, argv);
  if (!valid_joint_index(options.joint_index)) {
    std::cerr << "--joint-index must be in [0, " << (kBodyJointCount - 1) << "]" << std::endl;
    return 2;
  }

  LatestSample latest;
  std::mutex latest_mutex;

  unitree::robot::ChannelFactory::Instance()->Init(options.domain_id, "");
  auto subscriber = std::make_shared<unitree::robot::ChannelSubscriber<unitree_hg::msg::dds_::LowState_>>(
      options.topic);
  subscriber->InitChannel(
      [&latest, &latest_mutex, joint_index = options.joint_index](const void* data) {
        if (data == nullptr) {
          return;
        }
        auto message = *static_cast<const unitree_hg::msg::dds_::LowState_*>(data);
        const bool sample_crc_ok = crc_ok(message);
        const auto& motor_state = message.motor_state();

        std::lock_guard<std::mutex> lock(latest_mutex);
        latest.available = true;
        latest.crc_ok = sample_crc_ok;
        latest.tick = message.tick();
        latest.q = motor_state[joint_index].q();
        latest.dq = motor_state[joint_index].dq();
        latest.tau = motor_state[joint_index].tau_est();
        latest.messages += 1;
        if (!sample_crc_ok) {
          latest.crc_failures += 1;
        }
      },
      1);

  const auto deadline = std::chrono::steady_clock::now() +
                        std::chrono::duration_cast<std::chrono::steady_clock::duration>(
                            std::chrono::duration<double>(options.duration_seconds));
  while (std::chrono::steady_clock::now() < deadline) {
    {
      std::lock_guard<std::mutex> lock(latest_mutex);
      if (latest.messages >= options.min_messages && latest.crc_failures == 0) {
        break;
      }
    }
    std::this_thread::sleep_for(std::chrono::milliseconds(20));
  }

  subscriber->CloseChannel();

  std::lock_guard<std::mutex> lock(latest_mutex);
  if (!latest.available) {
    std::cerr << "no native lowstate received on topic=" << options.topic
              << " domain_id=" << options.domain_id << std::endl;
    return 1;
  }

  std::cout << "native_lowstate messages=" << latest.messages
            << " crc_failures=" << latest.crc_failures
            << " tick=" << latest.tick
            << " joint_index=" << options.joint_index
            << " q=" << latest.q
            << " dq=" << latest.dq
            << " tau=" << latest.tau
            << std::endl;
  return latest.messages >= options.min_messages && latest.crc_failures == 0 ? 0 : 1;
}
