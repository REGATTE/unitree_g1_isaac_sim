#include <array>
#include <chrono>
#include <cstdint>
#include <iostream>
#include <mutex>
#include <string>
#include <thread>

#include <unitree/dds_wrapper/common/crc.h>
#include <unitree/idl/hg/LowCmd_.hpp>
#include <unitree/idl/hg/LowState_.hpp>
#include <unitree/robot/channel/channel_factory.hpp>
#include <unitree/robot/channel/channel_publisher.hpp>
#include <unitree/robot/channel/channel_subscriber.hpp>

namespace {

constexpr int kBodyJointCount = 29;
constexpr int kMotorSlotCount = 35;

struct Options {
  int domain_id{1};
  std::string lowstate_topic{"rt/lowstate"};
  std::string lowcmd_topic{"rt/lowcmd"};
  int joint_index{15};
  float offset_rad{0.05f};
  float default_kp{20.0f};
  float default_kd{1.0f};
  double duration_seconds{2.0};
  double rate_hz{50.0};
  double seed_timeout_seconds{5.0};
};

struct SeedState {
  bool available{false};
  std::array<float, kBodyJointCount> positions{};
};

Options parse_args(int argc, char** argv) {
  Options options;
  for (int index = 1; index < argc; ++index) {
    const std::string arg = argv[index];
    if (arg == "--domain-id" && index + 1 < argc) {
      options.domain_id = std::stoi(argv[++index]);
    } else if (arg == "--lowstate-topic" && index + 1 < argc) {
      options.lowstate_topic = argv[++index];
    } else if (arg == "--lowcmd-topic" && index + 1 < argc) {
      options.lowcmd_topic = argv[++index];
    } else if (arg == "--joint-index" && index + 1 < argc) {
      options.joint_index = std::stoi(argv[++index]);
    } else if (arg == "--offset-rad" && index + 1 < argc) {
      options.offset_rad = std::stof(argv[++index]);
    } else if (arg == "--default-kp" && index + 1 < argc) {
      options.default_kp = std::stof(argv[++index]);
    } else if (arg == "--default-kd" && index + 1 < argc) {
      options.default_kd = std::stof(argv[++index]);
    } else if (arg == "--duration" && index + 1 < argc) {
      options.duration_seconds = std::stod(argv[++index]);
    } else if (arg == "--rate-hz" && index + 1 < argc) {
      options.rate_hz = std::stod(argv[++index]);
    } else if (arg == "--seed-timeout" && index + 1 < argc) {
      options.seed_timeout_seconds = std::stod(argv[++index]);
    }
  }
  return options;
}

bool valid_joint_index(int index) {
  return index >= 0 && index < kBodyJointCount;
}

unitree_hg::msg::dds_::LowCmd_ build_command(const Options& options, const SeedState& seed) {
  unitree_hg::msg::dds_::LowCmd_ command;
  command.mode_pr(0);
  command.mode_machine(0);
  auto& motor_cmd = command.motor_cmd();
  for (int index = 0; index < kBodyJointCount; ++index) {
    auto& motor = motor_cmd[static_cast<std::size_t>(index)];
    motor.mode(1);
    motor.q(seed.positions[static_cast<std::size_t>(index)] + (index == options.joint_index ? options.offset_rad : 0.0f));
    motor.dq(0.0f);
    motor.tau(0.0f);
    motor.kp(options.default_kp);
    motor.kd(options.default_kd);
  }
  for (int index = kBodyJointCount; index < kMotorSlotCount; ++index) {
    motor_cmd[static_cast<std::size_t>(index)] = {};
  }
  command.crc(0);
  command.crc(crc32_core(reinterpret_cast<std::uint32_t*>(&command),
                         (sizeof(unitree_hg::msg::dds_::LowCmd_) >> 2) - 1));
  return command;
}

}  // namespace

int main(int argc, char** argv) {
  const Options options = parse_args(argc, argv);
  if (!valid_joint_index(options.joint_index)) {
    std::cerr << "--joint-index must be in [0, " << (kBodyJointCount - 1) << "]" << std::endl;
    return 2;
  }
  if (options.rate_hz <= 0.0 || options.duration_seconds < 0.0) {
    std::cerr << "--rate-hz must be positive and --duration must be non-negative" << std::endl;
    return 2;
  }

  SeedState seed;
  std::mutex seed_mutex;

  unitree::robot::ChannelFactory::Instance()->Init(options.domain_id, "");
  auto lowstate_subscriber = std::make_shared<unitree::robot::ChannelSubscriber<unitree_hg::msg::dds_::LowState_>>(
      options.lowstate_topic);
  lowstate_subscriber->InitChannel(
      [&seed, &seed_mutex](const void* data) {
        if (data == nullptr) {
          return;
        }
        const auto* message = static_cast<const unitree_hg::msg::dds_::LowState_*>(data);
        const auto& motor_state = message->motor_state();
        std::lock_guard<std::mutex> lock(seed_mutex);
        for (int index = 0; index < kBodyJointCount; ++index) {
          seed.positions[static_cast<std::size_t>(index)] = motor_state[static_cast<std::size_t>(index)].q();
        }
        seed.available = true;
      },
      1);

  const auto seed_deadline = std::chrono::steady_clock::now() +
                             std::chrono::duration_cast<std::chrono::steady_clock::duration>(
                                 std::chrono::duration<double>(options.seed_timeout_seconds));
  while (std::chrono::steady_clock::now() < seed_deadline) {
    {
      std::lock_guard<std::mutex> lock(seed_mutex);
      if (seed.available) {
        break;
      }
    }
    std::this_thread::sleep_for(std::chrono::milliseconds(20));
  }
  lowstate_subscriber->CloseChannel();

  SeedState seed_copy;
  {
    std::lock_guard<std::mutex> lock(seed_mutex);
    if (!seed.available) {
      std::cerr << "did not receive native lowstate seed on topic=" << options.lowstate_topic
                << " domain_id=" << options.domain_id << std::endl;
      return 1;
    }
    seed_copy = seed;
  }

  auto publisher = std::make_shared<unitree::robot::ChannelPublisher<unitree_hg::msg::dds_::LowCmd_>>(
      options.lowcmd_topic);
  publisher->InitChannel();
  const auto command = build_command(options, seed_copy);
  const auto period = std::chrono::duration_cast<std::chrono::steady_clock::duration>(
      std::chrono::duration<double>(1.0 / options.rate_hz));
  const auto publish_deadline = std::chrono::steady_clock::now() +
                                std::chrono::duration_cast<std::chrono::steady_clock::duration>(
                                    std::chrono::duration<double>(options.duration_seconds));
  int published = 0;
  while (std::chrono::steady_clock::now() < publish_deadline) {
    publisher->Write(command, 20000);
    ++published;
    std::this_thread::sleep_for(period);
  }
  publisher->CloseChannel();

  const float seed_q = seed_copy.positions[static_cast<std::size_t>(options.joint_index)];
  std::cout << "native_lowcmd published=" << published
            << " topic=" << options.lowcmd_topic
            << " joint_index=" << options.joint_index
            << " seed_q=" << seed_q
            << " target_q=" << (seed_q + options.offset_rad)
            << " offset_rad=" << options.offset_rad
            << std::endl;
  return published > 0 ? 0 : 1;
}
