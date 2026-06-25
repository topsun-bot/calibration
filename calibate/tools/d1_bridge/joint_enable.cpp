#include <cstdlib>
#include <iostream>
#include <sstream>
#include <string>
#include <thread>
#include <chrono>

#include <unitree/robot/channel/channel_publisher.hpp>
#include "msg/ArmString_.hpp"

#define TOPIC_CMD "rt/arm_Command"

using namespace unitree::robot;
using namespace unitree::common;

static void init_channel_factory() {
  const char* iface = std::getenv("D1_NETWORK_INTERFACE");
  if (iface && iface[0] != '\0') {
    ChannelFactory::Instance()->Init(0, iface);
  } else {
    ChannelFactory::Instance()->Init(0);
  }
}

int main(int argc, char** argv) {
  if (argc != 2) {
    std::cerr << "用法: d1_joint_enable <mode>\n"
              << "  mode=0  卸力/拖拽（可手拖摆位）\n"
              << "  mode=1  使能/位置保持\n";
    return 1;
  }

  const int mode = std::atoi(argv[1]);
  if (mode != 0 && mode != 1) {
    std::cerr << "mode 必须为 0 或 1\n";
    return 1;
  }

  init_channel_factory();
  ChannelPublisher<unitree_arm::msg::dds_::ArmString_> publisher(TOPIC_CMD);
  publisher.InitChannel();

  std::ostringstream oss;
  oss << "{\"seq\":4,\"address\":1,\"funcode\":5,\"data\":{\"mode\":" << mode << "}}";

  unitree_arm::msg::dds_::ArmString_ msg{};
  msg.data_() = oss.str();
  publisher.Write(msg);

  std::this_thread::sleep_for(std::chrono::milliseconds(200));
  std::cout << "{\"ok\":true,\"command\":" << msg.data_() << "}" << std::endl;
  return 0;
}
