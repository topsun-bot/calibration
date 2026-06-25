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
  if (argc != 8) {
    std::cerr << "用法: d1_send_joints angle0 angle1 ... angle6 (度)\n";
    return 1;
  }

  init_channel_factory();
  ChannelPublisher<unitree_arm::msg::dds_::ArmString_> publisher(TOPIC_CMD);
  publisher.InitChannel();

  std::ostringstream oss;
  oss << "{\"seq\":4,\"address\":1,\"funcode\":2,\"data\":{\"mode\":1";
  for (int i = 0; i < 7; ++i) {
    oss << ",\"angle" << i << "\":" << std::atof(argv[i + 1]);
  }
  oss << "}}";

  unitree_arm::msg::dds_::ArmString_ msg{};
  msg.data_() = oss.str();
  publisher.Write(msg);

  std::this_thread::sleep_for(std::chrono::milliseconds(200));
  std::cout << "{\"ok\":true,\"command\":" << msg.data_() << "}" << std::endl;
  return 0;
}
