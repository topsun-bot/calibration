#include <cstdlib>
#include <iostream>
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

int main() {
  init_channel_factory();
  ChannelPublisher<unitree_arm::msg::dds_::ArmString_> publisher(TOPIC_CMD);
  publisher.InitChannel();

  unitree_arm::msg::dds_::ArmString_ msg{};
  msg.data_() = "{\"seq\":4,\"address\":1,\"funcode\":2,\"data\":{\"mode\":1,"
                 "\"angle0\":0,\"angle1\":0,\"angle2\":0,\"angle3\":0,"
                 "\"angle4\":0,\"angle5\":0,\"angle6\":0}}";
  publisher.Write(msg);

  std::this_thread::sleep_for(std::chrono::milliseconds(200));
  std::cout << "{\"ok\":true,\"action\":\"zero\"}" << std::endl;
  return 0;
}
