#include <atomic>
#include <chrono>
#include <cstdlib>
#include <iostream>
#include <sstream>
#include <thread>

#include <unitree/robot/channel/channel_subscriber.hpp>
#include "msg/PubServoInfo_.hpp"

#define TOPIC_STATE "current_servo_angle"

using namespace unitree::robot;
using namespace unitree::common;

static std::atomic<bool> got_msg{false};
static unitree_arm::msg::dds_::PubServoInfo_ latest{};

static void init_channel_factory() {
  const char* iface = std::getenv("D1_NETWORK_INTERFACE");
  if (iface && iface[0] != '\0') {
    ChannelFactory::Instance()->Init(0, iface);
  } else {
    ChannelFactory::Instance()->Init(0);
  }
}

static void handler(const void* msg) {
  latest = *(const unitree_arm::msg::dds_::PubServoInfo_*)msg;
  got_msg = true;
}

int main() {
  init_channel_factory();
  ChannelSubscriber<unitree_arm::msg::dds_::PubServoInfo_> subscriber(TOPIC_STATE);
  subscriber.InitChannel(handler);

  for (int i = 0; i < 50 && !got_msg.load(); ++i) {
    std::this_thread::sleep_for(std::chrono::milliseconds(100));
  }

  if (!got_msg.load()) {
    std::cerr << "超时: 未收到 current_servo_angle\n";
    return 1;
  }

  std::cout << "{\"joints_deg\":["
            << latest.servo0_data_() << ","
            << latest.servo1_data_() << ","
            << latest.servo2_data_() << ","
            << latest.servo3_data_() << ","
            << latest.servo4_data_() << ","
            << latest.servo5_data_() << ","
            << latest.servo6_data_() << "]}" << std::endl;
  return 0;
}
