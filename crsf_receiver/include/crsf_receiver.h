#ifndef CRSF_RECEIVER_HPP
#define CRSF_RECEIVER_HPP

#include <chrono>
#include <functional>
#include <vector>

#include "std_msgs/msg/string.hpp"
#include "std_msgs/msg/int32_multi_array.hpp"

#include "rclcpp/rclcpp.hpp"
#include "rclcpp/qos.hpp"
#include <CppLinuxSerial/SerialPort.hpp>

#include "crsf_parser.h"
#include "crsf_receiver_msg/msg/crsf_channels16.hpp"
#include "crsf_receiver_msg/msg/crsf_link_info.hpp"

using namespace std::chrono_literals;
using namespace mn;
using namespace crsf_receiver_msg::msg;


class CrsfReceiverNode: public rclcpp::Node
{
public:
  explicit CrsfReceiverNode();
  ~CrsfReceiverNode();
  
private:
  CppLinuxSerial::SerialPort serial;
  CrsfParser parser;

  std::string device;
  rclcpp::TimerBase::SharedPtr timer_;

  rclcpp::Publisher<CRSFChannels16>::SharedPtr channels_publisher;
  rclcpp::Publisher<CRSFLinkInfo>::SharedPtr link_publisher;

  void main_timer_callback();
};


#endif 