#include "rclcpp/rclcpp.hpp"

#include "crsf_receiver.h"


int main(int argc, char const *argv[])
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<CrsfReceiverNode>());
    rclcpp::shutdown();
    return 0;
}
