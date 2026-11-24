#include "rclcpp/rclcpp.hpp"

#include "crsf_receiver.h"


int main(int argc, char const *argv[])
{
    rclcpp::init(argc, argv);
    try {
        rclcpp::spin(std::make_shared<CrsfReceiverNode>());
    } catch (const std::exception& e) {
        RCLCPP_FATAL(rclcpp::get_logger("crsf_reader_node"), "%s", e.what());
        rclcpp::shutdown();
        return 1;
    }
    rclcpp::shutdown();
    return 0;
}
