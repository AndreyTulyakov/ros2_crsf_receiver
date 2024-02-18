#include "crsf_receiver.h"


CrsfReceiverNode::CrsfReceiverNode(): Node("crsf_reader_node")
{
    this->declare_parameter("device", "/dev/serial0");
    this->declare_parameter("baudrate", CRSF_BAUDRATE);
    this->declare_parameter("link_stats", true);
    this->declare_parameter("receiver_rate", 100);

    channels_publisher = this->create_publisher<crsf_receiver_msg::msg::CRSFChannels16>(
        "rc/channels", 
        rclcpp::QoS(1).best_effort().durability_volatile()
    );

    link_publisher = this->create_publisher<crsf_receiver_msg::msg::CRSFLinkInfo>(
        "rc/link", 
        rclcpp::QoS(1).best_effort().durability_volatile()
    );

    device = this->get_parameter("device").as_string();
    int baudrate = this->get_parameter("baudrate").as_int();
    int rate = this->get_parameter("receiver_rate").as_int();
    int period = 1000 / rate;

    RCLCPP_INFO(this->get_logger(), "Receiver rate is %dhz (period %dms)", rate, period);
    RCLCPP_INFO(this->get_logger(), "Target serial device is: %s", device.c_str());
    RCLCPP_INFO(this->get_logger(), "Selected baudrate: %d", baudrate);

    timer_ = this->create_wall_timer(
        std::chrono::milliseconds(period), 
        std::bind(&CrsfReceiverNode::main_timer_callback, this)
    );
    
    serial.SetDevice(device);
    serial.SetBaudRate(baudrate);
    serial.SetTimeout(period / 2);
}

void CrsfReceiverNode::main_timer_callback()
{
    if(serial.GetState() == CppLinuxSerial::State::CLOSED) {
        try {
            serial.Open();
            
        } catch(const CppLinuxSerial::Exception& e) {
            RCLCPP_WARN(this->get_logger(), "Can not open serial port: %s", device.c_str());
            return;
        }
    }

    if(serial.Available())
    {
        serial.ReadBinary(parser.rx_buffer);
        parser.parse_incoming_bytes();
    }

    if(parser.is_channels_actual()) {
        CRSFChannels16 message = convert_to_channels_message(parser.get_channels_values());
        channels_publisher->publish(message);
    }

    if(parser.is_link_statistics_actual()) {
        CRSFLinkInfo message = convert_to_link_info(parser.get_link_info());
        link_publisher->publish(message);
    }
}

CrsfReceiverNode::~CrsfReceiverNode() {
    if(serial.GetState() == CppLinuxSerial::State::OPEN) {
        serial.Close();
    }
}
