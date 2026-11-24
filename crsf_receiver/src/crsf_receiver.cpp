#include "crsf_receiver.h"
#include "utils.h"
#include "baudrate_helper.h"

#include <limits>
#include <optional>
#include <stdexcept>

CrsfReceiverNode::CrsfReceiverNode(): Node("crsf_reader_node")
{
    this->declare_parameter("device", "/dev/serial0");
    this->declare_parameter("baudrate", CRSF_BAUDRATE);
    this->declare_parameter("link_stats", true);
    this->declare_parameter("receiver_rate", 100);

    device = this->get_parameter("device").as_string();
    const auto configured_baudrate = this->get_parameter("baudrate").as_int();
    const auto rate = this->get_parameter("receiver_rate").as_int();
    link_stats = this->get_parameter("link_stats").as_bool();
    if (rate < 1 || rate > 1000) {
        throw std::invalid_argument("receiver_rate must be between 1 and 1000 Hz");
    }
    if (configured_baudrate <= 0 ||
        configured_baudrate > std::numeric_limits<unsigned int>::max()) {
        throw std::invalid_argument("baudrate must be a positive unsigned integer");
    }
    if (device.empty()) {
        throw std::invalid_argument("device must not be empty");
    }
    baudrate = static_cast<unsigned int>(configured_baudrate);

    channels_publisher = this->create_publisher<CRSFChannels16>(
        "rc/channels", rclcpp::QoS(1).best_effort().durability_volatile());
    link_publisher = this->create_publisher<CRSFLinkInfo>(
        "rc/link", rclcpp::QoS(1).best_effort().durability_volatile());

    // One worker suffices for a single UART. Never read or close the port from
    // the ROS timer while an ASIO completion can be using it.
    serial_io_context = std::make_unique<IoContext>(1);
    serial_driver = std::make_unique<SerialDriver>(*serial_io_context);
    serial_driver->init_port(device,
        SerialPortConfig(115200, FlowControl::NONE, Parity::NONE, StopBits::ONE));

    timer_ = this->create_wall_timer(std::chrono::nanoseconds(1000000000 / rate),
        std::bind(&CrsfReceiverNode::main_timer_callback, this));
    last_reconnect_attempt = std::chrono::steady_clock::now();
    open_serial_port();
}

void CrsfReceiverNode::open_serial_port()
{
    if (serial_driver->port()->is_open()) {
        return;
    }
    try {
        serial_driver->port()->open();
        // ASIO only exposes standard baud rates on this platform.
        if (!set_custom_baudrate(device.c_str(), baudrate)) {
            throw std::runtime_error("cannot configure requested UART baud rate");
        }
        {
            std::lock_guard<std::mutex> lock(parser_mutex);
            parser = CrsfParser();
            published_channels_sequence = 0;
            published_link_sequence = 0;
        }
        serial_driver->port()->async_receive(
            std::bind(&CrsfReceiverNode::receive_callback, this,
                std::placeholders::_1, std::placeholders::_2));
        RCLCPP_INFO(this->get_logger(), "Serial port %s opened at %u baud",
            device.c_str(), baudrate);
    } catch (const std::exception& e) {
        if (serial_driver->port()->is_open()) {
            serial_driver->port()->close();
        }
        RCLCPP_WARN(this->get_logger(), "Cannot open/configure %s: %s; retrying in 1 s",
            device.c_str(), e.what());
    }
}

void CrsfReceiverNode::receive_callback(
    const std::vector<uint8_t>& buffer, const size_t& bytes_transferred)
{
    std::lock_guard<std::mutex> lock(parser_mutex);
    parser.rx_buffer.insert(parser.rx_buffer.end(), buffer.begin(),
        buffer.begin() + bytes_transferred);
    parser.parse_incoming_bytes();
}

void CrsfReceiverNode::main_timer_callback()
{
    const auto now = std::chrono::steady_clock::now();
    if (now - last_reconnect_attempt >= std::chrono::seconds(1)) {
        last_reconnect_attempt = now;
        serial_io_context->post([this]() { open_serial_port(); });
    }

    std::optional<CRSFChannels16> channels;
    std::optional<CRSFLinkInfo> link;
    {
        std::lock_guard<std::mutex> lock(parser_mutex);
        parser.parse_incoming_bytes();
        if (parser.is_channels_actual() &&
            parser.channels_sequence() != published_channels_sequence) {
            channels = convert_to_channels_message(parser.get_channels_values());
            published_channels_sequence = parser.channels_sequence();
        }
        if (link_stats && parser.is_link_statistics_actual() &&
            parser.link_statistics_sequence() != published_link_sequence) {
            link = convert_to_link_info(parser.get_link_info());
            published_link_sequence = parser.link_statistics_sequence();
        }
    }
    // Publishing must not hold up reception while DDS serializes a message.
    if (channels) { channels_publisher->publish(*channels); }
    if (link) { link_publisher->publish(*link); }
}

CrsfReceiverNode::~CrsfReceiverNode()
{
    if (timer_) { timer_->cancel(); }
    // Join callbacks before parser, mutex or driver members are destroyed.
    if (serial_io_context) { serial_io_context->waitForExit(); }
    if (serial_driver && serial_driver->port()->is_open()) {
        serial_driver->port()->close();
    }
}
