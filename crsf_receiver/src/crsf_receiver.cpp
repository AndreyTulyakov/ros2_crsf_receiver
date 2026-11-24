#include "crsf_receiver.h"
#include "utils.h"
#include "baudrate_helper.h"
#include "battery_telemetry.h"

#include <cmath>
#include <limits>
#include <optional>
#include <stdexcept>

CrsfReceiverNode::CrsfReceiverNode(): Node("crsf_reader_node")
{
    this->declare_parameter("device", "/dev/serial0");
    this->declare_parameter("baudrate", CRSF_BAUDRATE);
    this->declare_parameter("link_stats", true);
    this->declare_parameter("receiver_rate", 100);
    this->declare_parameter("telemetry_enabled", false);
    this->declare_parameter("telemetry_battery_topic", "telemetry/battery");
    this->declare_parameter("telemetry_rate", 2.0);
    this->declare_parameter("telemetry_timeout", 1.0);

    device = this->get_parameter("device").as_string();
    const auto configured_baudrate = this->get_parameter("baudrate").as_int();
    const auto rate = this->get_parameter("receiver_rate").as_int();
    link_stats = this->get_parameter("link_stats").as_bool();
    telemetry_enabled = this->get_parameter("telemetry_enabled").as_bool();
    const auto battery_topic = this->get_parameter("telemetry_battery_topic").as_string();
    const auto telemetry_rate = this->get_parameter("telemetry_rate").as_double();
    telemetry_timeout = this->get_parameter("telemetry_timeout").as_double();
    if (!std::isfinite(telemetry_rate) || telemetry_rate < 0.1 || telemetry_rate > 20.0) {
        throw std::invalid_argument("telemetry_rate must be between 0.1 and 20 Hz");
    }
    if (!std::isfinite(telemetry_timeout) || telemetry_timeout <= 0 || telemetry_timeout > 3600) {
        throw std::invalid_argument("telemetry_timeout must be between 0 (exclusive) and 3600 s");
    }
    if (telemetry_enabled && battery_topic.empty()) {
        throw std::invalid_argument("telemetry_battery_topic must not be empty");
    }
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
    if (telemetry_enabled) {
        battery_subscription = this->create_subscription<sensor_msgs::msg::BatteryState>(
            battery_topic, rclcpp::SensorDataQoS().keep_last(1),
            [this](sensor_msgs::msg::BatteryState::ConstSharedPtr battery) {
                battery_callback(*battery);
            });
        telemetry_timer = this->create_wall_timer(
            std::chrono::nanoseconds(static_cast<int64_t>(1000000000.0 / telemetry_rate)),
            std::bind(&CrsfReceiverNode::telemetry_timer_callback, this));
    }
    last_reconnect_attempt = std::chrono::steady_clock::now();
    serial_io_context->post([this]() { open_serial_port(); });
}

void CrsfReceiverNode::open_serial_port()
{
    if (serial_driver->port()->is_open()) {
        open_telemetry_port();
        return;
    }
    if (telemetry_writer) {
        telemetry_writer->close();
        telemetry_writer.reset();
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
        {
            std::lock_guard<std::mutex> lock(battery_mutex);
            battery_sample.reset();
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
    if (serial_driver->port()->is_open()) { open_telemetry_port(); }
}

void CrsfReceiverNode::open_telemetry_port()
{
    if (!telemetry_enabled || (telemetry_writer && telemetry_writer->is_open())) { return; }
    try {
        telemetry_writer = std::make_shared<SerialTelemetryWriter>(
            serial_io_context->ios(), device,
            [logger = this->get_logger()](const asio::error_code& error) {
                RCLCPP_WARN(logger, "Battery telemetry write failed: %s", error.message().c_str());
            });
    } catch (const std::exception& e) {
        // A TX failure must not prevent reception of RC commands.
        RCLCPP_WARN(this->get_logger(), "Cannot open battery telemetry UART: %s; retrying in 1 s",
            e.what());
    }
}

void CrsfReceiverNode::battery_callback(const sensor_msgs::msg::BatteryState& battery)
{
    auto frame = encode_battery_telemetry(battery);
    {
        std::lock_guard<std::mutex> lock(battery_mutex);
        if (frame) {
            battery_sample = BatterySample{std::move(*frame), std::chrono::steady_clock::now()};
        } else {
            battery_sample.reset();
        }
    }
    if (!frame) {
        RCLCPP_WARN_THROTTLE(this->get_logger(), *this->get_clock(), 5000,
            "Ignoring invalid battery telemetry: require present battery, finite voltage/current, "
            "capacity > 0, charge in [0, capacity], percentage in [0, 1] or NaN, and CRSF field ranges");
    }
}

void CrsfReceiverNode::telemetry_timer_callback()
{
    serial_io_context->post([this]() {
        // Read the current cache on the I/O thread, after any reconnect reset.
        // A task queued before reconnect must not replay a pre-reconnect sample.
        std::optional<BatterySample> sample;
        {
            std::lock_guard<std::mutex> lock(battery_mutex);
            sample = battery_sample;
        }
        if (!sample) { return; }
        const auto expires_at = sample->received_at + std::chrono::duration_cast<
            std::chrono::steady_clock::duration>(std::chrono::duration<double>(telemetry_timeout));
        if (serial_driver->port()->is_open() && telemetry_writer && telemetry_writer->is_open()) {
            telemetry_writer->send(std::move(sample->frame), expires_at);
        }
    });
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
    if (telemetry_timer) { telemetry_timer->cancel(); }
    // Join callbacks before parser, mutex or driver members are destroyed.
    if (serial_io_context) { serial_io_context->waitForExit(); }
    if (telemetry_writer) {
        telemetry_writer->close();
        telemetry_writer.reset();
    }
    if (serial_driver && serial_driver->port()->is_open()) {
        serial_driver->port()->close();
    }
}
