#ifndef BATTERY_TELEMETRY_HPP
#define BATTERY_TELEMETRY_HPP

#include <optional>
#include <vector>
#include "sensor_msgs/msg/battery_state.hpp"

// Encode the legacy 8-byte CRSF battery payload for ELRS/handset compatibility.
// Unknown or inconsistent mandatory inputs produce no packet.
std::optional<std::vector<uint8_t>> encode_battery_telemetry(
    const sensor_msgs::msg::BatteryState& battery);

#endif
