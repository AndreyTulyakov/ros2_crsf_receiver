#include "battery_telemetry.h"
#include "crc8.h"
#include "crsf_protocol.h"

#include <algorithm>
#include <cmath>

std::optional<std::vector<uint8_t>> encode_battery_telemetry(
    const sensor_msgs::msg::BatteryState& battery)
{
    const double voltage = battery.voltage;
    const double current = battery.current;
    const double capacity = battery.capacity;
    const double charge = battery.charge;
    if (!battery.present || !std::isfinite(voltage) || voltage < 0 || voltage > 3276.7 ||
        !std::isfinite(current) || std::abs(current) > 3276.7 ||
        !std::isfinite(capacity) || capacity <= 0 || !std::isfinite(charge) ||
        charge < 0 || charge > capacity) {
        return std::nullopt;
    }
    double remaining = battery.percentage;
    if (std::isnan(remaining)) {
        remaining = charge / capacity;
    }
    const double consumed_mah = (capacity - charge) * 1000.0;
    if (!std::isfinite(remaining) || remaining < 0 || remaining > 1 ||
        consumed_mah > 0xffffff) {
        return std::nullopt;
    }

    const auto volts = static_cast<uint16_t>(std::llround(voltage * 10));
    // ROS uses negative current for discharge; legacy CRSF handsets expect
    // positive consumption. Report charging as 0 A, never as unsigned wraparound.
    const auto amps = static_cast<uint16_t>(std::llround(std::max(0.0, -current) * 10));
    const auto consumed = static_cast<uint32_t>(std::llround(consumed_mah));
    std::vector<uint8_t> frame{
        CRSF_SYNC_BYTE, 10, CRSF_FRAMETYPE_BATTERY_SENSOR,
        static_cast<uint8_t>(volts >> 8), static_cast<uint8_t>(volts),
        static_cast<uint8_t>(amps >> 8), static_cast<uint8_t>(amps),
        static_cast<uint8_t>(consumed >> 16), static_cast<uint8_t>(consumed >> 8),
        static_cast<uint8_t>(consumed), static_cast<uint8_t>(std::llround(remaining * 100)), 0};
    Crc8 crc(0xd5);
    frame.back() = crc.calc(frame.data() + 2, 9);
    return frame;
}
