#include <gtest/gtest.h>
#include "battery_telemetry.h"

#include <limits>

namespace {
sensor_msgs::msg::BatteryState battery()
{
    sensor_msgs::msg::BatteryState msg;
    msg.present = true;
    msg.voltage = 25.2f;
    msg.current = -12.3f;
    msg.capacity = 10.0f;
    msg.charge = 7.5f;
    msg.percentage = 0.75f;
    return msg;
}
uint8_t checksum(const std::vector<uint8_t>& frame)
{
    uint8_t crc = 0;
    for (std::size_t i = 2; i + 1 < frame.size(); ++i) {
        crc ^= frame[i];
        for (unsigned int bit = 0; bit < 8; ++bit) {
            crc = (crc << 1) ^ ((crc & 0x80) ? 0xd5 : 0);
        }
    }
    return crc;
}

TEST(BatteryTelemetry, EncodesWireLayoutUnitsAndCrc)
{
    auto frame = encode_battery_telemetry(battery());
    ASSERT_TRUE(frame);
    ASSERT_EQ(frame->size(), 12u);
    // 25.2 V, 12.3 A discharge, 2500 mAh consumed, 75% remaining.
    std::vector<uint8_t> expected{0xc8, 10, 8, 0, 252, 0, 123, 0, 9, 196, 75, 0};
    expected.back() = checksum(expected);
    EXPECT_EQ(*frame, expected);
}

TEST(BatteryTelemetry, DerivesUnknownPercentageFromRemainingCharge)
{
    auto msg = battery();
    msg.percentage = std::numeric_limits<float>::quiet_NaN();
    auto frame = encode_battery_telemetry(msg);
    ASSERT_TRUE(frame);
    EXPECT_EQ((*frame)[10], 75);
}

TEST(BatteryTelemetry, ChargingReportsZeroConsumptionCurrent)
{
    auto msg = battery();
    msg.current = 12.3f;
    auto frame = encode_battery_telemetry(msg);
    ASSERT_TRUE(frame);
    EXPECT_EQ((*frame)[5], 0);
    EXPECT_EQ((*frame)[6], 0);
}

TEST(BatteryTelemetry, FullAndEmptyBatteryAreValid)
{
    auto msg = battery();
    msg.charge = msg.capacity;
    msg.percentage = 1.0f;
    auto frame = encode_battery_telemetry(msg);
    ASSERT_TRUE(frame);
    EXPECT_EQ((*frame)[7], 0);
    EXPECT_EQ((*frame)[8], 0);
    EXPECT_EQ((*frame)[9], 0);
    EXPECT_EQ((*frame)[10], 100);
    msg.charge = 0;
    msg.percentage = 0;
    frame = encode_battery_telemetry(msg);
    ASSERT_TRUE(frame);
    EXPECT_EQ((*frame)[10], 0);
}

TEST(BatteryTelemetry, DoesNotTransmitAbsentBattery)
{
    auto msg = battery();
    msg.present = false;
    EXPECT_FALSE(encode_battery_telemetry(msg));
}

TEST(BatteryTelemetry, RejectsUnknownMandatoryMeasurements)
{
    for (auto field : {&sensor_msgs::msg::BatteryState::voltage,
            &sensor_msgs::msg::BatteryState::current, &sensor_msgs::msg::BatteryState::capacity,
            &sensor_msgs::msg::BatteryState::charge}) {
        for (auto value : {std::numeric_limits<float>::quiet_NaN(),
                std::numeric_limits<float>::infinity()}) {
            auto msg = battery();
            msg.*field = value;
            EXPECT_FALSE(encode_battery_telemetry(msg));
        }
    }
}

TEST(BatteryTelemetry, RejectsInconsistentOrUnrepresentableInputs)
{
    for (auto value : {-1.0f, 4000.0f}) {
        auto msg = battery(); msg.voltage = value;
        EXPECT_FALSE(encode_battery_telemetry(msg));
    }
    for (auto value : {-4000.0f, 4000.0f}) {
        auto msg = battery(); msg.current = value;
        EXPECT_FALSE(encode_battery_telemetry(msg));
    }
    for (auto value : {-1.0f, 0.0f}) {
        auto msg = battery(); msg.capacity = value;
        EXPECT_FALSE(encode_battery_telemetry(msg));
    }
    for (auto value : {-1.0f, 11.0f}) {
        auto msg = battery(); msg.charge = value;
        EXPECT_FALSE(encode_battery_telemetry(msg));
    }
    for (auto value : {-0.1f, 1.1f, std::numeric_limits<float>::infinity()}) {
        auto msg = battery(); msg.percentage = value;
        EXPECT_FALSE(encode_battery_telemetry(msg));
    }
    auto msg = battery(); msg.capacity = 20000; msg.charge = 0;
    EXPECT_FALSE(encode_battery_telemetry(msg));
}
}  // namespace
