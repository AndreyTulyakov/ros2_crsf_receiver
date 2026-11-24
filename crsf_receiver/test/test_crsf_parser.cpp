#include <gtest/gtest.h>
#include "crsf_parser.h"

#include <array>
#include <algorithm>
#include <random>
#include <stdexcept>
#include <thread>

namespace {
using Bytes = std::vector<uint8_t>;
using Channels = std::array<int, CRSF_NUM_CHANNELS>;

Bytes frame(uint8_t type, const Bytes& payload)
{
    Bytes data{CRSF_SYNC_BYTE, static_cast<uint8_t>(payload.size() + 2), type};
    data.insert(data.end(), payload.begin(), payload.end());
    uint8_t crc = 0;
    for (std::size_t i = 2; i < data.size(); ++i) {
        crc ^= data[i];
        for (int bit = 0; bit < 8; ++bit) {
            crc = (crc << 1) ^ ((crc & 0x80) ? 0xd5 : 0);
        }
    }
    data.push_back(crc);
    return data;
}

Bytes channels_frame(const Channels& channels)
{
    Bytes payload(22, 0);
    for (std::size_t ch = 0; ch < channels.size(); ++ch) {
        for (unsigned int bit = 0; bit < 11; ++bit) {
            if (channels[ch] & (1u << bit)) {
                payload[(11 * ch + bit) / 8] |= 1u << ((11 * ch + bit) % 8);
            }
        }
    }
    return frame(0x16, payload);
}

void feed(CrsfParser& parser, const Bytes& data)
{
    parser.rx_buffer.insert(parser.rx_buffer.end(), data.begin(), data.end());
    parser.parse_incoming_bytes();
}

const Channels values{172, 992, 1811, 0, 2047, 100, 200, 300,
    400, 500, 600, 700, 800, 900, 1000, 1100};
const Bytes link_payload{70, 75, 99, 249, 1, 2, 3, 80, 95, 252};

TEST(CrsfParser, StartupIsInvalidAndStorageIsInitialized)
{
    CrsfParser parser;
    EXPECT_FALSE(parser.is_channels_actual());
    EXPECT_FALSE(parser.is_link_statistics_actual());
    EXPECT_FALSE(parser.is_link_up());
    for (unsigned int ch = 1; ch <= 16; ++ch) {
        EXPECT_EQ(parser.get_channel_value(ch), 0);
    }
    EXPECT_EQ(parser.get_link_info().uplink_status, 0);
}

TEST(CrsfParser, DecodesAllChannelsAtEverySerialSplit)
{
    auto data = channels_frame(values);
    for (std::size_t split = 0; split < data.size(); ++split) {
        CrsfParser parser;
        feed(parser, Bytes(data.begin(), data.begin() + split));
        EXPECT_FALSE(parser.is_channels_actual());
        feed(parser, Bytes(data.begin() + split, data.end()));
        ASSERT_TRUE(parser.is_channels_actual());
        for (std::size_t ch = 0; ch < values.size(); ++ch) {
            EXPECT_EQ(parser.get_channel_value(ch + 1), values[ch]);
        }
        EXPECT_EQ(parser.channels_sequence(), 1u);
        EXPECT_TRUE(parser.rx_buffer.empty());
    }
}

TEST(CrsfParser, HandlesByteByByteInputAndConcatenatedFrames)
{
    CrsfParser parser;
    auto data = channels_frame(values);
    for (auto byte : data) { feed(parser, {byte}); }
    Bytes batch = data;
    auto link = frame(0x14, link_payload);
    batch.insert(batch.end(), link.begin(), link.end());
    batch.insert(batch.end(), data.begin(), data.end());
    feed(parser, batch);
    EXPECT_EQ(parser.channels_sequence(), 3u);
    EXPECT_EQ(parser.link_statistics_sequence(), 1u);
    EXPECT_TRUE(parser.rx_buffer.empty());
}

TEST(CrsfParser, LinkStatisticsHasSignedSnrAndDoesNotEnableControl)
{
    CrsfParser parser;
    feed(parser, frame(0x14, link_payload));
    auto link = parser.get_link_info();
    EXPECT_EQ(link.uplink_rssi_ant1, 70);
    EXPECT_EQ(link.uplink_rssi_ant2, 75);
    EXPECT_EQ(link.uplink_status, 99);
    EXPECT_EQ(link.uplink_snr, -7);
    EXPECT_EQ(link.active_antenna, 1);
    EXPECT_EQ(link.rf_mode, 2);
    EXPECT_EQ(link.uplink_tx_power, 3);
    EXPECT_EQ(link.downlink_rssi, 80);
    EXPECT_EQ(link.downlink_status, 95);
    EXPECT_EQ(link.downlink_snr, -4);
    EXPECT_TRUE(parser.is_link_statistics_actual());
    EXPECT_FALSE(parser.is_channels_actual());
    EXPECT_FALSE(parser.is_link_up());
}

TEST(CrsfParser, RejectsEveryUndersizedKnownPayload)
{
    for (std::size_t size = 0; size < 22; ++size) {
        CrsfParser parser;
        feed(parser, frame(0x16, Bytes(size, 0)));
        EXPECT_FALSE(parser.is_channels_actual()) << size;
    }
    for (std::size_t size = 0; size < 10; ++size) {
        CrsfParser parser;
        feed(parser, frame(0x14, Bytes(size, 0)));
        EXPECT_FALSE(parser.is_link_statistics_actual()) << size;
    }
}

TEST(CrsfParser, AllowsFutureTrailingPayloadFields)
{
    CrsfParser parser;
    auto data = channels_frame(values);
    Bytes payload(data.begin() + 3, data.end() - 1);
    payload.push_back(1);
    feed(parser, frame(0x16, payload));
    EXPECT_EQ(parser.get_channel_value(16), values[15]);
    auto link = link_payload;
    link.push_back(42);
    feed(parser, frame(0x14, link));
    EXPECT_TRUE(parser.is_link_statistics_actual());
}

TEST(CrsfParser, GodboltRepeatedSyncDoesNotLoopAndRecovers)
{
    CrsfParser parser;
    feed(parser, {0xc8, 0xc8, 0x01, 0x02});
    EXPECT_TRUE(parser.rx_buffer.empty());
    feed(parser, channels_frame(values));
    EXPECT_EQ(parser.channels_sequence(), 1u);
}

TEST(CrsfParser, RejectsCrcErrorsWithoutDiscardingFollowingFrame)
{
    CrsfParser parser;
    auto data = channels_frame(values);
    auto corrupt = data;
    corrupt.back() ^= 1;
    feed(parser, corrupt);
    EXPECT_FALSE(parser.is_channels_actual());

    Bytes batch{0xc8, 30, 1};
    batch.insert(batch.end(), data.begin(), data.end());
    batch.resize(32, 0);
    feed(parser, batch);
    EXPECT_EQ(parser.channels_sequence(), 1u);
    EXPECT_EQ(parser.get_channel_value(3), 1811);
}

TEST(CrsfParser, IncompleteFrameExpiresAndPreservesFollowingValidFrame)
{
    CrsfParser parser;
    Bytes batch{0xc8, 62, 1};
    auto data = channels_frame(values);
    batch.insert(batch.end(), data.begin(), data.end());
    feed(parser, batch);
    EXPECT_FALSE(parser.is_channels_actual());
    std::this_thread::sleep_for(std::chrono::milliseconds(120));
    parser.parse_incoming_bytes();
    EXPECT_EQ(parser.channels_sequence(), 1u);
    EXPECT_FALSE(parser.is_channels_actual());
    EXPECT_TRUE(parser.rx_buffer.empty());
}

TEST(CrsfParser, FreshnessExpiresAndTelemetryDoesNotExtendControl)
{
    CrsfParser parser;
    feed(parser, channels_frame(values));
    EXPECT_TRUE(parser.is_channels_actual());
    EXPECT_TRUE(parser.is_link_up());
    std::this_thread::sleep_for(std::chrono::milliseconds(70));
    EXPECT_FALSE(parser.is_channels_actual());
    std::this_thread::sleep_for(std::chrono::milliseconds(150));
    feed(parser, frame(0x14, link_payload));
    EXPECT_FALSE(parser.is_link_up());
    EXPECT_TRUE(parser.is_link_statistics_actual());
    std::this_thread::sleep_for(std::chrono::milliseconds(1050));
    EXPECT_FALSE(parser.is_link_statistics_actual());
}

TEST(CrsfParser, UnknownMinimumAndMaximumFramesAreConsumedWithoutControl)
{
    CrsfParser parser;
    feed(parser, frame(0x0b, {}));
    feed(parser, frame(0x7f, Bytes(60, 0)));
    EXPECT_TRUE(parser.rx_buffer.empty());
    EXPECT_FALSE(parser.is_link_up());
}

TEST(CrsfParser, RejectsInvalidChannelIndexes)
{
    CrsfParser parser;
    EXPECT_THROW(parser.get_channel_value(0), std::out_of_range);
    EXPECT_THROW(parser.get_channel_value(17), std::out_of_range);
}

TEST(CrsfParser, RandomChannelsAndChunkBoundariesRemainExact)
{
    std::mt19937 random(20261124);
    CrsfParser parser;
    for (unsigned int iteration = 0; iteration < 2000; ++iteration) {
        Channels expected{};
        for (auto& value : expected) { value = random() % 2048; }
        auto data = channels_frame(expected);
        std::size_t offset = 0;
        while (offset < data.size()) {
            auto end = std::min(data.size(), offset + 1 + random() % 9);
            feed(parser, Bytes(data.begin() + offset, data.begin() + end));
            offset = end;
        }
        ASSERT_EQ(parser.channels_sequence(), iteration + 1);
        for (unsigned int ch = 1; ch <= 16; ++ch) {
            ASSERT_EQ(parser.get_channel_value(ch), expected[ch - 1]);
        }
    }
}

TEST(CrsfParser, NoiseCannotGrowPendingBufferBeyondOneFrame)
{
    std::mt19937 random(42);
    CrsfParser parser;
    for (unsigned int iteration = 0; iteration < 10000; ++iteration) {
        Bytes noise(13);
        for (auto& byte : noise) { byte = random(); }
        feed(parser, noise);
        ASSERT_LT(parser.rx_buffer.size(), CRSF_MAX_PACKET_SIZE);
    }
    // A full maximum-size frame of junk forces any pending candidate to finish.
    feed(parser, Bytes(64, 0xff));
    feed(parser, channels_frame(values));
    EXPECT_EQ(parser.get_channel_value(3), 1811);
}
}  // namespace
