#include <gtest/gtest.h>
#include "serial_telemetry_writer.h"

#include <fcntl.h>
#include <algorithm>
#include <cstdlib>
#include <termios.h>
#include <thread>
#include <unistd.h>

namespace {
struct Pty {
    int master = -1;
    int slave = -1;
    std::string path;
    Pty()
    {
        master = ::posix_openpt(O_RDWR | O_NOCTTY | O_NONBLOCK);
        if (master < 0 || ::grantpt(master) || ::unlockpt(master)) {
            throw std::runtime_error("cannot open test PTY");
        }
        path = ::ptsname(master);
        slave = ::open(path.c_str(), O_RDWR | O_NOCTTY);
        termios settings{};
        ::tcgetattr(slave, &settings);
        ::cfmakeraw(&settings);
        ::tcsetattr(slave, TCSANOW, &settings);
    }
    ~Pty() { if (slave >= 0) { ::close(slave); } if (master >= 0) { ::close(master); } }
    std::vector<uint8_t> read_all()
    {
        std::vector<uint8_t> result;
        uint8_t data[1024];
        for (;;) {
            auto size = ::read(master, data, sizeof(data));
            if (size <= 0) { return result; }
            result.insert(result.end(), data, data + size);
        }
    }
};

TEST(SerialTelemetryWriter, OwnsBuffersAndCoalescesWaitingFrames)
{
    asio::io_context context;
    Pty pty;
    auto writer = std::make_shared<SerialTelemetryWriter>(context, pty.path);
    const auto deadline = SerialTelemetryWriter::Clock::now() + std::chrono::seconds(3);
    writer->send(std::vector<uint8_t>(64, 0x31), deadline);
    for (unsigned int i = 0; i < 10000; ++i) {
        writer->send(std::vector<uint8_t>(64, 0x32), deadline);
    }
    writer->send(std::vector<uint8_t>(64, 0x33), deadline);
    context.run();
    auto expected = std::vector<uint8_t>(128, 0x33);
    std::fill(expected.begin(), expected.begin() + 64, 0x31);
    EXPECT_EQ(pty.read_all(), expected);
}

TEST(SerialTelemetryWriter, DiscardsExpiredFramesBeforeStarting)
{
    asio::io_context context;
    Pty pty;
    auto writer = std::make_shared<SerialTelemetryWriter>(context, pty.path);
    const auto now = SerialTelemetryWriter::Clock::now();
    writer->send({0x30}, now - std::chrono::seconds(1));
    writer->send({0x31}, now + std::chrono::seconds(3));
    writer->send({0x32}, now + std::chrono::milliseconds(10));
    std::this_thread::sleep_for(std::chrono::milliseconds(30));
    context.run();
    EXPECT_EQ(pty.read_all(), (std::vector<uint8_t>{0x31}));
}

TEST(SerialTelemetryWriter, DestructionWithPendingWriteIsSafe)
{
    asio::io_context context;
    Pty pty;
    auto writer = std::make_shared<SerialTelemetryWriter>(context, pty.path);
    writer->send(std::vector<uint8_t>(64, 0x31),
        SerialTelemetryWriter::Clock::now() + std::chrono::seconds(3));
    writer->close();
    writer.reset();
    EXPECT_NO_THROW(context.run());
}

TEST(SerialTelemetryWriter, BackpressureDoesNotBlockOtherIoOrShutdown)
{
    asio::io_context context;
    Pty pty;
    auto writer = std::make_shared<SerialTelemetryWriter>(context, pty.path);
    const int fill = ::open(pty.path.c_str(), O_WRONLY | O_NOCTTY | O_NONBLOCK);
    ASSERT_GE(fill, 0);
    std::vector<uint8_t> padding(4096, 0x31);
    while (::write(fill, padding.data(), padding.size()) > 0) {}
    ::close(fill);
    writer->send(std::vector<uint8_t>(64, 0x32),
        SerialTelemetryWriter::Clock::now() + std::chrono::seconds(3));
    bool other_io_ran = false;
    asio::post(context, [&]() { other_io_ran = true; });
    asio::steady_timer stop(context, std::chrono::milliseconds(20));
    stop.async_wait([&](const asio::error_code&) { writer->close(); });
    context.run();
    EXPECT_TRUE(other_io_ran);
    EXPECT_FALSE(writer->is_open());
}

TEST(SerialTelemetryWriter, WriteFailureClosesDescriptorAndReportsError)
{
    asio::io_context context;
    Pty pty;
    bool error_reported = false;
    auto writer = std::make_shared<SerialTelemetryWriter>(context, pty.path,
        [&](const asio::error_code& error) { error_reported = static_cast<bool>(error); });
    ::close(pty.master);
    pty.master = -1;
    writer->send({0x31}, SerialTelemetryWriter::Clock::now() + std::chrono::seconds(3));
    context.run();
    EXPECT_TRUE(error_reported);
    EXPECT_FALSE(writer->is_open());
}
}  // namespace
