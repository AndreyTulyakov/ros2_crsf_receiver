#ifndef SERIAL_TELEMETRY_WRITER_HPP
#define SERIAL_TELEMETRY_WRITER_HPP

#include <asio.hpp>
#include <chrono>
#include <functional>
#include <memory>
#include <optional>
#include <string>
#include <vector>

// Use through shared_ptr. All methods run on the serial I/O thread, except
// close() may be called after that thread has been stopped and joined.
// A separate nonblocking descriptor shares the existing UART settings without
// changing serial_driver or depending on its unsafe async_send buffer lifetime.
class SerialTelemetryWriter : public std::enable_shared_from_this<SerialTelemetryWriter>
{
public:
    using Clock = std::chrono::steady_clock;
    using ErrorHandler = std::function<void(const asio::error_code&)>;
    SerialTelemetryWriter(asio::io_context& context, const std::string& device,
        ErrorHandler on_error = {});
    bool is_open() const { return descriptor_.is_open(); }
    void send(std::vector<uint8_t> frame, Clock::time_point expires_at);
    void close();

private:
    struct Pending {
        std::vector<uint8_t> frame;
        Clock::time_point expires_at;
    };
    asio::posix::stream_descriptor descriptor_;
    ErrorHandler on_error_;
    bool active_ = false;
    std::optional<Pending> pending_;
    void start(Pending packet);
};

#endif
