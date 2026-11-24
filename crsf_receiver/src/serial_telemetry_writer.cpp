#include "serial_telemetry_writer.h"

#include <cerrno>
#include <fcntl.h>
#include <system_error>
#include <unistd.h>
#include <utility>

SerialTelemetryWriter::SerialTelemetryWriter(
    asio::io_context& context, const std::string& device, ErrorHandler on_error)
: descriptor_(context), on_error_(std::move(on_error))
{
    const int fd = ::open(device.c_str(), O_WRONLY | O_NOCTTY | O_NONBLOCK | O_CLOEXEC);
    if (fd < 0) { throw std::system_error(errno, std::generic_category(), "open telemetry UART"); }
    asio::error_code error;
    descriptor_.assign(fd, error);
    if (error) {
        ::close(fd);
        throw std::system_error(error);
    }
}

void SerialTelemetryWriter::send(std::vector<uint8_t> frame, Clock::time_point expires_at)
{
    if (!is_open() || frame.empty() || frame.size() > 64 || Clock::now() >= expires_at) {
        return;
    }
    Pending packet{std::move(frame), expires_at};
    if (active_) {
        // Battery telemetry is a sample stream: retain only the newest waiting
        // sample. At most one active frame and one pending frame are owned.
        pending_ = std::move(packet);
    } else {
        start(std::move(packet));
    }
}

void SerialTelemetryWriter::start(Pending packet)
{
    if (!is_open() || Clock::now() >= packet.expires_at) { return; }
    active_ = true;
    auto bytes = std::make_shared<std::vector<uint8_t>>(std::move(packet.frame));
    // Composed async_write handles partial writes. The completion owns the
    // bytes even if the writer is closed or destroyed while an operation waits.
    asio::async_write(descriptor_, asio::buffer(*bytes),
        [weak = weak_from_this(), bytes](const asio::error_code& error, std::size_t) {
            if (auto self = weak.lock()) {
                self->active_ = false;
                if (error) {
                    self->close();
                    if (error != asio::error::operation_aborted && self->on_error_) {
                        self->on_error_(error);
                    }
                } else if (self->pending_) {
                    auto next = std::move(*self->pending_);
                    self->pending_.reset();
                    self->start(std::move(next));
                }
            }
        });
}

void SerialTelemetryWriter::close()
{
    pending_.reset();
    asio::error_code ignored;
    descriptor_.close(ignored);
}
