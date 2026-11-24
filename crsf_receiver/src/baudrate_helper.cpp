#include "baudrate_helper.h"
#include <asm/termbits.h>
#include <sys/ioctl.h>
#include <fcntl.h>
#include <unistd.h>

bool set_custom_baudrate(const char* device, unsigned int baudrate)
{
    if (baudrate == 0) { return false; }
    const int fd = ::open(device, O_RDWR | O_NOCTTY | O_CLOEXEC);
    if (fd < 0) { return false; }

    struct termios2 tio{};
    bool ok = ::ioctl(fd, TCGETS2, &tio) == 0;
    if (ok) {
        tio.c_cflag &= ~CBAUD;
        tio.c_cflag |= BOTHER;
        tio.c_ispeed = baudrate;
        tio.c_ospeed = baudrate;
        ok = ::ioctl(fd, TCSETS2, &tio) == 0;
    }
    if (ok) {
        ok = ::ioctl(fd, TCGETS2, &tio) == 0 &&
            tio.c_ispeed == baudrate && tio.c_ospeed == baudrate;
    }
    ::close(fd);
    return ok;
}
