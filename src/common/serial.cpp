#include "common/serial.h"

#include <fcntl.h>
#include <termios.h>
#include <unistd.h>
#include <poll.h>
#include <cstdio>
#include <cerrno>
#include <cstring>

namespace truggy {

static speed_t baud_to_speed(int baud) {
    switch (baud) {
        case 9600:   return B9600;
        case 19200:  return B19200;
        case 38400:  return B38400;
        case 57600:  return B57600;
        case 115200: return B115200;
        case 230400: return B230400;
        case 460800: return B460800;
        default:     return B115200;
    }
}

int serial_open(serial_port_t& port, const char* path, int baud) {
    int fd = open(path, O_RDWR | O_NOCTTY | O_NONBLOCK);
    if (fd < 0) {
        fprintf(stderr, "serial_open: cannot open %s: %s\n", path, strerror(errno));
        return -1;
    }

    struct termios tty;
    if (tcgetattr(fd, &tty) != 0) {
        fprintf(stderr, "serial_open: tcgetattr failed: %s\n", strerror(errno));
        close(fd);
        return -1;
    }

    speed_t spd = baud_to_speed(baud);
    cfsetispeed(&tty, spd);
    cfsetospeed(&tty, spd);

    // 8N1, no flow control, raw mode
    tty.c_cflag = (tty.c_cflag & ~CSIZE) | CS8;
    tty.c_cflag &= ~(PARENB | CSTOPB | CRTSCTS);
    tty.c_cflag |= (CLOCAL | CREAD);

    tty.c_iflag &= ~(IXON | IXOFF | IXANY | IGNBRK | BRKINT |
                      PARMRK | ISTRIP | INLCR | IGNCR | ICRNL);
    tty.c_oflag &= ~OPOST;
    tty.c_lflag &= ~(ECHO | ECHONL | ICANON | ISIG | IEXTEN);

    // Non-blocking: return immediately with whatever is available
    tty.c_cc[VMIN]  = 0;
    tty.c_cc[VTIME] = 0;

    if (tcsetattr(fd, TCSANOW, &tty) != 0) {
        fprintf(stderr, "serial_open: tcsetattr failed: %s\n", strerror(errno));
        close(fd);
        return -1;
    }

    tcflush(fd, TCIOFLUSH);

    port.fd = fd;
    return fd;
}

void serial_close(serial_port_t& port) {
    if (port.fd >= 0) {
        close(port.fd);
        port.fd = -1;
    }
}

int serial_write(serial_port_t& port, const uint8_t* buf, size_t len) {
    return (int)write(port.fd, buf, len);
}

int serial_read(serial_port_t& port, uint8_t* buf, size_t max_len) {
    ssize_t n = read(port.fd, buf, max_len);
    if (n < 0) {
        if (errno == EAGAIN || errno == EWOULDBLOCK) return 0;
        return -1;
    }
    return (int)n;
}

bool serial_poll(serial_port_t& port, int timeout_ms) {
    struct pollfd pfd;
    pfd.fd = port.fd;
    pfd.events = POLLIN;
    return poll(&pfd, 1, timeout_ms) > 0 && (pfd.revents & POLLIN);
}

uint8_t crc8(const uint8_t* data, size_t len) {
    uint8_t crc = 0x00;
    for (size_t i = 0; i < len; i++) {
        crc ^= data[i];
        for (int j = 0; j < 8; j++) {
            if (crc & 0x80)
                crc = (crc << 1) ^ 0x31;
            else
                crc <<= 1;
        }
    }
    return crc;
}

} // namespace truggy
