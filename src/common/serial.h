#pragma once

#include <cstdint>
#include <cstddef>

namespace truggy {

struct serial_port_t {
    int fd;
};

// Open serial port. Returns fd >= 0 on success, -1 on error.
int serial_open(serial_port_t& port, const char* path, int baud);

// Close serial port.
void serial_close(serial_port_t& port);

// Write bytes. Returns number of bytes written, -1 on error.
int serial_write(serial_port_t& port, const uint8_t* buf, size_t len);

// Non-blocking read. Returns number of bytes read (0 if none available), -1 on error.
int serial_read(serial_port_t& port, uint8_t* buf, size_t max_len);

// Wait for data with timeout (milliseconds). Returns true if data available.
bool serial_poll(serial_port_t& port, int timeout_ms);

// CRC-8 (Dallas/Maxim, polynomial 0x31)
uint8_t crc8(const uint8_t* data, size_t len);

} // namespace truggy
