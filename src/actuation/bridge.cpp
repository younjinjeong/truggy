#include "actuation/bridge.h"
#include "common/serial.h"
#include "common/timing.h"
#include <cstdio>

namespace truggy {

void actuation_loop(shared_bus_t* bus, const config_t& cfg) {
    fprintf(stderr, "[T3] Actuation thread starting\n");

    serial_port_t port;
    int fd = serial_open(port, cfg.serial.port.c_str(), cfg.serial.baud);
    if (fd < 0) {
        fprintf(stderr, "[T3] Failed to open serial port %s\n",
                cfg.serial.port.c_str());
        return;
    }
    fprintf(stderr, "[T3] Serial port %s opened (fd=%d)\n",
            cfg.serial.port.c_str(), fd);

    uint64_t period_us = 1000000 / cfg.serial.telemetry_rate;
    uint64_t next_tick = now_us();

    while (bus->alive.load(std::memory_order_relaxed)) {
        scoped_timer_t timer("actuation", 2.0);

        // TODO(epic2): parse telemetry, write IMU/wheel to bus
        // TODO(epic2): read control from bus, encode and send command

        next_tick += period_us;
        sleep_until_us(next_tick);
    }

    serial_close(port);
    fprintf(stderr, "[T3] Actuation thread stopped\n");
}

} // namespace truggy
