#include "actuation/bridge.h"
#include "common/serial.h"
#include "common/timing.h"
#include <cstdio>
#include <cstring>

namespace truggy {

/* ── Protocol constants ──────────────────────────────────────────────────── */

static constexpr uint8_t CMD_SYNC  = 0xAA;
static constexpr uint8_t CMD_END   = 0x55;
static constexpr int     CMD_SIZE  = 8;

static constexpr uint8_t TELEM_SYNC = 0xBB;
static constexpr int     TELEM_SIZE = 24;

/* ── Telemetry parsing ───────────────────────────────────────────────────── */

static inline int16_t unpack16(const uint8_t* buf, int idx) {
    return (int16_t)((buf[idx] << 8) | buf[idx + 1]);
}

static bool parse_telemetry(const uint8_t* pkt,
                            imu_sample_t& imu, wheel_sample_t& wheels,
                            uint64_t timestamp_us) {
    /* Verify CRC */
    uint8_t expected = crc8(pkt + 1, 22);
    if (pkt[23] != expected) return false;

    /* Quaternion (scale: /10000) */
    imu.qw = (float)unpack16(pkt, 1)  / 10000.0f;
    imu.qx = (float)unpack16(pkt, 3)  / 10000.0f;
    imu.qy = (float)unpack16(pkt, 5)  / 10000.0f;
    imu.qz = (float)unpack16(pkt, 7)  / 10000.0f;

    /* Acceleration (scale: /1000, m/s^2) */
    imu.ax = (float)unpack16(pkt, 9)  / 1000.0f;
    imu.ay = (float)unpack16(pkt, 11) / 1000.0f;
    imu.az = (float)unpack16(pkt, 13) / 1000.0f;

    /* Gyroscope (scale: /1000, rad/s) */
    imu.gx = (float)unpack16(pkt, 15) / 1000.0f;
    imu.gy = (float)unpack16(pkt, 17) / 1000.0f;
    imu.gz = (float)unpack16(pkt, 19) / 1000.0f;

    imu.timestamp_us = timestamp_us;

    /* Wheel speed (scale: /100, rev/s) */
    wheels.left_rps  = (float)unpack16(pkt, 21) / 100.0f;
    wheels.right_rps = 0.0f;  /* single wheel in 24-byte packet */
    wheels.timestamp_us = timestamp_us;

    return true;
}

/* ── Command encoding ────────────────────────────────────────────────────── */

static void encode_command(uint8_t* buf, float steering, float throttle,
                           bool armed) {
    buf[0] = CMD_SYNC;

    int16_t steer_raw = (int16_t)(steering * 10000.0f);
    int16_t throt_raw = (int16_t)(throttle * 10000.0f);

    buf[1] = (uint8_t)(steer_raw >> 8);
    buf[2] = (uint8_t)(steer_raw & 0xFF);
    buf[3] = (uint8_t)(throt_raw >> 8);
    buf[4] = (uint8_t)(throt_raw & 0xFF);
    buf[5] = armed ? 0x01 : 0x00;
    buf[6] = crc8(buf + 1, 5);
    buf[7] = CMD_END;
}

/* ── Receive buffer with sync-byte scanning ──────────────────────────────── */

struct rx_state_t {
    uint8_t buf[256];
    int len;
};

static int scan_telemetry(rx_state_t& rx, imu_sample_t& imu,
                          wheel_sample_t& wheels, uint64_t ts) {
    int parsed = 0;

    while (rx.len >= TELEM_SIZE) {
        /* Find sync byte */
        int sync_pos = -1;
        for (int i = 0; i <= rx.len - TELEM_SIZE; i++) {
            if (rx.buf[i] == TELEM_SYNC) {
                sync_pos = i;
                break;
            }
        }

        if (sync_pos < 0) {
            /* No sync found, keep last few bytes in case partial */
            if (rx.len > TELEM_SIZE - 1) {
                int keep = TELEM_SIZE - 1;
                memmove(rx.buf, rx.buf + rx.len - keep, keep);
                rx.len = keep;
            }
            break;
        }

        /* Discard bytes before sync */
        if (sync_pos > 0) {
            memmove(rx.buf, rx.buf + sync_pos, rx.len - sync_pos);
            rx.len -= sync_pos;
        }

        /* Try to parse */
        if (rx.len >= TELEM_SIZE) {
            if (parse_telemetry(rx.buf, imu, wheels, ts)) {
                parsed++;
            }
            /* Consume the packet regardless of CRC result */
            memmove(rx.buf, rx.buf + TELEM_SIZE, rx.len - TELEM_SIZE);
            rx.len -= TELEM_SIZE;
        }
    }

    return parsed;
}

/* ── Actuation loop (Thread 3) ───────────────────────────────────────────── */

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

    rx_state_t rx = {};
    uint64_t cmd_period_us = 1000000 / cfg.serial.command_rate;
    uint64_t next_cmd = now_us();
    uint32_t telem_count = 0;
    uint32_t cmd_count = 0;

    while (bus->alive.load(std::memory_order_relaxed)) {
        uint64_t ts = now_us();

        /* ── Read telemetry from Arduino ─────────────────────────────────── */
        int n = serial_read(port, rx.buf + rx.len,
                           sizeof(rx.buf) - rx.len);
        if (n > 0) rx.len += n;

        imu_sample_t imu;
        wheel_sample_t wheels;
        int parsed = scan_telemetry(rx, imu, wheels, ts);

        if (parsed > 0) {
            /* Write to shared bus */
            seqlock_write(bus->imu_lock, bus->imu, imu);
            seqlock_write(bus->wheel_lock, bus->wheels, wheels);
            telem_count += parsed;
        }

        /* ── Send commands to Arduino ────────────────────────────────────── */
        if (ts >= next_cmd) {
            control_2d_t ctrl = seqlock_read(bus->control_lock, bus->control);
            bool armed = bus->armed.load(std::memory_order_relaxed);

            float steer = armed ? ctrl.steering : 0.0f;
            float throt = armed ? ctrl.throttle : 0.0f;

            uint8_t cmd[CMD_SIZE];
            encode_command(cmd, steer, throt, armed);
            serial_write(port, cmd, CMD_SIZE);

            cmd_count++;
            next_cmd += cmd_period_us;
            if (ts > next_cmd + cmd_period_us * 2) {
                next_cmd = ts + cmd_period_us;
            }
        }

        /* ── Timing ──────────────────────────────────────────────────────── */
        /* Print stats every 5 seconds */
        if (telem_count > 0 && telem_count % 500 == 0) {
            fprintf(stderr, "[T3] telem=%u cmd=%u\n", telem_count, cmd_count);
        }

        /* Small sleep to avoid busy-waiting, but keep responsive */
        if (!serial_poll(port, 1)) {
            /* No data available, brief sleep */
        }
    }

    /* Send neutral command before closing */
    uint8_t neutral[CMD_SIZE];
    encode_command(neutral, 0.0f, 0.0f, false);
    serial_write(port, neutral, CMD_SIZE);

    serial_close(port);
    fprintf(stderr, "[T3] Actuation thread stopped (telem=%u, cmd=%u)\n",
            telem_count, cmd_count);
}

} // namespace truggy
