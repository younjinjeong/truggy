/*
 * TruggyAD Serial Protocol — Shared between Arduino Uno and ESP32
 *
 * Binary protocol between MCU and Jetson Nano over USB serial (115200 baud).
 * Both firmware versions produce identical packets so the Jetson-side
 * bridge (src/actuation/bridge.cpp) works unchanged.
 */

#ifndef TRUGGY_PROTOCOL_H
#define TRUGGY_PROTOCOL_H

#include <stdint.h>

/* ── Sync bytes and sizes ────────────────────────────────────────────────── */

#define PROTO_CMD_SYNC   0xAA
#define PROTO_CMD_END    0x55
#define PROTO_CMD_SIZE   8

#define PROTO_TELEM_SYNC 0xBB
#define PROTO_TELEM_SIZE 24

#define PROTO_SERIAL_BAUD 115200
#define PROTO_WATCHDOG_MS 250

/* ── CRC-8 (Dallas/Maxim, polynomial 0x31) ───────────────────────────────── */

static inline uint8_t proto_crc8(const uint8_t* data, uint8_t len) {
    uint8_t crc = 0x00;
    for (uint8_t i = 0; i < len; i++) {
        crc ^= data[i];
        for (uint8_t j = 0; j < 8; j++) {
            if (crc & 0x80)
                crc = (crc << 1) ^ 0x31;
            else
                crc <<= 1;
        }
    }
    return crc;
}

/* ── Packing helpers (big-endian int16) ──────────────────────────────────── */

static inline void proto_pack16(uint8_t* buf, int idx, float val, float scale) {
    int16_t v = (int16_t)(val * scale);
    buf[idx]     = (uint8_t)(v >> 8);
    buf[idx + 1] = (uint8_t)(v & 0xFF);
}

static inline int16_t proto_unpack16(const uint8_t* buf, int idx) {
    return (int16_t)((buf[idx] << 8) | buf[idx + 1]);
}

/* ── Telemetry packet builder ────────────────────────────────────────────── */

static inline void proto_build_telemetry(
    uint8_t* pkt,
    float qw, float qx, float qy, float qz,
    float ax, float ay, float az,
    float gx, float gy, float gz,
    float wheel_l_rps)
{
    pkt[0] = PROTO_TELEM_SYNC;
    proto_pack16(pkt, 1,  qw, 10000.0f);
    proto_pack16(pkt, 3,  qx, 10000.0f);
    proto_pack16(pkt, 5,  qy, 10000.0f);
    proto_pack16(pkt, 7,  qz, 10000.0f);
    proto_pack16(pkt, 9,  ax, 1000.0f);
    proto_pack16(pkt, 11, ay, 1000.0f);
    proto_pack16(pkt, 13, az, 1000.0f);
    proto_pack16(pkt, 15, gx, 1000.0f);
    proto_pack16(pkt, 17, gy, 1000.0f);
    proto_pack16(pkt, 19, gz, 1000.0f);
    proto_pack16(pkt, 21, wheel_l_rps, 100.0f);
    pkt[23] = proto_crc8(pkt + 1, 22);
}

/* ── Command packet parser ───────────────────────────────────────────────── */

typedef struct {
    float steering;   /* [-0.99, 0.99] */
    float throttle;   /* [-0.99, max_throttle] */
    uint8_t flags;    /* bit0=arm, bit1=estop */
    uint8_t valid;    /* 1 if parse succeeded */
} proto_command_t;

static inline proto_command_t proto_parse_command(const uint8_t* cmd) {
    proto_command_t result;
    result.valid = 0;

    if (cmd[0] != PROTO_CMD_SYNC || cmd[PROTO_CMD_SIZE - 1] != PROTO_CMD_END)
        return result;

    uint8_t expected_crc = proto_crc8(cmd + 1, 5);
    if (cmd[6] != expected_crc)
        return result;

    result.steering = (float)proto_unpack16(cmd, 1) / 10000.0f;
    result.throttle = (float)proto_unpack16(cmd, 3) / 10000.0f;
    result.flags = cmd[5];
    result.valid = 1;
    return result;
}

#endif /* TRUGGY_PROTOCOL_H */
