/*
 * TruggyAD Serial Protocol v2 — Shared between Arduino Uno and ESP32
 *
 * Binary protocol between MCU and Jetson Nano over USB serial (115200 baud).
 * Both firmware versions produce identical packets so the Jetson-side
 * bridge (src/actuation/bridge.cpp) works unchanged.
 *
 * v2 changes: extended telemetry to 28 bytes with RC mode + RC steering
 */

#ifndef TRUGGY_PROTOCOL_H
#define TRUGGY_PROTOCOL_H

#include <stdint.h>

/* ── Sync bytes and sizes ────────────────────────────────────────────────── */

#define PROTO_CMD_SYNC   0xAA
#define PROTO_CMD_END    0x55
#define PROTO_CMD_SIZE   8

#define PROTO_TELEM_SYNC 0xBB
#define PROTO_TELEM_SIZE 28       /* v2: was 24, now 28 with RC data */

#define PROTO_SERIAL_BAUD 115200
#define PROTO_WATCHDOG_MS 250

/* ── Drive mode ──────────────────────────────────────────────────────────── */

#define PROTO_MODE_MANUAL    0x00  /* RC passthrough, Jetson commands ignored */
#define PROTO_MODE_AUTO      0x01  /* Jetson commands control servos */
#define PROTO_MODE_FAILSAFE  0x02  /* No RC signal, neutral output */

/* RC mode switch threshold: > 1700us = auto, < 1300us = manual */
#define PROTO_RC_AUTO_THRESHOLD   1700
#define PROTO_RC_MANUAL_THRESHOLD 1300

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

/* ── Telemetry packet builder (v2: 28 bytes) ─────────────────────────────── */
/*
 * Byte layout:
 *   0:     sync (0xBB)
 *   1-2:   qw (int16, *10000)
 *   3-4:   qx
 *   5-6:   qy
 *   7-8:   qz
 *   9-10:  ax (int16, *1000, m/s^2)
 *  11-12:  ay
 *  13-14:  az
 *  15-16:  gx (int16, *1000, rad/s)
 *  17-18:  gy
 *  19-20:  gz
 *  21-22:  wheel_L (int16, *100, rev/s)
 *  23:     drive_mode (MANUAL/AUTO/FAILSAFE)
 *  24-25:  rc_steering (int16, raw PWM us, 1000-2000)
 *  26:     rc_throttle_pct (int8, -100 to +100, percent of range)
 *  27:     CRC-8 over bytes 1-26
 */

static inline void proto_build_telemetry(
    uint8_t* pkt,
    float qw, float qx, float qy, float qz,
    float ax, float ay, float az,
    float gx, float gy, float gz,
    float wheel_l_rps,
    uint8_t drive_mode,
    uint16_t rc_steering_us,
    int8_t rc_throttle_pct)
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
    pkt[23] = drive_mode;
    pkt[24] = (uint8_t)(rc_steering_us >> 8);
    pkt[25] = (uint8_t)(rc_steering_us & 0xFF);
    pkt[26] = (uint8_t)rc_throttle_pct;
    pkt[27] = proto_crc8(pkt + 1, 26);
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
