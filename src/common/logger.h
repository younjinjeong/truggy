#pragma once

#include "common/types.h"
#include <cstdio>
#include <cstdint>

namespace truggy {

// Binary telemetry logger for offline analysis
// Writes state + control + timing to a binary file at configurable rate

struct log_entry_t {
    uint64_t timestamp_us;
    // State (7 floats)
    float x, y, yaw, roll, u_x, u_y, yaw_rate;
    // Control (2 floats)
    float steering, throttle;
    // Timing (3 floats)
    float perception_ms, planning_ms, state_ms;
};

struct logger_t {
    FILE* fp;
    uint64_t next_log_us;
    uint64_t interval_us;
    uint32_t count;
};

inline bool logger_open(logger_t& log, const char* path, int hz) {
    log.fp = fopen(path, "wb");
    if (!log.fp) {
        fprintf(stderr, "[logger] Failed to open %s\n", path);
        return false;
    }
    log.interval_us = 1000000 / hz;
    log.next_log_us = 0;
    log.count = 0;

    // Write header: magic + version + entry size
    uint32_t magic = 0x54524731;  // "TRG1"
    uint32_t version = 1;
    uint32_t entry_size = sizeof(log_entry_t);
    fwrite(&magic, 4, 1, log.fp);
    fwrite(&version, 4, 1, log.fp);
    fwrite(&entry_size, 4, 1, log.fp);

    fprintf(stderr, "[logger] Opened %s at %d Hz\n", path, hz);
    return true;
}

inline void logger_write(logger_t& log, uint64_t now,
                          const state_7d_t& state,
                          const control_2d_t& control) {
    if (now < log.next_log_us) return;
    if (!log.fp) return;

    log_entry_t entry;
    entry.timestamp_us = now;
    entry.x = state.x;
    entry.y = state.y;
    entry.yaw = state.yaw;
    entry.roll = state.roll;
    entry.u_x = state.u_x;
    entry.u_y = state.u_y;
    entry.yaw_rate = state.yaw_rate;
    entry.steering = control.steering;
    entry.throttle = control.throttle;
    entry.perception_ms = 0;
    entry.planning_ms = 0;
    entry.state_ms = 0;

    fwrite(&entry, sizeof(entry), 1, log.fp);
    log.count++;
    log.next_log_us = now + log.interval_us;
}

inline void logger_close(logger_t& log) {
    if (log.fp) {
        fclose(log.fp);
        fprintf(stderr, "[logger] Closed (%u entries written)\n", log.count);
        log.fp = nullptr;
    }
}

} // namespace truggy
