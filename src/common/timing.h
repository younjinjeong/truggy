#pragma once

#include <cstdint>
#include <cstdio>
#include <time.h>

namespace truggy {

inline uint64_t now_us() {
    struct timespec ts;
    clock_gettime(CLOCK_MONOTONIC, &ts);
    return (uint64_t)ts.tv_sec * 1000000ULL + (uint64_t)ts.tv_nsec / 1000ULL;
}

inline double elapsed_ms(uint64_t start_us) {
    return (double)(now_us() - start_us) / 1000.0;
}

struct scoped_timer_t {
    const char* name;
    double budget_ms;
    uint64_t start;

    scoped_timer_t(const char* name_, double budget_ms_)
        : name(name_), budget_ms(budget_ms_), start(now_us()) {}

    ~scoped_timer_t() {
        double dt = elapsed_ms(start);
        if (dt > budget_ms) {
            fprintf(stderr, "[TIMING] %s: %.1f ms (budget: %.1f ms)\n",
                    name, dt, budget_ms);
        }
    }
};

// Sleep until target microsecond timestamp
inline void sleep_until_us(uint64_t target_us) {
    uint64_t now = now_us();
    if (now >= target_us) return;
    struct timespec ts;
    uint64_t diff = target_us - now;
    ts.tv_sec = diff / 1000000ULL;
    ts.tv_nsec = (diff % 1000000ULL) * 1000;
    clock_nanosleep(CLOCK_MONOTONIC, 0, &ts, nullptr);
}

} // namespace truggy
