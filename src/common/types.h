#pragma once

#include <atomic>
#include <cstdint>
#include <cstring>

namespace truggy {

// ── Seqlock ─────────────────────────────────────────────────────────────────
// Lock-free single-writer / multi-reader synchronization.
// Writer: begin_write() → modify data → end_write()
// Reader: do { s = read_begin(); copy data; } while (!read_valid(s));

struct alignas(64) seqlock_t {
    std::atomic<uint32_t> seq{0};

    void begin_write() {
        seq.store(seq.load(std::memory_order_relaxed) + 1,
                  std::memory_order_release);
    }

    void end_write() {
        seq.store(seq.load(std::memory_order_relaxed) + 1,
                  std::memory_order_release);
    }

    uint32_t read_begin() const {
        return seq.load(std::memory_order_acquire);
    }

    bool read_valid(uint32_t s) const {
        std::atomic_thread_fence(std::memory_order_acquire);
        return (s & 1) == 0 && seq.load(std::memory_order_relaxed) == s;
    }
};

// ── State vector (7D) ───────────────────────────────────────────────────────
// Matches AutoRally: [x, y, yaw, roll, u_x, u_y, yaw_rate]
// ref: run_control_loop.cuh:119

struct state_7d_t {
    float x;         // world position X (m)
    float y;         // world position Y (m)
    float yaw;       // heading angle (rad)
    float roll;      // roll angle (rad)
    float u_x;       // body-frame longitudinal velocity (m/s)
    float u_y;       // body-frame lateral velocity (m/s)
    float yaw_rate;  // yaw angular velocity (rad/s)
    uint64_t timestamp_us;
};

// ── Control vector (2D) ─────────────────────────────────────────────────────
// ref: path_integral_main.cu:94

struct control_2d_t {
    float steering;  // [-0.99, 0.99]
    float throttle;  // [-0.99, max_throttle]
    uint64_t timestamp_us;
};

// ── IMU sample from BNO085 ──────────────────────────────────────────────────

struct imu_sample_t {
    float qw, qx, qy, qz;  // quaternion (rotation)
    float ax, ay, az;        // linear acceleration (m/s^2)
    float gx, gy, gz;        // angular velocity (rad/s)
    uint64_t timestamp_us;
};

// ── Wheel encoder data ──────────────────────────────────────────────────────

struct wheel_sample_t {
    float left_rps;   // left wheel revolutions per second
    float right_rps;  // right wheel revolutions per second
    uint64_t timestamp_us;
};

// ── VIO pose from ZED SDK ───────────────────────────────────────────────────

struct vio_pose_t {
    float x, y, z;           // position (m)
    float qw, qx, qy, qz;   // orientation quaternion
    float vx, vy, vz;        // velocity (m/s)
    float confidence;         // tracking confidence [0, 1]
    uint64_t timestamp_us;
};

// ── Shared bus ──────────────────────────────────────────────────────────────
// All inter-thread communication. Each field has its own seqlock.
// Costmap uses atomic pointer swap (double-buffering) since it's 96KB.

struct alignas(128) shared_bus_t {
    // State estimate: written by T2 (State Est), read by T1 (Planning)
    seqlock_t state_lock;
    state_7d_t state;

    // Control command: written by T1 (Planning), read by T3 (Actuation)
    seqlock_t control_lock;
    control_2d_t control;

    // IMU data: written by T3 (Actuation), read by T2 (State Est)
    seqlock_t imu_lock;
    imu_sample_t imu;

    // Wheel encoder: written by T3 (Actuation), read by T2 (State Est)
    seqlock_t wheel_lock;
    wheel_sample_t wheels;

    // VIO pose: written by T0 (Perception), read by T2 (State Est)
    seqlock_t vio_lock;
    vio_pose_t vio;

    // Costmap: double-buffered GPU pointer, written by T0, read by T1
    std::atomic<float*> costmap_ptr{nullptr};
    std::atomic<uint64_t> costmap_stamp{0};

    // System flags
    std::atomic<bool> alive{true};
    std::atomic<bool> armed{false};
};

// ── Seqlock helpers ─────────────────────────────────────────────────────────

template<typename T>
void seqlock_write(seqlock_t& lock, T& dest, const T& src) {
    lock.begin_write();
    std::memcpy(&dest, &src, sizeof(T));
    lock.end_write();
}

template<typename T>
T seqlock_read(const seqlock_t& lock, const T& src) {
    T tmp;
    uint32_t s;
    do {
        s = lock.read_begin();
        std::memcpy(&tmp, &src, sizeof(T));
    } while (!lock.read_valid(s));
    return tmp;
}

} // namespace truggy
