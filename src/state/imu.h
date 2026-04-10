#pragma once

#include <cmath>

namespace truggy {

// Quaternion to Euler angles (ZYX convention)
// BNO085 outputs Hamilton quaternion (w, x, y, z)
struct euler_t {
    float roll;   // rotation around X (rad)
    float pitch;  // rotation around Y (rad)
    float yaw;    // rotation around Z (rad)
};

inline euler_t quat_to_euler(float qw, float qx, float qy, float qz) {
    euler_t e;

    // Roll (X axis)
    float sinr = 2.0f * (qw * qx + qy * qz);
    float cosr = 1.0f - 2.0f * (qx * qx + qy * qy);
    e.roll = atan2f(sinr, cosr);

    // Pitch (Y axis) — clamped to avoid NaN at gimbal lock
    float sinp = 2.0f * (qw * qy - qz * qx);
    if (fabsf(sinp) >= 1.0f)
        e.pitch = copysignf(M_PI / 2.0f, sinp);
    else
        e.pitch = asinf(sinp);

    // Yaw (Z axis)
    float siny = 2.0f * (qw * qz + qx * qy);
    float cosy = 1.0f - 2.0f * (qy * qy + qz * qz);
    e.yaw = atan2f(siny, cosy);

    return e;
}

// Extract yaw from quaternion (avoids computing roll/pitch)
inline float quat_to_yaw(float qw, float qx, float qy, float qz) {
    float siny = 2.0f * (qw * qz + qx * qy);
    float cosy = 1.0f - 2.0f * (qy * qy + qz * qz);
    return atan2f(siny, cosy);
}

// Normalize quaternion (ensure unit length after accumulation errors)
inline void quat_normalize(float& qw, float& qx, float& qy, float& qz) {
    float norm = sqrtf(qw * qw + qx * qx + qy * qy + qz * qz);
    if (norm > 1e-6f) {
        float inv = 1.0f / norm;
        qw *= inv; qx *= inv; qy *= inv; qz *= inv;
    }
}

} // namespace truggy
