#include "state/ekf.h"
#include <cstring>
#include <cmath>

namespace truggy {

void ekf_init(ekf_t& ekf, const ekf_config_t& cfg) {
    memset(ekf.x, 0, sizeof(ekf.x));
    memset(ekf.P, 0, sizeof(ekf.P));
    for (int i = 0; i < 7; i++) ekf.P[i][i] = 1.0f;
    ekf.cfg = cfg;
}

void ekf_predict(ekf_t& ekf, const imu_sample_t& imu, float dt) {
    // TODO(epic5): implement full EKF prediction
    (void)ekf; (void)imu; (void)dt;
}

void ekf_correct_vio(ekf_t& ekf, const vio_pose_t& vio) {
    // TODO(epic5): implement VIO correction
    (void)ekf; (void)vio;
}

void ekf_correct_wheel(ekf_t& ekf, float wheel_speed) {
    // TODO(epic5): implement wheel encoder correction
    (void)ekf; (void)wheel_speed;
}

state_7d_t ekf_get_state(const ekf_t& ekf, uint64_t timestamp_us) {
    state_7d_t s;
    s.x        = ekf.x[0];
    s.y        = ekf.x[1];
    s.yaw      = ekf.x[2];
    s.roll     = ekf.x[3];
    s.u_x      = ekf.x[4];
    s.u_y      = ekf.x[5];
    s.yaw_rate = ekf.x[6];
    s.timestamp_us = timestamp_us;
    return s;
}

} // namespace truggy
