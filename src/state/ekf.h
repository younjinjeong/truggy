#pragma once

#include "common/types.h"
#include "common/config.h"

namespace truggy {

struct ekf_t {
    float x[7];      // state vector
    float P[7][7];   // covariance matrix
    ekf_config_t cfg;
};

void ekf_init(ekf_t& ekf, const ekf_config_t& cfg);
void ekf_predict(ekf_t& ekf, const imu_sample_t& imu, float dt);
void ekf_correct_vio(ekf_t& ekf, const vio_pose_t& vio);
void ekf_correct_wheel(ekf_t& ekf, float wheel_speed);
state_7d_t ekf_get_state(const ekf_t& ekf, uint64_t timestamp_us);

} // namespace truggy
