#pragma once

#include "common/types.h"
#include "common/config.h"
#include <Eigen/Dense>

namespace truggy {

// 7-state Extended Kalman Filter
// State: [x, y, yaw, roll, u_x, u_y, yaw_rate]
// Matches MPPI state vector exactly (ref: run_control_loop.cuh:119)

using Vec7f = Eigen::Matrix<float, 7, 1>;
using Mat7f = Eigen::Matrix<float, 7, 7>;

struct ekf_t {
    Vec7f x;     // state estimate
    Mat7f P;     // covariance
    Mat7f Q;     // process noise
    ekf_config_t cfg;
    uint64_t last_predict_us;
    uint64_t last_vio_stamp;
};

void ekf_init(ekf_t& ekf, const ekf_config_t& cfg);

// Prediction step from IMU at 100 Hz
void ekf_predict(ekf_t& ekf, const imu_sample_t& imu, float dt);

// Correction from ZED VIO pose at 30 Hz
// Returns false if innovation is too large (jump detection)
bool ekf_correct_vio(ekf_t& ekf, const vio_pose_t& vio);

// Correction from wheel encoder speed
void ekf_correct_wheel(ekf_t& ekf, float wheel_speed_mps);

// Extract state for MPPI
state_7d_t ekf_get_state(const ekf_t& ekf, uint64_t timestamp_us);

// Thread 2 entry point
void state_estimation_loop(shared_bus_t* bus, const config_t& cfg);

} // namespace truggy
