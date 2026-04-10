#include "state/ekf.h"
#include "state/imu.h"
#include "common/timing.h"
#include <cstdio>
#include <cmath>

namespace truggy {

// State indices
enum { SX = 0, SY, SYAW, SROLL, SUX, SUY, SYAW_RATE };

void ekf_init(ekf_t& ekf, const ekf_config_t& cfg) {
    ekf.x.setZero();
    ekf.P.setIdentity();
    ekf.cfg = cfg;
    ekf.last_predict_us = 0;
    ekf.last_vio_stamp = 0;

    // Process noise (diagonal)
    ekf.Q.setZero();
    ekf.Q(SX, SX)         = cfg.q_pos;
    ekf.Q(SY, SY)         = cfg.q_pos;
    ekf.Q(SYAW, SYAW)     = cfg.q_yaw;
    ekf.Q(SROLL, SROLL)   = cfg.q_yaw;       // roll same order as yaw
    ekf.Q(SUX, SUX)       = cfg.q_vel;
    ekf.Q(SUY, SUY)       = cfg.q_vel;
    ekf.Q(SYAW_RATE, SYAW_RATE) = cfg.q_yaw_rate;
}

// ── Prediction ──────────────────────────────────────────────────────────
// Kinematic model matching AutoRally's computeKinematics + IMU integration
// ref: neural_net_model.cu:345-347

void ekf_predict(ekf_t& ekf, const imu_sample_t& imu, float dt) {
    if (dt <= 0.0f || dt > 0.1f) return;  // sanity check

    float yaw = ekf.x[SYAW];
    float u_x = ekf.x[SUX];
    float u_y = ekf.x[SUY];
    float yaw_rate = ekf.x[SYAW_RATE];

    // ── State prediction ────────────────────────────────────────────────
    // Position: kinematic integration (same as MPPI dynamics)
    ekf.x[SX] += dt * (cosf(yaw) * u_x - sinf(yaw) * u_y);
    ekf.x[SY] += dt * (sinf(yaw) * u_x + cosf(yaw) * u_y);
    ekf.x[SYAW] += dt * yaw_rate;

    // Roll: direct from BNO085 quaternion (on-chip fusion, very accurate)
    euler_t euler = quat_to_euler(imu.qw, imu.qx, imu.qy, imu.qz);
    ekf.x[SROLL] = euler.roll;

    // Body-frame velocity: integrate accelerometer + Coriolis correction
    ekf.x[SUX] += dt * (imu.ax + u_y * yaw_rate);
    ekf.x[SUY] += dt * (imu.ay - u_x * yaw_rate);

    // Yaw rate: direct from gyro z-axis
    ekf.x[SYAW_RATE] = imu.gz;

    // ── Jacobian F = df/dx ──────────────────────────────────────────────
    Mat7f F = Mat7f::Identity();

    // dx/d(yaw)
    F(SX, SYAW) = dt * (-sinf(yaw) * u_x - cosf(yaw) * u_y);
    // dx/d(u_x)
    F(SX, SUX) = dt * cosf(yaw);
    // dx/d(u_y)
    F(SX, SUY) = -dt * sinf(yaw);

    // dy/d(yaw)
    F(SY, SYAW) = dt * (cosf(yaw) * u_x - sinf(yaw) * u_y);
    // dy/d(u_x)
    F(SY, SUX) = dt * sinf(yaw);
    // dy/d(u_y)
    F(SY, SUY) = dt * cosf(yaw);

    // dyaw/d(yaw_rate)
    F(SYAW, SYAW_RATE) = dt;

    // du_x/d(u_y) and du_x/d(yaw_rate) from Coriolis
    F(SUX, SUY) = dt * yaw_rate;
    F(SUX, SYAW_RATE) = dt * u_y;

    // du_y/d(u_x) and du_y/d(yaw_rate) from Coriolis
    F(SUY, SUX) = -dt * yaw_rate;
    F(SUY, SYAW_RATE) = -dt * u_x;

    // ── Covariance prediction ───────────────────────────────────────────
    ekf.P = F * ekf.P * F.transpose() + ekf.Q * (dt * dt);
}

// ── VIO Correction ──────────────────────────────────────────────────────
// Corrects [x, y, yaw] from ZED visual-inertial odometry

bool ekf_correct_vio(ekf_t& ekf, const vio_pose_t& vio) {
    // Measurement: [x, y, yaw]
    Eigen::Matrix<float, 3, 1> z;
    z << vio.x, vio.y, quat_to_yaw(vio.qw, vio.qx, vio.qy, vio.qz);

    // Predicted measurement
    Eigen::Matrix<float, 3, 1> z_hat;
    z_hat << ekf.x[SX], ekf.x[SY], ekf.x[SYAW];

    // Innovation
    Eigen::Matrix<float, 3, 1> y = z - z_hat;

    // Wrap yaw innovation to [-pi, pi]
    while (y[2] > M_PI)  y[2] -= 2.0f * M_PI;
    while (y[2] < -M_PI) y[2] += 2.0f * M_PI;

    // Jump detection: skip if innovation is too large (VIO relocalization)
    float mahal_sq = y[0] * y[0] + y[1] * y[1];
    if (mahal_sq > 4.0f) {  // >2m position jump
        return false;
    }

    // Measurement matrix H (3x7)
    Eigen::Matrix<float, 3, 7> H = Eigen::Matrix<float, 3, 7>::Zero();
    H(0, SX) = 1.0f;
    H(1, SY) = 1.0f;
    H(2, SYAW) = 1.0f;

    // Measurement noise R (scale by inverse confidence)
    float conf_scale = (vio.confidence > 0.1f) ? (1.0f / vio.confidence) : 10.0f;
    Eigen::Matrix<float, 3, 3> R = Eigen::Matrix<float, 3, 3>::Zero();
    R(0, 0) = ekf.cfg.r_vio_pos * conf_scale;
    R(1, 1) = ekf.cfg.r_vio_pos * conf_scale;
    R(2, 2) = ekf.cfg.r_vio_yaw * conf_scale;

    // Kalman gain
    Eigen::Matrix<float, 3, 3> S = H * ekf.P * H.transpose() + R;
    Eigen::Matrix<float, 7, 3> K = ekf.P * H.transpose() * S.inverse();

    // State update
    ekf.x += K * y;

    // Covariance update (Joseph form for numerical stability)
    Mat7f I = Mat7f::Identity();
    Mat7f IKH = I - K * H;
    ekf.P = IKH * ekf.P * IKH.transpose() + K * R * K.transpose();

    return true;
}

// ── Wheel Encoder Correction ────────────────────────────────────────────
// Corrects [u_x] from wheel speed measurement

void ekf_correct_wheel(ekf_t& ekf, float wheel_speed_mps) {
    // Measurement: u_x (forward speed)
    float z = wheel_speed_mps;
    float z_hat = ekf.x[SUX];
    float y_inn = z - z_hat;

    // H matrix (1x7): measures u_x only
    Eigen::Matrix<float, 1, 7> H = Eigen::Matrix<float, 1, 7>::Zero();
    H(0, SUX) = 1.0f;

    float R = ekf.cfg.r_wheel;
    float S = (H * ekf.P * H.transpose())(0, 0) + R;
    Eigen::Matrix<float, 7, 1> K = ekf.P * H.transpose() / S;

    ekf.x += K * y_inn;

    Mat7f I = Mat7f::Identity();
    ekf.P = (I - K * H) * ekf.P;
}

state_7d_t ekf_get_state(const ekf_t& ekf, uint64_t timestamp_us) {
    state_7d_t s;
    s.x        = ekf.x[SX];
    s.y        = ekf.x[SY];
    s.yaw      = ekf.x[SYAW];
    s.roll     = ekf.x[SROLL];
    s.u_x      = ekf.x[SUX];
    s.u_y      = ekf.x[SUY];
    s.yaw_rate = ekf.x[SYAW_RATE];
    s.timestamp_us = timestamp_us;
    return s;
}

// ── Thread 2: State Estimation Loop ─────────────────────────────────────

void state_estimation_loop(shared_bus_t* bus, const config_t& cfg) {
    fprintf(stderr, "[T2] State estimation thread starting\n");

    ekf_t ekf;
    ekf_init(ekf, cfg.ekf);

    uint64_t last_imu_stamp = 0;
    uint64_t last_vio_stamp = 0;
    uint32_t predict_count = 0;
    uint32_t vio_count = 0;
    uint32_t wheel_count = 0;

    while (bus->alive.load(std::memory_order_relaxed)) {
        // Read IMU from bus (seqlock)
        imu_sample_t imu = seqlock_read(bus->imu_lock, bus->imu);

        if (imu.timestamp_us > last_imu_stamp && imu.timestamp_us > 0) {
            float dt = (last_imu_stamp > 0)
                ? (float)(imu.timestamp_us - last_imu_stamp) / 1e6f
                : 0.01f;
            last_imu_stamp = imu.timestamp_us;

            // Predict from IMU
            ekf_predict(ekf, imu, dt);
            predict_count++;

            // Correct from wheel encoder
            wheel_sample_t wheels = seqlock_read(bus->wheel_lock, bus->wheels);
            if (wheels.timestamp_us > 0) {
                float wheel_speed = wheels.left_rps * cfg.vehicle.wheel_radius *
                                    2.0f * M_PI;
                ekf_correct_wheel(ekf, wheel_speed);
                wheel_count++;
            }

            // Write state to bus
            state_7d_t state = ekf_get_state(ekf, now_us());
            seqlock_write(bus->state_lock, bus->state, state);
        }

        // Check for new VIO (30 Hz, less frequent than IMU)
        vio_pose_t vio = seqlock_read(bus->vio_lock, bus->vio);
        if (vio.timestamp_us > last_vio_stamp && vio.timestamp_us > 0) {
            last_vio_stamp = vio.timestamp_us;
            if (ekf_correct_vio(ekf, vio)) {
                vio_count++;
            }
            // Write updated state
            state_7d_t state = ekf_get_state(ekf, now_us());
            seqlock_write(bus->state_lock, bus->state, state);
        }

        // Stats every 5 seconds
        if (predict_count > 0 && predict_count % 500 == 0) {
            fprintf(stderr, "[T2] predict=%u vio=%u wheel=%u "
                    "pos=(%.2f,%.2f) yaw=%.1f° u_x=%.2f\n",
                    predict_count, vio_count, wheel_count,
                    ekf.x[0], ekf.x[1],
                    ekf.x[2] * 180.0f / M_PI,
                    ekf.x[4]);
        }

        // Sleep ~1ms (target 100Hz+, driven by IMU rate)
        sleep_until_us(now_us() + 1000);
    }

    fprintf(stderr, "[T2] State estimation stopped "
            "(predict=%u, vio=%u, wheel=%u)\n",
            predict_count, vio_count, wheel_count);
}

} // namespace truggy
