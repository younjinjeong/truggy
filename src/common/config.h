#pragma once

#include <string>

namespace truggy {

struct vehicle_config_t {
    float wheelbase;
    float track_width;
    float wheel_radius;
    float mass;
};

struct camera_config_t {
    int resolution;
    int fps;
    std::string depth_mode;
    float mount_height;
    float mount_pitch;
};

struct serial_config_t {
    std::string port;
    int baud;
    int command_rate;
    int telemetry_rate;
    int watchdog_ms;
};

struct mppi_config_t {
    int hz;
    int num_timesteps;
    float gamma;
    int num_iters;
    int optimization_stride;
    float init_steering;
    float init_throttle;
    float steering_std;
    float throttle_std;
    float max_throttle;
    float min_throttle;
    float max_steering;
    float min_steering;
    std::string model_path;
};

struct costmap_config_t {
    int width;
    int height;
    float cell_size;
    float x_min, x_max;
    float y_min, y_max;
};

struct cost_config_t {
    float desired_speed;
    float speed_coeff;
    float track_coeff;
    float max_slip_angle;
    float slip_penalty;
    float track_slop;
    float crash_coeff;
    float steering_coeff;
    float throttle_coeff;
    float boundary_threshold;
    float discount;
    bool l1_cost;
};

struct ekf_config_t {
    float q_pos;
    float q_yaw;
    float q_vel;
    float q_yaw_rate;
    float r_vio_pos;
    float r_vio_yaw;
    float r_wheel;
};

struct config_t {
    vehicle_config_t vehicle;
    camera_config_t camera;
    serial_config_t serial;
    mppi_config_t mppi;
    costmap_config_t costmap;
    cost_config_t costs;
    ekf_config_t ekf;
};

// Load config from YAML files. Returns false on error.
bool load_config(config_t& cfg,
                 const char* truggy_yaml,
                 const char* costs_yaml);

} // namespace truggy
