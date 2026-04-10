#include "common/config.h"
#include <yaml-cpp/yaml.h>
#include <cstdio>

namespace truggy {

static bool load_truggy_yaml(config_t& cfg, const char* path) {
    YAML::Node root;
    try { root = YAML::LoadFile(path); }
    catch (...) {
        fprintf(stderr, "Failed to load %s\n", path);
        return false;
    }

    auto v = root["vehicle"];
    cfg.vehicle.wheelbase    = v["wheelbase"].as<float>(0.35f);
    cfg.vehicle.track_width  = v["track_width"].as<float>(0.30f);
    cfg.vehicle.wheel_radius = v["wheel_radius"].as<float>(0.06f);
    cfg.vehicle.mass         = v["mass"].as<float>(5.0f);

    auto c = root["camera"];
    cfg.camera.resolution   = c["resolution"].as<int>(720);
    cfg.camera.fps          = c["fps"].as<int>(30);
    cfg.camera.depth_mode   = c["depth_mode"].as<std::string>("PERFORMANCE");
    cfg.camera.mount_height = c["mount_height"].as<float>(0.25f);
    cfg.camera.mount_pitch  = c["mount_pitch"].as<float>(-12.0f);

    auto s = root["serial"];
    cfg.serial.port           = s["port"].as<std::string>("/dev/ttyACM0");
    cfg.serial.baud           = s["baud"].as<int>(115200);
    cfg.serial.command_rate   = s["command_rate"].as<int>(50);
    cfg.serial.telemetry_rate = s["telemetry_rate"].as<int>(100);
    cfg.serial.watchdog_ms    = s["watchdog_ms"].as<int>(250);

    auto m = root["mppi"];
    cfg.mppi.hz                  = m["hz"].as<int>(50);
    cfg.mppi.num_timesteps       = m["num_timesteps"].as<int>(100);
    cfg.mppi.gamma               = m["gamma"].as<float>(0.15f);
    cfg.mppi.num_iters           = m["num_iters"].as<int>(1);
    cfg.mppi.optimization_stride = m["optimization_stride"].as<int>(1);
    cfg.mppi.init_steering       = m["init_steering"].as<float>(0.0f);
    cfg.mppi.init_throttle       = m["init_throttle"].as<float>(0.0f);
    cfg.mppi.steering_std        = m["steering_std"].as<float>(0.275f);
    cfg.mppi.throttle_std        = m["throttle_std"].as<float>(0.3f);
    cfg.mppi.max_throttle        = m["max_throttle"].as<float>(0.65f);
    cfg.mppi.min_throttle        = m["min_throttle"].as<float>(-0.99f);
    cfg.mppi.max_steering        = m["max_steering"].as<float>(0.99f);
    cfg.mppi.min_steering        = m["min_steering"].as<float>(-0.99f);
    cfg.mppi.model_path          = m["model_path"].as<std::string>("models/dynamics_model.npz");

    auto cm = root["costmap"];
    cfg.costmap.width     = cm["width"].as<int>(200);
    cfg.costmap.height    = cm["height"].as<int>(120);
    cfg.costmap.cell_size = cm["cell_size"].as<float>(0.05f);
    cfg.costmap.x_min     = cm["x_min"].as<float>(-5.0f);
    cfg.costmap.x_max     = cm["x_max"].as<float>(5.0f);
    cfg.costmap.y_min     = cm["y_min"].as<float>(0.0f);
    cfg.costmap.y_max     = cm["y_max"].as<float>(6.0f);

    auto e = root["ekf"];
    cfg.ekf.q_pos      = e["q_pos"].as<float>(0.01f);
    cfg.ekf.q_yaw      = e["q_yaw"].as<float>(0.001f);
    cfg.ekf.q_vel      = e["q_vel"].as<float>(0.1f);
    cfg.ekf.q_yaw_rate = e["q_yaw_rate"].as<float>(0.01f);
    cfg.ekf.r_vio_pos  = e["r_vio_pos"].as<float>(0.05f);
    cfg.ekf.r_vio_yaw  = e["r_vio_yaw"].as<float>(0.02f);
    cfg.ekf.r_wheel    = e["r_wheel"].as<float>(0.1f);

    return true;
}

static bool load_costs_yaml(config_t& cfg, const char* path) {
    YAML::Node root;
    try { root = YAML::LoadFile(path); }
    catch (...) {
        fprintf(stderr, "Failed to load %s\n", path);
        return false;
    }

    auto co = root["costs"];
    cfg.costs.desired_speed       = co["desired_speed"].as<float>(4.0f);
    cfg.costs.speed_coeff         = co["speed_coeff"].as<float>(4.25f);
    cfg.costs.track_coeff         = co["track_coeff"].as<float>(200.0f);
    cfg.costs.max_slip_angle      = co["max_slip_angle"].as<float>(1.25f);
    cfg.costs.slip_penalty        = co["slip_penalty"].as<float>(10.0f);
    cfg.costs.track_slop          = co["track_slop"].as<float>(0.0f);
    cfg.costs.crash_coeff         = co["crash_coeff"].as<float>(10000.0f);
    cfg.costs.steering_coeff      = co["steering_coeff"].as<float>(0.0f);
    cfg.costs.throttle_coeff      = co["throttle_coeff"].as<float>(0.0f);
    cfg.costs.boundary_threshold  = co["boundary_threshold"].as<float>(0.65f);
    cfg.costs.discount            = co["discount"].as<float>(0.9f);
    cfg.costs.l1_cost             = co["l1_cost"].as<bool>(false);

    return true;
}

bool load_config(config_t& cfg, const char* truggy_yaml, const char* costs_yaml) {
    return load_truggy_yaml(cfg, truggy_yaml) &&
           load_costs_yaml(cfg, costs_yaml);
}

} // namespace truggy
