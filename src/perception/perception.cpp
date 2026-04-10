#include "perception/perception.h"
#include "common/timing.h"
#include <cstdio>

#ifdef HAS_ZED_SDK
#include <sl/Camera.hpp>
#endif

namespace truggy {

#ifdef HAS_ZED_SDK

// ── ZED SDK perception ─────────────────────────────────────────────────
void perception_loop(shared_bus_t* bus, const config_t& cfg) {
    fprintf(stderr, "[T0] Perception thread starting (ZED SDK)\n");

    sl::Camera camera;
    sl::InitParameters init_params;
    init_params.camera_resolution = sl::RESOLUTION::HD720;
    init_params.camera_fps = cfg.camera.fps;
    init_params.coordinate_units = sl::UNIT::METER;
    init_params.coordinate_system = sl::COORDINATE_SYSTEM::RIGHT_HANDED_Z_UP;

    // Depth mode based on config
    if (cfg.camera.depth_mode == "NEURAL") {
        init_params.depth_mode = sl::DEPTH_MODE::NEURAL;
    } else if (cfg.camera.depth_mode == "ULTRA") {
        init_params.depth_mode = sl::DEPTH_MODE::ULTRA;
    } else {
        init_params.depth_mode = sl::DEPTH_MODE::PERFORMANCE;
    }
    init_params.depth_minimum_distance = 0.3f;
    init_params.depth_maximum_distance = 20.0f;

    auto err = camera.open(init_params);
    if (err != sl::ERROR_CODE::SUCCESS) {
        fprintf(stderr, "[T0] Failed to open ZED camera: %s\n",
                sl::toString(err).c_str());
        return;
    }
    fprintf(stderr, "[T0] ZED camera opened (serial: %u, fw: %s)\n",
            camera.getCameraInformation().serial_number,
            camera.getCameraInformation().camera_configuration.firmware_version == 0
                ? "unknown" : "ok");

    // Enable positional tracking (VIO)
    sl::PositionalTrackingParameters tracking_params;
    tracking_params.enable_area_memory = true;
    err = camera.enablePositionalTracking(tracking_params);
    if (err != sl::ERROR_CODE::SUCCESS) {
        fprintf(stderr, "[T0] Failed to enable tracking: %s\n",
                sl::toString(err).c_str());
    } else {
        fprintf(stderr, "[T0] Positional tracking enabled\n");
    }

    sl::RuntimeParameters runtime_params;
    runtime_params.enable_depth = true;

    sl::Mat depth_map;
    sl::Mat left_image;
    sl::Pose camera_pose;
    uint32_t frame_count = 0;

    while (bus->alive.load(std::memory_order_relaxed)) {
        scoped_timer_t timer("perception", 33.3);

        // Grab frame (blocking, ~33ms at 30fps)
        err = camera.grab(runtime_params);
        if (err != sl::ERROR_CODE::SUCCESS) {
            if (err != sl::ERROR_CODE::END_OF_SVOFILE_REACHED) {
                fprintf(stderr, "[T0] grab() failed: %s\n",
                        sl::toString(err).c_str());
            }
            sleep_until_us(now_us() + 10000);
            continue;
        }

        frame_count++;
        uint64_t ts = now_us();

        // ── Extract VIO pose ────────────────────────────────────────────
        auto tracking_state = camera.getPosition(camera_pose,
                                                  sl::REFERENCE_FRAME::WORLD);
        if (tracking_state == sl::POSITIONAL_TRACKING_STATE::OK ||
            tracking_state == sl::POSITIONAL_TRACKING_STATE::SEARCHING) {

            sl::Translation pos = camera_pose.getTranslation();
            sl::Orientation ori = camera_pose.getOrientation();

            vio_pose_t vio;
            vio.x = pos.x;
            vio.y = pos.y;
            vio.z = pos.z;
            vio.qw = ori.w;
            vio.qx = ori.x;
            vio.qy = ori.y;
            vio.qz = ori.z;
            // ZED SDK doesn't directly give velocity, approximate from pose
            vio.vx = 0;
            vio.vy = 0;
            vio.vz = 0;
            vio.confidence = camera_pose.pose_confidence / 100.0f;
            vio.timestamp_us = ts;

            seqlock_write(bus->vio_lock, bus->vio, vio);
        }

        // ── Retrieve depth map (GPU-resident) ───────────────────────────
        camera.retrieveMeasure(depth_map, sl::MEASURE::DEPTH, sl::MEM::GPU);

        // ── Retrieve left image (GPU-resident) ──────────────────────────
        camera.retrieveImage(left_image, sl::VIEW::LEFT, sl::MEM::GPU);

        // TODO(epic3-segmentation): run TensorRT segmentation on left_image
        // TODO(epic3-costmap): generate BEV costmap from depth + segmentation
        // For now, costmap remains at its initial zero state

        // Stats every 5 seconds
        if (frame_count % 150 == 0) {
            fprintf(stderr, "[T0] frame=%u tracking=%d confidence=%.0f%%\n",
                    frame_count,
                    (int)tracking_state,
                    camera_pose.pose_confidence);
        }
    }

    camera.disablePositionalTracking();
    camera.close();
    fprintf(stderr, "[T0] Perception thread stopped (frames=%u)\n", frame_count);
}

#else  // !HAS_ZED_SDK

// ── Stub perception (no ZED SDK) ────────────────────────────────────────
void perception_loop(shared_bus_t* bus, const config_t& cfg) {
    fprintf(stderr, "[T0] Perception thread starting (NO ZED SDK — stub mode)\n");
    (void)cfg;

    while (bus->alive.load(std::memory_order_relaxed)) {
        sleep_until_us(now_us() + 33333);  // ~30 Hz placeholder
    }

    fprintf(stderr, "[T0] Perception thread stopped (stub)\n");
}

#endif // HAS_ZED_SDK

} // namespace truggy
