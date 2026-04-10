// MPPI Benchmark — run with synthetic costmap to measure timing
// Usage: ./mppi_bench [num_iterations]

#include "planning/mppi.h"
#include "planning/dynamics.h"
#include "planning/costs.h"
#include "common/timing.h"
#include <cstdio>
#include <cstdlib>
#include <cmath>

using namespace truggy;

// Generate a synthetic circular track costmap on GPU
static void generate_circle_costmap(float* gpu_buf, int w, int h,
                                     float cell_size, float x_min, float y_min) {
    float* host = new float[w * h];
    float cx = 0.0f, cy = 3.0f;  // center of track
    float r_inner = 1.5f, r_outer = 3.0f;

    for (int row = 0; row < h; row++) {
        for (int col = 0; col < w; col++) {
            float x = x_min + col * cell_size;
            float y = y_min + row * cell_size;
            float dist = sqrtf((x - cx) * (x - cx) + (y - cy) * (y - cy));
            if (dist < r_inner || dist > r_outer) {
                host[row * w + col] = 1.0f;  // off-track
            } else {
                host[row * w + col] = 0.0f;  // on-track
            }
        }
    }
    cudaMemcpy(gpu_buf, host, w * h * sizeof(float), cudaMemcpyHostToDevice);
    delete[] host;
}

int main(int argc, char** argv) {
    int num_iters = argc > 1 ? atoi(argv[1]) : 100;

    fprintf(stderr, "=== MPPI Benchmark ===\n");
    fprintf(stderr, "Rollouts: %d, Timesteps: 100, Block: %dx%d\n",
            MPPI_NUM_ROLLOUTS, MPPI_BLOCKSIZE_X, MPPI_BLOCKSIZE_Y);

    // CUDA device info
    cudaDeviceProp prop;
    cudaGetDeviceProperties(&prop, 0);
    fprintf(stderr, "GPU: %s (sm_%d%d)\n", prop.name, prop.major, prop.minor);

    // Initialize dynamics (zero weights for benchmarking)
    dynamics_model_t dynamics;
    dynamics.dt = 0.02f;
    dynamics.net_structure[0] = 6;
    dynamics.net_structure[1] = 32;
    dynamics.net_structure[2] = 32;
    dynamics.net_structure[3] = 4;
    dynamics.control_rngs[0] = make_float2(-0.99f, 0.99f);
    dynamics.control_rngs[1] = make_float2(-0.99f, 0.65f);
    memset(dynamics.net_params, 0, sizeof(dynamics.net_params));

    int stride = 0;
    for (int i = 0; i < DYN_NUM_LAYERS - 1; i++) {
        dynamics.stride_idcs[2 * i] = stride;
        stride += dynamics.net_structure[i + 1] * dynamics.net_structure[i];
        dynamics.stride_idcs[2 * i + 1] = stride;
        stride += dynamics.net_structure[i + 1];
    }
    dynamics.stride_idcs[DYN_NUM_LAYERS * 2] = stride;

    cudaMalloc(&dynamics.theta_d, DYN_NUM_PARAMS * sizeof(float));
    cudaMalloc(&dynamics.stride_idcs_d, (2 * DYN_NUM_LAYERS + 1) * sizeof(int));
    cudaMalloc(&dynamics.net_structure_d, DYN_NUM_LAYERS * sizeof(int));
    cudaMalloc(&dynamics.control_rngs_d, DYN_CONTROL_DIM * sizeof(float2));
    dynamics_params_to_device(dynamics);

    // Initialize costs
    cost_config_t cost_cfg = {4.0f, 4.25f, 200.0f, 1.25f, 10.0f, 0.0f,
                               10000.0f, 0.0f, 0.0f, 0.65f, 0.9f, false};
    costmap_config_t map_cfg = {200, 120, 0.05f, -5.0f, 5.0f, 0.0f, 6.0f};
    mppi_costs_t costs;
    costs_init(costs, cost_cfg, map_cfg);

    // Generate synthetic costmap
    float* costmap_gpu;
    cudaMalloc(&costmap_gpu, 200 * 120 * sizeof(float));
    generate_circle_costmap(costmap_gpu, 200, 120, 0.05f, -5.0f, 0.0f);
    costs_update_costmap_gpu(costs, costmap_gpu, 200, 120);

    // Initialize MPPI
    mppi_config_t mppi_cfg;
    mppi_cfg.hz = 50;
    mppi_cfg.num_timesteps = 100;
    mppi_cfg.gamma = 0.15f;
    mppi_cfg.num_iters = 1;
    mppi_cfg.optimization_stride = 1;
    mppi_cfg.init_steering = 0.0f;
    mppi_cfg.init_throttle = 0.0f;
    mppi_cfg.steering_std = 0.275f;
    mppi_cfg.throttle_std = 0.3f;
    mppi_cfg.max_throttle = 0.65f;

    mppi_controller_t mppi;
    mppi_init(mppi, &dynamics, &costs, mppi_cfg);

    // Warmup
    float state[7] = {0, 0, 0, 0, 2.0f, 0, 0};  // moving forward at 2 m/s
    for (int i = 0; i < 5; i++) {
        mppi_compute_control(mppi, state);
    }

    // Benchmark
    fprintf(stderr, "\nRunning %d iterations...\n", num_iters);
    double total_ms = 0.0;
    double min_ms = 1e9, max_ms = 0.0;

    for (int i = 0; i < num_iters; i++) {
        mppi_slide_control_seq(mppi, 1);
        uint64_t t0 = now_us();
        mppi_compute_control(mppi, state);
        double dt = elapsed_ms(t0);
        total_ms += dt;
        if (dt < min_ms) min_ms = dt;
        if (dt > max_ms) max_ms = dt;

        float steer, throt;
        mppi_get_control(mppi, &steer, &throt);
        if (i % 20 == 0) {
            fprintf(stderr, "  [%3d] %.1f ms  steer=%.3f throt=%.3f\n",
                    i, dt, steer, throt);
        }
    }

    fprintf(stderr, "\n=== Results ===\n");
    fprintf(stderr, "  Avg: %.2f ms (%.0f Hz)\n",
            total_ms / num_iters, 1000.0 / (total_ms / num_iters));
    fprintf(stderr, "  Min: %.2f ms  Max: %.2f ms\n", min_ms, max_ms);
    fprintf(stderr, "  Rollouts: %d  Timesteps: %d\n",
            MPPI_NUM_ROLLOUTS, mppi_cfg.num_timesteps);
    fprintf(stderr, "  State evals/s: %.1fM\n",
            (double)MPPI_NUM_ROLLOUTS * mppi_cfg.num_timesteps *
            (1000.0 / (total_ms / num_iters)) / 1e6);

    // Cleanup
    mppi_free(mppi);
    costs_free(costs);
    dynamics_free(dynamics);
    cudaFree(costmap_gpu);

    return 0;
}
