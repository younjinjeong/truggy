#pragma once

#include "planning/dynamics.h"
#include "planning/costs.h"
#include "common/types.h"
#include "common/config.h"
#include <curand.h>

namespace truggy {

static constexpr int MPPI_BLOCKSIZE_WRX = 64;  // reduction block width

struct mppi_controller_t {
    // GPU arrays
    float* state_d;          // STATE_DIM
    float* nu_d;             // CONTROL_DIM (exploration variance)
    float* traj_costs_d;     // NUM_ROLLOUTS
    float* U_d;              // num_timesteps * CONTROL_DIM
    float* du_d;             // NUM_ROLLOUTS * num_timesteps * CONTROL_DIM

    // Host arrays
    float* U;                // nominal control sequence
    float* du;               // control update
    float* traj_costs;       // per-rollout costs
    float* control_hist;     // history for Savitzky-Golay (4 values)

    // Models
    dynamics_model_t* dynamics;
    mppi_costs_t* costs;

    // Parameters
    int num_timesteps;
    int num_rollouts;
    int hz;
    float gamma;
    int num_iters;
    int opt_stride;
    float nu[2];  // exploration variance [steering, throttle]

    // CUDA
    curandGenerator_t gen;
    cudaStream_t stream;
};

// Host functions
bool mppi_init(mppi_controller_t& mppi, dynamics_model_t* dynamics,
               mppi_costs_t* costs, const mppi_config_t& cfg);
void mppi_compute_control(mppi_controller_t& mppi, float* state);
void mppi_slide_control_seq(mppi_controller_t& mppi, int stride);
void mppi_reset_controls(mppi_controller_t& mppi);
void mppi_get_control(const mppi_controller_t& mppi, float* steering, float* throttle);
void mppi_free(mppi_controller_t& mppi);

// Thread 1 entry point
void planning_loop(shared_bus_t* bus, const config_t& cfg);

} // namespace truggy
