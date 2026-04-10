#pragma once

#include "common/config.h"
#include <cuda_runtime.h>

namespace truggy {

// Half-wheelbase distances for front/back collision points
// 1/8 truggy: 0.35m wheelbase → ±0.175m
static constexpr float COST_FRONT_D = 0.175f;
static constexpr float COST_BACK_D  = -0.175f;

// Cost parameters (transferred to GPU as a struct)
// ref: AutoRally costs.cuh CostParams
struct cost_params_gpu_t {
    float desired_speed;
    float speed_coeff;
    float track_coeff;
    float max_slip_ang;
    float slip_penalty;
    float track_slop;
    float crash_coeff;
    float steering_coeff;
    float throttle_coeff;
    float boundary_threshold;
    float discount;
    int num_timesteps;
    // Costmap transform (ego-centric BEV, simplified from AutoRally)
    float map_x_min, map_x_max;
    float map_y_min, map_y_max;
};

struct mppi_costs_t {
    cost_params_gpu_t params;
    cost_params_gpu_t* params_d;  // GPU copy
    cudaTextureObject_t costmap_tex;
    cudaArray_t costmap_array_d;
    cudaChannelFormatDesc channel_desc;
    int width, height;
    bool l1_cost;
    bool tex_initialized;
};

// Host functions
void costs_init(mppi_costs_t& costs, const cost_config_t& cfg,
                const costmap_config_t& map_cfg);
void costs_update_params(mppi_costs_t& costs, const cost_config_t& cfg);
void costs_params_to_device(mppi_costs_t& costs);
void costs_update_costmap_gpu(mppi_costs_t& costs, float* gpu_costmap,
                               int width, int height);
void costs_free(mppi_costs_t& costs);

// Device functions (called from rollout kernel)
__device__ float costs_compute(float* s, float* u, float* du, float* vars,
                               int* crash, int timestep,
                               const cost_params_gpu_t* params,
                               cudaTextureObject_t costmap_tex,
                               bool l1_cost);
__device__ void costs_get_crash(float* s, int* crash);
__device__ float costs_terminal(float* s);

} // namespace truggy
