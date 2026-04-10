#pragma once

#include <cuda_runtime.h>
#include <cstdint>
#include <string>

namespace truggy {

// NeuralNetModel<7, 2, 3, 6, 32, 32, 4>
// Ported from AutoRally neural_net_model.cuh
//
// Template params (hardcoded for our specific model):
//   STATE_DIM=7, CONTROL_DIM=2, K_DIM=3 (kinematics dims)
//   Network layers: 6 → 32 → 32 → 4
//   DYNAMICS_DIM = STATE_DIM - K_DIM = 4
//   NUM_PARAMS = (6+1)*32 + (32+1)*32 + (32+1)*4 = 224 + 1056 + 132 = 1412
//   LARGEST_LAYER = 33 (32 + 1 PRIME_PADDING)
//   SHARED_MEM_REQUEST_BLK = 2 * 33 = 66 floats per rollout

static constexpr int DYN_STATE_DIM   = 7;
static constexpr int DYN_CONTROL_DIM = 2;
static constexpr int DYN_K_DIM       = 3;
static constexpr int DYN_DIM         = 4;  // STATE_DIM - K_DIM
static constexpr int DYN_NUM_LAYERS  = 4;  // 6, 32, 32, 4
static constexpr int DYN_LARGEST_LAYER = 33;  // max(6,32,32,4) + 1 padding
static constexpr int DYN_NUM_PARAMS  = 1412;
static constexpr int DYN_SHARED_MEM_REQUEST_GRD = 0;
static constexpr int DYN_SHARED_MEM_REQUEST_BLK = 2 * DYN_LARGEST_LAYER;  // 66

struct dynamics_model_t {
    float* theta_d;         // GPU global memory params
    int* stride_idcs_d;     // stride indices on GPU
    int* net_structure_d;   // layer sizes on GPU
    float2* control_rngs_d; // control constraints on GPU
    float dt;

    // Host-side arrays
    float net_params[DYN_NUM_PARAMS];
    int net_structure[DYN_NUM_LAYERS];
    int stride_idcs[DYN_NUM_LAYERS * 2 + 1];
    float2 control_rngs[DYN_CONTROL_DIM];
};

// Host functions
bool dynamics_load(dynamics_model_t& model, const char* npz_path,
                   float dt, float max_throttle);
void dynamics_params_to_device(dynamics_model_t& model);
void dynamics_free(dynamics_model_t& model);

// Device functions (called from rollout kernel)
__device__ void dynamics_cuda_init(float* theta_s);
__device__ void dynamics_enforce_constraints(float* state, float* control,
                                             const float2* control_rngs_d);
__device__ void dynamics_compute_state_deriv(float* state, float* control,
                                             float* state_der, float* theta_s,
                                             const int* stride_idcs_d,
                                             const int* net_structure_d,
                                             float dt);
__device__ void dynamics_increment_state(float* state, float* state_der, float dt);

} // namespace truggy
