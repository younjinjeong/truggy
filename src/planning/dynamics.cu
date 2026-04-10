// Ported from AutoRally neural_net_model.cu (Georgia Tech, BSD License)
// Original author: Grady Williams <gradyrw@gmail.com>

#include "planning/dynamics.h"
#include <cnpy.h>
#include <cstdio>
#include <cstring>
#include <cmath>

#define MPPI_NNET_NONLINEARITY(x) tanhf(x)

// Constant memory for NN weights (fast path)
#ifdef MPPI_NNET_USING_CONSTANT_MEM__
__device__ __constant__ float NNET_PARAMS[truggy::DYN_NUM_PARAMS];
#endif

namespace truggy {

// ── Host: load weights from .npz ─────────────────────────────────────────

bool dynamics_load(dynamics_model_t& model, const char* npz_path,
                   float dt, float max_throttle) {
    model.dt = dt;
    model.net_structure[0] = 6;   // input: [roll, u_x, u_y, yaw_rate, steering, throttle]
    model.net_structure[1] = 32;
    model.net_structure[2] = 32;
    model.net_structure[3] = 4;   // output: [droll, du_x, du_y, dyaw_rate]

    // Control constraints
    model.control_rngs[0] = make_float2(-0.99f, 0.99f);   // steering
    model.control_rngs[1] = make_float2(-0.99f, max_throttle);  // throttle

    // Load weights from .npz
    cnpy::npz_t param_dict;
    try {
        param_dict = cnpy::npz_load(npz_path);
    } catch (...) {
        fprintf(stderr, "[dynamics] Failed to load model: %s\n", npz_path);
        return false;
    }

    // Pack weights and biases into flat parameter array
    // Layout: [W1, b1, W2, b2, W3, b3] with stride indices
    int stride = 0;
    for (int layer = 0; layer < DYN_NUM_LAYERS - 1; layer++) {
        std::string w_name = "dynamics_W" + std::to_string(layer + 1);
        std::string b_name = "dynamics_b" + std::to_string(layer + 1);

        auto it_w = param_dict.find(w_name);
        auto it_b = param_dict.find(b_name);
        if (it_w == param_dict.end() || it_b == param_dict.end()) {
            fprintf(stderr, "[dynamics] Missing key: %s or %s\n",
                    w_name.c_str(), b_name.c_str());
            return false;
        }

        int rows = model.net_structure[layer + 1];
        int cols = model.net_structure[layer];
        double* w_data = it_w->second.data<double>();
        double* b_data = it_b->second.data<double>();

        // Weight matrix stride index
        model.stride_idcs[2 * layer] = stride;
        for (int j = 0; j < rows; j++) {
            for (int k = 0; k < cols; k++) {
                model.net_params[stride + j * cols + k] = (float)w_data[j * cols + k];
            }
        }
        stride += rows * cols;

        // Bias vector stride index
        model.stride_idcs[2 * layer + 1] = stride;
        for (int j = 0; j < rows; j++) {
            model.net_params[stride + j] = (float)b_data[j];
        }
        stride += rows;
    }
    model.stride_idcs[DYN_NUM_LAYERS * 2] = stride;

    fprintf(stderr, "[dynamics] Loaded %d parameters from %s\n", stride, npz_path);

    // Allocate GPU memory
    cudaMalloc(&model.theta_d, DYN_NUM_PARAMS * sizeof(float));
    cudaMalloc(&model.stride_idcs_d, (2 * DYN_NUM_LAYERS + 1) * sizeof(int));
    cudaMalloc(&model.net_structure_d, DYN_NUM_LAYERS * sizeof(int));
    cudaMalloc(&model.control_rngs_d, DYN_CONTROL_DIM * sizeof(float2));

    dynamics_params_to_device(model);
    return true;
}

void dynamics_params_to_device(dynamics_model_t& model) {
    cudaMemcpy(model.theta_d, model.net_params,
               DYN_NUM_PARAMS * sizeof(float), cudaMemcpyHostToDevice);
    cudaMemcpy(model.stride_idcs_d, model.stride_idcs,
               (2 * DYN_NUM_LAYERS + 1) * sizeof(int), cudaMemcpyHostToDevice);
    cudaMemcpy(model.net_structure_d, model.net_structure,
               DYN_NUM_LAYERS * sizeof(int), cudaMemcpyHostToDevice);
    cudaMemcpy(model.control_rngs_d, model.control_rngs,
               DYN_CONTROL_DIM * sizeof(float2), cudaMemcpyHostToDevice);

#ifdef MPPI_NNET_USING_CONSTANT_MEM__
    cudaMemcpyToSymbol(NNET_PARAMS, model.theta_d,
                       DYN_NUM_PARAMS * sizeof(float));
#endif
}

void dynamics_free(dynamics_model_t& model) {
    cudaFree(model.theta_d);
    cudaFree(model.stride_idcs_d);
    cudaFree(model.net_structure_d);
    cudaFree(model.control_rngs_d);
}

// ── Device: kinematics (analytical, not learned) ─────────────────────────

__device__ static void compute_kinematics(float* state, float* state_der) {
    // ref: neural_net_model.cu:345-347
    state_der[0] = __cosf(state[2]) * state[4] - __sinf(state[2]) * state[5];
    state_der[1] = __sinf(state[2]) * state[4] + __cosf(state[2]) * state[5];
    state_der[2] = -state[6];  // Negative yaw derivative (AutoRally convention)
}

// ── Device: NN forward pass (dynamics) ───────────────────────────────────
// ref: neural_net_model.cu:351-403

__device__ static void compute_dynamics(float* state, float* control,
                                        float* state_der, float* theta_s,
                                        const int* stride_idcs_d,
                                        const int* net_structure_d) {
    int tdy = threadIdx.y;

    // theta_s layout: [curr_act | next_act] per rollout, LARGEST_LAYER each
    int tdx = threadIdx.x;
    float* curr_act = &theta_s[(2 * DYN_LARGEST_LAYER) * tdx];
    float* next_act = &theta_s[(2 * DYN_LARGEST_LAYER) * tdx + DYN_LARGEST_LAYER];

    // Load input: [roll, u_x, u_y, yaw_rate, steering, throttle]
    for (int i = tdy; i < DYN_DIM; i += blockDim.y) {
        curr_act[i] = state[i + (DYN_STATE_DIM - DYN_DIM)];  // state[3..6]
    }
    for (int i = tdy; i < DYN_CONTROL_DIM; i += blockDim.y) {
        curr_act[DYN_DIM + i] = control[i];
    }
    __syncthreads();

    // Forward pass through layers
    for (int layer = 0; layer < DYN_NUM_LAYERS - 1; layer++) {
        float* W;
        float* b;
#ifdef MPPI_NNET_USING_CONSTANT_MEM__
        W = &NNET_PARAMS[stride_idcs_d[2 * layer]];
        b = &NNET_PARAMS[stride_idcs_d[2 * layer + 1]];
#else
        // Fallback: these would need theta_d passed in (not used with constant mem)
        W = nullptr;  // not supported without constant memory
        b = nullptr;
#endif
        for (int j = tdy; j < net_structure_d[layer + 1]; j += blockDim.y) {
            float tmp = 0.0f;
            for (int k = 0; k < net_structure_d[layer]; k++) {
                tmp += W[j * net_structure_d[layer] + k] * curr_act[k];
            }
            tmp += b[j];
            if (layer < DYN_NUM_LAYERS - 2) {
                tmp = MPPI_NNET_NONLINEARITY(tmp);  // tanh
            }
            next_act[j] = tmp;
        }

        // Swap pointers
        float* swap = curr_act;
        curr_act = next_act;
        next_act = swap;
        __syncthreads();
    }

    // Write output to state derivative [3..6]
    for (int i = tdy; i < DYN_DIM; i += blockDim.y) {
        state_der[i + (DYN_STATE_DIM - DYN_DIM)] = curr_act[i];
    }
    __syncthreads();
}

// ── Device: public interface called by rollout kernel ────────────────────

__device__ void dynamics_cuda_init(float* theta_s) {
    // No-op for constant memory path
    (void)theta_s;
}

__device__ void dynamics_enforce_constraints(float* state, float* control,
                                             const float2* control_rngs_d) {
    for (int i = 0; i < DYN_CONTROL_DIM; i++) {
        if (control[i] < control_rngs_d[i].x) control[i] = control_rngs_d[i].x;
        if (control[i] > control_rngs_d[i].y) control[i] = control_rngs_d[i].y;
    }
}

__device__ void dynamics_compute_state_deriv(float* state, float* control,
                                             float* state_der, float* theta_s,
                                             const int* stride_idcs_d,
                                             const int* net_structure_d,
                                             float dt) {
    if (threadIdx.y == 0) {
        compute_kinematics(state, state_der);
    }
    compute_dynamics(state, control, state_der, theta_s,
                     stride_idcs_d, net_structure_d);
}

__device__ void dynamics_increment_state(float* state, float* state_der, float dt) {
    for (int i = threadIdx.y; i < DYN_STATE_DIM; i += blockDim.y) {
        state[i] += state_der[i] * dt;
        state_der[i] = 0.0f;
    }
}

} // namespace truggy
