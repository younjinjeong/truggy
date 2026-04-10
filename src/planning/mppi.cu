// Ported from AutoRally mppi_controller.cu (Georgia Tech, BSD License)
// Original author: Grady Williams <gradyrw@gmail.com>
// Adapted: no ROS, no DDP, no Managed, struct-oriented, fixed template params

#include "planning/mppi.h"
#include "common/timing.h"
#include <cstdio>
#include <cstring>
#include <cmath>

// Use defines from CMakeLists.txt: MPPI_NUM_ROLLOUTS, MPPI_BLOCKSIZE_X, MPPI_BLOCKSIZE_Y

namespace truggy {

// ── Kernel 1: Rollout ───────────────────────────────────────────────────
// ref: mppi_controller.cu:47-152

__global__ void rollout_kernel(
    int num_timesteps, int num_rollouts,
    float* state_d, float* U_d, float* du_d, float* nu_d,
    float* costs_d,
    // Dynamics model data (passed by pointer, not object copy)
    const float2* control_rngs_d,
    const int* stride_idcs_d,
    const int* net_structure_d,
    float dt,
    // Cost function data
    const cost_params_gpu_t* cost_params_d,
    cudaTextureObject_t costmap_tex,
    bool l1_cost,
    int opt_delay)
{
    int tdx = threadIdx.x;
    int tdy = threadIdx.y;
    int bdx = blockIdx.x;
    int global_idx = blockDim.x * bdx + tdx;

    // Shared memory
    extern __shared__ float shmem[];

    // Layout shared memory manually
    const int bx = blockDim.x;
    float* state_shared     = shmem;                                          // bx * 7
    float* state_der_shared = state_shared + bx * DYN_STATE_DIM;              // bx * 7
    float* control_shared   = state_der_shared + bx * DYN_STATE_DIM;          // bx * 2
    float* control_var      = control_shared + bx * DYN_CONTROL_DIM;          // bx * 2
    float* explore_var      = control_var + bx * DYN_CONTROL_DIM;             // bx * 2
    int*   crash_status     = (int*)(explore_var + bx * DYN_CONTROL_DIM);     // bx
    float* theta            = (float*)(crash_status + bx);                    // bx * 66

    // Per-rollout pointers
    float* s     = &state_shared[tdx * DYN_STATE_DIM];
    float* s_der = &state_der_shared[tdx * DYN_STATE_DIM];
    float* u     = &control_shared[tdx * DYN_CONTROL_DIM];
    float* du    = &control_var[tdx * DYN_CONTROL_DIM];
    float* nu    = &explore_var[tdx * DYN_CONTROL_DIM];
    int*   crash = &crash_status[tdx];

    float running_cost = 0.0f;

    // Initialize dynamics shared memory
    dynamics_cuda_init(theta);

    if (global_idx < num_rollouts) {
        for (int i = tdy; i < DYN_STATE_DIM; i += blockDim.y) {
            s[i] = state_d[i];
            s_der[i] = 0.0f;
        }
        for (int i = tdy; i < DYN_CONTROL_DIM; i += blockDim.y) {
            u[i] = 0.0f;
            du[i] = 0.0f;
            nu[i] = nu_d[i];
        }
        crash[0] = 0;
    }
    __syncthreads();

    // Simulation loop
    for (int t = 0; t < num_timesteps; t++) {
        if (global_idx < num_rollouts) {
            for (int j = tdy; j < DYN_CONTROL_DIM; j += blockDim.y) {
                int du_idx = DYN_CONTROL_DIM * num_timesteps * global_idx +
                             t * DYN_CONTROL_DIM + j;
                if (global_idx == 0 || t < opt_delay) {
                    // Noise-free rollout (first rollout = nominal)
                    du[j] = 0.0f;
                    u[j] = U_d[t * DYN_CONTROL_DIM + j];
                } else if (global_idx >= (int)(0.99f * num_rollouts)) {
                    // Pure noise rollout (top 1%)
                    du[j] = du_d[du_idx] * nu[j];
                    u[j] = du[j];
                } else {
                    // Normal rollout: nominal + noise
                    du[j] = du_d[du_idx] * nu[j];
                    u[j] = U_d[t * DYN_CONTROL_DIM + j] + du[j];
                }
                // Write back actual control used
                du_d[du_idx] = u[j];
            }
        }
        __syncthreads();

        // Enforce constraints
        if (tdy == 0 && global_idx < num_rollouts) {
            dynamics_enforce_constraints(s, u, control_rngs_d);
        }
        __syncthreads();

        // Compute cost (running average)
        if (tdy == 0 && global_idx < num_rollouts && t > 0 && crash[0] > -1) {
            running_cost += (costs_compute(s, u, du, nu, crash, t,
                            cost_params_d, costmap_tex, l1_cost) -
                            running_cost) / (float)t;
        }

        // Compute dynamics
        if (global_idx < num_rollouts) {
            dynamics_compute_state_deriv(s, u, s_der, theta,
                                        stride_idcs_d, net_structure_d, dt);
        }
        __syncthreads();

        // Update state
        if (global_idx < num_rollouts) {
            dynamics_increment_state(s, s_der, dt);
        }

        // Check crash
        if (tdy == 0 && global_idx < num_rollouts) {
            costs_get_crash(s, crash);
        }
    }

    // Write final cost
    if (global_idx < num_rollouts && tdy == 0) {
        costs_d[global_idx] = running_cost + costs_terminal(s);
    }
}

// ── Kernel 2: Normalize and exponentiate ────────────────────────────────
// ref: mppi_controller.cu:154-164

__global__ void norm_exp_kernel(float* costs_d, int num_rollouts,
                                float gamma, float baseline) {
    int idx = blockDim.x * blockIdx.x + threadIdx.x;
    if (idx < num_rollouts) {
        float cost2go = costs_d[idx] - baseline;
        costs_d[idx] = expf(-gamma * cost2go);
    }
}

// ── Kernel 3: Weighted reduction ────────────────────────────────────────
// ref: mppi_controller.cu:166-214

__global__ void weighted_reduction_kernel(float* weights_d, float* du_d,
                                           float normalizer,
                                           int num_timesteps, int num_rollouts) {
    int tdx = threadIdx.x;
    int bdx = blockIdx.x;  // one block per timestep

    const int stride = MPPI_BLOCKSIZE_WRX;
    const int num_blocks_wr = (num_rollouts - 1) / MPPI_BLOCKSIZE_WRX + 1;

    extern __shared__ float u_system[];
    // u_system: num_blocks_wr * CONTROL_DIM

    for (int j = 0; j < DYN_CONTROL_DIM; j++) {
        u_system[tdx * DYN_CONTROL_DIM + j] = 0.0f;
    }
    __syncthreads();

    if (stride * tdx < num_rollouts) {
        for (int i = 0; i < stride; i++) {
            int rollout_idx = stride * tdx + i;
            if (rollout_idx < num_rollouts) {
                float weight = weights_d[rollout_idx] / normalizer;
                for (int j = 0; j < DYN_CONTROL_DIM; j++) {
                    float u_val = du_d[rollout_idx * (num_timesteps * DYN_CONTROL_DIM) +
                                       bdx * DYN_CONTROL_DIM + j];
                    u_system[tdx * DYN_CONTROL_DIM + j] += weight * u_val;
                }
            }
        }
    }
    __syncthreads();

    // Final reduction (thread 0 sums all blocks)
    if (tdx == 0 && bdx < num_timesteps) {
        float u_final[DYN_CONTROL_DIM] = {0.0f};
        for (int i = 0; i < num_blocks_wr; i++) {
            for (int j = 0; j < DYN_CONTROL_DIM; j++) {
                u_final[j] += u_system[DYN_CONTROL_DIM * i + j];
            }
        }
        for (int j = 0; j < DYN_CONTROL_DIM; j++) {
            du_d[DYN_CONTROL_DIM * bdx + j] = u_final[j];
        }
    }
}

// ── Host: MPPI controller ───────────────────────────────────────────────

bool mppi_init(mppi_controller_t& mppi, dynamics_model_t* dynamics,
               mppi_costs_t* costs, const mppi_config_t& cfg) {
    mppi.dynamics = dynamics;
    mppi.costs = costs;
    mppi.num_timesteps = cfg.num_timesteps;
    mppi.num_rollouts = MPPI_NUM_ROLLOUTS;
    mppi.hz = cfg.hz;
    mppi.gamma = cfg.gamma;
    mppi.num_iters = cfg.num_iters;
    mppi.opt_stride = cfg.optimization_stride;
    mppi.nu[0] = cfg.steering_std;
    mppi.nu[1] = cfg.throttle_std;

    int T = cfg.num_timesteps;
    int N = MPPI_NUM_ROLLOUTS;
    int C = DYN_CONTROL_DIM;

    // Host arrays
    mppi.U = new float[T * C]();
    mppi.du = new float[T * C]();
    mppi.traj_costs = new float[N]();
    mppi.control_hist = new float[2 * C]();

    // Initialize controls
    for (int t = 0; t < T; t++) {
        mppi.U[t * C + 0] = cfg.init_steering;
        mppi.U[t * C + 1] = cfg.init_throttle;
    }

    // GPU arrays
    cudaMalloc(&mppi.state_d, DYN_STATE_DIM * sizeof(float));
    cudaMalloc(&mppi.nu_d, C * sizeof(float));
    cudaMalloc(&mppi.traj_costs_d, N * sizeof(float));
    cudaMalloc(&mppi.U_d, T * C * sizeof(float));
    cudaMalloc(&mppi.du_d, N * T * C * sizeof(float));

    // Transfer exploration variance
    cudaMemcpy(mppi.nu_d, mppi.nu, C * sizeof(float), cudaMemcpyHostToDevice);

    // curand
    cudaStreamCreate(&mppi.stream);
    curandCreateGenerator(&mppi.gen, CURAND_RNG_PSEUDO_DEFAULT);
    curandSetPseudoRandomGeneratorSeed(mppi.gen, 1234ULL);
    curandSetStream(mppi.gen, mppi.stream);

    fprintf(stderr, "[mppi] Init: %d rollouts, %d timesteps, gamma=%.2f\n",
            N, T, mppi.gamma);
    fprintf(stderr, "[mppi] GPU memory: du_d=%.1f KB, total=%.1f KB\n",
            (float)(N * T * C * 4) / 1024.0f,
            (float)(N * T * C * 4 + N * 4 + T * C * 4 + DYN_STATE_DIM * 4) / 1024.0f);

    return true;
}

static void savitzky_golay(mppi_controller_t& mppi) {
    // ref: mppi_controller.cu:416-446
    int T = mppi.num_timesteps;
    int C = DYN_CONTROL_DIM;
    float filter[5] = {-3.0f/35, 12.0f/35, 17.0f/35, 12.0f/35, -3.0f/35};

    // Build padded sequence: [hist[0..1], U[0..T-1], U[T-1], U[T-1]]
    int padded_len = T + 4;
    float* padded = (float*)alloca(padded_len * C * sizeof(float));

    for (int j = 0; j < C; j++) {
        padded[0 * C + j] = mppi.control_hist[0 * C + j];
        padded[1 * C + j] = mppi.control_hist[1 * C + j];
    }
    for (int i = 0; i < T; i++) {
        for (int j = 0; j < C; j++) {
            padded[(i + 2) * C + j] = mppi.U[i * C + j];
        }
    }
    for (int j = 0; j < C; j++) {
        padded[(T + 2) * C + j] = mppi.U[(T - 1) * C + j];
        padded[(T + 3) * C + j] = mppi.U[(T - 1) * C + j];
    }

    for (int i = 0; i < T; i++) {
        for (int j = 0; j < C; j++) {
            float val = 0.0f;
            for (int k = 0; k < 5; k++) {
                val += filter[k] * padded[(i + k) * C + j];
            }
            mppi.U[i * C + j] = val;
        }
    }
}

void mppi_compute_control(mppi_controller_t& mppi, float* state) {
    int T = mppi.num_timesteps;
    int N = mppi.num_rollouts;
    int C = DYN_CONTROL_DIM;

    // Transfer params
    costs_params_to_device(*mppi.costs);
    dynamics_params_to_device(*mppi.dynamics);

    cudaMemcpyAsync(mppi.state_d, state, DYN_STATE_DIM * sizeof(float),
                    cudaMemcpyHostToDevice, mppi.stream);

    for (int iter = 0; iter < mppi.num_iters; iter++) {
        cudaMemcpyAsync(mppi.U_d, mppi.U, C * T * sizeof(float),
                        cudaMemcpyHostToDevice, mppi.stream);

        // Generate random noise
        curandGenerateNormal(mppi.gen, mppi.du_d, N * T * C, 0.0f, 1.0f);

        // Shared memory size for rollout kernel
        int bx = MPPI_BLOCKSIZE_X;
        int shmem_size = bx * DYN_STATE_DIM * sizeof(float) * 2 +  // state + state_der
                         bx * DYN_CONTROL_DIM * sizeof(float) * 3 + // control + var + explore
                         bx * sizeof(int) +                          // crash
                         bx * DYN_SHARED_MEM_REQUEST_BLK * sizeof(float); // theta

        int grid_x = (N - 1) / MPPI_BLOCKSIZE_X + 1;
        dim3 block(MPPI_BLOCKSIZE_X, MPPI_BLOCKSIZE_Y, 1);
        dim3 grid(grid_x, 1, 1);

        rollout_kernel<<<grid, block, shmem_size, mppi.stream>>>(
            T, N, mppi.state_d, mppi.U_d, mppi.du_d, mppi.nu_d,
            mppi.traj_costs_d,
            mppi.dynamics->control_rngs_d,
            mppi.dynamics->stride_idcs_d,
            mppi.dynamics->net_structure_d,
            mppi.dynamics->dt,
            mppi.costs->params_d,
            mppi.costs->costmap_tex,
            mppi.costs->l1_cost,
            mppi.opt_stride);

        // Copy costs back
        cudaMemcpyAsync(mppi.traj_costs, mppi.traj_costs_d,
                        N * sizeof(float), cudaMemcpyDeviceToHost, mppi.stream);
        cudaStreamSynchronize(mppi.stream);

        // Find baseline (min cost)
        float baseline = mppi.traj_costs[0];
        for (int i = 1; i < N; i++) {
            if (mppi.traj_costs[i] < baseline) baseline = mppi.traj_costs[i];
        }

        // Normalize and exponentiate
        int norm_grid = (N - 1) / MPPI_BLOCKSIZE_X + 1;
        norm_exp_kernel<<<norm_grid, MPPI_BLOCKSIZE_X, 0, mppi.stream>>>(
            mppi.traj_costs_d, N, mppi.gamma, baseline);

        cudaMemcpyAsync(mppi.traj_costs, mppi.traj_costs_d,
                        N * sizeof(float), cudaMemcpyDeviceToHost, mppi.stream);
        cudaStreamSynchronize(mppi.stream);

        // Compute normalizer
        float normalizer = 0.0f;
        for (int i = 0; i < N; i++) normalizer += mppi.traj_costs[i];

        // Weighted reduction
        int wr_blocks = (N - 1) / MPPI_BLOCKSIZE_WRX + 1;
        int wr_shmem = wr_blocks * C * sizeof(float);
        weighted_reduction_kernel<<<T, wr_blocks, wr_shmem, mppi.stream>>>(
            mppi.traj_costs_d, mppi.du_d, normalizer, T, N);

        // Copy control update back
        cudaMemcpyAsync(mppi.du, mppi.du_d, T * C * sizeof(float),
                        cudaMemcpyDeviceToHost, mppi.stream);
        cudaStreamSynchronize(mppi.stream);

        // Apply update
        for (int i = 0; i < T * C; i++) {
            mppi.U[i] = mppi.du[i];
        }
    }

    // Smooth
    savitzky_golay(mppi);
}

void mppi_slide_control_seq(mppi_controller_t& mppi, int stride) {
    int T = mppi.num_timesteps;
    int C = DYN_CONTROL_DIM;

    // Update history for Savitzky-Golay
    if (stride == 1) {
        mppi.control_hist[0] = mppi.control_hist[2];
        mppi.control_hist[1] = mppi.control_hist[3];
        mppi.control_hist[2] = mppi.U[0];
        mppi.control_hist[3] = mppi.U[1];
    } else if (stride >= 2) {
        int t = stride - 2;
        for (int i = 0; i < 4; i++) {
            mppi.control_hist[i] = mppi.U[t + i];
        }
    }

    // Slide sequence
    for (int i = 0; i < T - stride; i++) {
        for (int j = 0; j < C; j++) {
            mppi.U[i * C + j] = mppi.U[(i + stride) * C + j];
        }
    }
    // Fill tail with init values
    for (int s = 1; s <= stride; s++) {
        mppi.U[(T - s) * C + 0] = 0.0f;  // init_steering
        mppi.U[(T - s) * C + 1] = 0.0f;  // init_throttle
    }
}

void mppi_reset_controls(mppi_controller_t& mppi) {
    int T = mppi.num_timesteps;
    int C = DYN_CONTROL_DIM;
    for (int i = 0; i < T * C; i++) {
        mppi.U[i] = 0.0f;
    }
    memset(mppi.control_hist, 0, 2 * C * sizeof(float));
}

void mppi_get_control(const mppi_controller_t& mppi,
                      float* steering, float* throttle) {
    *steering = mppi.U[0];
    *throttle = mppi.U[1];
}

void mppi_free(mppi_controller_t& mppi) {
    cudaFree(mppi.state_d);
    cudaFree(mppi.nu_d);
    cudaFree(mppi.traj_costs_d);
    cudaFree(mppi.U_d);
    cudaFree(mppi.du_d);
    curandDestroyGenerator(mppi.gen);
    cudaStreamDestroy(mppi.stream);
    delete[] mppi.U;
    delete[] mppi.du;
    delete[] mppi.traj_costs;
    delete[] mppi.control_hist;
}

// ── Thread 1: Planning loop ─────────────────────────────────────────────
// ref: run_control_loop.cuh:105-189

void planning_loop(shared_bus_t* bus, const config_t& cfg) {
    fprintf(stderr, "[T1] Planning thread starting\n");

    // Initialize dynamics
    dynamics_model_t dynamics;
    if (!dynamics_load(dynamics, cfg.mppi.model_path.c_str(),
                       1.0f / cfg.mppi.hz, cfg.mppi.max_throttle)) {
        fprintf(stderr, "[T1] Failed to load dynamics model, using zero weights\n");
        // Continue with zero weights for testing
        dynamics.dt = 1.0f / cfg.mppi.hz;
        dynamics.net_structure[0] = 6;
        dynamics.net_structure[1] = 32;
        dynamics.net_structure[2] = 32;
        dynamics.net_structure[3] = 4;
        dynamics.control_rngs[0] = make_float2(-0.99f, 0.99f);
        dynamics.control_rngs[1] = make_float2(-0.99f, cfg.mppi.max_throttle);
        memset(dynamics.net_params, 0, sizeof(dynamics.net_params));
        // Compute stride indices for zero weights
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
    }

    // Initialize costs
    mppi_costs_t costs;
    costs_init(costs, cfg.costs, cfg.costmap);

    // Initialize MPPI controller
    mppi_controller_t mppi;
    mppi_init(mppi, &dynamics, &costs, cfg.mppi);

    uint64_t period_us = 1000000 / cfg.mppi.hz;
    uint64_t next_tick = now_us();
    uint32_t iter_count = 0;

    float state[DYN_STATE_DIM] = {0};

    while (bus->alive.load(std::memory_order_relaxed)) {
        scoped_timer_t timer("planning", 1000.0 / cfg.mppi.hz);

        // Read state from bus
        state_7d_t s = seqlock_read(bus->state_lock, bus->state);
        state[0] = s.x;
        state[1] = s.y;
        state[2] = s.yaw;
        state[3] = s.roll;
        state[4] = s.u_x;
        state[5] = s.u_y;
        state[6] = s.yaw_rate;

        // Check for new costmap
        // TODO(epic3): rebind costmap texture when costmap_stamp changes

        // Slide control sequence
        int stride = mppi.opt_stride;
        if (stride > 0 && stride < mppi.num_timesteps) {
            mppi_slide_control_seq(mppi, stride);
        }

        // Compute optimal control
        mppi_compute_control(mppi, state);

        // Write control to bus
        float steer, throt;
        mppi_get_control(mppi, &steer, &throt);
        control_2d_t ctrl;
        ctrl.steering = steer;
        ctrl.throttle = throt;
        ctrl.timestamp_us = now_us();
        seqlock_write(bus->control_lock, bus->control, ctrl);

        iter_count++;
        if (iter_count % 100 == 0) {
            fprintf(stderr, "[T1] iter=%u steer=%.3f throt=%.3f\n",
                    iter_count, steer, throt);
        }

        next_tick += period_us;
        sleep_until_us(next_tick);
    }

    // Cleanup
    mppi_free(mppi);
    costs_free(costs);
    dynamics_free(dynamics);
    fprintf(stderr, "[T1] Planning thread stopped (iters=%u)\n", iter_count);
}

} // namespace truggy
