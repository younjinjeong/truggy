// Ported from AutoRally costs.cu (Georgia Tech, BSD License)
// Original author: Grady Williams <gradyrw@gmail.com>
// Simplified: single-channel float costmap, ego-centric BEV coordinates

#include "planning/costs.h"
#include <cstdio>
#include <cstring>

namespace truggy {

// ── Host functions ──────────────────────────────────────────────────────

void costs_init(mppi_costs_t& costs, const cost_config_t& cfg,
                const costmap_config_t& map_cfg) {
    costs.width = map_cfg.width;
    costs.height = map_cfg.height;
    costs.l1_cost = cfg.l1_cost;
    costs.tex_initialized = false;

    // Set params
    costs.params.desired_speed = cfg.desired_speed;
    costs.params.speed_coeff = cfg.speed_coeff;
    costs.params.track_coeff = cfg.track_coeff;
    costs.params.max_slip_ang = cfg.max_slip_angle;
    costs.params.slip_penalty = cfg.slip_penalty;
    costs.params.track_slop = cfg.track_slop;
    costs.params.crash_coeff = cfg.crash_coeff;
    costs.params.steering_coeff = cfg.steering_coeff;
    costs.params.throttle_coeff = cfg.throttle_coeff;
    costs.params.boundary_threshold = cfg.boundary_threshold;
    costs.params.discount = cfg.discount;
    costs.params.map_x_min = map_cfg.x_min;
    costs.params.map_x_max = map_cfg.x_max;
    costs.params.map_y_min = map_cfg.y_min;
    costs.params.map_y_max = map_cfg.y_max;

    // Allocate GPU params
    cudaMalloc(&costs.params_d, sizeof(cost_params_gpu_t));

    // Allocate texture array (single-channel float)
    costs.channel_desc = cudaCreateChannelDesc(32, 0, 0, 0,
                                                cudaChannelFormatKindFloat);
    cudaMallocArray(&costs.costmap_array_d, &costs.channel_desc,
                    costs.width, costs.height);

    // Initialize with zeros
    int size = costs.width * costs.height * sizeof(float);
    float* zeros = (float*)calloc(costs.width * costs.height, sizeof(float));
    cudaMemcpyToArray(costs.costmap_array_d, 0, 0, zeros, size,
                      cudaMemcpyHostToDevice);
    free(zeros);

    // Create initial texture object
    struct cudaResourceDesc res_desc;
    memset(&res_desc, 0, sizeof(res_desc));
    res_desc.resType = cudaResourceTypeArray;
    res_desc.res.array.array = costs.costmap_array_d;

    struct cudaTextureDesc tex_desc;
    memset(&tex_desc, 0, sizeof(tex_desc));
    tex_desc.addressMode[0] = cudaAddressModeClamp;
    tex_desc.addressMode[1] = cudaAddressModeClamp;
    tex_desc.filterMode = cudaFilterModeLinear;
    tex_desc.readMode = cudaReadModeElementType;
    tex_desc.normalizedCoords = 1;

    cudaCreateTextureObject(&costs.costmap_tex, &res_desc, &tex_desc, nullptr);
    costs.tex_initialized = true;

    costs_params_to_device(costs);
    fprintf(stderr, "[costs] Initialized %dx%d costmap\n",
            costs.width, costs.height);
}

void costs_update_params(mppi_costs_t& costs, const cost_config_t& cfg) {
    costs.params.desired_speed = cfg.desired_speed;
    costs.params.speed_coeff = cfg.speed_coeff;
    costs.params.track_coeff = cfg.track_coeff;
    costs.params.max_slip_ang = cfg.max_slip_angle;
    costs.params.slip_penalty = cfg.slip_penalty;
    costs.params.crash_coeff = cfg.crash_coeff;
    costs.l1_cost = cfg.l1_cost;
    costs_params_to_device(costs);
}

void costs_params_to_device(mppi_costs_t& costs) {
    cudaMemcpy(costs.params_d, &costs.params,
               sizeof(cost_params_gpu_t), cudaMemcpyHostToDevice);
}

void costs_update_costmap_gpu(mppi_costs_t& costs, float* gpu_costmap,
                               int width, int height) {
    // Copy from GPU buffer to CUDA array
    cudaMemcpy2DToArray(costs.costmap_array_d, 0, 0,
                        gpu_costmap, width * sizeof(float),
                        width * sizeof(float), height,
                        cudaMemcpyDeviceToDevice);

    // Rebuild texture object
    if (costs.tex_initialized) {
        cudaDestroyTextureObject(costs.costmap_tex);
    }

    struct cudaResourceDesc res_desc;
    memset(&res_desc, 0, sizeof(res_desc));
    res_desc.resType = cudaResourceTypeArray;
    res_desc.res.array.array = costs.costmap_array_d;

    struct cudaTextureDesc tex_desc;
    memset(&tex_desc, 0, sizeof(tex_desc));
    tex_desc.addressMode[0] = cudaAddressModeClamp;
    tex_desc.addressMode[1] = cudaAddressModeClamp;
    tex_desc.filterMode = cudaFilterModeLinear;
    tex_desc.readMode = cudaReadModeElementType;
    tex_desc.normalizedCoords = 1;

    cudaCreateTextureObject(&costs.costmap_tex, &res_desc, &tex_desc, nullptr);
    costs.tex_initialized = true;
}

void costs_free(mppi_costs_t& costs) {
    if (costs.tex_initialized) {
        cudaDestroyTextureObject(costs.costmap_tex);
    }
    cudaFreeArray(costs.costmap_array_d);
    cudaFree(costs.params_d);
}

// ── Device: cost functions ──────────────────────────────────────────────
// ref: costs.cu:301-414

__device__ void costs_get_crash(float* s, int* crash) {
    if (fabsf(s[3]) > 1.57f) {
        crash[0] = 1;
    }
}

__device__ static float get_speed_cost(float* s, const cost_params_gpu_t* p, bool l1) {
    float error = s[4] - p->desired_speed;
    float cost = l1 ? fabsf(error) : error * error;
    return p->speed_coeff * cost;
}

__device__ static float get_crash_cost(float* s, int* crash, int timestep,
                                        const cost_params_gpu_t* p) {
    if (crash[0] > 0) {
        return p->crash_coeff;
    }
    return 0.0f;
}

__device__ static float get_stabilizing_cost(float* s, const cost_params_gpu_t* p) {
    if (fabsf(s[4]) > 0.001f) {
        float slip = -atanf(s[5] / fabsf(s[4]));
        float cost = p->slip_penalty * slip * slip;
        if (fabsf(slip) > p->max_slip_ang) {
            cost += p->crash_coeff;
        }
        return cost;
    }
    return 0.0f;
}

__device__ static float get_control_cost(float* u, float* du, float* vars,
                                          const cost_params_gpu_t* p) {
    float cost = 0.0f;
    cost += p->steering_coeff * du[0] * (u[0] - du[0]) / (vars[0] * vars[0]);
    cost += p->throttle_coeff * du[1] * (u[1] - du[1]) / (vars[1] * vars[1]);
    return cost;
}

__device__ static float get_track_cost(float* s, int* crash,
                                        const cost_params_gpu_t* p,
                                        cudaTextureObject_t tex) {
    // Compute front and back positions
    float x_front = s[0] + COST_FRONT_D * __cosf(s[2]);
    float y_front = s[1] + COST_FRONT_D * __sinf(s[2]);
    float x_back  = s[0] + COST_BACK_D * __cosf(s[2]);
    float y_back  = s[1] + COST_BACK_D * __sinf(s[2]);

    // Ego-centric BEV coordinate transform (simplified from AutoRally's coorTransform)
    // u = (x - x_min) / (x_max - x_min), v = (y - y_min) / (y_max - y_min)
    float inv_x = 1.0f / (p->map_x_max - p->map_x_min);
    float inv_y = 1.0f / (p->map_y_max - p->map_y_min);

    float u_front = (x_front - p->map_x_min) * inv_x;
    float v_front = (y_front - p->map_y_min) * inv_y;
    float u_back  = (x_back - p->map_x_min) * inv_x;
    float v_back  = (y_back - p->map_y_min) * inv_y;

    // Sample costmap texture (normalized coords, single-channel float)
    float cost_front = tex2D<float>(tex, u_front, v_front);
    float cost_back  = tex2D<float>(tex, u_back, v_back);

    float track_cost = (fabsf(cost_front) + fabsf(cost_back)) / 2.0f;
    if (fabsf(track_cost) < p->track_slop) {
        track_cost = 0.0f;
    } else {
        track_cost = p->track_coeff * track_cost;
    }

    // Check if off track
    if (cost_front >= p->boundary_threshold ||
        cost_back >= p->boundary_threshold) {
        crash[0] = 1;
    }

    return track_cost;
}

__device__ float costs_compute(float* s, float* u, float* du, float* vars,
                               int* crash, int timestep,
                               const cost_params_gpu_t* params,
                               cudaTextureObject_t costmap_tex,
                               bool l1_cost) {
    float control_cost = get_control_cost(u, du, vars, params);
    float track_cost = get_track_cost(s, crash, params, costmap_tex);
    float speed_cost = get_speed_cost(s, params, l1_cost);
    float crash_cost = __powf(params->discount, (float)timestep) *
                       get_crash_cost(s, crash, timestep, params);
    float stabilizing_cost = get_stabilizing_cost(s, params);

    float cost = control_cost + speed_cost + crash_cost +
                 track_cost + stabilizing_cost;

    if (cost > 1e12f || isnan(cost)) {
        cost = 1e12f;
    }
    return cost;
}

__device__ float costs_terminal(float* s) {
    return 0.0f;
}

} // namespace truggy
