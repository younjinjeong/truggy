#pragma once

#include <cuda_runtime.h>
#include <cstdint>

namespace truggy {

struct costmap_buffers_t {
    float* buf_a;  // GPU double-buffer A
    float* buf_b;  // GPU double-buffer B
    int active;    // 0 = A is readable, 1 = B is readable
    int width, height;
    float cell_size;
    float x_min, y_min;
};

// Allocate double-buffered GPU costmap
void costmap_alloc(costmap_buffers_t& cm, int width, int height,
                   float cell_size, float x_min, float y_min);

// Generate BEV costmap from depth + segmentation on GPU
// depth: float depth map on GPU (meters), depth_w x depth_h
// seg_mask: uint8 segmentation mask on GPU (0=drivable, 1=obstacle, 2=boundary)
//           can be nullptr (no segmentation, depth-only obstacles)
void costmap_generate(costmap_buffers_t& cm,
                      const float* depth_gpu, int depth_w, int depth_h,
                      const uint8_t* seg_mask_gpu, int seg_w, int seg_h,
                      float cam_height, float cam_pitch_rad,
                      float fx, float fy, float cx, float cy,
                      cudaStream_t stream);

// Get pointer to current readable buffer
float* costmap_get_readable(const costmap_buffers_t& cm);

void costmap_free(costmap_buffers_t& cm);

} // namespace truggy
