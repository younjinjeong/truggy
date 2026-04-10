#include <cstdio>
#include <cstdint>

namespace truggy {

// TODO(epic3): implement CUDA BEV costmap generation kernel
// Grid: 200x120, 5cm/cell, ego-centric
// Input: depth map (GPU) + segmentation mask (GPU)
// Output: float costmap, bind to CUDA texture for MPPI

__global__ void generate_costmap_kernel(
    const float* depth, const uint8_t* seg_mask,
    float* costmap_out,
    int depth_w, int depth_h,
    int map_w, int map_h,
    float cell_size,
    float cam_height, float cam_pitch) {
    // placeholder
}

void init_costmap_buffers(int w, int h) {
    fprintf(stderr, "[costmap] init %dx%d (stub)\n", w, h);
}

} // namespace truggy
