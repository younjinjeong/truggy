// BEV (Bird's Eye View) Costmap Generator
// Projects stereo depth + segmentation mask into an ego-centric overhead grid
// Grid: 200x120 cells, 5cm/cell = 10m wide x 6m forward

#include "perception/costmap.h"
#include <cstdio>
#include <cstring>

namespace truggy {

// ── CUDA kernel: depth-to-BEV projection ────────────────────────────────
// Each thread handles one costmap cell. Back-projects to camera pixel,
// samples depth, and assigns cost based on obstacle distance + segmentation.

__global__ void generate_costmap_kernel(
    float* costmap_out,
    const float* depth_gpu, int depth_w, int depth_h,
    const uint8_t* seg_mask_gpu, int seg_w, int seg_h,
    int map_w, int map_h,
    float cell_size, float x_min, float y_min,
    float cam_height, float cos_pitch, float sin_pitch,
    float fx, float fy, float cx, float cy)
{
    int col = blockIdx.x * blockDim.x + threadIdx.x;
    int row = blockIdx.y * blockDim.y + threadIdx.y;
    if (col >= map_w || row >= map_h) return;

    // World coordinates of this cell (ego-centric: X=right, Y=forward)
    float world_x = x_min + (col + 0.5f) * cell_size;
    float world_y = y_min + (row + 0.5f) * cell_size;

    // Skip cells behind the camera
    if (world_y < 0.1f) {
        costmap_out[row * map_w + col] = 0.0f;
        return;
    }

    // Project world point to camera pixel
    // Camera frame: Z forward, X right, Y down
    float cam_z = world_y * cos_pitch + cam_height * sin_pitch;
    float cam_x = world_x;
    float cam_y = -(world_y * sin_pitch - cam_height * cos_pitch);

    if (cam_z < 0.3f) {
        costmap_out[row * map_w + col] = 0.0f;
        return;
    }

    float inv_z = 1.0f / cam_z;
    int px = (int)(fx * cam_x * inv_z + cx);
    int py = (int)(fy * cam_y * inv_z + cy);

    float cost = 0.0f;

    // Sample depth map
    if (px >= 0 && px < depth_w && py >= 0 && py < depth_h) {
        float measured_depth = depth_gpu[py * depth_w + px];

        if (isfinite(measured_depth) && measured_depth > 0.3f) {
            // If measured depth is closer than expected, there's an obstacle
            float expected_depth = cam_z;
            if (measured_depth < expected_depth - 0.3f) {
                cost = 1.0f;  // obstacle
            }
        } else {
            // Invalid depth = unknown, treat as mild cost
            cost = 0.3f;
        }
    } else {
        // Outside camera FOV = unknown
        cost = 0.2f;
    }

    // Override with segmentation if available
    if (seg_mask_gpu && cost < 0.5f) {
        // Map pixel coords to segmentation resolution
        int sx = px * seg_w / depth_w;
        int sy = py * seg_h / depth_h;
        if (sx >= 0 && sx < seg_w && sy >= 0 && sy < seg_h) {
            uint8_t seg_class = seg_mask_gpu[sy * seg_w + sx];
            if (seg_class == 1) cost = 1.0f;       // obstacle
            else if (seg_class == 2) cost = 0.7f;   // boundary
            // class 0 = drivable, keep existing cost
        }
    }

    costmap_out[row * map_w + col] = cost;
}

// ── Host functions ──────────────────────────────────────────────────────

void costmap_alloc(costmap_buffers_t& cm, int width, int height,
                   float cell_size, float x_min, float y_min) {
    cm.width = width;
    cm.height = height;
    cm.cell_size = cell_size;
    cm.x_min = x_min;
    cm.y_min = y_min;
    cm.active = 0;

    int size = width * height * sizeof(float);
    cudaMalloc(&cm.buf_a, size);
    cudaMalloc(&cm.buf_b, size);
    cudaMemset(cm.buf_a, 0, size);
    cudaMemset(cm.buf_b, 0, size);

    fprintf(stderr, "[costmap] Allocated %dx%d double-buffer (%.1f KB each)\n",
            width, height, size / 1024.0f);
}

void costmap_generate(costmap_buffers_t& cm,
                      const float* depth_gpu, int depth_w, int depth_h,
                      const uint8_t* seg_mask_gpu, int seg_w, int seg_h,
                      float cam_height, float cam_pitch_rad,
                      float fx, float fy, float cx, float cy,
                      cudaStream_t stream) {
    // Write to inactive buffer
    float* write_buf = (cm.active == 0) ? cm.buf_b : cm.buf_a;

    float cos_p = cosf(cam_pitch_rad);
    float sin_p = sinf(cam_pitch_rad);

    dim3 block(16, 16);
    dim3 grid((cm.width + 15) / 16, (cm.height + 15) / 16);

    generate_costmap_kernel<<<grid, block, 0, stream>>>(
        write_buf,
        depth_gpu, depth_w, depth_h,
        seg_mask_gpu, seg_w, seg_h,
        cm.width, cm.height,
        cm.cell_size, cm.x_min, cm.y_min,
        cam_height, cos_p, sin_p,
        fx, fy, cx, cy);

    cudaStreamSynchronize(stream);

    // Swap active buffer
    cm.active = 1 - cm.active;
}

float* costmap_get_readable(const costmap_buffers_t& cm) {
    return (cm.active == 0) ? cm.buf_a : cm.buf_b;
}

void costmap_free(costmap_buffers_t& cm) {
    cudaFree(cm.buf_a);
    cudaFree(cm.buf_b);
}

} // namespace truggy
