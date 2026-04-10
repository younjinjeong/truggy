#include <cstdio>

namespace truggy {

// TODO(epic4): port 3 CUDA kernels from AutoRally mppi_controller.cu
// - rolloutKernel (lines 48-152)
// - normExpKernel
// - weightedReductionKernel
// + computeControl() flow (lines 498-559)

void mppi_init() {
    fprintf(stderr, "[mppi] init (stub)\n");
}

} // namespace truggy
