#include <cstdio>

// Constant memory for NN weights (1412 floats)
// ref: path_integral_main.cu:47
#ifdef MPPI_NNET_USING_CONSTANT_MEM__
__device__ __constant__ float NNET_PARAMS[1412];
#endif

namespace truggy {

// TODO(epic4): port NeuralNetModel<7,2,3,6,32,32,4> from AutoRally
// ref: neural_net_model.cu

void dynamics_init() {
    fprintf(stderr, "[dynamics] init (stub)\n");
}

} // namespace truggy
