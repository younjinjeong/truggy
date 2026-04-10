# MPPI Reference

Model Predictive Path Integral (MPPI) control algorithm, ported from Georgia Tech AutoRally.

## Algorithm Overview

MPPI is a sampling-based optimal control method that:
1. Samples N random trajectories (rollouts) in parallel on GPU
2. Evaluates each trajectory's cost on a costmap
3. Computes optimal control as a cost-weighted average of all samples

Unlike MPC with gradient optimization, MPPI uses importance sampling — no linearization, handles non-convex costs naturally.

## Key Parameters

| Parameter | Nano 2GB | Orin Nano | AutoRally Original |
|-----------|----------|-----------|-------------------|
| Rollouts | 768 | 4096 | 1920 |
| BLOCKSIZE_X | 8 | 16 | 8 |
| BLOCKSIZE_Y | 4 | 8 | 16 |
| Timesteps | 100 | 100 | 100 |
| dt | 0.02 s | 0.02 s | 0.02 s |
| Horizon | 2.0 s | 2.0 s | 2.0 s |
| Control Hz | 50 | 110 | 50 |
| gamma | 0.15 | 0.15 | 0.15 |
| Steering std | 0.275 | 0.275 | 0.275 |
| Throttle std | 0.3 | 0.3 | 0.3 |

## State Vector (7D)

```
state[0] = x          // world position X (meters)
state[1] = y          // world position Y (meters)
state[2] = yaw        // heading angle (radians)
state[3] = roll       // roll angle (radians)
state[4] = u_x        // body-frame longitudinal velocity (m/s)
state[5] = u_y        // body-frame lateral velocity (m/s)
state[6] = yaw_rate   // yaw angular velocity (rad/s)
```

**Critical:** The yaw derivative uses `state_der[2] = -state[6]` (negative sign). This sign convention must be consistent across EKF and dynamics model.

## Control Vector (2D)

```
control[0] = steering   // [-0.99, 0.99]
control[1] = throttle   // [-0.99, max_throttle]
```

## Three CUDA Kernels

### 1. `rolloutKernel`

Parallel trajectory simulation. Each CUDA thread computes one rollout.

```
Grid:  ((NUM_ROLLOUTS-1)/BLOCKSIZE_X + 1, 1, 1)
Block: (BLOCKSIZE_X, BLOCKSIZE_Y, 1)
```

Per-thread loop (100 timesteps):
```
for each timestep t:
    u = U[t] + du * noise           // sample perturbed control
    u = clamp(u, min, max)          // enforce constraints
    s_dot = dynamics(s, u)          // neural net forward pass
    s = s + dt * s_dot              // Euler integration
    cost += evaluate_cost(s, u, t)  // accumulate running cost
```

Shared memory layout per block:
```
state_shared[BLOCKSIZE_X * STATE_DIM]           // current states
state_der_shared[BLOCKSIZE_X * STATE_DIM]       // state derivatives
control_shared[BLOCKSIZE_X * CONTROL_DIM]       // nominal controls
control_var_shared[BLOCKSIZE_X * CONTROL_DIM]   // control perturbations
exploration_variance[BLOCKSIZE_X * CONTROL_DIM] // noise parameters
crash_status[BLOCKSIZE_X]                       // collision flags
theta[SHARED_MEM_GRD + SHARED_MEM_BLK * BLOCKSIZE_X]  // NN weights
```

### 2. `normExpKernel`

Softmax cost weighting:
```
w[i] = exp(-gamma * (cost[i] - min_cost))
```

### 3. `weightedReductionKernel`

Optimal control via weighted average:
```
for each timestep t:
    u_optimal[t] = sum(w[i] * u[i][t]) / sum(w[i])
```

Block: `(BLOCKSIZE_WRX=64, 1, 1)`, Grid: `(num_timesteps, 1, 1)`

## `computeControl()` Flow

```
1. cudaMemcpy state → GPU
2. cudaMemcpy control sequence → GPU
3. curandGenerateNormal → noise (count = rollouts * timesteps * 2)
4. rolloutKernel<<<...>>>       // parallel rollouts
5. cudaMemcpy costs → CPU, find min cost (baseline)
6. normExpKernel<<<...>>>       // softmax weighting
7. cudaMemcpy weights → CPU, compute sum (normalizer)
8. weightedReductionKernel<<<...>>>  // weighted average
9. cudaMemcpy control update → CPU
10. Savitzky-Golay smoothing filter
11. Compute nominal trajectory (CPU)
```

## Neural Net Dynamics Model

Architecture: `NeuralNetModel<7, 2, 3, 6, 32, 32, 4>`

```
Input (6D):  [u_x, u_y, yaw_rate, steering, throttle, dt]
Hidden 1:    32 neurons, tanh activation
Hidden 2:    32 neurons, tanh activation
Output (4D): [du_x/dt, du_y/dt, dyaw_rate/dt, droll/dt]
```

Total parameters: 1412 floats (stored in CUDA constant memory)

### Kinematics (computed separately, not learned)

```
dx/dt     = cos(yaw) * u_x - sin(yaw) * u_y
dy/dt     = sin(yaw) * u_x + cos(yaw) * u_y
dyaw/dt   = -yaw_rate    // NOTE: negative sign
```

### Weight Format (.npz)

Keys: `dynamics_W1`, `dynamics_b1`, `dynamics_W2`, `dynamics_b2`, `dynamics_W3`, `dynamics_b3`

Loaded via cnpy library. Matrices in row-major, double precision (converted to float).

## Cost Function

Five components summed per timestep:

### 1. Speed Cost
```
speed_cost = speed_coeff * (u_x - desired_speed)^2
```

### 2. Track Cost (texture lookup)
```
cost = tex2D(costmap_texture, u, v)
track_cost = track_coeff * cost
```
Where (u, v) are normalized costmap coordinates from the vehicle's position.

### 3. Stabilizing Cost (slip angle)
```
slip_angle = atan2(u_y, max(u_x, 1.0))
if |slip_angle| > max_slip_angle:
    stabilizing_cost = slip_penalty * (|slip_angle| - max_slip_angle)
```

### 4. Control Cost (information-theoretic)
```
control_cost = gamma * (steering_coeff * u_steer * du_steer / var_steer
                      + throttle_coeff * u_throt * du_throt / var_throt)
```

### 5. Crash Cost
```
if |roll| > PI/2 (1.57 rad):
    crash_cost = crash_coeff
    terminate rollout
```

### Running Cost Accumulation
```
running_cost += (cost_t - running_cost) / t   // running average
final_cost = running_cost + terminal_cost
```

## Costmap

### AutoRally (static)
- Pre-built from GPS survey of track
- Loaded from `.npz` file at startup
- Bound to CUDA texture object once

### TruggyAD (real-time)
- Generated every frame from stereo depth + segmentation
- 200x120 cells, 5 cm/cell (10m x 6m ego-centric)
- Rebuilt and rebound to CUDA texture at 30 Hz
- Double-buffered to avoid tearing

## Cost Parameter Defaults

```yaml
desired_speed: 4.0        # m/s (conservative start)
speed_coeff: 4.25
track_coeff: 200.0
max_slip_angle: 1.25      # radians
slip_penalty: 10.0
crash_coeff: 10000.0
steering_coeff: 0.0       # no control regularization initially
throttle_coeff: 0.0
boundary_threshold: 0.65
discount: 0.9
```

## Performance Targets

| Metric | Nano 2GB | Orin Nano |
|--------|----------|-----------|
| computeControl() | < 20 ms | < 10 ms |
| Control rate | 50 Hz | 110 Hz |
| Sensor-to-actuator | < 50 ms | < 15 ms |
| Max safe speed | ~8 m/s | ~20 m/s |
| State evaluations/s | 4.2M | 45M |

## AutoRally Source References

| Topic | File | Lines |
|-------|------|-------|
| rolloutKernel | `mppi_controller.cu` | 48-152 |
| computeControl | `mppi_controller.cu` | 498-559 |
| Savitzky-Golay | `mppi_controller.cu` | 416-446 |
| Dynamics (GPU) | `neural_net_model.cu` | 343-403 |
| Kinematics | `neural_net_model.cu` | 193-197 |
| Cost functions | `costs.cu` | 301-409 |
| Texture binding | `costs.cu` | 129-155 |
| Template instantiation | `path_integral_main.cu` | 66-78 |
| Control loop | `run_control_loop.cuh` | 105-189 |
