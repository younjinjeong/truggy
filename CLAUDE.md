# TruggyAD — Vision-Only Autonomous Driving 1/8 RC Truggy

## Project

Port AutoRally MPPI controller to standalone C++ binary for 1/8 scale RC Truggy.
No ROS, no GPS, no LiDAR. ZED stereo camera + BNO085 IMU + wheel encoders only.
Reference code at `../autorally/`.

## Architecture

Single process, 4 pthreads, seqlock shared bus (`shared_bus_t` in `src/common/types.h`).

| Thread | Module | Rate | CPU Core | Priority |
|--------|--------|------|----------|----------|
| T0 | Perception (ZED + TensorRT + costmap) | 30 Hz | Core 3 | default |
| T1 | Planning (MPPI CUDA) | 50 Hz | Core 0 | SCHED_FIFO 90 |
| T2 | State Estimation (EKF) | 100 Hz | Core 1 | default |
| T3 | Actuation (Arduino serial) | 100 Hz | Core 2 | SCHED_FIFO 80 |

## Coding Style

- **Language:** C++17, compiled with CUDA (`.cu` files)
- **Style:** C-like C++ — structs over classes, free functions, `snake_case`
- **No exceptions**, no RTTI, no `dynamic_cast`, no `typeid`
- **Minimal STL** — fixed-size arrays in hot paths, no `std::vector` in real-time threads
- **Headers:** `.h` for declarations, `.cpp` for CPU impl, `.cu` for CUDA impl
- **Memory:** No heap allocation in hot loops. Pre-allocate everything at startup.
- **Comments:** Only where logic is non-obvious. No boilerplate comments.

## Build

```bash
# Jetson Nano 2GB (native)
cmake -B build -DTARGET_NANO_2GB=ON -DCMAKE_EXPORT_COMPILE_COMMANDS=ON
cmake --build build -j4

# Orin Nano Super
cmake -B build -DTARGET_ORIN_NANO=ON -DCMAKE_EXPORT_COMPILE_COMMANDS=ON
cmake --build build -j6

# Remote build (WSL2 RTX 4080)
./scripts/remote-build.sh --native     # x86_64 for testing
./scripts/remote-build.sh              # cross-arch sm_53 for Nano
```

## CUDA Targets

| Platform | Arch | Rollouts | Block Size | Flags |
|----------|------|----------|------------|-------|
| Nano 2GB | sm_53 | 768 | 8x4 | `--maxrregcount=32 -use_fast_math` |
| Orin Nano | sm_87 | 4096 | 16x8 | `--maxrregcount=48 -use_fast_math` |

## State Vector (7D) — matches AutoRally exactly

```
state[0] = x          // world X (m)
state[1] = y          // world Y (m)
state[2] = yaw        // heading (rad)
state[3] = roll       // roll angle (rad)
state[4] = u_x        // body-frame longitudinal velocity (m/s)
state[5] = u_y        // body-frame lateral velocity (m/s)
state[6] = yaw_rate   // yaw angular velocity (rad/s)
```

**Critical:** `dyaw/dt = -state[6]` (negative sign in kinematics, ref: `neural_net_model.cu:348`)

## Control Vector (2D)

```
control[0] = steering   // [-0.99, 0.99]
control[1] = throttle   // [-0.99, max_throttle]
```

## Key Constants

- Costmap: 200x120 cells, 5cm/cell, ego-centric BEV
- Serial: 115200 baud, 8N1, binary protocol (8-byte cmd, 24-byte telemetry)
- NN dynamics: 6→32→32→4, tanh, 1412 params in CUDA constant memory
- CRC-8: polynomial 0x31 (Dallas/Maxim)

## Dependencies

- CUDA toolkit (10.2 on Nano, 12.x on Orin)
- ZED SDK 4.x (C++ API)
- TensorRT 8.x/10.x (C++ API)
- Eigen3 (EKF matrix ops)
- yaml-cpp (config loading)
- zlib (cnpy dependency)
- curand (MPPI noise generation)

## File Layout

```
src/common/     — types.h, ring_buffer.h, config.h/.cpp, serial.h/.cpp, timing.h
src/perception/ — perception.h/.cpp, segmentation.h/.cpp, costmap.h/.cu
src/planning/   — mppi.h/.cu, dynamics.h/.cu, costs.h/.cu
src/state/      — ekf.h/.cpp, imu.h
src/actuation/  — bridge.h/.cpp
```

## AutoRally Reference Paths

```
../autorally/autorally_control/include/autorally_control/path_integral/
  mppi_controller.cu    — 3 CUDA kernels (rollout, normExp, weightedReduction)
  neural_net_model.cu   — NN dynamics model
  costs.cu              — cost functions (track, speed, crash, stabilizing)
  run_control_loop.cuh  — control loop (our Thread 1 replaces this)
  path_integral_main.cu — template instantiation pattern
```
