# Architecture

## Overview

TruggyAD is a **single-process, multi-threaded** C++ application. Four pthreads communicate through a shared memory bus using seqlock synchronization. There is no ROS dependency.

## Thread Architecture

```
┌──────────────────────────────────────────────────────────────┐
│                     truggy_ad (single binary)                 │
│                                                               │
│  ┌───────────────┐  ┌───────────────┐  ┌───────────────┐     │
│  │ Thread 0      │  │ Thread 1      │  │ Thread 2      │     │
│  │ Perception    │  │ Planning      │  │ State Est.    │     │
│  │               │  │               │  │               │     │
│  │ ZED capture   │  │ MPPI (CUDA)   │  │ EKF fusion    │     │
│  │ Segmentation  │  │ 768 rollouts  │  │ IMU+VIO+Enc   │     │
│  │ Costmap gen   │  │ 50 Hz ctrl    │  │ 100 Hz        │     │
│  │ VIO pose      │  │               │  │               │     │
│  │ 30 Hz         │  │               │  │               │     │
│  └──────┬────────┘  └──────┬────────┘  └──────┬────────┘     │
│         │                  │                   │              │
│         └───── shared_bus_t (seqlock) ─────────┘              │
│                            │                                  │
│                   ┌────────┴─────────┐                        │
│                   │ Thread 3         │                        │
│                   │ Actuation        │                        │
│                   │ Arduino Serial   │                        │
│                   │ 115200 baud      │                        │
│                   │ 100 Hz           │                        │
│                   └──────────────────┘                        │
└──────────────────────────────────────────────────────────────┘
```

## Data Flow

```
ZED Camera (720p/30fps)
    │
    ├──► Left Image ──► Track Segmentation NN (TensorRT) ──► Binary Mask
    │                                                            │
    ├──► Stereo Depth Map ─────────────────────────────────┐     │
    │                                                      │     │
    ├──► Visual Odometry (ZED SDK Positional Tracking) ──┐ │     │
    │                                                    │ │     │
    │                                                    v v     v
    │                                        ┌─── Costmap Generation ───┐
    │                                        │ Depth + Mask → BEV Grid  │
    │                                        │ 200x120 cells, 5cm/cell  │
    │                                        └──────────┬───────────────┘
    │                                                   │
    │                                                   v
BNO085 IMU (100Hz) ──┐                    ┌─── MPPI Controller (CUDA) ───┐
                      │                    │ 768 rollouts x 100 timesteps  │
Wheel Encoders ───────┼──► EKF ──state──► │ Sample trajectories on GPU    │
                      │         vector    │ Evaluate costs on costmap     │
VIO Pose (30Hz) ──────┘                   │ Weight by cost → optimal ctrl │
                                          └──────────┬──────────────────┘
                                                     │
                                                     v
                                          ┌─── Arduino Uno ────┐
                                          │ Steering PWM       │
                                          │ Throttle PWM       │
                                          │ Run-Stop Relay     │
                                          └────────────────────┘
```

## Shared Bus (`shared_bus_t`)

All inter-thread communication flows through a single shared struct using seqlocks.

| Field | Writer | Reader(s) | Rate | Mechanism |
|-------|--------|-----------|------|-----------|
| `state` | Thread 2 (State Est.) | Thread 1 (Planning) | 100 Hz | seqlock |
| `control` | Thread 1 (Planning) | Thread 3 (Actuation) | 50 Hz | seqlock |
| `imu` | Thread 3 (Actuation) | Thread 2 (State Est.) | 100 Hz | seqlock |
| `wheels` | Thread 3 (Actuation) | Thread 2 (State Est.) | 100 Hz | seqlock |
| `vio` | Thread 0 (Perception) | Thread 2 (State Est.) | 30 Hz | seqlock |
| `costmap_ptr` | Thread 0 (Perception) | Thread 1 (Planning) | 30 Hz | atomic ptr swap (double-buffer) |

### Seqlock Protocol

Single-writer, multi-reader lock-free synchronization:
- Writer increments sequence counter (odd = write in progress)
- Reader checks sequence before and after read; retries if mismatch or odd
- All seqlock fields are `alignas(64)` to prevent false sharing

### Costmap Double-Buffering

The costmap (200x120 floats = 96KB) is too large for seqlock. Instead:
- Two GPU buffers allocated at startup
- Perception writes to buffer A, MPPI reads buffer B
- After write completes, atomic pointer swap + timestamp update
- MPPI checks timestamp to detect new costmap, rebinds CUDA texture

## Thread Scheduling

| Thread | CPU Core | Priority | Scheduler |
|--------|----------|----------|-----------|
| T0 Perception | Core 3 | Default | SCHED_OTHER |
| T1 Planning | Core 0 | 90 | SCHED_FIFO |
| T2 State Est. | Core 1 | Default | SCHED_OTHER |
| T3 Actuation | Core 2 | 80 | SCHED_FIFO |

Planning (T1) has highest priority to minimize control jitter.

## Startup Sequence

1. Parse args, load YAML configs
2. Allocate `shared_bus_t` (aligned)
3. Initialize CUDA device
4. Start T3 (Actuation) — establish Arduino serial connection
5. Wait for first IMU packet (confirms Arduino alive)
6. Start T2 (State Estimation)
7. Start T0 (Perception) — opens ZED camera
8. Wait for first VIO pose (confirms ZED alive)
9. Start T1 (Planning) — begins MPPI control

## Shutdown Sequence

1. `SIGINT` handler sets `bus->alive = false`
2. Write neutral control to bus (steering=0, throttle=0)
3. Join T1 (Planning)
4. Join T0 (Perception)
5. Join T2 (State Estimation)
6. Join T3 (Actuation) — sends final neutral command
7. Cleanup CUDA resources

## Why Not ROS

| Concern | ROS | TruggyAD |
|---------|-----|----------|
| Latency | ~1ms per topic hop | ~0 (shared memory) |
| Memory | 200+ MB overhead | Single binary |
| Jetson 2GB | Barely fits | Comfortable |
| Dependencies | catkin, boost, etc. | Minimal |
| Complexity | Launch files, nodelets | One main.cpp |

## Why C++ (not pure C)

| Library | C API Available | Notes |
|---------|----------------|-------|
| CUDA Runtime | Partial | `__global__` requires C++ compiler |
| ZED SDK | No (C++ only) | `sl::Camera`, `sl::Mat` |
| TensorRT | No (C++ only) | `nvinfer1::IRuntime` |
| Eigen3 | No (templates) | EKF matrix operations |

**Coding style:** C-like C++ — structs over classes, no exceptions/RTTI, minimal STL, `snake_case`, manual memory management. Arduino firmware is pure C.
