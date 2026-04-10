# TruggyAD

Vision-only autonomous driving platform on a 1/8 scale RC Truggy, porting Georgia Tech AutoRally's MPPI controller.

## Quick Links

| Page | Description |
|------|-------------|
| [Architecture](Architecture.md) | 4-thread design, shared bus, data flow |
| [Hardware](Hardware.md) | Jetson Nano 2GB, Arduino Uno, ZED camera, sensors |
| [Serial Protocol](Serial-Protocol.md) | Binary protocol between Jetson and Arduino |
| [MPPI Reference](MPPI-Reference.md) | MPPI algorithm, CUDA kernels, parameters |
| [Vehicle Dynamics](Vehicle-Dynamics.md) | 1/8 truggy physical model and dynamics |
| [Build Guide](Build-Guide.md) | Native build, cross-compilation, dependencies |

## Project Summary

- **Goal:** Autonomous track driving without GPS or LiDAR
- **Sensors:** ZED stereo camera (depth + VIO) + BNO085 IMU + wheel encoders
- **Controller:** MPPI (Model Predictive Path Integral) on CUDA
- **Hardware:** Jetson Nano 2GB (initial), Orin Nano Super (upgrade)
- **Software:** Single C++ binary, 4 threads, no ROS

## Repository Structure

```
truggy/
├── docs/              # This documentation
├── config/            # YAML configuration files
├── src/               # C++/CUDA source code
│   ├── common/        # Shared types, config, serial, timing
│   ├── perception/    # ZED capture, segmentation, costmap
│   ├── planning/      # MPPI controller (CUDA)
│   ├── state/         # EKF sensor fusion
│   └── actuation/     # Arduino serial bridge
├── firmware/          # Arduino Uno firmware
├── models/            # NN models (ONNX, TensorRT)
├── scripts/           # Python training/tooling scripts
├── tools/             # C++ test/benchmark utilities
├── sim/               # Gazebo Harmonic simulation
└── third_party/       # cnpy library
```

## GitHub Issue Tracking

| Epic | Issue | Focus |
|------|-------|-------|
| Epic 0 | #1 | Developer environment & skills |
| Epic 1 | #6 | Foundation (build system, shared types) |
| Epic 2 | #14 | Arduino firmware & actuation |
| Epic 3 | #18 | Perception pipeline |
| Epic 4 | #24 | MPPI controller port |
| Epic 5 | #30 | State estimation (EKF) |
| Epic 6 | #33 | Integration & testing |
| Epic 7 | #39 | Orin Nano upgrade |
