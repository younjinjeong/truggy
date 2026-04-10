# TruggyAD

**Vision-only autonomous driving on a 1/8 scale RC Truggy using MPPI path integral control on NVIDIA Jetson.**

No GPS. No LiDAR. Just a stereo camera, an IMU, and 768 parallel trajectory rollouts on a 128-core GPU.

Ported from [Georgia Tech AutoRally](https://github.com/AutoRally/autorally), stripped of ROS, rebuilt as a single C++ binary with 4 real-time threads.

## Architecture

```
┌──────────────────────────────────────────────────────────────┐
│                     truggy (single binary)                    │
│                                                               │
│  ┌───────────────┐  ┌───────────────┐  ┌───────────────┐     │
│  │ Thread 0      │  │ Thread 1      │  │ Thread 2      │     │
│  │ Perception    │  │ Planning      │  │ State Est.    │     │
│  │ ZED 720p/30Hz │  │ MPPI (CUDA)   │  │ EKF 100Hz    │     │
│  │ Segmentation  │  │ 768 rollouts  │  │ IMU+VIO+Enc  │     │
│  │ BEV Costmap   │  │ 50 Hz         │  │              │     │
│  └──────┬────────┘  └──────┬────────┘  └──────┬────────┘     │
│         │                  │                   │              │
│         └───── shared_bus_t (seqlock) ─────────┘              │
│                            │                                  │
│                   ┌────────┴─────────┐                        │
│                   │ Thread 3         │                        │
│                   │ Actuation        │                        │
│                   │ Arduino 115200   │                        │
│                   └──────────────────┘                        │
└──────────────────────────────────────────────────────────────┘
```

## Quick Start

```bash
git clone https://github.com/younjinjeong/truggy.git
cd truggy
cmake -B build -DTARGET_NANO_2GB=ON -DCMAKE_EXPORT_COMPILE_COMMANDS=ON
cmake --build build -j4
./build/truggy --sim --log data/test.bin
```

## Prerequisites

| Dependency | Version | Required | Notes |
|------------|---------|----------|-------|
| CUDA Toolkit | 10.2 (Nano) / 12.x (Orin) | Yes | GPU kernels |
| CMake | 3.18+ | Yes | Build system |
| Eigen3 | 3.3+ | Yes | EKF matrix ops |
| yaml-cpp | 0.6+ | Yes | Config loading |
| zlib | 1.2+ | Yes | cnpy dependency |
| ZED SDK | 4.x / 5.x | Optional | Camera + VIO |
| TensorRT | 8.x / 10.x | Optional | Segmentation NN |
| Gazebo Harmonic | Latest | Optional | Simulation |

```bash
# Ubuntu (Jetson or x86_64)
sudo apt install cmake build-essential libeigen3-dev libyaml-cpp-dev zlib1g-dev
```

## Build

### Jetson Nano 2GB (native)

```bash
cmake -B build -DTARGET_NANO_2GB=ON -DCMAKE_EXPORT_COMPILE_COMMANDS=ON
cmake --build build -j4
```

### Remote build (WSL2 + RTX 4080)

```bash
./scripts/remote-build.sh              # compile for Nano sm_53
./scripts/remote-build.sh --native     # compile for local GPU
./scripts/remote-build.sh --orin       # compile for Orin sm_87
./scripts/remote-build.sh --bench      # build + run MPPI benchmark
```

### CMake Options

| Option | Default | Description |
|--------|---------|-------------|
| `TARGET_NANO_2GB` | OFF | Jetson Nano 2GB (sm_53, 768 rollouts) |
| `TARGET_ORIN_NANO` | OFF | Orin Nano Super (sm_87, 4096 rollouts) |
| `BUILD_WITHOUT_HARDWARE_DEPS` | OFF | Skip ZED SDK + TensorRT (CI/testing) |
| `BUILD_TOOLS` | ON | Build serial_test and mppi_bench |

## Run

### Hardware mode (with MCU + ZED camera)

```bash
# Flash firmware — choose your board:
./scripts/flash_firmware.sh                  # Arduino Uno
./scripts/flash_firmware.sh --esp32          # M5StickC Plus 2

# Set Jetson to max performance
sudo nvpmodel -m 0 && sudo jetson_clocks

# Run
./build/truggy --config config/truggy.yaml --costs config/mppi_costs.yaml
```

### Simulation mode (no hardware needed)

```bash
./build/truggy --sim
./build/truggy --sim --log data/test.bin   # with telemetry recording
```

### View telemetry

```bash
python3 tools/viewer.py data/test.bin
```

## Gazebo Simulation

### Install Gazebo Harmonic

```bash
sudo apt install gz-harmonic
```

### Run the simulation

```bash
# Terminal 1: Launch the track world
export GZ_SIM_RESOURCE_PATH=$(pwd)/sim/gazebo
gz sim sim/gazebo/track.sdf

# Terminal 2: Send manual steering commands (test)
gz topic -t /cmd_vel -m gz.msgs.Twist \
  -p 'linear: {x: 2.0}, angular: {z: 0.3}'

# Terminal 3: Run TruggyAD in sim mode
./build/truggy --sim --log data/gazebo_test.bin
```

### What the simulation provides

- **Vehicle model** (`truggy.sdf`): 1/8 scale, 4 wheels, Ackermann steering plugin
- **Sensors**: stereo camera (720p/30Hz), depth camera, IMU (100Hz with noise)
- **Track** (`track.sdf`): 20m x 10m oval with inner/outer walls
- **Control**: `/cmd_vel` topic (Twist messages)

> **Note**: The gz-transport bridge connecting Gazebo sensor outputs to the shared_bus_t is not yet implemented. Currently `--sim` runs Thread 0 (Perception) in stub mode. For full closed-loop sim, the bridge needs to pipe Gazebo camera/IMU data into the perception and state estimation threads.

## MPPI Benchmark

```bash
./build/mppi_bench 100
```

Example output (RTX 4080 SUPER):
```
Rollouts: 768, Timesteps: 100, Block: 8x4
Avg: 1.74 ms (576 Hz)
State evals/s: 44.3M
GPU memory: 603.8 KB
```

## Circuit Connection Diagram

### Option A: Arduino Uno

```
                        ┌─────────────────────────────────┐
                        │       ZED Stereo Camera          │
                        │  ┌───┐   120mm    ┌───┐         │
                        │  │ L │            │ R │         │
                        │  └───┘            └───┘         │
                        └──────────┬──────────────────────┘
                                   │ USB 3.0
┌──────────────────────────────────┼──────────────────────────────────┐
│  JETSON NANO 2GB                 │                                  │
│  USB 3.0 ←───────────────────────┘                                  │
│  USB 2.0 ←──────────────────────────────────────┐                   │
└─────────────────────────────────────────────────┼───────────────────┘
                                                  │ USB Serial
┌─────────────────────────────────────────────────┼───────────────────┐
│  ARDUINO UNO (ATmega328P, 16 MHz)               │                   │
│  ┌──────────┐    I2C (A4=SDA, A5=SCL)    ┌──────┴──────┐           │
│  │ BNO085   │◄──────────────────────────►│ USB Serial  │           │
│  │ IMU      │    400 kHz, addr 0x4A      │ 115200 baud │           │
│  └──────────┘                            └─────────────┘           │
│  D2 (INT0) ◄──── Left Wheel Encoder                                │
│  D3 (INT1) ◄──── Right Wheel Encoder                               │
│  D9  (PWM) ────► Steering Servo (1000-2000us)                      │
│  D10 (PWM) ────► ESC/Throttle  (1000-2000us)                      │
│  D7  (OUT) ────► Run-Stop Relay (HIGH=STOP)                        │
└─────────────────────────────────────────────────────────────────────┘
```

### Option B: M5StickC Plus 2 (ESP32)

```
┌──────────────────────────────────────────────────────────────────────┐
│  JETSON NANO 2GB                                                     │
│  USB 3.0 ←──── ZED Stereo Camera                                    │
│  USB 2.0 ←──────────────────────────────────────┐                    │
└─────────────────────────────────────────────────┼────────────────────┘
                                                  │ USB-C Serial
┌─────────────────────────────────────────────────┼────────────────────┐
│  M5StickC Plus 2 (ESP32, 240 MHz)               │                    │
│  ┌──────────┐  ┌─────────┐               ┌─────┴──────┐            │
│  │ MPU6886  │  │ 1.14"   │               │ USB-C      │            │
│  │ (built-in│  │ TFT     │               │ 115200 baud│            │
│  │ 6-axis)  │  │ Display │               └────────────┘            │
│  └──────────┘  └─────────┘                                          │
│  Grove: G32/G33 (I2C) ◄──► BNO085 IMU (addr 0x4A)                  │
│  HAT:   G26  ◄──── Left Wheel Encoder (interrupt)                   │
│         G36  ◄──── Right Wheel Encoder (input-only interrupt)       │
│         G0   ────► Steering Servo PWM                               │
│         G25  ────► ESC/Throttle PWM                                 │
└─────────────────────────────────────────────────────────────────────┘
```

```
Power Distribution (both options):
  LiPo Battery (2S-3S, 7.4-11.1V)
    ├──► ESC ──► Brushless Motor
    ├──► BEC (5V regulated) ──► MCU + Servo + Encoders
    └──► Jetson Nano (via barrel jack or USB-C PD)
```

See [docs/Circuit-Diagram.md](docs/Circuit-Diagram.md) for Mermaid diagram and SchemeIt component list.

## Project Structure

```
truggy/
├── src/
│   ├── main.cpp              # Entry point, 4 threads, signal handling
│   ├── common/               # types.h, config, serial, timing, logger, ring_buffer
│   ├── perception/           # ZED capture, BEV costmap (CUDA), segmentation
│   ├── planning/             # MPPI controller, dynamics NN, cost functions (CUDA)
│   ├── state/                # 7-state EKF, IMU utilities
│   └── actuation/            # Arduino serial bridge
├── firmware/
│   ├── common/protocol.h    # Shared binary protocol (CRC-8, packet format)
│   ├── truggy_bridge/       # Arduino Uno firmware
│   └── truggy_bridge_esp32/ # M5StickC Plus 2 firmware
├── config/                   # truggy.yaml, mppi_costs.yaml
├── sim/gazebo/               # Gazebo Harmonic SDF models
├── scripts/                  # run.sh, remote-build.sh, flash_firmware.sh
├── tools/                    # serial_test, mppi_bench, viewer.py
├── docs/                     # Architecture, Hardware, MPPI, Protocol, Build Guide
├── cmake/                    # Cross-compilation toolchain
├── deploy/clangd/            # LSP Docker/k8s deployment
└── third_party/cnpy/         # .npz file I/O (MIT license)
```

## Configuration

Key parameters in `config/truggy.yaml`:

| Parameter | Value | Description |
|-----------|-------|-------------|
| `mppi.num_timesteps` | 100 | Prediction horizon (2.0s at 50Hz) |
| `mppi.gamma` | 0.15 | Softmax temperature |
| `mppi.steering_std` | 0.275 | Exploration noise |
| `costs.desired_speed` | 4.0 m/s | Target speed |
| `costs.track_coeff` | 200.0 | Track boundary penalty |
| `costs.crash_coeff` | 10000.0 | Rollover penalty |
| `costmap.cell_size` | 0.05 m | BEV grid resolution |
| `vehicle.wheelbase` | 0.35 m | 1/8 scale truggy |

## Firmware

Two MCU firmware options sharing the same binary serial protocol (`firmware/common/protocol.h`). The Jetson-side bridge works unchanged with either board.

| Board | Firmware | Pros | Cons |
|-------|----------|------|------|
| **Arduino Uno** | `firmware/truggy_bridge/` | 5V logic, simple, proven | 2KB RAM, no display |
| **M5StickC Plus 2** | `firmware/truggy_bridge_esp32/` | 240MHz, 520KB RAM, TFT display, WiFi, built-in MPU6886 | 3.3V logic, limited GPIO (6 pins) |

```bash
./scripts/flash_firmware.sh              # Arduino Uno (default)
./scripts/flash_firmware.sh --esp32      # M5StickC Plus 2
```

### ESP32 Pin Mapping (M5StickC Plus 2)

| Pin | Function | Connector |
|-----|----------|-----------|
| G32 | BNO085 I2C SDA | Grove |
| G33 | BNO085 I2C SCL | Grove |
| G26 | Wheel Encoder L (INT) | HAT |
| G36 | Wheel Encoder R (input-only INT) | HAT |
| G0 | Steering Servo PWM | HAT |
| G25 | ESC Throttle PWM | HAT |

## Hardware BOM

| Component | Model | Interface | Purpose |
|-----------|-------|-----------|---------|
| Compute | Jetson Nano 2GB | — | Main computer (128 CUDA cores) |
| Camera | ZED Stereo | USB 3.0 | Depth + VIO + RGB |
| IMU | BNO085 | I2C (0x4A) | Quaternion + accel + gyro |
| MCU (option A) | Arduino Uno | USB Serial | Sensor hub + PWM output |
| MCU (option B) | M5StickC Plus 2 | USB Serial | Sensor hub + PWM + TFT display |
| Encoders | Optical x2 | Digital INT | Wheel speed measurement |
| Servo | RC Steering | PWM | Steering control |
| ESC | Brushless | PWM | Throttle control |
| Battery | LiPo 2S-3S | — | Power source |
| Chassis | TLR 1/8 Truggy | — | Vehicle platform |

## Performance Targets

| Metric | Nano 2GB | Orin Nano |
|--------|----------|-----------|
| MPPI compute | < 20 ms | < 10 ms |
| Control rate | 50 Hz | 110 Hz |
| Sensor-to-actuator | < 50 ms | < 15 ms |
| Max safe speed | ~8 m/s | ~20 m/s |
| GPU memory | ~753 MB | ~1.5 GB |

## Documentation

| Page | Description |
|------|-------------|
| [Architecture](docs/Architecture.md) | 4-thread design, shared bus, data flow |
| [Hardware](docs/Hardware.md) | Pin assignments, memory budget, sensors |
| [Serial Protocol](docs/Serial-Protocol.md) | Binary protocol (8B cmd, 24B telemetry) |
| [MPPI Reference](docs/MPPI-Reference.md) | Algorithm, CUDA kernels, parameters |
| [Vehicle Dynamics](docs/Vehicle-Dynamics.md) | Physical model, tire behavior |
| [Build Guide](docs/Build-Guide.md) | Build, cross-compile, troubleshooting |
| [Circuit Diagram](docs/Circuit-Diagram.md) | Wiring diagram (Mermaid + SchemeIt) |

## Contributing

See [CONTRIBUTING.md](CONTRIBUTING.md) for development workflow, coding conventions, and branch naming.

**Workflow:** GitHub Issue (Story) -> feature branch (`<issue-id>/<name>`) -> build & test -> Gazebo test -> PR -> merge

## License

[Apache 2.0](LICENSE)
