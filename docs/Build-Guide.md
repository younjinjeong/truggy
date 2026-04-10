# Build Guide

## Prerequisites

### Jetson Nano 2GB (Native Build)

```bash
# JetPack 4.6 base install (provides CUDA 10.2, TensorRT 8.x, cuDNN)

# Additional packages
sudo apt update
sudo apt install -y \
  cmake \
  build-essential \
  libeigen3-dev \
  libyaml-cpp-dev \
  libopencv-dev \
  zlib1g-dev \
  python3-pip \
  clangd-12

# ZED SDK (download from Stereolabs for JetPack 4.6 / L4T 32.6)
# https://www.stereolabs.com/developers/release
# Run installer: ./ZED_SDK_Tegra_L4T32.6_v4.x.run

# Verify
nvcc --version          # CUDA 10.2
dpkg -l | grep tensorrt # TensorRT 8.x
python3 -c "import cv2; print(cv2.__version__)"
```

### WSL2 Cross-Compilation (RTX 4080)

```bash
# Install aarch64 cross-compiler
sudo apt install -y gcc-aarch64-linux-gnu g++-aarch64-linux-gnu

# CUDA cross-compilation toolkit
# Download CUDA toolkit for aarch64 from NVIDIA
# https://developer.nvidia.com/cuda-toolkit-archive

# Eigen3, yaml-cpp, zlib headers (for cross-compile)
sudo apt install -y libeigen3-dev libyaml-cpp-dev zlib1g-dev

# ZED SDK headers only (copy from Jetson or download)
# TensorRT headers only (copy from Jetson)
```

## Clone and Build

### Native Build (Jetson)

```bash
git clone https://github.com/younjinjeong/truggy.git
cd truggy

# Nano 2GB build
cmake -B build \
  -DTARGET_NANO_2GB=ON \
  -DCMAKE_EXPORT_COMPILE_COMMANDS=ON
cmake --build build -j4

# Run
./build/truggy --config config/truggy.yaml
```

### Orin Nano Build

```bash
cmake -B build \
  -DTARGET_ORIN_NANO=ON \
  -DCMAKE_EXPORT_COMPILE_COMMANDS=ON
cmake --build build -j6
```

### Cross-Compilation (WSL2 → Jetson)

```bash
cmake -B build-cross \
  -DCMAKE_TOOLCHAIN_FILE=cmake/jetson-nano-toolchain.cmake \
  -DTARGET_NANO_2GB=ON \
  -DCMAKE_EXPORT_COMPILE_COMMANDS=ON
cmake --build build-cross -j$(nproc)

# Deploy to Jetson
scp build-cross/truggy jetson@192.168.1.100:~/truggy/
```

## CMake Options

| Option | Default | Description |
|--------|---------|-------------|
| `TARGET_NANO_2GB` | OFF | Build for Jetson Nano 2GB (sm_53, 768 rollouts) |
| `TARGET_ORIN_NANO` | OFF | Build for Orin Nano Super (sm_87, 4096 rollouts) |
| `USE_GAZEBO_SIM` | OFF | Include Gazebo Harmonic simulation bridge |
| `BUILD_TOOLS` | ON | Build test/benchmark tools |
| `CMAKE_EXPORT_COMPILE_COMMANDS` | — | Generate compile_commands.json for LSP |

## Build Targets

| Target | Description |
|--------|-------------|
| `truggy` | Main autonomous driving binary |
| `serial_test` | Arduino serial communication test |
| `mppi_bench` | MPPI benchmark with synthetic costmap |
| `costmap_test` | Offline costmap generation test |

## TensorRT Engine Build

The TensorRT engine must be built **on the target device** (engine files are not portable between GPU architectures).

```bash
# On Jetson Nano (after training segmentation model)
python3 scripts/build_engine.py \
  --onnx models/track_seg.onnx \
  --output models/track_seg_engine.trt \
  --fp32                          # Nano: FP32 only

# On Orin Nano
python3 scripts/build_engine.py \
  --onnx models/track_seg.onnx \
  --output models/track_seg_engine.trt \
  --fp16                          # Orin: FP16 for speed
```

## Performance Tuning (Jetson)

```bash
# Required before running for consistent performance
sudo nvpmodel -m 0        # MAXN mode
sudo jetson_clocks         # Lock clocks at maximum

# Monitor resource usage
tegrastats                 # GPU/CPU/MEM utilization

# CUDA profiling
nsys profile ./build/truggy --config config/truggy.yaml
```

## Arduino Firmware Build

```bash
# Using Arduino CLI
arduino-cli compile --fqbn arduino:avr:uno firmware/truggy_bridge/
arduino-cli upload -p /dev/ttyACM0 --fqbn arduino:avr:uno firmware/truggy_bridge/

# Or use the helper script
./scripts/flash_firmware.sh
```

## Troubleshooting

### CUDA out of memory on Nano 2GB
- Check `tegrastats` for memory usage
- Ensure ZED depth mode is `PERFORMANCE` (not ULTRA/QUALITY)
- Reduce MPPI rollouts if needed (edit config/truggy.yaml)
- Kill unnecessary processes: `sudo systemctl stop gdm` (if desktop running)

### clangd not finding CUDA headers
- Ensure `compile_commands.json` is generated in `build/`
- Check `.clangd` points to correct `CompilationDatabase: build`
- May need to add CUDA include path manually: `-I/usr/local/cuda/include`

### Serial port permission denied
```bash
sudo usermod -aG dialout $USER
# Log out and back in
```

### ZED SDK not found by CMake
- Default install path: `/usr/local/zed`
- Set `ZED_DIR=/usr/local/zed` if CMake can't find it
