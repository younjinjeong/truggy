# CMake toolchain file for cross-compiling to Jetson Nano (aarch64)
# Usage: cmake -DCMAKE_TOOLCHAIN_FILE=cmake/jetson-nano-toolchain.cmake ..
#
# Prerequisites on the host (WSL2 / x86_64 Linux):
#   sudo apt install gcc-aarch64-linux-gnu g++-aarch64-linux-gnu
#   CUDA toolkit for aarch64 cross-compilation
#
# You also need the Jetson sysroot with target libraries:
#   - ZED SDK headers/libs (from Jetson or SDK installer)
#   - TensorRT headers/libs (from JetPack)
#   - Eigen3, yaml-cpp, zlib headers
#
# Set JETSON_SYSROOT to the path containing these (default: /opt/jetson-sysroot)

set(CMAKE_SYSTEM_NAME Linux)
set(CMAKE_SYSTEM_PROCESSOR aarch64)

# Cross-compiler
set(CMAKE_C_COMPILER aarch64-linux-gnu-gcc)
set(CMAKE_CXX_COMPILER aarch64-linux-gnu-g++)

# Sysroot with Jetson target libraries
set(JETSON_SYSROOT "/opt/jetson-sysroot" CACHE PATH "Jetson Nano sysroot with target libs")

set(CMAKE_FIND_ROOT_PATH ${JETSON_SYSROOT})
set(CMAKE_FIND_ROOT_PATH_MODE_PROGRAM NEVER)
set(CMAKE_FIND_ROOT_PATH_MODE_LIBRARY ONLY)
set(CMAKE_FIND_ROOT_PATH_MODE_INCLUDE ONLY)
set(CMAKE_FIND_ROOT_PATH_MODE_PACKAGE ONLY)

# CUDA cross-compilation
# nvcc can compile for sm_53 on any host GPU
# The host compiler is the cross-compiler
set(CMAKE_CUDA_HOST_COMPILER ${CMAKE_CXX_COMPILER})
set(CMAKE_CUDA_ARCHITECTURES "53")

# CUDA toolkit path (host-side nvcc that targets aarch64)
# If using NVIDIA's cross-compilation CUDA toolkit:
set(CMAKE_CUDA_COMPILER "/usr/local/cuda/bin/nvcc" CACHE FILEPATH "nvcc path")

# Target libraries (adjust paths based on your sysroot layout)
set(ZED_DIR "${JETSON_SYSROOT}/usr/local/zed" CACHE PATH "ZED SDK path in sysroot")
set(TENSORRT_DIR "${JETSON_SYSROOT}/usr/lib/aarch64-linux-gnu" CACHE PATH "TensorRT path in sysroot")
