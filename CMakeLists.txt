cmake_minimum_required(VERSION 3.18)
project(truggy LANGUAGES CXX CUDA)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CUDA_STANDARD 17)
set(CMAKE_CUDA_STANDARD_REQUIRED ON)

# ── Platform targets ─────────────────────────────────────────────────────────
option(TARGET_NANO_2GB  "Build for Jetson Nano 2GB (sm_53, 768 rollouts)"  OFF)
option(TARGET_ORIN_NANO "Build for Orin Nano Super (sm_87, 4096 rollouts)" OFF)
option(BUILD_WITHOUT_HARDWARE_DEPS "Skip ZED SDK and TensorRT (CI/testing)" OFF)
option(BUILD_TOOLS "Build test and benchmark tools" ON)

if(TARGET_ORIN_NANO)
    set(CMAKE_CUDA_ARCHITECTURES 87)
    add_compile_definitions(TARGET_ORIN_NANO)
    add_compile_definitions(MPPI_NUM_ROLLOUTS=4096)
    add_compile_definitions(MPPI_BLOCKSIZE_X=16)
    add_compile_definitions(MPPI_BLOCKSIZE_Y=8)
    message(STATUS "Target: Orin Nano Super (sm_87, 4096 rollouts)")
elseif(TARGET_NANO_2GB)
    set(CMAKE_CUDA_ARCHITECTURES 53)
    add_compile_definitions(TARGET_NANO_2GB)
    add_compile_definitions(MPPI_NUM_ROLLOUTS=768)
    add_compile_definitions(MPPI_BLOCKSIZE_X=8)
    add_compile_definitions(MPPI_BLOCKSIZE_Y=4)
    message(STATUS "Target: Jetson Nano 2GB (sm_53, 768 rollouts)")
else()
    # Default: detect local GPU or use sm_53
    if(NOT DEFINED CMAKE_CUDA_ARCHITECTURES)
        set(CMAKE_CUDA_ARCHITECTURES 53)
    endif()
    add_compile_definitions(MPPI_NUM_ROLLOUTS=768)
    add_compile_definitions(MPPI_BLOCKSIZE_X=8)
    add_compile_definitions(MPPI_BLOCKSIZE_Y=4)
    message(STATUS "Target: default (sm_${CMAKE_CUDA_ARCHITECTURES})")
endif()

# ── CUDA flags ───────────────────────────────────────────────────────────────
set(CMAKE_CUDA_FLAGS "${CMAKE_CUDA_FLAGS} --maxrregcount=32 -use_fast_math")
set(CMAKE_CUDA_SEPARABLE_COMPILATION ON)  # needed for cross-file __device__ calls
add_compile_definitions(MPPI_NNET_USING_CONSTANT_MEM__)

# ── Compiler flags ───────────────────────────────────────────────────────────
add_compile_options(-Wall -Wextra -Wno-unused-parameter -O2)
# No RTTI for project code. cnpy needs it (uses typeid).
# yaml-cpp throws internally, so we keep exceptions enabled globally.
set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -fno-rtti")

# ── Dependencies ─────────────────────────────────────────────────────────────

# CUDA (already found by enabling CUDA language)
find_package(CUDAToolkit REQUIRED)

# Eigen3
find_package(Eigen3 3.3 REQUIRED NO_MODULE)

# yaml-cpp
find_package(yaml-cpp REQUIRED)

# zlib (for cnpy)
find_package(ZLIB REQUIRED)

# Threads
find_package(Threads REQUIRED)

# ZED SDK (optional — skip for CI builds)
if(NOT BUILD_WITHOUT_HARDWARE_DEPS)
    # ZED SDK installs to /usr/local/zed
    list(APPEND CMAKE_PREFIX_PATH "/usr/local/zed")
    find_package(ZED 4 QUIET)
    if(ZED_FOUND)
        message(STATUS "ZED SDK found: ${ZED_INCLUDE_DIRS}")
        add_compile_definitions(HAS_ZED_SDK)
    else()
        message(STATUS "ZED SDK not found — perception module will use stubs")
    endif()

    # TensorRT
    find_library(NVINFER_LIB nvinfer)
    find_library(NVONNXPARSER_LIB nvonnxparser)
    if(NVINFER_LIB)
        message(STATUS "TensorRT found: ${NVINFER_LIB}")
        add_compile_definitions(HAS_TENSORRT)
    else()
        message(STATUS "TensorRT not found — segmentation module will use stubs")
    endif()
else()
    message(STATUS "BUILD_WITHOUT_HARDWARE_DEPS=ON — skipping ZED SDK and TensorRT")
endif()

# ── Third-party: cnpy ────────────────────────────────────────────────────────
add_library(cnpy STATIC third_party/cnpy/cnpy.cpp)
target_include_directories(cnpy PUBLIC third_party/cnpy)
target_link_libraries(cnpy PUBLIC ZLIB::ZLIB)
# cnpy uses typeid and throw — override project-wide -fno-rtti/-fno-exceptions
target_compile_options(cnpy PRIVATE -frtti -fexceptions)

# ── Main target ──────────────────────────────────────────────────────────────
add_executable(truggy
    src/main.cpp
    src/common/config.cpp
    src/common/serial.cpp
    src/state/ekf.cpp
    src/actuation/bridge.cpp
)

# CUDA sources (added separately for proper .cu compilation)
target_sources(truggy PRIVATE
    src/perception/costmap.cu
    src/planning/mppi.cu
    src/planning/dynamics.cu
    src/planning/costs.cu
)

target_include_directories(truggy PRIVATE
    ${CMAKE_CURRENT_SOURCE_DIR}/src
)

target_link_libraries(truggy PRIVATE
    Eigen3::Eigen
    yaml-cpp::yaml-cpp
    CUDA::curand
    CUDA::cudart
    Threads::Threads
    cnpy
)

# Link ZED SDK if available
if(ZED_FOUND)
    target_include_directories(truggy PRIVATE ${ZED_INCLUDE_DIRS})
    target_link_libraries(truggy PRIVATE ${ZED_LIBRARIES})
endif()

# Link TensorRT if available
if(NVINFER_LIB)
    target_link_libraries(truggy PRIVATE ${NVINFER_LIB} ${NVONNXPARSER_LIB})
endif()

# ── Tools ────────────────────────────────────────────────────────────────────
if(BUILD_TOOLS)
    add_executable(serial_test tools/serial_test.cpp src/common/serial.cpp)
    target_include_directories(serial_test PRIVATE src)
    target_link_libraries(serial_test PRIVATE Threads::Threads)

    add_executable(mppi_bench
        tools/mppi_bench.cu
        src/planning/mppi.cu
        src/planning/dynamics.cu
        src/planning/costs.cu
    )
    target_include_directories(mppi_bench PRIVATE src)
    target_link_libraries(mppi_bench PRIVATE
        CUDA::curand
        CUDA::cudart
        cnpy
    )
endif()

# ── Export compile_commands.json for clangd ───────────────────────────────────
set(CMAKE_EXPORT_COMPILE_COMMANDS ON)
