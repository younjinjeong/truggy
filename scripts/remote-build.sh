#!/bin/bash
# Remote CUDA build on WSL2 machine with RTX 4080 SUPER
#
# Usage:
#   ./scripts/remote-build.sh              # sync + build (default: Nano 2GB target)
#   ./scripts/remote-build.sh --orin       # build for Orin Nano Super
#   ./scripts/remote-build.sh --native     # build for local x86_64 (testing on RTX 4080)
#   ./scripts/remote-build.sh --clean      # clean build directory first
#   ./scripts/remote-build.sh --bench      # build + run MPPI benchmark
#
# Prerequisites:
#   - SSH key auth set up for younj@192.168.50.228
#   - Repo cloned at ~/truggy on remote

set -euo pipefail

REMOTE_HOST="younj@192.168.50.228"
REMOTE_DIR="~/truggy"
CUDA_PATH="/usr/local/cuda/bin"

# Parse args
TARGET="NANO_2GB"
CLEAN=false
BENCH=false
for arg in "$@"; do
    case "$arg" in
        --orin)   TARGET="ORIN_NANO" ;;
        --native) TARGET="NATIVE" ;;
        --clean)  CLEAN=true ;;
        --bench)  BENCH=true ;;
        --help|-h)
            head -12 "$0" | tail -10
            exit 0
            ;;
    esac
done

echo "=== Remote CUDA Build ==="
echo "Host: ${REMOTE_HOST}"
echo "Target: ${TARGET}"
echo ""

# Step 1: Sync local changes to remote
echo "[1/4] Syncing source to remote..."
rsync -az --delete \
    --exclude='build*/' \
    --exclude='.git/' \
    --exclude='models/*.engine' \
    --exclude='models/*.trt' \
    -e ssh \
    "$(git rev-parse --show-toplevel)/" \
    "${REMOTE_HOST}:${REMOTE_DIR}/"

# Step 2: Configure cmake
echo "[2/4] Configuring cmake..."
CMAKE_ARGS="-DCMAKE_EXPORT_COMPILE_COMMANDS=ON"

case "$TARGET" in
    NANO_2GB)
        CMAKE_ARGS="${CMAKE_ARGS} -DTARGET_NANO_2GB=ON"
        # Cross-compile for sm_53 on x86_64 machine
        CMAKE_ARGS="${CMAKE_ARGS} -DCMAKE_CUDA_ARCHITECTURES=53"
        ;;
    ORIN_NANO)
        CMAKE_ARGS="${CMAKE_ARGS} -DTARGET_ORIN_NANO=ON"
        CMAKE_ARGS="${CMAKE_ARGS} -DCMAKE_CUDA_ARCHITECTURES=87"
        ;;
    NATIVE)
        # Build for local GPU (RTX 4080 = sm_89) for testing
        CMAKE_ARGS="${CMAKE_ARGS} -DCMAKE_CUDA_ARCHITECTURES=89"
        CMAKE_ARGS="${CMAKE_ARGS} -DBUILD_WITHOUT_HARDWARE_DEPS=ON"
        ;;
esac

CLEAN_CMD=""
if [ "$CLEAN" = true ]; then
    CLEAN_CMD="rm -rf build &&"
fi

ssh "${REMOTE_HOST}" "export PATH=${CUDA_PATH}:\$PATH && cd ${REMOTE_DIR} && ${CLEAN_CMD} cmake -B build ${CMAKE_ARGS} 2>&1"

# Step 3: Build
echo "[3/4] Building..."
NPROC=$(ssh "${REMOTE_HOST}" "nproc")
ssh "${REMOTE_HOST}" "export PATH=${CUDA_PATH}:\$PATH && cd ${REMOTE_DIR} && cmake --build build -j${NPROC} 2>&1"

echo ""
echo "[4/4] Build complete!"

# Step 4: Optional benchmark
if [ "$BENCH" = true ]; then
    echo ""
    echo "=== Running MPPI Benchmark ==="
    ssh "${REMOTE_HOST}" "cd ${REMOTE_DIR} && ./build/mppi_bench 2>&1" || echo "(benchmark binary not yet built)"
fi

# Show binary info
echo ""
echo "=== Build Artifacts ==="
ssh "${REMOTE_HOST}" "ls -lh ${REMOTE_DIR}/build/truggy ${REMOTE_DIR}/build/mppi_bench ${REMOTE_DIR}/build/serial_test 2>/dev/null || echo '(no binaries yet — need src/ files)'"
