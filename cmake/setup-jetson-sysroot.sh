#!/bin/bash
# Setup Jetson Nano sysroot for cross-compilation from WSL2/x86_64
#
# This script copies required headers and libraries from a running Jetson Nano
# to create a sysroot on the host machine for cross-compilation.
#
# Usage:
#   ./cmake/setup-jetson-sysroot.sh <jetson-ip> [sysroot-path]
#
# Example:
#   ./cmake/setup-jetson-sysroot.sh 192.168.1.100 /opt/jetson-sysroot

set -euo pipefail

JETSON_IP="${1:?Usage: $0 <jetson-ip> [sysroot-path]}"
SYSROOT="${2:-/opt/jetson-sysroot}"
JETSON_USER="${JETSON_USER:-jetson}"

echo "=== Setting up Jetson Nano sysroot ==="
echo "Jetson: ${JETSON_USER}@${JETSON_IP}"
echo "Sysroot: ${SYSROOT}"
echo ""

sudo mkdir -p "${SYSROOT}"

echo "[1/5] Syncing system headers..."
sudo rsync -avz --delete \
    "${JETSON_USER}@${JETSON_IP}:/usr/include/" \
    "${SYSROOT}/usr/include/"

echo "[2/5] Syncing system libraries..."
sudo rsync -avz --delete \
    "${JETSON_USER}@${JETSON_IP}:/usr/lib/aarch64-linux-gnu/" \
    "${SYSROOT}/usr/lib/aarch64-linux-gnu/"

echo "[3/5] Syncing CUDA toolkit..."
sudo rsync -avz --delete \
    "${JETSON_USER}@${JETSON_IP}:/usr/local/cuda-10.2/" \
    "${SYSROOT}/usr/local/cuda-10.2/"

echo "[4/5] Syncing ZED SDK..."
sudo rsync -avz --delete \
    "${JETSON_USER}@${JETSON_IP}:/usr/local/zed/" \
    "${SYSROOT}/usr/local/zed/" 2>/dev/null || \
    echo "  WARNING: ZED SDK not found on Jetson (install it first)"

echo "[5/5] Fixing symlinks..."
# Fix absolute symlinks to be relative within the sysroot
cd "${SYSROOT}"
find . -type l | while read link; do
    target=$(readlink "$link")
    if [[ "$target" = /* ]]; then
        # Convert absolute target to sysroot-relative
        new_target="${SYSROOT}${target}"
        if [ -e "$new_target" ]; then
            sudo ln -sf "$new_target" "$link"
        fi
    fi
done

echo ""
echo "=== Sysroot created at ${SYSROOT} ==="
echo ""
echo "To cross-compile:"
echo "  cmake -B build-cross \\"
echo "    -DCMAKE_TOOLCHAIN_FILE=cmake/jetson-nano-toolchain.cmake \\"
echo "    -DJETSON_SYSROOT=${SYSROOT} \\"
echo "    -DTARGET_NANO_2GB=ON"
echo "  cmake --build build-cross -j\$(nproc)"
echo ""
echo "To deploy:"
echo "  scp build-cross/truggy ${JETSON_USER}@${JETSON_IP}:~/truggy/"
