#!/bin/bash
# Run TruggyAD on Jetson with proper power/clock settings
# Usage: ./scripts/run.sh [--sim] [--log]
set -euo pipefail

SCRIPT_DIR="$(cd "$(dirname "$0")" && pwd)"
PROJECT_DIR="$(dirname "$SCRIPT_DIR")"
BUILD_DIR="${PROJECT_DIR}/build"
BINARY="${BUILD_DIR}/truggy"

if [ ! -f "$BINARY" ]; then
    echo "Binary not found. Building..."
    cmake -B "$BUILD_DIR" -DTARGET_NANO_2GB=ON -DCMAKE_EXPORT_COMPILE_COMMANDS=ON "$PROJECT_DIR"
    cmake --build "$BUILD_DIR" -j4
fi

# Set power mode and clocks (requires sudo, non-fatal if not root)
echo "[run] Setting performance mode..."
sudo nvpmodel -m 0 2>/dev/null || echo "  (nvpmodel skipped, not on Jetson or not root)"
sudo jetson_clocks 2>/dev/null || echo "  (jetson_clocks skipped)"

# Build args
ARGS="--config ${PROJECT_DIR}/config/truggy.yaml --costs ${PROJECT_DIR}/config/mppi_costs.yaml"

# Check for --log flag
LOG_ARG=""
for arg in "$@"; do
    if [ "$arg" = "--log" ]; then
        TIMESTAMP=$(date +%Y%m%d_%H%M%S)
        LOG_FILE="${PROJECT_DIR}/data/telemetry_${TIMESTAMP}.bin"
        mkdir -p "${PROJECT_DIR}/data"
        LOG_ARG="--log ${LOG_FILE}"
        echo "[run] Logging to ${LOG_FILE}"
    fi
done

echo "[run] Starting TruggyAD..."
echo "[run] Binary: ${BINARY}"
exec "$BINARY" $ARGS $LOG_ARG "$@"
