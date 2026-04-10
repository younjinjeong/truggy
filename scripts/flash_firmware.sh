#!/bin/bash
# Flash Arduino Uno firmware
# Usage: ./scripts/flash_firmware.sh [port]
set -euo pipefail

PORT="${1:-/dev/ttyACM0}"
SCRIPT_DIR="$(cd "$(dirname "$0")" && pwd)"
FIRMWARE="${SCRIPT_DIR}/../firmware/truggy_bridge"

echo "[flash] Compiling firmware..."
arduino-cli compile --fqbn arduino:avr:uno "$FIRMWARE"

echo "[flash] Uploading to ${PORT}..."
arduino-cli upload -p "$PORT" --fqbn arduino:avr:uno "$FIRMWARE"

echo "[flash] Done. Firmware flashed to ${PORT}"
