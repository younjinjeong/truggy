#!/bin/bash
# Flash TruggyAD firmware to Arduino Uno or M5StickC Plus 2
#
# Usage:
#   ./scripts/flash_firmware.sh                  # Arduino Uno (default)
#   ./scripts/flash_firmware.sh --esp32           # M5StickC Plus 2
#   ./scripts/flash_firmware.sh --esp32 /dev/ttyUSB0
set -euo pipefail

SCRIPT_DIR="$(cd "$(dirname "$0")" && pwd)"
FIRMWARE_DIR="$(dirname "$SCRIPT_DIR")/firmware"
TARGET="uno"
PORT=""

for arg in "$@"; do
    case "$arg" in
        --esp32) TARGET="esp32" ;;
        /dev/*) PORT="$arg" ;;
        --help|-h)
            echo "Usage: $0 [--esp32] [port]"
            echo "  Default: Arduino Uno on /dev/ttyACM0"
            echo "  --esp32: M5StickC Plus 2 on /dev/ttyUSB0"
            exit 0
            ;;
    esac
done

if [ "$TARGET" = "esp32" ]; then
    PORT="${PORT:-/dev/ttyUSB0}"
    FQBN="m5stack:esp32:m5stick_c_plus2"
    SKETCH="${FIRMWARE_DIR}/truggy_bridge_esp32"
    echo "[flash] Target: M5StickC Plus 2 (ESP32)"

    # Ensure M5Stack board package is installed
    arduino-cli core list | grep -q m5stack || {
        echo "[flash] Installing M5Stack board package..."
        arduino-cli config add board_manager.additional_urls \
            "https://m5stack.oss-cn-shenzhen.aliyuncs.com/resource/arduino/package_m5stack_index.json"
        arduino-cli core update-index
        arduino-cli core install m5stack:esp32
    }

    # Install required libraries
    arduino-cli lib install "M5StickCPlus2" "ESP32Servo" "Adafruit BNO08x" 2>/dev/null || true
else
    PORT="${PORT:-/dev/ttyACM0}"
    FQBN="arduino:avr:uno"
    SKETCH="${FIRMWARE_DIR}/truggy_bridge"
    echo "[flash] Target: Arduino Uno"

    arduino-cli lib install "Adafruit BNO08x" 2>/dev/null || true
fi

echo "[flash] Compiling ${SKETCH}..."
arduino-cli compile --fqbn "$FQBN" "$SKETCH"

echo "[flash] Uploading to ${PORT}..."
arduino-cli upload -p "$PORT" --fqbn "$FQBN" "$SKETCH"

echo "[flash] Done! Firmware flashed to ${PORT}"
