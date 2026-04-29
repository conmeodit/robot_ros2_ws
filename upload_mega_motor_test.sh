#!/bin/bash

set -e

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
PROJECT_DIR="$SCRIPT_DIR/firmware/mega_motor_test"
BOARD="arduino:avr:mega"
PORT="${1:-}"

if ! command -v arduino-cli >/dev/null 2>&1; then
    echo "arduino-cli not found. Install it or run ./upload_mega.sh once to install/setup it."
    exit 1
fi

if ! arduino-cli core list | grep -q "arduino:avr"; then
    arduino-cli core update-index
    arduino-cli core install arduino:avr
fi

if [ -z "$PORT" ]; then
    echo "Available boards/ports:"
    arduino-cli board list
    echo ""
    PORT="$(arduino-cli board list | awk '/usb-1a86|USB Serial|Arduino Mega|ttyACM|ttyUSB/ {print $1; exit}')"
fi

if [ -z "$PORT" ]; then
    echo "No Arduino serial port found. Pass it explicitly, for example:"
    echo "  ./upload_mega_motor_test.sh /dev/serial/by-id/usb-1a86_USB_Serial-if00-port0"
    exit 1
fi

echo "Compiling $PROJECT_DIR"
arduino-cli compile --fqbn "$BOARD" "$PROJECT_DIR"

echo "Uploading motor test sketch to $PORT"
arduino-cli upload -p "$PORT" --fqbn "$BOARD" "$PROJECT_DIR"

echo ""
echo "Done. Open serial monitor at 115200 baud:"
echo "  python3 -m serial.tools.miniterm $PORT 115200"
echo ""
echo "Commands: F B L R S, PWM,<left>,<right>, TEST, MOTOR,1, ESTOP,0"
