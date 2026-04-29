#!/bin/bash

set -e

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
PROJECT_DIR="$SCRIPT_DIR/firmware/arduino_mega_base"
BOARD="arduino:avr:mega"
REQUIRED_CORE="arduino:avr"
REQUIRED_LIBS=("Servo")

ensure_arduino_cli() {
    if command -v arduino-cli >/dev/null 2>&1; then
        return 0
    fi

    echo "arduino-cli not found; installing to ~/.local/bin"
    mkdir -p "$HOME/.local/bin"

    local tmp_dir
    tmp_dir="$(mktemp -d)"
    trap 'rm -rf "$tmp_dir"' RETURN

    curl -fsSL https://raw.githubusercontent.com/arduino/arduino-cli/master/install.sh -o "$tmp_dir/install.sh"
    sh "$tmp_dir/install.sh" --dest-dir "$HOME/.local/bin"
    export PATH="$HOME/.local/bin:$PATH"

    if ! command -v arduino-cli >/dev/null 2>&1; then
        echo "Failed to install arduino-cli"
        exit 1
    fi
}

ensure_core_and_libs() {
    arduino-cli core update-index >/dev/null
    arduino-cli lib update-index >/dev/null

    if ! arduino-cli core list | grep -q "$REQUIRED_CORE"; then
        arduino-cli core install "$REQUIRED_CORE"
    fi

    for lib in "${REQUIRED_LIBS[@]}"; do
        if ! arduino-cli lib list | awk '{print $1}' | grep -qx "$lib"; then
            arduino-cli lib install "$lib"
        fi
    done
}

detect_mega_ports() {
    local candidates=()
    local path

    for path in \
        /dev/serial/by-id/*1a86* \
        /dev/serial/by-id/*CH340* \
        /dev/serial/by-id/*USB_Serial* \
        /dev/serial/by-id/*Arduino* \
        /dev/ttyACM* \
        /dev/ttyUSB*; do
        [ -e "$path" ] || continue
        case "$path" in
            *CP210*|*Silicon_Labs*) continue ;;
        esac
        candidates+=("$path")
    done

    if [ "${#candidates[@]}" -eq 0 ]; then
        arduino-cli board list | awk '/Mega|Arduino|1a86|CH340|USB Serial|ttyACM|ttyUSB/ {print $1}'
        return
    fi

    printf "%s\n" "${candidates[@]}" | awk '!seen[$0]++'
}

echo "=== Upload Arduino Mega base firmware ==="

if [ ! -d "$PROJECT_DIR" ]; then
    echo "Sketch directory not found: $PROJECT_DIR"
    exit 1
fi

ensure_arduino_cli
ensure_core_and_libs

echo "Compiling $PROJECT_DIR"
arduino-cli compile --fqbn "$BOARD" "$PROJECT_DIR"

mapfile -t PORTS < <(detect_mega_ports)
if [ "${#PORTS[@]}" -eq 0 ]; then
    echo "No Arduino Mega serial port found."
    echo "Available boards:"
    arduino-cli board list
    exit 1
fi

echo "Candidate Arduino ports:"
printf "  %s\n" "${PORTS[@]}"

for PORT in "${PORTS[@]}"; do
    echo "Uploading to $PORT"
    if arduino-cli upload -p "$PORT" --fqbn "$BOARD" "$PROJECT_DIR"; then
        echo "Upload succeeded on $PORT"
        echo ""
        echo "Verify USB telemetry:"
        echo "  python3 -m serial.tools.miniterm $PORT 115200"
        echo "Expected boot line: BOOT,arduino_mega_base,usb_baud=115200"
        exit 0
    fi
done

echo "Upload failed on every candidate port."
exit 1
