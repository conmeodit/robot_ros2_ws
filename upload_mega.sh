#!/bin/bash

# Script tự động nạp code cho Arduino Mega
# Sử dụng: ./upload_mega.sh

set -e

PROJECT_DIR="/home/linh-pham/robot_ros2_ws/firmware/arduino_mega_base"
BOARD="arduino:avr:mega"
REQUIRED_CORE="arduino:avr"
REQUIRED_LIBS=("Servo")

ensure_arduino_cli() {
    if command -v arduino-cli >/dev/null 2>&1; then
        return 0
    fi

    echo "⚙️  Chưa có arduino-cli, đang tự cài local cho user..."
    mkdir -p "$HOME/.local/bin"

    local tmp_dir
    tmp_dir=$(mktemp -d)
    trap 'rm -rf "$tmp_dir"' RETURN

    curl -fsSL https://raw.githubusercontent.com/arduino/arduino-cli/master/install.sh -o "$tmp_dir/install.sh"
    sh "$tmp_dir/install.sh" --dest-dir "$HOME/.local/bin"

    export PATH="$HOME/.local/bin:$PATH"

    if ! command -v arduino-cli >/dev/null 2>&1; then
        echo "❌ Cài arduino-cli thất bại."
        exit 1
    fi

    echo "✅ Đã cài arduino-cli: $(arduino-cli version)"
}

ensure_core_and_libs() {
    echo "🔄 Đang cập nhật index core/lib..."
    arduino-cli core update-index >/dev/null
    arduino-cli lib update-index >/dev/null

    if ! arduino-cli core list | grep -q "$REQUIRED_CORE"; then
        echo "📦 Đang cài core $REQUIRED_CORE..."
        arduino-cli core install "$REQUIRED_CORE"
    else
        echo "✅ Core $REQUIRED_CORE đã sẵn sàng"
    fi

    for lib in "${REQUIRED_LIBS[@]}"; do
        if ! arduino-cli lib list | awk -F' ' '{print $1}' | grep -qx "$lib"; then
            echo "📚 Đang cài thư viện $lib..."
            arduino-cli lib install "$lib"
        else
            echo "✅ Thư viện $lib đã sẵn sàng"
        fi
    done
}

echo "=== Arduino Mega Auto Upload ==="
ensure_arduino_cli
ensure_core_and_libs

echo "Đang kiểm tra cổng serial..."

# List tất cả cổng available
PORTS=$(arduino-cli board list | grep -oE "^/dev/tty[A-Za-z0-9]+" | sort | uniq)

if [ -z "$PORTS" ]; then
    echo "❌ Không tìm thấy cổng serial nào. Vui lòng kiểm tra kết nối USB."
    exit 1
fi

echo "Các cổng serial khả dụng:"
echo "$PORTS"
echo ""

# Compile trước
echo "📦 Đang compile sketch..."
arduino-cli compile --fqbn "$BOARD" "$PROJECT_DIR" > /dev/null 2>&1
echo "✅ Compile thành công"
echo ""

# Thử upload vào từng cổng
echo "📤 Đang nạp code..."
UPLOAD_SUCCESS=0

for PORT in $PORTS; do
    echo "  Thử cổng: $PORT"
    
    if arduino-cli upload -p "$PORT" --fqbn "$BOARD" "$PROJECT_DIR" 2>/dev/null; then
        echo ""
        echo "✅ Nạp thành công vào: $PORT"
        UPLOAD_SUCCESS=1
        break
    else
        echo "  ❌ Cổng này không phải Arduino Mega hoặc nạp thất bại"
    fi
done

echo ""
if [ $UPLOAD_SUCCESS -eq 1 ]; then
    echo "🎉 Nạp code xong! Board sẵn sàng."
    exit 0
else
    echo "❌ Không thể nạp code vào bất kỳ cổng nào."
    echo "Kiểm tra:"
    echo "  1. Arduino Mega có cắm vào Pi không?"
    echo "  2. Cáp USB có phải dây truyền dữ liệu không? (không phải dây sạc)"
    echo "  3. Thử nhấn nút RESET trên board trước khi nạp"
    exit 1
fi
