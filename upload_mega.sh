#!/bin/bash

# Script tự động nạp code cho Arduino Mega
# Sử dụng: ./upload_mega.sh

set -e

PROJECT_DIR="/home/linhpham/robot_ros2_ws/firmware/arduino_mega_base"
BOARD="arduino:avr:mega"

echo "=== Arduino Mega Auto Upload ==="
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
