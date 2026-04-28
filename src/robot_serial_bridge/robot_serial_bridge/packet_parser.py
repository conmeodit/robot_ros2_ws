"""Parse machine-readable telemetry from Arduino Mega.

Expected CSV format from firmware:
    $TELE,left_ticks,right_ticks,accelX,accelY,accelZ,gyroX,gyroY,gyroZ

Returns a dict on success, None on parse failure.
"""

from dataclasses import dataclass
from typing import Optional


# MPU6500 raw-to-SI conversion constants
_ACCEL_SCALE = 9.80665 / 16384.0   # ±2g range → m/s²
_GYRO_SCALE = 3.14159265 / (180.0 * 131.0)  # ±250°/s range → rad/s


@dataclass
class TelemetryFrame:
    """Parsed telemetry from one $TELE line."""
    left_ticks: int
    right_ticks: int
    accel_x: float   # m/s²
    accel_y: float   # m/s²
    accel_z: float   # m/s²
    gyro_x: float    # rad/s
    gyro_y: float    # rad/s
    gyro_z: float    # rad/s


def parse_telemetry(line: str) -> Optional[TelemetryFrame]:
    """Parse a single telemetry line from the Arduino Mega.

    Args:
        line: Raw line from serial, e.g.
              ``$TELE,1234,-567,120,-45,16384,30,-12,5``

    Returns:
        A :class:`TelemetryFrame` on success, ``None`` if the line is not
        a valid telemetry packet.
    """
    line = line.strip()
    if not line.startswith('$TELE,'):
        return None

    parts = line[6:].split(',')
    if len(parts) != 8:
        return None

    try:
        left_ticks = int(parts[0])
        right_ticks = int(parts[1])
        raw_ax = int(parts[2])
        raw_ay = int(parts[3])
        raw_az = int(parts[4])
        raw_gx = int(parts[5])
        raw_gy = int(parts[6])
        raw_gz = int(parts[7])
    except ValueError:
        return None

    return TelemetryFrame(
        left_ticks=left_ticks,
        right_ticks=right_ticks,
        accel_x=raw_ax * _ACCEL_SCALE,
        accel_y=raw_ay * _ACCEL_SCALE,
        accel_z=raw_az * _ACCEL_SCALE,
        gyro_x=raw_gx * _GYRO_SCALE,
        gyro_y=raw_gy * _GYRO_SCALE,
        gyro_z=raw_gz * _GYRO_SCALE,
    )
