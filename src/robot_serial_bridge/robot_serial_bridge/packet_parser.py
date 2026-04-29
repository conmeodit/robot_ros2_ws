"""Parse machine-readable telemetry from Arduino Mega.

Current firmware format:
    STAT,mode,estop,motor_enabled,left_ticks,right_ticks,ax,ay,az,gx,gy,gz,left_pwm,right_pwm

Legacy format is also accepted:
    $TELE,left_ticks,right_ticks,accelX,accelY,accelZ,gyroX,gyroY,gyroZ
"""

from dataclasses import dataclass
from typing import Optional


_ACCEL_SCALE = 9.80665 / 16384.0
_GYRO_SCALE = 3.14159265 / (180.0 * 131.0)


@dataclass
class TelemetryFrame:
    """Parsed telemetry from one Arduino telemetry line."""

    left_ticks: int
    right_ticks: int
    accel_x: float
    accel_y: float
    accel_z: float
    gyro_x: float
    gyro_y: float
    gyro_z: float


def parse_telemetry(line: str) -> Optional[TelemetryFrame]:
    """Parse a single telemetry line from the Arduino Mega."""
    line = line.strip()
    try:
        if line.startswith('STAT,'):
            parts = [part.strip() for part in line.split(',')]
            if len(parts) < 12:
                return None
            left_ticks = int(parts[4])
            right_ticks = int(parts[5])
            raw_ax = int(parts[6])
            raw_ay = int(parts[7])
            raw_az = int(parts[8])
            raw_gx = int(parts[9])
            raw_gy = int(parts[10])
            raw_gz = int(parts[11])
        elif line.startswith('$TELE,'):
            parts = [part.strip() for part in line[6:].split(',')]
            if len(parts) != 8:
                return None
            left_ticks = int(parts[0])
            right_ticks = int(parts[1])
            raw_ax = int(parts[2])
            raw_ay = int(parts[3])
            raw_az = int(parts[4])
            raw_gx = int(parts[5])
            raw_gy = int(parts[6])
            raw_gz = int(parts[7])
        else:
            return None
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
