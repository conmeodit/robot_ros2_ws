# Arduino Mega Base Controller

Sketch: `arduino_mega_base.ino`

## Serial protocol
- RX command from Pi: `CMD_VEL,<linear_x_mps>,<angular_z_rps>`
- Optional: `ESTOP,1|0`, `MOTOR,1|0`
- Arm control: `SERVO,<id>,<deg>`
  - `id=1..5`, riêng khớp nâng `id=2` sẽ chạy đồng bộ 2 servo (`D31/D32`)
- TX telemetry to Pi (20 Hz):
  - `STAT,mode,estop,motor_enabled,left_ticks,right_ticks,battery_voltage,battery_current,fault,fault_text`

## Default pins
- Left BTS7960: `RPWM=D5`, `LPWM=D6`
- Right BTS7960: `RPWM=D7`, `LPWM=D8`
- Left encoder: `A=D2`, `B=D3`
- Right encoder: `A=D18`, `B=D19`
- UART link to HC-05: `Serial2` (`TX2=D16`, `RX2=D17`)

## Tune before run
- `WHEEL_RADIUS_M`
- `WHEEL_BASE_M`
- `TICKS_PER_REV`
- PID gains: `KP`, `KI`, `KD`

## Safety
- Command timeout is `500 ms`: robot auto stops if no command.
