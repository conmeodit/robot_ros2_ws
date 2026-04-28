# Arduino Mega Base Controller

Sketch: `arduino_mega_base.ino`

## Serial protocol
- RX command from Pi: `CMD_VEL,<linear_x_mps>,<angular_z_rps>`
- Optional: `ESTOP,1|0`, `MOTOR,1|0`
- Arm control: `SERVO,<id>,<deg>`
  - `id=1..5`, riêng khớp nâng `id=2` chạy đồng bộ 2 servo (`D23/D24`)
- TX telemetry to Pi (20 Hz):
  - `$TELE,left_ticks,right_ticks,accelX,accelY,accelZ,gyroX,gyroY,gyroZ`

## Default pins
- Left BTS7960: `RPWM=D6`, `LPWM=D5`
- Right BTS7960: `RPWM=D8`, `LPWM=D7`
- Left encoder: `A=D2`, `B=D3`
- Right encoder: `A=D19`, `B=D18`
- Servos: `S1=D22`, `S2_LEFT=D23`, `S2_RIGHT=D24`, `S3=D25`, `S4=D26`, `S5=D27`
- MPU6500: I2C address `0x68`
- UART link to HC-05: `Serial2` (`TX2=D16`, `RX2=D17`)

## Tune before run
- `WHEEL_BASE_M`
- `MAX_WHEEL_SPEED_MPS`
- `TICKS_PER_REV` is configured on the ROS 2 odometry node.

## Safety
- Command timeout is `500 ms`: robot auto stops if no command.
