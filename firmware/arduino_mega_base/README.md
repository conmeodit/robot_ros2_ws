# Arduino Mega Base Controller

Sketch: `arduino_mega_base.ino`

## Serial protocol

- RX command from Pi by USB `Serial` or Bluetooth `Serial2`: `CMD_VEL,<linear_x_mps>,<angular_z_rps>`
- Optional: `ESTOP,1|0`, `MOTOR,1|0`
- Arm control: `SERVO,<id>,<deg>`
  - `id=1..5`; lift joint `id=2` drives both lift servos (`D23/D24`) together.
- TX telemetry to Pi/phone at 20 Hz:
  - `STAT,mode,estop,motor_enabled,left_ticks,right_ticks,ax,ay,az,gx,gy,gz,left_pwm,right_pwm`
  - MPU6500 raw scale is parsed by ROS as `16384 LSB/g` and `131 LSB/(deg/s)`.

## Default pins

- Left BTS7960: `RPWM=D6`, `LPWM=D5`
- Right BTS7960: `RPWM=D8`, `LPWM=D7`
- Left encoder: `A=D2`, `B=D3`
- Right encoder: `A=D19`, `B=D18`
- Servos: `S1=D22`, `S2_LEFT=D23`, `S2_RIGHT=D24`, `S3=D25`, `S4=D26`, `S5=D27`
- MPU6500: I2C address `0x68`
- USB link to Pi: `Serial` (`115200`)
- UART link to HC-05: `Serial2` (`TX2=D16`, `RX2=D17`, `9600`)

## Tune before run

- ROS odometry parameters live in `src/vacuum_driver/config/real_hardware.yaml`.
- `WHEEL_SEPARATION_M`
- `MAX_LINEAR_CMD_MPS`, `MAX_ANGULAR_CMD_RADPS`
- `MIN_DRIVE_PWM`, `MAX_DRIVE_PWM`
- `LEFT_MOTOR_INVERTED`, `RIGHT_MOTOR_INVERTED`

## Safety

- Command timeout is `500 ms`: robot auto stops if no command.
