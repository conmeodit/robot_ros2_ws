# vacuum_driver

ROS 2 package for the real/Webots vacuum robot.

Provided nodes:

- `pure_driver`: Webots bridge for `/scan`, `/odom`, TF and `/cmd_vel`.
- `real_driver`: Arduino Mega bridge for real `/cmd_vel`, `/odom`, `/odom/raw`, `/imu/data_raw`, `/imu/data`, and hardware telemetry topics.
- `scan_filter_node`: real lidar preprocessing from `/scan/raw` to `/scan`.
- `slam_session_manager_node`: resets `slam_toolbox` at launch startup.
- `autonomous_cleaning_node`: frontier exploration followed by visited-cell coverage.

## Build

```bash
rm -rf ~/ros2_ws/build/vacuum_driver ~/ros2_ws/install/vacuum_driver
cd ~/ros2_ws
colcon build --base-paths src/vacuum_driver --packages-select vacuum_driver --symlink-install
source install/setup.bash
```

## Run

```bash
ros2 run vacuum_driver pure_driver
```

Mapping only:

```bash
ros2 launch vacuum_driver mapping.launch.py
```

Autonomous mapping and full reachable-area coverage:

```bash
ros2 launch vacuum_driver autonomous_mapping.launch.py
```

Default real hardware assumptions:

- Arduino Mega command/telemetry: `/dev/rfcomm0` at `9600`.
- SLAMTEC/SLLIDAR lidar: `lidar_port:=auto`, `lidar_baudrate:=auto`, publishes raw scan to `/scan/raw`.
- `scan_filter_node` publishes filtered `/scan` for SLAM and autonomy.
- `real_driver` sends motor commands from `/cmd_vel` to Arduino and publishes encoder odometry on `/odom`.
- Odometry is standardized to `wheel_radius=0.0425`, `wheel_separation=0.42`, `encoder_ticks_per_rev=1320.0`.
- Encoder direction is auto-detected on the first small straight forward command. Disable `auto_detect_encoder_direction` only after confirming fixed encoder signs.
- Keep `real_driver` as the single owner of the Arduino serial link and `/odom` path.

USB Arduino example:

```bash
ros2 launch vacuum_driver autonomous_mapping.launch.py arduino_port:=/dev/ttyACM0 arduino_baudrate:=115200
```

Useful real data topics:

- `/hardware/mega/telemetry`: raw `STAT,...` telemetry line.
- `/hardware/encoder_ticks`: `[left_ticks, right_ticks]`.
- `/hardware/motor_pwm`: `[left_pwm, right_pwm]`.
- `/imu/data_raw` and `/imu/data`: MPU6500 data from Arduino.
- `/odom/raw` and `/odom`: encoder odometry used by SLAM.

RViz topics used by the autonomous node:

- `/autonomy/path`: current A* path.
- `/autonomy/markers`: robot body, footprint, target, frontier cells and visited cells.
