# robot_vision

ROS 2 package for top-down camera trash detection.

Pipeline:

```text
/camera/image_raw
  -> robot_vision_node
  -> /vision/debug_image
  -> /vision/trash_obstacles
  -> /vision/trash_markers
```

Put the YOLOv8 model at:

```bash
src/robot_vision/models/best.pt
```

Runtime dependency:

```bash
python3 -m pip install ultralytics
```

Run with autonomy:

```bash
ros2 launch vacuum_driver autonomous_mapping.launch.py use_vision:=true use_rviz:=true
```

Calibrate `config/vision.yaml` before real obstacle avoidance. The preferred setup is a homography that maps image pixels to floor coordinates in meters for the downward camera.
