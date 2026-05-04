Put the trained YOLOv8 model here as `best.pt`.

Default launch path:

```bash
src/robot_vision/models/best.pt
```

You can also pass a model explicitly:

```bash
ros2 launch vacuum_driver autonomous_mapping.launch.py use_vision:=true vision_model_path:=/absolute/path/to/best.pt
```
