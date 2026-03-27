# specs ROS 2 Package

This package contains three ROS 2 Python nodes that subscribe to `/camera/image_raw` and process camera frames:

- `video_display.py` — displays camera frames in grayscale
- `apriltag_test.py` — detects a single AprilTag and draws the detection
- `localization.py` — detects AprilTags and localizes a bot relative to board corners

## Requirements

- ROS 2 installed
- `opencv-python` / `opencv-contrib-python` or system OpenCV with AprilTag support
- `cv_bridge`
- `sensor_msgs`
- `rclpy`

## Build

From the package directory:

```bash
cd /home/mbot/scribbles-and-specs/Specs
colcon build --symlink-install
```

## Source the workspace

```bash
source install/setup.bash
```

## Run

```bash
ros2 run specs video_display.py
ros2 run specs apriltag_test.py
ros2 run specs localization.py
```

If you need to run the package from a workspace root without building the install tree, use `ros2 run` after sourcing the workspace.

## Notes

- The nodes expect an active ROS 2 camera publisher on `/camera/image_raw`. This is provided by the class firmware ([mbot_ws](https://rob550-docs.github.io/docs/botlab/mbot-system-setup-Pi5)) and can be run with the following command

```bash
ros2 run camera_ros camera_node --ros-args -p width:=640 -p height:=480 -p format:=BGR888
```

- Press `q` in the OpenCV window to exit the node.
