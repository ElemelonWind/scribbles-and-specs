# specs ROS 2 Package

This package contains ROS 2 Python nodes that subscribe to `/camera/image_raw` and process camera frames, plus a comms bridge that streams bot pose and waypoints to the Scribbles ESP32:

- `video_display.py` — displays camera frames in grayscale
- `apriltag_test.py` — detects a single AprilTag and draws the detection
- `localization.py` — detects AprilTags, localizes the bot, publishes `/specs/board_*` and `/specs/bot_pose`
- `ink_detection.py` — detects ink marks inside the board region and publishes `/specs/ink_waypoints`
- `ink_detection_test.py` — standalone (non-ROS) test harness for the ink detection pipeline on a single image
- `socket_test.py` — opens a TCP socket to Scribbles, sends a `ping`, and logs the response
- `specs_comms.py` — subscribes to `/specs/bot_pose` + `/specs/ink_waypoints` and sends 3-byte packets to the ESP32

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
chmod +x localization.py ink_detection.py socket_test.py specs_comms.py # might need to run
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
ros2 run specs ink_detection.py
ros2 run specs specs_comms.py
```

To run the full integration (bot pose + waypoints → ESP32), in separate shells:

```bash
# 1. Camera
ros2 run camera_ros camera_node --ros-args -p width:=640 -p height:=480 -p format:=BGR888

# 2. Localization (publishes /specs/bot_pose and /specs/board_*)
ros2 run specs localization.py

# 3. Ink detection (publishes /specs/ink_waypoints)
ros2 run specs ink_detection.py

# 4. Comms bridge (forwards to the ESP32)
ros2 run specs specs_comms.py \
    --ros-args -p host:=192.168.4.1 -p port:=3333 \
                -p board_size_m:=1.0 -p error_bound_m:=0.05 \
                -p waypoint_period_s:=2.0
```

`specs_comms.py` parameters:
- `host` / `port` — ESP32 address (default `192.168.4.1:3333`, the softAP default)
- `board_size_m` — physical side length of the whiteboard (used to convert the 5cm error bound into normalized board coords)
- `error_bound_m` — how close the bot must be to the current waypoint before advancing
- `waypoint_period_s` — how often the current waypoint is re-sent (default 2.0s)

## ESP32 ↔ Specs message format

`specs_comms.py` sends raw 3-byte packets over the TCP socket:

```
Byte 0: [bit7: type] [bits 6..0: heading_7bit]
          type = 0  →  location (bot pose)
          type = 1  →  waypoint (next target)
Byte 1: x (0-255, grid)
Byte 2: y (0-255, grid)
```

- `heading_7bit` encodes 0-360° into 7 bits (≈2.8° per step): `heading_7bit = round(deg * 128 / 360) & 0x7F`. The ESP32 decodes back to degrees with `deg = heading_7bit * 360 / 128`.
- Waypoints always have heading 0.
- Location packets are sent every time a new pose is available (as fast as the camera / localizer can produce them).
- Waypoint packets are sent every `waypoint_period_s`. Once the bot is within `error_bound_m` of the current waypoint, the next waypoint is advanced automatically and will be sent on the next tick.

The ESP32 ([`Scribbles/src/main.cpp`](../Scribbles/src/main.cpp)) reads 3-byte packets from the TCP socket and prints decoded `LOC` / `WP` lines to the serial console. Build/flash via PlatformIO (`pio run -t upload` in `Scribbles/`).

## Notes

- The nodes expect an active ROS 2 camera publisher on `/camera/image_raw`. This is provided by the class firmware ([mbot_ws](https://rob550-docs.github.io/docs/botlab/mbot-system-setup-Pi5)) and can be run with:

```bash
ros2 run camera_ros camera_node --ros-args -p width:=640 -p height:=480 -p format:=BGR888
```

- Press `q` in the OpenCV window to exit the node.

- The ESP32 creates a hotspot for the socket. While ssh'ed into the pi, run `bash connect-esp32-hotspot.sh` and re-ssh using the new IP displayed on the display.
