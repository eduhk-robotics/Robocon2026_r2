# Robocon2026 R2

This workspace targets ROS 2 Jazzy on Ubuntu 24.04 and is intended to be built
and deployed inside a Docker container on Raspberry Pi (ARM64).

## Docker build

Create a `Dockerfile` in this directory with the following contents:

```Dockerfile
FROM ros:jazzy-ros-base

ENV DEBIAN_FRONTEND=noninteractive

RUN apt-get update && apt-get install -y --no-install-recommends \
    python3-pip \
    python3-colcon-common-extensions \
    can-utils \
    && rm -rf /var/lib/apt/lists/*

WORKDIR /workspace
COPY 2026R2_ws /workspace/2026R2_ws

RUN /bin/bash -lc "source /opt/ros/jazzy/setup.bash && \
    cd /workspace/2026R2_ws && \
    rosdep update && rosdep install --from-paths src -r -y && \
    colcon build --symlink-install"

CMD ["/bin/bash"]
```

Build the image:

```bash
docker build -t robocon2026-r2:jazzy .
```

## Host CAN setup (gs_usb / CANable)

Bring up `can0` on the host before starting the container (bitrate example below):

```bash
sudo modprobe gs_usb
sudo ip link set can0 up type can bitrate 1000000
```

Verify:

```bash
ip link show can0
```

## Run container (USB / CAN / I2C)

Adjust device paths for your hardware. For USB serial, prefer `/dev/serial/by-id/...`.

```bash
docker run --rm -it \
  --network host \
  --ipc host \
  --device /dev/serial/by-id/YOUR_DEVICE_ID \
  --device /dev/i2c-1 \
  --cap-add NET_ADMIN \
  robocon2026-r2:jazzy
```

Notes:
- `--network host` is recommended for ROS 2 DDS discovery on Raspberry Pi.
- If CAN is already up on the host, you can often drop `--cap-add NET_ADMIN`.
- If you see permission issues, try adding `--privileged` for quick diagnosis.

## Run ROS 2

Inside the container:

```bash
source /opt/ros/jazzy/setup.bash
source /workspace/2026R2_ws/install/setup.bash
ros2 launch r2_launch launch.py
```

**ROS Topics and Message Structures**

- **Nodes**: `local_navigation_node` (navigation/navigation_node.py), `general_navigation_node` (navigation/general_navigation_node.py), and `omni_wheel_speed_node` (navigation/omni_wheel_speed_node.py).

- **`ps4` (topic)**: subscribed by `local_navigation_node`.
  - Type: `sensor_msgs/Joy`.
  - Used fields: `axes[0]` = left analog horizontal, `axes[1]` = left analog vertical, `axes[3]` = right analog horizontal; buttons currently not used.

- **`local_driving` (topic)**: published by `local_navigation_node`, subscribed by `omni_wheel_speed_node`.
  - Type: `std_msgs/Float32MultiArray`.
  - Data format: `[direction_deg, plane_speed, rotation_speed]` where:
    - `direction_deg`: float, degrees (0–360), adjusted (+90) in code.
    - `plane_speed`: float, 0–8192 (joystick magnitude scaled to 0–8192).
    - `rotation_speed`: float, -8192..8192 (right stick horizontal scaled, sign inverted).

- **`general_driving` (topic)**: published by `general_navigation_node`.
  - Type: `std_msgs/Float32MultiArray`.
  - Data format: `[x0, y0, yaw0_deg, x1, y1, yaw1_deg]` (absolute segment).
  - Behavior: publishes each segment from `config/general_path.yaml` at a fixed interval.
  - Parameters:
    - `map_file`: override the path to the YAML map file.
    - `publish_period_s`: publish interval in seconds.
    - `loop`: whether to loop through segments.

- **`damiao_control` (topic)**: published by `omni_wheel_speed_node`.
  - Type: `std_msgs/Float32MultiArray`.
  - Message per motor: `[motor_id, mode, speed, position]` where:
    - `motor_id`: motor index as float (1,2,3,4...).
    - `mode`: 1.0 (speed mode).
    - `speed`: float, computed wheel speed converted and divided by `19.20321` in code before sending.
    - `position`: 0.0 (unused for mode 1).

- **QoS**: both publishers/subscriptions use queue size `10` (default in code).

This is a concise mapping of topics and payload formats used by the navigation nodes.

**General Path Map**

- Default map file: `2026R2_ws/src/navigation/config/general_path.yaml` (installed to `share/navigation/config`).
- Structure (segments with absolute poses):

```yaml
frame_id: "map"
units:
  distance: "m"
  angle: "deg"
segments:
  - start: {x: 0.0, y: 0.0, yaw_deg: 0}
    end:   {x: 2.0, y: 0.0, yaw_deg: 0}
```
