# OriginCar - Intelligent Line-Following & Obstacle-Avoidance Robot

[![ROS2](https://img.shields.io/badge/ROS2-Humble-blue)](https://docs.ros.org/en/humble/)
[![Platform](https://img.shields.io/badge/Platform-Horizon%20RDK-orange)](https://developer.horizon.ai)
[![License](https://img.shields.io/badge/License-Apache%202.0-green)](LICENSE)

An intelligent autonomous robot car based on ROS2 framework and AI vision technology, featuring line-following, obstacle avoidance, human tracking, and QR code recognition capabilities.

## Features

- **Intelligent Line-Following**: YOLOv5-based visual line detection with PID control algorithm
- **Autonomous Obstacle Avoidance**: Real-time cone detection with intelligent avoidance strategy
- **Human Tracking**: Horizon Mono2d body detection for following human targets
- **QR Code Recognition**: ZBar-based QR code decoding for task triggers
- **AI Image Analysis**: Integration with Doubao Vision LLM for advanced image understanding
- **Multi-Control Modes**: Support for Twist velocity and Ackermann steering control

## Hardware Specifications

| Parameter | Value |
|-----------|-------|
| Dimensions | 276mm × 164mm × 80mm |
| Wheel Radius | 30mm |
| Wheel Track (L-R) | 189mm |
| Wheelbase (F-R) | 168.2mm |
| Serial Baud Rate | 115200 bps |
| Line-Following Speed | 0.45-0.6 m/s |
| Obstacle Avoidance Speed | 0.25 m/s |


## Software Modules

| Package | Description |
|---------|-------------|
| `origincar_base` | Base control node, serial communication, odometry & IMU publishing |
| `origincar_competition` | Competition control node, line-following & obstacle avoidance |
| `origincar_body_tracking` | Human tracking node with vision-based tracking strategy |
| `origincar_description` | URDF robot model for visualization |
| `origincar_msg` | Custom message definitions |
| `qr_decoder` | QR code recognition node |
| `image_upload_analyzer` | LLM image analysis node |

## Installation

### Prerequisites

- ROS2 Humble
- Horizon RDK Board (with TROS installed)
- Python 3.8+

### Dependencies

```bash
# Install Python dependencies
pip install pyzbar
pip install volcengine-python-sdk[ark]

# Install TROS packages (on RDK board)
sudo apt install tros-mono2d-body-detection
```

### Build

```bash
cd ~/Desktop/dev_ws
colcon build
source install/setup.bash
```

## Quick Start

### Human Tracking (One-Line Start)

```bash
cd ~/Desktop/dev_ws
chmod +x start_body_tracking.sh
./start_body_tracking.sh
```

### Line-Following & Obstacle Avoidance

```bash
# Step 1: Start base node
ros2 launch origincar_base origincar_bringup.launch.py

# Step 2: Start vision processing
ros2 launch racing_obstacle_detection_yolo racing_obstacle_detection_yolo

# Step 3: Start competition control (includes camera & QR decoder)
ros2 launch origincar_competition start.launch.py

# Step 4: Start web interface (optional)
ros2 launch rosbridge_server rosbridge_websocket_launch.xml
```

### Human Tracking (Step-by-Step)

```bash
# Step 1: Start base node
ros2 launch origincar_base origincar_bringup.launch.py

# Step 2: Set camera type
export CAM_TYPE=mipi  # For USB camera: export CAM_TYPE=usb

# Step 3: Copy model config (if needed)
cp -r /opt/tros/lib/mono2d_body_detection/config/ .

# Step 4: Start body detection
ros2 launch mono2d_body_detection mono2d_body_detection.launch.py

# Step 5: Start human tracking
ros2 launch origincar_body_tracking body_tracking.launch.py
```

### QR Code Recognition

```bash
ros2 run origincar_qr li4
```

### AI Image Analysis

```bash
ros2 run image_upload_analyzer image_upload_analyzer
```

## ROS2 Topics

| Topic | Type | Description |
|-------|------|-------------|
| `/cmd_vel` | `geometry_msgs/Twist` | Velocity control command |
| `/odom` | `nav_msgs/Odometry` | Odometry data |
| `/imu/data_raw` | `sensor_msgs/Imu` | IMU raw data |
| `/origincar_competition` | `ai_msgs/PerceptionTargets` | YOLO detection results |
| `/hobot_mono2d_body_detection` | `ai_msgs/PerceptionTargets` | Body detection results |
| `/sign` | `std_msgs/String` | QR code recognition result |
| `/PowerVoltage` | `std_msgs/Float32` | Battery voltage |

## Serial Communication Protocol

**Frame Format**:
- Header: `0x7B`
- Tail: `0x7D`
- Baud Rate: `115200 bps`

## Configuration

### YOLO Model Config

Edit `config/yolov5sconfig.json`:
```json
{
    "model_file": "<model_path>",
    "class_num": 2,
    "cls_names_list": "config/origincar.list"
}
```

### Body Tracking Parameters

Key parameters in `origincar_body_tracking`:
- `activate_robot_rotate_thr`: Rotation activation threshold
- `move_step`: Movement step size
- `track_serial_lost_num_thr`: Lost frame threshold

## Troubleshooting

**Q: Robot not moving?**
- Check if body detection is running: `ros2 topic list | grep body`
- Expected topic: `/hobot_mono2d_body_detection`

**Q: Serial port not found?**
- Verify connection: `ls /dev/ttyACM*`
- Check permission: `sudo chmod 666 /dev/ttyACM0`

**Q: YOLO detection not working?**
- Verify model file path in config
- Check camera stream: `ros2 topic hz /image`

**Q: QR code not recognized?**
- Ensure `pyzbar` is installed
- Check image quality and lighting

## License

Apache License 2.0

## Acknowledgments

- [Horizon Robotics](https://developer.horizon.ai) - TROS framework & BPU acceleration
- [ROS2](https://docs.ros.org) - Robot Operating System
- [YOLOv5](https://github.com/ultralytics/yolov5) - Object detection model
- [ZBar](http://zbar.sourceforge.net) - QR code decoder

