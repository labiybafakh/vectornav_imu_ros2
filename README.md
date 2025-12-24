# VectorNav IMU ROS2 Package

A ROS2 package for publishing `sensor_msgs/Imu` messages from VectorNav IMU sensors using the VectorNav SDK.

## Features

- Publishes standard `sensor_msgs/Imu` messages
- Configurable via ROS2 parameters
- Auto-connects to VectorNav sensors
- Supports multiple VectorNav sensor models

## Dependencies

- ROS2 (Humble or later)
- VectorNav SDK (included in parent directory)
- rclcpp
- sensor_msgs
- geometry_msgs
- tf2

## Installation

1. Place the VectorNav SDK in your third-party folder
2. Copy this package to your ROS2 workspace
3. Build with colcon:

```bash
colcon build --packages-select vectornav_imu_ros2
```

## Usage

### Basic Launch

```bash
ros2 launch vectornav_imu_ros2 vectornav_imu.launch.py
```

### Custom Parameters

```bash
ros2 launch vectornav_imu_ros2 vectornav_imu.launch.py port:=/dev/ttyUSB1 baud_rate:=230400
```

### Available Parameters

- `port`: Serial port (default: `/dev/ttyUSB0`)
- `baud_rate`: Baud rate (default: `115200`)
- `frame_id`: TF frame ID (default: `imu_link`)
- `publish_rate`: Publishing rate in Hz (default: `100.0`)

## Published Topics

- `/imu/data` (`sensor_msgs/Imu`): IMU data with orientation, angular velocity, and linear acceleration

## Configuration

Edit `config/vectornav_imu_params.yaml` for persistent parameter configuration.

## Hardware Setup

1. Connect VectorNav sensor via USB or serial
2. Ensure proper permissions: `sudo usermod -a -G dialout $USER`
3. Verify port: `ls /dev/ttyUSB*`

## Troubleshooting

- Check serial port permissions
- Verify sensor connection and power
- Confirm baud rate matches sensor configuration
- Check ROS2 logs for connection errors