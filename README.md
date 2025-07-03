# GoPiGo3 ROS 2 Package

A ROS 2 package for controlling the GoPiGo3 robot using Twist messages. This package provides a bridge between ROS 2's standard geometry messages and the GoPiGo3 robot hardware.

**Note: This was a test implementation for the bachelor thesis "Autonomous Robot for Plant Management and Energy Efficiency on Photovoltaic Green Rooftops". The attempt to use the GoPiGo3 header as a motor driver failed due to compatibility issues with the chosen OS system and hardware components.**

## Overview
- **Version:** 0.1.0
- **License:** Apache-2.0

This package contains a ROS 2 node that subscribes to `cmd_vel` topic and translates velocity commands into GoPiGo3 motor controls.

## Dependencies
- ROS 2
- rclpy (ROS 2 Python client library)
- geometry_msgs (ROS 2 standard geometry messages)
- easygopigo3 (GoPiGo3 Python library)

## Directory Structure

- **`package.xml`** - ROS 2 package manifest with dependencies and metadata
- **`setup.py`** - Python package setup with entry points
- **`setup.cfg`** - Python package configuration

### `gopigo3_ros/`
Main package source code:
- **`gopigo3_controller.py`** - ROS 2 node that converts cmd_vel messages to GoPiGo3 motor commands

### `resource/`
- **`gopigo3_ros`** - ROS 2 package marker file

### `test/`
Code quality tests:
- **`test_copyright.py`** - Copyright header validation
- **`test_flake8.py`** - Python style checking  
- **`test_pep257.py`** - Docstring compliance

## Quick Start

**Build:**
```bash
colcon build --packages-select gopigo3_ros
```

**Run:**
```bash
ros2 run gopigo3_ros gopigo3_controller
```

**Control:**
```bash
ros2 topic pub /cmd_vel geometry_msgs/Twist "linear: {x: 0.5}, angular: {z: 0.0}"
```

## Technical Details
- **Subscribed Topics:** `/cmd_vel` (geometry_msgs/Twist)
- **Dependencies:** rclpy, geometry_msgs, easygopigo3
- **Scaling:** Linear velocity × 100, Angular velocity × 50
- **Limitations:** Hardware compatibility issues prevented successful deployment
