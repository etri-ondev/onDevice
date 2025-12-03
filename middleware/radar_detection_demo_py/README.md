# Radar Detection Demo

This package provides a ROS2-based middleware demonstration for radar detection data.
It includes nodes for publishing data from Excel files, bridging UDP traffic to ROS2 topics, and subscribing to radar data.

## Nodes

### 1. excel_publisher
- Reads radar detection data from a CSV/Excel file.
- Publishes data to the `radar/table` topic.
- Supports UDP fallback when the Ethernet link is down.

### 2. udp2ros_bridge
- Listens for UDP packets containing radar data.
- Converts JSON-formatted UDP packets into ROS2 `RadarDetection` messages.
- Publishes to the `radar/table` topic.

### 3. radar_subscriber
- Subscribes to the `radar/table` topic.
- Logs received detection data to the console for verification.

## Installation

### Prerequisites
- ROS 2 Humble
- Python 3.10+
- Python packages: `pandas`, `openpyxl`, `netifaces` (optional)

### Build
```bash
cd ~/ros2_ws
colcon build --packages-select radar_detection_demo_py
source install/setup.bash
```

## Usage

### Run Publisher
```bash
ros2 run radar_detection_demo_py excel_publisher
```

### Run Subscriber

```bash
ros2 run radar_detection_demo_py radar_subscriber
```

### Run UDP Bridge

```bash
ros2 run radar_detection_demo_py udp2ros_bridge
```
