# CLAUDE.md

This file provides guidance to Claude Code (claude.ai/code) when working with code in this repository.

## Project Overview

ROS2 Humble package for YOLO26 object detection. Uses Ultralytics YOLO for inference and publishes detections as `vision_msgs/Detection2DArray`. Containerized with Docker for deployment.

## Build & Run Commands

```bash
# Build Docker image
docker compose build

# Start YOLO detection node (GPU)
docker compose -f compose.yaml --profile yolo26_ros2 up

# Start USB webcam (separate terminal)
docker compose -f compose.yaml --profile webcam up

# Build ROS2 workspace inside container
source /opt/ros/humble/setup.bash && colcon build --symlink-install
```

## Monitoring & Debug

```bash
# Echo detection results
ros2 topic echo /yolo26/detections

# From container
docker exec -it yolo26_ros2 bash -lc "ros2 topic echo /yolo26/detections"

# View debug image with bounding boxes
rqt_image_view /yolo26/debug_image
```

## Architecture

Single ROS2 node (`Yolo26Node`) with this data flow:

```
/camera/image_raw → [Image Subscription] → [Frame Buffer] → [Timer Loop 15Hz]
                                                                    ↓
                                                           [YOLO Inference]
                                                                    ↓
                                            /yolo26/detections (Detection2DArray)
                                            /yolo26/debug_image (annotated Image)
```

**Key files:**
- `ros2_ws/src/yolo26_ros2/yolo26_ros2/yolo26_node.py` - Main node implementation
- `ros2_ws/src/yolo26_ros2/launch/yolo26.launch.py` - Launch file with parameters
- `ros2_ws/src/yolo26_ros2/config/yolo26.yaml` - Default configuration
- `models/classes.yaml` - Object class definitions
- `compose.yaml` - Docker compose with service profiles

## Launch Parameters

| Parameter | Default | Description |
|-----------|---------|-------------|
| `model` | required | Path to YOLO .pt weights |
| `image` | `/camera/image_raw` | Input image topic |
| `detections` | `/yolo26/detections` | Output detections topic |
| `device` | `0` | GPU device ID or "cpu" |
| `conf` | `0.25` | Confidence threshold |
| `iou` | `0.45` | IoU threshold |
| `rate` | `15.0` | Processing rate (Hz) |
| `transport` | `raw` | Image transport: "raw" or "compressed" |
| `publish_debug` | `true` | Publish debug visualization |

## Docker Configuration

- **Base image:** `ros:humble-ros-base-jammy`
- **GPU:** Enabled via `gpus: all`
- **Network:** Host mode (required for ROS2 DDS)
- **Profiles:** `yolo26_ros2` (detection), `webcam` (v4l2_camera)

## Dependencies

ROS2: `rclpy`, `sensor_msgs`, `vision_msgs`, `cv_bridge`, `image_transport`
Python: `ultralytics` (YOLO library)
