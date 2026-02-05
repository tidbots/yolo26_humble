# CLAUDE.md

This file provides guidance to Claude Code (claude.ai/code) when working with code in this repository.

## Project Overview

ROS2 Humble package for YOLO26 object detection. Uses Ultralytics YOLO for inference and publishes detections as `vision_msgs/Detection2DArray`. Containerized with Docker for deployment.

## Build & Run Commands

```bash
# Build Docker images (with profiles)
docker compose -f compose.yaml --profile webcam --profile yolo --profile yolo-tracking build

# Start webcam + YOLO (tracking無効、15Hz)
xhost +local:docker  # Allow X11 access for OpenCV debug window
docker compose -f compose.yaml --profile webcam --profile yolo up

# Start webcam + YOLO with tracking (ByteTrack、30Hz)
xhost +local:docker
docker compose -f compose.yaml --profile webcam --profile yolo-tracking up

# Start USB webcam only
docker compose -f compose.yaml --profile webcam up

# Build ROS2 workspace inside container
source /opt/ros/humble/setup.bash && colcon build --symlink-install
```

## Running with Custom Parameters

```bash
# Run YOLO with tracking enabled at 30Hz (without entering container)
docker exec yolo26_ros2 bash -lc "source /ros2_ws/install/setup.bash && ros2 launch yolo26_ros2 yolo26.launch.py model:=/models/best.pt tracking:=true rate:=30.0"

# Run in background
docker exec -d yolo26_ros2 bash -lc "source /ros2_ws/install/setup.bash && ros2 launch yolo26_ros2 yolo26.launch.py model:=/models/best.pt tracking:=true rate:=30.0"

# Custom confidence threshold
docker exec yolo26_ros2 bash -lc "source /ros2_ws/install/setup.bash && ros2 launch yolo26_ros2 yolo26.launch.py model:=/models/best.pt conf:=0.5"
```

## Monitoring & Debug

```bash
# Echo detection results
ros2 topic echo /yolo26/detections

# From container
docker exec -it yolo26_ros2 bash -lc "ros2 topic echo /yolo26/detections"

# View debug image with bounding boxes (OpenCV window auto-opens when cv_debug_window:=true)
# Or use rqt_image_view:
rqt_image_view /yolo26/debug_image
```

**OpenCV Debug Window:** When `cv_debug_window:=true` (default), an OpenCV window "YOLO26 Debug" displays detection results directly. Requires `xhost +local:docker` before starting containers.

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
- `scripts/webcam_node.py` - OpenCV-based webcam publisher node

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
| `publish_debug` | `true` | Publish debug visualization to topic |
| `cv_debug_window` | `true` | Show OpenCV debug window (requires X11) |
| `tracking` | `false` | Enable ByteTrack object tracking |
| `tracker` | `bytetrack.yaml` | Tracker config: bytetrack.yaml or botsort.yaml |
| `smoothing` | `15` | Moving average window (frames) for bbox/conf smoothing |
| `appear_frames` | `3` | Frames to confirm track appearance (hysteresis) |
| `disappear_frames` | `5` | Frames to confirm track disappearance (hysteresis) |

## Docker Configuration

- **Base image:** `ros:humble-ros-base-jammy`
- **GPU:** Enabled via `gpus: all`
- **Network:** Host mode (required for ROS2 DDS)
- **IPC:** Host mode (required for DDS shared memory transport between containers)
- **X11 Forwarding:** Enabled for OpenCV debug window (`DISPLAY`, `/tmp/.X11-unix`)
- **Profiles:** `yolo` (detection 15Hz), `yolo-tracking` (ByteTrack 30Hz), `webcam` (OpenCV camera node)

## Dependencies

ROS2: `rclpy`, `sensor_msgs`, `vision_msgs`, `cv_bridge`, `image_transport`
Python: `ultralytics` (YOLO library)
