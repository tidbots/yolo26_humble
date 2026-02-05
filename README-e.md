# yolo26_humble

ROS2 Humble package for YOLO26 object detection with ByteTrack tracking and detection stabilization.

## Features

- YOLO26 object detection (Ultralytics)
- ByteTrack / BoTSORT object tracking
- Detection stabilization:
  - Bounding box smoothing (moving average)
  - Confidence smoothing
  - Class prediction stabilization (mode lock)
  - Hysteresis (appear/disappear thresholds)
- OpenCV debug window display
- Docker containerized (GPU support)
- Webcam input support (OpenCV + V4L2)

## Quick Start

```bash
git clone https://github.com/tidbots/yolo26_humble.git
cd yolo26_humble

# Build Docker images
docker compose -f compose.yaml --profile webcam --profile yolo --profile yolo-tracking build

# Allow X11 access for OpenCV debug window
xhost +local:docker

# Start webcam + YOLO (without tracking, 15Hz)
docker compose -f compose.yaml --profile webcam --profile yolo up

# Or with tracking enabled (ByteTrack, 30Hz)
docker compose -f compose.yaml --profile webcam --profile yolo-tracking up
```

## Docker Profiles

| Profile | Description |
|---------|-------------|
| `webcam` | USB camera node (OpenCV + V4L2) |
| `yolo` | YOLO detection (tracking disabled, 15Hz) |
| `yolo-tracking` | YOLO detection with ByteTrack (30Hz) |

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
| `tracker` | `bytetrack.yaml` | Tracker: bytetrack.yaml or botsort.yaml |
| `smoothing` | `15` | Moving average window (frames) |
| `appear_frames` | `3` | Frames to confirm appearance |
| `disappear_frames` | `5` | Frames to confirm disappearance |

## Running with Custom Parameters

```bash
# Run YOLO with tracking enabled at 30Hz (without entering container)
docker exec yolo26_ros2 bash -lc "source /ros2_ws/install/setup.bash && ros2 launch yolo26_ros2 yolo26.launch.py model:=/models/best.pt tracking:=true rate:=30.0"

# Run in background
docker exec -d yolo26_ros2 bash -lc "source /ros2_ws/install/setup.bash && ros2 launch yolo26_ros2 yolo26.launch.py model:=/models/best.pt tracking:=true rate:=30.0"

# Custom confidence threshold
docker exec yolo26_ros2 bash -lc "source /ros2_ws/install/setup.bash && ros2 launch yolo26_ros2 yolo26.launch.py model:=/models/best.pt conf:=0.5"
```

## Monitoring

### Detection results (Detection2DArray)

Host side (if ROS2 installed):
```bash
ros2 topic echo /yolo26/detections
```

From container:
```bash
docker exec -it yolo26_ros2 bash -lc "ros2 topic echo /yolo26/detections"
```

### Debug image

OpenCV debug window "YOLO26 Debug" opens automatically when `cv_debug_window:=true`.

Or use rqt_image_view:
```bash
rqt_image_view /yolo26/debug_image
```

## Model Files

Place models in `/models/`:
- `/models/best.pt` - YOLO weights
- `/models/classes.yaml` - Class names (optional)

Download pre-trained models:
```bash
cd models
wget https://github.com/ultralytics/assets/releases/download/v8.4.0/yolo26n.pt
wget https://github.com/ultralytics/assets/releases/download/v8.4.0/yolo26s.pt
wget https://github.com/ultralytics/assets/releases/download/v8.4.0/yolo26m.pt
wget https://github.com/ultralytics/assets/releases/download/v8.4.0/yolo26l.pt
wget https://github.com/ultralytics/assets/releases/download/v8.4.0/yolo26x.pt
```

## Detection Stabilization

When `tracking:=true`, the following stabilization is applied:

1. **Bounding Box Smoothing**: Moving average over `smoothing` frames
2. **Confidence Smoothing**: Moving average of detection scores
3. **Class Lock**: After track confirmation, class is locked to most frequent
4. **Hysteresis**:
   - New tracks require `appear_frames` consecutive detections
   - Lost tracks persist for `disappear_frames` before removal

This reduces flickering caused by lighting changes and temporary detection failures.

## Architecture

```
/camera/image_raw → [Subscription] → [Frame Buffer] → [Timer Loop]
                                                           ↓
                                                    [YOLO Inference]
                                                           ↓
                                                    [ByteTrack] (optional)
                                                           ↓
                                                    [Stabilization]
                                                           ↓
                                    /yolo26/detections (Detection2DArray)
                                    /yolo26/debug_image (annotated Image)
```

## License

MIT
