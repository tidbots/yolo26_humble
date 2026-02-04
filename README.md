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
- Docker containerized (GPU support)
- Webcam input support (v4l2_camera)

## Quick Start

```bash
git clone https://github.com/tidbots/yolo26_humble.git
cd yolo26_humble
docker compose build
```

## Usage

### 1) Start YOLO node (GPU)
```bash
docker compose -f compose.yaml --profile yolo up
```

### 2) Start USB webcam (/dev/video0)
```bash
docker compose -f compose.yaml --profile webcam up
```

### 3) With tracking enabled (recommended)
```bash
# Inside container or modify compose.yaml
ros2 launch yolo26_ros2 yolo26.launch.py \
  model:=/models/best.pt \
  tracking:=true \
  rate:=30.0
```

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
| `tracking` | `false` | Enable ByteTrack object tracking |
| `tracker` | `bytetrack.yaml` | Tracker: bytetrack.yaml or botsort.yaml |
| `smoothing` | `15` | Moving average window (frames) |
| `appear_frames` | `3` | Frames to confirm appearance |
| `disappear_frames` | `5` | Frames to confirm disappearance |

### Camera Parameters

| Parameter | Default | Description |
|-----------|---------|-------------|
| `use_camera` | `false` | Launch v4l2_camera node |
| `video_device` | `/dev/video0` | Video device path |
| `pixel_format` | `YUYV` | Camera pixel format |

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

### Debug image (rqt)

Host side:
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
