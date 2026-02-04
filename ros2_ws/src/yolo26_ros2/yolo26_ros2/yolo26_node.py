#!/usr/bin/env python3
from __future__ import annotations

import os
import threading
import time
from collections import deque, Counter
from dataclasses import dataclass, field
from typing import Any, Dict, List, Optional, Tuple, Union

import numpy as np
import rclpy
from rclpy.node import Node

from sensor_msgs.msg import Image, CompressedImage
from std_msgs.msg import Header
from vision_msgs.msg import Detection2DArray, Detection2D, ObjectHypothesisWithPose, BoundingBox2D
from geometry_msgs.msg import Pose2D
from cv_bridge import CvBridge

import yaml
import cv2


@dataclass
class TrackState:
    """State for a tracked object with smoothing and hysteresis."""
    track_id: int
    smoothing_window: int
    # Bbox history: (cx, cy, w, h)
    bbox_history: deque = field(default_factory=deque)
    # Confidence history
    conf_history: deque = field(default_factory=deque)
    # Class history
    class_history: deque = field(default_factory=deque)
    # Hysteresis counters
    appear_count: int = 0
    disappear_count: int = 0
    # Whether this track is "confirmed" (passed appear threshold)
    confirmed: bool = False
    # Locked class (once confirmed, use most frequent class)
    locked_class: Optional[int] = None

    def __post_init__(self):
        self.bbox_history = deque(maxlen=self.smoothing_window)
        self.conf_history = deque(maxlen=self.smoothing_window)
        self.class_history = deque(maxlen=self.smoothing_window)

    def update(self, cx: float, cy: float, w: float, h: float, conf: float, cls: int) -> None:
        """Add new detection data."""
        self.bbox_history.append((cx, cy, w, h))
        self.conf_history.append(conf)
        self.class_history.append(cls)
        self.disappear_count = 0  # Reset disappear counter on detection

    def get_smoothed_bbox(self) -> Tuple[float, float, float, float]:
        """Get moving average of bbox."""
        if len(self.bbox_history) == 0:
            return 0.0, 0.0, 0.0, 0.0
        arr = np.array(self.bbox_history)
        return float(arr[:, 0].mean()), float(arr[:, 1].mean()), float(arr[:, 2].mean()), float(arr[:, 3].mean())

    def get_smoothed_conf(self) -> float:
        """Get moving average of confidence."""
        if len(self.conf_history) == 0:
            return 0.0
        return float(np.mean(self.conf_history))

    def get_stable_class(self) -> int:
        """Get most frequent class (mode) or locked class if confirmed."""
        if self.locked_class is not None:
            return self.locked_class
        if len(self.class_history) == 0:
            return -1
        counter = Counter(self.class_history)
        return counter.most_common(1)[0][0]

    def lock_class(self) -> None:
        """Lock the class to current most frequent."""
        if len(self.class_history) > 0:
            counter = Counter(self.class_history)
            self.locked_class = counter.most_common(1)[0][0]


def load_classes_yaml(path: str) -> Optional[Dict[int, str]]:
    if not path:
        return None
    if not os.path.exists(path):
        return None

    with open(path, "r", encoding="utf-8") as f:
        data = yaml.safe_load(f)

    # 形式1: {0: person, 1: bottle, ...}
    if isinstance(data, dict):
        out: Dict[int, str] = {}
        for k, v in data.items():
            try:
                ki = int(k)
            except Exception:
                continue
            if isinstance(v, str):
                out[ki] = v
        return out if out else None

    # 形式2: ["person", "bottle", ...]
    if isinstance(data, list):
        out = {i: str(name) for i, name in enumerate(data)}
        return out if out else None

    return None


class Yolo26Node(Node):
    def __init__(self) -> None:
        super().__init__("yolo26_node")

        # ---- params ----
        self.declare_parameter("image_topic", "/camera/image_raw")
        self.declare_parameter("detections_topic", "/yolo26/detections")
        self.declare_parameter("debug_image_topic", "/yolo26/debug_image")
        self.declare_parameter("image_transport", "raw")  # raw / compressed
        self.declare_parameter("publish_debug_image", True)

        self.declare_parameter("model_path", "")
        self.declare_parameter("device", "0")
        self.declare_parameter("conf_thres", 0.25)
        self.declare_parameter("iou_thres", 0.45)
        self.declare_parameter("max_det", 300)
        self.declare_parameter("half", False)

        self.declare_parameter("classes_yaml", "")
        self.declare_parameter("use_class_names", True)

        self.declare_parameter("process_rate_hz", 15.0)

        # Tracking parameters
        self.declare_parameter("enable_tracking", False)
        self.declare_parameter("tracker", "bytetrack.yaml")
        self.declare_parameter("smoothing_window", 15)  # frames for moving average (~1sec at 15Hz)
        # Hysteresis parameters
        self.declare_parameter("appear_frames", 3)  # frames to confirm appearance
        self.declare_parameter("disappear_frames", 5)  # frames to confirm disappearance

        self.image_topic: str = self.get_parameter("image_topic").get_parameter_value().string_value
        self.detections_topic: str = self.get_parameter("detections_topic").get_parameter_value().string_value
        self.debug_image_topic: str = self.get_parameter("debug_image_topic").get_parameter_value().string_value
        self.image_transport: str = self.get_parameter("image_transport").get_parameter_value().string_value
        self.publish_debug_image: bool = self.get_parameter("publish_debug_image").get_parameter_value().bool_value

        self.model_path: str = self.get_parameter("model_path").get_parameter_value().string_value
        self.device: str = self.get_parameter("device").get_parameter_value().string_value
        self.conf_thres: float = float(self.get_parameter("conf_thres").value)
        self.iou_thres: float = float(self.get_parameter("iou_thres").value)
        self.max_det: int = int(self.get_parameter("max_det").value)
        self.half: bool = bool(self.get_parameter("half").value)

        self.classes_yaml: str = self.get_parameter("classes_yaml").get_parameter_value().string_value
        self.use_class_names: bool = bool(self.get_parameter("use_class_names").value)

        self.process_rate_hz: float = float(self.get_parameter("process_rate_hz").value)
        self.process_period_s: float = 1.0 / max(self.process_rate_hz, 0.1)

        # Tracking parameters
        self.enable_tracking: bool = bool(self.get_parameter("enable_tracking").value)
        self.tracker: str = self.get_parameter("tracker").get_parameter_value().string_value
        self.smoothing_window: int = int(self.get_parameter("smoothing_window").value)
        self.appear_frames: int = int(self.get_parameter("appear_frames").value)
        self.disappear_frames: int = int(self.get_parameter("disappear_frames").value)

        # Track states: track_id -> TrackState
        self._track_states: Dict[int, TrackState] = {}

        self.bridge = CvBridge()

        # ---- class names ----
        self.class_map: Optional[Dict[int, str]] = load_classes_yaml(self.classes_yaml)

        # ---- model ----
        self.model = self._load_model(self.model_path)

        # ---- pubs/subs ----
        self.pub_det = self.create_publisher(Detection2DArray, self.detections_topic, 10)
        self.pub_dbg = self.create_publisher(Image, self.debug_image_topic, 10) if self.publish_debug_image else None

        self._lock = threading.Lock()
        self._latest_frame: Optional[np.ndarray] = None
        self._latest_header: Optional[Header] = None
        self._last_infer_ts = 0.0

        # Subscribe raw or compressed
        if self.image_transport.lower() == "compressed":
            topic = self.image_topic.rstrip("/") + "/compressed"
            self.sub_img = self.create_subscription(CompressedImage, topic, self._cb_compressed, 10)
            self.get_logger().info(f"Subscribe: {topic} (sensor_msgs/CompressedImage)")
        else:
            self.sub_img = self.create_subscription(Image, self.image_topic, self._cb_raw, 10)
            self.get_logger().info(f"Subscribe: {self.image_topic} (sensor_msgs/Image)")

        # Timer inference loop
        self.timer = self.create_timer(self.process_period_s, self._on_timer)

        tracking_info = f"rate={self.process_rate_hz}Hz, tracking={self.enable_tracking}"
        if self.enable_tracking:
            tracking_info += f" (tracker={self.tracker}, smoothing={self.smoothing_window}, appear={self.appear_frames}, disappear={self.disappear_frames})"
        self.get_logger().info(f"yolo26_ros2 node started. {tracking_info}")

    def _load_model(self, model_path: str):
        if not model_path:
            self.get_logger().error("Parameter 'model_path' is empty. Set model:=/models/best.pt in launch/compose.")
            return None
        if not os.path.exists(model_path):
            self.get_logger().error(f"Model file not found: {model_path}")
            return None

        try:
            from ultralytics import YOLO  # type: ignore
        except Exception as e:
            self.get_logger().error(
                "Failed to import ultralytics. Install it in the container: pip3 install ultralytics. "
                f"Error: {e}"
            )
            return None

        try:
            model = YOLO(model_path)
            # If no classes_yaml given, try to use model.names
            if self.class_map is None:
                try:
                    names = getattr(model, "names", None)
                    if isinstance(names, dict):
                        self.class_map = {int(k): str(v) for k, v in names.items()}
                    elif isinstance(names, list):
                        self.class_map = {i: str(v) for i, v in enumerate(names)}
                except Exception:
                    pass

            self.get_logger().info(f"Loaded model: {model_path}")
            return model
        except Exception as e:
            self.get_logger().error(f"Failed to load YOLO model: {e}")
            return None

    def _cb_raw(self, msg: Image) -> None:
        try:
            cv_img = self.bridge.imgmsg_to_cv2(msg, desired_encoding="bgr8")
        except Exception as e:
            self.get_logger().warn(f"cv_bridge imgmsg_to_cv2 failed: {e}")
            return
        with self._lock:
            self._latest_frame = cv_img
            self._latest_header = msg.header

    def _cb_compressed(self, msg: CompressedImage) -> None:
        try:
            np_arr = np.frombuffer(msg.data, dtype=np.uint8)
            cv_img = cv2.imdecode(np_arr, cv2.IMREAD_COLOR)
            if cv_img is None:
                raise RuntimeError("cv2.imdecode returned None")
        except Exception as e:
            self.get_logger().warn(f"decode compressed image failed: {e}")
            return
        # CompressedImage header exists
        with self._lock:
            self._latest_frame = cv_img
            self._latest_header = msg.header

    def _on_timer(self) -> None:
        # throttle if no frame
        with self._lock:
            frame = None if self._latest_frame is None else self._latest_frame.copy()
            header = self._latest_header

        if frame is None or header is None:
            return

        now = time.time()
        # avoid reprocessing too fast if timer overlaps
        if now - self._last_infer_ts < (self.process_period_s * 0.5):
            return
        self._last_infer_ts = now

        if self.model is None:
            # model not loaded; keep alive
            return

        det_msg, dbg_img = self._infer_and_make_msgs(frame, header)
        if det_msg is not None:
            self.pub_det.publish(det_msg)
        if self.pub_dbg is not None and dbg_img is not None:
            try:
                self.pub_dbg.publish(self.bridge.cv2_to_imgmsg(dbg_img, encoding="bgr8"))
            except Exception as e:
                self.get_logger().warn(f"publish debug image failed: {e}")

    def _class_id_to_label(self, cls_id: int) -> str:
        if self.use_class_names and self.class_map and cls_id in self.class_map:
            return self.class_map[cls_id]
        return str(cls_id)

    def _get_or_create_track(self, track_id: int) -> TrackState:
        """Get existing track state or create new one."""
        if track_id not in self._track_states:
            self._track_states[track_id] = TrackState(
                track_id=track_id,
                smoothing_window=self.smoothing_window
            )
        return self._track_states[track_id]

    def _update_track_hysteresis(self, active_ids: set) -> None:
        """Update hysteresis counters for all tracks."""
        stale_ids = []
        for tid, state in self._track_states.items():
            if tid not in active_ids:
                # Track not detected this frame
                state.disappear_count += 1
                if state.disappear_count >= self.disappear_frames:
                    stale_ids.append(tid)
            else:
                # Track detected - update appear counter
                state.appear_count += 1
                if state.appear_count >= self.appear_frames and not state.confirmed:
                    state.confirmed = True
                    state.lock_class()  # Lock class once confirmed

        # Remove tracks that have disappeared for too long
        for tid in stale_ids:
            del self._track_states[tid]

    def _draw_stabilized_detections(self, frame: np.ndarray, detections: List[Tuple]) -> np.ndarray:
        """Draw stabilized detections on frame.

        Args:
            frame: BGR image
            detections: List of (track_id, cx, cy, w, h, conf, class_label)
        """
        img = frame.copy()
        for track_id, cx, cy, w, h, conf, class_label in detections:
            x1 = int(cx - w / 2)
            y1 = int(cy - h / 2)
            x2 = int(cx + w / 2)
            y2 = int(cy + h / 2)

            # Color based on track_id
            color = self._get_track_color(track_id)

            # Draw bounding box
            cv2.rectangle(img, (x1, y1), (x2, y2), color, 2)

            # Label
            label = f"ID:{track_id} {class_label} {conf:.2f}"
            (tw, th), _ = cv2.getTextSize(label, cv2.FONT_HERSHEY_SIMPLEX, 0.6, 1)
            cv2.rectangle(img, (x1, y1 - th - 10), (x1 + tw, y1), color, -1)
            cv2.putText(img, label, (x1, y1 - 5), cv2.FONT_HERSHEY_SIMPLEX, 0.6, (255, 255, 255), 1)

        return img

    def _get_track_color(self, track_id: int) -> Tuple[int, int, int]:
        """Generate consistent color for track ID."""
        np.random.seed(track_id * 13 + 7)
        return tuple(int(c) for c in np.random.randint(50, 255, 3))

    def _infer_and_make_msgs(self, frame_bgr: np.ndarray, header: Header) -> Tuple[Optional[Detection2DArray], Optional[np.ndarray]]:
        # Ultralytics expects numpy image; BGR is OK for its internal pipeline
        try:
            if self.enable_tracking:
                # Use ByteTrack for tracking with persist=True
                results = self.model.track(
                    source=frame_bgr,
                    device=self.device,
                    conf=self.conf_thres,
                    iou=self.iou_thres,
                    max_det=self.max_det,
                    half=self.half,
                    verbose=False,
                    tracker=self.tracker,
                    persist=True,
                )
            else:
                results = self.model.predict(
                    source=frame_bgr,
                    device=self.device,
                    conf=self.conf_thres,
                    iou=self.iou_thres,
                    max_det=self.max_det,
                    half=self.half,
                    verbose=False,
                )
        except Exception as e:
            self.get_logger().warn(f"model inference failed: {e}")
            return None, None

        if not results:
            det_arr = Detection2DArray()
            det_arr.header = header
            return det_arr, (frame_bgr if not self.publish_debug_image else frame_bgr)

        r0 = results[0]

        det_arr = Detection2DArray()
        det_arr.header = header

        # boxes
        try:
            boxes = getattr(r0, "boxes", None)
            if boxes is None:
                dbg_img = frame_bgr if self.publish_debug_image else None
                return det_arr, dbg_img

            xyxy = boxes.xyxy.cpu().numpy()  # (N,4)
            conf = boxes.conf.cpu().numpy()  # (N,)
            cls = boxes.cls.cpu().numpy().astype(int)  # (N,)

            # Get track IDs if tracking is enabled
            track_ids = None
            if self.enable_tracking and boxes.id is not None:
                track_ids = boxes.id.cpu().numpy().astype(int)
        except Exception as e:
            self.get_logger().warn(f"extract boxes failed: {e}")
            dbg_img = frame_bgr if self.publish_debug_image else None
            return det_arr, dbg_img

        active_track_ids = set()
        stabilized_detections = []  # For custom debug rendering

        for i, ((x1, y1, x2, y2), score, cid) in enumerate(zip(xyxy, conf, cls)):
            x1f, y1f, x2f, y2f = float(x1), float(y1), float(x2), float(y2)
            w = max(0.0, x2f - x1f)
            h = max(0.0, y2f - y1f)
            cx = x1f + w / 2.0
            cy = y1f + h / 2.0

            track_id = -1
            final_cx, final_cy, final_w, final_h = cx, cy, w, h
            final_conf = float(score)
            final_class = int(cid)

            if self.enable_tracking and track_ids is not None:
                track_id = int(track_ids[i])
                active_track_ids.add(track_id)

                # Get or create track state
                state = self._get_or_create_track(track_id)
                state.update(cx, cy, w, h, float(score), int(cid))

                # Apply smoothing
                final_cx, final_cy, final_w, final_h = state.get_smoothed_bbox()
                final_conf = state.get_smoothed_conf()
                final_class = state.get_stable_class()

                # Skip if not yet confirmed (hysteresis)
                if not state.confirmed:
                    continue

            det = Detection2D()
            det.header = header
            det.id = str(track_id) if track_id >= 0 else ""

            bbox = BoundingBox2D()
            bbox.center = Pose2D(x=final_cx, y=final_cy, theta=0.0)
            bbox.size_x = final_w
            bbox.size_y = final_h
            det.bbox = bbox

            hyp = ObjectHypothesisWithPose()
            hyp.hypothesis.class_id = self._class_id_to_label(final_class)
            hyp.hypothesis.score = final_conf
            det.results.append(hyp)

            det_arr.detections.append(det)

            # Store for debug rendering
            if self.publish_debug_image:
                class_label = self._class_id_to_label(final_class)
                stabilized_detections.append((track_id, final_cx, final_cy, final_w, final_h, final_conf, class_label))

        # Update hysteresis for all tracks
        if self.enable_tracking:
            self._update_track_hysteresis(active_track_ids)

        # Debug image
        dbg_img = None
        if self.publish_debug_image:
            if self.enable_tracking:
                # Use custom stabilized rendering
                dbg_img = self._draw_stabilized_detections(frame_bgr, stabilized_detections)
            else:
                # Use ultralytics default rendering
                try:
                    dbg_img = r0.plot()
                except Exception:
                    dbg_img = frame_bgr

        return det_arr, dbg_img


def main() -> None:
    rclpy.init()
    node = Yolo26Node()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
