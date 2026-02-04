#!/usr/bin/env python3
from __future__ import annotations

import os
import threading
import time
from collections import deque
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

        # Smoothing buffer: track_id -> deque of (cx, cy, w, h)
        self._smoothing_buffer: Dict[int, deque] = {}

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

        tracking_info = f"tracking={self.enable_tracking}"
        if self.enable_tracking:
            tracking_info += f" (tracker={self.tracker}, smoothing={self.smoothing_window} frames)"
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

    def _smooth_bbox(self, track_id: int, cx: float, cy: float, w: float, h: float) -> Tuple[float, float, float, float]:
        """Apply moving average smoothing to bounding box coordinates."""
        if track_id not in self._smoothing_buffer:
            self._smoothing_buffer[track_id] = deque(maxlen=self.smoothing_window)

        self._smoothing_buffer[track_id].append((cx, cy, w, h))

        buf = self._smoothing_buffer[track_id]
        if len(buf) == 1:
            return cx, cy, w, h

        arr = np.array(buf)
        return float(arr[:, 0].mean()), float(arr[:, 1].mean()), float(arr[:, 2].mean()), float(arr[:, 3].mean())

    def _cleanup_smoothing_buffer(self, active_ids: set) -> None:
        """Remove stale track IDs from smoothing buffer."""
        stale_ids = set(self._smoothing_buffer.keys()) - active_ids
        for tid in stale_ids:
            del self._smoothing_buffer[tid]

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

        # debug image
        dbg_img = None
        if self.publish_debug_image:
            try:
                dbg_img = r0.plot()  # annotated BGR
            except Exception:
                dbg_img = frame_bgr

        # boxes
        try:
            boxes = getattr(r0, "boxes", None)
            if boxes is None:
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
            return det_arr, dbg_img

        active_track_ids = set()

        for i, ((x1, y1, x2, y2), score, cid) in enumerate(zip(xyxy, conf, cls)):
            x1f, y1f, x2f, y2f = float(x1), float(y1), float(x2), float(y2)
            w = max(0.0, x2f - x1f)
            h = max(0.0, y2f - y1f)
            cx = x1f + w / 2.0
            cy = y1f + h / 2.0

            # Apply smoothing if tracking is enabled
            track_id = -1
            if self.enable_tracking and track_ids is not None:
                track_id = int(track_ids[i])
                active_track_ids.add(track_id)
                cx, cy, w, h = self._smooth_bbox(track_id, cx, cy, w, h)

            det = Detection2D()
            det.header = header
            det.id = str(track_id) if track_id >= 0 else ""

            bbox = BoundingBox2D()
            bbox.center = Pose2D(x=cx, y=cy, theta=0.0)
            bbox.size_x = w
            bbox.size_y = h
            det.bbox = bbox

            hyp = ObjectHypothesisWithPose()
            hyp.hypothesis.class_id = self._class_id_to_label(int(cid))
            hyp.hypothesis.score = float(score)
            # poseは不明なのでデフォルト（0）でOK
            det.results.append(hyp)

            det_arr.detections.append(det)

        # Cleanup stale track IDs from smoothing buffer
        if self.enable_tracking:
            self._cleanup_smoothing_buffer(active_track_ids)

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
