#!/usr/bin/env python3
import math
from collections import deque
from concurrent.futures import Future, ThreadPoolExecutor
from threading import Lock
from typing import Optional, Tuple, List, Dict

import cv2
import numpy as np
import rclpy
from cv_bridge import CvBridge
from rclpy.logging import LoggingSeverity
from rclpy.node import Node
from sensor_msgs.msg import CompressedImage, Image, Range

TARGET_TOPICS = [
    "/image_raw",
    "/image_raw/compressed",
    "/image_raw/compressedDepth",
    "/image_raw/theora",
]

FRAME_GOOD = "GOOD"
FRAME_OK = "OK"
FRAME_BAD = "BAD"


class FrameQualityFilter:
    def __init__(self, config: Dict[str, float]):
        self.prev_gray_motion_small: Optional[np.ndarray] = None
        self.motion_history = deque(maxlen=8)
        self.lap_history = deque(maxlen=30)

        self.min_laplacian_var = config["min_laplacian_var"]
        self.min_brightness = config["min_brightness"]
        self.max_brightness = config["max_brightness"]
        self.max_translation_px = config["max_translation_px"]
        self.max_rotation_deg = config["max_rotation_deg"]
        self.max_jitter_std = config["max_jitter_std"]
        self.roi_fraction = config["roi_fraction"]

        self.adaptive_laplacian = bool(config["adaptive_laplacian"])
        self.laplacian_percentile = config["laplacian_percentile"]
        self.min_laplacian_floor = config["min_laplacian_floor"]
        self.min_lap_samples = int(config["min_lap_samples"])
        self.frame_score_threshold = config["frame_score_threshold"]
        self.motion_downscale = max(0.2, min(1.0, config["motion_downscale"]))
        self.min_feature_points = int(config["min_feature_points"])
        self.min_motion_noise_px = float(config["min_motion_noise_px"])

    def _laplacian_var(self, gray: np.ndarray) -> float:
        return float(cv2.Laplacian(gray, cv2.CV_64F).var())

    def _current_laplacian_threshold(self) -> float:
        if not self.adaptive_laplacian or len(self.lap_history) < self.min_lap_samples:
            return self.min_laplacian_var

        adaptive = float(np.percentile(self.lap_history, self.laplacian_percentile))
        return max(self.min_laplacian_floor, min(self.min_laplacian_var, adaptive))

    def _is_blurry(self, gray: np.ndarray) -> Tuple[bool, float, float]:
        lap_var = self._laplacian_var(gray)
        threshold = self._current_laplacian_threshold()
        return lap_var < threshold, lap_var, threshold

    def _is_bad_brightness(self, gray: np.ndarray) -> Tuple[bool, float]:
        mean = float(np.mean(gray))
        return mean < self.min_brightness or mean > self.max_brightness, mean

    def _estimate_global_motion(
        self, prev_gray: np.ndarray, gray: np.ndarray
    ) -> Tuple[float, float]:
        pts_prev = cv2.goodFeaturesToTrack(
            prev_gray,
            maxCorners=200,
            qualityLevel=0.01,
            minDistance=8,
            blockSize=7,
        )
        if pts_prev is None or len(pts_prev) < self.min_feature_points:
            return 0.0, 0.0

        pts_curr, status, _ = cv2.calcOpticalFlowPyrLK(prev_gray, gray, pts_prev, None)
        if pts_curr is None or status is None:
            return 0.0, 0.0

        good_prev = pts_prev[status.flatten() == 1]
        good_curr = pts_curr[status.flatten() == 1]
        if len(good_prev) < self.min_feature_points or len(good_curr) < self.min_feature_points:
            return 0.0, 0.0

        affine, _ = cv2.estimateAffinePartial2D(good_prev, good_curr)
        if affine is None:
            return 0.0, 0.0

        tx = float(affine[0, 2])
        ty = float(affine[1, 2])
        translation = math.hypot(tx, ty)

        rotation_rad = math.atan2(float(affine[1, 0]), float(affine[0, 0]))
        rotation_deg = abs(math.degrees(rotation_rad))
        return translation, rotation_deg

    def center_roi_bounds(self, frame_bgr: np.ndarray) -> Tuple[int, int, int, int]:
        h, w = frame_bgr.shape[:2]
        frac = min(max(self.roi_fraction, 0.2), 1.0)
        roi_h = int(h * frac)
        roi_w = int(w * frac)
        y1 = (h - roi_h) // 2
        x1 = (w - roi_w) // 2
        return x1, y1, roi_w, roi_h

    def center_roi(self, frame_bgr: np.ndarray) -> Tuple[np.ndarray, Tuple[int, int]]:
        x1, y1, roi_w, roi_h = self.center_roi_bounds(frame_bgr)
        return frame_bgr[y1 : y1 + roi_h, x1 : x1 + roi_w], (x1, y1)

    def _frame_score(
        self,
        lap_var: float,
        brightness: float,
        translation: float,
        rotation_deg: float,
        jitter_std: float,
        lap_threshold: float,
    ) -> float:
        blur_score = min(lap_var / max(1.0, lap_threshold * 2.0), 1.0)

        mid = (self.min_brightness + self.max_brightness) / 2.0
        span = max(1.0, (self.max_brightness - self.min_brightness) / 2.0)
        brightness_score = max(0.0, 1.0 - (abs(brightness - mid) / span))

        trans_score = max(0.0, 1.0 - min(translation / max(1e-6, self.max_translation_px), 1.0))
        rot_score = max(0.0, 1.0 - min(rotation_deg / max(1e-6, self.max_rotation_deg), 1.0))
        jitter_score = max(0.0, 1.0 - min(jitter_std / max(1e-6, self.max_jitter_std), 1.0))

        return 0.35 * blur_score + 0.2 * brightness_score + 0.2 * trans_score + 0.15 * rot_score + 0.1 * jitter_score

    def evaluate_frame(self, frame_bgr: np.ndarray) -> Tuple[str, str, Dict[str, float]]:
        if frame_bgr is None or frame_bgr.size == 0:
            return FRAME_BAD, "empty_frame", {}

        h, w = frame_bgr.shape[:2]
        if h < 120 or w < 160:
            return FRAME_BAD, "too_small", {"h": float(h), "w": float(w)}

        roi, _ = self.center_roi(frame_bgr)
        gray_roi = cv2.cvtColor(roi, cv2.COLOR_BGR2GRAY)
        gray_full = cv2.cvtColor(frame_bgr, cv2.COLOR_BGR2GRAY)
        gray_full_small = cv2.resize(
            gray_full,
            None,
            fx=self.motion_downscale,
            fy=self.motion_downscale,
            interpolation=cv2.INTER_AREA,
        )

        metrics: Dict[str, float] = {"h": float(h), "w": float(w)}

        is_blur, lap_var, lap_threshold = self._is_blurry(gray_roi)
        metrics["lap_var"] = lap_var
        metrics["lap_threshold"] = lap_threshold
        if is_blur:
            return FRAME_BAD, "blurry", metrics

        bad_brightness, brightness = self._is_bad_brightness(gray_roi)
        metrics["brightness"] = brightness
        if bad_brightness:
            return FRAME_BAD, "bad_brightness", metrics

        translation = 0.0
        rotation_deg = 0.0
        jitter_std = 0.0

        if self.prev_gray_motion_small is not None:
            translation_small, rotation_deg = self._estimate_global_motion(
                self.prev_gray_motion_small, gray_full_small
            )
            translation = translation_small / self.motion_downscale
            if translation < self.min_motion_noise_px:
                translation = 0.0
            metrics["translation"] = translation
            metrics["rotation_deg"] = rotation_deg

            if translation > self.max_translation_px:
                return FRAME_BAD, "camera_translation", metrics

            if rotation_deg > self.max_rotation_deg:
                return FRAME_BAD, "camera_rotation", metrics

            projected = list(self.motion_history)
            projected.append(translation)
            if len(projected) >= 5:
                jitter_std = float(np.std(projected))
                metrics["jitter_std"] = jitter_std
                if jitter_std > self.max_jitter_std:
                    return FRAME_OK, "camera_vibration_borderline", metrics

        score = self._frame_score(
            lap_var=lap_var,
            brightness=brightness,
            translation=translation,
            rotation_deg=rotation_deg,
            jitter_std=jitter_std,
            lap_threshold=lap_threshold,
        )
        metrics["frame_score"] = score

        if score < self.frame_score_threshold:
            return FRAME_OK, "borderline_score", metrics

        self.motion_history.append(translation)
        self.lap_history.append(lap_var)
        self.prev_gray_motion_small = gray_full_small
        return FRAME_GOOD, "ok", metrics


class YoloRunner:
    def __init__(self, logger, enabled: bool, model_path: str, device: str):
        self.enabled = enabled
        self.logger = logger
        self.model = None

        if not self.enabled:
            return

        try:
            from ultralytics import YOLO  # type: ignore

            self.model = YOLO(model_path)
            if device and str(device).lower() != "auto":
                self.model.to(device)
            self.logger.info(f"YOLO enabled with model: {model_path} device={device}")
        except Exception as exc:
            self.enabled = False
            self.logger.warn(f"YOLO disabled (init failed): {exc}")

    def detect(self, frame_bgr: np.ndarray, roi_offset: Tuple[int, int] = (0, 0)) -> List[dict]:
        if not self.enabled or self.model is None:
            return []

        ox, oy = roi_offset
        results = self.model(frame_bgr, verbose=False)[0]
        detections = []
        for box in results.boxes:
            cls_id = int(box.cls[0])
            conf = float(box.conf[0])
            x1, y1, x2, y2 = box.xyxy[0].tolist()
            detections.append(
                {
                    "class": self.model.names.get(cls_id, str(cls_id)),
                    "conf": conf,
                    "bbox": [x1 + ox, y1 + oy, x2 + ox, y2 + oy],
                }
            )
        return detections


class DetectionStabilizer:
    def __init__(self, iou_threshold: float = 0.4, smooth_alpha: float = 0.4, min_hits: int = 2, max_miss: int = 3):
        self.iou_threshold = iou_threshold
        self.smooth_alpha = smooth_alpha
        self.min_hits = min_hits
        self.max_miss = max_miss
        self.tracks: List[Dict] = []
        self.next_track_id = 1

    @staticmethod
    def _iou(a: List[float], b: List[float]) -> float:
        ax1, ay1, ax2, ay2 = a
        bx1, by1, bx2, by2 = b

        ix1, iy1 = max(ax1, bx1), max(ay1, by1)
        ix2, iy2 = min(ax2, bx2), min(ay2, by2)
        iw, ih = max(0.0, ix2 - ix1), max(0.0, iy2 - iy1)
        inter = iw * ih

        area_a = max(0.0, (ax2 - ax1)) * max(0.0, (ay2 - ay1))
        area_b = max(0.0, (bx2 - bx1)) * max(0.0, (by2 - by1))
        union = area_a + area_b - inter
        if union <= 0:
            return 0.0
        return inter / union

    def update(self, detections: List[dict]) -> List[dict]:
        for track in self.tracks:
            track["misses"] += 1

        for det in detections:
            best_idx = -1
            best_iou = 0.0
            for idx, track in enumerate(self.tracks):
                if track["class"] != det["class"]:
                    continue
                iou = self._iou(track["bbox"], det["bbox"])
                if iou > self.iou_threshold and iou > best_iou:
                    best_iou = iou
                    best_idx = idx

            if best_idx >= 0:
                track = self.tracks[best_idx]
                prev = track["bbox"]
                curr = det["bbox"]
                track["bbox"] = [
                    self.smooth_alpha * prev[0] + (1 - self.smooth_alpha) * curr[0],
                    self.smooth_alpha * prev[1] + (1 - self.smooth_alpha) * curr[1],
                    self.smooth_alpha * prev[2] + (1 - self.smooth_alpha) * curr[2],
                    self.smooth_alpha * prev[3] + (1 - self.smooth_alpha) * curr[3],
                ]
                track["conf"] = det["conf"]
                track["hits"] += 1
                track["misses"] = 0
            else:
                self.tracks.append(
                    {
                        "id": self.next_track_id,
                        "class": det["class"],
                        "bbox": det["bbox"],
                        "conf": det["conf"],
                        "hits": 1,
                        "misses": 0,
                    }
                )
                self.next_track_id += 1

        self.tracks = [t for t in self.tracks if t["misses"] <= self.max_miss]

        stable = []
        for t in self.tracks:
            if t["hits"] >= self.min_hits:
                stable.append(
                    {
                        "id": t["id"],
                        "class": t["class"],
                        "conf": t["conf"],
                        "bbox": t["bbox"],
                    }
                )
        return stable


class MultiTopicImageSubscriber(Node):
    def __init__(self):
        super().__init__("multi_topic_image_subscriber")

        self.declare_parameter("show_debug", False)
        self.declare_parameter("process_interval_ms", 100)
        self.declare_parameter("cooldown_bad_frames", 3)

        self.declare_parameter("enable_yolo", False)
        self.declare_parameter("yolo_model_path", "yolov8n.pt")
        self.declare_parameter("yolo_device", "auto")
        self.declare_parameter("yolo_use_roi", False)
        self.declare_parameter("yolo_interval_ms", 200)
        self.declare_parameter("detection_conf_threshold", 0.5)
        self.declare_parameter("min_box_area_px", 180.0)
        self.declare_parameter("max_center_distance_norm", 1.0)
        self.declare_parameter("center_priority_power", 1.5)

        self.declare_parameter("min_laplacian_var", 45.0)
        self.declare_parameter("min_brightness", 30.0)
        self.declare_parameter("max_brightness", 235.0)
        self.declare_parameter("max_translation_px", 18.0)
        self.declare_parameter("max_rotation_deg", 4.0)
        self.declare_parameter("max_jitter_std", 6.0)
        self.declare_parameter("roi_fraction", 0.6)

        self.declare_parameter("adaptive_laplacian", True)
        self.declare_parameter("laplacian_percentile", 30.0)
        self.declare_parameter("min_laplacian_floor", 25.0)
        self.declare_parameter("min_lap_samples", 10)
        self.declare_parameter("frame_score_threshold", 0.55)
        self.declare_parameter("motion_downscale", 0.5)
        self.declare_parameter("min_feature_points", 6)
        self.declare_parameter("min_motion_noise_px", 1.0)

        self.declare_parameter("enable_ultrasonic_fusion", False)
        self.declare_parameter("ultrasonic_topic", "/ultrasonic")
        self.declare_parameter("ultrasonic_stop_distance_m", 0.25)
        self.declare_parameter("ultrasonic_center_max_norm", 0.4)

        self.show_debug = bool(self.get_parameter("show_debug").value)
        self.process_interval_sec = float(self.get_parameter("process_interval_ms").value) / 1000.0
        self.cooldown_bad_frames = int(self.get_parameter("cooldown_bad_frames").value)

        self.yolo_use_roi = bool(self.get_parameter("yolo_use_roi").value)
        self.yolo_interval_sec = float(self.get_parameter("yolo_interval_ms").value) / 1000.0
        self.detection_conf_threshold = float(self.get_parameter("detection_conf_threshold").value)
        self.min_box_area_px = float(self.get_parameter("min_box_area_px").value)
        self.max_center_distance_norm = float(self.get_parameter("max_center_distance_norm").value)
        self.center_priority_power = float(self.get_parameter("center_priority_power").value)
        self.enable_ultrasonic_fusion = bool(self.get_parameter("enable_ultrasonic_fusion").value)
        self.ultrasonic_stop_distance_m = float(self.get_parameter("ultrasonic_stop_distance_m").value)
        self.ultrasonic_center_max_norm = float(self.get_parameter("ultrasonic_center_max_norm").value)

        filter_config = {
            "min_laplacian_var": float(self.get_parameter("min_laplacian_var").value),
            "min_brightness": float(self.get_parameter("min_brightness").value),
            "max_brightness": float(self.get_parameter("max_brightness").value),
            "max_translation_px": float(self.get_parameter("max_translation_px").value),
            "max_rotation_deg": float(self.get_parameter("max_rotation_deg").value),
            "max_jitter_std": float(self.get_parameter("max_jitter_std").value),
            "roi_fraction": float(self.get_parameter("roi_fraction").value),
            "adaptive_laplacian": bool(self.get_parameter("adaptive_laplacian").value),
            "laplacian_percentile": float(self.get_parameter("laplacian_percentile").value),
            "min_laplacian_floor": float(self.get_parameter("min_laplacian_floor").value),
            "min_lap_samples": float(self.get_parameter("min_lap_samples").value),
            "frame_score_threshold": float(self.get_parameter("frame_score_threshold").value),
            "motion_downscale": float(self.get_parameter("motion_downscale").value),
            "min_feature_points": int(self.get_parameter("min_feature_points").value),
            "min_motion_noise_px": float(self.get_parameter("min_motion_noise_px").value),
        }

        self.bridge = CvBridge()
        self.filter = FrameQualityFilter(filter_config)
        self.yolo = YoloRunner(
            self.get_logger(),
            bool(self.get_parameter("enable_yolo").value),
            str(self.get_parameter("yolo_model_path").value),
            str(self.get_parameter("yolo_device").value),
        )
        self.det_stabilizer = DetectionStabilizer()

        self.subscriptions = {}
        self.accepted_count = 0
        self.rejected_count = 0
        self.cooldown_remaining = 0
        self.skip_counter = 0
        self.last_process_time = 0.0
        self.fps_window_count = 0
        self.fps_window_start = self.get_clock().now().nanoseconds / 1e9
        self.last_fused_stop = False
        self.latest_ultrasonic_m: Optional[float] = None

        self.yolo_executor: Optional[ThreadPoolExecutor] = None
        self.pending_yolo_future: Optional[Future] = None
        self.latest_detections: List[dict] = []
        self.latest_frame_for_yolo: Optional[np.ndarray] = None
        self.latest_frame_offset_for_yolo: Tuple[int, int] = (0, 0)
        self.last_yolo_submit_time = 0.0
        self.det_lock = Lock()

        if self.yolo.enabled:
            self.yolo_executor = ThreadPoolExecutor(max_workers=1)

        self.ultrasonic_sub = None
        if self.enable_ultrasonic_fusion:
            ultrasonic_topic = str(self.get_parameter("ultrasonic_topic").value)
            self.ultrasonic_sub = self.create_subscription(
                Range, ultrasonic_topic, self._ultrasonic_callback, 10
            )
            self.get_logger().info(
                f"Ultrasonic fusion enabled on {ultrasonic_topic}, stop_distance={self.ultrasonic_stop_distance_m:.2f}m"
            )

        self.get_logger().info("Starting multi-topic image subscriber")
        self.get_logger().info(f"Target topics: {TARGET_TOPICS}")
        self.get_logger().info(
            "Camera resolution must be set in camera driver (subscriber cannot force capture resolution)."
        )

        self._subscription_timer = self.create_timer(1.0, self._ensure_subscriptions)

    def _ensure_subscriptions(self) -> None:
        available = dict(self.get_topic_names_and_types())

        for topic in TARGET_TOPICS:
            if topic in self.subscriptions:
                continue

            if topic not in available:
                continue

            types = available[topic]
            if "sensor_msgs/msg/Image" in types:
                sub = self.create_subscription(Image, topic, self._image_callback, 10)
                self.subscriptions[topic] = sub
                self.get_logger().info(f"Subscribed to {topic} [Image]")
            elif "sensor_msgs/msg/CompressedImage" in types:
                sub = self.create_subscription(
                    CompressedImage, topic, self._compressed_callback, 10
                )
                self.subscriptions[topic] = sub
                self.get_logger().info(f"Subscribed to {topic} [CompressedImage]")
            else:
                self.get_logger().warn(f"Skipping {topic}; unsupported type(s): {types}")

        if not self.subscriptions:
            self.get_logger().info("Waiting for image topics to appear...")

    def _decode_compressed(self, msg: CompressedImage) -> Optional[np.ndarray]:
        data = np.frombuffer(msg.data, dtype=np.uint8)
        frame = cv2.imdecode(data, cv2.IMREAD_COLOR)
        return frame

    def _image_callback(self, msg: Image) -> None:
        try:
            frame = self.bridge.imgmsg_to_cv2(msg, desired_encoding="bgr8")
        except Exception as exc:
            self.get_logger().warn(f"Image decode failed: {exc}")
            return

        self._process_frame(frame, msg.header.stamp.sec, msg.header.stamp.nanosec)

    def _compressed_callback(self, msg: CompressedImage) -> None:
        frame = self._decode_compressed(msg)
        if frame is None:
            self.get_logger().warn("Compressed decode failed")
            return

        self._process_frame(frame, msg.header.stamp.sec, msg.header.stamp.nanosec)

    def _should_process_now(self) -> bool:
        now = self.get_clock().now().nanoseconds / 1e9
        if now - self.last_process_time < self.process_interval_sec:
            return False
        self.last_process_time = now
        return True

    def _ultrasonic_callback(self, msg: Range) -> None:
        if msg.range is None:
            return
        if msg.range < msg.min_range or msg.range > msg.max_range:
            return
        self.latest_ultrasonic_m = float(msg.range)

    def _post_filter_and_rank_detections(self, detections: List[dict], frame_shape: Tuple[int, int, int]) -> List[dict]:
        h, w = frame_shape[:2]
        cx = w / 2.0
        cy = h / 2.0
        max_dist = math.hypot(cx, cy)

        filtered = []
        for det in detections:
            x1, y1, x2, y2 = [float(v) for v in det["bbox"]]
            bw = max(0.0, x2 - x1)
            bh = max(0.0, y2 - y1)
            area = bw * bh
            if area < self.min_box_area_px:
                continue

            box_cx = (x1 + x2) / 2.0
            box_cy = (y1 + y2) / 2.0
            dist_norm = min(1.0, math.hypot(box_cx - cx, box_cy - cy) / max(1.0, max_dist))
            if dist_norm > self.max_center_distance_norm:
                continue

            center_weight = max(0.0, 1.0 - dist_norm) ** self.center_priority_power
            weighted_score = float(det.get("conf", 0.0)) * center_weight

            enriched = dict(det)
            enriched["box_area"] = area
            enriched["center_dist_norm"] = dist_norm
            enriched["weighted_score"] = weighted_score
            filtered.append(enriched)

        filtered.sort(key=lambda d: d["weighted_score"], reverse=True)
        return filtered

    def _log_fps(self) -> None:
        now = self.get_clock().now().nanoseconds / 1e9
        self.fps_window_count += 1
        elapsed = now - self.fps_window_start
        if elapsed >= 2.0:
            fps = self.fps_window_count / elapsed
            self.get_logger().info(f"Pipeline FPS={fps:.2f}")
            self.fps_window_count = 0
            self.fps_window_start = now

    def _poll_yolo_result(self) -> None:
        with self.det_lock:
            future = self.pending_yolo_future

        if future is None:
            return
        if not future.done():
            return

        try:
            detections = future.result()
            detections = [
                d for d in detections if float(d.get("conf", 0.0)) >= self.detection_conf_threshold
            ]
            stable = self.det_stabilizer.update(detections)
            with self.det_lock:
                self.latest_detections = stable
        except Exception as exc:
            self.get_logger().warn(f"YOLO inference failed: {exc}")
        finally:
            queued_frame = None
            queued_offset = (0, 0)
            now = self.get_clock().now().nanoseconds / 1e9
            with self.det_lock:
                self.pending_yolo_future = None
                if self.latest_frame_for_yolo is not None:
                    if now - self.last_yolo_submit_time >= self.yolo_interval_sec:
                        queued_frame = self.latest_frame_for_yolo
                        queued_offset = self.latest_frame_offset_for_yolo
                        self.latest_frame_for_yolo = None

            if queued_frame is not None and self.yolo_executor is not None:
                with self.det_lock:
                    self.last_yolo_submit_time = now
                    self.pending_yolo_future = self.yolo_executor.submit(
                        self.yolo.detect, queued_frame, queued_offset
                    )

    def _start_yolo(self, frame: np.ndarray) -> None:
        if not self.yolo.enabled or self.yolo_executor is None:
            return

        if self.yolo_use_roi:
            roi, offset = self.filter.center_roi(frame)
            infer_frame = roi.copy()
            infer_offset = offset
        else:
            infer_frame = frame
            infer_offset = (0, 0)

        now = self.get_clock().now().nanoseconds / 1e9

        with self.det_lock:
            if self.pending_yolo_future is None:
                if now - self.last_yolo_submit_time >= self.yolo_interval_sec:
                    self.last_yolo_submit_time = now
                    self.pending_yolo_future = self.yolo_executor.submit(
                        self.yolo.detect, infer_frame, infer_offset
                    )
                else:
                    self.latest_frame_for_yolo = infer_frame
                    self.latest_frame_offset_for_yolo = infer_offset
            else:
                self.latest_frame_for_yolo = infer_frame
                self.latest_frame_offset_for_yolo = infer_offset

    def _process_frame(self, frame: np.ndarray, sec: int, nsec: int) -> None:
        if not self._should_process_now():
            return

        self._poll_yolo_result()
        state, reason, metrics = self.filter.evaluate_frame(frame)

        if self.show_debug and self.get_logger().get_effective_level() <= LoggingSeverity.DEBUG:
            self.get_logger().debug(
                "frame_state=%s reason=%s score=%.2f lap_var=%.1f lap_thr=%.1f bright=%.1f trans=%.2f rot=%.2f jitter=%.2f"
                % (
                    state,
                    reason,
                    metrics.get("frame_score", -1.0),
                    metrics.get("lap_var", -1.0),
                    metrics.get("lap_threshold", -1.0),
                    metrics.get("brightness", -1.0),
                    metrics.get("translation", -1.0),
                    metrics.get("rotation_deg", -1.0),
                    metrics.get("jitter_std", -1.0),
                )
            )

        if state == FRAME_BAD:
            self.rejected_count += 1
            self.skip_counter += 1
            self.cooldown_remaining = self.cooldown_bad_frames
            if self.rejected_count % 20 == 0:
                self.get_logger().info(
                    f"Rejected frames: {self.rejected_count} (latest reason: {reason})"
                )
            return

        if self.cooldown_remaining > 0:
            self.cooldown_remaining -= 1
            self.skip_counter += 1
            return

        if state == FRAME_OK:
            self.skip_counter += 1
            return

        self.accepted_count += 1
        self._start_yolo(frame)

        with self.det_lock:
            detections_copy = list(self.latest_detections)
        ranked = self._post_filter_and_rank_detections(detections_copy, frame.shape)
        high_conf = [
            d for d in ranked if float(d.get("conf", 0.0)) >= self.detection_conf_threshold
        ]

        fused_stop = False
        if self.enable_ultrasonic_fusion and self.latest_ultrasonic_m is not None:
            lead_center = high_conf[0].get("center_dist_norm", 1.0) if high_conf else 1.0
            fused_stop = (
                bool(high_conf)
                and lead_center < self.ultrasonic_center_max_norm
                and (self.latest_ultrasonic_m < self.ultrasonic_stop_distance_m)
            )
            if fused_stop and not self.last_fused_stop:
                self.get_logger().warn(
                    f"FUSED STOP: object detected + ultrasonic {self.latest_ultrasonic_m:.2f}m"
                )
        self.last_fused_stop = fused_stop

        self._log_fps()

        if self.accepted_count % 15 == 0:
            self.get_logger().info(
                f"Accepted={self.accepted_count} Rejected={self.rejected_count} "
                f"Skipped={self.skip_counter} StableDet={len(detections_copy)} "
                f"HighConf={len(high_conf)} FusedStop={int(fused_stop)} stamp={sec}.{nsec:09d}"
            )

        if self.show_debug:
            draw = frame.copy()
            for det in high_conf:
                x1, y1, x2, y2 = [int(v) for v in det["bbox"]]
                label = f"id={det['id']} {det['class']} {det['conf']:.2f} cd={det['center_dist_norm']:.2f}"
                cv2.rectangle(draw, (x1, y1), (x2, y2), (0, 255, 0), 2)
                cv2.putText(draw, label, (x1, max(20, y1 - 5)), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 255, 0), 1)

            if fused_stop:
                cv2.putText(
                    draw,
                    f"FUSED STOP ({self.latest_ultrasonic_m:.2f}m)",
                    (20, 65),
                    cv2.FONT_HERSHEY_SIMPLEX,
                    0.8,
                    (0, 0, 255),
                    2,
                )

            cv2.putText(
                draw,
                f"GOOD score={metrics.get('frame_score', 0.0):.2f} det={len(high_conf)}",
                (20, 35),
                cv2.FONT_HERSHEY_SIMPLEX,
                0.8,
                (0, 255, 0),
                2,
            )
            cv2.imshow("ROS2 Filtered Stream", draw)
            cv2.waitKey(1)


def main(args=None):
    rclpy.init(args=args)
    node = MultiTopicImageSubscriber()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        if node.yolo_executor is not None:
            node.yolo_executor.shutdown(wait=True)
        node.destroy_node()
        cv2.destroyAllWindows()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
