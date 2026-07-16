from __future__ import annotations

import json
import math
import os
from pathlib import Path
import time

import cv2
from cv_bridge import CvBridge
import numpy as np
import rclpy
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import CompressedImage, Image
from std_msgs.msg import Float32, Int32, String
import torch
from ultralytics import YOLO

from h753_perception.perception_core import (
    blue_clothing_ratio,
    clip_box,
    median_depth_m,
)


def stamp_seconds(msg: Image) -> float:
    return float(msg.header.stamp.sec) + (
        float(msg.header.stamp.nanosec) * 1e-9
    )


class YoloPerceptionNode(Node):
    """Run legacy person/depth and blue-clothing pipelines together."""

    def __init__(self) -> None:
        super().__init__("h753_yolo_perception")

        # Preserve the team's model and inference values from yolo_project.
        self.declare_parameter(
            "person_model_path", "/home/jyl1015/yolo_project/yolov8s.pt"
        )
        self.declare_parameter(
            "blue_model_path", "/home/jyl1015/yolo_project/yolov8s.pt"
        )
        self.declare_parameter("person_confidence", 0.40)
        self.declare_parameter("blue_person_confidence", 0.50)
        self.declare_parameter("inference_rate_hz", 5.0)

        # This workspace's single RealSense driver owns aligned RGB-D input.
        self.declare_parameter("color_topic", "/camera/camera/color/image_raw")
        self.declare_parameter(
            "depth_topic",
            "/camera/camera/aligned_depth_to_color/image_raw",
        )
        self.declare_parameter("max_depth_age_s", 0.25)
        self.declare_parameter("depth_roi_fraction", 0.20)
        self.declare_parameter("depth_scale_m", 0.001)

        # HSV and torso parameters are copied from person_blue.py.
        self.declare_parameter("blue_h_min", 100)
        self.declare_parameter("blue_h_max", 130)
        self.declare_parameter("blue_s_min", 80)
        self.declare_parameter("blue_v_min", 50)
        self.declare_parameter("blue_ratio_threshold", 0.15)
        self.declare_parameter("torso_top_ratio", 0.15)
        self.declare_parameter("torso_bottom_ratio", 0.60)

        # Mode 4 keeps GUI features off; Mode 5 overrides both for HSV tuning.
        self.declare_parameter("show_window", False)
        self.declare_parameter("tuning_mode", False)
        self.declare_parameter("publish_raw_result_image", False)
        self.declare_parameter("jpeg_quality", 85)

        self.person_confidence = float(
            self.get_parameter("person_confidence").value
        )
        self.blue_confidence = float(
            self.get_parameter("blue_person_confidence").value
        )
        self.max_depth_age_s = max(
            0.0,
            float(self.get_parameter("max_depth_age_s").value),
        )
        self.depth_roi_fraction = float(
            self.get_parameter("depth_roi_fraction").value
        )
        self.depth_scale_m = float(self.get_parameter("depth_scale_m").value)
        self.blue_h_min = int(self.get_parameter("blue_h_min").value)
        self.blue_h_max = int(self.get_parameter("blue_h_max").value)
        self.blue_s_min = int(self.get_parameter("blue_s_min").value)
        self.blue_v_min = int(self.get_parameter("blue_v_min").value)
        self.blue_ratio_threshold = float(
            self.get_parameter("blue_ratio_threshold").value
        )
        self.torso_top_ratio = float(
            self.get_parameter("torso_top_ratio").value
        )
        self.torso_bottom_ratio = float(
            self.get_parameter("torso_bottom_ratio").value
        )
        self.show_window = bool(self.get_parameter("show_window").value)
        self.tuning_mode = bool(self.get_parameter("tuning_mode").value)
        self.publish_raw_result_image = bool(
            self.get_parameter("publish_raw_result_image").value
        )
        self.jpeg_quality = max(
            1,
            min(100, int(self.get_parameter("jpeg_quality").value)),
        )

        person_model_path = Path(
            str(self.get_parameter("person_model_path").value)
        ).expanduser()
        blue_model_path = Path(
            str(self.get_parameter("blue_model_path").value)
        ).expanduser()
        for model_path in (person_model_path, blue_model_path):
            if not model_path.is_file():
                raise FileNotFoundError(f"YOLO model not found: {model_path}")

        self.device: int | str = 0 if torch.cuda.is_available() else "cpu"
        self.get_logger().info(
            f"Loading person model {person_model_path.name} and "
            f"blue-clothing model {blue_model_path.name}"
        )
        self.person_model = YOLO(str(person_model_path))
        self.blue_model = YOLO(str(blue_model_path))
        self.get_logger().info(f"YOLO models ready on {self.device}")

        self.bridge = CvBridge()
        self.latest_color: np.ndarray | None = None
        self.latest_color_msg: Image | None = None
        self.latest_color_stamp = -1.0
        self.latest_depth: np.ndarray | None = None
        self.latest_depth_stamp = -1.0
        self.processed_color_stamp = -1.0
        self.last_error_log_at = 0.0
        self.frame_count = 0

        self.person_found_pub = self.create_publisher(
            Int32, "/yolo/person_found", 10
        )
        self.blue_person_pub = self.create_publisher(
            Int32, "/yolo/blue_person", 10
        )
        self.target_pub = self.create_publisher(String, "/yolo/target", 10)
        self.distance_pub = self.create_publisher(
            Float32,
            "/yolo/nearest_distance_m",
            10,
        )
        self.status_pub = self.create_publisher(String, "/yolo/status", 10)
        self.compressed_image_pub = self.create_publisher(
            CompressedImage,
            "/yolo/detected_image/compressed",
            1,
        )
        self.raw_image_pub = self.create_publisher(
            Image, "/yolo/result_image", 1
        )

        color_topic = str(self.get_parameter("color_topic").value)
        depth_topic = str(self.get_parameter("depth_topic").value)
        self.color_sub = self.create_subscription(
            Image,
            color_topic,
            self._color_callback,
            qos_profile_sensor_data,
        )
        self.depth_sub = self.create_subscription(
            Image,
            depth_topic,
            self._depth_callback,
            qos_profile_sensor_data,
        )

        self._configure_tuning_windows()
        inference_rate_hz = max(
            0.2,
            float(self.get_parameter("inference_rate_hz").value),
        )
        self.timer = self.create_timer(
            1.0 / inference_rate_hz, self._process_latest
        )
        self.get_logger().info(
            f"Listening for aligned RGB-D at {inference_rate_hz:.1f} Hz: "
            f"{color_topic}, {depth_topic}"
        )

    def _configure_tuning_windows(self) -> None:
        if not (self.show_window or self.tuning_mode):
            return
        if not os.environ.get("DISPLAY"):
            self.get_logger().warn(
                "DISPLAY is unset; disabling YOLO preview and "
                "HSV tuning windows"
            )
            self.show_window = False
            self.tuning_mode = False
            return
        try:
            if self.tuning_mode:
                cv2.namedWindow("HSV Tuner", cv2.WINDOW_NORMAL)
                cv2.resizeWindow("HSV Tuner", 400, 200)
                cv2.createTrackbar(
                    "H_min", "HSV Tuner", self.blue_h_min, 179, lambda _: None
                )
                cv2.createTrackbar(
                    "H_max", "HSV Tuner", self.blue_h_max, 179, lambda _: None
                )
                cv2.createTrackbar(
                    "S_min", "HSV Tuner", self.blue_s_min, 255, lambda _: None
                )
                cv2.createTrackbar(
                    "V_min", "HSV Tuner", self.blue_v_min, 255, lambda _: None
                )
                cv2.createTrackbar(
                    "Ratio%",
                    "HSV Tuner",
                    int(self.blue_ratio_threshold * 100.0),
                    100,
                    lambda _: None,
                )
        except cv2.error as exc:
            self.get_logger().warn(
                f"Could not create OpenCV tuning window: {exc}"
            )
            self.show_window = False
            self.tuning_mode = False

    def _hsv_parameters(self) -> tuple[int, int, int, int, float]:
        if not self.tuning_mode:
            return (
                self.blue_h_min,
                self.blue_h_max,
                self.blue_s_min,
                self.blue_v_min,
                self.blue_ratio_threshold,
            )
        return (
            cv2.getTrackbarPos("H_min", "HSV Tuner"),
            cv2.getTrackbarPos("H_max", "HSV Tuner"),
            cv2.getTrackbarPos("S_min", "HSV Tuner"),
            cv2.getTrackbarPos("V_min", "HSV Tuner"),
            cv2.getTrackbarPos("Ratio%", "HSV Tuner") / 100.0,
        )

    def _color_callback(self, msg: Image) -> None:
        try:
            self.latest_color = self.bridge.imgmsg_to_cv2(
                msg,
                desired_encoding="bgr8",
            )
            self.latest_color_msg = msg
            self.latest_color_stamp = stamp_seconds(msg)
        # cv_bridge exception types differ across ROS 2 releases.
        except Exception as exc:
            self._log_processing_error("color conversion", exc)

    def _depth_callback(self, msg: Image) -> None:
        try:
            self.latest_depth = self.bridge.imgmsg_to_cv2(
                msg,
                desired_encoding="passthrough",
            )
            self.latest_depth_stamp = stamp_seconds(msg)
        except Exception as exc:
            self._log_processing_error("depth conversion", exc)

    def _depth_for_color(self) -> tuple[np.ndarray | None, bool]:
        if self.latest_depth is None:
            return None, False
        age = abs(self.latest_color_stamp - self.latest_depth_stamp)
        if age > self.max_depth_age_s:
            return None, False
        return self.latest_depth, True

    def _process_latest(self) -> None:
        if self.latest_color is None or self.latest_color_msg is None:
            return
        if self.latest_color_stamp == self.processed_color_stamp:
            return

        # Pseudocode:
        #   consume only the newest RGB frame (no inference backlog)
        #   detect all people and estimate center-ROI median depth
        #   run the legacy blue-clothing classifier on the same frame
        #   publish one coherent set of flags, status, distance, and overlay
        frame = self.latest_color.copy()
        source_msg = self.latest_color_msg
        self.processed_color_stamp = self.latest_color_stamp
        depth_image, depth_fresh = self._depth_for_color()
        started_at = time.monotonic()

        try:
            person_results = self.person_model(
                frame,
                verbose=False,
                device=self.device,
                classes=[0],
                conf=self.person_confidence,
            )
            blue_results = self.blue_model(
                frame, verbose=False, device=self.device
            )
            annotated = frame.copy()
            people_with_depth: list[
                tuple[float, tuple[int, int, int, int]]
            ] = []
            person_count = 0

            person_boxes = person_results[0].boxes
            if person_boxes is not None:
                for box in person_boxes:
                    coords = clip_box(
                        box.xyxy[0].detach().cpu().numpy().astype(int),
                        frame.shape[1],
                        frame.shape[0],
                    )
                    confidence = float(box.conf[0].detach().cpu().item())
                    person_count += 1
                    x1, y1, x2, y2 = coords
                    cv2.rectangle(
                        annotated, (x1, y1), (x2, y2), (0, 255, 0), 2
                    )
                    cv2.putText(
                        annotated,
                        f"person {confidence:.2f}",
                        (x1, max(y1 - 8, 15)),
                        cv2.FONT_HERSHEY_SIMPLEX,
                        0.55,
                        (0, 255, 0),
                        2,
                    )
                    distance = median_depth_m(
                        depth_image,
                        coords,
                        self.depth_roi_fraction,
                        self.depth_scale_m,
                    )
                    if distance is not None:
                        people_with_depth.append((distance, coords))

            nearest_distance = math.nan
            if people_with_depth:
                people_with_depth.sort(key=lambda item: item[0])
                nearest_distance, target_box = people_with_depth[0]
                tx1, ty1, tx2, ty2 = target_box
                cv2.rectangle(
                    annotated, (tx1, ty1), (tx2, ty2), (0, 0, 255), 3
                )
                cv2.putText(
                    annotated,
                    f"TARGET {nearest_distance:.2f}m",
                    (tx1, max(ty1 - 10, 15)),
                    cv2.FONT_HERSHEY_SIMPLEX,
                    0.7,
                    (0, 0, 255),
                    2,
                )
                self.target_pub.publish(String(data="DETECTED"))

            h_min, h_max, s_min, v_min, blue_threshold = self._hsv_parameters()
            blue_person_found = False
            blue_person_count = 0
            first_mask = None
            blue_boxes = blue_results[0].boxes
            if blue_boxes is not None:
                for box in blue_boxes:
                    class_id = int(box.cls[0].detach().cpu().item())
                    confidence = float(box.conf[0].detach().cpu().item())
                    if class_id != 0 or confidence < self.blue_confidence:
                        continue
                    coords = clip_box(
                        box.xyxy[0].detach().cpu().numpy().astype(int),
                        frame.shape[1],
                        frame.shape[0],
                    )
                    ratio, mask, torso_box = blue_clothing_ratio(
                        frame,
                        coords,
                        h_min,
                        h_max,
                        s_min,
                        v_min,
                        self.torso_top_ratio,
                        self.torso_bottom_ratio,
                    )
                    if first_mask is None and mask is not None:
                        first_mask = mask
                    is_blue = ratio >= blue_threshold
                    if is_blue:
                        blue_person_found = True
                        blue_person_count += 1
                        color = (255, 100, 0)
                        label = f"BLUE {ratio * 100:.0f}% ({confidence:.2f})"
                        thickness = 3
                    else:
                        color = (128, 128, 128)
                        label = f"person {ratio * 100:.0f}% ({confidence:.2f})"
                        thickness = 1
                    x1, y1, x2, y2 = coords
                    cv2.rectangle(
                        annotated, (x1, y1), (x2, y2), color, thickness
                    )
                    cv2.putText(
                        annotated,
                        label,
                        (x1, max(y1 - 8, 15)),
                        cv2.FONT_HERSHEY_SIMPLEX,
                        0.55,
                        color,
                        2,
                    )
                    if self.tuning_mode and torso_box is not None:
                        rx1, ry1, rx2, ry2 = torso_box
                        cv2.rectangle(
                            annotated,
                            (rx1, ry1),
                            (rx2, ry2),
                            (0, 255, 255),
                            1,
                        )

            person_found = person_count > 0
            status_text = "PERSON FOUND!" if person_found else "searching..."
            status_color = (0, 255, 0) if person_found else (180, 180, 180)
            cv2.putText(
                annotated,
                status_text,
                (10, 30),
                cv2.FONT_HERSHEY_SIMPLEX,
                0.8,
                status_color,
                2,
            )
            if self.tuning_mode:
                info = (
                    f"H:{h_min}-{h_max} S:{s_min}+ V:{v_min}+ | "
                    f"thresh:{blue_threshold * 100:.0f}%"
                )
                cv2.putText(
                    annotated,
                    info,
                    (10, frame.shape[0] - 15),
                    cv2.FONT_HERSHEY_SIMPLEX,
                    0.5,
                    (0, 255, 255),
                    1,
                )

            inference_ms = (time.monotonic() - started_at) * 1000.0
            self.person_found_pub.publish(Int32(data=int(person_found)))
            self.blue_person_pub.publish(Int32(data=int(blue_person_found)))
            self.distance_pub.publish(Float32(data=float(nearest_distance)))
            self.status_pub.publish(
                String(
                    data=json.dumps(
                        {
                            "person_found": person_found,
                            "person_count": person_count,
                            "blue_person_found": blue_person_found,
                            "blue_person_count": blue_person_count,
                            "nearest_distance_m": (
                                None
                                if math.isnan(nearest_distance)
                                else round(nearest_distance, 3)
                            ),
                            "depth_fresh": depth_fresh,
                            "inference_ms": round(inference_ms, 1),
                            "device": str(self.device),
                        },
                        separators=(",", ":"),
                    )
                )
            )
            self._publish_images(source_msg, annotated)
            self._show_images(annotated, first_mask)

            self.frame_count += 1
            if self.frame_count % 30 == 0:
                self.get_logger().info(
                    f"People={person_count}, blue={blue_person_count}, "
                    f"depth_fresh={depth_fresh}, "
                    f"inference={inference_ms:.0f} ms"
                )
        except Exception as exc:
            self._log_processing_error("YOLO inference", exc)

    def _publish_images(
        self, source_msg: Image, annotated: np.ndarray
    ) -> None:
        encoded, jpeg = cv2.imencode(
            ".jpg",
            annotated,
            [cv2.IMWRITE_JPEG_QUALITY, self.jpeg_quality],
        )
        if encoded:
            compressed = CompressedImage()
            compressed.header = source_msg.header
            compressed.format = "jpeg"
            compressed.data = jpeg.tobytes()
            self.compressed_image_pub.publish(compressed)
        if self.publish_raw_result_image:
            raw = self.bridge.cv2_to_imgmsg(annotated, encoding="bgr8")
            raw.header = source_msg.header
            self.raw_image_pub.publish(raw)

    def _show_images(
        self, annotated: np.ndarray, mask: np.ndarray | None
    ) -> None:
        if not self.show_window:
            return
        cv2.imshow("YOLO Person + Blue Filter", annotated)
        if self.tuning_mode and mask is not None:
            cv2.imshow("Blue Mask (torso)", mask)
        cv2.waitKey(1)

    def _log_processing_error(self, stage: str, exc: Exception) -> None:
        now = time.monotonic()
        if now - self.last_error_log_at >= 2.0:
            self.last_error_log_at = now
            self.get_logger().error(f"{stage} failed: {exc}")

    def destroy_node(self) -> bool:
        if self.show_window or self.tuning_mode:
            cv2.destroyAllWindows()
        return super().destroy_node()


def main(args=None) -> None:
    rclpy.init(args=args)
    node = None
    try:
        node = YoloPerceptionNode()
        rclpy.spin(node)
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    finally:
        if node is not None:
            node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
