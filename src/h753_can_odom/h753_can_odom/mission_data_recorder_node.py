#!/usr/bin/env python3
from __future__ import annotations

import json
import math
import os
import time
from dataclasses import dataclass
from pathlib import Path as FilePath
from typing import Any
from uuid import uuid4

import rclpy
from action_msgs.msg import GoalStatusArray
from geometry_msgs.msg import PoseStamped
from nav_msgs.msg import Path
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from rclpy.qos import (
    DurabilityPolicy,
    HistoryPolicy,
    QoSProfile,
    ReliabilityPolicy,
    qos_profile_sensor_data,
)
from sensor_msgs.msg import CompressedImage
from std_msgs.msg import Int32, String, UInt8
from std_srvs.srv import Trigger
from tf2_ros import Buffer, TransformException, TransformListener

from h753_can_odom.mission_data_core import (
    DetectionEventGate,
    MapManifest,
    MissionStore,
    NavigationMissionTracker,
    PoseSample,
    decide_route_sample,
    quaternion_to_yaw,
    stamp_to_ns,
    yaw_to_quaternion,
)


@dataclass
class PendingImage:
    detection_id: str
    image_id: str
    mission_id: str
    detection_stamp_ns: int
    not_before: float
    expires_at: float


@dataclass
class PendingPoseEvent:
    topic: str
    yolo_payload: dict[str, Any]
    detected_at_ns: int
    expires_at: float


class MissionDataRecorderNode(Node):
    """Record map-frame robot paths and YOLO detection poses on the Jetson."""

    def __init__(self) -> None:
        super().__init__('h753_mission_data_recorder')

        self.declare_parameter('robot_id', 'h753_jetson_01')
        self.declare_parameter(
            'database_path',
            '/home/jyl1015/.ros/h753_mission/mission_outbox.db',
        )
        self.declare_parameter(
            'data_directory',
            '/home/jyl1015/.ros/h753_mission',
        )
        self.declare_parameter('localization_backend', 'amcl')
        self.declare_parameter(
            'map_yaml_path',
            '/home/jyl1015/ros2_graduation_project_ws/'
            'maps/go2/go2_map.yaml',
        )
        self.declare_parameter(
            'posegraph_base_path',
            '/home/jyl1015/ros2_graduation_project_ws/maps/h753_map',
        )
        self.declare_parameter('map_frame_id', 'map')
        self.declare_parameter('base_frame_id', 'base_link')
        self.declare_parameter('mode_topic', '/robot_mode')
        self.declare_parameter(
            'navigation_status_topic',
            '/navigate_to_pose/_action/status',
        )
        self.declare_parameter(
            'navigate_through_poses_status_topic',
            '/navigate_through_poses/_action/status',
        )
        self.declare_parameter(
            'follow_waypoints_status_topic',
            '/follow_waypoints/_action/status',
        )
        self.declare_parameter('complete_on_navigation_result', True)
        self.declare_parameter('active_modes', [3, 4])
        self.declare_parameter('detection_modes', [4])
        self.declare_parameter('auto_start_mission', True)
        self.declare_parameter('sample_rate_hz', 2.0)
        self.declare_parameter('min_distance_m', 0.05)
        self.declare_parameter('min_yaw_deg', 5.0)
        self.declare_parameter('max_record_interval_s', 5.0)
        self.declare_parameter('max_pose_jump_m', 1.0)
        self.declare_parameter('max_yaw_jump_deg', 60.0)
        self.declare_parameter('path_history_limit', 10000)
        self.declare_parameter('path_publish_rate_hz', 1.0)
        self.declare_parameter('person_topic', '/yolo/person_found')
        self.declare_parameter('blue_person_topic', '/yolo/blue_person')
        self.declare_parameter('yolo_status_topic', '/yolo/status')
        self.declare_parameter(
            'representative_image_topic',
            '/yolo/detected_image/compressed',
        )
        self.declare_parameter(
            'gateway_status_topic',
            '/vlm/gateway/status',
        )
        self.declare_parameter('mission_path_topic', '/mission/path')
        self.declare_parameter('mission_status_topic', '/mission/status')
        self.declare_parameter(
            'detection_event_topic',
            '/mission/detection_event',
        )
        self.declare_parameter('start_service', '/mission/start_new')
        self.declare_parameter('end_service', '/mission/end')
        self.declare_parameter('image_wait_s', 0.25)
        self.declare_parameter('image_expiry_s', 2.0)
        self.declare_parameter('max_image_age_s', 2.0)
        self.declare_parameter('detection_pose_expiry_s', 5.0)
        self.declare_parameter('fallback_rearm_cooldown_s', 15.0)
        self.declare_parameter('fallback_rearm_clear_s', 2.0)

        self.robot_id = str(self.get_parameter('robot_id').value)
        self.database_path = FilePath(
            str(self.get_parameter('database_path').value)
        ).expanduser()
        self.data_directory = FilePath(
            str(self.get_parameter('data_directory').value)
        ).expanduser()
        self.localization_backend = str(
            self.get_parameter('localization_backend').value
        )
        self.map_frame_id = str(self.get_parameter('map_frame_id').value)
        self.base_frame_id = str(self.get_parameter('base_frame_id').value)
        self.active_modes = {
            int(mode) for mode in self.get_parameter('active_modes').value
        }
        self.detection_modes = {
            int(mode) for mode in self.get_parameter('detection_modes').value
        }
        self.auto_start_mission = bool(
            self.get_parameter('auto_start_mission').value
        )
        self.complete_on_navigation_result = bool(
            self.get_parameter('complete_on_navigation_result').value
        )
        self.sample_rate_hz = float(
            self.get_parameter('sample_rate_hz').value
        )
        self.min_distance_m = float(
            self.get_parameter('min_distance_m').value
        )
        self.min_yaw_rad = math.radians(
            float(self.get_parameter('min_yaw_deg').value)
        )
        self.max_record_interval_s = float(
            self.get_parameter('max_record_interval_s').value
        )
        self.max_pose_jump_m = float(
            self.get_parameter('max_pose_jump_m').value
        )
        self.max_yaw_jump_rad = math.radians(
            float(self.get_parameter('max_yaw_jump_deg').value)
        )
        self.path_history_limit = int(
            self.get_parameter('path_history_limit').value
        )
        self.path_publish_rate_hz = float(
            self.get_parameter('path_publish_rate_hz').value
        )
        self.image_wait_s = float(self.get_parameter('image_wait_s').value)
        self.image_expiry_s = float(
            self.get_parameter('image_expiry_s').value
        )
        self.max_image_age_ns = int(
            float(self.get_parameter('max_image_age_s').value)
            * 1_000_000_000
        )
        self.detection_pose_expiry_s = float(
            self.get_parameter('detection_pose_expiry_s').value
        )

        if self.sample_rate_hz <= 0.0:
            raise ValueError('sample_rate_hz must be positive')
        if self.min_distance_m <= 0.0:
            raise ValueError('min_distance_m must be positive')
        if self.max_record_interval_s <= 0.0:
            raise ValueError('max_record_interval_s must be positive')
        if self.max_pose_jump_m <= self.min_distance_m:
            raise ValueError('max_pose_jump_m must exceed min_distance_m')
        if self.path_publish_rate_hz <= 0.0:
            raise ValueError('path_publish_rate_hz must be positive')

        self.data_directory.mkdir(parents=True, exist_ok=True)
        self.image_directory = self.data_directory / 'images'
        self.image_directory.mkdir(parents=True, exist_ok=True)
        self.manifest = MapManifest.from_sources(
            self.localization_backend,
            str(self.get_parameter('map_yaml_path').value),
            str(self.get_parameter('posegraph_base_path').value),
            self.map_frame_id,
        )
        self.store = MissionStore(self.database_path)

        self.current_mode: int | None = None
        self.current_mission_id: str | None = None
        self.recording_enabled = False
        self.last_sample: PoseSample | None = None
        self.last_route_seq = -1
        self.segment_id = 0
        self.last_record_monotonic: float | None = None
        self.path_dirty = True
        self.latest_yolo_status: dict[str, Any] = {}
        self.latest_image: tuple[int, bytes, str] | None = None
        self.pending_images: list[PendingImage] = []
        self.pending_pose_event: PendingPoseEvent | None = None
        self.last_tf_error = ''
        self.last_tf_warning_at = 0.0
        self.last_detection_id: str | None = None
        self.navigation_mission_tracker = NavigationMissionTracker()
        self.awaiting_navigation_goal = False

        self.person_topic = str(self.get_parameter('person_topic').value)
        self.blue_person_topic = str(
            self.get_parameter('blue_person_topic').value
        )
        self.detection_gate = DetectionEventGate(
            (self.person_topic, self.blue_person_topic),
            fallback_cooldown_s=float(
                self.get_parameter('fallback_rearm_cooldown_s').value
            ),
            fallback_clear_s=float(
                self.get_parameter('fallback_rearm_clear_s').value
            ),
        )

        latched_qos = QoSProfile(
            history=HistoryPolicy.KEEP_LAST,
            depth=1,
            reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.TRANSIENT_LOCAL,
        )
        event_qos = QoSProfile(
            history=HistoryPolicy.KEEP_LAST,
            depth=10,
            reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.TRANSIENT_LOCAL,
        )
        self.path_pub = self.create_publisher(
            Path,
            str(self.get_parameter('mission_path_topic').value),
            latched_qos,
        )
        self.status_pub = self.create_publisher(
            String,
            str(self.get_parameter('mission_status_topic').value),
            latched_qos,
        )
        self.detection_pub = self.create_publisher(
            String,
            str(self.get_parameter('detection_event_topic').value),
            event_qos,
        )

        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)
        self.create_subscription(
            UInt8,
            str(self.get_parameter('mode_topic').value),
            self._mode_callback,
            latched_qos,
        )
        navigation_status_topics = {
            'navigate_to_pose': str(
                self.get_parameter('navigation_status_topic').value
            ),
            'navigate_through_poses': str(
                self.get_parameter(
                    'navigate_through_poses_status_topic'
                ).value
            ),
            'follow_waypoints': str(
                self.get_parameter('follow_waypoints_status_topic').value
            ),
        }
        for source, topic in navigation_status_topics.items():
            self.create_subscription(
                GoalStatusArray,
                topic,
                lambda msg, source=source: self._navigation_status_callback(
                    source,
                    msg,
                ),
                10,
            )
        self.create_subscription(
            Int32,
            self.person_topic,
            lambda msg: self._detection_gate_callback(
                self.person_topic,
                msg,
            ),
            10,
        )
        self.create_subscription(
            Int32,
            self.blue_person_topic,
            lambda msg: self._detection_gate_callback(
                self.blue_person_topic,
                msg,
            ),
            10,
        )
        self.create_subscription(
            String,
            str(self.get_parameter('yolo_status_topic').value),
            self._yolo_status_callback,
            10,
        )
        self.create_subscription(
            CompressedImage,
            str(self.get_parameter('representative_image_topic').value),
            self._image_callback,
            qos_profile_sensor_data,
        )
        self.create_subscription(
            String,
            str(self.get_parameter('gateway_status_topic').value),
            self._gateway_status_callback,
            10,
        )
        self.create_service(
            Trigger,
            str(self.get_parameter('start_service').value),
            self._start_new_mission_service,
        )
        self.create_service(
            Trigger,
            str(self.get_parameter('end_service').value),
            self._end_mission_service,
        )

        self.create_timer(1.0 / self.sample_rate_hz, self._route_timer)
        self.create_timer(
            1.0 / self.path_publish_rate_hz,
            self._path_timer,
        )
        self.create_timer(0.10, self._pending_image_timer)
        self.create_timer(1.0, self._status_timer)

        active = self.store.active_mission()
        if active is not None and active['map_id'] == self.manifest.map_id:
            self._restore_mission(active)
            self.get_logger().info(
                f'Restored active mission {self.current_mission_id}'
            )
        self._publish_status()
        self.get_logger().info(
            'Mission data recorder ready: '
            f'db={self.database_path}, map_id={self.manifest.map_id[:12]}, '
            f'active_modes={sorted(self.active_modes)}, '
            f'detection_modes={sorted(self.detection_modes)}'
        )

    def destroy_node(self) -> bool:
        self.store.close()
        return super().destroy_node()

    def _mode_callback(self, msg: UInt8) -> None:
        self.current_mode = int(msg.data)
        self.recording_enabled = self.current_mode in self.active_modes
        if self.current_mode != 4:
            self.navigation_mission_tracker.reset()
            self.awaiting_navigation_goal = False
        if (
            self.recording_enabled
            and self.auto_start_mission
            and self.current_mission_id is None
            and self._mission_auto_start_allowed()
        ):
            self._ensure_mission()
        self._publish_status()

    def _mission_auto_start_allowed(self) -> bool:
        return not (
            self.current_mode == 4 and self.awaiting_navigation_goal
        )

    @staticmethod
    def _navigation_goal_id(status: Any) -> str:
        return bytes(status.goal_info.goal_id.uuid).hex()

    @staticmethod
    def _navigation_goal_stamp_ns(status: Any) -> int:
        stamp = status.goal_info.stamp
        return stamp_to_ns(stamp.sec, stamp.nanosec)

    def _navigation_status_callback(
        self,
        source: str,
        msg: GoalStatusArray,
    ) -> None:
        if not self.complete_on_navigation_result or self.current_mode != 4:
            return

        statuses = sorted(
            msg.status_list,
            key=self._navigation_goal_stamp_ns,
        )
        for status in statuses:
            transition = self.navigation_mission_tracker.observe(
                source,
                self._navigation_goal_id(status),
                int(status.status),
            )
            if transition is None:
                continue
            if transition.event == 'started':
                if self.current_mission_id is None:
                    self._ensure_mission()
                self.awaiting_navigation_goal = False
                self.get_logger().info(
                    'Navigation route linked to mission: '
                    f'{transition.source}/{transition.goal_id}'
                )
            elif transition.event == 'promoted':
                self.awaiting_navigation_goal = False
                self.get_logger().info(
                    'Navigation mission owner promoted to parent route: '
                    f'{transition.source}/{transition.goal_id}'
                )
            elif transition.event == 'replaced':
                self._finish_current_mission(
                    status='ABORTED',
                    end_reason='NAVIGATION_GOAL_REPLACED',
                )
                self._ensure_mission()
                self.awaiting_navigation_goal = False
                self.get_logger().warn(
                    'Navigation route replaced; started a new mission: '
                    f'{transition.source}/{transition.goal_id}'
                )
            elif transition.event == 'succeeded':
                self._finish_current_mission(
                    status='COMPLETED',
                    end_reason='NAVIGATION_GOAL_SUCCEEDED',
                )
            elif transition.event in ('canceled', 'aborted'):
                self._finish_current_mission(
                    status='ABORTED',
                    end_reason=(
                        'NAVIGATION_GOAL_CANCELED'
                        if transition.event == 'canceled'
                        else 'NAVIGATION_GOAL_ABORTED'
                    ),
                )

    def _ensure_mission(self, force_new: bool = False) -> bool:
        now_ns = self.get_clock().now().nanoseconds
        mission, created = self.store.create_or_resume_mission(
            self.robot_id,
            self.manifest,
            now_ns,
            self.current_mode,
            force_new=force_new,
        )
        self._restore_mission(mission)
        if created:
            self.get_logger().info(
                f'Started mission {self.current_mission_id} '
                f'on map {self.manifest.map_id[:12]}'
            )
        return created

    def _restore_mission(self, mission: dict[str, Any]) -> None:
        self.current_mission_id = str(mission['mission_id'])
        latest = self.store.latest_route_point(self.current_mission_id)
        if latest is None:
            self.last_sample = None
            self.last_route_seq = -1
            self.segment_id = 0
        else:
            self.last_sample = PoseSample(
                int(latest['stamp_ns']),
                float(latest['x']),
                float(latest['y']),
                float(latest['yaw']),
            )
            self.last_route_seq = int(latest['route_seq'])
            self.segment_id = int(latest['segment_id'])
        self.last_record_monotonic = None
        self.path_dirty = True
        self._publish_path()

    def _route_timer(self) -> None:
        if not self.recording_enabled:
            return
        if (
            self.current_mission_id is None
            and not self._mission_auto_start_allowed()
        ):
            return
        if self.current_mission_id is None and not self._ensure_mission():
            if self.current_mission_id is None:
                return
        self._record_current_pose(force=False)

    def _lookup_pose(self) -> PoseSample | None:
        try:
            transform = self.tf_buffer.lookup_transform(
                self.map_frame_id,
                self.base_frame_id,
                rclpy.time.Time(),
            )
        except TransformException as exc:
            self.last_tf_error = str(exc)
            now = time.monotonic()
            if now - self.last_tf_warning_at >= 5.0:
                self.get_logger().warn(
                    f'Waiting for TF {self.map_frame_id}->'
                    f'{self.base_frame_id}: {exc}'
                )
                self.last_tf_warning_at = now
            return None

        stamp_ns = stamp_to_ns(
            transform.header.stamp.sec,
            transform.header.stamp.nanosec,
        )
        if stamp_ns <= 0:
            stamp_ns = self.get_clock().now().nanoseconds
        rotation = transform.transform.rotation
        self.last_tf_error = ''
        return PoseSample(
            stamp_ns=stamp_ns,
            x=float(transform.transform.translation.x),
            y=float(transform.transform.translation.y),
            yaw=quaternion_to_yaw(
                rotation.x,
                rotation.y,
                rotation.z,
                rotation.w,
            ),
        )

    def _record_current_pose(self, force: bool) -> PoseSample | None:
        if self.current_mission_id is None:
            return None
        sample = self._lookup_pose()
        if sample is None:
            return None
        now = time.monotonic()
        elapsed = (
            float('inf')
            if self.last_record_monotonic is None
            else now - self.last_record_monotonic
        )
        decision = decide_route_sample(
            self.last_sample,
            sample,
            elapsed,
            self.min_distance_m,
            self.min_yaw_rad,
            self.max_record_interval_s,
            self.max_pose_jump_m,
            self.max_yaw_jump_rad,
            force=force,
        )
        if not decision.record:
            return self.last_sample
        quality_flags = ''
        if decision.pose_jump:
            self.segment_id += 1
            quality_flags = 'POSE_JUMP'
            self.get_logger().warn(
                'Map pose jump detected; starting route segment '
                f'{self.segment_id}'
            )
        self.last_route_seq = self.store.add_route_point(
            self.current_mission_id,
            sample,
            self.segment_id,
            quality_flags,
        )
        self.last_sample = sample
        self.last_record_monotonic = now
        self.path_dirty = True
        return sample

    def _detection_gate_callback(self, topic: str, msg: Int32) -> None:
        should_create = self.detection_gate.update_gate(
            topic,
            int(msg.data),
            time.monotonic(),
        )
        if not should_create:
            return
        if self.current_mode not in self.detection_modes:
            self.get_logger().warn(
                f'Ignored YOLO event outside detection modes: '
                f'mode={self.current_mode}, topic={topic}'
            )
            return
        if self.current_mission_id is None:
            if not self._mission_auto_start_allowed():
                self.get_logger().warn(
                    'Ignored YOLO event after mission completion while '
                    'waiting for the next navigation goal'
                )
                return
            self._ensure_mission()
        detected_at_ns = self.get_clock().now().nanoseconds
        sample = self._record_current_pose(force=True)
        if sample is None or self.current_mission_id is None:
            self.pending_pose_event = PendingPoseEvent(
                topic=topic,
                yolo_payload=dict(self.latest_yolo_status),
                detected_at_ns=detected_at_ns,
                expires_at=(
                    time.monotonic() + self.detection_pose_expiry_s
                ),
            )
            self.get_logger().warn(
                'YOLO detected a person but no valid map pose was available; '
                'waiting briefly for TF instead of writing a false coordinate'
            )
            return

        self._create_detection(
            topic,
            sample,
            detected_at_ns=detected_at_ns,
        )

    def _create_detection(
        self,
        topic: str,
        sample: PoseSample,
        yolo_snapshot: dict[str, Any] | None = None,
        detected_at_ns: int | None = None,
    ) -> None:
        if self.current_mission_id is None:
            return

        yolo_payload = dict(
            self.latest_yolo_status
            if yolo_snapshot is None
            else yolo_snapshot
        )
        yolo_payload['trigger_topic'] = topic
        yolo_payload['gate_state'] = {
            key: int(value)
            for key, value in self.detection_gate.gate_state.items()
        }
        image_id = str(uuid4())
        detection_id, payload = self.store.add_detection(
            self.current_mission_id,
            self.manifest,
            sample,
            self.last_route_seq,
            yolo_payload,
            image_id,
            detected_at_ns=detected_at_ns,
        )
        self.last_detection_id = detection_id
        now = time.monotonic()
        self.pending_images.append(
            PendingImage(
                detection_id=detection_id,
                image_id=image_id,
                mission_id=self.current_mission_id,
                detection_stamp_ns=sample.stamp_ns,
                not_before=now + self.image_wait_s,
                expires_at=now + self.image_expiry_s,
            )
        )
        self.detection_pub.publish(
            String(
                data=json.dumps(
                    payload,
                    ensure_ascii=False,
                    separators=(',', ':'),
                    sort_keys=True,
                )
            )
        )
        self.get_logger().warn(
            f'Recorded detection {detection_id} at '
            f'({sample.x:.2f}, {sample.y:.2f}), '
            f'route_seq={self.last_route_seq}'
        )
        self._publish_status()

    def _yolo_status_callback(self, msg: String) -> None:
        try:
            payload = json.loads(msg.data)
        except (TypeError, ValueError):
            return
        if isinstance(payload, dict):
            self.latest_yolo_status = payload

    def _gateway_status_callback(self, msg: String) -> None:
        try:
            payload = json.loads(msg.data)
        except (TypeError, ValueError):
            return
        if not isinstance(payload, dict):
            return
        armed = payload.get('local_detection_armed')
        if isinstance(armed, bool):
            if self.detection_gate.update_gateway(armed, time.monotonic()):
                self.get_logger().info(
                    'Detection event recording re-armed with VLM gateway'
                )

    def _image_callback(self, msg: CompressedImage) -> None:
        stamp_ns = stamp_to_ns(msg.header.stamp.sec, msg.header.stamp.nanosec)
        if stamp_ns <= 0:
            stamp_ns = self.get_clock().now().nanoseconds
        self.latest_image = (stamp_ns, bytes(msg.data), str(msg.format))

    def _pending_image_timer(self) -> None:
        self._retry_pending_pose_event()
        if not self.pending_images:
            return
        now = time.monotonic()
        remaining: list[PendingImage] = []
        for pending in self.pending_images:
            if now < pending.not_before:
                remaining.append(pending)
                continue
            if self.latest_image is None:
                if now < pending.expires_at:
                    remaining.append(pending)
                else:
                    self.get_logger().warn(
                        f'No representative image for '
                        f'{pending.detection_id}'
                    )
                continue
            image_stamp_ns, data, image_format = self.latest_image
            age_ns = abs(image_stamp_ns - pending.detection_stamp_ns)
            if age_ns > self.max_image_age_ns:
                if now < pending.expires_at:
                    remaining.append(pending)
                else:
                    self.get_logger().warn(
                        f'Representative image was stale for '
                        f'{pending.detection_id}'
                    )
                continue
            suffix = '.png' if 'png' in image_format.lower() else '.jpg'
            directory = self.image_directory / pending.mission_id
            directory.mkdir(parents=True, exist_ok=True)
            image_path = directory / f'{pending.detection_id}{suffix}'
            temporary = image_path.with_suffix(image_path.suffix + '.tmp')
            temporary.write_bytes(data)
            os.replace(temporary, image_path)
            self.store.attach_detection_image(
                pending.detection_id,
                pending.image_id,
                image_path,
                image_stamp_ns,
            )
            self.get_logger().info(
                f'Saved representative image: {image_path}'
            )
        self.pending_images = remaining

    def _retry_pending_pose_event(self) -> None:
        pending = self.pending_pose_event
        if pending is None:
            return
        if time.monotonic() >= pending.expires_at:
            self.get_logger().error(
                'Detection event expired without a valid map pose; '
                'no false coordinate was stored'
            )
            self.pending_pose_event = None
            return
        sample = self._record_current_pose(force=True)
        if sample is None:
            return
        self.pending_pose_event = None
        self._create_detection(
            pending.topic,
            sample,
            yolo_snapshot=pending.yolo_payload,
            detected_at_ns=pending.detected_at_ns,
        )

    def _start_new_mission_service(
        self,
        _request: Trigger.Request,
        response: Trigger.Response,
    ) -> Trigger.Response:
        self._ensure_mission(force_new=True)
        response.success = True
        response.message = f'Started mission {self.current_mission_id}'
        self._publish_status()
        return response

    def _end_mission_service(
        self,
        _request: Trigger.Request,
        response: Trigger.Response,
    ) -> Trigger.Response:
        ended = self._finish_current_mission(
            status='COMPLETED',
            end_reason='OPERATOR_REQUESTED',
        )
        if ended is None:
            response.success = False
            response.message = 'No active mission'
            return response
        response.success = True
        response.message = f'Ended mission {ended}'
        self._publish_path()
        self._publish_status()
        return response

    def _finish_current_mission(
        self,
        status: str,
        end_reason: str,
    ) -> str | None:
        ended = self.store.end_active_mission(
            self.get_clock().now().nanoseconds,
            status=status,
            end_reason=end_reason,
        )
        if ended is None:
            return None
        self.current_mission_id = None
        self.last_sample = None
        self.last_route_seq = -1
        self.segment_id = 0
        self.last_record_monotonic = None
        self.path_dirty = True
        self.awaiting_navigation_goal = self.current_mode == 4
        self._publish_path()
        self._publish_status()
        self.get_logger().info(
            f'Ended mission {ended}: {status}/{end_reason}'
        )
        return ended

    def _path_timer(self) -> None:
        if not self.path_dirty:
            return
        self._publish_path()

    def _publish_path(self) -> None:
        message = Path()
        message.header.stamp = self.get_clock().now().to_msg()
        message.header.frame_id = self.map_frame_id
        if self.current_mission_id is None:
            self.path_pub.publish(message)
            self.path_dirty = False
            return
        points = self.store.route_points(
            self.current_mission_id,
            self.path_history_limit,
        )
        for point in points:
            pose = PoseStamped()
            pose.header.frame_id = self.map_frame_id
            pose.header.stamp = rclpy.time.Time(
                nanoseconds=int(point['stamp_ns'])
            ).to_msg()
            pose.pose.position.x = float(point['x'])
            pose.pose.position.y = float(point['y'])
            z, w = yaw_to_quaternion(float(point['yaw']))
            pose.pose.orientation.z = z
            pose.pose.orientation.w = w
            message.poses.append(pose)
        self.path_pub.publish(message)
        self.path_dirty = False

    def _status_timer(self) -> None:
        if self.detection_gate.tick(time.monotonic()):
            self.get_logger().info(
                'Detection event recording re-armed by local fallback'
            )
        self._publish_status()

    def _publish_status(self) -> None:
        payload = {
            'state': (
                'recording'
                if self.recording_enabled and self.current_mission_id
                else 'paused'
            ),
            'mission_id': self.current_mission_id,
            'robot_id': self.robot_id,
            'mode': self.current_mode,
            'recording_enabled': self.recording_enabled,
            'map_id': self.manifest.map_id,
            'map_backend': self.manifest.backend,
            'frame_id': self.map_frame_id,
            'last_route_seq': self.last_route_seq,
            'segment_id': self.segment_id,
            'last_detection_id': self.last_detection_id,
            'detection_event_armed': self.detection_gate.local_armed,
            'gateway_detection_armed': self.detection_gate.gateway_armed,
            'tf_error': self.last_tf_error,
            'pending_images': len(self.pending_images),
            'pending_detection_pose': self.pending_pose_event is not None,
            'navigation_goal_id': (
                self.navigation_mission_tracker.active_goal_id
            ),
            'navigation_source': (
                self.navigation_mission_tracker.active_source
            ),
            'awaiting_navigation_goal': self.awaiting_navigation_goal,
            'pending': self.store.pending_counts(),
            'database_path': str(self.database_path),
        }
        self.status_pub.publish(
            String(
                data=json.dumps(
                    payload,
                    ensure_ascii=False,
                    separators=(',', ':'),
                    sort_keys=True,
                )
            )
        )


def main(args: list[str] | None = None) -> None:
    rclpy.init(args=args)
    node: MissionDataRecorderNode | None = None
    try:
        node = MissionDataRecorderNode()
        rclpy.spin(node)
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    finally:
        if node is not None:
            node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
