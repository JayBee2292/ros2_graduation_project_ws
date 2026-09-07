#!/usr/bin/env python3
from __future__ import annotations

import json
import os
import time
from concurrent.futures import Future, ThreadPoolExecutor
from dataclasses import dataclass
from typing import Any

import rclpy
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy
from std_msgs.msg import String
from std_srvs.srv import Trigger

from h753_can_odom.mission_data_core import (
    MissionStore,
    OutboxEvent,
)
from h753_can_odom.mission_upload_core import (
    MissionApiClient,
    MissionUploadError,
)


@dataclass(frozen=True)
class UploadJob:
    kind: str
    event: OutboxEvent | None = None
    mission_id: str = ''
    points: tuple[dict[str, Any], ...] = ()


class MissionUploaderNode(Node):
    """Upload durable Jetson outbox entries without blocking ROS callbacks."""

    def __init__(self) -> None:
        super().__init__('h753_mission_uploader')

        self.declare_parameter(
            'database_path',
            '/home/jyl1015/.ros/h753_mission/mission_outbox.db',
        )
        self.declare_parameter('api_base_url', '')
        self.declare_parameter('allow_insecure_http', False)
        self.declare_parameter(
            'api_key_environment',
            'H753_MISSION_API_KEY',
        )
        self.declare_parameter('api_key_header', 'X-API-Key')
        self.declare_parameter(
            'map_upload_path',
            '/api/v1/maps:upload',
        )
        self.declare_parameter('map_yaml_field', 'yaml_file')
        self.declare_parameter('map_image_field', 'pgm_file')
        self.declare_parameter('request_timeout_s', 5.0)
        self.declare_parameter('poll_period_s', 0.5)
        self.declare_parameter('route_batch_size', 100)
        self.declare_parameter('retry_initial_s', 1.0)
        self.declare_parameter('retry_max_s', 60.0)
        self.declare_parameter(
            'status_topic',
            '/mission/upload/status',
        )
        self.declare_parameter(
            'retry_blocked_service',
            '/mission/upload/retry_blocked',
        )

        self.database_path = str(
            self.get_parameter('database_path').value
        )
        self.api_base_url = str(
            self.get_parameter('api_base_url').value
        ).strip()
        self.allow_insecure_http = bool(
            self.get_parameter('allow_insecure_http').value
        )
        self.request_timeout_s = float(
            self.get_parameter('request_timeout_s').value
        )
        self.poll_period_s = float(
            self.get_parameter('poll_period_s').value
        )
        self.route_batch_size = int(
            self.get_parameter('route_batch_size').value
        )
        self.retry_initial_s = float(
            self.get_parameter('retry_initial_s').value
        )
        self.retry_max_s = float(
            self.get_parameter('retry_max_s').value
        )
        api_key_environment = str(
            self.get_parameter('api_key_environment').value
        )
        api_key_header = str(
            self.get_parameter('api_key_header').value
        )
        map_upload_path = str(
            self.get_parameter('map_upload_path').value
        )
        map_yaml_field = str(
            self.get_parameter('map_yaml_field').value
        )
        map_image_field = str(
            self.get_parameter('map_image_field').value
        )
        token = os.environ.get(api_key_environment, '')

        if self.poll_period_s <= 0.0:
            raise ValueError('poll_period_s must be positive')
        if self.route_batch_size <= 0:
            raise ValueError('route_batch_size must be positive')
        if self.retry_initial_s <= 0.0:
            raise ValueError('retry_initial_s must be positive')
        if self.retry_max_s < self.retry_initial_s:
            raise ValueError('retry_max_s must be >= retry_initial_s')

        self.store = MissionStore(self.database_path)
        self.client: MissionApiClient | None = None
        if self.api_base_url:
            self.client = MissionApiClient(
                self.api_base_url,
                token=token,
                api_key_header=api_key_header,
                map_upload_path=map_upload_path,
                map_yaml_field=map_yaml_field,
                map_image_field=map_image_field,
                timeout_s=self.request_timeout_s,
                allow_insecure_http=self.allow_insecure_http,
            )
        self.executor = ThreadPoolExecutor(
            max_workers=1,
            thread_name_prefix='mission-upload',
        )
        self.future: Future[dict[str, Any]] | None = None
        self.active_job: UploadJob | None = None
        self.last_error = ''
        self.last_error_retryable: bool | None = None
        self.last_http_status: int | None = None
        self.last_success_at_ns: int | None = None
        self.sent_items = 0
        self.route_failures = 0
        self.route_retry_not_before = 0.0

        latched_qos = QoSProfile(depth=1)
        latched_qos.reliability = ReliabilityPolicy.RELIABLE
        latched_qos.durability = DurabilityPolicy.TRANSIENT_LOCAL
        self.status_pub = self.create_publisher(
            String,
            str(self.get_parameter('status_topic').value),
            latched_qos,
        )
        self.retry_blocked_service = self.create_service(
            Trigger,
            str(self.get_parameter('retry_blocked_service').value),
            self._retry_blocked,
        )
        self.create_timer(self.poll_period_s, self._poll)
        self.create_timer(1.0, self._publish_status)
        self._publish_status()
        if self.client is None:
            self.get_logger().info(
                'Mission uploader disabled: api_base_url is empty; '
                'records remain durable in the Jetson outbox'
            )
        else:
            self.get_logger().info(
                f'Mission uploader ready: {self.api_base_url}'
            )

    def destroy_node(self) -> bool:
        self.executor.shutdown(wait=False, cancel_futures=True)
        self.store.close()
        return super().destroy_node()

    def _poll(self) -> None:
        if self.client is None:
            return
        if self.future is not None:
            if not self.future.done():
                return
            self._finish_job()
        if self.future is not None:
            return

        event = self.store.next_outbox_event(time.time_ns())
        route = self.store.pending_route_batch(self.route_batch_size)
        if event is not None and event.event_type in (
            'map',
            'mission_start',
            'detection_image',
        ):
            self._start_event_job(event)
            return
        if event is not None and event.event_type == 'detection':
            route_end_seq = int(event.payload.get('route_end_seq', -1))
            if not self.store.has_unsent_route_through(
                event.mission_id,
                route_end_seq,
            ):
                self._start_event_job(event)
                return
        route_retry_ready = (
            time.monotonic() >= self.route_retry_not_before
        )
        if route is not None and route_retry_ready:
            mission_id, points = route
            self._start_route_job(mission_id, points)
            return
        if event is not None and event.event_type != 'detection':
            self._start_event_job(event)

    def _start_event_job(self, event: OutboxEvent) -> None:
        if self.client is None:
            return
        self.active_job = UploadJob(kind='event', event=event)
        self.future = self.executor.submit(self.client.upload_event, event)

    def _start_route_job(
        self,
        mission_id: str,
        points: list[dict[str, Any]],
    ) -> None:
        if self.client is None:
            return
        immutable_points = tuple(points)
        self.active_job = UploadJob(
            kind='route',
            mission_id=mission_id,
            points=immutable_points,
        )
        self.future = self.executor.submit(
            self.client.upload_route_batch,
            mission_id,
            list(immutable_points),
        )

    def _finish_job(self) -> None:
        if self.future is None or self.active_job is None:
            return
        future = self.future
        job = self.active_job
        self.future = None
        self.active_job = None
        try:
            acknowledgement = future.result()
        except Exception as exc:
            self.last_error = str(exc)
            permanent = (
                isinstance(exc, MissionUploadError)
                and not exc.retryable
            )
            self.last_error_retryable = not permanent
            self.last_http_status = (
                exc.status_code
                if isinstance(exc, MissionUploadError)
                else None
            )
            if permanent and job.kind == 'event' and job.event is not None:
                self.store.mark_outbox_blocked(
                    job.event.event_key,
                    self.last_error,
                )
            elif permanent:
                sequences = [
                    int(point['route_seq']) for point in job.points
                ]
                self.store.mark_route_batch_blocked(
                    job.mission_id,
                    sequences,
                    self.last_error,
                )
                self.route_failures = 0
                self.route_retry_not_before = 0.0
            elif job.kind == 'event' and job.event is not None:
                attempt = job.event.attempts + 1
                delay = min(
                    self.retry_max_s,
                    self.retry_initial_s * (2 ** min(attempt - 1, 10)),
                )
                self.store.mark_outbox_failed(
                    job.event.event_key,
                    self.last_error,
                    time.time_ns() + int(delay * 1_000_000_000),
                )
            else:
                self.route_failures += 1
                delay = min(
                    self.retry_max_s,
                    self.retry_initial_s
                    * (2 ** min(self.route_failures - 1, 10)),
                )
                self.route_retry_not_before = time.monotonic() + delay
            if permanent:
                self.get_logger().error(
                    'Mission upload blocked pending operator review: '
                    f'{exc}'
                )
            else:
                self.get_logger().warn(
                    f'Mission upload retry scheduled: {exc}'
                )
            self._publish_status()
            return

        sent_at_ns = time.time_ns()
        self.last_success_at_ns = sent_at_ns
        self.last_error = ''
        self.last_error_retryable = None
        self.last_http_status = None
        if job.kind == 'event' and job.event is not None:
            self.store.mark_outbox_sent(
                job.event.event_key,
                sent_at_ns,
                acknowledgement,
            )
            self.sent_items += 1
        else:
            sequences = [int(point['route_seq']) for point in job.points]
            self.store.mark_route_batch_sent(job.mission_id, sequences)
            self.sent_items += len(sequences)
            self.route_failures = 0
            self.route_retry_not_before = 0.0
        self._publish_status()

    def _retry_blocked(
        self,
        _request: Trigger.Request,
        response: Trigger.Response,
    ) -> Trigger.Response:
        count = self.store.retry_blocked()
        self.last_error = ''
        self.last_error_retryable = None
        self.last_http_status = None
        self.route_failures = 0
        self.route_retry_not_before = 0.0
        response.success = count > 0
        response.message = (
            f'Requeued {count} blocked upload item(s)'
            if count > 0
            else 'No blocked upload items'
        )
        self._publish_status()
        return response

    def _publish_status(self) -> None:
        pending = self.store.pending_counts()
        if self.client is None:
            state = 'disabled_local_only'
        elif self.future is not None:
            state = 'uploading'
        elif pending['outbox_blocked'] or pending['route_blocked']:
            state = 'blocked'
        elif self.last_error:
            state = 'retry_wait'
        elif pending['outbox'] or pending['route_points']:
            state = 'pending'
        else:
            state = 'synced'
        payload = {
            'state': state,
            'api_base_url': self.api_base_url,
            'pending': pending,
            'sent_items': self.sent_items,
            'last_success_at_ns': self.last_success_at_ns,
            'last_error': self.last_error,
            'last_error_retryable': self.last_error_retryable,
            'last_http_status': self.last_http_status,
            'active_job': (
                None if self.active_job is None else self.active_job.kind
            ),
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
    node: MissionUploaderNode | None = None
    try:
        node = MissionUploaderNode()
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
