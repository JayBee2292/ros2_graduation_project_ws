#!/usr/bin/env python3
from __future__ import annotations

import math
import time
from collections import deque
from dataclasses import dataclass

import rclpy
from rclpy.executors import ExternalShutdownException
from action_msgs.msg import GoalStatus
from geometry_msgs.msg import PoseStamped
from nav2_msgs.action import ComputePathToPose, NavigateToPose
from nav_msgs.msg import OccupancyGrid
from rclpy.action import ActionClient
from rclpy.node import Node
from rclpy.time import Time
from std_msgs.msg import Bool
from tf2_ros import Buffer, TransformException, TransformListener


@dataclass(frozen=True)
class FrontierGoal:
    x: float
    y: float
    size: int
    distance: float
    score: float


class FrontierExplorerNode(Node):
    def __init__(self) -> None:
        super().__init__('h753_frontier_explorer')

        self.declare_parameter('map_topic', '/map')
        self.declare_parameter('map_frame_id', 'map')
        self.declare_parameter('base_frame_id', 'base_link')
        self.declare_parameter('navigate_action', '/navigate_to_pose')
        self.declare_parameter('planner_action', '/compute_path_to_pose')
        self.declare_parameter('enabled_topic', '/frontier_explorer/enabled')
        self.declare_parameter('start_enabled', False)
        self.declare_parameter('planning_period_s', 2.0)
        self.declare_parameter('planner_timeout_s', 5.0)
        self.declare_parameter('goal_timeout_s', 45.0)
        self.declare_parameter('min_frontier_cells', 5)
        self.declare_parameter('min_goal_distance_m', 0.60)
        self.declare_parameter('max_goal_distance_m', 4.0)
        self.declare_parameter('goal_clearance_m', 0.28)
        self.declare_parameter('blacklist_radius_m', 0.60)
        self.declare_parameter('blacklist_timeout_s', 180.0)
        self.declare_parameter('frontier_size_weight', 0.03)
        self.declare_parameter('free_threshold', 20)

        self.map_frame_id = str(self.get_parameter('map_frame_id').value)
        self.base_frame_id = str(self.get_parameter('base_frame_id').value)
        self.map_topic = str(self.get_parameter('map_topic').value)
        self.enabled = bool(self.get_parameter('start_enabled').value)
        self.planner_timeout_s = float(self.get_parameter('planner_timeout_s').value)
        self.goal_timeout_s = float(self.get_parameter('goal_timeout_s').value)
        self.min_frontier_cells = int(self.get_parameter('min_frontier_cells').value)
        self.min_goal_distance_m = float(self.get_parameter('min_goal_distance_m').value)
        self.max_goal_distance_m = float(self.get_parameter('max_goal_distance_m').value)
        self.goal_clearance_m = float(self.get_parameter('goal_clearance_m').value)
        self.blacklist_radius_m = float(self.get_parameter('blacklist_radius_m').value)
        self.blacklist_timeout_s = float(self.get_parameter('blacklist_timeout_s').value)
        self.frontier_size_weight = float(self.get_parameter('frontier_size_weight').value)
        self.free_threshold = int(self.get_parameter('free_threshold').value)

        self.map_msg: OccupancyGrid | None = None
        self.goal_handle = None
        self.goal: FrontierGoal | None = None
        self.goal_started_at: float | None = None
        self.goal_pending = False
        self.cancel_requested = False
        self.planner_goal_handle = None
        self.planner_candidate: FrontierGoal | None = None
        self.planner_robot_xy: tuple[float, float] | None = None
        self.planner_started_at: float | None = None
        self.planner_pending = False
        self.blacklist: list[tuple[float, float, float]] = []
        self.last_wait_log_at = 0.0
        self.map_subscription = None

        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)
        self.navigate_client = ActionClient(
            self,
            NavigateToPose,
            str(self.get_parameter('navigate_action').value),
        )
        self.planner_client = ActionClient(
            self,
            ComputePathToPose,
            str(self.get_parameter('planner_action').value),
        )
        self.create_subscription(
            Bool,
            str(self.get_parameter('enabled_topic').value),
            self._enabled_callback,
            10,
        )
        self.create_timer(
            float(self.get_parameter('planning_period_s').value),
            self._planning_timer_callback,
        )
        self._set_map_subscription(self.enabled)

        self.get_logger().info(
            f'Frontier explorer ready: enabled={self.enabled}. '
            'Publish std_msgs/Bool on /frontier_explorer/enabled to pause or resume.'
        )

    def destroy_node(self) -> bool:
        self._cancel_planner_check()
        self._cancel_active_goal()
        self.planner_client.destroy()
        self.navigate_client.destroy()
        return super().destroy_node()

    def _map_callback(self, msg: OccupancyGrid) -> None:
        self.map_msg = msg

    def _enabled_callback(self, msg: Bool) -> None:
        if self.enabled == msg.data:
            return
        self.enabled = msg.data
        self.get_logger().info(f'Frontier exploration enabled={self.enabled}')
        self._set_map_subscription(self.enabled)
        if not self.enabled:
            self._cancel_planner_check()
            self._cancel_active_goal()

    def _set_map_subscription(self, enabled: bool) -> None:
        if enabled and self.map_subscription is None:
            self.map_subscription = self.create_subscription(
                OccupancyGrid,
                self.map_topic,
                self._map_callback,
                10,
            )
            return
        if not enabled and self.map_subscription is not None:
            self.destroy_subscription(self.map_subscription)
            self.map_subscription = None
            self.map_msg = None

    def _planning_timer_callback(self) -> None:
        if not self.enabled:
            return

        if self.goal_handle is not None:
            if (
                self.goal_started_at is not None
                and time.monotonic() - self.goal_started_at > self.goal_timeout_s
                and not self.cancel_requested
            ):
                self.get_logger().warn('Exploration goal timed out; cancelling and blacklisting it')
                self._blacklist_current_goal()
                self._cancel_active_goal()
            return

        if self.goal_pending:
            return
        if self.planner_goal_handle is not None:
            if (
                self.planner_started_at is not None
                and time.monotonic() - self.planner_started_at > self.planner_timeout_s
            ):
                self.get_logger().warn('Frontier planner validation timed out; blacklisting it')
                self._blacklist_planner_candidate()
                self._cancel_planner_check()
            return
        if self.planner_pending:
            return
        if self.map_msg is None:
            self._log_waiting('Waiting for /map')
            return
        if not self.navigate_client.wait_for_server(timeout_sec=0.05):
            self._log_waiting('Waiting for Nav2 /navigate_to_pose action server')
            return
        if not self.planner_client.wait_for_server(timeout_sec=0.05):
            self._log_waiting('Waiting for Nav2 /compute_path_to_pose action server')
            return

        robot_xy = self._lookup_robot_xy()
        if robot_xy is None:
            self._log_waiting(f'Waiting for TF {self.map_frame_id} -> {self.base_frame_id}')
            return

        goal = self._choose_frontier_goal(self.map_msg, robot_xy)
        if goal is None:
            self._log_waiting('No eligible frontier found; waiting for map changes')
            return
        self._validate_goal(goal, robot_xy)

    def _lookup_robot_xy(self) -> tuple[float, float] | None:
        try:
            transform = self.tf_buffer.lookup_transform(
                self.map_frame_id,
                self.base_frame_id,
                Time(),
            )
        except TransformException:
            return None
        return (
            transform.transform.translation.x,
            transform.transform.translation.y,
        )

    def _choose_frontier_goal(
        self,
        map_msg: OccupancyGrid,
        robot_xy: tuple[float, float],
    ) -> FrontierGoal | None:
        width = int(map_msg.info.width)
        height = int(map_msg.info.height)
        resolution = float(map_msg.info.resolution)
        data = map_msg.data
        if width <= 0 or height <= 0 or resolution <= 0.0 or len(data) != width * height:
            return None

        self._prune_blacklist()
        frontier_cells = {
            index
            for index, value in enumerate(data)
            if self._is_frontier_cell(index, value, data, width, height)
        }
        clusters = self._cluster_frontiers(frontier_cells, width, height)
        clearance_cells = max(0, int(math.ceil(self.goal_clearance_m / resolution)))
        goals: list[FrontierGoal] = []

        for cluster in clusters:
            if len(cluster) < self.min_frontier_cells:
                continue
            candidate = self._cluster_goal_cell(
                cluster,
                data,
                width,
                height,
                clearance_cells,
            )
            if candidate is None:
                continue
            x, y = self._cell_to_world(map_msg, candidate)
            distance = math.hypot(x - robot_xy[0], y - robot_xy[1])
            if (
                distance < self.min_goal_distance_m
                or distance > self.max_goal_distance_m
                or self._is_blacklisted(x, y)
            ):
                continue
            goals.append(FrontierGoal(
                x=x,
                y=y,
                size=len(cluster),
                distance=distance,
                score=distance - self.frontier_size_weight * len(cluster),
            ))

        return min(goals, key=lambda goal: goal.score) if goals else None

    def _is_frontier_cell(
        self,
        index: int,
        value: int,
        data,
        width: int,
        height: int,
    ) -> bool:
        if value < 0 or value > self.free_threshold:
            return False
        row, col = divmod(index, width)
        for next_row, next_col in (
            (row - 1, col),
            (row + 1, col),
            (row, col - 1),
            (row, col + 1),
        ):
            if 0 <= next_row < height and 0 <= next_col < width:
                if data[next_row * width + next_col] < 0:
                    return True
        return False

    @staticmethod
    def _cluster_frontiers(frontier_cells: set[int], width: int, height: int) -> list[list[int]]:
        remaining = set(frontier_cells)
        clusters: list[list[int]] = []
        while remaining:
            start = remaining.pop()
            queue = deque([start])
            cluster = [start]
            while queue:
                index = queue.popleft()
                row, col = divmod(index, width)
                for next_row in range(row - 1, row + 2):
                    for next_col in range(col - 1, col + 2):
                        if not (0 <= next_row < height and 0 <= next_col < width):
                            continue
                        neighbor = next_row * width + next_col
                        if neighbor in remaining:
                            remaining.remove(neighbor)
                            queue.append(neighbor)
                            cluster.append(neighbor)
            clusters.append(cluster)
        return clusters

    def _cluster_goal_cell(
        self,
        cluster: list[int],
        data,
        width: int,
        height: int,
        clearance_cells: int,
    ) -> int | None:
        center_row = sum(index // width for index in cluster) / len(cluster)
        center_col = sum(index % width for index in cluster) / len(cluster)
        candidates = sorted(
            cluster,
            key=lambda index: (
                (index // width - center_row) ** 2 + (index % width - center_col) ** 2
            ),
        )
        for index in candidates:
            if self._has_obstacle_clearance(index, data, width, height, clearance_cells):
                return index
        return None

    @staticmethod
    def _has_obstacle_clearance(
        index: int,
        data,
        width: int,
        height: int,
        clearance_cells: int,
    ) -> bool:
        row, col = divmod(index, width)
        for next_row in range(max(0, row - clearance_cells), min(height, row + clearance_cells + 1)):
            for next_col in range(max(0, col - clearance_cells), min(width, col + clearance_cells + 1)):
                if (next_row - row) ** 2 + (next_col - col) ** 2 > clearance_cells ** 2:
                    continue
                if data[next_row * width + next_col] > 50:
                    return False
        return True

    @staticmethod
    def _cell_to_world(map_msg: OccupancyGrid, index: int) -> tuple[float, float]:
        width = int(map_msg.info.width)
        row, col = divmod(index, width)
        resolution = float(map_msg.info.resolution)
        origin = map_msg.info.origin.position
        return (
            origin.x + (col + 0.5) * resolution,
            origin.y + (row + 0.5) * resolution,
        )

    def _make_goal_pose(self, goal: FrontierGoal, robot_xy: tuple[float, float]) -> PoseStamped:
        heading = math.atan2(goal.y - robot_xy[1], goal.x - robot_xy[0])
        pose = PoseStamped()
        pose.header.stamp = self.get_clock().now().to_msg()
        pose.header.frame_id = self.map_frame_id
        pose.pose.position.x = goal.x
        pose.pose.position.y = goal.y
        pose.pose.orientation.z = math.sin(heading / 2.0)
        pose.pose.orientation.w = math.cos(heading / 2.0)
        return pose

    def _validate_goal(self, goal: FrontierGoal, robot_xy: tuple[float, float]) -> None:
        action_goal = ComputePathToPose.Goal()
        action_goal.goal = self._make_goal_pose(goal, robot_xy)
        action_goal.use_start = False
        self.planner_candidate = goal
        self.planner_robot_xy = robot_xy
        self.planner_pending = True
        self.get_logger().info(
            f'Validating frontier goal: x={goal.x:.2f} y={goal.y:.2f} '
            f'distance={goal.distance:.2f}m cells={goal.size}'
        )
        future = self.planner_client.send_goal_async(action_goal)
        future.add_done_callback(self._planner_response_callback)

    def _planner_response_callback(self, future) -> None:
        self.planner_pending = False
        try:
            goal_handle = future.result()
        except Exception as exc:  # noqa: BLE001
            self.get_logger().error(f'Failed to submit frontier planner check: {exc}')
            self._blacklist_planner_candidate()
            self._reset_planner_check()
            return
        if not goal_handle.accepted:
            self.get_logger().warn('Nav2 planner rejected frontier validation request')
            self._blacklist_planner_candidate()
            self._reset_planner_check()
            return
        if not self.enabled or self.planner_candidate is None:
            goal_handle.cancel_goal_async()
            self._reset_planner_check()
            return

        self.planner_goal_handle = goal_handle
        self.planner_started_at = time.monotonic()
        result_future = goal_handle.get_result_async()
        result_future.add_done_callback(self._planner_result_callback)

    def _planner_result_callback(self, future) -> None:
        candidate = self.planner_candidate
        robot_xy = self.planner_robot_xy
        try:
            result = future.result()
            valid_path = (
                result.status == GoalStatus.STATUS_SUCCEEDED
                and bool(result.result.path.poses)
            )
        except Exception as exc:  # noqa: BLE001
            self.get_logger().error(f'Frontier planner validation failed: {exc}')
            valid_path = False

        self._reset_planner_check()
        if candidate is None or not self.enabled:
            return
        if not valid_path:
            self.get_logger().warn(
                f'No Nav2 path to frontier x={candidate.x:.2f} y={candidate.y:.2f}; '
                'blacklisting it before navigation'
            )
            self._blacklist_goal(candidate)
            return

        if robot_xy is None:
            robot_xy = self._lookup_robot_xy()
        if robot_xy is None:
            self._log_waiting(f'Waiting for TF {self.map_frame_id} -> {self.base_frame_id}')
            return
        self._send_goal(candidate, robot_xy)

    def _send_goal(self, goal: FrontierGoal, robot_xy: tuple[float, float]) -> None:
        pose = self._make_goal_pose(goal, robot_xy)

        action_goal = NavigateToPose.Goal()
        action_goal.pose = pose
        self.goal = goal
        self.goal_pending = True
        self.cancel_requested = False
        self.get_logger().info(
            f'Sending frontier goal: x={goal.x:.2f} y={goal.y:.2f} '
            f'distance={goal.distance:.2f}m cells={goal.size}'
        )
        future = self.navigate_client.send_goal_async(action_goal)
        future.add_done_callback(self._goal_response_callback)

    def _goal_response_callback(self, future) -> None:
        self.goal_pending = False
        try:
            goal_handle = future.result()
        except Exception as exc:  # noqa: BLE001
            self.get_logger().error(f'Failed to submit exploration goal: {exc}')
            self._blacklist_current_goal()
            self.goal = None
            return
        if not goal_handle.accepted:
            self.get_logger().warn('Nav2 rejected exploration goal')
            self._blacklist_current_goal()
            self.goal = None
            return

        self.goal_handle = goal_handle
        self.goal_started_at = time.monotonic()
        if not self.enabled:
            self._cancel_active_goal()
        result_future = goal_handle.get_result_async()
        result_future.add_done_callback(self._goal_result_callback)

    def _goal_result_callback(self, future) -> None:
        try:
            status = future.result().status
        except Exception as exc:  # noqa: BLE001
            self.get_logger().error(f'Exploration goal result failed: {exc}')
            status = GoalStatus.STATUS_UNKNOWN

        if status == GoalStatus.STATUS_SUCCEEDED:
            self.get_logger().info('Exploration goal reached')
        elif status == GoalStatus.STATUS_CANCELED and not self.enabled:
            self.get_logger().info('Exploration goal cancelled because exploration is paused')
        else:
            self.get_logger().warn(f'Exploration goal ended with status={status}; blacklisting it')
            self._blacklist_current_goal()

        self.goal_handle = None
        self.goal = None
        self.goal_started_at = None
        self.cancel_requested = False

    def _cancel_active_goal(self) -> None:
        if self.goal_handle is None or self.cancel_requested:
            return
        self.cancel_requested = True
        self.goal_handle.cancel_goal_async()

    def _cancel_planner_check(self) -> None:
        if self.planner_goal_handle is not None:
            self.planner_goal_handle.cancel_goal_async()
        self._reset_planner_check()

    def _reset_planner_check(self) -> None:
        self.planner_goal_handle = None
        self.planner_candidate = None
        self.planner_robot_xy = None
        self.planner_started_at = None
        self.planner_pending = False

    def _blacklist_current_goal(self) -> None:
        if self.goal is None:
            return
        self._blacklist_goal(self.goal)

    def _blacklist_planner_candidate(self) -> None:
        if self.planner_candidate is None:
            return
        self._blacklist_goal(self.planner_candidate)

    def _blacklist_goal(self, goal: FrontierGoal) -> None:
        self.blacklist.append((
            goal.x,
            goal.y,
            time.monotonic() + self.blacklist_timeout_s,
        ))

    def _prune_blacklist(self) -> None:
        now = time.monotonic()
        self.blacklist = [
            (x, y, expires_at)
            for x, y, expires_at in self.blacklist
            if expires_at > now
        ]

    def _is_blacklisted(self, x: float, y: float) -> bool:
        return any(
            math.hypot(x - blocked_x, y - blocked_y) <= self.blacklist_radius_m
            for blocked_x, blocked_y, _expires_at in self.blacklist
        )

    def _log_waiting(self, message: str) -> None:
        now = time.monotonic()
        if now - self.last_wait_log_at < 10.0:
            return
        self.last_wait_log_at = now
        self.get_logger().info(message)


def main(args: list[str] | None = None) -> None:
    rclpy.init(args=args)
    node = FrontierExplorerNode()
    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    finally:
        try:
            node.destroy_node()
        except KeyboardInterrupt:
            pass
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
