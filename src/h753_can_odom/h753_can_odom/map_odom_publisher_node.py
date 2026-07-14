#!/usr/bin/env python3
from __future__ import annotations

import rclpy
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from tf2_ros import Buffer, TransformException, TransformListener


class MapOdomPublisher(Node):
    def __init__(self) -> None:
        super().__init__('h753_map_odom_publisher')

        self.declare_parameter('map_frame_id', 'map')
        self.declare_parameter('base_frame_id', 'base_link')
        self.declare_parameter('odom_topic', 'map_odom')
        self.declare_parameter('twist_topic', 'odom_vel')
        self.declare_parameter('publish_rate_hz', 5.0)

        self.map_frame_id = self.get_parameter('map_frame_id').value
        self.base_frame_id = self.get_parameter('base_frame_id').value
        self.latest_twist = Twist()
        self.warned_missing_tf = False

        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)
        self.odom_pub = self.create_publisher(
            Odometry,
            self.get_parameter('odom_topic').value,
            20,
        )
        self.create_subscription(
            Twist,
            self.get_parameter('twist_topic').value,
            self._twist_callback,
            20,
        )
        self.timer = self.create_timer(
            1.0 / float(self.get_parameter('publish_rate_hz').value),
            self._timer_callback,
        )

    def _twist_callback(self, msg: Twist) -> None:
        self.latest_twist = msg

    def _timer_callback(self) -> None:
        try:
            transform = self.tf_buffer.lookup_transform(
                self.map_frame_id,
                self.base_frame_id,
                rclpy.time.Time(),
            )
        except TransformException as exc:
            if not self.warned_missing_tf:
                self.get_logger().warn(
                    f'Waiting for TF {self.map_frame_id}->{self.base_frame_id}: {exc}'
                )
                self.warned_missing_tf = True
            return

        self.warned_missing_tf = False
        odom = Odometry()
        odom.header.stamp = transform.header.stamp
        odom.header.frame_id = self.map_frame_id
        odom.child_frame_id = self.base_frame_id
        odom.pose.pose.position.x = transform.transform.translation.x
        odom.pose.pose.position.y = transform.transform.translation.y
        odom.pose.pose.position.z = transform.transform.translation.z
        odom.pose.pose.orientation = transform.transform.rotation
        odom.twist.twist = self.latest_twist
        odom.pose.covariance[0] = 0.02
        odom.pose.covariance[7] = 0.02
        odom.pose.covariance[35] = 0.05
        odom.twist.covariance[0] = 0.05
        odom.twist.covariance[35] = 0.10
        self.odom_pub.publish(odom)


def main(args: list[str] | None = None) -> None:
    rclpy.init(args=args)
    node = MapOdomPublisher()
    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
