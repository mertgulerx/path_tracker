#!/usr/bin/env python3
"""
Copyright 2026 Mert Guler

Licensed under the Apache License, Version 2.0 (the "License");
you may not use this file except in compliance with the License.
You may obtain a copy of the License at

    http://www.apache.org/licenses/LICENSE-2.0

Unless required by applicable law or agreed to in writing, software
distributed under the License is distributed on an "AS IS" BASIS,
WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
See the License for the specific language governing permissions and
limitations under the License.
"""

import math

from geometry_msgs.msg import PoseStamped
from nav_msgs.msg import Path
import rclpy
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy
from rclpy.qos import HistoryPolicy
from rclpy.qos import QoSProfile
from rclpy.qos import ReliabilityPolicy
from rclpy.time import Time
from std_msgs.msg import Empty as EmptyMsg
from std_srvs.srv import Empty as EmptySrv
from tf2_ros import Buffer
from tf2_ros import TransformException
from tf2_ros import TransformListener


class PathTracker(Node):
    def __init__(self) -> None:
        super().__init__("path_tracker")

        self.declare_parameter("global_frame", "map")
        self.declare_parameter("robot_base_frame", "base_footprint")
        self.declare_parameter("path_topic", "/path_tracker/path")
        self.declare_parameter("initial_pose_topic", "/path_tracker/initial_pose")
        self.declare_parameter("reset_topic", "/path_tracker/reset")
        self.declare_parameter("update_rate_hz", 5.0)
        self.declare_parameter("min_translation_delta_m", 0.05)

        self.global_frame = str(self.get_parameter("global_frame").value)
        self.robot_base_frame = str(self.get_parameter("robot_base_frame").value)
        self.path_topic = str(self.get_parameter("path_topic").value)
        self.initial_pose_topic = str(self.get_parameter("initial_pose_topic").value)
        self.reset_topic = str(self.get_parameter("reset_topic").value)
        self.update_rate_hz = float(self.get_parameter("update_rate_hz").value)
        self.min_translation_delta_m = float(
            self.get_parameter("min_translation_delta_m").value
        )

        if self.update_rate_hz <= 0.0:
            raise RuntimeError("update_rate_hz must be greater than 0.0")
        if self.min_translation_delta_m < 0.0:
            raise RuntimeError("min_translation_delta_m must be non-negative")

        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)

        path_qos = QoSProfile(
            history=HistoryPolicy.KEEP_LAST,
            depth=1,
            reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.TRANSIENT_LOCAL,
        )

        self.path_publisher = self.create_publisher(Path, self.path_topic, path_qos)
        self.initial_pose_publisher = self.create_publisher(
            PoseStamped, self.initial_pose_topic, path_qos
        )
        self.reset_subscriber = self.create_subscription(
            EmptyMsg, self.reset_topic, self._on_reset_topic, 10
        )
        self.reset_service = self.create_service(
            EmptySrv, "~/reset_path", self._on_reset_service
        )

        self.path_msg = Path()
        self.path_msg.header.frame_id = self.global_frame
        self.last_pose: PoseStamped | None = None
        self.initial_pose: PoseStamped | None = None
        self.last_lookup_warning_ns = 0

        self.timer = self.create_timer(1.0 / self.update_rate_hz, self._record_pose)

        self.get_logger().info(
            "Path tracker enabled: "
            f"frame='{self.global_frame}', "
            f"base='{self.robot_base_frame}', "
            f"path_topic='{self.path_topic}', "
            f"initial_pose_topic='{self.initial_pose_topic}', "
            f"reset_topic='{self.reset_topic}', "
            f"rate={self.update_rate_hz:.2f} Hz"
        )

    def _reset_path(self) -> None:
        self.last_pose = None
        self.initial_pose = None
        self.path_msg = Path()
        self.path_msg.header.frame_id = self.global_frame
        self.path_msg.header.stamp = self.get_clock().now().to_msg()
        self.path_publisher.publish(self.path_msg)
        self.get_logger().info("Tracked robot path reset.")

    def _on_reset_topic(self, _: EmptyMsg) -> None:
        self._reset_path()

    def _on_reset_service(
        self, request: EmptySrv.Request, response: EmptySrv.Response
    ) -> EmptySrv.Response:
        del request
        self._reset_path()
        return response

    def _record_pose(self) -> None:
        try:
            transform = self.tf_buffer.lookup_transform(
                self.global_frame,
                self.robot_base_frame,
                Time(),
            )
        except TransformException as exc:
            self._warn_lookup_failure(str(exc))
            return

        pose = PoseStamped()
        pose.header = transform.header
        pose.header.frame_id = self.global_frame
        pose.pose.position.x = transform.transform.translation.x
        pose.pose.position.y = transform.transform.translation.y
        pose.pose.position.z = transform.transform.translation.z
        pose.pose.orientation = transform.transform.rotation

        if not self._should_append_pose(pose):
            return

        if self.initial_pose is None:
            self.initial_pose = pose
            self.initial_pose_publisher.publish(self.initial_pose)

        self.last_pose = pose
        self.path_msg.header.stamp = pose.header.stamp
        self.path_msg.poses.append(pose)
        self.path_publisher.publish(self.path_msg)

    def _should_append_pose(self, pose: PoseStamped) -> bool:
        if self.last_pose is None:
            return True

        dx = pose.pose.position.x - self.last_pose.pose.position.x
        dy = pose.pose.position.y - self.last_pose.pose.position.y
        dz = pose.pose.position.z - self.last_pose.pose.position.z
        distance = math.sqrt(dx * dx + dy * dy + dz * dz)
        return distance >= self.min_translation_delta_m

    def _warn_lookup_failure(self, message: str) -> None:
        now_ns = self.get_clock().now().nanoseconds
        if now_ns - self.last_lookup_warning_ns < 5_000_000_000:
            return
        self.last_lookup_warning_ns = now_ns
        self.get_logger().warn(
            "Waiting for TF transform "
            f"'{self.global_frame}' -> '{self.robot_base_frame}': {message}"
        )


def main(args=None) -> None:
    rclpy.init(args=args)
    node = PathTracker()
    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
