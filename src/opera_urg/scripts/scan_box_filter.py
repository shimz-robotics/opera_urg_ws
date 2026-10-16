#!/usr/bin/env python3
"""Keep LaserScan returns inside an axis-aligned box in a target frame."""

import math
from functools import partial

import rclpy
from rclpy.node import Node
from rclpy.qos import HistoryPolicy, QoSProfile, ReliabilityPolicy, qos_profile_sensor_data
from rclpy.time import Time
from sensor_msgs.msg import LaserScan
from tf2_ros import Buffer, TransformException, TransformListener


def rotate_xy(qx, qy, qz, qw, x, y):
    """Rotate a point in the XY plane by a quaternion."""
    cx = qy * 0.0 - qz * y
    cy = qz * x - qx * 0.0
    cz = qx * y - qy * x
    x += 2.0 * (qw * cx + qy * cz - qz * cy)
    y += 2.0 * (qw * cy + qz * cx - qx * cz)
    return x, y


class ScanBoxFilter(Node):
    def __init__(self):
        super().__init__("scan_box_filter")
        self.declare_parameter("target_frame", "base_link")
        self.declare_parameter("min_x", 0.1)
        self.declare_parameter("max_x", 2.0)
        self.declare_parameter("min_y", -2.0)
        self.declare_parameter("max_y", 7.0)
        self.declare_parameter("input_topics", ["/urg_10/scan", "/urg_11/scan"])

        self.target_frame = self.get_parameter("target_frame").value
        self.min_x = float(self.get_parameter("min_x").value)
        self.max_x = float(self.get_parameter("max_x").value)
        self.min_y = float(self.get_parameter("min_y").value)
        self.max_y = float(self.get_parameter("max_y").value)

        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)
        self.publishers_ = {}
        # RViz2 の LaserScan は Reliable で購読する。Best Effort だと点が出ない。
        filtered_qos = QoSProfile(
            reliability=ReliabilityPolicy.RELIABLE,
            history=HistoryPolicy.KEEP_LAST,
            depth=10,
        )

        topics = list(self.get_parameter("input_topics").value)
        for topic in topics:
            self.publishers_[topic] = self.create_publisher(
                LaserScan, topic + "_filtered", filtered_qos)
            self.create_subscription(
                LaserScan,
                topic,
                partial(self.on_scan, topic),
                qos_profile_sensor_data,
            )
        self.get_logger().info(
            "box in %s: %.3f < x < %.3f, %.3f < y < %.3f"
            % (self.target_frame, self.min_x, self.max_x, self.min_y, self.max_y)
        )

    def on_scan(self, topic, msg: LaserScan):
        try:
            transform = self.tf_buffer.lookup_transform(
                self.target_frame, msg.header.frame_id, Time())
        except TransformException as exc:
            self.get_logger().warn(
                "no transform %s -> %s: %s" % (msg.header.frame_id, self.target_frame, exc),
                throttle_duration_sec=2.0,
            )
            return

        translation = transform.transform.translation
        rotation = transform.transform.rotation
        ranges = list(msg.ranges)
        for index, distance in enumerate(ranges):
            if not math.isfinite(distance) or distance < msg.range_min or distance > msg.range_max:
                continue
            angle = msg.angle_min + index * msg.angle_increment
            x = distance * math.cos(angle)
            y = distance * math.sin(angle)
            x, y = rotate_xy(rotation.x, rotation.y, rotation.z, rotation.w, x, y)
            x += translation.x
            y += translation.y
            if not (self.min_x < x < self.max_x and self.min_y < y < self.max_y):
                ranges[index] = float("nan")

        filtered = LaserScan()
        filtered.header = msg.header
        filtered.angle_min = msg.angle_min
        filtered.angle_max = msg.angle_max
        filtered.angle_increment = msg.angle_increment
        filtered.time_increment = msg.time_increment
        filtered.scan_time = msg.scan_time
        filtered.range_min = msg.range_min
        filtered.range_max = msg.range_max
        filtered.ranges = ranges
        filtered.intensities = msg.intensities
        self.publishers_[topic].publish(filtered)


def main():
    rclpy.init()
    node = ScanBoxFilter()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
