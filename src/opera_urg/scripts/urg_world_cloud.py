#!/usr/bin/env python3
"""urg_position.yaml の位置で、URGのスキャンを世界座標の点群にして配信する。"""

import math
import os
import struct
from functools import partial

import yaml
from ament_index_python.packages import get_package_share_directory

import rclpy
from rclpy.node import Node
from rclpy.qos import HistoryPolicy, QoSProfile, ReliabilityPolicy, qos_profile_sensor_data
from sensor_msgs.msg import LaserScan, PointCloud2, PointField


def sensor_name(ip):
    return "urg_%s" % str(ip).rsplit(".", 1)[-1]


def load_positions(path):
    with open(path, encoding="utf-8") as handle:
        data = yaml.safe_load(handle) or {}
    entries = []
    for item in data.get("urgs") or []:
        entries.append({
            "ip": str(item["ip"]),
            "x": float(item["x"]),
            "y": float(item["y"]),
            "yaw": float(item["yaw"]),
        })
    if not entries:
        raise SystemExit("%s に URG の位置がありません。" % path)
    return entries


def transform_scan(msg, origin_x, origin_y, yaw):
    cos_yaw = math.cos(yaw)
    sin_yaw = math.sin(yaw)
    intensities = msg.intensities
    points = []
    for index, distance in enumerate(msg.ranges):
        if not math.isfinite(distance) or distance < msg.range_min or distance > msg.range_max:
            continue
        angle = msg.angle_min + index * msg.angle_increment
        xs = distance * math.cos(angle)
        ys = distance * math.sin(angle)
        xw = origin_x + xs * cos_yaw - ys * sin_yaw
        yw = origin_y + xs * sin_yaw + ys * cos_yaw
        intensity = float(intensities[index]) if index < len(intensities) else 0.0
        points.append((xw, yw, intensity))
    return points


def cloud_message(points, stamp):
    cloud = PointCloud2()
    cloud.header.stamp = stamp
    cloud.header.frame_id = "world"
    cloud.height = 1
    cloud.width = len(points)
    cloud.fields = [
        PointField(name="x", offset=0, datatype=PointField.FLOAT32, count=1),
        PointField(name="y", offset=4, datatype=PointField.FLOAT32, count=1),
        PointField(name="z", offset=8, datatype=PointField.FLOAT32, count=1),
        PointField(name="intensity", offset=12, datatype=PointField.FLOAT32, count=1),
    ]
    cloud.is_bigendian = False
    cloud.point_step = 16
    cloud.row_step = 16 * len(points)
    cloud.is_dense = True
    cloud.data = b"".join(
        struct.pack("ffff", float(x), float(y), 0.0, float(intensity))
        for x, y, intensity in points
    )
    return cloud


class WorldCloud(Node):
    def __init__(self):
        super().__init__("urg_world_cloud")
        share = get_package_share_directory("opera_urg")
        default_path = os.path.join(share, "config", "urg_position.yaml")
        self.declare_parameter("position_file", default_path)
        path = self.get_parameter("position_file").value
        positions = load_positions(path)
        qos = QoSProfile(
            reliability=ReliabilityPolicy.RELIABLE,
            history=HistoryPolicy.KEEP_LAST,
            depth=10,
        )
        for entry in positions:
            name = sensor_name(entry["ip"])
            publisher = self.create_publisher(PointCloud2, "/%s/points" % name, qos)
            self.create_subscription(
                LaserScan,
                "/%s/scan" % name,
                partial(self.on_scan, entry, publisher),
                qos_profile_sensor_data,
            )
            self.get_logger().info(
                "%s -> /%s/points  world (%.3f, %.3f, yaw %.4f)"
                % (entry["ip"], name, entry["x"], entry["y"], entry["yaw"])
            )

    def on_scan(self, entry, publisher, msg):
        points = transform_scan(msg, entry["x"], entry["y"], entry["yaw"])
        publisher.publish(cloud_message(points, msg.header.stamp))


def main():
    rclpy.init()
    node = WorldCloud()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
