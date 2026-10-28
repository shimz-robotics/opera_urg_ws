#!/usr/bin/env python3
"""世界座標の点群から直線を探し、中点を配信する。"""

import math
import random
import struct
from functools import partial

import rclpy
from geometry_msgs.msg import PoseStamped
from rclpy.node import Node
from rclpy.qos import HistoryPolicy, QoSProfile, ReliabilityPolicy
from sensor_msgs.msg import PointCloud2, PointField


def orient_line(direction, max_angle, axis):
    """axis の正方向に近い向きへ揃える。その向きが軸から max_angle を超える場合は None。"""
    ux, uy = direction
    if axis == "x":
        if ux < 0.0:
            ux, uy = -ux, -uy
        angle = math.atan2(uy, ux)
    elif axis == "y":
        if uy < 0.0:
            ux, uy = -ux, -uy
        angle = math.atan2(-ux, uy)
    else:
        raise ValueError("axis must be x or y")
    if abs(angle) > max_angle:
        return None
    return (ux, uy), math.atan2(uy, ux)


def ransac_line(points, iterations, threshold, max_angle, axis="y"):
    best = []
    direction = None
    origin = None
    if len(points) < 2:
        return best, origin, direction
    for _ in range(iterations):
        p1, p2 = random.sample(points, 2)
        dx = p2[0] - p1[0]
        dy = p2[1] - p1[1]
        norm = math.hypot(dx, dy)
        if norm < 0.05:
            continue
        ux, uy = dx / norm, dy / norm
        oriented = orient_line((ux, uy), max_angle, axis)
        if oriented is None:
            continue
        ux, uy = oriented[0]
        inliers = []
        for point in points:
            distance = abs((point[0] - p1[0]) * uy - (point[1] - p1[1]) * ux)
            if distance <= threshold:
                inliers.append(point)
        if len(inliers) > len(best):
            best = inliers
            origin = p1
            direction = (ux, uy)
    return best, origin, direction


def segment_points(inliers, origin, direction, length, tolerance, max_gap):
    """端から端の長さが目標に近い連続点群を返す。長い直線の途中は不採用。"""
    ux, uy = direction
    projected = []
    for point in inliers:
        offset = (point[0] - origin[0]) * ux + (point[1] - origin[1]) * uy
        projected.append((offset, point))
    projected.sort(key=lambda item: item[0])
    clusters = []
    start = 0
    for index in range(1, len(projected)):
        if projected[index][0] - projected[index - 1][0] > max_gap:
            clusters.append(projected[start:index])
            start = index
    if projected:
        clusters.append(projected[start:])

    best_points = None
    best_error = None
    for cluster in clusters:
        if len(cluster) < 5:
            continue
        span = cluster[-1][0] - cluster[0][0]
        error = abs(span - length)
        if error <= tolerance and (best_error is None or error < best_error):
            best_points = [item[1] for item in cluster]
            best_error = error
    if best_points is None:
        return None
    return best_points, best_error


def fit_segment(points, max_angle, axis="y"):
    """点群の主軸を直線とし、中心はその重心にする。"""
    count = float(len(points))
    if count < 5.0:
        return None
    cx = sum(point[0] for point in points) / count
    cy = sum(point[1] for point in points) / count
    sxx = syy = sxy = 0.0
    for x, y in points:
        dx = x - cx
        dy = y - cy
        sxx += dx * dx
        syy += dy * dy
        sxy += dx * dy
    angle = 0.5 * math.atan2(2.0 * sxy, sxx - syy)
    oriented = orient_line((math.cos(angle), math.sin(angle)), max_angle, axis)
    if oriented is None:
        return None
    return (cx, cy), oriented[0]


def perpendicular_distance(point, origin, direction):
    ux, uy = direction
    return abs((point[0] - origin[0]) * uy - (point[1] - origin[1]) * ux)


def refine_segment(points, max_angle, max_distance, length, tolerance, axis="y"):
    """推定直線から離れた点を除いて引き直し、両端の中点を返す。"""
    fitted = fit_segment(points, max_angle, axis)
    if fitted is None:
        return None
    origin, direction = fitted
    kept = [
        point for point in points
        if perpendicular_distance(point, origin, direction) < max_distance
    ]
    if len(kept) < 5:
        return None
    refitted = fit_segment(kept, max_angle, axis)
    if refitted is None:
        return None
    center, refined_direction = refitted
    ux, uy = refined_direction
    offsets = [
        (point[0] - center[0]) * ux + (point[1] - center[1]) * uy
        for point in kept
    ]
    low = min(offsets)
    high = max(offsets)
    error = abs((high - low) - length)
    if error > tolerance:
        return None
    midpoint_offset = (low + high) / 2.0
    midpoint = (
        center[0] + midpoint_offset * ux,
        center[1] + midpoint_offset * uy,
    )
    return midpoint, refined_direction, kept, error


def points_off_line(points, origin, direction, threshold):
    ux, uy = direction
    ox, oy = origin
    return [
        point for point in points
        if abs((point[0] - ox) * uy - (point[1] - oy) * ux) > threshold
    ]


def angle_diff(left, right):
    return abs((left - right + math.pi) % (2.0 * math.pi) - math.pi)


def segment_cloud(points, stamp, frame_id):
    cloud = PointCloud2()
    cloud.header.stamp = stamp
    cloud.header.frame_id = frame_id
    cloud.height = 1
    cloud.width = len(points)
    cloud.fields = [
        PointField(name="x", offset=0, datatype=PointField.FLOAT32, count=1),
        PointField(name="y", offset=4, datatype=PointField.FLOAT32, count=1),
        PointField(name="z", offset=8, datatype=PointField.FLOAT32, count=1),
    ]
    cloud.is_bigendian = False
    cloud.point_step = 12
    cloud.row_step = 12 * len(points)
    cloud.is_dense = True
    cloud.data = b"".join(struct.pack("fff", float(x), float(y), 0.0) for x, y in points)
    return cloud


def cloud_points(msg, scan_range):
    """PointCloud2 の XY のうち、世界座標の範囲内だけを返す。境界は含まない。"""
    offsets = {}
    for field in msg.fields:
        if field.name in ("x", "y") and field.datatype == PointField.FLOAT32:
            offsets[field.name] = field.offset
    if "x" not in offsets or "y" not in offsets:
        return None
    data = msg.data if isinstance(msg.data, (bytes, bytearray)) else bytes(msg.data)
    points = []
    step = msg.point_step
    count = msg.width * msg.height
    for index in range(count):
        base = index * step
        x = struct.unpack_from("f", data, base + offsets["x"])[0]
        y = struct.unpack_from("f", data, base + offsets["y"])[0]
        if not (math.isfinite(x) and math.isfinite(y)):
            continue
        if scan_range["min_x"] < x < scan_range["max_x"] and scan_range["min_y"] < y < scan_range["max_y"]:
            points.append((x, y))
    return points


def line_yaw(direction):
    """+Y 側を向いた直線の yaw。基準軸の +X から反時計回り、単位はラジアン。"""
    return math.atan2(direction[1], direction[0])


def find_segments(
    points,
    length,
    tolerance,
    refine_distance,
    max_gap,
    max_angle,
    inlier_distance,
    ransac_iterations,
    angle_axis="y",
):
    """長さが近い線分を、誤差の小さい順に返す。"""
    remaining = list(points)
    found = []
    for _attempt in range(4):
        inliers, origin, direction = ransac_line(
            remaining, ransac_iterations, inlier_distance, max_angle, angle_axis)
        if direction is None or len(inliers) < 5:
            break
        segment = segment_points(
            inliers, origin, direction, length, tolerance, max_gap)
        if segment is not None:
            clustered, _error = segment
            refined = refine_segment(
                clustered, max_angle, refine_distance, length, tolerance, angle_axis)
            if refined is not None:
                center, fitted_direction, kept, error = refined
                found.append((error, center[0], center[1], fitted_direction, kept))
        remaining = points_off_line(remaining, origin, direction, inlier_distance)
        if len(remaining) < 5:
            break
    found.sort(key=lambda item: item[0])
    return found


def segment_from_direction(
    points,
    origin,
    direction,
    length,
    tolerance,
    refine_distance,
    max_gap,
    max_angle,
    inlier_distance,
    angle_axis,
):
    """既知の直線の近くから、長さが合う線分を返す。"""
    inliers = [
        point for point in points
        if perpendicular_distance(point, origin, direction) <= inlier_distance
    ]
    if len(inliers) < 5:
        return None
    segment = segment_points(
        inliers, origin, direction, length, tolerance, max_gap)
    if segment is None:
        return None
    clustered, _error = segment
    refined = refine_segment(
        clustered, max_angle, refine_distance, length, tolerance, angle_axis)
    if refined is None:
        return None
    center, fitted_direction, kept, error = refined
    return (error, center[0], center[1], fitted_direction, kept)


class LineCenter(Node):
    def __init__(self):
        super().__init__("line_center")
        self.declare_parameter("target_frame", "world")
        self.declare_parameter("min_x", -0.05)
        self.declare_parameter("max_x", 2.0)
        self.declare_parameter("min_y", -2.0)
        self.declare_parameter("max_y", 7.0)
        self.declare_parameter("line_length", 2.15)
        self.declare_parameter("length_tolerance", 0.05)
        self.declare_parameter("refine_distance", 0.020)
        self.declare_parameter("inlier_distance", 0.03)
        self.declare_parameter("max_gap", 0.15)
        self.declare_parameter("ransac_iterations", 80)
        self.declare_parameter("max_angle_from_plus_x_deg", 70.0)
        self.declare_parameter("max_center_jump", 0.3)
        self.declare_parameter("max_yaw_jump_deg", 15.0)
        self.declare_parameter("jump_reset_count", 10)
        self.declare_parameter("smooth_new_ratio", 0.2)
        self.declare_parameter(
            "input_topics", ["/urg_10/points", "/urg_11/points"])

        self.target_frame = self.get_parameter("target_frame").value
        self.scan_range = {
            "min_x": float(self.get_parameter("min_x").value),
            "max_x": float(self.get_parameter("max_x").value),
            "min_y": float(self.get_parameter("min_y").value),
            "max_y": float(self.get_parameter("max_y").value),
        }
        self.line_length = float(self.get_parameter("line_length").value)
        self.length_tolerance = float(self.get_parameter("length_tolerance").value)
        self.refine_distance = float(self.get_parameter("refine_distance").value)
        self.inlier_distance = float(self.get_parameter("inlier_distance").value)
        self.max_gap = float(self.get_parameter("max_gap").value)
        self.ransac_iterations = int(self.get_parameter("ransac_iterations").value)
        self.max_angle_from_x = math.radians(
            float(self.get_parameter("max_angle_from_plus_x_deg").value))
        self.max_center_jump = float(self.get_parameter("max_center_jump").value)
        self.max_yaw_jump = math.radians(float(self.get_parameter("max_yaw_jump_deg").value))
        self.jump_reset_count = int(self.get_parameter("jump_reset_count").value)
        self.smooth_new_ratio = float(self.get_parameter("smooth_new_ratio").value)
        self.last_pose = None
        self.smooth_pose = None
        self.jump_rejects = 0

        self.latest = {}
        qos = QoSProfile(
            reliability=ReliabilityPolicy.RELIABLE,
            history=HistoryPolicy.KEEP_LAST,
            depth=10,
        )
        for topic in list(self.get_parameter("input_topics").value):
            self.create_subscription(
                PointCloud2, topic, partial(self.on_cloud, topic), qos)
        self.publisher = self.create_publisher(PoseStamped, "/line_center", qos)
        self.points_publisher = self.create_publisher(PointCloud2, "/line_points", qos)
        box = self.scan_range
        self.get_logger().info(
            "world box: %.3f < x < %.3f, %.3f < y < %.3f"
            % (box["min_x"], box["max_x"], box["min_y"], box["max_y"])
        )

    def on_cloud(self, topic, msg):
        if msg.header.frame_id and msg.header.frame_id != self.target_frame:
            self.get_logger().warn(
                "%s frame is %s, expected %s" % (topic, msg.header.frame_id, self.target_frame),
                throttle_duration_sec=2.0,
            )
            return
        points = cloud_points(msg, self.scan_range)
        if points is None:
            self.get_logger().warn("%s has no x,y fields" % topic, throttle_duration_sec=2.0)
            return
        self.latest[topic] = points
        cloud = [point for group in self.latest.values() for point in group]
        chosen = self.select_candidate(self.line_candidates(cloud))
        if chosen is None:
            self.points_publisher.publish(
                segment_cloud([], msg.header.stamp, self.target_frame))
            return
        center_x, center_y, direction, segment = chosen
        yaw = line_yaw(direction) + math.pi / 2.0
        center_x, center_y, yaw = self.smooth_pose_value(center_x, center_y, yaw)
        out = PoseStamped()
        out.header.stamp = msg.header.stamp
        out.header.frame_id = self.target_frame
        out.pose.position.x = center_x
        out.pose.position.y = center_y
        out.pose.position.z = 0.0
        out.pose.orientation.z = math.sin(yaw / 2.0)
        out.pose.orientation.w = math.cos(yaw / 2.0)
        self.publisher.publish(out)
        self.points_publisher.publish(
            segment_cloud(segment, msg.header.stamp, self.target_frame))

    def line_candidates(self, cloud):
        if self.last_pose is not None:
            x, y, yaw = self.last_pose
            seeded = segment_from_direction(
                cloud,
                (x, y),
                (math.cos(yaw), math.sin(yaw)),
                self.line_length,
                self.length_tolerance,
                self.refine_distance,
                self.max_gap,
                self.max_angle_from_x,
                self.inlier_distance,
                "x",
            )
            if seeded is not None:
                return [seeded]
        for _trial in range(4):
            found = find_segments(
                cloud,
                self.line_length,
                self.length_tolerance,
                self.refine_distance,
                self.max_gap,
                self.max_angle_from_x,
                self.inlier_distance,
                self.ransac_iterations,
                angle_axis="x",
            )
            if found:
                return found
        return []

    def select_candidate(self, candidates):
        if not candidates:
            return None
        if self.last_pose is None:
            _error, x, y, direction, points = candidates[0]
            self.last_pose = (x, y, line_yaw(direction))
            self.jump_rejects = 0
            return x, y, direction, points
        last_x, last_y, last_yaw = self.last_pose
        for _error, x, y, direction, points in candidates:
            yaw = line_yaw(direction)
            if (math.hypot(x - last_x, y - last_y) <= self.max_center_jump
                    and angle_diff(yaw, last_yaw) <= self.max_yaw_jump):
                self.last_pose = (x, y, yaw)
                self.jump_rejects = 0
                return x, y, direction, points
        self.jump_rejects += 1
        if self.jump_rejects >= self.jump_reset_count:
            self.last_pose = None
            self.smooth_pose = None
            self.jump_rejects = 0
        return None

    def smooth_pose_value(self, x, y, yaw):
        if self.smooth_pose is None:
            self.smooth_pose = (x, y, yaw)
            return self.smooth_pose
        px, py, pyaw = self.smooth_pose
        alpha = self.smooth_new_ratio
        delta = (yaw - pyaw + math.pi) % (2.0 * math.pi) - math.pi
        self.smooth_pose = (
            (1.0 - alpha) * px + alpha * x,
            (1.0 - alpha) * py + alpha * y,
            pyaw + alpha * delta,
        )
        return self.smooth_pose


def main():
    rclpy.init()
    node = LineCenter()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
