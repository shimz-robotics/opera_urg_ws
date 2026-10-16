#!/usr/bin/env python3
"""範囲内の2台分の点から約2mの直線を探し、中点を配信する。"""

import math
import random
import struct
from functools import partial

import rclpy
from geometry_msgs.msg import PoseStamped
from rclpy.node import Node
from rclpy.qos import HistoryPolicy, QoSProfile, ReliabilityPolicy
from rclpy.time import Time
from sensor_msgs.msg import LaserScan, PointCloud2, PointField
from tf2_ros import Buffer, TransformException, TransformListener


def rotate_xy(qx, qy, qz, qw, x, y):
    """Rotate a point in the XY plane by a quaternion."""
    cx = -qz * y
    cy = qz * x
    cz = qx * y - qy * x
    x += 2.0 * (qw * cx + qy * cz - qz * cy)
    y += 2.0 * (qw * cy + qz * cx - qx * cz)
    return x, y


def orient_toward_plus_y(direction, max_angle):
    """+Y に近い向きへ揃える。その向きが +Y から max_angle を超える場合は None。"""
    ux, uy = direction
    if uy < 0.0:
        ux, uy = -ux, -uy
    angle_from_y = math.atan2(-ux, uy)
    if abs(angle_from_y) > max_angle:
        return None
    return (ux, uy), math.atan2(uy, ux)


def ransac_line(points, iterations, threshold, max_angle_from_y):
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
        oriented = orient_toward_plus_y((ux, uy), max_angle_from_y)
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


def fit_segment(points, max_angle_from_y):
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
    oriented = orient_toward_plus_y((math.cos(angle), math.sin(angle)), max_angle_from_y)
    if oriented is None:
        return None
    return (cx, cy), oriented[0]


def perpendicular_distance(point, origin, direction):
    ux, uy = direction
    return abs((point[0] - origin[0]) * uy - (point[1] - origin[1]) * ux)


def refine_segment(points, max_angle_from_y, max_distance, length, tolerance):
    """推定直線から離れた点を除いて引き直し、両端の中点を返す。"""
    fitted = fit_segment(points, max_angle_from_y)
    if fitted is None:
        return None
    origin, direction = fitted
    kept = [
        point for point in points
        if perpendicular_distance(point, origin, direction) < max_distance
    ]
    if len(kept) < 5:
        return None
    refitted = fit_segment(kept, max_angle_from_y)
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


def line_yaw(direction):
    """+Y 側を向いた直線の yaw。base_link の X 軸から反時計回り、単位はラジアン。"""
    return math.atan2(direction[1], direction[0])


class LineCenter(Node):
    def __init__(self):
        super().__init__("line_center")
        self.declare_parameter("target_frame", "base_link")
        self.declare_parameter("line_length", 2.15)
        self.declare_parameter("length_tolerance", 0.05)
        self.declare_parameter("refine_distance", 0.020)
        self.declare_parameter("inlier_distance", 0.03)
        self.declare_parameter("max_gap", 0.15)
        self.declare_parameter("ransac_iterations", 80)
        self.declare_parameter("max_angle_from_plus_y_deg", 70.0)
        self.declare_parameter("max_center_jump", 0.3)
        self.declare_parameter("max_yaw_jump_deg", 15.0)
        self.declare_parameter("jump_reset_count", 10)
        self.declare_parameter("smooth_new_ratio", 0.2)
        self.declare_parameter(
            "input_topics", ["/urg_10/scan_filtered", "/urg_11/scan_filtered"])

        self.target_frame = self.get_parameter("target_frame").value
        self.line_length = float(self.get_parameter("line_length").value)
        self.length_tolerance = float(self.get_parameter("length_tolerance").value)
        self.refine_distance = float(self.get_parameter("refine_distance").value)
        self.inlier_distance = float(self.get_parameter("inlier_distance").value)
        self.max_gap = float(self.get_parameter("max_gap").value)
        self.ransac_iterations = int(self.get_parameter("ransac_iterations").value)
        self.max_angle_from_y = math.radians(
            float(self.get_parameter("max_angle_from_plus_y_deg").value))
        self.max_center_jump = float(self.get_parameter("max_center_jump").value)
        self.max_yaw_jump = math.radians(float(self.get_parameter("max_yaw_jump_deg").value))
        self.jump_reset_count = int(self.get_parameter("jump_reset_count").value)
        self.smooth_new_ratio = float(self.get_parameter("smooth_new_ratio").value)
        self.last_pose = None
        self.smooth_pose = None
        self.jump_rejects = 0

        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)
        self.latest = {}
        qos = QoSProfile(
            reliability=ReliabilityPolicy.RELIABLE,
            history=HistoryPolicy.KEEP_LAST,
            depth=10,
        )
        for topic in list(self.get_parameter("input_topics").value):
            self.create_subscription(
                LaserScan, topic, partial(self.on_scan, topic), qos)
        self.publisher = self.create_publisher(PoseStamped, "/line_center", qos)
        self.points_publisher = self.create_publisher(PointCloud2, "/line_points", qos)

    def on_scan(self, topic, msg):
        points = self.scan_points(msg)
        if points is None:
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
        remaining = list(cloud)
        found = []
        for _attempt in range(4):
            inliers, origin, direction = ransac_line(
                remaining,
                self.ransac_iterations,
                self.inlier_distance,
                self.max_angle_from_y,
            )
            if direction is None or len(inliers) < 5:
                break
            segment = segment_points(
                inliers,
                origin,
                direction,
                self.line_length,
                self.length_tolerance,
                self.max_gap,
            )
            if segment is not None:
                points, _error = segment
                refined = refine_segment(
                    points,
                    self.max_angle_from_y,
                    self.refine_distance,
                    self.line_length,
                    self.length_tolerance,
                )
                if refined is not None:
                    center, fitted_direction, kept, error = refined
                    found.append((error, center[0], center[1], fitted_direction, kept))
            remaining = points_off_line(
                remaining, origin, direction, self.inlier_distance)
            if len(remaining) < 5:
                break
        found.sort(key=lambda item: item[0])
        return found

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

    def scan_points(self, msg):
        try:
            transform = self.tf_buffer.lookup_transform(
                self.target_frame, msg.header.frame_id, Time())
        except TransformException:
            return None
        translation = transform.transform.translation
        rotation = transform.transform.rotation
        points = []
        for index, distance in enumerate(msg.ranges):
            if not math.isfinite(distance):
                continue
            angle = msg.angle_min + index * msg.angle_increment
            x = distance * math.cos(angle)
            y = distance * math.sin(angle)
            x, y = rotate_xy(rotation.x, rotation.y, rotation.z, rotation.w, x, y)
            points.append((x + translation.x, y + translation.y))
        return points


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
