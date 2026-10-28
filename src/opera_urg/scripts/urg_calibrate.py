#!/usr/bin/env python3
"""センサ座標のスキャンから線分を抽出し、URGの世界座標を求める。"""

import math
import os
import time
from functools import partial

import yaml
from ament_index_python.packages import get_package_share_directory

import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import LaserScan

from line_center import find_segments, line_yaw


INLIER_DISTANCE = 0.03
RANSAC_ITERATIONS = 80


def sensor_name(ip):
    return "urg_%s" % str(ip).rsplit(".", 1)[-1]


def wrap_angle(angle):
    return (angle + math.pi) % (2.0 * math.pi) - math.pi


def require(mapping, key, path):
    if key not in mapping:
        raise SystemExit("%s に %s がありません。" % (path, key))
    return mapping[key]


def load_yaml(path):
    with open(path, encoding="utf-8") as handle:
        data = yaml.safe_load(handle)
    if not isinstance(data, dict):
        raise SystemExit("%s の内容を読めません。" % path)
    return data


def load_calibration(calib_path, line_path):
    calib = load_yaml(calib_path)
    line_file = load_yaml(line_path)
    duration = float(require(calib, "duration_sec", calib_path))
    line = require(calib, "line", calib_path)
    urgs = require(calib, "urgs", calib_path)
    world = require(line_file, "line_world", line_path)
    if duration <= 0.0:
        raise SystemExit("duration_sec は正の数にしてください。")
    length = float(require(line, "length", calib_path))
    tolerance = float(require(line, "length_tolerance", calib_path))
    refine_distance = float(require(line, "refine_distance", calib_path))
    max_angle_deg = float(require(line, "max_angle_from_plus_y_deg", calib_path))
    max_gap = float(require(line, "max_gap", calib_path))
    if length <= 0.0 or tolerance < 0.0 or refine_distance <= 0.0 or max_gap <= 0.0:
        raise SystemExit("線分の長さ、許容幅、垂直距離、点間隔の設定を確認してください。")
    if not 0.0 < max_angle_deg <= 180.0:
        raise SystemExit("max_angle_from_plus_y_deg は 0 より大きく 180 以下にしてください。")
    if not isinstance(urgs, list) or not urgs:
        raise SystemExit("urgs に対象の URG を書いてください。")
    sensors = []
    seen = set()
    for index, urg in enumerate(urgs):
        ip = str(require(urg, "ip", calib_path))
        if ip in seen:
            raise SystemExit("IP %s が重複しています。" % ip)
        seen.add(ip)
        region = require(urg, "scan_range", calib_path)
        scan_range = {
            "min_x": float(require(region, "min_x", calib_path)),
            "max_x": float(require(region, "max_x", calib_path)),
            "min_y": float(require(region, "min_y", calib_path)),
            "max_y": float(require(region, "max_y", calib_path)),
        }
        if scan_range["min_x"] >= scan_range["max_x"] or scan_range["min_y"] >= scan_range["max_y"]:
            raise SystemExit("%s の抽出範囲が逆になっています。" % ip)
        sensors.append({"ip": ip, "scan_range": scan_range, "index": index})
    line_world = {
        "x": float(require(world, "x", line_path)),
        "y": float(require(world, "y", line_path)),
        "yaw": wrap_angle(float(require(world, "yaw", line_path))),
    }
    return {
        "duration_sec": duration,
        "length": length,
        "tolerance": tolerance,
        "refine_distance": refine_distance,
        "max_angle_from_y": math.radians(max_angle_deg),
        "max_gap": max_gap,
        "sensors": sensors,
        "line_world": line_world,
    }


def points_in_range(msg, scan_range):
    points = []
    for index, distance in enumerate(msg.ranges):
        if not math.isfinite(distance) or distance < msg.range_min or distance > msg.range_max:
            continue
        angle = msg.angle_min + index * msg.angle_increment
        x = distance * math.cos(angle)
        y = distance * math.sin(angle)
        if scan_range["min_x"] < x < scan_range["max_x"] and scan_range["min_y"] < y < scan_range["max_y"]:
            points.append((x, y))
    return points


def median(values):
    ordered = sorted(values)
    mid = len(ordered) // 2
    if len(ordered) % 2 == 1:
        return ordered[mid]
    return 0.5 * (ordered[mid - 1] + ordered[mid])


def circular_mean(angles):
    sin_sum = sum(math.sin(angle) for angle in angles)
    cos_sum = sum(math.cos(angle) for angle in angles)
    return math.atan2(sin_sum, cos_sum)


def sensor_pose(line_world, center_x, center_y, line_in_sensor):
    """既知の世界線分と、センサ座標で見た線分から URG の位置と X 軸の向きを求める。"""
    yaw = wrap_angle(line_world["yaw"] - line_in_sensor)
    cos_yaw = math.cos(yaw)
    sin_yaw = math.sin(yaw)
    x = line_world["x"] - (center_x * cos_yaw - center_y * sin_yaw)
    y = line_world["y"] - (center_x * sin_yaw + center_y * cos_yaw)
    return x, y, yaw


def format_angle(yaw):
    return "%.4f rad (%.2f deg)" % (yaw, math.degrees(yaw))


def load_saved_positions(path):
    if not os.path.exists(path):
        return []
    data = load_yaml(path)
    entries = data.get("urgs") or []
    saved = []
    for entry in entries:
        saved.append({
            "ip": str(entry["ip"]),
            "x": float(entry["x"]),
            "y": float(entry["y"]),
            "yaw": float(entry["yaw"]),
        })
    return saved


def write_positions(path, entries):
    lines = [
        "# x, y はメートル。yaw は URG の X 軸が世界の +X となす角。反時計回りが正。",
        "# 算出プログラムが同意後にこのファイルを更新する。",
        "urgs:",
    ]
    for entry in entries:
        lines.append("  - ip: \"%s\"" % entry["ip"])
        lines.append("    x: %.6f" % entry["x"])
        lines.append("    y: %.6f" % entry["y"])
        lines.append("    yaw: %.6f" % entry["yaw"])
    with open(path, "w", encoding="utf-8") as handle:
        handle.write("\n".join(lines) + "\n")


class ScanCollector(Node):
    def __init__(self, settings):
        super().__init__("urg_calibrate")
        self.settings = settings
        self.counts = {}
        self.samples = {}
        for sensor in settings["sensors"]:
            ip = sensor["ip"]
            topic = "/%s/scan" % sensor_name(ip)
            self.counts[ip] = 0
            self.samples[ip] = []
            self.create_subscription(
                LaserScan,
                topic,
                partial(self.collect, sensor),
                qos_profile_sensor_data,
            )
            self.get_logger().info("購読: %s" % topic)

    def collect(self, sensor, msg):
        ip = sensor["ip"]
        self.counts[ip] += 1
        points = points_in_range(msg, sensor["scan_range"])
        found = find_segments(
            points,
            self.settings["length"],
            self.settings["tolerance"],
            self.settings["refine_distance"],
            self.settings["max_gap"],
            self.settings["max_angle_from_y"],
            INLIER_DISTANCE,
            RANSAC_ITERATIONS,
        )
        if not found:
            return
        _error, x, y, direction, _kept = found[0]
        self.samples[ip].append((x, y, line_yaw(direction)))


def summarize(settings, node):
    results = []
    print("")
    print(
        "世界線分: 中心 (%.3f, %.3f) m, 向き %s"
        % (settings["line_world"]["x"], settings["line_world"]["y"], format_angle(settings["line_world"]["yaw"]))
    )
    for sensor in settings["sensors"]:
        ip = sensor["ip"]
        count = node.counts[ip]
        samples = node.samples[ip]
        print("")
        print(ip)
        print("  受信スキャン: %d" % count)
        print("  線分を抽出したスキャン: %d" % len(samples))
        if not samples:
            print("  線分を抽出できませんでした。この URG は書き込みません。")
            continue
        center_x = median([sample[0] for sample in samples])
        center_y = median([sample[1] for sample in samples])
        direction = circular_mean([sample[2] for sample in samples])
        x, y, yaw = sensor_pose(settings["line_world"], center_x, center_y, direction)
        print("  センサ座標の線分: 中心 (%.3f, %.3f) m, 向き %s" % (center_x, center_y, format_angle(direction)))
        print("  世界座標の URG: 中心 (%.3f, %.3f) m, X軸 %s" % (x, y, format_angle(yaw)))
        results.append({"ip": ip, "x": x, "y": y, "yaw": yaw})
    return results


def merge_positions(previous, results):
    updated = {entry["ip"]: entry for entry in previous}
    for entry in results:
        updated[entry["ip"]] = entry
    order = [entry["ip"] for entry in results]
    for entry in previous:
        if entry["ip"] not in order:
            order.append(entry["ip"])
    return [updated[ip] for ip in order]


def main():
    share = get_package_share_directory("opera_urg")
    config = os.path.join(share, "config")
    calib_path = os.path.join(config, "urg_calib.yaml")
    line_path = os.path.join(config, "line_world.yaml")
    position_path = os.path.join(config, "urg_position.yaml")
    settings = load_calibration(calib_path, line_path)

    rclpy.init()
    node = ScanCollector(settings)
    deadline = time.monotonic() + settings["duration_sec"]
    next_report = time.monotonic() + 1.0
    results = []
    try:
        while time.monotonic() < deadline and rclpy.ok():
            rclpy.spin_once(node, timeout_sec=0.1)
            now = time.monotonic()
            if now >= next_report:
                parts = ["%s %d" % (sensor["ip"], node.counts[sensor["ip"]]) for sensor in settings["sensors"]]
                print("受信スキャン: " + ", ".join(parts), flush=True)
                next_report = now + 1.0
        results = summarize(settings, node)
    except KeyboardInterrupt:
        print("中断しました。")
    finally:
        node.destroy_node()
        rclpy.shutdown()

    if not results:
        print("書き込む結果がありません。")
        return
    answer = input("urg_position.yaml に書き込みますか? [y/N]: ").strip()
    if answer not in ("y", "Y"):
        print("書き込みませんでした。")
        return
    entries = merge_positions(load_saved_positions(position_path), results)
    write_positions(position_path, entries)
    print("書き込みました: %s" % position_path)
    print("配信側を起動し直すと、新しい位置が使われます。")


if __name__ == "__main__":
    main()
