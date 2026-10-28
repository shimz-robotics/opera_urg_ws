import os

import yaml
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node


def sensor_name(ip):
    return "urg_%s" % str(ip).rsplit(".", 1)[-1]


def urg_node(name, ip, frame):
    return Node(
        package="urg_node",
        executable="urg_node_driver",
        name=name,
        namespace=name,
        output="screen",
        parameters=[{
            "ip_address": ip,
            "ip_port": 10940,
            "laser_frame_id": frame,
            "angle_min": -3.14,
            "angle_max": 3.14,
            "cluster": 1,
            "skip": 0,
            "calibrate_time": False,
            "publish_intensity": True,
            "publish_multiecho": False,
        }],
    )


def static_tf(child, x, y, yaw):
    return Node(
        package="tf2_ros",
        executable="static_transform_publisher",
        name="%s_tf" % child,
        arguments=[
            "--x", str(x),
            "--y", str(y),
            "--z", "0",
            "--yaw", str(yaw),
            "--pitch", "0",
            "--roll", "0",
            "--frame-id", "world",
            "--child-frame-id", child,
        ],
    )


def generate_launch_description():
    share = get_package_share_directory("opera_urg")
    position_path = os.path.join(share, "config", "urg_position.yaml")
    with open(position_path, encoding="utf-8") as position_file:
        positions = yaml.safe_load(position_file) or {}
    urgs = positions.get("urgs") or []
    if not urgs:
        raise RuntimeError("%s に URG の位置がありません。" % position_path)

    nodes = []
    for entry in urgs:
        ip = str(entry["ip"])
        name = sensor_name(ip)
        frame = "laser_%s" % ip.rsplit(".", 1)[-1]
        nodes.append(static_tf(frame, entry["x"], entry["y"], entry["yaw"]))
        nodes.append(urg_node(name, ip, frame))
    nodes.append(Node(
        package="opera_urg",
        executable="urg_world_cloud.py",
        name="urg_world_cloud",
        output="screen",
        parameters=[{"position_file": position_path}],
    ))
    return LaunchDescription(nodes)
