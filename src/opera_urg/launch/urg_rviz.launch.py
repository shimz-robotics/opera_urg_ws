import os

import yaml
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node


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


def static_tf(child, x="0", y="0", yaw="0"):
    return Node(
        package="tf2_ros",
        executable="static_transform_publisher",
        name=f"{child}_tf",
        arguments=[
            "--x", x,
            "--y", y,
            "--z", "0",
            "--yaw", yaw,
            "--pitch", "0",
            "--roll", "0",
            "--frame-id", "base_link",
            "--child-frame-id", child,
        ],
    )


def generate_launch_description():
    share = get_package_share_directory("opera_urg")
    rviz = os.path.join(share, "rviz", "urg.rviz")
    setup_path = os.path.join(share, "config", "urg_setup.yaml")
    with open(setup_path, encoding="utf-8") as setup_file:
        setup = yaml.safe_load(setup_file)
    urg_11 = setup["urg_11"]
    scan_range = setup["scan_range"]
    line = setup["line"]
    line_filter = setup["line_filter"]

    return LaunchDescription([
        static_tf("laser_10"),
        static_tf(
            "laser_11",
            x=str(urg_11["x"]),
            y=str(urg_11["y"]),
            yaw=str(urg_11["yaw"]),
        ),
        urg_node("urg_10", "192.168.0.10", "laser_10"),
        urg_node("urg_11", "192.168.0.11", "laser_11"),
        Node(
            package="opera_urg",
            executable="scan_box_filter.py",
            name="scan_box_filter",
            output="screen",
            parameters=[{
                "target_frame": "base_link",
                "min_x": float(scan_range["min_x"]),
                "max_x": float(scan_range["max_x"]),
                "min_y": float(scan_range["min_y"]),
                "max_y": float(scan_range["max_y"]),
            }],
        ),
        Node(
            package="opera_urg",
            executable="line_center.py",
            name="line_center",
            output="screen",
            parameters=[{
                "line_length": float(line["length"]),
                "length_tolerance": float(line["length_tolerance"]),
                "refine_distance": float(line["refine_distance"]),
                "max_center_jump": float(line_filter["max_center_jump"]),
                "max_yaw_jump_deg": float(line_filter["max_yaw_jump_deg"]),
                "jump_reset_count": int(line_filter["jump_reset_count"]),
                "smooth_new_ratio": float(line_filter["smooth_new_ratio"]),
            }],
        ),
        Node(
            package="rviz2",
            executable="rviz2",
            name="rviz2",
            arguments=["-d", rviz],
            output="screen",
        ),
    ])
