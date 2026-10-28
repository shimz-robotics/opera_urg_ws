import os

import yaml
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    share = get_package_share_directory("opera_urg")
    rviz = os.path.join(share, "rviz", "urg_world.rviz")
    setup_path = os.path.join(share, "config", "urg_setup.yaml")
    with open(setup_path, encoding="utf-8") as setup_file:
        setup = yaml.safe_load(setup_file)
    scan_range = setup["scan_range"]
    line = setup["line"]
    line_filter = setup["line_filter"]

    return LaunchDescription([
        Node(
            package="opera_urg",
            executable="line_center.py",
            name="line_center",
            output="screen",
            parameters=[{
                "target_frame": "world",
                "input_topics": ["/urg_10/points", "/urg_11/points"],
                "min_x": float(scan_range["min_x"]),
                "max_x": float(scan_range["max_x"]),
                "min_y": float(scan_range["min_y"]),
                "max_y": float(scan_range["max_y"]),
                "line_length": float(line["length"]),
                "length_tolerance": float(line["length_tolerance"]),
                "refine_distance": float(line["refine_distance"]),
                "max_angle_from_plus_x_deg": float(line["max_angle_from_plus_x_deg"]),
                "max_gap": float(line["max_gap"]),
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
