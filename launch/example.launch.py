#!/usr/bin/python3

import os
from datetime import datetime

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import ExecuteProcess
from launch_ros.actions import Node


def generate_launch_description():
    package_name = "kelo_tulip"
    robot_name = os.environ.get("ROBOT_NAME", "example")

    config_path = os.path.join(
        get_package_share_directory(package_name),
        "config",
        robot_name + ".yaml"
    )

    # Create a timestamp for this recording
    timestamp = datetime.now().strftime("%Y-%m-%d_%H-%M-%S")

    # Rosbag output directory
    bag_dir = os.path.join(
        os.path.expanduser("~/.ros/bag"),
        timestamp
    )

    bag_record = ExecuteProcess(
        cmd=[
            "ros2", "bag", "record",
            "-a",
            "--output", bag_dir,
            "--max-bag-size", "104857600",
        ],
        output="screen",
    )

    return LaunchDescription([
        Node(
            package="kelo_tulip",
            executable="platform_driver",
            name="platform_driver",
            output="screen",
            parameters=[config_path]
        ),

        bag_record,
    ])

