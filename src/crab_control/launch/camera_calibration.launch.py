from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():

    return LaunchDescription([

        Node(
            package="crab_control",
            executable="camera_node",
            output="screen",
        ),

        Node(
            package="camera_calibration",
            executable="cameracalibrator",
            output="screen",
            arguments=[
                "--size", "7x10",
                "--square", "0.015",
                "--ros-args",
                "-r", "image:=/camera/image_raw",
            ],
        ),
    ])