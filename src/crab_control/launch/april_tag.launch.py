from launch import LaunchDescription
from launch_ros.actions import Node

import os
from ament_index_python.packages import get_package_share_directory


def generate_launch_description():

    config = os.path.join(
        get_package_share_directory("crab_control"),
        "config",
        "tags.yaml"
    )


    return LaunchDescription([


        Node(
            package="crab_control",
            executable="camera_node",
            name="stellar_camera",
            output="screen",
        ),


        Node(
            package="apriltag_ros",
            executable="apriltag_node",
            name="apriltag",
            output="screen",
            parameters=[
                config
            ],
            remappings=[

                # Use raw camera stream
                ("image_rect", "/camera/image_raw"),

                # Use calibration
                ("camera_info", "/camera/camera_info"),

            ],
        ),

    ])
