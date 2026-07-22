from launch import LaunchDescription
from launch.actions import (
    ExecuteProcess,
    DeclareLaunchArgument,
    RegisterEventHandler,
    TimerAction,
    OpaqueFunction,
)
from launch.event_handlers import OnProcessStart
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

import os
import shutil


def copy_controller(context):
    bag_dir = LaunchConfiguration("bag_name").perform(context)

    src = "/home/vmookim/crab_ws/src/crab_control/crab_control/servo_controller.py"
    dst = os.path.join(bag_dir, "servo_controller.py")

    shutil.copy2(src, dst)

    print(f"Copied controller to {dst}")

    return []


def generate_launch_description():

    bag_name = LaunchConfiguration("bag_name")

    bag_record = ExecuteProcess(
        cmd=[
            "ros2", "bag", "record",
            "-o", bag_name,
            "--topics",
            "/load_cell_data",
            "/servo/encoder_data",
            "/servo/position_data",
        ],
        output="screen",
    )

    return LaunchDescription([

        DeclareLaunchArgument(
            "bag_name",
            default_value="test_bag"
        ),

        bag_record,

        RegisterEventHandler(
            OnProcessStart(
                target_action=bag_record,
                on_start=[
                    TimerAction(
                        period=2.0,
                        actions=[
                            OpaqueFunction(function=copy_controller)
                        ]
                    )
                ],
            )
        ),

        Node(
            package="crab_control",
            executable="load_cell_node",
            output="screen"
        ),

        Node(
            package="crab_control",
            executable="servo_node",
            output="screen"
        ),

        Node(
            package="crab_control",
            executable="servo_controller",
            output="screen"
        ),
    ])