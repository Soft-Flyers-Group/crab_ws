from launch import LaunchDescription
from launch.actions import ExecuteProcess, DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    bag_name = LaunchConfiguration('bag_name')

    return LaunchDescription([
        DeclareLaunchArgument(
            'bag_name',
            default_value='test_bag'
        ),

        ExecuteProcess(
            cmd=[
                'ros2', 'bag', 'record',
                '-o', bag_name,
                '/load_cell_data',
                '/servo/encoder_data',
                '/servo/position_data'
            ],
            output='screen'
        ),

        Node(
            package='crab_control',
            executable='load_cell_node',
            output='screen'
        ),

        Node(
            package='crab_control',
            executable='servo_node',
            output='screen'
        ),

        Node(
            package='crab_control',
            executable='servo_controller',
            output='screen'
        ),
    ])