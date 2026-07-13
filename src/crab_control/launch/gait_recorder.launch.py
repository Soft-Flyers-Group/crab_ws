from launch import LaunchDescription
from launch.actions import ExecuteProcess, DeclareLaunchArgument, TimerAction
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    bag_name = LaunchConfiguration('bag_name')

    return LaunchDescription([
        DeclareLaunchArgument(
            'bag_name',
            default_value='test_bag'
        ),

        # Start immediately
        Node(
            package='crab_control',
            executable='load_cell_node',
            output='screen'
        ),

        # Wait 5 seconds, then start everything else
        TimerAction(
            period=5.0,
            actions=[
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
                    executable='servo_node',
                    output='screen'
                ),

                Node(
                    package='crab_control',
                    executable='servo_controller',
                    output='screen'
                ),
            ]
        ),
    ])