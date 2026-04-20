from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    return LaunchDescription([
        Node(
            package='data_comm_analysis',
            executable='data_comm_analysis_node',
            name='data_comm_analysis_node',
            output='screen',
            parameters=[{
                'serial_port': '/dev/ttyS3',
                'baud_rate': 500000,
            }],
        )
    ])
