from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    return LaunchDescription([
        Node(
            package='spray_control_pkg',
            executable='spray_mission_node',
            name='spray_mission_node',
            output='screen',
            parameters=[{
                'map_frame': 'map',
                'base_frame': 'base_link',
                'flight_height_cm': 150.0,
                'home_x_cm': 0.0,
                'home_y_cm': 0.0,
                'spray_blink_count': 2,
                # TODO: 实测后填入起点区块 A(21) 与全覆盖绿色区块坐标
                # 'block_a_x_cm': 75.0,
                # 'block_a_y_cm': 350.0,
                # 'green_blocks': [id, x_cm, y_cm, ...]  # 蛇形顺序
            }],
        ),
    ])
