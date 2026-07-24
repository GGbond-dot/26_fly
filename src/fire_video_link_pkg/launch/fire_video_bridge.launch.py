"""跨域视频桥（飞机侧）。

单独起：
    ros2 launch fire_video_link_pkg fire_video_bridge.launch.py

改共享域（不改配置文件）：
    ros2 launch fire_video_link_pkg fire_video_bridge.launch.py outer_domain:=10

注意：本节点自己给两个 Context 显式设域，所以进程的 ROS_DOMAIN_ID 环境变量
对转发行为没有影响；但 ROS_LOCALHOST_ONLY 必须是 0，否则共享域出不了本机。
"""

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue


def generate_launch_description():
    default_cfg = os.path.join(
        get_package_share_directory('fire_video_link_pkg'),
        'config', 'video_bridge.yaml')

    config_file = LaunchConfiguration('config_file')
    inner_domain = LaunchConfiguration('inner_domain')
    outer_domain = LaunchConfiguration('outer_domain')

    return LaunchDescription([
        DeclareLaunchArgument('config_file', default_value=default_cfg,
                              description='桥接规则 yaml'),
        DeclareLaunchArgument('inner_domain', default_value='26',
                              description='飞机内部 ROS_DOMAIN_ID'),
        DeclareLaunchArgument('outer_domain', default_value='6',
                              description='消防车所在 ROS_DOMAIN_ID（共享域）'),

        Node(
            package='fire_video_link_pkg',
            executable='fire_video_bridge_node',
            # 节点名必须和 yaml 里的键一致，否则参数加载不进来
            name='fire_video_bridge_cfg',
            output='screen',
            emulate_tty=True,
            parameters=[
                config_file,
                # launch 参数替换出来是字符串，必须显式声明 int，
                # 否则 declare_parameter<int> 会抛 InvalidParameterType
                {
                    'inner_domain': ParameterValue(inner_domain, value_type=int),
                    'outer_domain': ParameterValue(outer_domain, value_type=int),
                },
            ],
        ),
    ])
