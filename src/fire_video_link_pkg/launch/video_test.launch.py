"""视频链路一键测试（飞机侧）—— 相机 + 检测 + 跨域桥，一条命令全起。

    ros2 launch fire_video_link_pkg video_test.launch.py

车端同时跑：
    ros2 launch fire_video_receiver_pkg video_receiver.launch.py

看车端窗口有没有画面即可。桥每 5 秒打一行吞吐，是判断"飞机发没发出去"的唯一依据。

没接相机时加 fake:=true，改发测试图（画面上有帧号，卡没卡一眼看得出）：
    ros2 launch fire_video_link_pkg video_test.launch.py fake:=true
"""

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, ExecuteProcess, IncludeLaunchDescription
from launch.conditions import IfCondition, UnlessCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue


def generate_launch_description():
    pkg_share = get_package_share_directory('fire_video_link_pkg')
    vision_launch = os.path.join(
        get_package_share_directory('fire_vision_pkg'),
        'launch', 'fire_vision.launch.py')

    fake = LaunchConfiguration('fake')
    tune = LaunchConfiguration('tune')
    outer_domain = LaunchConfiguration('outer_domain')

    return LaunchDescription([
        DeclareLaunchArgument('fake', default_value='false',
                              description='true=不开相机，发带帧号的测试图'),
        DeclareLaunchArgument('tune', default_value='false',
                              description='true=弹相机调参窗（滑条+SAVE），需要 DISPLAY'),
        DeclareLaunchArgument('outer_domain', default_value='6',
                              description='消防车所在域'),

        # 真视觉：相机 + 火源检测 + 压缩调试图
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(vision_launch),
            condition=UnlessCondition(fake),
        ),

        # 假图：没相机时用，画面上有帧号，能一眼看出卡没卡
        ExecuteProcess(
            cmd=['python3', os.path.join(pkg_share, 'scripts', 'video_link_test.py'),
                 '--publish', '--rate', '10'],
            output='screen',
            condition=IfCondition(fake),
        ),

        # 相机调参窗：拖滑条实时调曝光/gain，点 SAVE 固化到 ~/.ros/fire_vision_camera.yaml
        ExecuteProcess(
            cmd=['python3', os.path.join(pkg_share, 'scripts', 'camera_tune.py')],
            output='screen',
            condition=IfCondition(tune),
        ),

        # 跨域桥：把压缩图从域 26 搬到域 6
        Node(
            package='fire_video_link_pkg',
            executable='fire_video_bridge_node',
            name='fire_video_bridge_cfg',
            output='screen',
            emulate_tty=True,
            parameters=[
                os.path.join(pkg_share, 'config', 'video_bridge.yaml'),
                {'outer_domain': ParameterValue(outer_domain, value_type=int)},
            ],
        ),
    ])
