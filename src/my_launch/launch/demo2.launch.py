import os
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    # 1. 获取包路径
    my_carto_pkg_share = FindPackageShare(package='my_carto_pkg').find('my_carto_pkg')
    uart_to_stm32_pkg_share = FindPackageShare(package='uart_to_stm32').find('uart_to_stm32')
    pid_control_pkg_share = FindPackageShare(package='pid_control_pkg').find('pid_control_pkg')
    activity_control_pkg_share = FindPackageShare(package='activity_control_pkg').find('activity_control_pkg')
    pillar_detector_pkg_share = FindPackageShare(package='pillar_detector_pkg').find('pillar_detector_pkg')
    visual_pkg_share = FindPackageShare(package='visual_pkg').find('visual_pkg')
    laser_array_pkg_share = FindPackageShare(package='laser_array_pkg').find('laser_array_pkg')

    # 2. Launch 文件
    fly_carto_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(my_carto_pkg_share, 'launch', 'fly_carto.launch.py')
        )
    )
    uart_to_stm32_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(uart_to_stm32_pkg_share, 'launch', 'uart_to_stm32.launch.py')
        )
    )
    position_pid_controller_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(pid_control_pkg_share, 'launch', 'position_pid_controller.launch.py')
        )
    )

    # ── 替换为飞行中检测相关节点（不再启动 route_test / 原 pillar_detector） ──
    pillar_scan_mission_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(activity_control_pkg_share, 'launch', 'pillar_scan_mission.launch.py')
        )
    )
    pillar_detector_tf_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(pillar_detector_pkg_share, 'launch', 'pillar_detector_tf.launch.py')
        )
    )

    visual_pkg_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(visual_pkg_share, 'launch', 'visual_pkg.launch.py')
        )
    )

    # 面阵激光地面高度（抗高台版），发布 /laser_array/ground_height
    # 注意：与 laser_array.launch.py 里的原 driver 互斥，同一时间只能起一个
    laser_array_ground_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(laser_array_pkg_share, 'launch', 'laser_array_ground.launch.py')
        )
    )

    launch_items = [
        fly_carto_launch,
        uart_to_stm32_launch,
        position_pid_controller_launch,
        pillar_scan_mission_launch,
        pillar_detector_tf_launch,
        visual_pkg_launch,
        laser_array_ground_launch,
    ]

    return LaunchDescription(launch_items)
