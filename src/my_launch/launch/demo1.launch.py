import os
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.substitutions import FindPackageShare

def generate_launch_description():
    # 1. 获取包路径
    my_carto_pkg_share = FindPackageShare(package='my_carto_pkg').find('my_carto_pkg')
    uart_to_stm32_pkg_share = FindPackageShare(package='uart_to_stm32').find('uart_to_stm32')
    data_comm_analysis_pkg_share = FindPackageShare(package='data_comm_analysis').find('data_comm_analysis')
    pid_control_pkg_share = FindPackageShare(package='pid_control_pkg').find('pid_control_pkg')
    activity_control_pkg_share = FindPackageShare(package='activity_control_pkg').find('activity_control_pkg')
    pillar_detector_pkg_share = FindPackageShare(package='pillar_detector_pkg').find('pillar_detector_pkg')
    
    # 2. 定义 Launch 文件包含
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

    data_comm_analysis_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(data_comm_analysis_pkg_share, 'launch', 'data_comm_analysis.launch.py')
        )
    )
    
    position_pid_controller_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(pid_control_pkg_share, 'launch', 'position_pid_controller.launch.py')
        )
    )   

    route_test_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(activity_control_pkg_share, 'launch', 'route_test.launch.py')
        )
    )

    pillar_detector_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(pillar_detector_pkg_share, 'launch', 'pillar_detector.launch.py')
        )
    )

    launch_items = [
        fly_carto_launch, # 立即启动
        uart_to_stm32_launch,
        data_comm_analysis_launch,
        position_pid_controller_launch,
        route_test_launch,
        pillar_detector_launch,
    ]

    return LaunchDescription(launch_items)
