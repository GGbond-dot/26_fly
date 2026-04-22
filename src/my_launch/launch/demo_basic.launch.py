import os
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    """demo_basic：2026 杭电电赛 X 题（搬运飞行器）——基础要求。

    任务：中心起飞 → 爬升 → 对角巡航 → 对角起停区降落；记录总用时。
    航点（cm，yaw deg）：
        (0,0,13,0) → (0,0,150,0) → (250,-250,150,0) → (250,-250,13,0)
    对应 route_target_publisher.cpp 中的 RouteId=3，
    通过向 /route_choice 发布 UInt8=3 触发。

    全程高度闭环使用面阵激光（抗高台），因此：
      - PID 用 position_pid_controller_ground（z 轴重映射到 /laser_array/ground_height）
      - 启动 laser_array_ground 发布 /laser_array/ground_height
    不启动 visual / pillar_detector（发挥部分才用）。
    """

    my_carto_pkg_share           = FindPackageShare(package='my_carto_pkg').find('my_carto_pkg')
    uart_to_stm32_pkg_share      = FindPackageShare(package='uart_to_stm32').find('uart_to_stm32')
    data_comm_analysis_pkg_share = FindPackageShare(package='data_comm_analysis').find('data_comm_analysis')
    pid_control_pkg_share        = FindPackageShare(package='pid_control_pkg').find('pid_control_pkg')
    activity_control_pkg_share   = FindPackageShare(package='activity_control_pkg').find('activity_control_pkg')
    laser_array_pkg_share        = FindPackageShare(package='laser_array_pkg').find('laser_array_pkg')

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

    # z 轴走面阵激光（抗高台版 PID）
    position_pid_controller_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(pid_control_pkg_share, 'launch', 'position_pid_controller_ground.launch.py')
        )
    )

    # 面阵激光地面高度（发布 /laser_array/ground_height）
    laser_array_ground_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(laser_array_pkg_share, 'launch', 'laser_array_ground.launch.py')
        )
    )

    # 航点发布：订阅 /route_choice，发 3 即加载基础赛对角航线（RouteId=3）
    route_test_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(activity_control_pkg_share, 'launch', 'route_test.launch.py')
        )
    )

    return LaunchDescription([
        fly_carto_launch,
        uart_to_stm32_launch,
        data_comm_analysis_launch,
        laser_array_ground_launch,
        position_pid_controller_launch,
        route_test_launch,
    ])
