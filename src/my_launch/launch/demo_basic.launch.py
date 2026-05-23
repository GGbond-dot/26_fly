import os
from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument,
    ExecuteProcess,
    IncludeLaunchDescription,
    LogInfo,
    TimerAction,
)
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    """demo_basic：2026 杭电电赛 X 题（搬运飞行器）——基础要求。

    任务：中心起飞 → 爬升 → 对角巡航 → 对角起停区降落；记录总用时。
    起始态 (0,0,13,0)（中心、贴地待命），航点序列（cm，yaw deg）：
        (0,0,150,0) → (250,-250,150,0) → (250,-250,13,0)
    对应 route_target_publisher.cpp 中的 RouteId=3，
    通过向 /route_choice 发布 UInt8=3 触发。

    全程高度闭环使用面阵激光（抗高台），因此：
      - PID 用 position_pid_controller_ground（z 轴重映射到 /laser_array/ground_height）
      - 启动 laser_array_ground 发布 /laser_array/ground_height
    不启动 visual / pillar_detector（发挥部分才用）。
    """

    my_carto_pkg_share           = FindPackageShare(package='my_carto_pkg').find('my_carto_pkg')
    uart_to_stm32_pkg_share      = FindPackageShare(package='uart_to_stm32').find('uart_to_stm32')
    pid_control_pkg_share        = FindPackageShare(package='pid_control_pkg').find('pid_control_pkg')
    activity_control_pkg_share   = FindPackageShare(package='activity_control_pkg').find('activity_control_pkg')
    laser_array_pkg_share        = FindPackageShare(package='laser_array_pkg').find('laser_array_pkg')

    enable_debug_topic_info = LaunchConfiguration("enable_debug_topic_info")

    enable_debug_topic_info_arg = DeclareLaunchArgument(
        "enable_debug_topic_info",
        default_value="true",
        description="Print topic pub/sub topology during bring-up for debugging.",
    )

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

    # 启动后 8 秒自动发 /route_choice=3（2Hz，10 次），触发 route_test 加载
    # RouteId=3 航线，同时让 uart_to_stm32 打开 /target_velocity 到 STM32
    # 的转发。重复发布用于规避跨设备启动时 DDS 发现延迟导致的一次性消息漏收。
    # demo_basic 至此无需任何手动终端命令即可完成起飞 → 对角巡航 → 降落。
    kick_route_choice = TimerAction(
        period=8.0,
        actions=[
            ExecuteProcess(
                cmd=[
                    'ros2', 'topic', 'pub',
                    '-r', '2',
                    '--times', '10',
                    '/route_choice',
                    'std_msgs/msg/UInt8', 'data: 3',
                ],
                output='screen',
            ),
        ],
    )

    # 调试：打印关键高度/目标话题的 pub-sub 拓扑，快速判断是否是链路断开或重映射问题
    debug_topic_info = TimerAction(
        period=9.0,
        actions=[
            ExecuteProcess(
                cmd=['ros2', 'topic', 'info', '/laser_array/ground_height', '-v'],
                output='screen',
            ),
            ExecuteProcess(
                cmd=['ros2', 'topic', 'info', '/height', '-v'],
                output='screen',
            ),
            ExecuteProcess(
                cmd=['ros2', 'topic', 'info', '/target_position', '-v'],
                output='screen',
            ),
        ],
        condition=IfCondition(enable_debug_topic_info),
    )

    return LaunchDescription([
        enable_debug_topic_info_arg,
        LogInfo(msg=["demo_basic: enable_debug_topic_info=", enable_debug_topic_info]),
        fly_carto_launch,
        uart_to_stm32_launch,
        laser_array_ground_launch,
        position_pid_controller_launch,
        route_test_launch,
        kick_route_choice,
        debug_topic_info,
    ])
