import os

from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.substitutions import FindPackageShare


def _include(package_name, launch_file, launch_arguments=None):
    package_share = FindPackageShare(package=package_name).find(package_name)
    return IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(package_share, "launch", launch_file)
        ),
        launch_arguments=(launch_arguments or {}).items(),
    )


def generate_launch_description():
    """rotate_test_basic：旋转飞控测试的「最小飞控底座」。

    只拉飞控基础链路三样，给 inventory_control_pkg 的 rotate_test 提供它需要的
    tf(map->laser_link) / /height / 位置PID(/target_velocity)：
        1) fly_carto             cartographer，出 tf map->laser_link + /scan
        2) uart_to_stm32         串口桥，发 /height、把 /target_velocity(0x31) 转给飞控
        3) position_pid_controller  标准位置 PID（z 走 /height，**不用面阵激光**）

    ⚠ 本 launch 不含任务节点——航点由 rotate_test 单独起（第二个终端）：
        终端1：ros2 launch my_launch rotate_test_basic.launch.py
        终端2：ros2 launch inventory_control_pkg rotate_test.launch.py
               （更柔和：test_yaw_step_deg:=45）

    高度走普通激光/STM32 上报的 /height（标准 PID），刻意不引 laser_array_ground，
    也不用 position_pid_controller_ground（那是搬运抗高台的面阵版）。

    ⚠ 首飞重点：rotate_test 转到 180° 后会带偏航平移返航，正好验证刚补的
       0x31 transformVelocity（map->机体）旋转——小油门、人随时接管，盯
       「转到180后往home飞方向对不对」。
    """
    fly_carto_launch = _include(
        "my_carto_pkg",
        "fly_carto.launch.py",
        {"use_rviz": "false"},
    )

    uart_to_stm32_launch = _include(
        "uart_to_stm32",
        "uart_to_stm32.launch.py",
    )

    position_pid_controller_launch = _include(
        "pid_control_pkg",
        "position_pid_controller.launch.py",
    )

    return LaunchDescription([
        fly_carto_launch,
        uart_to_stm32_launch,
        position_pid_controller_launch,
    ])
