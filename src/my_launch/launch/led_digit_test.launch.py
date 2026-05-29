import os

from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    uart_to_stm32_share = FindPackageShare(package="uart_to_stm32").find("uart_to_stm32")

    uart_to_stm32_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(uart_to_stm32_share, "launch", "uart_to_stm32.launch.py")
        )
    )

    return LaunchDescription([
        uart_to_stm32_launch,
    ])
