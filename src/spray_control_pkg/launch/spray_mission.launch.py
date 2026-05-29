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
                # 坐标系（与 PID / uart_to_stm32 一致）
                'map_frame': 'map',
                'base_frame': 'laser_link',

                # 高度反馈 topic：来自 uart_to_stm32 转发的 STM32 高度上报
                'height_topic': '/height',

                # 飞行参数
                'flight_height_cm': 150.0,
                'land_height_cm':   0.0,
                'home_x_cm':        0.0,
                'home_y_cm':        0.0,
                'pos_tol_cm':       12.0,
                'yaw_tol_deg':      10.0,
                'height_tol_cm':    10.0,

                # 撒药颜色门控
                'spray_decision_timeout_sec':   1.5,
                'spray_data_stale_timeout_sec': 0.5,
                'spray_on_sec':                 0.3,
                'spray_off_sec':                0.3,

                # 发挥(2) 条码任务：发挥部分场地放杆塔时设 True
                'enable_barcode_task':      False,
                'pillar_left_offset_m':     0.8,
                'barcode_target_z_cm':      105.0,
                'barcode_wait_timeout_sec': 8.0,

                # 发挥(3) 圆周降落
                'enable_circle_landing':    False,
                'circle_radius_digit_div':  1.0,  # 1=个位（4位数→末位×10cm）；>=2=整个4位数（一般不可行）

                # TODO: 实测后填入起点 A(21) 与全覆盖绿色区块坐标
                # 'block_a_x_cm': 75.0,
                # 'block_a_y_cm': 350.0,
                # 'green_blocks': [id, x_cm, y_cm, ...]  # 蛇形顺序
            }],
        ),
    ])
