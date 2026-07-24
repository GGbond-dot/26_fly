from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    return LaunchDescription([
        Node(
            package='fire_link_pkg',
            executable='fire_link_node',
            name='fire_link_node',
            output='screen',
            parameters=[{
                # 组网（2026-07-23 改）：**无人机自己做热点**，消防车接进来。
                #   无人机 = 10.42.0.1（热点网关，本机；车端 drone_ip 必须填这个）
                #   消防车 = 10.42.0.163
                # 组网变更史：外接路由器静态IP → 车板热点(效果差,弃用) → 路由器共享 → 机端热点(当前)。
                # 机→车发往固定地址；本节点自己 bind 8890/8893 收，不需要知道自己的地址。
                # ⚠ sendto 发到错误地址**不报错**，IP 配错的现象是"按了键飞机毫无反应"而日志正常。
                'car_ip': '10.42.0.163',

                # 机 → 车
                'telemetry_port': 8892,     # 遥测，32B 0xF14E type=1
                'fire_report_port': 8889,   # 火源上报，16B 0xFC11，车端 fire_event_bridge 零改动

                # 车 → 机（本机 bind）
                'start_listen_port': 8893,   # 按键启动，32B 0xF14E type=3
                'status_listen_port': 8890,  # 车端任务状态，裸 ASCII 字符串

                # 题目要求"每秒 1 次"位置坐标，故默认 1Hz。
                # 车端显示器想更跟手可以调高，多发不违规。
                'telemetry_rate_hz': 1.0,

                # 火源坐标是一次性事件，UDP 丢了没有第二次机会 → 连发几次做冗余。
                # 车端按 seq 去重，重复包不会触发多次前往。
                'report_repeat': 5,
            }],
        ),
    ])
