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

                # 飞行参数（高度沿用朋友实测 130cm；激光点更准，已与队友确认接受基本要求(1)的风险）
                'flight_height_cm': 130.0,
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
                'enable_barcode_task':      True,
                # 柱子改为空中识别：起飞后开检测窗边飞边判，关窗即锁定（map 系 ROI，由 pillar_detector_tf 攒帧）
                'pillar_topic':             '/detected_pillars',  # tf 版输出（按票数降序）
                'pillar_detect_window_sec': 4.0,    # 检测窗时长上限，关窗锁定，不会“判太久”
                'pillar_observe_offset_m':  0.8,    # 离杆塔水平后撤距离（按相机焦距/视场标）
                'barcode_cam_axis':         '+x',   # 本机二维码相机装正前方→相机朝 +x；朋友装侧面用 '-y'
                'barcode_target_z_cm':      105.0,  # 柱子实测~130cm，先降到此高读条码再拉回巡航（降-读-升）
                'barcode_wait_timeout_sec': 8.0,

                # 发挥(3) 圆周降落
                'enable_circle_landing':    True,
                'circle_radius_digit_div':  1.0,  # 1=个位（4位数→末位×10cm）；>=2=整个4位数（一般不可行）

                # ── 全覆盖航点（沿用朋友实测坐标系：home十字=原点(0,0)，X=行方向指上、Y=列方向往右为负，50cm 栅格）──
                #   X: R1=250 R2=200 R3=150 R4=100 R5=50 R6=0     Y: C1=-50 C2=-100 … C7=-350
                #   顺序=朋友实测蛇形：左凸出区列向之字(21→28→27→20→19→26) → R1右半→R2←→R3→R4←→R5→R6←，终点 cell4
                #   两个固定白格(R4 的 C6/C7，id=0)与 3 个随机灰格全靠颜色门控自动跳过(仍作航点飞过，路径更平滑)。
                #   ⚠️ 仍建议场地先跑一次 home 记 tf 校验栅格是否对齐。
                'block_a_x_cm': 200.0,   # 起点 A=21 (R2,C1)
                'block_a_y_cm': -50.0,
                'green_blocks': [
                    # 左凸出区(C1~C3 × R1/R2) 列向之字
                    28.0, 250.0,  -50.0,  27.0, 250.0, -100.0,  20.0, 200.0, -100.0,
                    19.0, 200.0, -150.0,  26.0, 250.0, -150.0,
                    # R1 右半 →
                    25.0, 250.0, -200.0,  24.0, 250.0, -250.0,  23.0, 250.0, -300.0,
                    22.0, 250.0, -350.0,
                    # R2 ←
                    15.0, 200.0, -350.0,  16.0, 200.0, -300.0,  17.0, 200.0, -250.0,
                    18.0, 200.0, -200.0,
                    # R3 →
                    14.0, 150.0, -200.0,  13.0, 150.0, -250.0,  12.0, 150.0, -300.0,
                    11.0, 150.0, -350.0,
                    # R4 ← (C7/C6 为固定白格 id=0，飞过不撒)
                    0.0,  100.0, -350.0,   0.0,  100.0, -300.0,   9.0,  100.0, -250.0,
                    10.0, 100.0, -200.0,
                    # R5 →
                    8.0,  50.0,  -200.0,   7.0,  50.0,  -250.0,   6.0,  50.0,  -300.0,
                    5.0,  50.0,  -350.0,
                    # R6 ← (终点 cell 4)
                    1.0,  0.0,   -350.0,   2.0,  0.0,   -300.0,   3.0,  0.0,   -250.0,
                    4.0,  0.0,   -200.0,
                ],
            }],
        ),
    ])
