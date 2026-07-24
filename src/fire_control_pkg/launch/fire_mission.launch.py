from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue


def generate_launch_description():
    return LaunchDescription([
        # 两个开关做成 launch 参数，my_launch/fire_full.launch.py 直接透传，
        # 现场不用改代码重编就能在「只跑基本要求」和「带视觉发挥部分」之间切。
        DeclareLaunchArgument("enable_fire_task", default_value="false",
                              description="true=接视觉跑发挥部分(见红→抛包→上报)；false=只跑基本要求主链路"),
        DeclareLaunchArgument("auto_start", default_value="false",
                              description="true=不等消防车按键直接起飞（仅调试）。比赛时必须 false"),

        Node(
            package='fire_control_pkg',
            executable='fire_mission_node',
            name='fire_mission_node',
            output='screen',
            parameters=[{
                # 坐标系（与 PID / uart_to_stm32 一致）
                'map_frame': 'map',
                'base_frame': 'laser_link',
                'height_topic': '/height',

                # ── 场地 ↔ map 对齐 ──
                # map 原点 = Cartographer 上电位姿 = 起飞点。
                # 起飞点在场地系的坐标 = 左下黑色起降区的中心。
                # 该区按题目图 1 像素反算实测为 x 0~7 / y 0~7（**7×7dm，不是 11×7**，
                # 图上 11dm 那个标注量的是原点到红色停车区左沿），故中心 = (3.5, 3.5)。
                # ⚠️ 现场必须核对：把飞机摆好后跑一次 `ros2 run tf2_ros tf2_echo map laser_link`，
                #    确认 (0,0)，再确认机头朝场地 +x（即图 1 的向右）。斜摆就改 field_yaw_offset_deg。
                'home_field_x_dm': 3.5,
                'home_field_y_dm': 3.5,
                # 实测摆放：飞机 map +x 指向场地 +y（40dm 那条边的方向），
                # map +y 指向场地 −x。即场地 +x 在 map 系下的方位角 = −90°。
                # 验算：场地(1,0)→map(0,−1)；场地(0,1)→map(1,0)。
                # ⚠ 这个填错的后果是所有航点整体转 90°，飞机直接飞出场地。
                'field_yaw_offset_deg': -90.0,

                # ── 容差 ──
                'pos_tol_dm': 1.5,       # 15cm
                'yaw_tol_deg': 10.0,
                'height_tol_dm': 1.5,

                # ── 飞行高度（题目单位 dm）──
                'patrol_height_dm': 18.0,   # 基本要求(2)：18dm 左右
                'drop_height_dm': 10.0,     # 发挥(2)：降到 10dm 左右抛包
                'land_height_dm': 0.0,

                # ── 巡逻航线（覆盖宽 8dm → 40dm 分 5 条带）──
                # 蛇形：y=4 从 x=4→44，y=12 从 44→4，依此类推。
                # x 端点取 4/44 而非 2/46：覆盖足迹 8dm×8dm 以机身为中心，飞到 x=4 左沿
                # 正好压到 x=0、x=44 右沿压到 x=48，再往外飞足迹已出界，纯浪费。
                # 巡逻里程 = 5×40 + 4×8 = 232dm（原 252dm）。
                'strip_y_dm': [4.0, 12.0, 20.0, 28.0, 36.0],
                'patrol_x_min_dm': 4.0,
                'patrol_x_max_dm': 44.0,
                'patrol_yaw_deg': 0.0,      # 全程不转向，省时且避开 ±180 偏航跳变

                # ── 发挥部分 ──
                # 先只跑基本要求时保持 false：起飞→蛇形→返航→降落，不依赖视觉。
                # ⚠ 必须包 ParameterValue(value_type=bool)：substitution 默认当字符串传，
                #   节点侧 declare_parameter<bool> 会抛 InvalidParameterTypeException。
                'enable_fire_task': ParameterValue(
                    LaunchConfiguration("enable_fire_task"), value_type=bool),
                'fire_confirm_sec': 0.8,
                'fire_position_jitter_dm': 2.0,
                'hover_before_drop_sec': 3.0,   # 题目要求悬停 3s
                'drop_pulse_sec': 1.0,

                # ── 视觉伺服（闭环对中）──
                # 见了红先停在原地，靠 /fire/servo_error 一步步把火源修到画面中心，
                # 再降到 10dm 精对中一次。终点由"像素误差→0"决定，所以相机内参
                # (fx/fy/cam_offset) 只影响每步挪多远，不影响最终精度。
                'servo_deadband_ratio': 0.05,   # 归一化像素误差 <5% 算对好
                'servo_gain': 0.6,              # 每步走偏移量的 60%，防超调
                'servo_max_step_dm': 5.0,       # 单步位移上限，挡住误检把飞机拽飞
                'servo_timeout_sec': 12.0,      # 超时不再等对中，直接降高抛包（尽量拿分）
                'servo_data_timeout_sec': 1.0,  # 多久没收到误差算丢目标（保持悬停）
                # 火源上报的丢包冗余在 fire_link_pkg 的 report_repeat（默认 5），本包不重复发。

                'dist_sample_min_dm': 0.5,

                # 调试用：true 则不等消防车按键直接起飞。比赛时必须 false。
                'auto_start': ParameterValue(
                    LaunchConfiguration("auto_start"), value_type=bool),
            }],
        ),
    ])
