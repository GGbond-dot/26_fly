from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    return LaunchDescription([
        Node(
            package="activity_control_pkg",
            executable="pillar_pickup_mission",
            name="pillar_pickup_mission",
            output="screen",
            parameters=[{
                "map_frame": "map",
                "laser_link_frame": "laser_link",

                # 到达容差
                "position_tolerance_cm": 9.0,
                "yaw_tolerance_deg": 5.0,
                "height_tolerance_cm": 12.0,

                # 航线数值
                # SCAN 阶段飞行高度：用于起飞和扫描终点，低空飞行便于柱子检测
                "flight_height_cm": 30.0,
                "land_height_cm": 20.0,
                "scan_end_x_cm": 250.0,
                "landing_x_cm": 250.0,
                "landing_y_cm": -250.0,
                "pillar_visit_height_cm": 150.0,
                "pillar_wait_timeout_sec": 3.0,

                # 视觉对准
                "visual_align1_timeout_sec": 4.0,
                "visual_align2_timeout_sec": 1.5,
                # 抓取下降中停二次对准超时(s)：对上或超时就继续往下，防卡死
                "descend_recenter_timeout_sec": 2.5,
                "visual_pixel_tol_px": 15,
                "visual_align_required_hits": 3,
                "visual_jump_px": 100,
                "visual_stale_sec": 0.5,

                # 摄像头→点阵激光 xy 偏置（cm，机体系=map系，yaw全程0）。仅用于“测高”那步：
                # 视觉把柱子对到相机正中后，机体按此偏置平移，让点阵正对柱心再测高。
                # 轴映射(建图/map↔图像)：map +x ↔ 画面正上方，map +y ↔ 画面正左方。
                # 标定：点阵激光装在“画面正上方”=map +x 侧 → offset = 相机−激光 = (-d, 0)。
                # 2026-05-22：点阵激光拉近到相机 d=2.5cm → (-2.5, 0)。柱顶宽容差大，符号方向最关键。
                # ⚠️ 测高已挪回第一趟(机械臂收起、点阵无遮挡)，此偏置只在第一趟 MEASURE_HEIGHT 用。
                "cam_offset_dx_cm": -2.5,
                "cam_offset_dy_cm": 0.0,

                # 摄像头→机械臂吸取点 xy 偏置（cm，机体系）。用于“下降取物/放置”：
                # 测完高后机体按此偏置平移，让机械臂吸取点正对铁片中心再下降。
                # 2026-05-22：机械臂伸直后吸取点与摄像头光轴共线 → 真值 0/0（非占位）。
                # 铁片对到画面中心即可竖直下降取物。若伸臂后有微小残差再在此填小量微调。
                "arm_offset_dx_cm": 0.0,
                "arm_offset_dy_cm": 0.0,

                # 高度采样（点阵反推柱高）。sample_pillar_drop_thresh_cm 复用为“突变”阈值。
                "sample_duration_sec": 2.5,
                "sample_min_pillar_frames": 8,
                "sample_pillar_drop_thresh_cm": 20.0,

                # 测柱高：窗口内累计所有“点阵打中柱顶”(drop≥sample_pillar_drop_thresh_cm)的帧，
                # 攒够 height_min_hit_frames 帧后【分簇】：排序按 height_cluster_gap_cm 间隔切簇，
                # 在帧数≥height_min_cluster_frames 的簇里取【中位数最高】的那簇=真柱顶（柱顶给最大drop，
                # 柱沿/半遮挡给偏小值会把中位数拉低——5-23 柱0 被测成 38 即此因）。超时仍没合格簇 → 柱高未知。
                # 第一趟(MEASURE_HEIGHT,机械臂收起点阵无遮挡)测好存起来；第二趟抓取前再现场重测一次，
                # 与第一趟差 >live_height_consistency_cm 则判现场不可信、回退第一趟值（见下）。
                "measure_height_timeout_sec": 5.0,
                "height_min_hit_frames": 10,
                "height_cluster_gap_cm": 8.0,
                "height_min_cluster_frames": 3,
                # 第二趟抓取前现场重测与第一趟柱高的最大可信差(cm)。超过=现场不可信→回退第一趟值。
                # 量级参考：好的情况两趟差 ~4cm；5-23 柱0 暴雷差到 48cm。设 15 给激光互差留余量又能拦暴雷。
                "live_height_consistency_cm": 15.0,

                # 铁片占比采样（判大小，决定叠放顺序）。第一趟 CENTER 期间累积
                # /circle_area_ratio，少于此帧数则判该柱为空柱（无铁片）。
                "plate_min_ratio_frames": 3,

                # 题目发挥要求："找到黑铁片→滞空3s+声光提示"。第一趟 CENTER 占比帧够
                # （= 确认有铁片）即开蜂鸣器/LED(帧0x22)，悬停此时长后关声光再飞下一柱。
                # 空柱（占比不够）不触发声光。
                "survey_signal_hold_sec": 3.0,

                # 下降 / 抓取 / 叠放（z 全是面阵目标读数；下降参照 R=ARM_GROUND_AREA_CM 是宏，改在 hpp）
                # 抓取目标 z = R + 柱高 − grab_press；放置 z = R + 空柱高 + 已叠层×plate_thickness + drop_gap − drop_press。
                # mid_clearance_cm 已废弃（抓取改为分段下降），保留仅避免参数报错。
                "mid_clearance_cm": 30.0,
                # 抓取分段下降：每段下降量 seg(cm) + 末段盲降下限 tail(cm)。
                # 总下降量 D=巡航150−(R+柱高−grab_press)；剩余≥seg+tail(=50)就降30并对准一次，
                # 否则末段直接盲降到位（末段≈[20,49]cm）。
                "descend_seg_len_cm": 30.0,
                # 抓取最小片（pickup_order 末位）时用更密的 20cm 分段；放置不加密，仍用 descend_seg_len_cm。
                "descend_seg_len_min_plate_cm": 20.0,
                "descend_min_tail_cm": 20.0,
                # 下降安全底线(cm)：抓取下降中面阵读数 < 此值（臂尖将到地面以下=物理不可能，多半柱高测错
                # 或飞机偏离柱子在往地面狂降）立即中止下降、爬回重测重试，不用人工拔电。
                # 取 R(臂触地面阵读数,宏 ARM_GROUND_AREA_CM=27) 再减 ~5cm 余量 = 22。短柱正常抓取面阵不会低于 R，不误触发。
                "descend_abort_area_cm": 22.0,
                # 抓取下压量(cm)：z=R+柱高-grab_press。grab_press 越大降得越低。
                # 5-23 分段下降版实飞：grab_press=0 时臂尖停在柱顶面上方约4cm够不到 → 设 4。
                # 还吸不牢够不到→继续回调正值；反而下降过头压坏→回调（最小到 0 或设负值）。
                "grab_press_cm": 4.0,
                # 放置余隙(cm)：z=R+空柱高+已叠+drop_gap-drop_press。drop_gap 仍为 0。
                "drop_gap_cm": 0.0,
                # 放置下压量(cm)：值越大放置末段降得越低，确保铁片贴到叠面后再松磁。
                "drop_press_cm": 2.0,
                # 抓取末段盲降额外 y 偏置(cm)：只用于抓取 DESCEND_FINAL，放置不加。
                "grab_final_dy_cm": -2.0,
                # 放置释放时序：先伸臂，等机械臂到位后松磁，再等铁片落稳后收臂。
                "arm_extend_sec": 1.2,
                "drop_settle_sec": 0.5,
                # 抓取到位后悬停等待机械臂动作完成；放置侧现在用 arm_extend_sec + drop_settle_sec。
                "hover_grab_sec": 1.0,
                # 每片铁片厚度（cm）：叠放时每多一层，放置落点抬高这么多
                "plate_thickness_cm": 1.0,

                # 最大铁片抓取本来很准，跳过抓取视觉微调；放置仍使用空柱视觉对齐。
                "skip_largest_grab_visual_align": True,

                # 空柱放置 anchor：视觉确认后记录当前实际位置，后续下降/叠放复用。
                # 首次允许最多约 24cm 修正（覆盖 32cm 大空柱半边长 16cm + 飞行误差），
                # 已有 anchor 后单次只允许小步更新，防止放完后视觉锁到铁片把目标带偏。
                "empty_pillar_side_cm": 32.0,
                "drop_visual_anchor_enable": True,
                "drop_anchor_max_correction_cm": 24.0,
                "drop_anchor_max_update_step_cm": 8.0,
                "drop_visual_circle_veto_sec": 0.6,

                # traverse_only_mode / measure_only_mode：任一为 true → 只跑第一趟
                # (含测高 + 读占比) 然后降落，不抓取。测高已挪回第一趟，两者现等效，
                # 都可用来单独验证“测高 + 占比排序”而不真抓。正式跑抓取叠放时两者都设 False。
                "traverse_only_mode": False,
                "measure_only_mode": False,   # 正式跑：第一趟测高+占比 → 第二趟抓取叠放

                # 抓取观察 + 重试：CLIMB_BACK 到位后悬停 N 秒看 /circle_area_ratio 是否仍非 NaN，
                # 仍能识别出铁片即视为抓取失败，最多重试 pickup_max_attempts 次
                "pickup_check_observe_sec": 2.0,
                # OBSERVE 期间连续 N 帧还能看到真黑圆盘，才判抓取失败；NaN/空柱不累计。
                "pickup_observe_plate_frames_required": 3,
                "pickup_max_attempts": 3,
            }],
        )
    ])
