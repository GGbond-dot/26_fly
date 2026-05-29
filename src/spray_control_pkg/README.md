# spray_control_pkg — 植保飞行器（2021 电赛 G 题）

撒药任务状态机。底层（飞控、定位、激光高度、通信）复用既有包，本包只做任务逻辑。

## 任务概要

- 作业区 400cm(Y) × 500cm(X)，50×50cm 区块编号 1~28，绿色为播撒区。
- 起点区块 **A = 21**；起降点为"十"字。
- 激光笔垂直向下、可控开关；每格闪烁 **1~3 次**为正常撒药，>3 次重复（扣分）。
- 基本：起飞→升至 150±10cm→从 A 全覆盖→360s 内完成→精准降落（±10cm）。
- 发挥：①3~4 连续区块改灰重做 ②识别黑色杆塔环形 Code128b 4 位条码 LED 显示 ③以数字×10cm 为半径圆周降落 ④现场编程。

## 状态机

`TAKEOFF → GOTO_A → COVERAGE → RETURN → LAND → DONE`

COVERAGE 内航点按 `SprayWaypoint::spray / wait_barcode` 标志走子状态：

- `spray=true`：颜色门控撒药（采 3 帧 `/spray_allowed` → 见绿才闪激光 2 次 → 全灰/超时跳过）
- `wait_barcode=true`：悬停等 `/barcode_text` → 解析为数字 → 发 `/led_digit` 显示

启用条码任务（`enable_barcode_task: True`）时，节点起飞**前**先等 `/detected_pillar`，
拿到杆塔坐标后自动在 GOTO_A 之后插入条码观察航点。

启用圆周降落（`enable_circle_landing: True`）时，进入 RETURN 阶段前
若 `barcode_number_ > 0` 会改写最后两个航点为圆周点（半径 = N×10cm，N 默认取个位）。

## 接口

| 方向 | Topic | 类型 | 说明 |
|------|-------|------|------|
| 发布 | `/target_position` | Float32MultiArray | `[x_cm, y_cm, z_cm, yaw_deg]` |
| 发布 | `/active_controller` | UInt8 | 2=位置控制器接管, 0=停 |
| 发布 | `/electromagnet_control` | UInt8 | 1=激光开, 0=关。**激光复用电磁铁 STM32 GPIO**（帧 0x33） |
| 发布 | `/led_digit` | UInt8 | 1~9，STM32 LED 闪烁该次数显示条码数字（帧 0x12，**uart_to_stm32 新增**） |
| 发布 | `/mission_complete` | Empty | 收尾 |
| 订阅 | `/height` | Int16 | 离地高度(cm)，由 uart_to_stm32 转发 STM32 上报 |
| 订阅 | `/spray_allowed` | Bool | 下视相机中心 ROI 见绿色（drone_camera_pkg） |
| 订阅 | `/barcode_text` | String | pyzbar 识到的 Code128（barcode_camera_pkg） |
| 订阅 | `/detected_pillar` | Float32MultiArray | `[x_m, y_m]` 单杆塔坐标（pillar_detector_single_node） |
| tf | `map → laser_link` | — | 当前水平位姿（与 PID/uart 一致） |

## 待办

- [ ] **场地标定**：`buildCoverageWaypoints()` 里 `green_blocks` / `block_a` 为占位坐标，需按图 1 实测填写并排成蛇形顺序。
- [ ] STM32 固件加 0x12 帧 → LED 闪烁数字逻辑（uart_to_stm32 侧的 ROS topic 已加）。
- [ ] 现场标定 HSV 阈值（drone_camera_pkg）：避免灰格被误判为绿色。
- [ ] 发挥(3) 圆周降落时，确认 N 是取条码数字的"末位"还是"整数"（题目歧义，需现场确认）。

## 运行

```bash
ros2 launch spray_control_pkg spray_mission.launch.py
```
