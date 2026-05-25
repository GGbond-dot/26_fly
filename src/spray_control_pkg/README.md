# spray_control_pkg — 植保飞行器（2021 电赛 G 题）

撒药任务状态机骨架。负责全覆盖飞行路径调度、激光笔“撒药”动作、精准返航降落。
底层（飞控、定位、激光高度、通信）复用既有包，本包只做任务逻辑。

## 任务概要（来自题目 PDF）

- 作业区 400cm(Y) × 500cm(X)，50×50cm 区块编号 1~28，绿色为播撒区。
- 起点区块 **A = 21**；起降点为“十”字标识。
- 激光笔垂直向下、可控开关；每格闪烁 **1~3 次**为正常撒药，>3 次算重复（扣分）。
- 基本要求：起飞→升至 150±10cm→从 A 全覆盖撒药→360s 内完成→精准降落（±10cm）。
- 发挥部分：①3~4 连续区块改灰重做 ②识别黑色杆塔环形条形码(Code128b 4位)用 LED 显示
  ③在以数字×10cm 为半径的圆周上降落 ④现场编程。

## 状态机

`TAKEOFF → GOTO_A → COVERAGE(逐格撒药) → RETURN → LAND → DONE`

## 接口（沿用 activity_control_pkg 约定）

| 方向 | Topic | 类型 | 说明 |
|------|-------|------|------|
| 发布 | `/target_position` | Float32MultiArray | `[x_cm, y_cm, z_cm, yaw_deg]` |
| 发布 | `/active_controller` | UInt8 | 2=位置控制器接管, 0=停 |
| 发布 | `/laser_control` | UInt8 | 1=激光笔开, 0=关（**新执行机构，需 STM32 固件支持**）|
| 发布 | `/mission_complete` | Empty | 收尾 |
| 订阅 | `/laser_array_ground_height` | Int16 | 离地高度(cm) |
| tf | `map → base_link` | — | 当前水平位姿 |

## 待办（骨架未实现，按需补全）

- [ ] **场地标定**：`buildCoverageWaypoints()` 里 `green_blocks` / `block_a` 为占位坐标，需按图 1 实测填写并排成蛇形顺序。
- [ ] **激光笔固件**：`uart_to_stm32` 增加 `/laser_control` → STM32 GPIO 的映射（参考既有 `/electromagnet_control`）。
- [ ] 发挥(2)：杆塔条形码识别（柱子定位可复用 `pillar_detector_pkg` 的 `/detected_pillars`）。
- [ ] 发挥(3)：圆周降落落点计算。

## 运行

```bash
ros2 launch spray_control_pkg spray_mission.launch.py
```
