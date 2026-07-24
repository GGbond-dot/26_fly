# fire_control_pkg — 空地协同智能消防系统（2023 电赛 G 题）无人机侧

巡逻/灭火任务状态机。底层（飞控、Cartographer 定位、激光高度、串口）全部复用既有包，本包只做任务逻辑。

## 任务概要

- 巡防区 **48dm(X) × 40dm(Y)**，左下角为坐标原点，单位 dm。
  左下黑色 **7×7dm**（x 0~7 / y 0~7，中心 (3.5,3.5)）为无人机起降点。
  ⚠ 图上 `11dm` 那个标注量的是原点到**红色停车区**左沿，不是黑块宽度，别按 11×7 算中心。
  红色停车区实测 x 11~16 / y 0~5。
- 机腹激光笔垂直向下、**固定不可转动**，只用于指示巡逻航迹 → 起飞点亮、降落熄灭，无闪烁时序。
- 覆盖宽度 8dm，以缩短全覆盖时间为原则规划航线。
- 现场三个火花图案中**只有目标是红色**，另两个是其它颜色 → 视觉侧不做筛选，见红即目标。

## 状态机

```
WAIT_START → TAKEOFF → PATROL ──┬─(覆盖完成)──→ RETURN → LAND → DONE
                                │
                                └─(见红)→ APPROACH → DESCEND → HOVER(3s) → DROP → RESUME ─┘
```

`RESUME` 结束后回到被打断的巡逻航点 `patrol_resume_idx_` 继续覆盖，抛包只做一次
（`fire_handled_` 置位后不再响应检测）。

**火源坐标在进入 `APPROACH` 之前就已经发给消防车了**——`fire_confirmed_` 一置位
（`firePositionCallback` 里）立刻报一次，抛包完成后再报一次。这样抛包流程中间卡住
（比如 `DESCEND` 降不到 10dm）也不会拖累车端出发。车端 `state_ != IDLE` 会忽略第二次，
不会重复出发。

## 坐标系约定（重要）

航点、上报坐标一律用**题目场地系 dm**（左下原点），只在 `publishTarget()` / `getCurrentPose()`
两处与底层的 **map 系 cm** 互转（`fieldToMap` / `mapToField`）。这样航线可以直接照题目图 1 抄，
发给消防车的坐标也不用再换算。

map 原点 = Cartographer 上电位姿 = 起飞点，故转换只需知道**起飞点在场地系的坐标**
（`home_field_x_dm` / `home_field_y_dm`，默认取起降区中心 (5.5, 3.5)）。
正常摆放机头朝场地 +x，两系只差平移；斜摆则用 `field_yaw_offset_deg` 补一个旋转。

## 巡逻航线

5 条带，中心线 `y = 4/12/20/28/36 dm`，x 在 `[4, 44]` 之间蛇形往返，yaw 全程 0（不转向，
省时且避开 ±180 偏航跳变）。激光垂直向下、飞过即扫过，故每条带只需两个端点航点。

**端点为什么是 4/44 而不是 2/46**：覆盖足迹 8dm×8dm 以机身为中心，飞到 x=4 时足迹左沿正好
压到 x=0，飞到 x=44 右沿正好压到 x=48 —— 再往外飞足迹已经出界，是纯浪费。同理 y 的 4 和 36
也正好内缩 4dm，5 条带 ×8dm = 40dm 严丝合缝。**即"离边 4dm 以内的带状区域永远不用进"**。

巡逻里程 = 5×40 + 4×8 = **232dm**（原 252dm）。条带数为奇数，跑完停在右上 (44,36)，
返航腿约 52dm。竖着飞 6 条带里程同为 232dm、返航仅 40dm，但多一次转弯，暂不采用。
条带 y、x 起止全是参数，现场可调。

## 接口

| 方向 | Topic | 类型 | 说明 |
|------|-------|------|------|
| 发布 | `/target_position` | Float32MultiArray | `[x_cm, y_cm, z_cm, yaw_deg]`，**map 系** |
| 发布 | `/active_controller` | UInt8 | 2=位置控制器接管, 0=停 |
| 发布 | `/route_choice` | UInt8 | 1 → uart_to_stm32 开门转发 `/target_velocity` |
| 发布 | `/magnet/cmd` | Int32 | 1=机腹激光笔亮 / 2=灭。**走香橙派 40pin PB0 + WiringOP `gpio write`（低电平点亮），由 `magnet_control_pkg` 执行，不是飞控帧**。原先写的 `/electromagnet_control`(0x33) 是错的，盘点题已实测该路不稳并弃用 |
| 发布 | `/buzzer_led_control` | UInt8 | 1=机上报警 LED 亮（帧 0x22），发挥(1) 示警 |
| 发布 | `/drop_package` | UInt8 | 1=抛洒灭火包 / 0=复位。走飞控**舵机帧 0x11**（LEN=1）：uart 侧把 1→DATA `0x00`(舵机 1400 开)、0→DATA `0x01`(700 关，上电默认)。**开合时序由本包 `drop_pulse_sec_` 控制，不是固件** |
| 发布 | `/drone_pose` | Float32MultiArray | `[x_dm, y_dm]` **@1Hz** → 消防车实时显示（基本要求3） |
| 发布 | `/patrol_distance` | Float32MultiArray | `[dist_dm]` @1Hz，累计巡逻里程（基本要求4） |
| 发布 | `/fire_report` | Float32MultiArray | `[x_dm, y_dm]` 火源坐标 → 消防车（发挥2）。**报两次**：视觉确认时一次 + 抛包完成后一次。UDP 丢包冗余在 `fire_link_pkg` 侧做，本包每次只发一条 |
| 发布 | `/fire_status` | String | 状态/心跳，≥2Hz，断流即判飞机离线 |
| 发布 | `/mission_complete` | Empty | 收尾（帧 0x66） |
| 订阅 | `/fire_start` | Empty | 消防车按键启动（经 UDP 桥转成本地话题） |
| 订阅 | `/height` | Int16 | 离地高度 cm，uart_to_stm32 转发 STM32 上报 |
| 订阅 | `/fire/detected` | Bool | fire_vision_pkg：本帧看到红色火花图案 |
| 订阅 | `/fire/position_map` | Float32MultiArray | `[x_cm, y_cm]` **map 系**，反投影得到的火源地面坐标 |
| tf | `map → laser_link` | — | 当前水平位姿（与 PID/uart 一致） |

## 待办

- [ ] **场地标定**：`home_field_x_dm/y_dm` 现场核对（摆好飞机后 `tf2_echo map laser_link` 应为 0,0，
      且机头朝场地 +x）。
- [x] `fire_vision_pkg` 已实现（红色识别 + 反投影）。但颜色阈值/相机内参未标定，
      `enable_fire_task` 先保持 `false` 只跑基本要求，标完再 `enable_fire_task:=true`。
- [x] **机-车通信已接**：`fire_link_pkg` 把 `/drone_pose`、`/patrol_distance`、`/fire_report`
      桥到车端 UDP，`/fire_start`、`/car_status` 反向进来。机 192.168.10.197 ↔ 车 192.168.10.113。
      ⚠ 但四条通道一次真实收发都没跑过，先用 `scripts/fire_link_test.py` 对拉。
- [x] 抛包链路已定：飞控串口三（USART3, 921600 8N1, PB10/PB11）走 `AnoDTRaspRecvOneByte`
      协议，舵机 ID=0x11/LEN=1，DATA==0x01→700(关)、其他→1400(开)，上电默认 700。
      固件无需再加 0x13 帧，uart_to_stm32 直接发 0x11。
- [ ] 首次试飞：`enable_fire_task=False` + `auto_start=True`，验证 起飞→蛇形→返航→降落 主链路。

## 运行

整套（飞控底座 + 任务 + 机车链路 + 视觉），比赛/自启走这个：

```bash
ros2 launch my_launch fire_full.launch.py                    # 等消防车按键，只跑基本要求
ros2 launch my_launch fire_full.launch.py auto_start:=true   # 调试：不等按键直接起飞
ros2 launch my_launch fire_full.launch.py enable_fire_task:=true  # 标定完后带视觉跑发挥部分
```

只起本包（不飞控、只看状态机）：

```bash
ros2 launch fire_control_pkg fire_mission.launch.py
```
