# fire_link_pkg — 无人机 ↔ 消防车 UDP 桥（2023 电赛 G 题）

两块开发板的 `ROS_DOMAIN_ID` 不同（**消防车 6 / 无人机 26**，2026-07-23 与队友核对；
旧文档写的"车 10"是错的），控制面用定长 UDP 包单播，与车端跟随链路的
`pose_sender` / `leader_pose_receiver` 同样思路。

> 视频流不走本包：它塞不进 32 字节定长包，改由 `fire_video_link_pkg` 的跨域 DDS 桥
> 把 `/camera/down/compressed` 搬到共享域 6。两条通道并存，本包接口一行不变。

## 为什么不复用跟随用的 `PosePacket`

跟随那条链路是**单向**的、只装 x/y/yaw，共 24 字节，而且已在跟随功能上双板实测通过。
本题需要**双向**（车→机的按键启动），且要装火源坐标 / 累计里程 / 任务阶段，
所以另起一个 32 字节包型，不动已验证的旧链路。

## 包格式

见 `include/fire_link_pkg/fire_link_packet.hpp`。

> ⚠️ 该头文件在**两个仓库各存一份**，必须逐字节一致。改一处必须同步改另一处——
> `static_assert` 只保证总长 32 字节，字段顺序错位它是发现不了的。

小端定长 32 字节：

```
[magic u16=0xF14E][type u8][phase u8][seq u16][reserved u16][stamp_ms u32]
[x_dm f32][y_dm f32][distance_dm f32][height_dm f32][reserved u32]
```

坐标一律为**题目场地系 dm**（巡防区左下角原点，右上角 (48,40)），
车端拿到即可直接显示 / 直接当目标点用,两边都不需要再做坐标换算。

| type | 方向 | 内容 |
|---|---|---|
| 1 `TELEMETRY` | 机→车 | 无人机位置 + 累计里程 + 高度 + 阶段。默认 1Hz（基本要求 3/4） |
| 3 `CAR_START` | 车→机 | 消防车上按键启动无人机（基本要求 2），连发多次 |

> `type=2` 曾用于火源上报，**已废弃**——火源改走车端早就实现好的 `fire_event_bridge`
> （16 字节 / magic `0xFC11` / 8889 口，见 `FireEventPacket`），车端一行不用改。
> 这个值不要复用，免得跟旧脚本撞上。

## 组网

**两机接同一台路由器**，同网段固定 IP（车板自建热点实测效果差，已弃用）：

- **车 = `192.168.10.113`（固定）**
- **无人机 = 固定 IP，待定**

车端仍保留三级地址解析（学发送方地址 → 静态参数 → 子网广播）作为安全网，
详见接口约定文档 §一。

> 后续计划改**蓝牙**。届时只需替换本节点的收发实现，
> `/fire_start`、`/car_status` 等话题接口不变，下游一行不用动——
> 这正是把传输层单独拆成桥节点的原因。

## 端口

| 端口 | bind 方 | 用途 |
|---|---|---|
| 8889 | 车 | 机→车 火源上报（16B `0xFC11`） |
| 8890 | **机** | 车→机 任务状态字符串（裸 ASCII） |
| 8892 | 车 | 机→车 遥测（32B `0xF14E`） |
| 8893 | **机** | 车→机 按键启动（32B `0xF14E`） |
| ~~8888~~ | — | 旧跟随链路占用，本题不用 |

## 接口

| 方向 | Topic | 说明 |
|------|-------|------|
| 订阅 | `/drone_pose` | `[x_dm, y_dm]`，来自 fire_control_pkg |
| 订阅 | `/patrol_distance` | `[dist_dm]` |
| 订阅 | `/height` | Int16 cm |
| 订阅 | `/fire_status` | String，解析出阶段名转成 1 字节枚举 |
| 订阅 | `/fire_report` | `[x_dm, y_dm]`，收到即用**同一个 seq** 连发 `report_repeat`(5) 次到车端 8889。丢包冗余只在这里做，`fire_control_pkg` 侧每次只发一条话题（它会报两次：确认火源时 + 抛包完成后，两次 seq 不同、坐标相同） |
| 发布 | `/fire_start` | Empty，收到车端启动包时发布，触发 fire_control_pkg 起飞 |
| 发布 | `/car_status` | String，车端任务状态（`enroute`/`extinguishing`/…） |

阶段是从 `/fire_status` 的**中文文本**里解析的（只取 `阶段=` 到第一个逗号之间那一段），
没有另开一个专供本节点的话题——避免两处状态定义漂移。
**代价**：改 `fire_control_pkg::buildStatusText()` 里的阶段中文名，必须同步改
`fire_link_node.cpp::parsePhase()` 的对照表。

## 待办

- [x] 机端 `colcon build` 已通过（2026-07-22）。车端仓库是否编译过待队友确认。
- [ ] 链路对拉：从未跑通过一次真实收发。
- [x] 无人机固定 IP = **192.168.10.197**，已写进接口约定文档和 `fire_link.launch.py` 注释。
      ⚠ 车端 `fire_params.yaml` 的 `drone_ip` 仍可能是占位的 `.171`，要队友确认改成 `.197`。
- [ ] 路由器别开 AP 隔离（client isolation），否则两机互相 ping 不通。**先 ping 再查代码**。
- [ ] 防火墙：两块板都别拦 UDP 8889/8890/8892/8893 入站。
- [ ] 联调时 `telemetry_rate_hz` 先调到 5 看链路稳不稳，正式测试再降回 1。
- [ ] 赛场 2.4G 极拥挤，丢包严重时优先动路由器信道（1/6/11 挑干净的），别急着改代码。

## 运行

```bash
ros2 launch fire_link_pkg fire_link.launch.py
```

联调自测（不用真飞机）：

```bash
# 机端假装有位姿 → 车端屏幕应每秒刷新坐标
ros2 topic pub -r 1 /drone_pose std_msgs/msg/Float32MultiArray "{data: [12.0, 20.0]}"
# 机端假装报火源 → 车端应出发
ros2 topic pub --once /fire_report std_msgs/msg/Float32MultiArray "{data: [30.0, 25.0]}"
# 看车端回传的状态
ros2 topic echo /car_status
```

站在车端位置收发（`scripts/fire_link_test.py`，默认端口已对齐）：

```bash
./fire_link_test.py --start-after 3       # 默认 --drone-ip 192.168.10.197（本机），自环测试填 127.0.0.1
```
