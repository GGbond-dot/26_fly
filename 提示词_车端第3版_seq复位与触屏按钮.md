# 给车端 AI 的任务提示词（第 3 版 · 上机实测前的最后一轮）

> ⚠️ **本版已作废（2026-07-22）**，不要再发。
> 队友的 commit `204ade1` 已经完成了 A（seq 复位）、B（触屏按钮）、E（终态冗余）、
> F（标定文档），C/D 也做了脚本部分。当前有效的是**第 4 版**：
> 《提示词_车端第4版_定位失败处理.md》。
> 本文件保留仅作记录。

> 用法：把下面 `====` 之间的全部内容复制给队友的 AI 编码助手。
> 它是自包含的（对方冷启动、没有本对话上下文）。
>
> 本版基于对 `klayddd-beep/diansai_H_car` commit `6fb0793` 的完整代码复核。
> 上一版（第 2 版 · 组网方案变更）要求的事项**已全部完成**，不用重做。

====

## 背景

你在维护 2023 电赛 G 题「空地协同智能消防系统」的**消防车侧** ROS 2 Humble 工作区
（仓库 `diansai_H_car`，板子是 Orange Pi 5 Max，Ubuntu 22.04 / aarch64，车上带触摸屏）。

系统由一架无人机和一台消防车组成。无人机全覆盖巡逻找火源、抛灭火包并把火源坐标报给车；
车收到后避开街区开过去、激光照射灭火、返航。两块板 `ROS_DOMAIN_ID` 不同、DDS 不互通，
**所有跨机通信走 UDP**，包格式定义在仓库根目录的《G题_机车通信接口约定.md》。

**无人机侧当前状态**：三个包（`fire_control_pkg` / `fire_link_pkg` / `fire_vision_pkg`）
已在开发板 `colcon build` 通过，固定 IP `192.168.10.197`，四条 UDP 通道均已实现。
两机即将做**第一次真机对拉**，本轮任务就是扫清对拉前的障碍。

**硬约束：UDP 包的字节格式和端口号一律不许改。** 两侧结构体做过逐字段偏移比对，
任何改动都要两边同步，成本很高。本轮**不涉及**任何包格式变更。

---

## 先知会一件事：无人机侧改了火源上报时机（**你们不用改代码**）

原来是「抛完灭火包后才发火源坐标」，现在改成**报两次**：

1. **视觉确认火源的那一刻**就报第一次 —— 不等抛包。因为抛包流程
   （接近→降高→悬停3s→抛）中任何一步卡住，坐标就永远发不出去，
   而你们收不到坐标 = 发挥 2/3 的 25 分全丢。况且车开过去本来就要时间，早报只会更快。
2. **抛完之后**再报一次同样的坐标，作为第一次全丢时的补救。

两次的坐标完全相同，但 **seq 不同**（各自连发 5 个包）。

所以你们会看到 `/fire_event` 被发布**两次**，间隔大约十几秒。这是正常的，不是 bug。

**为什么你们不用改代码**：`fire_mission_manager::on_fire()` 里已经有
`if (state_ != State::IDLE) { report(current_status_); return; }` 的保护，
第二次事件到达时车已经在路上（`DRIVE_FIRE`），会被直接忽略，不会重复出发或重新规划。
我已经核对过这段代码，**请不要动它** —— 它现在承担了幂等保护的职责。

唯一需要你确认的是：**仪表盘上的火源标记不会因为收到第二次事件而闪烁或跳动**。

---

## 任务 A 🔴 修 seq 去重逻辑：发送端一重启就永久丢包

**这是本轮唯一的 P0，不修的话你们所有重复测试的结果都是假的。**

### 问题

`src/follower_pkg/src/fire_event_bridge.cpp` 第 66 行：

```cpp
if (p.magic != kMagic || !std::isfinite(p.x_dm) || !std::isfinite(p.y_dm) ||
  (have_seq_ && static_cast<int16_t>(p.seq - last_seq_) <= 0)) {continue;}
```

`src/follower_pkg/src/fire_link_bridge.cpp` 第 196 行是同样的判据：

```cpp
if (have_telemetry_seq_ && static_cast<int16_t>(packet.seq - last_telemetry_seq_) <= 0) {
  continue;
}
```

无人机侧的 `report_seq_` / `telemetry_seq_` 都是节点成员变量，**每次节点重启从 0 重新开始**。
而火源坐标全场只上报一次，所以**每一轮飞行的火源包 seq 永远都是 0**。

于是：

| 轮次 | 车端状态 | 结果 |
|---|---|---|
| 第 1 轮 | `have_seq_ = false` | 接受，`last_seq_ = 0` ✅ |
| 第 2 轮（飞机重启，车端没重启） | `int16(0 - 0) = 0 ≤ 0` | **丢弃 → 车永远不出发** ❌ |

遥测同理，只是表现不同：飞机重启后 `telemetry_seq_` 归零，而车端 `last_telemetry_seq_`
还停在几百，于是要等飞机重新数回那个值（1Hz 就是几百秒）屏幕才会恢复刷新
——现场看到的就是「仪表盘永久停更」。

**这两条通道对应的分数**：火源上报 = 发挥 2/3 的 25 分；遥测 = 基本要求 3、4 的 16 分。
而且失败是**完全静默**的：车端日志一切正常，收到的包被 `continue` 无声吃掉，
现场根本无从排查。

### 要求你做的

在两处 seq 判据上并入「**流中断即复位**」：距离上一个**有效包**超过一个超时阈值时，
把 `have_seq_` / `have_telemetry_seq_` 置回 `false`，当作一条全新的流重新起判。

- 阈值做成参数，默认 **3.0 秒**（与仪表盘的 `link_timeout_s` 同一个量级）
- 用 `std::chrono::steady_clock`，不要用 ROS 时间
- 复位时打一条 INFO 日志，例如
  `telemetry stream restarted (gap 12.4s), resetting sequence filter`
  ——现场一眼能看出发生过什么
- 乱序去重的原有行为必须保留：连续流内 `int16` 差值 ≤ 0 仍然丢弃

> 为什么不让无人机侧改？因为无论机端 seq 从几开始，都可能小于车端记住的值
> （随机、时间戳低位都一样有问题），**唯一正确的修法就是接收端识别"这是一条新的流"**。

### 验收

```bash
# 终端 1：起车端
ros2 launch car_launch fire_mission.launch.py

# 终端 2：模拟飞机第 1 轮，发火源包 seq=0
python3 -c "import socket,struct;s=socket.socket(socket.AF_INET,socket.SOCK_DGRAM);s.sendto(struct.pack('<HHfff',0xFC11,0,30.0,25.0,0.0),('127.0.0.1',8889))"
# → /fire_event 收到

# 等 5 秒（超过阈值），模拟飞机重启后的第 2 轮，seq 同样是 0
python3 -c "import socket,struct;s=socket.socket(socket.AF_INET,socket.SOCK_DGRAM);s.sendto(struct.pack('<HHfff',0xFC11,0,30.0,25.0,0.0),('127.0.0.1',8889))"
# → 修好之后：/fire_event 必须再次收到，日志出现 stream restarted
# → 没修：第二次被静默丢弃
```

遥测同样测一遍：连发一串递增 seq，停 5 秒，再从 seq=0 开始发，仪表盘必须恢复刷新。
**另外补一条反向用例**：不停顿地连发 `seq = 5, 3, 4`，`3` 和 `4` 必须仍然被丢弃
（证明乱序去重没被改坏）。

---

## 任务 B 🟠 仪表盘那个"启动按钮"现在点不动

车上有触摸屏，比赛时不会插键盘鼠标。

`src/follower_pkg/scripts/fire_dashboard.py` 第 401~413 行在屏幕底部画了一个圆角矩形按钮，
写着「空格 / 回车　启动无人机」，**看起来完全就是个可点的按钮**。

但第 513~517 行的 `process_events()` 只处理键盘：

```python
key = cv2.waitKey(1) & 0xFF
if key == 27: return False
if key in (13, 32): self.start_publisher.publish(Empty())
```

**没有 `cv2.setMouseCallback`**，触屏点下去（X11 下就是一次 `EVENT_LBUTTONDOWN`）
程序完全收不到。也就是说比赛现场戳那个按钮不会有任何反应，仍然得插键盘。

### 要求你做的

1. 在 `draw_panel()` 里把按钮矩形存成成员，例如
   `self.start_button_rect = (x, button_top, x + inner_w, button_top + button_h)`
   （渲染尺寸会随窗口大小变，所以每帧都要更新，不能只算一次）
2. 建窗之后注册 `cv2.setMouseCallback(WINDOW_NAME, self.on_mouse)`
3. 回调里：`event == cv2.EVENT_LBUTTONDOWN` 且坐标落在 `start_button_rect` 内
   → `self.start_publisher.publish(Empty())`
4. 按钮文字改成「**点击启动无人机**」（保留键盘空格/回车作为备份，不要删）
5. **防连点**：触发后 2 秒内忽略后续点击/按键。触屏很容易连点，
   虽然 `fire_link_bridge::request_start()` 有 `pending_start_packets_ > 0` 的保护、
   飞机侧也只认第一个启动包，但操作员需要看到反馈
6. **点击反馈**：触发后把按钮变色并显示「已发送启动指令」约 2 秒。
   现在点下去屏幕毫无变化，操作员会反复戳

### 验收

在真实车载屏幕上（不是笔记本远程），用手指点按钮，`ros2 topic echo /drone_start`
必须收到一次 `Empty`，且 `fire_link_bridge` 日志出现 `start requested by ROS topic`。

> **注意**：`fullscreen=true` 的 OpenCV 窗能否收到触摸事件，前提是板子上有桌面会话、
> 且触屏被驱动映射成鼠标设备（绝大多数情况是的）。这一条请**在真机上验**，
> 不要只在开发机用鼠标点一下就算过。

---

## 任务 C 🟠 物理启动按键仍然要接

`fire_params.yaml` 里 `button_gpio_value_path` 还是空字符串。

题目原文是「在消防车上**按键**启动无人机」。触屏按钮算不算"按键"取决于评委理解，
大概率算，但不值得赌。代码侧已经完全就绪（sysfs 读电平 + 50ms 去抖 + `button_active_low`），
README 里连 Orange Pi 5 Max 的 40-pin 导出步骤都写好了，成本就是一个按钮加两根杜邦线。

### 要求你做的

按 README 已有的步骤实际接一个按钮、导出 sysfs、把路径填进 `button_gpio_value_path`，
然后验证：按一下 → `fire_link_bridge` 日志出现 `start requested by GPIO button`。

**两条路并存**（触屏 + 物理键），评委问起来指哪个都行。
sysfs 导出/方向/上拉在重启后会丢，记得做成 root 的 systemd oneshot，
并让 ROS 任务服务 `After=` 它。

---

## 任务 D 🟠 开机自启

你们 README 自己也列了「ROS 任务开机自动运行：未配置」。

比赛现场没时间敲命令，也不该假设有人能 SSH 上去。要求：

1. 装一个 systemd service（或桌面自启动项）拉起
   `ros2 launch car_launch fire_mission.launch.py`
2. 仪表盘要显示，所以**必须在有桌面会话之后启动**
   （`After=graphical.target` / 用户级 service + 正确的 `DISPLAY`、`XAUTHORITY`）
3. 如果第 C 项做了 GPIO 按键，本服务要 `After=` 那个 sysfs 导出的 oneshot
4. 日志落盘到固定位置，断电重启后还能翻出上一次跑的记录
5. README 里写清楚怎么**停掉**它（现场调试时要能一条命令停）

验收：拔电重启板子，不碰键盘鼠标，屏幕自己进到全屏仪表盘、链路状态显示「离线」
（因为飞机还没开），此时开飞机 → 3 秒内变「正常」并开始刷坐标。

---

## 任务 E 🟡 `done` / `failed:*` 终态连发 3 次

`fire_event_bridge::reply()` 现在是把 `/fire_mission_status` 原样单发一次。

中间态（`enroute` / `extinguishing` / `returning`）是周期性的，丢一两个无所谓。
但 `done` 和 `failed:<reason>` 是**一次性事件**，UDP 丢了机端就永远不知道车干完了。

要求：终态（`done` 和 `failed:` 前缀）连发 3 次，间隔几十 ms。中间态维持单发。

---

## 任务 F 🟡 场地标定参数：把"怎么标"写进 README

`fire_params.yaml` 里这几个现在全是默认值/估计值，**不标定的话收到火源坐标车也会开到错地方**：

- `arena_origin_map_x_m / arena_origin_map_y_m / arena_origin_map_yaw_deg` —— 全是 `0.0`，完全没标
- `obstacles_dm` 六个街区 —— 照赛题 PDF 图估的
- `home_x_dm = 13.5, home_y_dm = 2.5`（红色出发区）—— 需现场核对

不需要你现在填数值（要到场地量），但要求你在 README 里补一节**可照做的标定流程**，
明确写清楚这两件容易搞错的事：

1. **方向约定**：`arena_origin_map_*` 填的是「**场地原点 (0,0) 在车的 map 系里的坐标**」，
   而无人机侧填的是「**飞机起飞点在场地系里的坐标**」——两边方向是**反的**，别填反。
   请在 README 里用一个具体数字的例子说明（比如车停在场地 (13.5, 2.5) dm 处上电，
   那 `arena_origin_map_x_m` 应该填多少），并说明 `yaw_deg` 的正方向。
2. **互验方法**：两边各自标完后，把无人机搬到场地某个已知格点，
   看车端屏幕显示的坐标对不对。**对得上才算标完**，只标不验等于没标。

车端的 map 原点 = Cartographer 上电位姿 = 车的出发位置，所以标定前必须把车摆到
出发区的固定点、朝固定方向，这一点也要写进 README（现场重启车就得重新摆）。

---

## 交付要求

1. **每一项都要在 Orange Pi 5 Max 真机上验证**，不接受只做静态检查。
   任务 B 必须用**手指点车载触摸屏**验，不是在开发机上用鼠标点。
2. 报告里逐条说明改了什么、怎么验的、结果如何。**没验的就明说「未验证」**，不要含糊带过。
3. 不许改动任何 UDP 包的字节布局和端口号。
4. 不许重构 `fire_mission_manager` 的规划逻辑，本次不涉及。
5. 任务 A 是 P0，如果时间不够，**A 和 B 必须完成**，C~F 可以往后排。

## 优先级

```
A（seq 复位）  🔴 不修的话所有重复测试的结果都是假的，且失败完全静默
B（触屏按钮）  🟠 不修的话比赛时车上得插键盘
C（物理按键）  🟠 题目原文要求"按键"，成本极低，别赌评委的理解
D（开机自启）  🟠 现场没时间敲命令
E（终态冗余）  🟡 不影响拿分，但机端会不知道车干完了
F（标定文档）  🟡 数值要到场地才能填，但流程现在就该写清楚
```

====

---

## 无人机侧状态（不用给队友，我自己的备忘）

- 机端三包已 `colcon build` 通过；固定 IP `192.168.10.197`
- `my_launch/fire_full.launch.py` 已建（飞控底座 + 任务 + UDP 桥 + 视觉一把起）
- `autostart_fly.sh` / `stop_fly.sh` 已从 D 题盘点切到 G 题消防
- 机端 seq 不需要改：车端做流复位即可覆盖机端重启的所有情况
- 机端待办：主链路试飞（`auto_start:=true` + `enable_fire_task:=false`）、
  视觉三项标定（颜色阈值 / 相机内参 / 反投影方向）、`home_field_x_dm/y_dm` 现场核对
  （⚠️ `fire_control_pkg` 和 `fire_vision_pkg` 两处的 `home_field_*` 必须填一样的值）
