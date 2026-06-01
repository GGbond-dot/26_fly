# G 题植保飞行器 — 开发笔记

> 2021 全国大学生电子设计竞赛 **G 题 植保飞行器** 落到本仓库的 `spray-task` 分支上的开发记录。
> 题目 PDF：`2021植保飞行器（G题）.pdf`。
> 朋友的参考实现：`github_temp/26sunmmer_test2/`（已在我以前的搬运代码上做了 G 题适配）。

---

## 1. 题目要点（任务驱动表）

作业区 400cm(Y) × 500cm(X)，50×50cm 区块编号 1~28，绿色为播撒区，灰色为非播撒区，起降点为黑底白"十"字（位于左下角附近，距离左 75cm/距离下 75cm），起点区块 A = **21**。

### 1.1 基本要求（共 50 分）

| 编号 | 内容 | 关键约束 | 分值 |
|---|---|---|---|
| (1) | "十"字垂直起飞 → 巡航高度 150±10cm | 高度闭环 | 5 |
| (2) | 寻找起点 A、开始撒药 | 视觉/坐标定位 A | 5 |
| (3) | 360s 内全覆盖播撒所有绿色区块 | **不得漏撒、重复撒**；每格闪 1~3 次正常，>3 次算重复 | 30 |
| (4) | 稳定准确降落在起降点 | 几何中心偏差 ≤ ±10cm | 10 |

### 1.2 发挥部分（共 50 分）

| 编号 | 内容 | 关键约束 | 分值 |
|---|---|---|---|
| (1) | 3~4 个连续绿色区块改灰，重做 (1)~(3) | **不能撒灰格**（需视觉判定逐格放行） | 20 |
| (2) | 识别 **Code128b 4 位数字** 圆环条形码（贴在 150cm 高黑色杆塔的 120~140cm 段），用 LED 闪烁次数显示 + 间隔重复 | 杆塔在非播撒区且距边缘 ≥100cm；杆塔径 3.5±0.5cm | 2 |
| (3) | 以"十"字中心为圆心，**N×10cm** 为半径（N=条形码数字）的圆周上稳定降落 | 几何中心点到圆周最近距偏差 ≤ ±10cm | 8 |
| (4) | 现场编程 30 分钟 | — | 15 |
| (5) | 其他 | — | 5 |

### 1.3 撒药判定细则（来自 PDF 三-3-(2)）

- 激光点在区块虚线格中闪烁 1~3 次 → 正常播撒。
- 同一格往复 / 闪烁 > 3 次 → 重复播撒（**扣分**）。
- 激光点未在虚线格内闪烁 → 漏撒（**扣分**）。
- 飞行经过但激光笔未开启 → **不计为播撒**。
- 激光笔光点闪烁周期 1~2s。

→ 单格策略：到点悬停后，开激光 0.3~0.5s × 1 次 = 总耗时约 0.3~0.5s/格，**闪 1 次**（2026-05-31 由 2 次改为 1 次，单次落在 1~3 次合法区间内，省悬停时间）。

---

## 2. 硬件 & 接口现状

- 飞控/底盘沿用搬运飞行器（同机），STM32 串口桥 `uart_to_stm32`。
- **激光笔**：物理上接在原 carry 项目的**电磁铁**位置（同一 STM32 GPIO）。因此 ROS 侧**沿用 `/electromagnet_control` (UInt8) 链路**控制激光开关，STM32 固件**不用改**。
  - `/electromagnet_control` UInt8：1 = 激光开（原"通电吸合"），0 = 激光关（原"断电释放"）
  - STM32 帧 ID `0x33`，1 字节 payload。
- **下视相机**（颜色识别绿/灰）：`/dev/video2`，新加 `drone_camera_pkg`（来自朋友）。
- **前视/侧视相机**（条形码 Code128b）：`/dev/video0`，新加 `barcode_camera_pkg`（来自朋友）。
- **LED 数字显示**：通过 `uart_to_stm32` 新增 `/led_digit` UInt8 帧（ID `0x12`）→ STM32 控板载 LED 闪烁 N 次。
- **高度源**：当前用 `/height`（由 `uart_to_stm32` 从 STM32 上报，Int16 cm），**面阵激光 4x4 暂停**（参见 memory：面阵后续可恢复）。
- **2D 激光**：`/scan` → `pillar_detector_pkg` 的 **tf 版 `pillar_detector_tf`**（柱子改空中识别后改用此版，2026-05-29 晚）：每帧用 TF `map→laser_link` 把点变到 map 系、按固定 bbox 过滤，由 `/pillar_detect_enable` 开/关检测窗，输出 `/detected_pillars`。单杆版 `pillar_detector_single_node` 和多杆版保留备份不用。
- **TF**：`map → laser_link`（PID/uart 都按 `laser_link` 配的；本机 spray 骨架原来写的 `base_link` 需改成 `laser_link`）。

---

## 3. 朋友代码评估与融合策略

朋友的代码（`github_temp/26sunmmer_test2/src/`）整体已跑通基本要求 + 发挥(1)(2) 大部分。
**核心思想**：把视觉/激光/解码做成独立小节点，主任务节点串航点表 + 撒药门控 + 杆塔/条码插入。

### 3.1 朋友各包用途速查

| 朋友包 | 类型 | 输入 | 输出 | 是否搬入本机 |
|---|---|---|---|---|
| `activity_control_pkg/route_target_publisher` | C++ 主任务节点（隐式状态机） | `/detected_pillar`, `/spray_allowed`, `/barcode_text`, `/height`, tf | `/target_position`, `/active_controller`, `/laser/cmd` | ❌ **不搬主节点**（用我的 SprayMissionNode），但把其内部 `handleSprayTarget` 和 `handleBarcodeTarget` 的算法精华移植进来 |
| `drone_camera_pkg/drone_camera_node` | C++ | `/dev/video2` 视频 | `/spray_allowed` Bool | ✅ 直接搬，原样用 |
| `barcode_camera_pkg/barcode_camera_node` | Python | `/dev/video0` 视频 | `/barcode_text` String | ✅ 直接搬，原样用 |
| `laser_control_pkg/laser_control_node` | Python（OPi GPIO 直驱） | `/laser/cmd` Int32 | GPIO | ❌ **不搬**。本机激光走电磁铁 STM32 链路 |
| `pillar_detector_pkg`（单杆版） | C++ | `/scan` | `/detected_pillar` Float32MultiArray [x,y] | ⚠️ 选择性搬。本机已有多杆版（输出 `/detected_pillars`），可保留双版本共存，G 题用单杆版 |
| `uart_to_stm32`（朋友删搬运订阅 + 加 `/led_digit` + 校验和） | C++ | UART | UART | ⚠️ 选择性合并：**保留**搬运的 servo/electromagnet/buzzer/laser_ground_height（同分支也可能要兼容），**只增量加** `/led_digit` 0x12 帧 + 校验和（可选） |

### 3.2 与本机骨架的差异点（要改的地方）

| 项 | 本机当前 `spray_mission_node` | 改成 | 原因 |
|---|---|---|---|
| 激光 topic | `/laser_control` UInt8 | **`/electromagnet_control` UInt8** | 复用电磁铁链路，STM32 不动 |
| 高度 topic | `/laser_array_ground_height` | **`/height`** | 面阵暂停，用 STM32 上报 |
| TF base frame | `base_link` | **`laser_link`** | 和 PID、uart 一致 |
| 撒药逻辑 | 到点就闪 N 次（盲撒） | **到点等 3 帧 `/spray_allowed` → 见绿才闪 2 次 → 不见绿/超时则跳过不撒** | 应对发挥(1) 灰格混入，避免重复/误撒扣分 |
| 起飞触发 | 无（一上电就走流程） | **照常起飞**；起飞后开柱子检测窗边飞边判（2026-05-29 晚定，柱子变细变高） | 柱子细(3cm)需在空中靠高柱身被激光打到；map 系 ROI 不受飞行影响 |
| 状态机阶段 | TAKEOFF → GOTO_A → COVERAGE → RETURN → LAND → DONE | **加 BARCODE_OBSERVE**（在 COVERAGE 之间或之后），最终阶段加 **CIRCLE_LAND**（发挥(3)） | 题目要求 |

---

## 4. 目标系统架构（融合后）

```
                ┌───────────────────────────────────┐
                │  spray_control_pkg                 │
                │  SprayMissionNode (C++ 状态机)     │
                │  TAKEOFF → GOTO_A → COVERAGE       │
                │     ↘ BARCODE_OBSERVE              │
                │  → RETURN → LAND / CIRCLE_LAND     │
                └────────┬──────────────────────────┘
                         │ pub
       ┌─────────────────┼──────────────┬──────────────────┐
       ▼                 ▼              ▼                  ▼
 /target_position  /active_controller /electromagnet_control /led_digit
 Float32MA[4]         UInt8            UInt8 激光开关       UInt8 数字
       │                 │              │                  │
       ▼                 ▼              ▼                  ▼
   pid_control_pkg   pid_control_pkg   uart_to_stm32  uart_to_stm32
       │                                     │                  │
       └→ /target_velocity ─────────────────→│  STM32 ←─────────┘
                                             │
                                             ├→ /height (Int16) ─┐
                                             │                   │
       ┌─── 订阅 ─────────────────────────────┘                  │
       │                                                         │
   SprayMissionNode  ◀────────────────────────────────────────────┘
       ▲
       ├──── /spray_allowed (Bool) ◀── drone_camera_pkg (video2 + HSV)
       ├──── /barcode_text  (String) ◀ barcode_camera_pkg (video0 + pyzbar)
       ├──── /detected_pillars (F32MA[x,y,...]) ◀ pillar_detector_tf (/scan, map系ROI, 票数降序)
       └───→ /pillar_detect_enable (Bool) ──→ pillar_detector_tf (起飞后开/关检测窗)
```

---

## 5. 接口契约（融合后定型）

### 5.1 SprayMissionNode 发布

| Topic | 类型 | 含义 |
|---|---|---|
| `/target_position` | Float32MultiArray | `[x_cm, y_cm, z_cm, yaw_deg]`，给位置 PID |
| `/active_controller` | UInt8 | 2 = 位置控制器接管；0 = 停 |
| `/electromagnet_control` | UInt8 | **1 = 激光开**，0 = 激光关（复用电磁铁帧 0x33） |
| `/led_digit` | UInt8 | 1~9，STM32 LED 闪烁该次数显示条形码数字（帧 0x12，**uart_to_stm32 新增**） |
| `/pillar_detect_enable` | Bool | 起飞后开/关柱子检测窗（true 攒帧 / false 聚类发布），驱动 `pillar_detector_tf` |
| `/mission_complete` | Empty | 任务完成信号 |

### 5.2 SprayMissionNode 订阅

| Topic | 类型 | 来源 | 含义 |
|---|---|---|---|
| `/height` | Int16 | uart_to_stm32 | 离地高度 cm |
| `/spray_allowed` | Bool | drone_camera_pkg | 当前下视中心 ROI 见绿色 → true |
| `/barcode_text` | String | barcode_camera_pkg | Code128 识别结果（如 "1234"） |
| `/detected_pillars` | Float32MultiArray | pillar_detector_tf | `[x_m, y_m, ...]` 多杆按票数降序，取首对=最佳；空=窗内没锁到 → 跳过条码 |
| tf `map → laser_link` | — | my_carto_pkg | 当前水平位姿 |

### 5.3 撒药子状态机（每个 spray 航点）

参考朋友 `handleSprayTarget`，本机实现版本：

```
进入 spray 航点（到位判定通过）
   ↓
spray_active = true
spray_frame_count = 0；spray_seen_green = false
   ↓
等下一帧 /spray_allowed（不同于本航点开始前的旧帧）
   ↓
采样 3 帧 → 任一为 true → spray_seen_green = true
   ↓
   ├── seen_green：发 /electromagnet_control=1，开 0.3s → 关 → advance
   │   （共闪 1 次，满足 1~3 次合法区间；2026-05-31 由 2 次改为 1 次）
   ├── 3 帧都 false：跳过本格 → advance（应对发挥(1) 灰格）
   └── 1.5s 超时仍未采够 3 帧：判 skip，告警 → advance
```

### 5.4 条码任务（BARCODE_OBSERVE 阶段）

参考朋友 `pillarCallback` + `handleBarcodeTarget`：

1. **柱子改为空中识别（2026-05-29 晚改，柱子变细 3cm + 变高）**：不再起飞前死等。节点正常起飞，起飞后立即发 `/pillar_detect_enable=true` 开一个 `pillar_detect_window_sec`（默认 4s）的检测窗 → `pillar_detector_tf` 窗内把 `/scan` 变换到 **map 系**、按固定 ROI 持续攒帧 → 关窗发 `false` → 检测器聚类后把结果发到 `/detected_pillars`（按票数降序，取首个=最佳），节点锁定缓存。
   - **为什么能在空中识别**：柱子虽细（3cm，单帧激光点很少），但**变高了**——无人机飞行姿态倾斜时 2D 激光平面跟着倾，矮柱会整段扫空，高柱总能在柱身某段被打到，所以空中可识别。
   - **为什么用 map 系 ROI**：ROI 框在 map 系柱子真实位置上，无人机边飞边转都不影响指向（旧 single 版 ROI 绑死 laser 系，只在停 home 时有效，一起飞就失效）。
   - **为什么设时间窗上限**：满足"别判太久"。窗内攒帧足够即锁定；窗内没攒够票（`/detected_pillars` 为空）则跳过条码任务，正常返航，不卡流程。
   - 检测器参数针对细杆放宽：`min_pts_per_group` 4→2、`group_dist_m`→0.18，靠 map 系窄 ROI + 多帧投票压误检。
2. **观察点 = 杆塔外侧后撤 `pillar_observe_offset_m`（默认 0.8m）**，方向 + yaw 由 `barcode_cam_axis` 决定（2026-05-29 改）：
   - `'+x'`（**本机**：相机装机体正前方）→ 停杆塔 −X 侧 `(px−d, py)`、yaw=0。**与撒药全程 yaw=0 一致，进出条码航点不转向，过渡最顺。**
   - `'-y'`（**朋友**：相机装侧面）→ 停杆塔 +Y 侧 `(px, py+d)`、yaw=−90。另支持 `'-x'`/`'+y'`，见 `insertBarcodeObserveWaypoint()`。
   - 旧参数 `pillar_left_offset_m` 已重命名为 `pillar_observe_offset_m`（朝向不再写死在 Y 轴）。
   - **底部相机↔激光偏置**：实测仅 ~1cm，忽略不补偿。
3. **柱子实测 ~130cm（= 巡航高），条码段在其上偏下** → 观察点展开成**三段航点**（`insertBarcodeObserveWaypoint` 一次插 3 个），**插在 `return` 航点之前**（不再放航线开头）：
   - `barcode_approach`：巡航高 130cm 水平飞到观察点上空（先到位，避免低空横移撞柱）。
   - `barcode_observe`：原地下降到 `barcode_target_z_cm`（默认 **105cm**，相机光轴对准条码段）悬停读 `/barcode_text`。
   - `barcode_climb`：原地拉回巡航高 130cm，再返航。
   - **放返航前的理由**：柱子在左下空白区，恰在"覆盖终点 cell 4 → 回 home"的顺路上；且条码数字只有圆周降落才用，越晚读越省飞行，几乎零额外航程。
4. 识别到 4 位数字后：发布 `/led_digit = N`（默认取个位，仅 2 分，简化 STM32 LED 逻辑），并把整数缓存到 `barcode_number_` 供 CIRCLE_LAND 用。超时（`barcode_wait_timeout_sec`）则跳过继续返航。

### 5.5 圆周降落（CIRCLE_LAND 阶段，发挥(3)）

仅当 `barcode_number_ > 0` 时进入；否则降级到 LAND。

```cpp
// 返航到起降点上空后调用
void enterCircleLanding(int barcode_number) {
  const double r_cm = (barcode_number % 10) * 10.0;       // 如 1234 → 4×10 = 40cm
  // 用 4 位数中的个位作半径乘数；或按需用其他位。题目"识别的数字×10cm" 的"数字"
  // 理解上有歧义，4 位数全用作 N 时半径 = 12340cm 显然不合理，
  // 比赛常规做法：取识别数的某一位（个位/或题目附加说明）。落地前需场地确认。

  // 选个最靠近 home_xy_ 的圆周点（防止飞远）
  const double yaw_rad = std::atan2(current_y - home_y, current_x - home_x);
  const double tx = home_x_cm + r_cm * std::cos(yaw_rad);
  const double ty = home_y_cm + r_cm * std::sin(yaw_rad);

  waypoints_.push_back({tx, ty, flight_height_cm, 0.0, false, 0, "circle_pre"});
  waypoints_.push_back({tx, ty, land_height_cm,   0.0, false, 0, "circle_land"});
}
```

> ⚠️ **题目歧义提示**：原文"以上述（2）中识别的数字 × 10cm 为半径" 中的"数字"既可能指 4 位数中的某一位，也可能指整个 4 位数。比赛前需要在测试时与裁判确认或两套方案备好；4 位数全用大概率不现实（最大 4 米以外了，超出场地）。建议默认取**末位**（个位）当 N。

---

## 6. 全覆盖航点规划

### 6.1 坐标系约定（沿用朋友实测 carto frame，2026-05-29 定版）

以 **home"十"字中心为原点 (0,0)**，**高度 130cm**（朋友实测值，全程含巡航/撒药；激光点更准，已接受基本要求(1) 150±10 的扣分风险）。

- **X = "行"方向，指向图上方**：R1=250, R2=200, R3=150, R4=100, R5=50, R6=0（每往下一行 −50）。
- **Y = "列"方向，往图右走为负**：C1=−50, C2=−100, C3=−150, C4=−200, C5=−250, C6=−300, C7=−350。
- 故 **A=21 (R2,C1) = (200, −50)**。

> 这套坐标直接取自朋友 `buildPlantProtectionRoute()`（`route_target_publisher.cpp:24`），已逐点反推验证与现场地图完全吻合。**首次场地测试仍先跑一次 home 记 tf 校验栅格对齐。**

### 6.2 区块布局与蛇形顺序（2026-05-29 定版，= 朋友实测路线）

**L 形**：上两行(R1/R2)满 7 列（C1~C7），下四行(R3~R6)只有右 4 列（C4~C7）。区域共 **30 个格** = 28 编号绿格 + **2 个固定白格**（R4 的 C6/C7，无编号）。现场另有 **3 个随机灰格**。白格/灰格**仍作航点飞过**，靠颜色门控 `/spray_allowed` 自动不撒（比绕过更平滑，也天然兼容随机灰格）。

布局（行 R1~R6 自上而下，列 C1~C7 自左而右）：

```
R1: 28 27 26 | 25 24 23 22      (C1..C7)
R2: 21 20 19 | 18 17 16 15      (C1..C7)   ← A=21 在 C1 = (200,-50)
R3:          | 14 13 12 11      (C4..C7)
R4:          | 10  9 ▓▓ ▓▓      (C4,C5 绿; C6,C7 固定白格)
R5:          |  8  7  6  5      (C4..C7)
R6:          |  4  3  2  1      (C4..C7)
```

**关键洞察：不需要绕障。** 柱子 ~130cm = 巡航高（在非绿格区，覆盖路径不经过它）；白格/灰格直穿不撒即可。

**定版蛇形顺序**（朋友实测；左凸出区用"列向之字"开头解决，无长转移；终点 cell 4 靠近柱子/home 顺路读码）：

```
左凸出区(C1~C3) 列向之字 : 21 → 28 → 27 → 20 → 19 → 26
R1 右半 → : 25 24 23 22
R2 ←      : 15 16 17 18
R3 →      : 14 13 12 11
R4 ←      : ▓(C7) ▓(C6) 9 10          (两白格飞过不撒)
R5 →      : 8 7 6 5
R6 ←      : 1 2 3 4                    (终点 cell 4)
```

已按此填入 `spray_mission.launch.py` 的 `green_blocks`（block_a=21→(200,−50)）。

> ⚠️ 坐标取自朋友实测，**顺序与坐标已定版**；现场只需跑 home 记 tf 确认栅格对齐，一般不用改数值。

### 6.3 360s 预算

- 28 格 × 单格悬停 ~0.5s（闪 1 次）≈ 14s 撒药耗时（原闪 2 次时为 42s）
- 起飞 + GOTO_A + 区块间过渡 + RETURN + LAND ≈ 80s（按 30cm/s 速度估）
- BARCODE_OBSERVE + 识别等待 ≈ 30s（含飞行 + 等条码）
- 总计约 152s，**留 200s 余量**给 PID 抖动 / 撒药决策超时 / 重试。
- 若超时风险高，单格撒药耗时压到 1.0s（开 0.3 / 关 0.2 / 开 0.3 / 关 0.2）。

---

## 7. 融合落地清单（按优先级排列）

### P0 — 编译能跑通的最小改动 ✅ **2026-05-29 完成**

- [x] **拷朋友的 `drone_camera_pkg` 整包到 `src/`**（C++，输出 `/spray_allowed`）。
- [x] **拷朋友的 `barcode_camera_pkg` 整包到 `src/`**（Python，输出 `/barcode_text`）。
- [x] **拷朋友的 `pillar_detector_node.cpp` 单杆版**改名为 `pillar_detector_single_node.cpp`（class 改名 `PillarDetectorSingleNode`，hpp/main/launch 一并加上），CMakeLists 增量新可执行 `pillar_detector_single_node`；**老的多杆版和 tf 版完全不动**。输出 topic `/detected_pillar`（单杆）。
- [x] **改 `spray_mission_node`**（完全重写 hpp/cpp）：
  - `/laser_control` → `/electromagnet_control`（沿用 0x33 帧）
  - `/laser_array_ground_height` → `/height`
  - `base_frame` 默认值 `base_link` → `laser_link`
  - 删除盲撒，实现 5.3 节的颜色门控撒药 `runSprayWithColorGate()`
  - 新增 `SprayWaypoint::wait_barcode` 字段；`runBarcodeObserve()` 子状态
  - 订阅 `/spray_allowed` Bool、`/barcode_text` String、`/detected_pillar` Float32MultiArray
  - 发布 `/led_digit` UInt8
  - `pillarCallback` 收到杆塔坐标时自动 `insertBarcodeObserveWaypoint()` 在 GOTO_A 之后插入观察航点
  - 新增 `prepareFinalLanding()`：若识到条码数字且 `enable_circle_landing=True`，把最后两个航点（return/land）改写为圆周点（默认 N=末位）
  - 状态机 enum 保持 5 个不变（TAKEOFF/GOTO_A/COVERAGE/RETURN/LAND/DONE），条码与圆周降落通过航点标志实现
- [x] **改 `uart_to_stm32`**（**纯增量**，旧订阅一个不删）：
  - hpp 加 `LED_DIGIT_FRAME_ID = 0x12`、`ledDigitCallback`、`sendLedDigitToSerial`、`led_digit_sub_`
  - cpp 加订阅 `/led_digit` 创建 + 两个新函数实现，模板对照 `sendElectromagnetToSerial`
- [ ] **STM32 固件加 0x12 帧处理**（见 §13）——这步要烧到飞控板，本仓库无源码。
- [ ] 编译机 `colcon build --packages-select drone_camera_pkg barcode_camera_pkg pillar_detector_pkg uart_to_stm32 spray_control_pkg` 走一遍。

### P1 — 基本要求(1)~(4) 跑通

- [x] 写 `my_launch/spray_basic.launch.py`：包含
  - `my_carto_pkg/fly_carto.launch.py`（不开 RViz）
  - `uart_to_stm32`
  - `pid_control_pkg/position_pid_controller.launch.py`
  - `drone_camera_pkg`（参数：video2、HSV 阈值同朋友的 demo1）
  - `spray_control_pkg/spray_mission.launch.py`
- [ ] 场地实测 home (x, y)、起点 A、28 格中心坐标，填进 `spray_mission.launch.py` 的 `green_blocks` 参数。
- [ ] 实测调试：撒药决策超时、PID 容差、单格悬停时间。

LED 单项联调可先运行 `ros2 launch my_launch led_digit_test.launch.py`，只启动 `uart_to_stm32`，再手动发布 `/led_digit` 检查 STM32 端 0x12 帧闪烁逻辑。

### P2 — 发挥(1) 灰格混入

- [ ] 不用额外改代码（颜色门控天然支持）。只在场地实测时，灰格上方 `/spray_allowed` 应自动给 false → 跳过。
- [ ] 校准 HSV 阈值，避免灰色在某些光照下误判为"绿"。
- [ ] 测试灰格在不同位置（行首/行中/行尾）的跳过表现。

### P3 — 发挥(2) 条码 + LED

- [ ] 在 launch 里加入 `pillar_detector_single_node` 和 `barcode_camera_pkg`。
- [ ] 实现 BARCODE_OBSERVE 阶段（见 5.4）。
- [ ] 现场用 http://barcode.cnaidc.com/html/BCGcode128b.php 生成 4 位条码贴在杆塔上做测试。
- [ ] STM32 固件确认能处理 0x12 帧、用 LED 闪烁 N 次 + 间隔 2s 重复。

### P4 — 发挥(3) 圆周降落

- [ ] 实现 CIRCLE_LAND 阶段（见 5.5）。
- [ ] 在 RETURN → LAND 之间分支：有 `barcode_number_` → CIRCLE_LAND，否则 LAND。
- [ ] 现场和裁判确认 "N=末位"还是"N=4位整数"。

### P5 — 发挥(4) 现场编程 & (5) 其他

- [ ] 准备 `spray_control_pkg/launch/onsite_template.launch.py` 模板（可调参数预留）。
- [ ] 现场题目下来后参考本笔记快速改 `buildCoverageWaypoints()`。

---

## 8. 关键代码片段（从朋友处移植后的本机风格）

### 8.1 `SprayMissionNode` 撒药子状态机片段

```cpp
// spray_mission_node.cpp
bool SprayMissionNode::runSpraySequence()
{
  const auto now = this->now();
  if (!spray_active_) {
    spray_active_ = true;
    spray_frame_count_ = 0;
    spray_seen_green_ = false;
    spray_start_time_ = now;
    last_sampled_allowed_time_ = rclcpp::Time(0, 0, get_clock()->get_clock_type());
    return false;  // 等下一周期采样
  }

  // 已经在闪：闪 1 次（开 on_sec → 关 → 完成）
  if (blink_step_ >= 0) {
    const double dt = (now - blink_edge_time_).seconds();
    if (blink_step_ == 0 && dt >= spray_on_sec_) {
      publishLaser(false);
      spray_active_ = false; blink_step_ = -1;
      return true;  // 撒药完成
    }
    return false;
  }

  // 颜色判定：等 3 帧不同于 spray_start_time_ 的新帧
  const bool fresh = has_spray_allowed_ &&
                     (last_spray_allowed_stamp_ - spray_start_time_).seconds() >= 0.0 &&
                     (last_spray_allowed_stamp_ - last_sampled_allowed_time_).nanoseconds() > 0;
  const double elapsed = (now - spray_start_time_).seconds();

  if (!fresh) {
    if (elapsed > spray_decision_timeout_sec_) {
      RCLCPP_WARN(get_logger(), "spray 决策超时（航点 %zu），跳过", current_idx_);
      spray_active_ = false; return true;  // 视作完成，advance
    }
    return false;
  }

  ++spray_frame_count_;
  spray_seen_green_ = spray_seen_green_ || latest_spray_allowed_;
  last_sampled_allowed_time_ = last_spray_allowed_stamp_;

  if (spray_frame_count_ < 3) return false;

  if (spray_seen_green_) {
    publishLaser(true);
    blink_step_ = 0;
    blink_edge_time_ = now;
    return false;  // 进入闪烁子状态
  } else {
    RCLCPP_INFO(get_logger(), "航点 %zu 视野无绿色（灰格），不撒药跳过", current_idx_);
    spray_active_ = false;
    return true;
  }
}

void SprayMissionNode::publishLaser(bool on) {
  // 注意：本机激光走电磁铁链路
  std_msgs::msg::UInt8 m;
  m.data = on ? 1 : 0;
  electromagnet_pub_->publish(m);  // 不再是 laser_pub_
}
```

### 8.2 `uart_to_stm32` 增量（LED 数字帧）

```cpp
// uart_to_stm32.hpp 新增
static constexpr uint8_t LED_DIGIT_FRAME_ID = 0x12;
void ledDigitCallback(const std_msgs::msg::UInt8::SharedPtr msg);
void sendLedDigitToSerial(uint8_t digit);
rclcpp::Subscription<std_msgs::msg::UInt8>::SharedPtr led_digit_sub_;

// uart_to_stm32.cpp configure() 里
led_digit_sub_ = node_->create_subscription<std_msgs::msg::UInt8>(
  "/led_digit", rclcpp::QoS(10),
  std::bind(&UartToStm32::ledDigitCallback, this, std::placeholders::_1));

void UartToStm32::ledDigitCallback(const std_msgs::msg::UInt8::SharedPtr msg) {
  sendLedDigitToSerial(msg->data);
}
void UartToStm32::sendLedDigitToSerial(uint8_t digit) {
  std::vector<uint8_t> data = { digit };
  if (!serial_comm_->send_protocol_data(LED_DIGIT_FRAME_ID, 1, data)) {
    RCLCPP_ERROR(node_->get_logger(), "发送 LED 数字帧失败 digit=%u", digit);
  }
}
```

STM32 固件侧需新增 0x12 帧处理：收到后清空 LED → 闪 N 次（每次亮 0.3s 灭 0.3s）→ 停 2s → 重复。

---

## 9. 风险点 & 已知坑

1. **颜色门控阈值飘移**：场地光照差异（顶部照明 + 窗外光）会让 HSV 绿色阈值不稳。
   - 应对：现场用 `ros2 topic echo /spray_allowed` 飞前一次扫场地，盯着真值/假值切换调阈值；HSV 的 V 通道下限设宽一点（30~40）。

2. **条形码识别距离**：朋友默认杆塔左侧 80cm + 高度 105cm；杆塔条码段是 120~140cm。
   - 风险：相机视场 + 焦距是否能看清 1.0~1.3m 距离上的 Code128。
   - 应对：用 pyzbar 实测 1m 距离下 4 位 Code128 是否能稳定解码（朋友 `barcode_target_z_cm=105` 是经验值，按需调到 110~125cm）。

3. **电磁铁帧复用作激光的副作用**：上电时 uart_to_stm32 会发一次 `0x33 0x00` 关电磁铁 → 等于关激光，符合预期。但如果 STM32 那边对电磁铁帧有"延迟去抖"等逻辑，激光快速闪烁可能会被吃掉。
   - 应对：闪烁 on/off 间隔不要太短（≥ 0.3s），并在 STM32 端 review 一下 0x33 处理路径。

4. **`/height` 单位约定**：uart_to_stm32 上报的 `/height` 是 cm 还是 mm 要核对。本机 spray_mission 状态机按 cm 用。
   - 应对：`ros2 topic echo /height` 实测确认数量级。

5. **检测窗内锁不到柱子**：细杆 3cm 空中点少，或 bbox 框歪、carto 漂移。
   - 应对：已不再起飞前死等（不会卡死起飞）。关窗后 `/detected_pillars` 为空则自动跳过条码任务、正常返航。
   - 调参：回放 `/pillar_debug_points` 看窗内落进 bbox 的点够不够 → 放宽 `min_pts_per_group`/加长 `pillar_detect_window_sec`/校准 bbox。基本要求阶段直接 `enable_barcode_task=False` 关掉。

6. **360s 总时限**：撒药决策每格最长 1.5s 超时 + 闪烁 0.3s（闪 1 次）= 1.8s/格，28 格 = 50s，加航点过渡和起降，余量较前宽松（原闪 2 次时 67s）。
   - 应对：把 `spray_decision_timeout_sec` 调到 1.0s；过渡航点之间用更快的速度（PID `max_linear_velocity` 提到 50cm/s）。

7. **`active_controller` 收尾值**：朋友收尾用 3（保持停），本机骨架用 0（停）。统一**用 0**，与搬运代码一致。

---

## 10. 与搬运项目（carry-task 分支）的关系

- `spray-task` 分支与 `carry-task`/`main` **并存**，不互相覆盖。
- 共用包：`pid_control_pkg`、`my_carto_pkg`、`bluesea2`、`serial_comm`、`uart_to_stm32`、`pillar_detector_pkg`（多杆版保留）。
- spray-task 独有新增：`spray_control_pkg`、`drone_camera_pkg`、`barcode_camera_pkg`、`pillar_detector_single_node`（在同一包里加新可执行）。
- spray-task 独有改动：`uart_to_stm32` 增加 `/led_digit` 帧（**不删搬运订阅**，纯增量）。
- 切回 carry-task 时新加的 spray 节点不被引用，不会干扰。

---

## 11. 调试日志习惯（沿用搬运笔记的传统）

- 每次实测后追加到 §12 `开发日志`，记录：
  - 日期 / 试飞次数
  - 关键参数当时值（PID、HSV、容差、超时）
  - 现象（哪一段过、哪一段挂）
  - 结论与下一步改动

## 12. 开发日志

（场地实测后开始填）

- 2026-05-29 上午：开发文档与融合方案落定。
- 2026-05-29 下午：**P0 全部代码改完**。本机未编译（按双设备开发约定，编译在另一台）。
  - 新增包：`drone_camera_pkg`、`barcode_camera_pkg`（直接拷自 `github_temp/26sunmmer_test2/`）。
  - `pillar_detector_pkg` 增量加单杆版可执行 `pillar_detector_single_node`，老多杆版不动。
  - `spray_control_pkg/spray_mission_node` 完全重写，接口与子状态机按 §5 落实。
  - `uart_to_stm32` 增量加 `/led_digit` 0x12 帧。
  - 下一步：编译机 colcon build 验证；STM32 固件加 0x12 帧（见 §13）；进 P1 场地实测。
- 2026-05-29 晚：**条码相机偏置 + 柱子降读 + 全覆盖路径定版**（与队友核对硬件 + 现场地图后）。
  - **二维码相机装机体正前方**（队友是侧装）→ 加参数 `barcode_cam_axis`（本机 `'+x'`），`pillar_left_offset_m` 改名 `pillar_observe_offset_m`，观察点方向/yaw 按相机朝向算（见 §5.4）。
  - **底部相机↔激光偏置实测 ~1cm，忽略不补偿**（队友代码本就没补，确认无需改）。
  - **柱子实测 ~130cm**（低于巡航 150）→ `insertBarcodeObserveWaypoint` 改成"接近/观察/拉高"三段、观察高保持 105cm、**插入位置从航线开头挪到 `return` 之前**（柱子在回 home 顺路上，数字只圆周降落用）。柱子坐标用起飞前锁定帧（最准）。
  - **现场地图确认是 L 形**（R1/R2 满 7 列，R3~R6 仅右 4 列；区域 30 格=28 编号绿格+2 固定白格）。白格/灰格作航点飞过、颜色门控不撒。
  - **坐标系定版：直接沿用朋友实测 carto frame**——home=原点(0,0)，**X=行方向指上(R1=250…R6=0)**，**Y=列方向往右为负(C1=−50…C7=−350)**，**全程高度 130cm**。A=21=(200,−50)。逐点反推验证朋友 `buildPlantProtectionRoute()` 与现场地图完全吻合 → 我之前自推的 Y 正/原点在角/150 高度那版**全错**，已废弃。
  - **蛇形顺序 = 朋友实测路线**（左凸出区列向之字开头、无长转移，终点 cell4），坐标+顺序填入 `green_blocks`。高度经队友确认用 130（接受基本要求(1)扣分风险）。
  - 下一步：同上，编译机 build + 场地跑 home 记 tf 校验栅格对齐。
- 2026-05-29 实测飞行 #1（`fly_log_20260529_181708`）：**起飞失败**。
  - 根因：`uart_to_stm32` 有速度转发门控，需先收到合法 `/route_choice`(=1/2/3) 才转发 `/target_velocity` 到飞控；`spray_mission_node` 只发了 `/active_controller`，从未发 `/route_choice`，门始终关闭，15 次 `Dropping /target_velocity` 被丢弃。
  - 修复：`spray_mission_node` 在 `publishTarget()` 中同时发布 `/route_choice=1`（用 `transient_local` QoS 防止 uart 节点重连漏接），`uart_to_stm32` 不动（避免影响搬运项目）。
- 2026-05-29 实测飞行 #2（`fly_log_20260529_191208`）：**首次全程完成**，30 个绿格全打，2 处异常：
  - **误打**：R4C7（`block_id=0`，固定白格）被颜色门控误判为绿（3帧全绿）→ 打了激光。根因：HSV 阈值过宽（H:25–100，S≥20，V≥40，阈值仅 4%=100/2500px）。
  - **漏打/正常**：R4C6（`block_id=0`，固定白格）3帧全灰 → 正确跳过。
  - **修复1**：`buildCoverageWaypoints()` 中 `wp.spray = (wp.block_id != 0)`，`id=0` 固定白格硬编码跳过，不再走颜色门控，从根源断掉固定白格误打。
  - **修复2**：收紧 `spray_basic.launch.py` HSV 参数（针对随机灰格仍需颜色门控的情形）：
    - `green_h_min` 25→35，`green_h_max` 100→85（去掉黄色和青蓝）
    - `green_s_min` 20→60（要求真实饱和度，排除反光/灰色）
    - `green_v_min` 40→60（排除暗杂色）
    - `green_pixel_threshold` 100→250（4%→10%，防偶发像素触发）
  - 下一步：编译机 build `spray_control_pkg`（launch 文件无需编译），再跑一次验证固定白格不再误打，随机灰格颜色门控仍有效。
- 2026-05-29 晚 #2：**柱子改空中识别 + 完整流程开关 + 录包瘦身**（柱子变细 3cm、变高，要求边飞边判、别判太久）。
  - **柱子识别从"起飞前锁定帧"改为"起飞后开检测窗边飞边判"**：换用 `pillar_detector_tf`（map 系 ROI，无人机飞/转都不影响指向）。`spray_mission_node` 新增发布 `/pillar_detect_enable`、订阅改 `/detected_pillars`（取票数最高对）、删起飞前死等、加 `managePillarDetectWindow()` + 参数 `pillar_detect_window_sec`(默认4s)/`pillar_topic`。窗内没锁到 → 自动跳过条码、正常返航。见 §5.4。
  - **bbox 按实测柱子 ~map(100,-100)cm 框 ±50cm**（`spray_basic.launch.py` 的 `pillar_detector_tf` 节点）；细杆调参 `min_pts_per_group` 4→2、`group_dist_m`→0.18。
  - **完整流程开关全开**：`spray_mission.launch.py` `enable_barcode_task`/`enable_circle_landing` → True（测基本+发挥(2)(3)全链路）。
  - **新增条码相机单测 launch**：`my_launch/barcode_test.launch.py`（只起 barcode 节点，连续解码+预览，调 1m 解码距离用）。
  - **录包瘦身**：不录整段 `/scan`（太大）。`pillar_detector_tf` 加旁路发布 `/pillar_debug_points`（窗内落进 bbox 的 map 系原始点，体积极小，可离线重调聚类参数，**不影响检测**）；`autostart_fly.sh` 录该 topic 取代 `/scan`，并开 `--compression-mode file --compression-format zstd`。
  - 下一步：编译机 `colcon build --packages-select pillar_detector_pkg spray_control_pkg my_launch`，场地跑完整流程，回放 `/pillar_debug_points` 验证空中识别能锁到柱子。
- 2026-05-29 实测飞行 #3（`fly_log_20260529_215504`）：**条码识别成功（'0002'，number=2），圆周降落执行，但两处问题**：
  - **大量漏打**：10 个绿格（block 6/9/10/12/13/14/17/18/23/24）被颜色门控判为灰→跳过。根因：上次 HSV 收紧过猛（S/V 均 60，threshold=250），绿地毯被误判灰。
    - 修复：`spray_basic.launch.py` `green_s_min` 60→45，`green_v_min` 60→45，`green_pixel_threshold` 250→180（约 7%，仍能挡固定白格）。
  - **条码 observe 阶段超时 8s**：条码在 `barcode_approach` 飞行途中已识别，但进入 `barcode_observe` 时 `runBarcodeObserve()` 无条件执行 `barcode_detected_ = false` 清空了结果，重新等待等不到超时。
    - 修复：`spray_mission_node.cpp` `runBarcodeObserve()` 开头，若 `barcode_number_ > 0` 不清空 `barcode_detected_`，直接在下一帧判断成功退出。
  - **条码观察不再下降**：取消"接近→降105cm→拉回"三段，改为"接近→同巡航高(130cm)悬停读码"两段，`barcode_observe` 高度改用 `flight_height_cm_`，删 `barcode_climb` 航点。
  - **LED 发布时机改到降落后**：原在 `runBarcodeObserve()` 成功时发 `/led_digit`（裁判看不到）。改为在 `heightCallback` 中监测：下降过程中高度 < 10cm（且曾高于 50cm，排除起飞前地面状态）时发，加 `led_digit_sent_` + `has_been_airborne_` 防重复/误触。
  - 下一步：编译机 `colcon build --packages-select spray_control_pkg`，再跑验证漏打修复 + LED 时机。
- 2026-05-31：**单格撒药由闪 2 次改为闪 1 次**。
  - 改 `spray_mission_node.cpp` `runSprayWithColorGate()` 闪烁状态机：原 3 步（开→关→开→关）压成 1 步（开 `spray_on_sec_` → 关 → 完成），删 step1/step2 分支。
  - 单格悬停撒药耗时从 ~0.9s 降到 ~0.3s，单次激光仍落在题目「1~3 次」合法区间；360s 预算更宽松。
  - `spray_off_sec_` 参数保留声明但撒药路径不再用（便于以后恢复闪 2 次）。
  - 同步更新本笔记 §1.3 / §5.3 / §6.3 / §8.1 / §9-6。下一步编译机 `colcon build --packages-select spray_control_pkg` 验证。

---

## 13. STM32 固件协议（OPi → STM32 全量帧表）

### 13.1 物理层

| 项 | 值 |
|---|---|
| 接口 | UART |
| 波特率 | 与现有搬运项目一致（具体值见 `uart_to_stm32.launch.py` 的 `serial_port_name` / 串口配置；常用 115200/921600，按既有固件） |
| 数据位 / 停止位 / 校验 | 8 / 1 / None |

### 13.2 帧结构

```
偏移  长度  字段          说明
 0     1   HEADER=0xAA   固定帧头
 1     1   ADDRESS=0xFF  固定地址
 2     1   ID            帧类型 ID（见下表）
 3     1   LEN           数据段长度（0~255）
 4    LEN  DATA[LEN]     数据段
 4+L   1   SUM_CHECK     和校验
 5+L   1   ADD_CHECK     附加校验
```

**最小帧长 = 6 字节**（LEN=0 时）。

### 13.3 校验算法

对 `[HEADER, ADDRESS, ID, LEN, DATA...]` 共 `4+LEN` 字节逐字节累加：

```c
void calculate_checksum(const uint8_t *frame, uint16_t len,
                        uint8_t *sum_check, uint8_t *add_check)
{
    uint8_t s = 0, a = 0;
    for (uint16_t i = 0; i < len; ++i) {
        s = (uint8_t)(s + frame[i]);
        a = (uint8_t)(a + s);
    }
    *sum_check = s;
    *add_check = a;
}
```

> 校验范围**不包含**末尾的 SUM/ADD 自己；累加是无符号 8bit 自然溢出。

### 13.4 ID 总表（OPi → STM32）

| ID | 长度 | 名称 | 数据含义 | 备注 |
|---|---|---|---|---|
| 0x11 | 1B | SERVO_FRAME | 0x00=收起机械臂 / 0x01=放下 | 搬运用，G 题不用 |
| 0x12 | 1B | **LED_DIGIT_FRAME**（**G 题新增**） | digit (0~9) | LED 闪烁此次数显示条码数字 |
| 0x22 | 1B | BUZZER_LED_FRAME | 0x00=关 / 0x01=蜂鸣器+LED 开 | 搬运用，G 题不用 |
| 0x31 | 16B | TARGET_VELOCITY_FRAME | 4×float32 LE：`vx_cm/s`, `vy_cm/s`, `vz_cm/s`, `vyaw_deg/s` | 位置 PID 给飞控的速度指令 |
| 0x32 | 12B | VELOCITY_FRAME | 3×float32 LE（map→机体变换后的线速度） | 旧 carry 链路 |
| 0x33 | 1B | ELECTROMAGNET_FRAME | 0x00=关 / 0x01=开 | **G 题：物理上接的是激光笔**，含义改成"激光关/开"，固件端**无需改**——只把这一路 GPIO 对应到激光笔即可 |
| 0x66 | 1B | MISSION_COMPLETE_FRAME | 固定 0x06 | 任务结束信号 |
| 0xF1 | 0B | ST_READY_QUERY | — | OPi 询问 STM32 是否就绪（OPi→ST） |
| 0x07 | 2B | LASER_GROUND_HEIGHT_FRAME | int16 LE，单位 cm | 搬运面阵激光地面高度转发，G 题暂停（spray_mission 用 `/height` 反向回链） |

### 13.5 ID 总表（STM32 → OPi）

| ID | 长度 | 名称 | 数据含义 | 备注 |
|---|---|---|---|---|
| `/height` 上报 | 2B | — | int16 LE，cm | uart_to_stm32 解析后发到 `/height`，spray_mission 直接订阅 |
| `is_st_ready` | 1B | — | 0x01=就绪 | 飞控就绪信号 |
| `mission_step` | 1B | — | 任务推进步骤反馈 | （按既有约定） |

### 13.6 G 题新增帧 0x12（LED 数字显示）详解

**目的**：发挥(2) "用 LED 闪烁次数显示条形码所表征的数字，间隔数秒后再次闪烁显示"。

**帧字节序列**（LEN=1）：

```
AA FF 12 01 [digit] [sum_check] [add_check]
```

**示例：digit = 4**

校验逐字节累加：

| 字节 | 值 | sum | add |
|---|---|---|---|
| HEADER | 0xAA | 0xAA | 0xAA |
| ADDRESS | 0xFF | 0xA9 | 0x53 |
| ID | 0x12 | 0xBB | 0x0E |
| LEN | 0x01 | 0xBC | 0xCA |
| DATA[0] | 0x04 | 0xC0 | 0x8A |

→ 完整帧：`AA FF 12 01 04 C0 8A`

**STM32 端处理建议**（伪代码）：

```c
// 收到 0x12 帧时调用
void on_led_digit_frame(uint8_t digit) {
    if (digit == 0) {
        led_off();
        return;
    }
    // 启动一个状态机，避免阻塞主循环
    g_led_task.digit = digit;
    g_led_task.round = 0;          // 已重复轮数
    g_led_task.max_rounds = 2;     // 题目"间隔数秒后再次闪烁" → 至少 2 轮
    g_led_task.phase = LED_ON;
    g_led_task.blinks_left = digit;
    g_led_task.next_edge_ms = HAL_GetTick() + 0;  // 立刻开始
    g_led_task.active = true;
    led_on();
}

// 主循环或 1kHz 定时器调用
void led_task_tick(void) {
    if (!g_led_task.active) return;
    uint32_t now = HAL_GetTick();
    if ((int32_t)(now - g_led_task.next_edge_ms) < 0) return;

    switch (g_led_task.phase) {
      case LED_ON:
        led_off();
        g_led_task.phase = LED_OFF;
        g_led_task.next_edge_ms = now + 400;   // 灭 400ms
        break;
      case LED_OFF:
        if (--g_led_task.blinks_left > 0) {
          led_on();
          g_led_task.phase = LED_ON;
          g_led_task.next_edge_ms = now + 400;  // 亮 400ms
        } else {
          // 本轮结束
          if (++g_led_task.round < g_led_task.max_rounds) {
            g_led_task.blinks_left = g_led_task.digit;
            g_led_task.phase = LED_PAUSE;
            g_led_task.next_edge_ms = now + 2000;  // 间隔 2s
          } else {
            g_led_task.active = false;            // 任务结束
          }
        }
        break;
      case LED_PAUSE:
        led_on();
        g_led_task.phase = LED_ON;
        g_led_task.next_edge_ms = now + 400;
        break;
    }
}
```

**关键时间参数**（可调）：
- 单次亮/灭：400ms（人眼能数清）
- 轮间间隔：2000ms（题目"间隔数秒"）
- 重复轮数：2 轮够展示，固件可选支持更多

**与 0x33 激光帧的关系**：两路完全独立，不要共用 GPIO。激光 0x33 是开/关电平直驱，LED 0x12 是固件内部闪烁状态机驱动。

### 13.7 解析骨架（参考实现）

STM32 端 UART 接收中断里把字节喂进解析状态机：

```c
typedef enum {
    PARSE_HEADER, PARSE_ADDR, PARSE_ID, PARSE_LEN, PARSE_DATA, PARSE_SUM, PARSE_ADD
} parse_state_t;

static parse_state_t state = PARSE_HEADER;
static uint8_t  id, len, idx;
static uint8_t  data_buf[260];
static uint8_t  sum_acc, add_acc;
static uint8_t  recv_sum;

void uart_rx_byte(uint8_t b) {
    switch (state) {
      case PARSE_HEADER:
        if (b == 0xAA) {
            sum_acc = b; add_acc = b;
            state = PARSE_ADDR;
        }
        break;
      case PARSE_ADDR:
        if (b == 0xFF) {
            sum_acc += b; add_acc += sum_acc;
            state = PARSE_ID;
        } else {
            state = PARSE_HEADER;
        }
        break;
      case PARSE_ID:
        id = b;
        sum_acc += b; add_acc += sum_acc;
        state = PARSE_LEN;
        break;
      case PARSE_LEN:
        len = b; idx = 0;
        sum_acc += b; add_acc += sum_acc;
        state = (len == 0) ? PARSE_SUM : PARSE_DATA;
        break;
      case PARSE_DATA:
        data_buf[idx++] = b;
        sum_acc += b; add_acc += sum_acc;
        if (idx == len) state = PARSE_SUM;
        break;
      case PARSE_SUM:
        recv_sum = b;
        state = PARSE_ADD;
        break;
      case PARSE_ADD:
        if (recv_sum == sum_acc && b == add_acc) {
            // 校验通过 → 派发
            on_protocol_frame(id, data_buf, len);
        }
        state = PARSE_HEADER;
        break;
    }
}

void on_protocol_frame(uint8_t id, const uint8_t *data, uint8_t len) {
    switch (id) {
      case 0x31:  // TARGET_VELOCITY，4×float32 LE
        if (len == 16) on_target_velocity(/* 解析 */);
        break;
      case 0x33:  // ELECTROMAGNET（激光复用）
        if (len == 1) gpio_set_laser(data[0]);
        break;
      case 0x12:  // LED_DIGIT，G 题新增
        if (len == 1) on_led_digit_frame(data[0]);
        break;
      case 0x11:
        if (len == 1) servo_set(data[0]);
        break;
      case 0x22:
        if (len == 1) buzzer_led_set(data[0]);
        break;
      case 0x66:
        if (len == 1 && data[0] == 0x06) mission_complete();
        break;
      // …
    }
}
```

固件如果用搬运代码的解析器，只需在 `on_protocol_frame` 里加 `case 0x12` 这一条分支即可。

