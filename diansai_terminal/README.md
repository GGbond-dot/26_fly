# 立体货架盘点无人机地面站

这是 D 题地面站部分的 PyQt5 + ROS 2 Humble 实现。应用默认全屏运行，右上角有关闭按钮；通过 ROS 2 DDS 订阅无人机盘点结果并发布定向盘点目标库位。

界面左侧是定点控制栏，内容区采用页签切换：`仓库地图` 页用于现场观察盘点进度，`盘点表格` 页用于查看 24 个坐标的详细结果。仓库图中未盘点点位为红色，成功盘点后变为绿色，异常结果为橙色；定点盘点路径按横平竖直的折线显示。PDF 只要求地面站实时显示盘点结果、结束后查询坐标、定点盘点显示航线图；计时属于测试评分条件，因此主界面不单独显示计时器。

## 运行

```bash
cd <地面站目录>
source /opt/ros/humble/setup.bash
source scripts/setup_dds.sh        # 跨机 DDS 环境，与飞机端同 ROS_DOMAIN_ID
python3 -m ground_station.main
```

调试窗口模式：

```bash
python3 -m ground_station.main --windowed
```

无 ROS 演示模式：

```bash
python3 -m ground_station.main --windowed --no-ros --demo
```

## ROS 话题

话题与消息格式直接对齐飞机 `inventory_control_pkg/inventory_mission_node`（无 `/uav` 前缀）。

地面站订阅（飞机发布）：

- `/inventory_result`，`std_msgs/String`，内容 `编号=7,货位=B3`（遍历盘点逐货位上报）
- `/inventory_target`，`std_msgs/String`：起飞前先报送编号 `7`；地面站下发货位后飞机回报确认 `目标编号=7,货位=C5`
- `/inventory_led`，`std_msgs/Empty`：每盘点一个驱动 LED 亮灭一次
- `/mission_complete`，`std_msgs/Empty`：本轮任务完成

地面站发布（飞机订阅）：

- `/inventory_target_slot`，`std_msgs/String`，内容为目标库位，例如 `C5`（定向盘点时地面站查表后下发）

## 跨机网络

两块香橙派需要处在同一网络。两端各自 `source scripts/setup_dds.sh`，
保证 `ROS_DOMAIN_ID` 一致（默认 26）、`ROS_LOCALHOST_ONLY=0`，且统一用
FastDDS（`RMW_IMPLEMENTATION=rmw_fastrtps_cpp`，两端 RMW 必须相同）。
多网口（docker0/有线/无线）发现不稳时，用 `NET_IFACE=wlan0 source scripts/setup_dds.sh`
把 FastDDS 绑定到实际联网的网卡。飞机端对应脚本在 26fly 仓库 `scripts/setup_dds.sh`。

## 模拟无人机

在另一个终端运行：

```bash
cd <地面站目录>
source /opt/ros/humble/setup.bash
source scripts/setup_dds.sh
python3 -m ground_station.fake_uav --delay 0.2 --target-id 7
```

模拟器按真实飞机的话题/格式依次发布 24 个 `/inventory_result`（`编号=N,货位=XY`）
和 `/inventory_led` 脉冲，再发 `/mission_complete`，然后报送目标编号到 `/inventory_target`；
收到地面站下发的 `/inventory_target_slot` 后回报确认。

## 配置和数据

- 航点配置：`config/slot_waypoints.json`
- 盘点状态：`data/inventory_state.json`

航点坐标采用仓库左下角为原点，单位米，`x` 向右，`y` 向上，`z` 为飞行高度。当前坐标按 PDF 场地图给出初始值，现场调试时可直接微调 `config/slot_waypoints.json`。

## 测试

```bash
python3 -m unittest discover -s tests
QT_QPA_PLATFORM=offscreen python3 -m ground_station.main --windowed --no-ros --smoke-test
```
