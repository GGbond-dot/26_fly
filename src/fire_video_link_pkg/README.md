# fire_video_link_pkg — 无人机↔消防车 跨域话题桥（视频流）

把飞机内部域（26）上的视频流搬到与消防车共享的域（6），**只搬显式列出的话题**，
飞机其余几十个话题继续留在 26，车端看不见、也不会重名。

与 `fire_link_pkg` 的关系：**互补，不替代**。

| 通道 | 走什么 | 归谁管 |
|---|---|---|
| 火源坐标 / 遥测 / 按键启动 / 任务状态 | 定长 UDP 包 | `fire_link_pkg`（已实现，别动） |
| **视频流** | **DDS 跨域桥** | 本包 |

控制面用 UDP 是因为包小、格式固定、双方契约清楚；视频流塞不进 32 字节定长包，
且实测 DDS 传图像是最优的，所以单开一条通道。

## 为什么要有这个桥

`ROS_DOMAIN_ID` 是**进程级**环境变量，ROS 2 没有"给某个话题单独指定域"的机制。
但一个进程里可以建多个 `rclcpp::Context`，每个绑一个域——本节点同时是域 26 和
域 6 的成员，把配置里列出的话题从前者转发到后者。

另一条路是给所有话题加命名空间前缀，但本仓库话题名几乎全是绝对名
（`/target_position`、`/height`…），绝对名不吃 `ROS_NAMESPACE` 也不吃 `__ns:=`，
等于要改一遍全部源码，不划算。

转发用 `GenericSubscription/GenericPublisher`，拿到的是已序列化字节流，
**不反序列化、不重新编码**，一帧图像只多一次内存拷贝。副作用是加新话题
只改 yaml、不用重新编译。

## 用法

```bash
source scripts/setup_dds.sh          # 必须 source，且 ROS_LOCALHOST_ONLY=0
ros2 launch fire_video_link_pkg fire_video_bridge.launch.py
```

改共享域不用动配置文件：

```bash
ros2 launch fire_video_link_pkg fire_video_bridge.launch.py outer_domain:=10
```

配置见 `config/video_bridge.yaml`，规则格式 `源话题|目标话题|类型|可靠性|深度`。

## 验证链路（分三段查，别一次查到底）

**① 飞机内部域收得到吗**

```bash
ROS_DOMAIN_ID=26 ros2 topic hz /fire/debug_image
```

**② 桥转发了吗** —— 看桥自己的日志，每 5 秒一行：

```
[桥] /fire/debug_image → /fly/fire/debug_image  14.8 Hz  912 KB/s
```

打 `近 5s 无数据` 就是入口没收到，问题在 ① 那段，跟网络无关。

**③ 共享域上出来了吗** —— 飞机上另开终端：

```bash
ROS_DOMAIN_ID=6 ros2 topic hz /fly/fire/debug_image
```

② 有数而 ③ 没有，基本只有一个原因：类型名写错了（`ros2 topic info /fire/debug_image`
对一下）。② ③ 都有而车端收不到，那就是网络问题，见下。

## 坑（按踩到的概率排序）

1. **域号必须跟队友对准。** `fire_link_pkg/README.md` 里写车端是 **10**，
   而队友口头说的是 **6**。域号错 = 完全收不到 + 零报错。以实机 `echo $ROS_DOMAIN_ID` 为准。
2. **类型名要逐字一致**，`sensor_msgs/msg/Image` 不是 `sensor_msgs/Image`。写错不报错，只是收不到。
3. **QoS 方向性**：best_effort 的订阅能收 reliable 的发布，反过来不行。所以图像入口用
   best_effort 最保险；如果源端是 best_effort 而你把桥配成 reliable，一帧都收不到。
4. **路由器可能不转发 DDS 多播。** 廉价 AP 对 Wi-Fi 多播很不友好，表现是同网段
   `ping` 得通但 `ros2 topic list` 互相看不见。这时给 FastDDS 配单播发现（initial peers），
   把对方 IP 写死——两边 IP 本来就是固定的（车 `192.168.10.113` / 机 `192.168.10.197`）。
5. **带宽**。原始 `sensor_msgs/Image` 640×480×3 @15Hz ≈ 13 MB/s，Wi-Fi 上会拖垮
   同链路的 UDP 控制包。所以传的是 `CompressedImage`，发送侧
   （`fire_vision_pkg` 的 `debug_image_*` 参数）限到 320 宽 / 10 Hz / 800 kbps，
   超码率时**降画质保帧率**。这几个值往大调之前先想清楚会挤掉谁。
6. `ROS_LOCALHOST_ONLY` 必须为 0，否则共享域出不了本机（`setup_dds.sh` 已设）。
