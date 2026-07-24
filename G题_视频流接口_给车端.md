# G 题 视频流接口（无人机 → 消防车）— 车端说明

> 2026-07-23。这是 `G题_机车通信接口约定.md` §十 的车端节选，可以单独看。
> 无人机侧已实现完毕（`fire_video_link_pkg` + `fire_vision_pkg`），**车端只需订阅一个话题**。

---

## 一句话

无人机把火源识别的调试画面（带检测框）以 **JPEG 压缩图**推到**你本来就在的域 6** 上，
话题 `/fly/fire/debug_image/compressed`，你订阅就能显示。**车端不需要改任何现有代码，
也不需要装新包。**

---

## 接口定义

| 项 | 值 |
|---|---|
| `ROS_DOMAIN_ID` | **6**（你车端本来的域，不用改） |
| 话题 | `/fly/fire/debug_image/compressed` |
| 类型 | `sensor_msgs/msg/CompressedImage` |
| `format` 字段 | `"jpeg"` |
| **QoS** | **`best_effort` + `KEEP_LAST(1)`** ← 见下面红字 |
| 分辨率 | 宽 320（高按原图比例，一般 240） |
| 帧率 | ≤ 10 Hz |
| 码率 | ≤ 800 kbps |
| `header.frame_id` | `laser_link` |
| `header.stamp` | 无人机本地时钟。**别用它和车端时钟做差**，两块板没做时钟同步 |

### 🔴 QoS 必须是 best_effort

用 `reliable` 订阅**一帧都收不到**，而且不会有任何报错——表现就是"话题在列表里但 echo 不出东西"。
DDS 的规则是 best_effort 的订阅可以收 reliable 的发布，反过来不行，我们发的是 best_effort。

Python 直接用现成的 `qos_profile_sensor_data` 就对了：

```python
from rclpy.qos import qos_profile_sensor_data
```

C++ 用 `rclcpp::SensorDataQoS()`。

**这是这条链路唯一容易踩的坑，如果你收不到图，先查这个。**

---

## 最快验收（不用写代码）

在车上，确认自己在域 6，然后：

```bash
echo $ROS_DOMAIN_ID                                  # 必须是 6
ros2 topic list | grep fly                           # 应该看到 /fly/fire/debug_image/compressed
ros2 topic hz  /fly/fire/debug_image/compressed      # 应该 ~10 Hz
ros2 topic bw  /fly/fire/debug_image/compressed      # 应该 < 100 KB/s
```

看画面：

```bash
ros2 run rqt_image_view rqt_image_view /fly/fire/debug_image/compressed
```

> `ros2 topic hz/bw` 内部用的是兼容 QoS，所以它能收到不代表你的节点也能收到——
> 你自己的订阅仍然必须显式写 best_effort。

前提：无人机侧要起了 `fire_video_bridge` 且视觉节点开了 `enable_debug_image`。
对拉之前跟我说一声，我把飞机侧起好。

---

## 集成到 `fire_dashboard.py`

只依赖 `cv2` + `numpy`，**故意不用 `cv_bridge`**（`CompressedImage` 用 `cv2.imdecode`
两行就解完了，没必要为此引一个依赖）：

```python
import cv2
import numpy as np
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import CompressedImage

# ---- 在你的 dashboard 节点 __init__ 里 ----
self.drone_frame = None          # 最近一帧 BGR 图，None = 还没收到
self.drone_frame_at = 0.0        # 收到时刻，用来判"链路断了"
self.create_subscription(
    CompressedImage,
    "/fly/fire/debug_image/compressed",
    self._on_drone_image,
    qos_profile_sensor_data,     # ← 必须，别换成 10
)

def _on_drone_image(self, msg: CompressedImage) -> None:
    buf = np.frombuffer(msg.data, dtype=np.uint8)
    img = cv2.imdecode(buf, cv2.IMREAD_COLOR)
    if img is None:              # 半帧/坏帧，丢掉就行，下一帧马上到
        return
    self.drone_frame = img
    self.drone_frame_at = time.monotonic()

# ---- 在你的刷新循环里 ----
def _draw_drone_view(self):
    stale = time.monotonic() - self.drone_frame_at > 2.0
    if self.drone_frame is None:
        return None              # 画个 "等待无人机画面"
    if stale:
        pass                     # 画面还在但已过期，建议叠一层灰或标 "LOST"
    return self.drone_frame
```

**建议加"超过 2 秒没新帧就标 LOST"**：这条链路是 best_effort，丢帧是正常的，
但整条断了和画面卡住在屏幕上长得一模一样，现场分不清会浪费很多时间。

---

## 带宽约定：**请不要要求我调大**

优先级是定死的：

> **UDP 控制包（遥测 8892 / 按键启动 8893）> 视频流帧率 > 视频画质**

控制包丢了直接丢分（基本要求 2 和 3/4），视频糊一点没人扣分。
所以飞机侧限到 320 宽 / 10 Hz / 800 kbps，超码率时**自动降 JPEG 质量、不降帧率**。

如果你觉得画面太糊，我们可以在实测控制包没受影响的前提下调，但**不能反过来先调清晰再看链路稳不稳**。

---

## 这条通道和原来四条 UDP 通道的关系

**完全独立并存，原来的一行都没改。**

| 通道 | 走什么 | 端口/域 |
|---|---|---|
| 火源坐标上报 | UDP | 车 :8889 |
| 任务状态回传 | UDP | 机 :8890 |
| 遥测 | UDP | 车 :8892 |
| 按键启动 | UDP | 机 :8893 |
| **视频流（新）** | **DDS** | **域 6** |

视频流为什么不走 UDP：帧塞不进 32 字节定长包，自己做分片重组等于重写传输层。

### 无人机为什么能出现在域 6 上

飞机内部跑在域 26，我在飞机上加了个桥进程，它同时是域 26 和域 6 的成员，
**只把上面那一个话题**搬到域 6。飞机其余几十个话题（`/target_position`、`/height`…）
仍然只在域 26，你在车上看不见，**也不会和你的话题重名**。

`/fly` 前缀是桥加的，专门跟你车端的话题分开。以后要加别的跨机话题，我在桥的配置里加一行就行。

---

## 收不到图怎么办（按这个顺序查）

1. **你的订阅 QoS 是 best_effort 吗** —— 十次有八次是这个
2. 类型是 `sensor_msgs/msg/CompressedImage`，不是 `Image`，话题末尾有 `/compressed`
3. `echo $ROS_DOMAIN_ID` 是不是 6
4. `ros2 topic list | grep fly` 有没有这个话题
   - **有** → 问题在你这侧（QoS/类型）
   - **没有** → 问题在飞机侧或网络，告诉我，我看飞机上桥的日志（它每 5 秒打一行吞吐，
     能直接分清是"飞机没发"还是"网络没过来"）
5. 两机 `ping` 得通但 `topic list` 互相看不见 → 路由器不转发 DDS 多播，
   这种情况我们两边都要配 FastDDS 单播发现（IP 是固定的：车 `.113` / 机 `.197`）

## 自测脚本

飞机侧仓库 `scripts/video_link_test.py` 可以直接拷到车上跑，只依赖 `rclpy`：

```bash
python3 video_link_test.py --dst-only
```

它会在域 6 上订阅、统计帧率码率，并直接告诉你问题在哪一侧。
