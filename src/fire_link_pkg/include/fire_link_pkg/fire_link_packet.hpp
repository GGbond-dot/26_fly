// fire_link_packet.hpp — 无人机 ↔ 消防车 UDP 包格式（2023 电赛 G 题）
//
// ⚠️ 本文件在两个仓库里各存一份，必须**逐字节一致**：
//     无人机侧 kian_26fly/src/fire_link_pkg/include/fire_link_pkg/fire_link_packet.hpp
//     消防车侧 kian_flycar/car/<接收包>/include/.../fire_link_packet.hpp
//   改一处必须同步改另一处，否则 static_assert 过得去、字段会错位。
//
// 为什么不复用跟随用的 24 字节 PosePacket：
//   那条链路是单向的、只装 x/y/yaw，且已在跟随功能上实测通过，不宜改动。
//   本题需要双向（车→机的按键启动）且要装火源坐标 / 累计里程 / 任务阶段。
//
// 小端定长 32 字节。不依赖 DDS 跨机发现（两板 ROS_DOMAIN_ID 不同，DDS 本来也不互通）；
// 丢包不重传——遥测下一包就是最新状态，一次性事件（火源上报/启动）靠连发多次做冗余。

#pragma once

#include <cstdint>

namespace fire_link
{

constexpr uint16_t kMagic = 0xF14E;   // 'FN' fire net

enum PacketType : uint8_t
{
  TYPE_TELEMETRY   = 1,   // 机→车：无人机位置 + 累计里程 + 高度 + 阶段（基本要求 3/4）
  TYPE_CAR_START   = 3,   // 车→机：消防车上按键启动无人机（基本要求 2），连发多次
};
// 注：火源上报**不用**本包型，它走车端早就实现好的 fire_event_bridge（见下方 FireEventPacket）。
// 曾经占用的 type=2 (FIRE_REPORT) 已废弃，不要复用这个值，免得跟旧固件/旧脚本撞上。

// 任务阶段，与 fire_control_pkg::MissionPhase 一一对应，供车端显示器显示飞机在干什么。
enum LinkPhase : uint8_t
{
  PHASE_WAIT_START = 0,
  PHASE_TAKEOFF    = 1,
  PHASE_PATROL     = 2,
  PHASE_APPROACH   = 3,
  PHASE_DESCEND    = 4,
  PHASE_HOVER      = 5,
  PHASE_DROP       = 6,
  PHASE_RESUME     = 7,
  PHASE_RETURN     = 8,
  PHASE_LAND       = 9,
  PHASE_DONE       = 10,
  PHASE_UNKNOWN    = 255,
};

// 坐标一律为**题目场地系 dm**（巡防区左下角为原点，右上角 (48,40)），
// 车端拿到即可直接显示 / 直接当目标点用，两边都不需要再做坐标换算。
struct __attribute__((packed)) FireLinkPacket
{
  uint16_t magic;         // kMagic
  uint8_t  type;          // PacketType
  uint8_t  phase;         // LinkPhase（仅 TYPE_TELEMETRY 有意义）
  uint16_t seq;           // 按 type 各自独立递增；接收端用 int16 差值判新旧，自然处理回绕
  uint16_t reserved16;
  uint32_t stamp_ms;      // 发送方单调时钟，接收端只用"收包本地时间"判超时，不做跨机时钟同步
  float    x_dm;          // TELEMETRY=无人机位置；FIRE_REPORT=火源位置；CAR_START=0
  float    y_dm;
  float    distance_dm;   // 累计巡逻里程（仅 TELEMETRY）
  float    height_dm;     // 离地高度（仅 TELEMETRY）
  uint32_t reserved32;
};

static_assert(sizeof(FireLinkPacket) == 32, "FireLinkPacket must be 32 bytes");

// ─────────────────────────────────────────────────────────────────────────
// 火源坐标上报（机 → 车 :8889）
//
// 这个包型是**车端 fire_event_bridge 早就实现好的**，无人机侧主动适配，
// 车端一行都不用改。格式与车端 `fire_event_bridge.cpp` 里的 Packet 一致。
constexpr uint16_t kFireEventMagic = 0xFC11;

struct __attribute__((packed)) FireEventPacket
{
  uint16_t magic;      // kFireEventMagic
  uint16_t seq;        // 递增；车端按 int16 差值判新旧，同 seq 连发即天然去重
  float    x_dm;       // 火源 x，场地系 dm
  float    y_dm;       // 火源 y，场地系 dm
  float    reserved;   // 置 0
};

static_assert(sizeof(FireEventPacket) == 16, "FireEventPacket must be 16 bytes");

}  // namespace fire_link
