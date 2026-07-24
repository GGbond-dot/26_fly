// fire_video_bridge_node —— 跨 ROS_DOMAIN_ID 话题桥（无人机 26 → 消防车 6）
//
// 为什么需要它：ROS_DOMAIN_ID 是**进程级**环境变量，没有"给某个话题单独指定域"
// 的机制。但一个进程里可以建多个 rclcpp::Context，每个 Context 绑一个域，
// 于是本节点同时是域 26 和域 6 的成员，把指定话题从前者搬到后者。
//
// 这样飞机内部几十个话题继续留在域 26，车端完全看不见，不会和车端重名；
// 只有这里显式列出的少数几个（视频流）出现在共享域 6 上，且能顺手改名加前缀。
//
// 转发走 GenericSubscription/GenericPublisher：拿到的是已序列化的字节流，
// **不反序列化、不重新编码**，一帧图像只多一次内存拷贝，比 decode/encode 便宜得多。
// 副作用是本节点不依赖 sensor_msgs 等任何消息包，加新话题只改配置不用重新编译。

#include <atomic>
#include <chrono>
#include <memory>
#include <sstream>
#include <string>
#include <thread>
#include <vector>

#include <rclcpp/rclcpp.hpp>
#include <rclcpp/generic_publisher.hpp>
#include <rclcpp/generic_subscription.hpp>

namespace
{

// 配置里一条转发规则的解析结果。
struct BridgeRule
{
  std::string src_topic;    // 内部域（26）上的原话题名
  std::string dst_topic;    // 共享域（6）上发布的名字
  std::string type_name;    // 形如 sensor_msgs/msg/Image
  std::string reliability;  // best_effort | reliable
  int depth = 5;

  // 转发计数，仅用于周期性打印链路状态
  std::atomic<uint64_t> msg_count{0};
  std::atomic<uint64_t> byte_count{0};
};

// 规则字符串格式："源话题|目标话题|类型[|可靠性[|深度]]"
// 用 | 而不是 : 分隔，是因为话题名和类型名里都可能出现 : 和 /。
bool parseRule(const std::string & line, BridgeRule & out, std::string & err)
{
  std::vector<std::string> f;
  std::stringstream ss(line);
  std::string item;
  while (std::getline(ss, item, '|')) {
    f.push_back(item);
  }
  if (f.size() < 3) {
    err = "字段少于3个（至少 源话题|目标话题|类型）";
    return false;
  }
  out.src_topic = f[0];
  out.dst_topic = f[1];
  out.type_name = f[2];
  out.reliability = (f.size() > 3 && !f[3].empty()) ? f[3] : "best_effort";
  if (f.size() > 4 && !f[4].empty()) {
    try {
      out.depth = std::stoi(f[4]);
    } catch (const std::exception &) {
      err = "深度不是整数: " + f[4];
      return false;
    }
  }
  if (out.reliability != "best_effort" && out.reliability != "reliable") {
    err = "可靠性只能是 best_effort / reliable，收到: " + out.reliability;
    return false;
  }
  if (out.src_topic.empty() || out.dst_topic.empty() || out.type_name.empty()) {
    err = "话题名或类型为空";
    return false;
  }
  return true;
}

rclcpp::QoS makeQos(const BridgeRule & r)
{
  rclcpp::QoS qos(rclcpp::KeepLast(static_cast<size_t>(r.depth)));
  if (r.reliability == "reliable") {
    qos.reliable();
  } else {
    qos.best_effort();
  }
  return qos;
}

// 建一个绑定到指定域的 Context。关键就是 set_domain_id——它让这个 Context 下
// 的所有节点/participant 跑在该域，与进程的 ROS_DOMAIN_ID 环境变量无关。
rclcpp::Context::SharedPtr makeContext(int domain_id, int argc, char ** argv)
{
  auto ctx = std::make_shared<rclcpp::Context>();
  rclcpp::InitOptions opts;
  opts.set_domain_id(static_cast<size_t>(domain_id));
  // 默认 shutdown_on_signal=true：Ctrl-C / systemd stop 时两个 Context 都会被
  // 一起 shutdown，两个 executor 的 spin 才能正常退出。
  ctx->init(argc, argv, opts);
  return ctx;
}

}  // namespace

int main(int argc, char ** argv)
{
  // 先用默认 Context 起一个"配置节点"，只为了读参数（launch 传进来的）。
  // 它跑在进程的 ROS_DOMAIN_ID 上，不订阅不发布，不参与转发。
  rclcpp::init(argc, argv);
  auto cfg_node = std::make_shared<rclcpp::Node>("fire_video_bridge_cfg");

  const int inner_domain = cfg_node->declare_parameter<int>("inner_domain", 26);
  const int outer_domain = cfg_node->declare_parameter<int>("outer_domain", 6);
  const double stats_period = cfg_node->declare_parameter<double>("stats_period_sec", 5.0);
  const auto rule_strs =
    cfg_node->declare_parameter<std::vector<std::string>>("rules", std::vector<std::string>{});

  auto logger = cfg_node->get_logger();

  if (rule_strs.empty()) {
    RCLCPP_ERROR(logger, "参数 rules 为空，没有任何话题要桥接，退出。");
    rclcpp::shutdown();
    return 1;
  }
  if (inner_domain == outer_domain) {
    RCLCPP_ERROR(
      logger, "inner_domain 和 outer_domain 都是 %d，桥没有意义（会自己转给自己）。退出。",
      inner_domain);
    rclcpp::shutdown();
    return 1;
  }

  std::vector<std::shared_ptr<BridgeRule>> rules;
  for (const auto & s : rule_strs) {
    auto r = std::make_shared<BridgeRule>();
    std::string err;
    if (!parseRule(s, *r, err)) {
      RCLCPP_ERROR(logger, "规则解析失败 [%s]：%s", s.c_str(), err.c_str());
      rclcpp::shutdown();
      return 1;
    }
    rules.push_back(r);
  }

  RCLCPP_INFO(
    logger, "跨域桥启动：内部域 %d → 共享域 %d，共 %zu 条话题",
    inner_domain, outer_domain, rules.size());

  // —— 两个域各建一个 Context + 节点 ——
  auto ctx_in = makeContext(inner_domain, argc, argv);
  auto ctx_out = makeContext(outer_domain, argc, argv);

  rclcpp::NodeOptions opt_in;
  opt_in.context(ctx_in);
  rclcpp::NodeOptions opt_out;
  opt_out.context(ctx_out);

  auto node_in = std::make_shared<rclcpp::Node>("fire_video_bridge_in", opt_in);
  auto node_out = std::make_shared<rclcpp::Node>("fire_video_bridge_out", opt_out);

  std::vector<rclcpp::GenericSubscription::SharedPtr> subs;
  std::vector<rclcpp::GenericPublisher::SharedPtr> pubs;

  for (const auto & r : rules) {
    const auto qos = makeQos(*r);

    // 出口先建，入口回调里才有得发。
    auto pub = node_out->create_generic_publisher(r->dst_topic, r->type_name, qos);

    auto sub = node_in->create_generic_subscription(
      r->src_topic, r->type_name, qos,
      [pub, r](std::shared_ptr<rclcpp::SerializedMessage> msg) {
        r->msg_count.fetch_add(1, std::memory_order_relaxed);
        r->byte_count.fetch_add(msg->size(), std::memory_order_relaxed);
        pub->publish(*msg);
      });

    pubs.push_back(pub);
    subs.push_back(sub);

    RCLCPP_INFO(
      logger, "  [域%d] %s  →  [域%d] %s   (%s, %s, depth=%d)",
      inner_domain, r->src_topic.c_str(), outer_domain, r->dst_topic.c_str(),
      r->type_name.c_str(), r->reliability.c_str(), r->depth);
  }

  // —— 周期性打印吞吐 ——
  // 比赛现场看不到 rqt，链路通没通只能靠日志判断：帧数一直是 0 就是没收到，
  // 帧数涨但车端收不到就是共享域/网络的问题，能一眼分清故障在桥的哪一侧。
  rclcpp::TimerBase::SharedPtr stats_timer;
  if (stats_period > 0.0) {
    stats_timer = node_in->create_wall_timer(
      std::chrono::duration<double>(stats_period),
      [rules, logger, stats_period]() {
        for (const auto & r : rules) {
          const uint64_t n = r->msg_count.exchange(0, std::memory_order_relaxed);
          const uint64_t b = r->byte_count.exchange(0, std::memory_order_relaxed);
          if (n == 0) {
            RCLCPP_WARN(
              logger, "[桥] %s 近 %.0fs 无数据（源节点没起？话题名/QoS 不匹配？）",
              r->src_topic.c_str(), stats_period);
          } else {
            RCLCPP_INFO(
              logger, "[桥] %s → %s  %.1f Hz  %.0f KB/s",
              r->src_topic.c_str(), r->dst_topic.c_str(),
              static_cast<double>(n) / stats_period,
              static_cast<double>(b) / stats_period / 1024.0);
          }
        }
      });
  }

  // 每个 Context 必须有自己的 executor（executor 也要显式绑 context，
  // 这是多域桥最容易漏的一步：不绑就默认用全局 context，节点根本不会被 spin）。
  rclcpp::ExecutorOptions eo_in;
  eo_in.context = ctx_in;
  rclcpp::ExecutorOptions eo_out;
  eo_out.context = ctx_out;

  auto exec_in = std::make_shared<rclcpp::executors::SingleThreadedExecutor>(eo_in);
  auto exec_out = std::make_shared<rclcpp::executors::SingleThreadedExecutor>(eo_out);
  exec_in->add_node(node_in);
  exec_out->add_node(node_out);

  std::thread t_out([exec_out]() {exec_out->spin();});
  exec_in->spin();

  exec_out->cancel();
  if (t_out.joinable()) {
    t_out.join();
  }

  rclcpp::shutdown(ctx_in);
  rclcpp::shutdown(ctx_out);
  rclcpp::shutdown();
  return 0;
}
