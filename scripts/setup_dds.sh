#!/usr/bin/env bash
# D题 盘点无人机 ←→ 地面站 跨机 DDS / ROS2 通信环境配置（飞机端）
#
# 在启动 ROS 节点前 source 本脚本。飞机端与地面站端 **必须配置一致**
# （地面站端见 diansai_terminal/scripts/setup_dds.sh，两边 ROS_DOMAIN_ID 要相同）：
#
#   source <ws>/scripts/setup_dds.sh
#
# autostart_fly.sh 已自动 source 本脚本。手动联调时在 ros2 launch 前先 source。
#
# 关键点（两端必须一致）：
#   ROS_DOMAIN_ID         同域才能互相发现，固定 26（强制覆盖外部值，
#                         临时改域用 DIANSAI_DOMAIN_ID）
#   ROS_LOCALHOST_ONLY=0  跨机通信必须为 0（=1 只走本机回环，发现不到对端）
#   RMW_IMPLEMENTATION    统一用 FastDDS（rmw_fastrtps_cpp），两端 RMW 必须相同
#
# 注意：本脚本只导出环境变量，不 source ROS（humble 已在 .bashrc / autostart 里 source）。

# —— 同一网络下两端必须相同 ——
# 强制覆盖：不沿用外部已有的 ROS_DOMAIN_ID。.bashrc / systemd 里残留的
# ROS_DOMAIN_ID=0 曾把本脚本静默劫持到默认域 0（谁都能进来的公共域）。
# 要临时换域，用 DIANSAI_DOMAIN_ID（两端必须给同一个值）：
#   DIANSAI_DOMAIN_ID=31 source scripts/setup_dds.sh
_WANT_DOMAIN="${DIANSAI_DOMAIN_ID:-26}"
if [ -n "${ROS_DOMAIN_ID:-}" ] && [ "${ROS_DOMAIN_ID}" != "$_WANT_DOMAIN" ]; then
  echo "[dds] 覆盖外部 ROS_DOMAIN_ID=${ROS_DOMAIN_ID} → ${_WANT_DOMAIN}"
fi
export ROS_DOMAIN_ID="$_WANT_DOMAIN"
unset _WANT_DOMAIN
export ROS_LOCALHOST_ONLY=0
export RMW_IMPLEMENTATION=rmw_fastrtps_cpp

# —— 可选：网卡绑定 ——
# 香橙派常有 docker0 / 有线 / 无线多个网口，DDS 广播可能跑错口导致发现不稳。
# 发现不到对端时，指定实际联网的网卡（如无线 wlan0）把 FastDDS 限定到它：
#   NET_IFACE=wlan0 source scripts/setup_dds.sh
if [ -n "${NET_IFACE:-}" ]; then
  _IP=$(ip -4 -o addr show "$NET_IFACE" 2>/dev/null | awk '{print $4}' | cut -d/ -f1 | head -n1)
  if [ -z "$_IP" ]; then
    echo "[dds] 警告：网卡 ${NET_IFACE} 没有 IPv4 地址，跳过绑定"
  else
    _DDS_XML="${TMPDIR:-/tmp}/fastdds_${NET_IFACE}.xml"
    cat > "$_DDS_XML" <<EOF
<?xml version="1.0" encoding="UTF-8" ?>
<dds xmlns="http://www.eprosima.com/XMLSchemas/fastRTPS_Profiles">
  <profiles>
    <transport_descriptors>
      <transport_descriptor>
        <transport_id>udp_iface</transport_id>
        <type>UDPv4</type>
        <interfaceWhiteList>
          <address>${_IP}</address>
        </interfaceWhiteList>
      </transport_descriptor>
    </transport_descriptors>
    <participant profile_name="iface_profile" is_default_profile="true">
      <rtps>
        <userTransports>
          <transport_id>udp_iface</transport_id>
        </userTransports>
        <useBuiltinTransports>false</useBuiltinTransports>
      </rtps>
    </participant>
  </profiles>
</dds>
EOF
    export FASTRTPS_DEFAULT_PROFILES_FILE="$_DDS_XML"
    echo "[dds] FastDDS 绑定网卡=${NET_IFACE} (IP=${_IP}), profile=$_DDS_XML"
  fi
fi

echo "[dds] ROS_DOMAIN_ID=$ROS_DOMAIN_ID  ROS_LOCALHOST_ONLY=$ROS_LOCALHOST_ONLY  RMW=$RMW_IMPLEMENTATION"
