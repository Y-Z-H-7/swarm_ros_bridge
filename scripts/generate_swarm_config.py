#!/usr/bin/env python3
"""
自动生成地面站与无人机群的 swarm_ros_bridge 配置文件。

一键生成：
  - ros_topics_gcs.yaml          （地面站配置）
  - ros_topics_drone{N}.yaml     （每架无人机的配置）
  - bridge_drone{N}.launch       （每架无人机的 launch 文件）

用法：
  命令行模式：
    python3 generate_swarm_config.py --gcs 192.168.0.137 \\
        --drones 192.168.0.121 192.168.0.195 192.168.0.243 \\
        --output-dir ../config/

  交互模式（不带参数直接运行）：
    python3 generate_swarm_config.py
"""

import argparse
import os
import sys
from typing import List, Tuple


# ─── 每架无人机发送的话题（这里使用接收端的名称）───────────────────
# 格式: (topic_name_template, msg_type)
# {drone_id} 会被替换为无人机编号
DRONE_TOPICS: List[Tuple[str, str]] = [
    ("/swarm_expl/drone_state_recv_{drone_id}", "exploration_manager/DroneState"),
    ("/swarm_expl/pair_opt_recv_{drone_id}", "exploration_manager/PairOpt"),
    ("/swarm_expl/pair_opt_res_recv_{drone_id}", "exploration_manager/PairOptResponse"),
    ("/planning/swarm_traj_recv_{drone_id}", "bspline/Bspline"),
    ("/swarm_expl/hgrid_recv_{drone_id}", "exploration_manager/HGrid"),
    ("/swarm_expl/grid_tour_recv_{drone_id}", "exploration_manager/GridTour"),
    ("/multi_map_manager/chunk_stamps_recv_{drone_id}", "plan_env/ChunkStamps"),
    ("/multi_map_manager/chunk_data_recv_{drone_id}", "plan_env/ChunkData"),
    ("/planning_vis/viewpoints_{drone_id}", "visualization_msgs/Marker"),
    ("/sdf_map/occupancy_all_{drone_id}", "sensor_msgs/PointCloud2"),
    ("/planning/travel_traj_{drone_id}", "visualization_msgs/Marker"),
    ("/planning/position_cmd_vis_{drone_id}", "visualization_msgs/Marker"),
    # Ground-station telemetry.  These topics must be bridged because the
    # ground station is not connected to each aircraft's ROS master directly.
    ("/uav{drone_id}/mavros/battery", "sensor_msgs/BatteryState"),
    ("/uav{drone_id}/mavros/local_position/pose", "geometry_msgs/PoseStamped"),
]

# 发送 topic 数量（用于端口范围）
NUM_DRONE_TOPICS = len(DRONE_TOPICS)

# 每台机载电脑有独立的 ROS master，MAVROS 默认没有 /uavN 前缀。
# TCP 端口标识数据通道，接收端可以使用不同的 topic_name 发布消息。
TELEMETRY_SOURCES = {
    "/uav{drone_id}/mavros/battery": "/mavros/battery",
    "/uav{drone_id}/mavros/local_position/pose": "/mavros/local_position/pose",
}

# RACER 在机载电脑发布 *_send_N；桥接到地面站后发布为 *_recv_N。
# 其余可视化话题原本就没有 send/recv 后缀。
def drone_source_topic(topic_template: str, drone_id: int) -> str:
    source = TELEMETRY_SOURCES.get(topic_template, topic_template)
    source = source.replace("_recv_{drone_id}", "_send_{drone_id}")
    return source.format(drone_id=drone_id)

# GCS 共享 send topic 端口
GCS_GOAL_PORT = 5099

# 无人机 send topic 起始端口
DRONE_SEND_START_PORT = 5001


def format_ip_section(drone_ips: List[str], gcs_ip: str, include_self: bool = True) -> str:
    """生成 IP 映射部分的 YAML 片段。"""
    lines = ["IP:"]
    if include_self:
        lines.append("  self: '*'   # '*' stands for all self IPs")
    for i, ip in enumerate(drone_ips, start=1):
        lines.append(f"  robot{i}: {ip}")
    lines.append(f"  gcs: {gcs_ip}")
    return "\n".join(lines)


def format_gcs_send_topics(num_drones: int) -> str:
    """生成 GCS send_topics YAML 片段。

    GCS 发出的 topic：
      - /move_base_simple/goal（共享，所有无人机接收）
      - /uav{N}/mission/land_request（每架无人机专有）
    """
    lines = []
    lines.append("####### Send these ROS messages to remote robots #######")
    lines.append("send_topics:")

    # 共享的 2D Nav Goal
    lines.append("  # 将地面站 RViz 产生的 \"2D Nav Goal\" 广播出去，作为集群起飞触发信号")
    lines.append("  - topic_name: /move_base_simple/goal")
    lines.append("    msg_type: geometry_msgs/PoseStamped")
    lines.append("    max_freq: 10")
    lines.append("    srcIP: gcs")
    lines.append(f"    srcPort: {GCS_GOAL_PORT}  # 为 Trigger 分配一个独立端口，避免冲突")

    # 每架无人机的 landing_request，端口从 GCS_GOAL_PORT-1 向下递减
    for i in range(1, num_drones + 1):
        port = GCS_GOAL_PORT - i
        lines.append("")
        lines.append(f"  # 任务级降落请求：由地面站前端发布 Bool(True)，桥接转发到 {i} 号无人机")
        lines.append(f"  - topic_name: /uav{i}/mission/land_request")
        lines.append("    msg_type: std_msgs/Bool")
        lines.append("    max_freq: 50")
        lines.append("    srcIP: gcs")
        lines.append(f"    srcPort: {port}  # send_topics 的端口必须互不相同")

    return "\n".join(lines)


def format_gcs_recv_topics(num_drones: int, start_port: int = DRONE_SEND_START_PORT) -> str:
    """生成 GCS recv_topics YAML 片段。

    GCS 从每架无人机接收探索话题和电量、位置话题。
    """
    lines = []
    lines.append("")
    lines.append("####### receive these ROS messages from remote robots #######")
    lines.append("recv_topics:")

    for drone_id in range(1, num_drones + 1):
        lines.append(f"  # ==========================================")
        lines.append(f"  # 接收来自 无人机 {drone_id} (Robot {drone_id}) 的所有数据")
        lines.append(f"  # ==========================================")
        for topic_idx, (topic_tpl, msg_type) in enumerate(DRONE_TOPICS):
            topic_name = topic_tpl.format(drone_id=drone_id)
            port = start_port + topic_idx
            lines.append("")
            lines.append(f"  - topic_name: {topic_name}")
            lines.append(f"    msg_type: {msg_type}")
            lines.append(f"    srcIP: robot{drone_id}")
            lines.append(f"    srcPort: {port}")
            if topic_idx < len(DRONE_TOPICS) - 1:
                lines.append("")
        lines.append("")

    return "\n".join(lines)


def format_drone_ip_section(drone_ips: List[str], gcs_ip: str) -> str:
    """生成无人机端 IP 映射部分。"""
    return format_ip_section(drone_ips, gcs_ip, include_self=True)


def format_drone_send_topics(drone_id: int, start_port: int = DRONE_SEND_START_PORT) -> str:
    """生成无人机端 send_topics YAML 片段。

    无人机将自己的探索话题和 MAVROS 遥测发往 GCS。
    """
    lines = []
    host_label = f"无人机 {drone_id} (Robot {drone_id})"
    lines.append(f"####### Send these ROS messages to GCS — {host_label} #######")
    lines.append("send_topics:")

    for topic_idx, (topic_tpl, msg_type) in enumerate(DRONE_TOPICS):
        topic_name = drone_source_topic(topic_tpl, drone_id)
        port = start_port + topic_idx
        lines.append(f"  - topic_name: {topic_name}")
        lines.append(f"    msg_type: {msg_type}")
        lines.append(f"    max_freq: {2 if topic_idx == 9 else 50}")
        lines.append("    srcIP: self")
        lines.append(f"    srcPort: {port}")
        if topic_idx < len(DRONE_TOPICS) - 1:
            lines.append("")

    return "\n".join(lines)


def format_drone_recv_topics(drone_id: int, num_drones: int,
                             start_port: int = DRONE_SEND_START_PORT) -> str:
    """生成无人机端 recv_topics YAML 片段。

    无人机接收：
      1. 来自 GCS 的指令（/move_base_simple/goal + /uav{N}/mission/land_request）
      2. 来自其他所有无人机的探索话题和遥测（带无人机编号）
    """
    lines = []
    lines.append("")
    lines.append("####### receive these ROS messages from GCS and other robots #######")
    lines.append("recv_topics:")

    # ── 来自 GCS 的指令 ──
    lines.append(f"  # ==========================================")
    lines.append(f"  # 接收来自 地面站 (GCS) 的指令")
    lines.append(f"  # ==========================================")
    lines.append("")
    lines.append(f"  # 2D Nav Goal — 起飞/集群触发信号")
    lines.append(f"  - topic_name: /move_base_simple/goal")
    lines.append(f"    msg_type: geometry_msgs/PoseStamped")
    lines.append(f"    srcIP: gcs")
    lines.append(f"    srcPort: {GCS_GOAL_PORT}")
    lines.append("")
    landing_port = GCS_GOAL_PORT - drone_id
    lines.append(f"  # 本机专属降落请求")
    lines.append(f"  - topic_name: /uav{drone_id}/mission/land_request")
    lines.append(f"    msg_type: std_msgs/Bool")
    lines.append(f"    srcIP: gcs")
    lines.append(f"    srcPort: {landing_port}")

    # ── 来自其他无人机的数据 ──
    other_drones = [d for d in range(1, num_drones + 1) if d != drone_id]
    for other_id in other_drones:
        lines.append("")
        lines.append(f"  # ==========================================")
        lines.append(f"  # 接收来自 无人机 {other_id} (Robot {other_id}) 的数据")
        lines.append(f"  # ==========================================")
        for topic_idx, (topic_tpl, msg_type) in enumerate(DRONE_TOPICS):
            topic_name = topic_tpl.format(drone_id=other_id)
            port = start_port + topic_idx
            lines.append("")
            lines.append(f"  - topic_name: {topic_name}")
            lines.append(f"    msg_type: {msg_type}")
            lines.append(f"    srcIP: robot{other_id}")
            lines.append(f"    srcPort: {port}")

    return "\n".join(lines)


def generate_gcs_config(drone_ips: List[str], gcs_ip: str,
                        start_port: int = DRONE_SEND_START_PORT) -> str:
    """生成完整的 ros_topics_gcs.yaml 内容。"""
    num_drones = len(drone_ips)

    header = "\n".join([
        "### Generated by scripts/generate_swarm_config.py",
        "# Message type support: include/ros_sub_pub.hpp",
        "",
        "### list all <IPname: IP> here ###",
    ])

    ip_section = format_ip_section(drone_ips, gcs_ip, include_self=True)
    send_topics = format_gcs_send_topics(num_drones)
    recv_topics = format_gcs_recv_topics(num_drones, start_port)

    return "\n".join([header, ip_section, "", send_topics, recv_topics]).rstrip() + "\n"


def generate_drone_config(drone_id: int, drone_ips: List[str], gcs_ip: str,
                          start_port: int = DRONE_SEND_START_PORT) -> str:
    """生成完整的 ros_topics_drone{N}.yaml 内容。"""
    num_drones = len(drone_ips)

    header = "\n".join([
        "### Generated by scripts/generate_swarm_config.py",
        "# Local telemetry: /mavros/*; received telemetry: /uavN/mavros/*",
        "",
        f"### 无人机 {drone_id} (Robot {drone_id}) 的桥接配置 ###",
        f"### 自动生成 — 请确认 IP 是否正确 ###",
    ])

    ip_section = format_drone_ip_section(drone_ips, gcs_ip)
    send_topics = format_drone_send_topics(drone_id, start_port)
    recv_topics = format_drone_recv_topics(drone_id, num_drones, start_port)

    return "\n".join([header, ip_section, "", send_topics, recv_topics, ""])


def generate_launch(drone_id: int) -> str:
    """生成 bridge_drone{N}.launch 内容。"""
    config_file = f"ros_topics_drone{drone_id}.yaml"
    node_name = f"swarm_bridge_node_drone{drone_id}"
    return "\n".join([
        '<?xml version="1.0"?>',
        '<launch>',
        '  <arg name="mavros_ns" default="/mavros" />',
        f'  <node pkg="swarm_ros_bridge" type="bridge_node" name="{node_name}" output="screen" >',
        f'    <rosparam command="load" file="$(find swarm_ros_bridge)/config/{config_file}" />',
        '    <remap from="/mavros/battery" to="$(arg mavros_ns)/battery" />',
        '    <remap from="/mavros/local_position/pose" to="$(arg mavros_ns)/local_position/pose" />',
        '  </node>',
        '</launch>',
        '',
    ])


def choose_launch_dir(output_dir: str, explicit_launch_dir: str = None) -> str:
    """在 ROS package 内生成配置时，launch 只写入 package/launch。"""
    if explicit_launch_dir:
        return explicit_launch_dir
    package_root = os.path.abspath(os.path.join(os.path.dirname(__file__), ".."))
    output_path = os.path.abspath(output_dir)
    if os.path.commonpath((package_root, output_path)) == package_root:
        return os.path.join(package_root, "launch")
    return output_dir


# ─── 交互式输入 ─────────────────────────────────────────────────────

def interactive_mode() -> Tuple[List[str], str, str, int]:
    """交互式询问用户输入无人机数量、IP 等信息。"""
    print("=" * 60)
    print("  Swarm ROS Bridge 配置文件生成器")
    print("=" * 60)
    print()

    # GCS IP
    while True:
        gcs_ip = input("地面站 IP 地址: ").strip()
        if gcs_ip:
            break
        print("  [错误] 地面站 IP 不能为空，请重新输入。")

    # 无人机数量
    while True:
        try:
            num_drones = int(input("无人机数量: ").strip())
            if num_drones < 1:
                print("  [错误] 无人机数量至少为 1。")
                continue
            break
        except ValueError:
            print("  [错误] 请输入有效的整数。")

    # 每架无人机的 IP
    drone_ips = []
    for i in range(1, num_drones + 1):
        while True:
            ip = input(f"无人机 {i} IP 地址: ").strip()
            if ip:
                drone_ips.append(ip)
                break
            print("  [错误] IP 不能为空，请重新输入。")

    # 输出目录
    while True:
        output_dir = input("输出目录 (默认 ./output): ").strip()
        if not output_dir:
            output_dir = "./output"
        if os.path.isdir(output_dir) or not os.path.exists(output_dir):
            break
        print(f"  [错误] '{output_dir}' 已存在且不是目录，请重新输入。")
    if not output_dir:
        output_dir = "./output"

    # 起始端口
    while True:
        port_str = input(f"无人机 recv 起始端口 (默认 {DRONE_SEND_START_PORT}): ").strip()
        if not port_str:
            start_port = DRONE_SEND_START_PORT
            break
        try:
            start_port = int(port_str)
            if start_port < 1024 or start_port > 65535:
                print("  [错误] 端口范围 1024-65535。")
                continue
            break
        except ValueError:
            print("  [错误] 请输入有效的整数。")

    return drone_ips, gcs_ip, output_dir, start_port


# ─── 主逻辑 ─────────────────────────────────────────────────────────

def main():
    parser = argparse.ArgumentParser(
        description="自动生成 swarm_ros_bridge 的 GCS 与无人机群配置文件",
        formatter_class=argparse.RawDescriptionHelpFormatter,
        epilog="示例:\n"
               "  python3 generate_swarm_config.py --gcs 192.168.0.137 \\\n"
               "      --drones 192.168.0.121 192.168.0.195 192.168.0.243 \\\n"
               "      --output-dir ../config/",
    )
    parser.add_argument("--gcs", type=str, help="地面站 IP 地址")
    parser.add_argument("--drones", type=str, nargs="+", help="无人机 IP 地址列表")
    parser.add_argument("--output-dir", type=str, default="./output",
                        help="输出目录 (默认 ./output)")
    parser.add_argument("--launch-dir", type=str,
                        help="launch 输出目录（包内默认 ./launch；包外默认与 --output-dir 相同）")
    parser.add_argument("--start-port", type=int, default=DRONE_SEND_START_PORT,
                        help=f"无人机发送 topic 起始端口 (默认 {DRONE_SEND_START_PORT})")

    args = parser.parse_args()

    # 判断是命令行模式还是交互模式
    if args.gcs is not None and args.drones is not None:
        # 命令行模式
        drone_ips = args.drones
        gcs_ip = args.gcs
        output_dir = args.output_dir
        start_port = args.start_port
    elif args.gcs is None and args.drones is None:
        # 交互模式
        drone_ips, gcs_ip, output_dir, start_port = interactive_mode()
    else:
        print("[错误] --gcs 和 --drones 必须同时提供（命令行模式），或都不提供（交互模式）。")
        sys.exit(1)

    num_drones = len(drone_ips)

    # 端口范围校验
    if start_port + NUM_DRONE_TOPICS - 1 > 65535:
        print(f"[错误] 起始端口 {start_port} + {NUM_DRONE_TOPICS} 个 topic 超出 65535 范围。")
        sys.exit(1)

    # 检查 GCS goal 端口是否与无人机端口冲突
    gcs_landing_min = GCS_GOAL_PORT - num_drones
    drone_port_range = range(start_port, start_port + NUM_DRONE_TOPICS)
    for port in range(gcs_landing_min, GCS_GOAL_PORT + 1):
        if port in drone_port_range:
            print(f"[警告] GCS send 端口 {port} 与无人机 send 端口范围 "
                  f"{start_port}-{start_port + NUM_DRONE_TOPICS - 1} 存在冲突，请调整 --start-port。")

    # 创建输出目录
    os.makedirs(output_dir, exist_ok=True)
    launch_dir = choose_launch_dir(output_dir, args.launch_dir)
    os.makedirs(launch_dir, exist_ok=True)

    print()
    print(f"无人机数量: {num_drones}")
    for i, ip in enumerate(drone_ips, start=1):
        print(f"  robot{i}: {ip}")
    print(f"地面站:     {gcs_ip}")
    print(f"输出目录:   {os.path.abspath(output_dir)}")
    print(f"无人机发送端口: {start_port}-{start_port + NUM_DRONE_TOPICS - 1}")
    print()

    # ── 生成 GCS 配置 ──
    gcs_content = generate_gcs_config(drone_ips, gcs_ip, start_port)
    gcs_path = os.path.join(output_dir, "ros_topics_gcs.yaml")
    with open(gcs_path, "w", encoding="utf-8") as f:
        f.write(gcs_content)
    print(f"✓ ros_topics_gcs.yaml  ({len(gcs_content)} bytes)")

    # ── 生成每架无人机配置 ──
    for drone_id in range(1, num_drones + 1):
        drone_content = generate_drone_config(drone_id, drone_ips, gcs_ip, start_port)
        drone_path = os.path.join(output_dir, f"ros_topics_drone{drone_id}.yaml")
        with open(drone_path, "w", encoding="utf-8") as f:
            f.write(drone_content)
        print(f"✓ ros_topics_drone{drone_id}.yaml  ({len(drone_content)} bytes)")

        launch_content = generate_launch(drone_id)
        launch_path = os.path.join(launch_dir, f"bridge_drone{drone_id}.launch")
        with open(launch_path, "w", encoding="utf-8") as f:
            f.write(launch_content)
        print(f"✓ bridge_drone{drone_id}.launch  ({len(launch_content)} bytes)")

    print()
    print(f"完成! YAML: {os.path.abspath(output_dir)}; launch: {os.path.abspath(launch_dir)}")

    # ── 统计摘要 ──
    print()
    print("─── 配置摘要 ───")
    print(f"GCS:")
    print(f"  send_topics:  1 (goal) + {num_drones} (landing) = {1 + num_drones} 个")
    print(f"  recv_topics:  {NUM_DRONE_TOPICS} × {num_drones} = {NUM_DRONE_TOPICS * num_drones} 个")
    print(f"各无人机:")
    gcs_recv_for_drone = 2  # goal + landing
    other_drone_recv = NUM_DRONE_TOPICS * (num_drones - 1) if num_drones > 1 else 0
    total_drone_recv = gcs_recv_for_drone + other_drone_recv
    print(f"  send_topics:  {NUM_DRONE_TOPICS} 个")
    print(f"  recv_topics:  {gcs_recv_for_drone} (from GCS) + {other_drone_recv} (from other drones) = {total_drone_recv} 个")


if __name__ == "__main__":
    main()
