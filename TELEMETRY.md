# fly 电量与位置显示

地面站与机载电脑可以使用同一份源码，分别启动各自的 launch 文件。
本配置按 `uav_front_end/mission_orchestrator.py` 中的设备编号对应：

| 设备 | IP | 启动文件 |
| --- | --- | --- |
| 地面站 | 192.168.0.137 | bridge_node_gcs.launch |
| nuc / robot1 | 192.168.0.121 | bridge_drone1.launch |
| fly / robot2 | 192.168.0.195 | bridge_drone2.launch |
| nuc2 / robot3 | 192.168.0.243 | bridge_drone3.launch |

## 遥测通道

| fly 本地源话题 | 消息类型 | TCP 端口 | 地面站发布话题 |
| --- | --- | --- | --- |
| /mavros/battery | sensor_msgs/BatteryState | 5013 | /uav2/mavros/battery |
| /mavros/local_position/pose | geometry_msgs/PoseStamped | 5014 | /uav2/mavros/local_position/pose |

发送和接收通过 IP、端口及消息类型配对，话题名称可以不同。
RACER 前 8 个探索话题在机载电脑使用 `*_send_2`，地面站发布为 `*_recv_2`；
沿用 5001—5008 原有端口，点云 5010 限频 2 Hz。
机载端使用 `/mavros`，是根据前端的 `roslaunch mavros px4.launch` 启动命令以及控制代码中的话题名称配置的；部署时仍需检查实机发布者。
地面站为每架飞机添加 `/uavN` 前缀，避免三架飞机互相覆盖。
机载电脑接收其他飞机的数据时也使用带编号的名称，不会把收到的电量或位置重新当作本机数据转发。

## 更新和编译

需要把本次修改的 `swarm_ros_bridge` 同时更新到地面站和 fly 的现有仓库，路径通常是
`~/swarm_ros_bridge_ws/src/swarm_ros_bridge`。地面站还需要更新 `uav_front_end` 中的
`ros_adapter.py`、`ui_app.py` 和 `run_ui.sh`。

仅切换分支或更新地面站不能改变 fly 正在运行的配置和二进制程序。
若旧仓库仍有 `Unmerged paths`，先保留并解决现有修改，再合并更新；不要强制重置。

地面站和 fly 各自编译（RACER 工作空间需已编译，包含探索自定义消息的头文件）：

```bash
source /opt/ros/noetic/setup.bash
source ~/racer/devel/setup.bash
cd ~/swarm_ros_bridge_ws
catkin_make -DRACER_WORKSPACE="$HOME/racer"
source devel/setup.bash
rospack find swarm_ros_bridge
```

`RACER_WORKSPACE` 默认取当前用户的 `~/racer`，不再固定为地面站用户目录。
如果实际工作空间在别处，传入对应绝对路径。

## 启动与检查

在 MAVROS 已启动的 fly 终端查看源消息：

```bash
source /opt/ros/noetic/setup.bash
source ~/swarm_ros_bridge_ws/devel/setup.bash
rostopic list | grep -E '/mavros/(battery|local_position/pose)$'
timeout 10s rostopic echo -n 1 /mavros/battery
timeout 10s rostopic echo -n 1 /mavros/local_position/pose
```

如果实机话题是 `/uav2/mavros/...`，启动时覆盖命名空间：

```bash
roslaunch swarm_ros_bridge bridge_drone2.launch mavros_ns:=/uav2/mavros
```

如果实际为默认 `/mavros/...`，使用：

```bash
roslaunch swarm_ros_bridge bridge_drone2.launch
```

更新配置后，停止旧 bridge 进程，再启动新 bridge；同一台电脑只运行一份相同配置的 bridge。
也可通过 UI 的基础链路按钮启动 fly，但若已有旧 tmux 会话，需先在对应 bridge 窗口重启进程。
本次遥测显示只需 MAVROS、两端 bridge 和 UI，无需点击探索或启动飞行控制阶段。

地面站先启动 UI（脚本会检查本地 ROS master）：

```bash
cd ~/uav_front_end
bash run_ui.sh
```

在 UI 点击启动地面站桥接的按钮，或另开地面站终端：

```bash
source /opt/ros/noetic/setup.bash
source ~/swarm_ros_bridge_ws/devel/setup.bash
roslaunch swarm_ros_bridge bridge_node_gcs.launch
```

在地面站另一个终端验证实际收到消息：

```bash
source /opt/ros/noetic/setup.bash
timeout 10s rostopic echo -n 1 /uav2/mavros/battery
timeout 10s rostopic echo -n 1 /uav2/mavros/local_position/pose
```

UI 日志会分别出现“已收到 2 号机电量”和“已收到 2 号机位置”，fly 卡片更新为电量、电压和米制坐标。
百分比未知时显示 `--`，仍显示有效电压；位置来自 MAVROS 的本地位置坐标，不是经纬度。
界面显示的是最近一次有效消息，不表示持续在线。

## 定位剩余问题

- fly 源话题没有消息：检查 MAVROS/飞控是否在发布；bridge 不能凭空产生电量或位置。
- fly 有消息、地面站没有：核对运行中的配置、IP、TCP 5013/5014、两端消息类型支持和实际启动的工作空间。
- 地面站 `rostopic echo` 有消息、UI 没显示：对照 UI 日志中的 ROS master，确保 UI 与地面站 bridge 使用同一个本地 master 和可达的 ROS 地址；检查 UI 是否报告 ROS 导入错误。
- 日志报 `Invalid ROS msg_type "sensor_msgs/BatteryState"`：对应设备仍在运行旧 bridge 二进制，重新编译并重启。

## 重新生成配置

在仓库根目录运行：

```bash
python3 scripts/generate_swarm_config.py \
  --gcs 192.168.0.137 \
  --drones 192.168.0.121 192.168.0.195 192.168.0.243 \
  --output-dir config --launch-dir launch
```

`config/` 和 `launch/` 是运行时目录；`scripts/temp/` 只是生成结果快照。
修改 IP 后需同步部署到两端。

离线验证配置配对（需要 PyYAML）：

```bash
python3 -m unittest discover -s tests -v
```

`uav_front_end/tests/test_telemetry_ui.py` 使用模拟 ROS 回调和真实 Qt 控件检查显示；
这些测试不替代两端实机 ROS/ZeroMQ 收发及 catkin 编译检查。
