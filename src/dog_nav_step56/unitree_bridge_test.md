# Unitree Bridge 集成测试文档

本文档记录 unitree_bridge_node 从环境搭建到真机测试的完整流程。

## 1. 环境准备

### 1.1 前置条件
- Docker 容器：`osrf/ros:humble-desktop-full` (或同等 Ubuntu 22.04 + ROS 2 Humble 环境)
- 项目代码已位于 `/root/ws/src/`

### 1.2 一键安装脚本
在容器内执行：
```bash
cd /root/ws
bash scripts/setup_unitree_env.sh
```

此脚本会自动完成：
- 安装 `libyaml-cpp-dev`、`ros-humble-rmw-cyclonedds-cpp`、`ros-humble-rosidl-generator-dds-idl`
- 拉取 `unitreerobotics/unitree_ros2` 仓库
- 编译 `cyclonedds_ws`（`unitree_api` + `unitree_go`）
- 编译 `dog_nav_interfaces` + `dog_nav_step56`

### 1.3 手动安装 (备选)
```bash
sudo apt update
sudo apt install -y libyaml-cpp-dev ros-humble-rmw-cyclonedds-cpp ros-humble-rosidl-generator-dds-idl

cd /root/ws/src
git clone https://github.com/unitreerobotics/unitree_ros2.git

cd /root/ws/src/unitree_ros2/cyclonedds_ws
source /opt/ros/humble/setup.bash
colcon build --packages-select unitree_api unitree_go

cd /root/ws
source /opt/ros/humble/setup.bash
source /root/ws/src/unitree_ros2/cyclonedds_ws/install/setup.bash
colcon build --packages-select dog_nav_interfaces dog_nav_step56
```

## 2. Go2 网络配置

### 2.1 物理连接
- 用网线连接电脑和 Go2
- `ifconfig` 查看网口名（如 `enp3s0`）
- 设置电脑 IP 为 `192.168.123.99`，掩码 `255.255.255.0`
- Go2 默认 IP：`192.168.123.161`

### 2.2 环境变量
```bash
export RMW_IMPLEMENTATION=rmw_cyclonedds_cpp
export CYCLONEDDS_URI='<CycloneDDS><Domain><General><Interfaces>
    <NetworkInterface name="enp3s0" priority="default" multicast="default" />
</Interfaces></General></Domain></CycloneDDS>'
```

> 将 `enp3s0` 替换为实际网口名。

### 2.3 验证连接
```bash
source /opt/ros/humble/setup.bash
source /root/ws/src/unitree_ros2/cyclonedds_ws/install/setup.bash
export RMW_IMPLEMENTATION=rmw_cyclonedds_cpp
export CYCLONEDDS_URI='<CycloneDDS><Domain><General><Interfaces>
    <NetworkInterface name="enp3s0" priority="default" multicast="default" />
</Interfaces></General></Domain></CycloneDDS>'

ros2 topic list  # 应看到 /lf/sportmodestate, /lf/lowstate 等话题
```

## 3. 模拟场景说明

`simulated_input_node` 内置 3 个场景，通过 `scenario` 参数切换，覆盖不同安全转换和步态切换。

### 3.1 场景列表

| 场景名 | 路径形态 | 地形/倾斜变化 | 测试目标 |
|:--|:--|:--|:--|
| `curved_normal` | 蛇形曲线 | 始终安全(terrain=20) | TROT 步态巡航，wz 周期性变化 |
| `terrain_escalation` | 直线 | 0-4s=20→4-8s=150→8-12s=200→12s+=240 | NORMAL→SLOW→PAUSE→STOP 四级降级 + Damp 急停 |
| `zigzag_with_tilt` | Z字折返 | 5-7s 倾斜 10度→触发 SLOW | 大角度转弯(CRAWL) + 倾斜事件 + 自动恢复 |

### 3.2 场景 1: curved_normal -- 蛇形巡航

```bash
ros2 launch dog_nav_step56 closed_loop_demo.launch.py use_bridge:=true scenario:=curved_normal
```

| 时间 | 预期安全 | 预期步态 | 预期 bridge API |
|:--|:--|:--|:--|
| 0~∞ | NORMAL | TROT | `Move(1008)` 持续，wz 周期性正负变化，`SpeedLevel(1015):level=4` |

### 3.3 场景 2: terrain_escalation -- 地形恶化链

```bash
ros2 launch dog_nav_step56 closed_loop_demo.launch.py use_bridge:=true scenario:=terrain_escalation
```

| 时间 | 地形代价 | 安全 | 步态 | 预期 bridge 输出 |
|:--|:--|:--|:--|:--|
| 0~4s | 20 | NORMAL | TROT | `Move(1008)` 高速前进, `TrotRun(1062)`, `SpeedLevel(4)` |
| 4~8s | 150 | **SLOW** | **WALK** | `Move(1008)` 速度x0.5, `GaitSwitch:WALK→1061`, `SpeedLevel(2)` |
| 8~12s | 200 | **PAUSE** | **STAND** | `→ StopMove(1003)` |
| 12s+ | 240 | **STOP** | STAND | `→ Damp(1001)` |

### 3.4 场景 3: zigzag_with_tilt -- Z字巡逻 + 倾斜

```bash
ros2 launch dog_nav_step56 closed_loop_demo.launch.py use_bridge:=true scenario:=zigzag_with_tilt
```

| 时间 | 事件 | 安全 | 步态 | 预期 bridge 输出 |
|:--|:--|:--|:--|:--|
| 0~5s | Z字路径大角度转向 | NORMAL | **CRAWL** | `Move(1008)` + `GaitSwitch:CRAWL→1063`, wz 大 |
| 5~7s | **倾斜 10度** | **SLOW** | WALK | `Move` 速度x0.5, `GaitSwitch:WALK→1061` |
| 7s+ | 倾斜消失 | NORMAL | TROT | `GaitSwitch:TROT→1062`, 恢复全速 |

## 4. 启动测试

### 4.1 仿真模式（无真机）
```bash
source /root/ws/install/setup.bash
ros2 launch dog_nav_step56 closed_loop_demo.launch.py scenario:=curved_normal
# bridge 默认不启动
```

### 4.2 仿真 + bridge (bridge 节点启动但无真机连接)
```bash
source /root/ws/install/setup.bash
source /root/ws/src/unitree_ros2/cyclonedds_ws/install/setup.bash
ros2 launch dog_nav_step56 closed_loop_demo.launch.py use_bridge:=true scenario:=terrain_escalation
```

### 4.3 真机模式（需先配好网络）
```bash
source /root/ws/install/setup.bash
source /root/ws/src/unitree_ros2/cyclonedds_ws/install/setup.bash
export RMW_IMPLEMENTATION=rmw_cyclonedds_cpp
export CYCLONEDDS_URI='<CycloneDDS><Domain><General><Interfaces>
    <NetworkInterface name="enp3s0" priority="default" multicast="default" />
</Interfaces></General></Domain></CycloneDDS>'

ros2 launch dog_nav_step56 closed_loop_demo.launch.py use_bridge:=true scenario:=curved_normal
```

### 4.4 仅启动 bridge（配合上游步骤1-4）
```bash
ros2 launch dog_nav_step56 unitree_bridge.launch.py network_iface:=enp3s0
```

## 5. Bridge 映射行为速查

| 条件 | 预期 API 调用 | 日志输出 |
|:------|-------------|---------|
| DogMotion safety_state=STOP | Damp(1001) | `→ Damp(1001): safety state STOP/MANUAL` |
| DogMotion safety_state=PAUSE | StopMove(1003) | `→ StopMove(1003)` |
| DogMotion safety_state=NORMAL, vx=0.3 | Move(1008) | `→ Move(1008): vx=0.300, vy=0.000, wz=0.000` |
| DogMotion gait 从 WALK 变为 TROT | TrotRun(1062) | `→ GaitSwitch: TROT → api_id=1062` |
| DogMotion speed_limit=0.35 | SpeedLevel(1015, level=2) | `→ SpeedLevel(1015): level=2` |

## 6. 话题验证

```bash
# 查看 DogMotion 输出
ros2 topic echo /motion_command

# 查看 bridge 发出的请求
ros2 topic echo /api/sport/request

# 查看 Go2 状态（真机模式下）
ros2 topic echo /lf/sportmodestate
```

## 7. 故障排查

| 问题 | 原因 | 解决 |
|:------|------|------|
| `ModuleNotFoundError: unitree_api` | cyclonedds_ws 未编译或未 source | source cyclonedds_ws/install/setup.bash |
| bridge 节点无输出 | motion_adapter 未运行 | 确认 `/motion_command` 话题有数据 |
| Go2 不动 | 网络未配置 / API 请求未到达 | 检查 `ros2 topic echo /api/sport/request` 有无数据 |
| `ros2 topic list` 看不到 Go2 话题 | CycloneDDS 网口不对 | 检查 `CYCLONEDDS_URI` 中网口名是否正确 |
| 步态切换过于频繁 | gait 值抖动 | bridge 已内置去抖逻辑，确认参数正常 |
| 场景名无效 | `scenario` 参数拼写错误 | 可用: `curved_normal`, `terrain_escalation`, `zigzag_with_tilt` |

## 8. 文件清单

```
src/dog_nav_step56/
├── config/unitree_bridge.yaml            # bridge 参数配置
├── launch/unitree_bridge.launch.py       # bridge 独立启动
├── dog_nav_step56/unitree_bridge_node.py # bridge 核心节点
├── dog_nav_step56/scenarios.py           # 模拟场景库（3个场景）
├── dog_nav_step56/simulated_input_node.py# [修改] 场景驱动
├── setup.py                              # [修改] 注册入口点
├── package.xml                           # [修改] 添加 unitree_api 依赖
└── unitree_bridge_test.md                # 本文档

scripts/
└── setup_unitree_env.sh                  # 一键环境安装
```
