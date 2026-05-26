# 宇树 Go2 Bridge 集成项目 —— 总结与真机对接指南

> 状态：**仿真验证通过，真机联调成功 ✅**
> 最后更新：2026-05-24

---

## 一、项目目标

为机器狗 2.5D 地形自主导航项目（步骤 5-6）新增 **Go2 真机控制能力**：导航系统输出的 `DogMotion` 高层面动作指令，经由 `unitree_bridge_node` 翻译为宇树 Sport API 调用，控制 Go2 真机执行。

### 设计原则

```
导航逻辑层 ←→ 动作语义层 ←→ 硬件适配层
(不改)      (不改)        (本次新增，可替换)
```

- `motion_adapter_node` —— 只管生成 `DogMotion`（速度/步态/高度/速度档位）
- `unitree_bridge_node` —— 只管翻译 `DogMotion` → 宇树 Sport API
- **解耦**：换 Go1 / B2 / 其他机器狗，只需换 bridge 节点

---

## 二、系统架构

### 2.1 全链路数据流

```
                              导航系统（步骤1-4，上游组员）
                                      │
                           ┌──────────┼──────────┐
                           │          │          │
                      /global_path  /odometry   /imu/data
                           │          │          │
                    ┌──────▼──────────▼──────────▼───────┐
                    │         步骤 5-6（我们的系统）         │
                    │                                    │
                    │  simulated_input                    │
                    │       │                            │
                    │  local_controller ──→ /cmd_vel     │
                    │       │                            │
                    │  safety_supervisor ─→ /safety_state│
                    │       │                            │
                    │  motion_adapter ───→ /motion_command│
                    │                           │        │
                    └───────────────────────────┼────────┘
                                                │
                                    ┌───────────▼───────────┐
                                    │  unitree_bridge_node   │
                                    │                        │
                                    │  DogMotion → Sport API │
                                    │        │               │
                                    │   /api/sport/request   │
                                    └───────────┬───────────┘
                                                │
                                          CycloneDDS
                                                │
                                    ┌───────────▼───────────┐
                                    │     Go2 真机            │
                                    │  (192.168.123.161)     │
                                    └────────────────────────┘
```

### 2.2 DogMotion → Sport API 映射关系

| DogMotion 字段 | 触发条件 | Unitree API | API ID | 说明 |
|:--|:--|:--|:--:|:--|
| `safety_state = STOP/MANUAL` | 紧急停止 | `Damp` | 1001 | 阻尼模式 |
| `safety_state = PAUSE` | 暂停 | `StopMove` | 1003 | 停止移动 |
| `safety_state = NORMAL/DEGRADED/SLOW` | 允许运动 | `Move` | 1008 | 10Hz 持续发送 `{"x":vx,"y":vy,"z":wz}` |
| `gait = WALK` (变化时) | 慢速行走 | `StaticWalk` | 1061 | 仅变化时发送一次 |
| `gait = TROT` (变化时) | 小跑 | `TrotRun` | 1062 | 仅变化时发送一次 |
| `gait = CRAWL` (变化时) | 复杂地形 | `EconomicGait` | 1063 | 仅变化时发送一次 |
| `speed_limit` (变化时) | 速度档位 | `SpeedLevel` | 1015 | ≤0.20→1, ≤0.35→2, ≤0.60→3, >0.60→4 |

### 2.3 安全阈值速查

| 安全等级 | 触发条件 | speed_scale | 步态 |
|:--|:--|:--|:--|
| NORMAL | 默认 | 1.0 | TROT |
| SLOW | tilt≥8° / terrain≥130 / cov≥0.25 / linear≥0.6 / angular≥0.8 | 0.5 | WALK |
| PAUSE | tilt≥14° / terrain≥180 / cov≥0.45 | 0.0 | STAND |
| STOP | tilt≥20° / terrain≥220 / cov≥0.8 | 0.0 | STAND+Damp |

---

## 三、已完成的交付清单

### 3.1 第一批交付 (2026-05-20) —— Bridge 核心

**新增文件：**
```
src/dog_nav_step56/
├── dog_nav_step56/unitree_bridge_node.py   # 核心桥接节点
├── config/unitree_bridge.yaml              # 参数配置
├── launch/unitree_bridge.launch.py         # 独立启动文件
└── unitree_bridge_test.md                  # 集成测试文档

scripts/
└── setup_unitree_env.sh                    # 一键环境安装
```

**修改文件：**
| 文件 | 改动 |
|:--|:--|
| `setup.py` | 注册 `unitree_bridge_node` 入口点 |
| `package.xml` | 新增 `<exec_depend>unitree_api</exec_depend>` |
| `launch/closed_loop_demo.launch.py` | 新增 `use_bridge:=true` 参数 |

### 3.2 第二批交付 (2026-05-24) —— 多场景测试

**新增文件：**
```
src/dog_nav_step56/dog_nav_step56/
└── scenarios.py                  # 3个模拟场景：curved_normal / terrain_escalation / zigzag_with_tilt

scripts/
└── run_scenario_tests.sh         # 自动测试脚本（依次跑3个场景并汇总）
```

**修改文件：**
| 文件 | 改动 |
|:--|:--|
| `simulated_input_node.py` | 废弃 `simulate_tilt`/`terrain_cost` 参数，统一为 `scenario` 参数，委托给 `scenarios.py` |
| `launch/closed_loop_demo.launch.py` | 新增 `scenario` 参数 |

### 3.3 三个模拟场景

| # | 场景名 | 路径形态 | 地形变化 | 测试目标 |
|:--:|:--|:--|:--|:--|
| 0 | `test_forward` | 纯直线 | 始终安全 terrain=20 | 最简真机验证——直线前行 |
| 1 | `curved_normal` | 蛇形曲线 | 始终安全 terrain=20 | TROT巡航、wz周期性变化 |
| 2 | `terrain_escalation` | 直线 | 0-4s=20→4-8s=150→8-12s=200→12s+=240 | NORMAL→SLOW→PAUSE→STOP 四级降级 + Damp(1001) |
| 3 | `zigzag_with_tilt` | Z字折返 | 5-7s 倾斜 10° | 大角度转弯CRAWL + 倾斜触发SLOW + 自动恢复 |

### 3.4 仿真验证结果 ✅

**场景 1 (curved_normal)** —— 容器内运行，bridge 输出：
```
→ Damp(1001)                          # 初始安全态
→ Move(1008): vx=0.381, wz=0.455
→ GaitSwitch: CRAWL → api_id=1063
→ SpeedLevel(1015): level=2
→ GaitSwitch: TROT → api_id=1062      # 加速→小跑
→ SpeedLevel(1015): level=4
→ Move(1008): vx=0.388, wz=0.383
→ StopMove(1003)                      # 安全波动(PAUSE)
→ Move(1008): vx=0.426, wz=0.181     # 恢复
→ GaitSwitch: WALK → api_id=1061      # 降步态
→ SpeedLevel(1015): level=2
→ Move(1008): vx=0, wz=-1.170         # 原地大角度转向
```

**场景 2 (terrain_escalation)** —— 安全降级链：
```
→ Move(1008): TROT SpeedLevel:4       # NORMAL
→ GaitSwitch: WALK SpeedLevel:2       # SLOW(terrain=150)
→ StopMove(1003)                      # PAUSE(terrain=200)
→ Damp(1001)                          # STOP(terrain=240)
```

**覆盖总结：** 全部 6 种 Sport API (Move/Damp/StopMove/StaticWalk/TrotRun/EconomicGait) 和全部 4 级安全降级均已通过仿真验证。

---

## 四、真机对接 —— 完整操作步骤

> ⚠️ **安全第一：测试过程遥控器始终在手，随时可拍急停。**
>
> ✅ **2026-05-24 真机联调成功**（VMware + 网线直连，test_forward 场景 Go2 正常行走）

### 运行环境：VMware 虚拟机（推荐）

Docker 的 UDP 多播在 WSL2 下不可靠。推荐 VMware Ubuntu 22.04：
- 网卡1：NAT（上网用）
- 网卡2：桥接到有线以太网卡（DDS 通信用，手动设静态 IP `192.168.123.x`）

### 4.0 前置：进入 VM 并 source 环境

```bash
cd ~/Daohang
source /opt/ros/humble/setup.bash
source src/unitree_ros2/cyclonedds_ws/install/setup.bash
source install/setup.bash

# 每次开机后重设网卡 IP
sudo ip addr add 192.168.123.100/24 dev ens38
```

### 4.1 物理连接

```bash
# 1. Go2 开机，遥控器放旁边
# 2. 网线连接电脑与 Go2 背部以太网口
# 3. 查看网口名
ifconfig

# 4. 设置电脑静态IP（假设网口名 enp3s0）
sudo ifconfig enp3s0 192.168.123.99 netmask 255.255.255.0
```

> Go2 默认 IP：`192.168.123.161`

### 4.2 通信验证

```bash
# 设置 CycloneDDS 网络（将 enp3s0 替换为实际网口名）
export RMW_IMPLEMENTATION=rmw_cyclonedds_cpp
export CYCLONEDDS_URI='<CycloneDDS><Domain><General><Interfaces>
    <NetworkInterface name="enp3s0" priority="default" multicast="default" />
</Interfaces></General></Domain></CycloneDDS>'

# 查看 Go2 话题
ros2 topic list | grep -E "lf|sport|lowstate|bms"
```

**预期看到：** `/lf/sportmodestate`、`/lf/lowstate`、`/lf/bmsstate` 等。

```bash
# 查看 Go2 当前状态
ros2 topic echo /lf/sportmodestate --once
```

### 4.3 安全性逐级启动（务必按顺序）

| 步骤 | 命令 | 目的 | Go2 会动吗 |
|:--:|:--|:--|:--:|
| **1** | `ros2 topic echo /motion_command` | 先看导航输出，不发指令给 Go2 | ❌ 不会 |
| **2** | `ros2 launch dog_nav_step56 unitree_bridge.launch.py` | 仅 bridge，看 API 是否正常发出 | ❌ 不会 |
| **3** | `ros2 launch dog_nav_step56 closed_loop_demo.launch.py use_bridge:=true scenario:=curved_normal` | 全链路，Go2 开始动 | ✅ 会 |

**每步确认日志无异常再进下一步。任何异常立刻 Ctrl+C 或按遥控器急停。**

### 4.4 三个场景的真机测试

#### 场景 1：蛇形巡航（先跑这个，最安全）

```bash
ros2 launch dog_nav_step56 closed_loop_demo.launch.py use_bridge:=true scenario:=curved_normal
```

**Go2 行为：** 小跑（TROT），沿蛇形路径前进约 3 米，持续右转，步态在 CRAWL/TROT/WALK 间自动切换。**全程安全地形，不会急停。**

#### 场景 2：地形恶化链（验证安全降级）

```bash
ros2 launch dog_nav_step56 closed_loop_demo.launch.py use_bridge:=true scenario:=terrain_escalation
```

**Go2 行为：**
- 0~4s：TROT 高速前进
- 4~8s：WALK 慢速行走
- 8~12s：停止移动（StopMove）
- 12s+：进入阻尼模式（Damp）

> ⚠️ 此场景 Go2 会在 12 秒后 Damp。如果遥控器不在手边，建议先不跑这个。

#### 场景 3：Z字巡逻 + 倾斜事件

```bash
ros2 launch dog_nav_step56 closed_loop_demo.launch.py use_bridge:=true scenario:=zigzag_with_tilt
```

**Go2 行为：**
- 0~5s：Z字路径大角度转向，CRAWL 步态
- 5~7s：模拟倾斜 → SLOW → WALK 步态
- 7s+：倾斜消失 → 恢复 TROT

### 4.5 真机同时查看里话题

另开一个终端：

```bash
# 查看 bridge 发出的 API 请求
ros2 topic echo /api/sport/request

# 查看 Go2 返回的状态
ros2 topic echo /lf/sportmodestate

# 查看导航系统输出的运动指令
ros2 topic echo /motion_command
```

### 4.6 故障排查

| 现象 | 检查项 |
|:--|:--|
| `ros2 topic list` 看不到 Go2 话题 | ① 网线是否插紧 ② IP 是否 192.168.123.x 网段 ③ `CYCLONEDDS_URI` 中网口名是否正确 |
| 能看到话题但 Go2 不动 | ① `ros2 topic echo /api/sport/request` 有无数据 ② Go2 是否在 Sport 模式 |
| 步态切换异常 | 查看 `GaitSwitch` 日志 |
| `ModuleNotFoundError: unitree_api` | 未 source：`source /root/ws/src/unitree_ros2/cyclonedds_ws/install/setup.bash` |

---

## 五、如何修改路径/创建自定义场景

### 5.1 场景文件位置

```
/root/ws/src/dog_nav_step56/dog_nav_step56/scenarios.py
```

### 5.2 场景结构说明

每个场景是一个类，继承自 `_BaseScenario`，通过 `@register('场景名')` 注册。需要定义的内容：

```python
@register('my_scenario')           # 场景名，launch 时用 scenario:=my_scenario
class MyScenario(_BaseScenario):

    # ---- 路径参数 ----
    path_length = 12                # 路点数量
    path_spacing = 0.5              # 路点间距 (m)

    # ---- 里程计（模拟机器人运动） ----
    speed = 0.25                    # 前进速度因子 (m/s per second)
    lateral_amp = 0.15              # 横向摆动幅度 (m)
    lateral_freq = 0.6              # 横向摆动频率 (rad/s)
    yaw_amp = 0.12                  # 航向摆动幅度 (rad)
    yaw_freq = 0.4                  # 航向摆动频率 (rad/s)

    # ---- 地形代价时间线 (start_sec, terrain_value) ----
    terrain_timeline = [
        (0.0, 20.0),               # 0s起: terrain=20 (安全)
        (7.0, 150.0),              # 7s起: terrain=150 (触发SLOW)
    ]

    # ---- 倾斜时间线 (start_sec, end_sec, tilt_rad) ----
    tilt_timeline = [
        (5.0, 7.0, 0.175),         # 5~7s: 倾斜10° (触发SLOW)
    ]

    # 自定义路径形状
    def build_path(self, stamp):
        # 返回 nav_msgs/Path 消息
        ...
```

### 5.3 修改路径的三种方式

**方式一：改现有场景的参数（最快）**

```python
# 把 curved_normal 的路径间距改大 → 路更长
@register('curved_normal')
class CurvedNormalScenario(_BaseScenario):
    path_spacing = 1.0   # 原来是 0.5，现在路点间距翻倍
    speed = 0.4           # 原来是 0.25，机器人走更快
```

**方式二：改路径形状**

覆盖 `build_path()` 方法：

```python
def build_path(self, stamp):
    path = NavPath()
    path.header.frame_id = 'map'
    path.header.stamp = stamp
    for i in range(self.path_length):
        pose = PoseStamped()
        pose.header.frame_id = 'map'
        pose.header.stamp = stamp
        # 圆形路径
        angle = i * 2 * math.pi / self.path_length
        pose.pose.position.x = 2.0 * math.cos(angle)
        pose.pose.position.y = 2.0 * math.sin(angle)
        pose.pose.position.z = 0.0
        pose.pose.orientation = _yaw_to_quat(0.0)
        path.poses.append(pose)
    return path
```

**方式三：从文件读取真实路径**

```python
def build_path(self, stamp):
    # 从 CSV 或 JSON 文件读取预规划路径点
    import json
    with open('/root/ws/my_path.json') as f:
        waypoints = json.load(f)
    path = NavPath()
    for wp in waypoints:
        pose = PoseStamped()
        pose.pose.position.x = wp['x']
        pose.pose.position.y = wp['y']
        path.poses.append(pose)
    return path
```

### 5.4 修改后如何生效

```bash
# 1. 编辑 scenarios.py 保存
# 2. 重新编译
cd /root/ws
source /opt/ros/humble/setup.bash
source /root/ws/src/unitree_ros2/cyclonedds_ws/install/setup.bash
colcon build --packages-select dog_nav_step56

# 3. source 新环境
source /root/ws/install/setup.bash

# 4. 用新的场景名启动
ros2 launch dog_nav_step56 closed_loop_demo.launch.py use_bridge:=true scenario:=my_scenario
```

---

## 六、快速命令参考

### 6.1 环境 source（每次进容器必做）

```bash
source /opt/ros/humble/setup.bash
source /root/ws/src/unitree_ros2/cyclonedds_ws/install/setup.bash
source /root/ws/install/setup.bash
```

### 6.2 真机模式一步启动

```bash
# 1. 设网卡 IP（每次开机都要做）
sudo ip addr add 192.168.123.100/24 dev ens38

# 2. 设置 CycloneDDS
export RMW_IMPLEMENTATION=rmw_cyclonedds_cpp
export CYCLONEDDS_URI='<CycloneDDS><Domain><General><Interfaces><NetworkInterface address="192.168.123.100"/></Interfaces></General></Domain></CycloneDDS>'

# 3. 启动（四选一）
ros2 launch dog_nav_step56 closed_loop_demo.launch.py use_bridge:=true scenario:=test_forward
ros2 launch dog_nav_step56 closed_loop_demo.launch.py use_bridge:=true scenario:=curved_normal
ros2 launch dog_nav_step56 closed_loop_demo.launch.py use_bridge:=true scenario:=terrain_escalation
ros2 launch dog_nav_step56 closed_loop_demo.launch.py use_bridge:=true scenario:=zigzag_with_tilt
```

### 6.3 仅 bridge（配合上游步骤 1-4）

```bash
ros2 launch dog_nav_step56 unitree_bridge.launch.py network_iface:=enp3s0
```

### 6.4 自动跑所有仿真测试（不连真机）

```bash
bash /root/ws/scripts/run_scenario_tests.sh
```

---

## 七、文件结构全景

```
/root/ws/
├── src/
│   ├── dog_nav_interfaces/msg/
│   │   ├── DogMotion.msg              # vx,vy,wz,gait,body_height,duration,speed_limit,safety_state
│   │   └── SafetyState.msg            # level,roll,pitch,terrain_cost,covariance,emergency_stop
│   │
│   ├── dog_nav_step56/
│   │   ├── dog_nav_step56/
│   │   │   ├── state_machine.py       # 安全状态机 + 步态选择
│   │   │   ├── scenarios.py           # ★ 3个模拟场景（5/24新增）
│   │   │   ├── simulated_input_node.py # [改] 按场景生成模拟数据
│   │   │   ├── local_controller_node.py
│   │   │   ├── safety_supervisor_node.py
│   │   │   ├── motion_adapter_node.py
│   │   │   └── unitree_bridge_node.py  # ★ Go2 SDK 桥接
│   │   ├── config/
│   │   │   ├── unitree_bridge.yaml
│   │   │   ├── motion_adapter.yaml
│   │   │   └── safety_supervisor.yaml
│   │   ├── launch/
│   │   │   ├── unitree_bridge.launch.py
│   │   │   ├── closed_loop_demo.launch.py # [改] +scenario参数
│   │   │   ├── step56.launch.py
│   │   │   └── step56_test.launch.py
│   │   ├── setup.py                   # [改] +unitree_bridge入口
│   │   ├── package.xml                # [改] +unitree_api依赖
│   │   └── unitree_bridge_test.md
│   │
│   └── unitree_ros2/cyclonedds_ws/    # 宇树官方仓库
│       └── src/unitree/
│           ├── unitree_api/msg/       # Request/RequestHeader
│           └── unitree_go/msg/        # SportModeState/SportModeCmd
│
├── scripts/
│   ├── setup_unitree_env.sh           # 一键环境安装
│   └── run_scenario_tests.sh          # ★ 场景自动测试脚本（5/24新增）
│
└── docs/unittree/
    └── unitree_bridge_项目总结与真机对接指南.md  # ★ 本文档
```

---

## 八、后续规划

| 优先级 | 任务 | 说明 |
|:--|:--|:--|
| **P0** | **真机联调** | 按第四章步骤执行，先跑 `curved_normal` |
| **P1** | 订阅 `/lf/sportmodestate` 回采 | 获取 Go2 真实姿态/速度/步态，用于闭环反馈 |
| **P2** | 替换 `simulated_input_node` | 接入上游步骤 1-4 的真实感知/规划输出 |
| **P3** | `body_height` 控制 | 当前 DogMotion 有 body_height 但 bridge 未映射 |
| **P4** | 完善异常恢复 | 通信断开自动安全状态、重连恢复 |

---

> **代码已就绪，环境已验证，6 种 API 全覆盖，静待真机上电。先跑 `curved_normal`，遥控器放手边。**
