# 对接交付清单 — 2.5D 局部地形 + 桥接真机联调

> 日期: 2026-05-28
> 仓库: `git@github.com:zch606/go2-nav.git` (main)
> 上次交付: step5-6 bridge 开发完成

---

## 一、本次新增内容

### 1.1 宇树 SDK 桥接（已完成真机联调）

| 文件 | 说明 |
|:--|:--|
| `src/dog_nav_step56/dog_nav_step56/unitree_bridge_node.py` | DogMotion → Unitree Sport API (Move/Damp/StopMove/GaitSwitch/SpeedLevel) |
| `src/dog_nav_step56/config/unitree_bridge.yaml` | 桥接参数 (话题名、频率等) |
| `src/dog_nav_step56/launch/unitree_bridge.launch.py` | 独立启动 bridge |
| `src/dog_nav_step56/dog_nav_step56/motion_adapter_node.py` | cmd_vel → DogMotion 适配 |
| `src/dog_nav_step56/dog_nav_step56/safety_supervisor_node.py` | 安全状态机 |
| `src/dog_nav_step56/dog_nav_step56/local_controller_node.py` | Pure Pursuit 路径跟踪 (lookahead 参数已校调) |
| `docs/unittree/unitree_bridge_项目总结与真机对接指南.md` | 真机操作手册 |

> ✅ 2026-05-24 真机调通：网线直连, DDS 单播, test_forward 场景 Go2 正常行走

### 1.2 D435i → 2.5D 局部地形（离线验证通过）

| 文件 | 说明 |
|:--|:--|
| `src/dog_nav_step56/dog_nav_step56/stereo_terrain_node.py` | 主节点: 订阅 D435i 深度图 → 发布高程栅格 |
| `src/dog_nav_step56/dog_nav_step56/grid_utils.py` | 工具: 深度→3D、坐标变换、栅格投影、坡度计算 |
| `src/dog_nav_step56/dog_nav_step56/test_stereo_terrain_offline.py` | 离线验证脚本 (ALL PASS) |
| `src/dog_nav_step56/config/stereo_terrain.yaml` | 参数配置 |
| `src/dog_nav_step56/launch/stereo_terrain.launch.py` | 独立启动 |
| `docs/stereo/stereo_2_5d_技术分析.md` | 技术文档 |

> ✅ 2026-05-28 离线 ALL PASS: 平坦地面 + 盒子 + 斜坡 全部正确检测

---

## 二、组员注意

### 2.1 环境要求

**VM (Ubuntu 22.04):**
```bash
# ROS2 Humble + 基础依赖（一次性）
sudo apt install -y ros-humble-ros-base python3-colcon-common-extensions python3-pip
pip3 install numpy pyyaml opencv-python

# 编译项目
cd ~/Daohang
source /opt/ros/humble/setup.bash
source src/unitree_ros2/cyclonedds_ws/install/setup.bash
colcon build --packages-select dog_nav_interfaces dog_nav_step56
```

**Go2 Jetson:**
```bash
# D435i 驱动（一次性）
sudo apt install -y ros-humble-realsense2-camera
pip3 install numpy opencv-python
```

### 2.2 真机 vs 仿真区别

| | 仿真模式 | 真机模式 |
|:--|:--|:--|
| 启动参数 | `scenario:=xxx` | `use_bridge:=true scenario:=xxx` |
| 里程计 | 公式推的假数据 | Go2 真实轮速/IMU |
| 路径 | 场景预设路点 | 规划器真实路径 |
| 地形成本 | YAML 里写死的值 | D435i 深度图实时算 |
| 指令发给 Go2 | ❌ 不发 | ✅ 通过 DDS 发 |
| Go2 会动吗 | ❌ 不会 | ✅ 会 |

### 2.3 真机全流程（从零开始）

**前置：网线连接**

```
电脑以太网口 ←── 网线 ──→ Go2 背部网口
```

Windows 设静态 IP：
```
IP: 192.168.123.99, 掩码: 255.255.255.0, 网关: 留空
```

**VMware 网卡：** VM 关机 → 设置 → 添加网络适配器 → 桥接模式 → 桥接到有线网卡。

**第 1 步：VM 设网口 + DDS（每次开机一次）**

```bash
sudo ip addr add 192.168.123.100/24 dev ens38
export RMW_IMPLEMENTATION=rmw_cyclonedds_cpp
export CYCLONEDDS_URI='<CycloneDDS><Domain><General><Interfaces><NetworkInterface address="192.168.123.100"/></Interfaces></General></Domain></CycloneDDS>'
ping 192.168.123.18   # 确认物理连通
```

**第 2 步：传代码 + 编译**

```bash
# Windows CMD（每次改代码后）：
scp -r "d:\Daohang\src\dog_nav_step56" viego@192.168.123.100:~/Daohang/src/

# VM 里编译：
cd ~/Daohang
source /opt/ros/humble/setup.bash
source src/unitree_ros2/cyclonedds_ws/install/setup.bash
colcon build --packages-select dog_nav_step56
source install/setup.bash
```

**第 3 步：验证 Go2 就绪**

```bash
# Go2 SSH:
ros2 topic list | grep sport    # 需看到 /lf/sportmodestate
```

**第 4 步：验证 D435i 深度图（可选，仅测地形）**

```bash
# Go2 SSH:
ros2 topic echo /camera/depth/image_rect_raw --once | head -1
# 没输出 → sudo apt install ros-humble-realsense2-camera

# VM:
ros2 launch dog_nav_step56 stereo_terrain.launch.py
ros2 topic echo /terrain_cost    # 20=平坦, >100=有障碍
```

**第 5 步：启动真机导航（遥控器放手边！）**

```bash
# VM:
export RMW_IMPLEMENTATION=rmw_cyclonedds_cpp
export CYCLONEDDS_URI='<CycloneDDS><Domain><General><Interfaces><NetworkInterface address="192.168.123.100"/></Interfaces></General></Domain></CycloneDDS>'

# 先跑最安全的 test_forward（纯直行 5m）
ros2 launch dog_nav_step56 closed_loop_demo.launch.py use_bridge:=true scenario:=test_forward

# 其他场景:
# curved_normal    — 蛇形巡航
# zigzag_with_tilt — Z字折返+倾斜
# round_trip       — 直行往返+坐下
```

**安全：遥控器始终在手，异常立刻拍急停。**

### 2.4 参数调优

`config/stereo_terrain.yaml` 中两个参数最可能需要调：

```yaml
cam_height: 0.35        # 实测后修正
cam_pitch_deg: 10.0     # 看不到地面→增大, 拍到腿→减小
```

改完 YAML 重启节点即可，不用重编译。

### 2.4 已知限制

| 项 | 说明 |
|:--|:--|
| Pure Pursuit + 模拟里程计 | 复杂路径可能原地转，真里程计接入后自然消除 |
| D435i 在 10° 下倾时 | 最近可见地面 ~2m, `depth_min=0.3` 可过滤 Go2 自身 |
| 坐标变换已离线验证 | 但真机上需验证 `cam_height`/`cam_pitch_deg` |

---

## 三、文件清单（从仓库 clone 即可）

```
src/dog_nav_step56/
├── dog_nav_step56/
│   ├── unitree_bridge_node.py        # ★ 桥接节点
│   ├── motion_adapter_node.py
│   ├── safety_supervisor_node.py
│   ├── local_controller_node.py      # ★ lookahead 已校准
│   ├── stereo_terrain_node.py        # ★ 2.5D 地形
│   ├── grid_utils.py                 # ★ 坐标变换
│   ├── test_stereo_terrain_offline.py
│   └── ...
├── config/
│   ├── unitree_bridge.yaml
│   ├── stereo_terrain.yaml           # ★ 调参入口
│   └── ...
├── launch/
│   ├── stereo_terrain.launch.py      # ★ 地形独立启动
│   ├── closed_loop_demo.launch.py
│   └── ...
└── setup.py                          # ★ 已注册新节点入口

docs/
├── unittree/unitree_bridge_项目总结与真机对接指南.md
└── stereo/stereo_2_5d_技术分析.md     # ★ 数学推导+参数说明
```

---

## 四、快速验证

```bash
# 离线跑一遍确保管线正常（只需 numpy）
python src/dog_nav_step56/dog_nav_step56/test_stereo_terrain_offline.py
# 预期: ALL PASS
```
