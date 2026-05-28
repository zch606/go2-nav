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

```bash
# Go2 Jetson 上
sudo apt install -y ros-humble-realsense2-camera   # D435i 驱动
pip3 install numpy opencv-python                     # 已有

# 编译
cd ~/go2-nav
source /opt/ros/humble/setup.bash
source src/unitree_ros2/cyclonedds_ws/install/setup.bash
colcon build --packages-select dog_nav_step56 dog_nav_interfaces
source install/setup.bash
```

### 2.2 真机启动顺序

```bash
# 1. 确保 D435i 驱动正常
ros2 topic echo /camera/depth/image_rect_raw --once | head -1

# 2. 启动 2.5D 地形感知
ros2 launch dog_nav_step56 stereo_terrain.launch.py

# 3. 验证输出
ros2 topic echo /terrain_cost       # 20~170, 越高越危险
ros2 topic echo /local_terrain --once  # 高程栅格

# 4. 合体启动（桥接 + 导航 + 地形）
ros2 launch dog_nav_step56 closed_loop_demo.launch.py \
    use_bridge:=true scenario:=test_forward
```

### 2.3 参数调优

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
