# D435i 深度相机 → 2.5D 局部地形图 技术分析

> 状态：方案设计阶段，待确认 Q2-Q4 后编码
> 日期：2026-05-28

---

## 一、总体目标

从 **Intel RealSense D435i** 深度相机实时生成机器人前方局部 2.5D 高程栅格图：
- **2D 栅格**（鸟瞰视角 BEV，x-y 平面）
- **每格存储高度 z**（第 2.5 维）
- **供下游 `safety_supervisor` 判断地形可通行性**

> D435i 自带片上 ASIC 完成立体匹配，直接输出深度图——**不需要手写 SGBM，管线极简。**

### D435i 关键参数

| 项目 | 值 |
|:--|:--|
| 基线 | 50mm |
| 深度分辨率 | 848×480 (推荐) / 640×480 |
| 深度 FOV | 87°×58° |
| 有效深度 | 0.3m ~ 10m+ |
| ROS2 驱动 | `ros-humble-realsense2-camera` (apt) |

---

## 二、管线设计

### 2.1 数据流（4 步）

```
Intel RealSense D435i
  │
  ├── /camera/depth/image_rect_raw   (16UC1, 单位 mm)
  └── /camera/depth/camera_info      (内参 K, D, P)
           │
      ┌────▼─────┐
      │ 1. 深度   │  深度图 + 内参 → 3D 点云 (相机坐标系)
      │  → 3D    │  Xc = Z * (u-cx)/fx
      │           │  Yc = Z * (v-cy)/fy
      │           │  Zc = Z (直接从深度图读)
      └────┬─────┘
           │
      ┌────▼─────┐
      │ 2. 坐标   │  相机坐标系 → 地面鸟瞰坐标
      │    变换   │  Xg = Xc
      │           │  Yg = Zc*cosθ - Yc*sinθ + H
      │           │  Zg = -Zc*sinθ - Yc*cosθ
      │           │  (H=相机高, θ=俯仰角)
      └────┬─────┘
           │
      ┌────▼─────┐
      │ 3. 栅格   │  3D 点 → NumPy 2D grid
      │    投影   │  grid[row][col] = max(Zg)
      │           │  过滤: Z=0(无效)、Z>6m(太远)、Z<0.3m(腿)
      └────┬─────┘
           │
      ┌────▼─────┐
      │ 4. 发布   │  ~/local_terrain 自定义消息
      │    输出   │  供 safety_supervisor 消费
      └──────────┘
```

### 2.2 依赖库

| 库 | 用途 | 大小 |
|:--|:--|:--|
| `numpy` | 栅格矩阵运算 | ~20MB |
| `opencv-python` | 深度图读取 + 基本操作 | ~50MB |
| `rclpy` | ROS2 节点 | 已安装 |
| `cv_bridge` | Image ↔ Mat | 已安装 |
| `ros-humble-realsense2-camera` | D435i 驱动 | Go2 预装 |

> **无大仓库、无 SLAM、无手写立体匹配。仅用 4 个基础 Python 库。**

---

## 三、坐标变换（核心）

### 3.1 相机 → 地面

D435i 安装在机器人正前方，高度 H，俯仰角 θ（正=下倾）。

```
世界坐标系: Xw=右, Yw=前, Zw=上
相机坐标系: Xc=右, Yc=下(图像), Zc=前(光轴)

R_c2w = [[1, 0,      0    ],      R_w2c = R_c2w^T
         [0, -sinθ,  cosθ ],
         [0, -cosθ,  -sinθ]]

P_world = R_c2w * P_camera + [0, 0, H]
```

导出：
```
xg = xc                          ← 左右不变
yg = -sinθ * yc + cosθ * zc      ← 前方距离
zg = H - cosθ * yc - sinθ * zc   ← 高程(0=地面)
```

> 本公式已离线测试验证：输入平坦地面+盒子(0.4m)+斜坡(0.15m)，输出全部正确。

### 3.2 深度 → 相机 3D

```
D435i 深度图单位: mm (16UC1)
zc = depth_mm * 0.001           ← mm→m
xc = (u - cx) * zc / fx
yc = (v - cy) * zc / fy
```

---

## 三、关键参数

| 参数 | 默认值 | 影响 |
|:--|:--|:--|
| 栅格分辨率 | 0.05m/格 | 精度 vs 计算量 |
| 地图范围 | 前 4m × 宽 3m | Go2 一步 ~0.3m |
| 有效深度 | 0.3m ~ 6m | 过滤无效和远处 |
| 更新频率 | 5 Hz | 深度处理便宜 |
| 高度判断 | z>0.03m=障碍 | 过滤地面噪声 |

---

## 四、待确认

| # | 问题 | 状态 |
|:--:|:--|:--:|
| Q1 | 相机型号 | ✅ D435i |
| Q2 | 安装高度 H=0.35m、俯仰角 θ=10° | ✅ 待真机验证 |
| Q3 | 地图范围 4m×3m、话题 /local_terrain | ✅ |
| Q4 | 跑 Jetson 上 | ✅ |

---

## 五、代码结构

```
src/dog_nav_step56/dog_nav_step56/
├── stereo_terrain_node.py    # 主节点: 订阅深度 → 发布 2.5D 栅格
├── grid_utils.py             # 工具: 3D投影、坐标变换、栅格化、坡度
└── test_stereo_terrain_offline.py  # 离线验证 (ALL PASS)

src/dog_nav_step56/config/
└── stereo_terrain.yaml       # 参数: 分辨率、范围、相机位姿

src/dog_nav_step56/launch/
└── stereo_terrain.launch.py  # 独立启动
```

节点设计：
- **输入:** `/camera/depth/image_rect_raw`, `/camera/depth/camera_info`
- **输出:** `/local_terrain` (sensor_msgs/Image 32FC1), `/terrain_cost` (std_msgs/Float32)
- **参数:** 见 `config/stereo_terrain.yaml`，全量可调

---

## 六、参数速查（需要调时看这个）

所有参数在 `config/stereo_terrain.yaml`，改完重启节点即可：

| 参数 | 默认值 | 说明 | 调整建议 |
|:--|:--|:--|:--|
| `grid_resolution` | 0.05 | 栅格精度 m/格 | 降低→更快但粗糙 |
| `grid_length_m` | 4.0 | 前方覆盖 m | 按需要调整 |
| `grid_width_m` | 3.0 | 左右覆盖 m | ±1.5m |
| `cam_height` | 0.35 | 相机离地 m | 实测后修正 |
| `cam_pitch_deg` | 10.0 | 俯仰角 ° | 看不到地面→增大；看到腿→减小 |
| `depth_min` | 0.3 | 最近深度 m | 拍到 Go2 腿→增大 |
| `depth_max` | 6.0 | 最远深度 m | D435i 有效 ~10m，但 >6m 噪声大 |
| `z_min` | 0.005 | 地面以下 m | 过滤噪声 |
| `z_max` | 0.80 | 障碍高度 m | 可能太高→调小，看到太多点→调大 |
| `publish_rate_hz` | 5.0 | 输出频率 | Jetson 跑这个轻松，10Hz 也没问题 |
| `aggregation` | max | 每格取值 | max=最高点/mean=平均 |

## 七、真机验证步骤

```bash
# 1. 确保 D435i 驱动在跑
ros2 topic echo /camera/depth/image_rect_raw --once | head -1

# 2. 启动地形节点
ros2 launch dog_nav_step56 stereo_terrain.launch.py

# 3. 看输出
ros2 topic echo /terrain_cost         # 地形成本 (20=平, >100=有障碍)
ros2 topic echo /local_terrain --once  # 高程栅格图
```
