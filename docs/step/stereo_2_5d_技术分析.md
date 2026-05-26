# 双目相机 → 2.5D 局部地形图 技术分析

> 状态：方案设计阶段，待用户确认后实施
> 日期：2026-05-26

---

## 一、总体目标

从 Go2 前置双目相机实时生成机器人前方局部 2.5D 高程栅格图：
- **2D 栅格**（鸟瞰视角 BEV，x-y 平面）
- **每格存储高度 z**（第 2.5 维，选 max/min/mean）
- **供下游导航模块判断地形可通行性**

---

## 二、管线设计

### 2.1 完整数据流

```
Go2 双目相机
  ├── 左目图像 (sensor_msgs/Image)
  └── 右目图像 (sensor_msgs/Image)
           │
      ┌────▼─────┐
      │ 1. 矫正   │  相机内参 K、畸变系数 D → cv2.undistort
      │    (可选)  │  如果已矫正则跳过
      └────┬─────┘
           │
      ┌────▼─────┐
      │ 2. 立体   │  cv2.StereoSGBM → 视差图 (disparity map)
      │    匹配   │  参数: blockSize, numDisparities, P1/P2
      └────┬─────┘
           │
      ┌────▼─────┐
      │ 3. 视差   │  Z = fB / d
      │  → 深度   │  f=focal length, B=baseline, d=disparity
      └────┬─────┘
           │
      ┌────▼─────┐
      │ 4. 3D投影 │  像素(u,v,d) → 相机坐标(Xc,Yc,Zc)
      │           │  Xc = (u - cx) * Z / fx
      │           │  Yc = (v - cy) * Z / fy
      └────┬─────┘
           │
      ┌────▼─────┐
      │ 5. 坐标   │  相机 → 机器人本体坐标
      │    变换   │  已知: 相机安装高度 H, 俯仰角 θ
      │           │  Xr = Xc
      │           │  Yr = Zc*cosθ - Yc*sinθ + H
      │           │  Zr = -Zc*sinθ - Yc*cosθ
      └────┬─────┘
           │
      ┌────▼─────┐
      │ 6. 栅格   │  3D点 → NumPy 2D grid
      │    投影   │  grid[col][row] = max(z) 或 mean(z)
      │           │  过滤: Z 超出范围、梯度异常
      └────┬─────┘
           │
      ┌────▼─────┐
      │ 7. 发布   │  自定义消息 或 OccupancyGrid
      │    输出   │
      └──────────┘
```

### 2.2 依赖库

| 库 | 用途 | 大小 | 可替代 |
|:--|:--|:--|:--|
| `numpy` | 栅格矩阵运算 | ~20MB | 不可替 |
| `opencv-python` (cv2) | 图像读取、SGBM 立体匹配 | ~50MB | ⚠️ 手写匹配器代码量大、效果差 |
| `rclpy` | ROS2 节点 | 已安装 | 不可替 |
| `cv_bridge` | ROS2 Image ↔ OpenCV Mat | 已安装 | 可手写但没必要 |

> **没有 OpenCV 大库（opencv-contrib 200MB+），只用 opencv-python 基础版 ~50MB。** 如果连 cv2 都不想用，立体匹配可手写块匹配（SAD/NCC），但代码量 ×4、速度 ×0.3。

---

## 三、关键参数及影响

| 参数 | 推荐值 | 影响 |
|:--|:--|:--|
| **栅格分辨率** | 0.05 m/格 | 精度 vs 计算量。0.10m 更轻量 |
| **地图范围** | 前方 4m × 左右各 1.5m (3m 宽) | Go2 一步 ~0.3m，4m 是 ~13 步的前瞻 |
| **视差搜索范围** | 64~128 | StereoSGBM numDisparities，影响有效深度范围 |
| **块大小** | 9 | SGBM blockSize，奇数，大=平滑但细节少 |
| **有效深度** | 0.5m ~ 6m | 太近(腿拍到)、太远(视差精度差)都过滤 |
| **更新频率** | 3~5 Hz | 深度计算量大，不宜过高 |
| **高度过滤** | z > 0.05m 视为障碍 | 过滤地面平面噪声 |

---

## 四、待确认信息（需要你提供）

### Q1: 双目相机参数

Go2 的前置双目相机是什么型号？在 Go2 SSH 里跑：

```bash
# 方式1: 看话题
ros2 topic list | grep -iE "camera|image|stereo|depth|rgb"

# 方式2: 如果话题已经有了，看相机信息
ros2 topic info /camera/depth/image_rect_raw 2>/dev/null
ros2 topic info /camera/infra1/image_rect_raw 2>/dev/null

# 方式3: 看 /camera_info
ros2 topic list | grep camera_info
```

**需要知道：**
- 相机型号（D435i? T265? Go2 自带双目?）
- 基线长度 B（Go2 自带 ≈ 20-25cm，D435i ≈ 5cm）
- 分辨率（默认 640×480? 848×480?）
- 是否已有标定话题 `/camera/.../camera_info`

### Q2: 安装位置

- 相机离地面高度？（Go2 机身顶端 ≈ 0.35m）
- 俯仰角？（水平安装 ≈ 0°，向下倾斜 ≈ 15°）

### Q3: 地图需求

- 地图范围：前方 ×m？宽度 ×m？（推荐 4m×3m）
- 输出格式：继续用 `safety_state.terrain_cost` 增强，还是独立话题？
- 地图是连续更新还是帧帧替换？

### Q4: 运行环境

- 代码跑在 **Go2 机载 Jetson** 上还是**你电脑**上？
  - 跑 Jetson：考虑算力（Jetson Orin NX ~70 TOPS 足够）
  - 跑电脑：需传图像流过来，网络带宽 640×480×2 双目 ≈ 7 MB/s 可以接受

---

## 五、代码结构（草案）

```
src/dog_nav_step56/dog_nav_step56/
├── stereo_terrain_node.py     # 主节点：订阅双目 → 发布 2.5D 图
└── stereo_utils.py            # 工具：栅格投影、过滤、高度提取

src/dog_nav_step56/config/
└── stereo_terrain.yaml        # 参数：分辨率、范围、相机位姿

src/dog_nav_step56/launch/
└── stereo_terrain.launch.py   # 独立启动（可配合 bridge 一起跑）
```

节点设计：
- **输入:** `left_image`, `right_image`, `camera_info_left`, `camera_info_right`
- **输出:** 自定义 `TerrainGrid` 消息（2D float32 数组 + metadata）或直接用 `OccupancyGrid`
- **参数:** 分辨率、地图长宽、相机安装位姿、最小深度、块大小

---

## 六、预期效果与限制

| 场景 | 预期效果 | 限制 |
|:--|:--|:--|
| 平坦地面 | 均匀高度，障碍标记干净 | 纹理弱可能视差噪声大 |
| 台阶/路缘 | 高度突变清晰可见 | 高度 5mm 以下的台阶可能被滤除 |
| 草地/碎石 | 平均高度变化平滑 | 细纹理视差噪声较多 |
| 玻璃/反光 | 视差缺失区域 | 双目对此类材质天生受限 |
| 暗光/夜晚 | 图像噪声大 | 双目需外部光源 |

---

> **审查后回复：Q1-Q4 的答案，以及你对管线设计的意见。确认后开始写代码。**
