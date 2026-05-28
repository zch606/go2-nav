"""
grid_utils.py — D435i 深度图 → 2.5D 高程栅格 工具函数

纯 NumPy 实现，无大库依赖。
"""

import math
import numpy as np


def depth_to_3d(depth_mm: np.ndarray,
                fx: float, fy: float,
                cx: float, cy: float) -> tuple[np.ndarray, np.ndarray, np.ndarray]:
    """
    深度图 → 3D 点云 (相机坐标系)。

    Args:
        depth_mm: 深度图 (H×W), 单位 mm, 0 表示无效
        fx, fy:  相机内参焦距
        cx, cy:  相机内参主点

    Returns:
        xc, yc, zc: 各 (H×W), 单位 m
    """
    rows, cols = depth_mm.shape
    uu, vv = np.meshgrid(np.arange(cols), np.arange(rows))

    zc = depth_mm.astype(np.float32) * 0.001  # mm → m
    xc = (uu - cx) * zc / fx
    yc = (vv - cy) * zc / fy

    return xc, yc, zc


def camera_to_ground(xc: np.ndarray, yc: np.ndarray, zc: np.ndarray,
                     cam_height: float, cam_pitch: float) -> tuple[np.ndarray, np.ndarray, np.ndarray]:
    """
    相机坐标系 → 地面鸟瞰坐标系。

    相机原点在机器人正前方，光轴下倾 cam_pitch 弧度。
    地面坐标系: Xg=前方, Yg=左方(鸟瞰), Zg=向上(高程)

    Args:
        xc, yc, zc:  相机坐标系 3D 点 (m)
        cam_height:   相机离地高度 (m)
        cam_pitch:    相机俯仰角 (rad), 正=下倾

    Returns:
        xg, yg, zg: 地面坐标系 3D 点 (m)
    """
    cos_t, sin_t = math.cos(cam_pitch), math.sin(cam_pitch)

    xg = xc                                      # 前方不变
    yg = zc * cos_t - yc * sin_t                 # 深度→鸟瞰距离
    zg = cam_height - zc * sin_t - yc * cos_t    # 高程 (0=地面)

    return xg, yg, zg


def points_to_grid(xg: np.ndarray, yg: np.ndarray, zg: np.ndarray,
                   zc: np.ndarray,
                   grid_res: float, grid_w: int, grid_h: int,
                   z_min: float, z_max: float,
                   depth_min: float, depth_max: float,
                   aggregation: str = 'max') -> np.ndarray:
    """
    3D 点云 → 2.5D 高程栅格。

    栅格坐标: row=y方向(前方), col=x方向(左右)

    Args:
        xg, yg, zg:  地面坐标系 3D 点 (m)
        zc:          原始深度值 (m), 用于过滤
        grid_res:    栅格分辨率 (m/格)
        grid_w:      栅格宽度 (格数, 左右方向)
        grid_h:      栅格高度 (格数, 前后方向)
        z_min:       最小高程 (m), 低于此值过滤
        z_max:       最大高程 (m), 高于此值过滤
        depth_min:   最小深度过滤 (m)
        depth_max:   最大深度过滤 (m)
        aggregation: 'max' 或 'mean'

    Returns:
        grid: (grid_h × grid_w), 无效格 = NaN
    """
    half_w = grid_w // 2

    col = (yg / grid_res + half_w).astype(np.int32)
    row = (xg / grid_res).astype(np.int32)

    valid = (
        (col >= 0) & (col < grid_w) &
        (row >= 0) & (row < grid_h) &
        (zc > depth_min) & (zc < depth_max) &
        (zg > z_min) & (zg < z_max)
    )

    grid = np.full((grid_h, grid_w), np.nan, dtype=np.float32)

    if not valid.any():
        return grid

    r = row[valid]
    c = col[valid]
    z = zg[valid]

    if aggregation == 'max':
        np.maximum.at(grid, (r, c), z)
    elif aggregation == 'mean':
        counts = np.zeros_like(grid)
        np.add.at(grid, (r, c), z)
        np.add.at(counts, (r, c), 1)
        mask = counts > 0
        grid[mask] /= counts[mask]
    else:
        raise ValueError(f"aggregation must be 'max' or 'mean', got '{aggregation}'")

    return grid


def grid_to_slope(grid: np.ndarray, resolution: float) -> tuple[np.ndarray, float]:
    """
    高程栅格 → 坡度图 + 平均成本。

    Args:
        grid:      高程栅格 (H×W), NaN 为无效
        resolution: 栅格分辨率 (m/格)

    Returns:
        slope_grid: 坡度图 (H×W, 弧度), NaN 为无效
        mean_cost:  平均地形成本 (0~255), 仿 terrain_cost
    """
    if np.all(np.isnan(grid)):
        return np.full_like(grid, np.nan), 20.0

    dy, dx = np.gradient(grid)
    dy /= resolution
    dx /= resolution
    slope = np.arctan(np.sqrt(dy ** 2 + dx ** 2))

    mean_cost = float(np.tanh(np.nanmean(slope) * 20) * 150 + 20)

    return slope, mean_cost
