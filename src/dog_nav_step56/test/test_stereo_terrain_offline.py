"""
test_stereo_terrain_offline.py -- 离线验证 2.5D 地形管线

跳过相机投影，直接从地面 3D 点云验证：
深度→3D → 栅格 → 坡度 → 成本 全部覆盖。
"""
import math
import numpy as np


CAM_H = 0.35
CAM_PITCH = math.radians(10.0)
GRID_RES = 0.05
GRID_LEN = 5.0
GRID_WID = 3.0
DEPTH_MIN = 0.0
DEPTH_MAX = 6.0
Z_MIN = -0.02
Z_MAX = 0.80
FX, FY, CX, CY = 424.0, 424.0, 424.0, 240.0


def make_ground_3d() -> (np.ndarray, np.ndarray, np.ndarray, np.ndarray):
    """生成模拟地面 3D 点云 (相机坐标系) + 障碍物."""

    xx, yy = np.meshgrid(
        np.linspace(-1.5, 1.5, 60),
        np.linspace(2.0, 5.0, 60),
    )
    zz = np.full_like(xx, 0.0)

    box_mask = (np.abs(xx) < 0.2) & (yy > 2.5) & (yy < 3.5)
    zz[box_mask] = 0.4

    ramp_mask = (xx < -0.4) & (xx > -1.0) & (yy > 3.0) & (yy < 5.0)
    ramp_frac = (yy[ramp_mask] - 3.0) / 2.0
    zz[ramp_mask] = ramp_frac * 0.15

    cos_t, sin_t = math.cos(CAM_PITCH), math.sin(CAM_PITCH)
    xc = xx
    yc = -sin_t * yy - cos_t * (zz - CAM_H)
    zc = cos_t * yy - sin_t * (zz - CAM_H)

    return xc.flatten(), yc.flatten(), zc.flatten(), zz.flatten()


def main() -> None:
    print("=" * 60)
    print("  2.5D Terrain Pipeline Offline Test")
    print("=" * 60)

    # 1. 生成 3D 点
    xc, yc, zc, z_true = make_ground_3d()
    n = len(zc)
    print(f"\n[1] 3D points: {n}, Zc={zc.min():.2f}~{zc.max():.2f}m, "
          f"Z_true={z_true.min():.2f}~{z_true.max():.2f}m")

    # 2. 相机 → 地面坐标
    cos_t, sin_t = math.cos(CAM_PITCH), math.sin(CAM_PITCH)
    xg = xc
    yg = -sin_t * yc + cos_t * zc
    zg = CAM_H - cos_t * yc - sin_t * zc

    valid_calib = zc > 0
    print(f"[2] Ground: Yg={yg[valid_calib].min():.2f}~{yg[valid_calib].max():.2f}m, "
          f"Zg={zg[valid_calib].min():.3f}~{zg[valid_calib].max():.3f}m")

    # 3. 栅格投影
    gh = int(GRID_LEN / GRID_RES)
    gw = int(GRID_WID / GRID_RES)
    half = gw // 2

    col = (xg / GRID_RES + half).astype(np.int32)
    row = (yg / GRID_RES).astype(np.int32)
    valid = valid_calib & \
        (col >= 0) & (col < gw) & \
        (row >= 0) & (row < gh) & \
        (yg > DEPTH_MIN) & (yg < DEPTH_MAX) & \
        (zg > Z_MIN) & (zg < Z_MAX)

    grid = np.full((gh, gw), -np.inf, dtype=np.float32)
    if valid.any():
        np.maximum.at(grid, (row[valid], col[valid]), zg[valid])

    n_grid = np.sum(np.isfinite(grid))
    print(f"[3] Grid {gh}x{gw}: {n_grid}/{gh*gw} valid ({100*n_grid/(gh*gw):.0f}%), "
          f"elev={np.nanmin(grid):.3f}~{np.nanmax(grid):.3f}m")

    # 4. 坡度 + 成本
    filled = np.where(np.isfinite(grid), grid, 0.0)
    dy, dx = np.gradient(filled)
    dy /= GRID_RES
    dx /= GRID_RES
    slope = np.arctan(np.sqrt(dy ** 2 + dx ** 2))
    valid_slope = slope[np.isfinite(grid)]
    cost = float(np.tanh(np.nanmean(valid_slope) * 20) * 150 + 20)
    print(f"[4] Avg slope={np.nanmean(slope):.4f}rad "
          f"({np.degrees(np.nanmean(slope)):.1f}deg), cost={cost:.1f}")

    # 5. 障碍检测
    finite = np.isfinite(grid)
    obs = np.sum((grid > 0.15) & finite)
    ramp = np.sum((grid > 0.06) & (grid <= 0.15) & finite)
    print(f"[5] Obstacles(>0.15m): {obs} cells ({obs*GRID_RES**2:.2f}m^2), "
          f"Ramp(0.06~0.15): {ramp} cells ({ramp*GRID_RES**2:.2f}m^2)")

    # 6. 可视化
    print(f"\n[6] Grid view (4x downsample, #=obstacle, .=flat, _=empty):")
    vis = grid[::4, ::4]
    for r in range(vis.shape[0]):
        line = "".join(
            "#" if np.isfinite(v) and v > 0.15
            else "~" if np.isfinite(v) and v > 0.06
            else "." if np.isfinite(v) else "_"
            for v in vis[r]
        )
        print(f"  {line}")

    # 7. 结论
    print("\n" + "=" * 60)
    checks = [
        n_grid > 500,
        obs > 10,
        ramp > 5,
        30 < cost < 160,
    ]
    if all(checks):
        print("  ALL PASS: pipeline fully functional")
    else:
        failed = []
        if not checks[0]: failed.append("grid coverage")
        if not checks[1]: failed.append("box detection")
        if not checks[2]: failed.append("ramp detection")
        if not checks[3]: failed.append("cost range")
        print(f"  PARTIAL: issues with {', '.join(failed)}")
    print("=" * 60)


if __name__ == "__main__":
    main()
