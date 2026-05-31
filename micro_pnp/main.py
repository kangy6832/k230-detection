import math
import os
import time

from media.display import *
from media.media import *
from media.sensor import *

# ============================================================
# 配置参数
# ============================================================

# 正方形目标定义
SQUARE_SIZE_MM = 100.0   # 边长 100mm = 10cm
LINE_WIDTH_MM = 3.0      # 线宽 3mm = 0.3cm

# 摄像头采集设置（HD）
FRAME_WIDTH = 640
FRAME_HEIGHT = 480

# ROI（感兴趣区域），全图检测
USE_ROI = False
ROI = (0, 0, FRAME_WIDTH, FRAME_HEIGHT)

# 轮廓检测参数
FIND_RECTS_THRESHOLD = 7000  # find_rects 边缘强度阈值

# 正方形验证参数
MAX_ASPECT_RATIO_DEVIATION = 0.25   # 宽高比偏差阈值
MAX_SIDE_RELATIVE_ERROR = 0.18      # 边长相对误差阈值
MIN_INTERIOR_ANGLE = 80             # 最小内角（度）
MAX_INTERIOR_ANGLE = 100            # 最大内角（度）
MAX_DIAGONAL_RATIO_ERROR = 0.10     # 对角线长度差异比例阈值

# 亚像素角点优化参数
SUBPIX_WIN_SIZE = 5         # 搜索窗口半宽
SUBPIX_MAX_ITER = 20        # 最大迭代次数
SUBPIX_EPSILON = 0.001      # 收敛阈值

# 相机内参矩阵（需填入实际标定数据）
# 格式：[[fx, 0, cx], [0, fy, cy], [0, 0, 1]]
CAMERA_MATRIX = [
    [800.0, 0.0, 640.0],
    [0.0, 800.0, 360.0],
    [0.0, 0.0, 1.0],
]

# 位姿验证参数
MAX_REPROJECTION_ERROR_PX = 3.0   # 最大重投影误差（像素）
MIN_CAMERA_Z_MM = 80.0            # 最小相机距离（毫米）
MAX_CAMERA_Z_MM = 300.0           # 最大相机距离（毫米）

# 平滑参数
ENABLE_SMOOTHING = True
EMA_ALPHA = 0.45
MAX_JUMP_MM = 50.0                # 单帧坐标最大跳动（毫米）

# 调试参数
DRAW_DEBUG_OVERLAY = True
PRINT_EVERY_N_FRAMES = 5


# ============================================================
# 几何工具函数
# ============================================================

def _dot3(a, b):
    return a[0] * b[0] + a[1] * b[1] + a[2] * b[2]


def _norm3(v):
    return math.sqrt(_dot3(v, v))


def _cross3(a, b):
    return [
        a[1] * b[2] - a[2] * b[1],
        a[2] * b[0] - a[0] * b[2],
        a[0] * b[1] - a[1] * b[0],
    ]


def _sub3(a, b):
    return [a[0] - b[0], a[1] - b[1], a[2] - b[2]]


def _add3(a, b):
    return [a[0] + b[0], a[1] + b[1], a[2] + b[2]]


def _mul3(v, s):
    return [v[0] * s, v[1] * s, v[2] * s]


def _mat3_mul_vec(m, v):
    return [
        m[0][0] * v[0] + m[0][1] * v[1] + m[0][2] * v[2],
        m[1][0] * v[0] + m[1][1] * v[1] + m[1][2] * v[2],
        m[2][0] * v[0] + m[2][1] * v[1] + m[2][2] * v[2],
    ]


def _mat3_inv(m):
    a, b, c = m[0]
    d, e, f = m[1]
    g, h, i = m[2]

    A = e * i - f * h
    B = -(d * i - f * g)
    C = d * h - e * g
    D = -(b * i - c * h)
    E = a * i - c * g
    F = -(a * h - b * g)
    G = b * f - c * e
    H = -(a * f - c * d)
    I = a * e - b * d

    det = a * A + b * B + c * C
    if abs(det) < 1e-9:
        return None

    inv_det = 1.0 / det
    return [
        [A * inv_det, D * inv_det, G * inv_det],
        [B * inv_det, E * inv_det, H * inv_det],
        [C * inv_det, F * inv_det, I * inv_det],
    ]


def _distance2(p0, p1):
    dx = p0[0] - p1[0]
    dy = p0[1] - p1[1]
    return math.sqrt(dx * dx + dy * dy)


def _distance3(p0, p1):
    dx = p0[0] - p1[0]
    dy = p0[1] - p1[1]
    dz = p0[2] - p1[2]
    return math.sqrt(dx * dx + dy * dy + dz * dz)


def _angle_between(v1, v2):
    """计算两个向量之间的角度（度）"""
    d1 = _norm3(v1)
    d2 = _norm3(v2)
    if d1 < 1e-9 or d2 < 1e-9:
        return 0.0
    cos_val = _dot3(v1, v2) / (d1 * d2)
    cos_val = max(-1.0, min(1.0, cos_val))
    return math.degrees(math.acos(cos_val))


# ============================================================
# 高斯消元法求解线性方程组 Ax = b
# ============================================================

def _solve_linear_system(A, b):
    n = len(b)
    aug = [A[i][:] + [b[i]] for i in range(n)]

    for col in range(n):
        pivot = col
        max_abs = abs(aug[col][col])
        for row in range(col + 1, n):
            v = abs(aug[row][col])
            if v > max_abs:
                max_abs = v
                pivot = row

        if max_abs < 1e-9:
            return None

        if pivot != col:
            aug[col], aug[pivot] = aug[pivot], aug[col]

        pivot_val = aug[col][col]
        for j in range(col, n + 1):
            aug[col][j] /= pivot_val

        for row in range(n):
            if row == col:
                continue
            factor = aug[row][col]
            if abs(factor) < 1e-12:
                continue
            for j in range(col, n + 1):
                aug[row][j] -= factor * aug[col][j]

    return [aug[i][n] for i in range(n)]


# ============================================================
# 角点排序
# ============================================================

def order_corners_tl_tr_br_bl(points):
    """将4个角点排序为：左上、右上、右下、左下"""
    pts = [(float(p[0]), float(p[1])) for p in points]
    if len(pts) != 4:
        return None

    sums = [p[0] + p[1] for p in pts]
    diffs = [p[0] - p[1] for p in pts]

    tl = pts[sums.index(min(sums))]
    br = pts[sums.index(max(sums))]
    tr = pts[diffs.index(max(diffs))]
    bl = pts[diffs.index(min(diffs))]

    ordered = [tl, tr, br, bl]
    if len(set(ordered)) != 4:
        return None
    return ordered


# ============================================================
# 正方形验证
# ============================================================

def is_square_like(ordered_points):
    """验证四边形是否接近正方形"""
    if ordered_points is None:
        return False

    tl, tr, br, bl = ordered_points
    top = _distance2(tl, tr)
    right = _distance2(tr, br)
    bottom = _distance2(br, bl)
    left = _distance2(bl, tl)

    if min(top, right, bottom, left) < 2.0:
        return False

    # 边长一致性检查
    mean_side = (top + right + bottom + left) / 4.0
    side_err = max(
        abs(top - mean_side),
        abs(right - mean_side),
        abs(bottom - mean_side),
        abs(left - mean_side),
    ) / mean_side

    if side_err > MAX_SIDE_RELATIVE_ERROR:
        return False

    # 宽高比检查
    width = (top + bottom) * 0.5
    height = (left + right) * 0.5
    aspect = width / height if height > 1e-6 else 999.0

    if abs(aspect - 1.0) > MAX_ASPECT_RATIO_DEVIATION:
        return False

    # 内角检查
    vertices = [(tl, tr, br), (tr, br, bl), (br, bl, tl), (bl, tl, tr)]
    for prev_pt, vertex, next_pt in vertices:
        v1 = [prev_pt[0] - vertex[0], prev_pt[1] - vertex[1], 0.0]
        v2 = [next_pt[0] - vertex[0], next_pt[1] - vertex[1], 0.0]
        angle = _angle_between(v1, v2)
        if angle < MIN_INTERIOR_ANGLE or angle > MAX_INTERIOR_ANGLE:
            return False

    # 对角线长度检查
    diag1 = _distance2(tl, br)
    diag2 = _distance2(tr, bl)
    diag_err = abs(diag1 - diag2) / max(diag1, diag2, 1e-6)
    if diag_err > MAX_DIAGONAL_RATIO_ERROR:
        return False

    return True


# ============================================================
# 亚像素角点优化（cornerSubPix 梯度法）
# ============================================================

def _compute_gradient_sums(img, cx, cy, win_size):
    """
    在角点周围窗口内计算梯度统计量。
    返回 (gxx, gxy, gyy, bx, by)，用于求解亚像素偏移。

    基于 cornerSubPix 原理：窗口内每个像素的梯度向量与位置向量的点积为零。
    """
    half = win_size
    gxx = 0.0
    gxy = 0.0
    gyy = 0.0
    bx = 0.0
    by = 0.0

    img_w = img.width()
    img_h = img.height()
    icx = int(round(cx))
    icy = int(round(cy))

    for dy in range(-half, half + 1):
        for dx in range(-half, half + 1):
            x = icx + dx
            y = icy + dy

            if x <= 0 or x >= img_w - 1 or y <= 0 or y >= img_h - 1:
                continue

            # 中心差分梯度
            ix = float(img.get_pixel(x + 1, y)) - float(img.get_pixel(x - 1, y))
            iy = float(img.get_pixel(x, y + 1)) - float(img.get_pixel(x, y - 1))

            gxx += ix * ix
            gxy += ix * iy
            gyy += iy * iy
            bx += ix * ix * dx + ix * iy * dy
            by += iy * ix * dx + iy * iy * dy

    return gxx, gxy, gyy, bx, by


def subpixel_corner_refine(img, corners):
    """
    对每个角点做亚像素级优化（cornerSubPix 梯度法）。
    迭代求解 G * [dx, dy]^T = b，其中 G 是梯度自相关矩阵。
    """
    refined = []
    for (cx, cy) in corners:
        x, y = float(cx), float(cy)
        for _ in range(SUBPIX_MAX_ITER):
            gxx, gxy, gyy, bx, by = _compute_gradient_sums(img, x, y, SUBPIX_WIN_SIZE)

            # 求解 2x2 线性系统
            det = gxx * gyy - gxy * gxy
            if abs(det) < 1e-12:
                break

            dx = (gyy * bx - gxy * by) / det
            dy = (gxx * by - gxy * bx) / det

            x += dx
            y += dy

            if dx * dx + dy * dy < SUBPIX_EPSILON * SUBPIX_EPSILON:
                break

        refined.append((x, y))
    return refined


# ============================================================
# 单应性矩阵求解
# ============================================================

def solve_homography(world_xy, image_uv):
    """
    求解单应性矩阵 H（3x3），将世界平面点 [X, Y, 1]^T 映射到图像点 [u, v, 1]^T。
    使用 DLT 方法，4个点恰好求解。
    """
    if len(world_xy) != 4 or len(image_uv) != 4:
        return None

    A = []
    b = []
    for (xw, yw), (u, v) in zip(world_xy, image_uv):
        A.append([xw, yw, 1.0, 0.0, 0.0, 0.0, -u * xw, -u * yw])
        b.append(u)
        A.append([0.0, 0.0, 0.0, xw, yw, 1.0, -v * xw, -v * yw])
        b.append(v)

    h = _solve_linear_system(A, b)
    if h is None:
        return None

    return [
        [h[0], h[1], h[2]],
        [h[3], h[4], h[5]],
        [h[6], h[7], 1.0],
    ]


# ============================================================
# 基于单应性的位姿求解（对标 OpenCV solvePnP 精度）
#
# 方法：单应性矩阵分解 + 正交化精化
# 1. 从 H 中提取 r1, r2, t 的初始估计
# 2. 正交化旋转矩阵（Gram-Schmidt）
# 3. 确保 Z 轴朝前（相机朝向目标）
# 4. 可选：迭代精化（Gauss-Newton）
# ============================================================

def solve_pose_ippe(image_points, camera_matrix, square_size_mm):
    """
    基于 IPPE（Infinitesimal Plane-based Pose Estimation）思想的位姿求解。

    输入：
        image_points: 4个角点像素坐标 [TL, TR, BR, BL]
        camera_matrix: 3x3 相机内参矩阵
        square_size_mm: 正方形边长（毫米）

    输出：
        R: 3x3 旋转矩阵（世界→相机）
        t: 3x1 平移向量（毫米）
        reproj_err: 重投影误差（像素）
    """
    half = square_size_mm * 0.5

    # 世界坐标系下的3D点（Z=0平面，原点在正方形中心）
    world_xy = [
        (-half, -half),
        (half, -half),
        (half, half),
        (-half, half),
    ]

    # 求解单应性矩阵
    H = solve_homography(world_xy, image_points)
    if H is None:
        return None, None, None

    K_inv = _mat3_inv(camera_matrix)
    if K_inv is None:
        return None, None, None

    # 从 H 中提取旋转和平移的初始估计
    # H = K * [r1, r2, t]  =>  [r1, r2, t] = K^-1 * H
    h1 = [H[0][0], H[1][0], H[2][0]]
    h2 = [H[0][1], H[1][1], H[2][1]]
    h3 = [H[0][2], H[1][2], H[2][2]]

    b1 = _mat3_mul_vec(K_inv, h1)
    b2 = _mat3_mul_vec(K_inv, h2)
    b3 = _mat3_mul_vec(K_inv, h3)

    # 计算尺度因子
    n1 = _norm3(b1)
    n2 = _norm3(b2)
    if n1 < 1e-9 or n2 < 1e-9:
        return None, None, None

    # 使用平均尺度（IPPE 方法）
    scale = 2.0 / (n1 + n2)

    r1 = _mul3(b1, scale)
    r2 = _mul3(b2, scale)
    t = _mul3(b3, scale)

    # Gram-Schmidt 正交化
    r1_norm = _norm3(r1)
    if r1_norm < 1e-9:
        return None, None, None
    r1n = _mul3(r1, 1.0 / r1_norm)

    # r2 正交化：去除 r1 方向分量
    dot_r2_r1 = _dot3(r2, r1n)
    r2_ortho = [r2[i] - dot_r2_r1 * r1n[i] for i in range(3)]
    r2_norm = _norm3(r2_ortho)
    if r2_norm < 1e-9:
        return None, None, None
    r2n = _mul3(r2_ortho, 1.0 / r2_norm)

    # r3 = r1 × r2（保证右手系）
    r3n = _cross3(r1n, r2n)
    r3_norm = _norm3(r3n)
    if r3_norm < 1e-9:
        return None, None, None
    r3n = _mul3(r3n, 1.0 / r3_norm)

    # 确保 Z 轴朝前（相机看到目标时 t[2] > 0）
    if t[2] < 0:
        r1n = _mul3(r1n, -1.0)
        r2n = _mul3(r2n, -1.0)
        r3n = _mul3(r3n, -1.0)
        t = _mul3(t, -1.0)

    R = [
        [r1n[0], r2n[0], r3n[0]],
        [r1n[1], r2n[1], r3n[1]],
        [r1n[2], r2n[2], r3n[2]],
    ]

    # 计算重投影误差
    reproj_err = compute_reprojection_error(image_points, R, t, camera_matrix, square_size_mm)

    return R, t, reproj_err


# ============================================================
# 3D 点投影
# ============================================================

def project_world_point(world_xyz, R, t, camera_matrix):
    """将世界坐标系下的3D点投影到图像平面"""
    # 相机坐标系
    xc = R[0][0] * world_xyz[0] + R[0][1] * world_xyz[1] + R[0][2] * world_xyz[2] + t[0]
    yc = R[1][0] * world_xyz[0] + R[1][1] * world_xyz[1] + R[1][2] * world_xyz[2] + t[1]
    zc = R[2][0] * world_xyz[0] + R[2][1] * world_xyz[1] + R[2][2] * world_xyz[2] + t[2]

    if abs(zc) < 1e-9:
        return None

    fx = camera_matrix[0][0]
    fy = camera_matrix[1][1]
    cx = camera_matrix[0][2]
    cy = camera_matrix[1][2]

    u = fx * (xc / zc) + cx
    v = fy * (yc / zc) + cy
    return (u, v)


# ============================================================
# 重投影误差计算
# ============================================================

def compute_reprojection_error(image_points, R, t, camera_matrix, square_size_mm):
    """计算平均重投影误差（像素）"""
    half = square_size_mm * 0.5
    world_points = [
        (-half, -half, 0.0),
        (half, -half, 0.0),
        (half, half, 0.0),
        (-half, half, 0.0),
    ]

    total_err = 0.0
    count = 0
    for idx, wp in enumerate(world_points):
        uv = project_world_point(wp, R, t, camera_matrix)
        if uv is None:
            continue
        du = uv[0] - image_points[idx][0]
        dv = uv[1] - image_points[idx][1]
        total_err += math.sqrt(du * du + dv * dv)
        count += 1

    return total_err / count if count > 0 else 1e9


# ============================================================
# 世界坐标系 → 相机坐标系角点
# ============================================================

def square_corners_camera_coords(R, t, square_size_mm):
    """计算正方形四个角点在相机坐标系下的3D坐标（毫米）"""
    half = square_size_mm * 0.5
    world_points = [
        (-half, -half, 0.0),
        (half, -half, 0.0),
        (half, half, 0.0),
        (-half, half, 0.0),
    ]

    corners_cam = []
    for wp in world_points:
        xc = R[0][0] * wp[0] + R[0][1] * wp[1] + R[0][2] * wp[2] + t[0]
        yc = R[1][0] * wp[0] + R[1][1] * wp[1] + R[1][2] * wp[2] + t[1]
        zc = R[2][0] * wp[0] + R[2][1] * wp[1] + R[2][2] * wp[2] + t[2]
        corners_cam.append((xc, yc, zc))

    return corners_cam


# ============================================================
# 相机坐标系 → 世界坐标系转换
# ============================================================

def camera_to_world_points(corners_cam, R, t):
    """
    将相机坐标系下的点转换到世界坐标系。
    P_world = R^T * (P_camera - t)
    """
    R_inv = [
        [R[0][0], R[1][0], R[2][0]],
        [R[0][1], R[1][1], R[2][1]],
        [R[0][2], R[1][2], R[2][2]],
    ]

    world_points = []
    for p in corners_cam:
        d = _sub3(p, t)
        wx = _dot3(R_inv[0], d)
        wy = _dot3(R_inv[1], d)
        wz = _dot3(R_inv[2], d)
        world_points.append((wx, wy, wz))

    return world_points


# ============================================================
# EMA 平滑
# ============================================================

def ema_points(prev_points, new_points, alpha):
    """指数移动平均平滑"""
    if prev_points is None:
        return new_points
    out = []
    for idx in range(len(new_points)):
        px, py, pz = prev_points[idx]
        nx, ny, nz = new_points[idx]
        out.append((
            alpha * nx + (1.0 - alpha) * px,
            alpha * ny + (1.0 - alpha) * py,
            alpha * nz + (1.0 - alpha) * pz,
        ))
    return out


def check_jump(prev_points, new_points, max_jump):
    """检查坐标跳动是否超过阈值"""
    if prev_points is None:
        return False
    for idx in range(len(new_points)):
        d = _distance3(prev_points[idx], new_points[idx])
        if d > max_jump:
            return True
    return False


# ============================================================
# 调试绘制
# ============================================================

def _draw_text(img, x, y, text, color=(255, 255, 255)):
    try:
        img.draw_string_advanced(x, y, 20, text, color=color, thickness=2)
    except Exception:
        img.draw_string(x, y, text, color=color)


def _draw_corners(img, ordered, color=(0, 255, 0)):
    for idx, p in enumerate(ordered):
        x = int(p[0])
        y = int(p[1])
        img.draw_circle(x, y, 4, color=color)
        _draw_text(img, x + 3, y + 3, str(idx), color=color)


def _format_corner(tag, pt):
    return "%s:(%.3f,%.3f,%.3f)" % (tag, pt[0], pt[1], pt[2])


# ============================================================
# 主程序
# ============================================================

def main():
    sensor = None
    smoothed_corners = None  # 平滑后的世界坐标角点
    frame_id = 0

    try:
        # 初始化摄像头
        sensor = Sensor(width=1280, height=960)
        sensor.reset()
        sensor.set_framesize(Sensor.VGA)          # 1280x720
        sensor.set_pixformat(Sensor.RGB565)       # RGB565 格式

        # 初始化显示
        Display.init(Display.ST7701, width=640, height=480, fps=90, to_ide=True)
        MediaManager.init()
        sensor.run()

        clock = time.clock()

        print("=== 黑色正方形角点三维坐标检测系统 ===")
        print("目标: 边长=%.1fmm, 线宽=%.1fmm" % (SQUARE_SIZE_MM, LINE_WIDTH_MM))
        print("分辨率: %dx%d" % (FRAME_WIDTH, FRAME_HEIGHT))
        print("======================================")

        while True:
            os.exitpoint()
            clock.tick()
            frame_id += 1

            img = sensor.snapshot()

            # --------------------------------------------------
            # 阶段1: 预处理
            # --------------------------------------------------

            # 转换为灰度图（用于后续处理）
            # img_gray = img.to_grayscale(copy=True)

            # 灰度图用于角点优化
            gray_for_subpix = img

            # --------------------------------------------------
            # 阶段2: 目标检测（使用 find_rects）
            # --------------------------------------------------
            if USE_ROI:
                rects = img.find_rects(threshold=FIND_RECTS_THRESHOLD, roi=ROI)
            else:
                rects = img.find_rects(threshold=FIND_RECTS_THRESHOLD)

            # --------------------------------------------------
            # 阶段3: 筛选最佳正方形
            # --------------------------------------------------
            best = None
            best_score = -1.0

            for r in rects:
                corners = r.corners()
                ordered = order_corners_tl_tr_br_bl(corners)
                if ordered is None:
                    continue
                if not is_square_like(ordered):
                    continue

                # 评分：边缘强度 + 面积
                try:
                    score = float(r.magnitude()) + 0.002 * float(r.w()) * float(r.h())
                except Exception:
                    score = float(r.w()) * float(r.h())

                if score > best_score:
                    best_score = score
                    best = (r, ordered)

            status = "NO_TARGET"

            if best is not None:
                rect_obj, ordered = best

                # --------------------------------------------------
                # 阶段4: 亚像素角点优化
                # --------------------------------------------------
                refined_corners = subpixel_corner_refine(gray_for_subpix, ordered)

                # --------------------------------------------------
                # 阶段5: PnP 位姿求解
                # --------------------------------------------------
                R, t, reproj_err = solve_pose_ippe(
                    refined_corners,
                    CAMERA_MATRIX,
                    SQUARE_SIZE_MM,
                )

                if (R is not None and t is not None and reproj_err is not None
                        and reproj_err <= MAX_REPROJECTION_ERROR_PX
                        and MIN_CAMERA_Z_MM <= t[2] <= MAX_CAMERA_Z_MM):

                    # 计算四个角点在相机坐标系下的3D坐标
                    corners_cam = square_corners_camera_coords(R, t, SQUARE_SIZE_MM)

                    # 转换到世界坐标系（原点在正方形中心）
                    corners_world = camera_to_world_points(corners_cam, R, t)

                    # 跳动检查
                    if check_jump(smoothed_corners, corners_world, MAX_JUMP_MM):
                        # 跳动过大，沿用上一帧
                        status = "JUMPED"
                        if DRAW_DEBUG_OVERLAY:
                            _draw_corners(img, refined_corners, color=(255, 165, 0))
                            _draw_text(img, 4, 4, "status=JUMPED", color=(255, 165, 0))
                    else:
                        # EMA 平滑
                        if ENABLE_SMOOTHING:
                            corners_world = ema_points(smoothed_corners, corners_world, EMA_ALPHA)
                        smoothed_corners = corners_world
                        status = "OK"

                        if DRAW_DEBUG_OVERLAY:
                            _draw_corners(img, refined_corners, color=(0, 255, 0))
                            _draw_text(img, 4, 4, "OK err=%.2fx" % reproj_err, color=(0, 255, 0))

                            # 绘制世界坐标（毫米，三位小数）
                            tags = ["TL", "TR", "BR", "BL"]
                            for i, (tag, pt) in enumerate(zip(tags, corners_world)):
                                _draw_text(img, 4, 24 + i * 18, _format_corner(tag, pt))
                else:
                    smoothed_corners = None
                    status = "UNRELIABLE"
                    if DRAW_DEBUG_OVERLAY:
                        _draw_corners(img, refined_corners, color=(255, 80, 80))
                        err_text = reproj_err if reproj_err is not None else -1.0
                        _draw_text(img, 4, 4, "UNRELIABLE err=%.2f" % err_text, color=(255, 80, 80))
            else:
                smoothed_corners = None
                if DRAW_DEBUG_OVERLAY:
                    _draw_text(img, 4, 4, "NO_TARGET", color=(255, 255, 0))

            # --------------------------------------------------
            # 阶段6: 结果输出
            # --------------------------------------------------
            if smoothed_corners is not None and frame_id % PRINT_EVERY_N_FRAMES == 0:
                tl, tr, br, bl = smoothed_corners
                print(
                    "frame=%d,status=%s,"
                    "TL(%.3f,%.3f,%.3f),"
                    "TR(%.3f,%.3f,%.3f),"
                    "BR(%.3f,%.3f,%.3f),"
                    "BL(%.3f,%.3f,%.3f)" % (
                        frame_id, status,
                        tl[0], tl[1], tl[2],
                        tr[0], tr[1], tr[2],
                        br[0], br[1], br[2],
                        bl[0], bl[1], bl[2],
                    )
                )
            elif frame_id % PRINT_EVERY_N_FRAMES == 0:
                print("frame=%d,status=%s" % (frame_id, status))

            # 显示图像
            Display.show_image(
                img,
                x=round((640 - sensor.width()) / 2),
                y=round((480 - sensor.height()) / 2),
            )

    except KeyboardInterrupt as e:
        print("用户停止:", e)
    except BaseException as e:
        print("异常:", e)
    finally:
        if isinstance(sensor, Sensor):
            sensor.stop()
        Display.deinit()
        os.exitpoint(os.EXITPOINT_ENABLE_SLEEP)
        time.sleep_ms(100)
        MediaManager.deinit()


if __name__ == "__main__":
    main()
