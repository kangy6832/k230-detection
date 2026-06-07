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
LINE_WIDTH_MM = 3.0      # 线宽 3mm

# 摄像头采集设置（VGA）
FRAME_WIDTH = 640
FRAME_HEIGHT = 480

# ROI（感兴趣区域），全图检测
USE_ROI = False
ROI = (0, 0, FRAME_WIDTH, FRAME_HEIGHT)

# 轮廓检测参数
FIND_RECTS_THRESHOLD = 7000

# 正方形验证参数（简化：去掉内角检查）
MAX_ASPECT_RATIO_DEVIATION = 0.25
MAX_SIDE_RELATIVE_ERROR = 0.18
MAX_DIAGONAL_RATIO_ERROR = 0.10

# 相机内参矩阵（VGA 分辨率，需实际标定）
# VGA: fx,fy 约为 HD 的一半；cx,cy 为图像中心
CAMERA_MATRIX = [
    [400.0, 0.0, 320.0],
    [0.0, 400.0, 240.0],
    [0.0, 0.0, 1.0],
]

# 位姿验证参数
MAX_REPROJECTION_ERROR_PX = 5.0   # 放宽容差（无亚像素优化）
MIN_CAMERA_Z_MM = 80.0
MAX_CAMERA_Z_MM = 300.0

# 平滑参数
ENABLE_SMOOTHING = True
EMA_ALPHA = 0.45
MAX_JUMP_MM = 50.0

# 调试参数
DRAW_DEBUG_OVERLAY = True
PRINT_EVERY_N_FRAMES = 5


# ============================================================
# 最小化几何工具（只保留必需）
# ============================================================

def _dot3(a, b):
    return a[0] * b[0] + a[1] * b[1] + a[2] * b[2]


def _norm3(v):
    return math.sqrt(v[0] * v[0] + v[1] * v[1] + v[2] * v[2])


def _cross3(a, b):
    return [
        a[1] * b[2] - a[2] * b[1],
        a[2] * b[0] - a[0] * b[2],
        a[0] * b[1] - a[1] * b[0],
    ]


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


# ============================================================
# 高斯消元法（保留，PnP 必需）
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
# 正方形验证（简化：去掉昂贵的 math.acos 内角检查）
# ============================================================

def is_square_like(ordered_points):
    if ordered_points is None:
        return False
    tl, tr, br, bl = ordered_points
    top = _distance2(tl, tr)
    right = _distance2(tr, br)
    bottom = _distance2(br, bl)
    left = _distance2(bl, tl)

    if min(top, right, bottom, left) < 2.0:
        return False

    # 边长一致性
    mean_side = (top + right + bottom + left) * 0.25
    if max(abs(top - mean_side), abs(right - mean_side),
           abs(bottom - mean_side), abs(left - mean_side)) / mean_side > MAX_SIDE_RELATIVE_ERROR:
        return False

    # 宽高比
    width = (top + bottom) * 0.5
    height = (left + right) * 0.5
    if height < 1e-6 or abs(width / height - 1.0) > MAX_ASPECT_RATIO_DEVIATION:
        return False

    # 对角线一致性（替代内角检查）
    diag1 = _distance2(tl, br)
    diag2 = _distance2(tr, bl)
    if abs(diag1 - diag2) / max(diag1, diag2, 1e-6) > MAX_DIAGONAL_RATIO_ERROR:
        return False

    return True


# ============================================================
# 单应性矩阵求解（保留，PnP 必需）
# ============================================================

def solve_homography(world_xy, image_uv):
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
# 位姿求解（保留核心，利用 Z=0 简化投影）
# ============================================================

def solve_pose_ippe(image_points, camera_matrix, square_size_mm):
    half = square_size_mm * 0.5
    world_xy = [(-half, -half), (half, -half), (half, half), (-half, half)]

    H = solve_homography(world_xy, image_points)
    if H is None:
        return None, None, None

    K_inv = _mat3_inv(camera_matrix)
    if K_inv is None:
        return None, None, None

    h1 = [H[0][0], H[1][0], H[2][0]]
    h2 = [H[0][1], H[1][1], H[2][1]]
    h3 = [H[0][2], H[1][2], H[2][2]]

    b1 = _mat3_mul_vec(K_inv, h1)
    b2 = _mat3_mul_vec(K_inv, h2)
    b3 = _mat3_mul_vec(K_inv, h3)

    n1 = _norm3(b1)
    n2 = _norm3(b2)
    if n1 < 1e-9 or n2 < 1e-9:
        return None, None, None

    scale = 2.0 / (n1 + n2)
    r1 = _mul3(b1, scale)
    r2 = _mul3(b2, scale)
    t = _mul3(b3, scale)

    # Gram-Schmidt 正交化
    r1_norm = _norm3(r1)
    if r1_norm < 1e-9:
        return None, None, None
    r1n = _mul3(r1, 1.0 / r1_norm)

    dot_r2_r1 = _dot3(r2, r1n)
    r2_ortho = [r2[i] - dot_r2_r1 * r1n[i] for i in range(3)]
    r2_norm = _norm3(r2_ortho)
    if r2_norm < 1e-9:
        return None, None, None
    r2n = _mul3(r2_ortho, 1.0 / r2_norm)

    r3n = _cross3(r1n, r2n)
    r3_norm = _norm3(r3n)
    if r3_norm < 1e-9:
        return None, None, None
    r3n = _mul3(r3n, 1.0 / r3_norm)

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

    # 利用 Z=0 简化重投影误差计算
    fx = camera_matrix[0][0]
    fy = camera_matrix[1][1]
    cx = camera_matrix[0][2]
    cy = camera_matrix[1][2]

    total_err = 0.0
    world_pts = [(-half, -half), (half, -half), (half, half), (-half, half)]
    for idx, (xw, yw) in enumerate(world_pts):
        # xc = R[0][0]*xw + R[0][1]*yw + t[0]  (Z=0 项消失)
        xc = R[0][0] * xw + R[0][1] * yw + t[0]
        yc = R[1][0] * xw + R[1][1] * yw + t[1]
        zc = R[2][0] * xw + R[2][1] * yw + t[2]
        if abs(zc) < 1e-9:
            return None, None, None
        u = fx * xc / zc + cx
        v = fy * yc / zc + cy
        du = u - image_points[idx][0]
        dv = v - image_points[idx][1]
        total_err += math.sqrt(du * du + dv * dv)

    reproj_err = total_err * 0.25  # 除以4
    return R, t, reproj_err


# ============================================================
# 相机坐标系下的角点3D坐标（利用 Z=0 简化）
# ============================================================

def square_corners_camera_coords(R, t, square_size_mm):
    half = square_size_mm * 0.5
    world_pts = [(-half, -half), (half, -half), (half, half), (-half, half)]
    corners_cam = []
    for xw, yw in world_pts:
        xc = R[0][0] * xw + R[0][1] * yw + t[0]
        yc = R[1][0] * xw + R[1][1] * yw + t[1]
        zc = R[2][0] * xw + R[2][1] * yw + t[2]
        corners_cam.append((xc, yc, zc))
    return corners_cam


# ============================================================
# EMA 平滑
# ============================================================

def ema_points(prev_points, new_points, alpha):
    if prev_points is None:
        return new_points
    out = []
    for i in range(len(new_points)):
        px, py, pz = prev_points[i]
        nx, ny, nz = new_points[i]
        out.append((
            alpha * nx + (1.0 - alpha) * px,
            alpha * ny + (1.0 - alpha) * py,
            alpha * nz + (1.0 - alpha) * pz,
        ))
    return out


def check_jump(prev_points, new_points, max_jump):
    if prev_points is None:
        return False
    for i in range(len(new_points)):
        d = _distance3(prev_points[i], new_points[i])
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
    return "%s:(%.1f,%.1f,%.1f)" % (tag, pt[0], pt[1], pt[2])


# ============================================================
# 主程序
# ============================================================

def main():
    sensor = None
    smoothed_corners = None  # 平滑后的相机坐标角点
    frame_id = 0

    try:
        sensor = Sensor(width=1280, height=960)
        sensor.reset()
        sensor.set_framesize(Sensor.VGA)
        sensor.set_pixformat(Sensor.RGB565)

        Display.init(Display.ST7701, width=640, height=480, fps=90, to_ide=True)
        MediaManager.init()
        sensor.run()

        clock = time.clock()

        print("=== 黑色正方形角点三维坐标检测系统 ===")
        print("目标: 边长=%.1fmm" % SQUARE_SIZE_MM)
        print("分辨率: %dx%d" % (FRAME_WIDTH, FRAME_HEIGHT))
        print("======================================")

        while True:
            os.exitpoint()
            clock.tick()
            frame_id += 1

            img = sensor.snapshot()

            # 目标检测
            if USE_ROI:
                rects = img.find_rects(threshold=FIND_RECTS_THRESHOLD, roi=ROI)
            else:
                rects = img.find_rects(threshold=FIND_RECTS_THRESHOLD)

            # 筛选最佳正方形
            best = None
            best_score = -1.0

            for r in rects:
                corners = r.corners()
                ordered = order_corners_tl_tr_br_bl(corners)
                if ordered is None:
                    continue
                if not is_square_like(ordered):
                    continue
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

                # 直接用 find_rects 的角点（不再做亚像素优化）
                refined_corners = ordered

                # PnP 位姿求解
                R, t, reproj_err = solve_pose_ippe(
                    refined_corners,
                    CAMERA_MATRIX,
                    SQUARE_SIZE_MM,
                )

                if (R is not None and t is not None and reproj_err is not None
                        and reproj_err <= MAX_REPROJECTION_ERROR_PX
                        and MIN_CAMERA_Z_MM <= t[2] <= MAX_CAMERA_Z_MM):

                    # 角点在相机坐标系下的3D坐标
                    corners_cam = square_corners_camera_coords(R, t, SQUARE_SIZE_MM)

                    # 跳动检查
                    if check_jump(smoothed_corners, corners_cam, MAX_JUMP_MM):
                        status = "JUMPED"
                        if DRAW_DEBUG_OVERLAY:
                            _draw_corners(img, refined_corners, color=(255, 165, 0))
                            _draw_text(img, 4, 4, "JUMPED", color=(255, 165, 0))
                    else:
                        if ENABLE_SMOOTHING:
                            corners_cam = ema_points(smoothed_corners, corners_cam, EMA_ALPHA)
                        smoothed_corners = corners_cam
                        status = "OK"

                        if DRAW_DEBUG_OVERLAY:
                            _draw_corners(img, refined_corners, color=(0, 255, 0))
                            _draw_text(img, 4, 4, "OK err=%.2f" % reproj_err, color=(0, 255, 0))
                            tags = ["TL", "TR", "BR", "BL"]
                            for i, (tag, pt) in enumerate(zip(tags, corners_cam)):
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

            # 结果输出
            if smoothed_corners is not None and frame_id % PRINT_EVERY_N_FRAMES == 0:
                tl, tr, br, bl = smoothed_corners
                print(
                    "frame=%d,%s,"
                    "TL(%.1f,%.1f,%.1f),"
                    "TR(%.1f,%.1f,%.1f),"
                    "BR(%.1f,%.1f,%.1f),"
                    "BL(%.1f,%.1f,%.1f)" % (
                        frame_id, status,
                        tl[0], tl[1], tl[2],
                        tr[0], tr[1], tr[2],
                        br[0], br[1], br[2],
                        bl[0], bl[1], bl[2],
                    )
                )
            elif frame_id % PRINT_EVERY_N_FRAMES == 0:
                print("frame=%d,%s" % (frame_id, status))

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
