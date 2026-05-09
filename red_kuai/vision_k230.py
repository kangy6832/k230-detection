"""
K230 / CanMV MicroPython 版红色箱子识别与位姿估计。

说明：
- 这个版本把原 OpenCV/C++ 方案迁移到 MicroPython 生态，保留核心流程：
  1. 红色区域检测
  2. 四角点排序
  3. 角点平滑
  4. 平面位姿估计
  5. 叠加显示
- K230 上通常没有直接可用的 ROS2 / TF2，因此这里改为：
  - 在板端绘制结果
  - 通过串口/控制台输出位姿
  - 如果你需要上位机 TF 广播，可以在 PC 端读取串口再发布
- 下面的相机内参 K 需要替换成你 K230 实际相机的标定结果。
"""

import math

try:
    import sensor
    import image
    import time
except ImportError as exc:
    raise ImportError("This script expects a CanMV/OpenMV-style MicroPython firmware.") from exc

try:
    import lcd
    HAS_DISPLAY = True
    DISPLAY_KIND = "lcd"
except ImportError:
    try:
        import display
        HAS_DISPLAY = True
        DISPLAY_KIND = "display"
    except ImportError:
        HAS_DISPLAY = False
        DISPLAY_KIND = "none"


# ================================================================
#  配置区
# ================================================================

BOX_MM = 350.0
HALF = BOX_MM / 2.0
MIN_AREA_PIXELS = 3000
SMOOTH_N = 6

# 物体顶面四角点，单位 mm，顺序：[左上, 右上, 右下, 左下]
OBJ_PTS = [
    (-HALF, -HALF),
    ( HALF, -HALF),
    ( HALF,  HALF),
    (-HALF,  HALF),
]

# 下面是示例相机内参，仅供迁移参考。
# K230 上请务必替换为你的实际标定结果，否则位姿会不准。
K = [
    [786.19375828781722, 0.0,                  668.98017421012958],
    [0.0,                 791.8946129798486,    373.97215705020159],
    [0.0,                 0.0,                  1.0],
]

# 这个版本的姿态求解直接基于平面单应性，不显式使用畸变参数。
# 如果你后续要提高精度，建议在 K230 上重新标定后再做去畸变或在上位机端修正。

# K230/CanMV 上常见的红色阈值通常需要按场景微调。
# 如果你的固件 find_blobs 使用的是 LAB 阈值，这里可以直接改成你现场可用的阈值。
RED_THRESHOLDS = [
    (20, 100, 20, 127, 0, 127),
    (0, 70, 20, 127, 0, 127),
]


# ================================================================
#  数学工具
# ================================================================

def dot(a, b):
    return a[0] * b[0] + a[1] * b[1] + a[2] * b[2]


def cross(a, b):
    return [
        a[1] * b[2] - a[2] * b[1],
        a[2] * b[0] - a[0] * b[2],
        a[0] * b[1] - a[1] * b[0],
    ]


def norm3(v):
    return math.sqrt(dot(v, v))


def normalize(v):
    n = norm3(v)
    if n < 1e-12:
        return [0.0, 0.0, 0.0]
    return [v[0] / n, v[1] / n, v[2] / n]


def mat3_mul(a, b):
    return [
        [
            a[0][0] * b[0][0] + a[0][1] * b[1][0] + a[0][2] * b[2][0],
            a[0][0] * b[0][1] + a[0][1] * b[1][1] + a[0][2] * b[2][1],
            a[0][0] * b[0][2] + a[0][1] * b[1][2] + a[0][2] * b[2][2],
        ],
        [
            a[1][0] * b[0][0] + a[1][1] * b[1][0] + a[1][2] * b[2][0],
            a[1][0] * b[0][1] + a[1][1] * b[1][1] + a[1][2] * b[2][1],
            a[1][0] * b[0][2] + a[1][1] * b[1][2] + a[1][2] * b[2][2],
        ],
        [
            a[2][0] * b[0][0] + a[2][1] * b[1][0] + a[2][2] * b[2][0],
            a[2][0] * b[0][1] + a[2][1] * b[1][1] + a[2][2] * b[2][1],
            a[2][0] * b[0][2] + a[2][1] * b[1][2] + a[2][2] * b[2][2],
        ],
    ]


def mat3_vec(a, v):
    return [
        a[0][0] * v[0] + a[0][1] * v[1] + a[0][2] * v[2],
        a[1][0] * v[0] + a[1][1] * v[1] + a[1][2] * v[2],
        a[2][0] * v[0] + a[2][1] * v[1] + a[2][2] * v[2],
    ]


def invert_3x3(m):
    a = m[0][0]
    b = m[0][1]
    c = m[0][2]
    d = m[1][0]
    e = m[1][1]
    f = m[1][2]
    g = m[2][0]
    h = m[2][1]
    i = m[2][2]

    det = (
        a * (e * i - f * h)
        - b * (d * i - f * g)
        + c * (d * h - e * g)
    )
    if abs(det) < 1e-12:
        raise ValueError("Singular matrix")

    inv_det = 1.0 / det
    return [
        [(e * i - f * h) * inv_det, (c * h - b * i) * inv_det, (b * f - c * e) * inv_det],
        [(f * g - d * i) * inv_det, (a * i - c * g) * inv_det, (c * d - a * f) * inv_det],
        [(d * h - e * g) * inv_det, (b * g - a * h) * inv_det, (a * e - b * d) * inv_det],
    ]


K_INV = invert_3x3(K)


def solve_linear_system(a, b):
    n = len(b)
    a = [row[:] for row in a]
    b = b[:]

    for col in range(n):
        pivot = col
        pivot_value = abs(a[col][col])
        for row in range(col + 1, n):
            value = abs(a[row][col])
            if value > pivot_value:
                pivot = row
                pivot_value = value
        if pivot_value < 1e-12:
            return None

        if pivot != col:
            a[col], a[pivot] = a[pivot], a[col]
            b[col], b[pivot] = b[pivot], b[col]

        div = a[col][col]
        inv_div = 1.0 / div
        for j in range(col, n):
            a[col][j] *= inv_div
        b[col] *= inv_div

        for row in range(n):
            if row == col:
                continue
            factor = a[row][col]
            if abs(factor) < 1e-12:
                continue
            for j in range(col, n):
                a[row][j] -= factor * a[col][j]
            b[row] -= factor * b[col]

    return b


def sort_corners(points):
    points = list(points)
    scores_s = [p[0] + p[1] for p in points]
    scores_d = [p[0] - p[1] for p in points]

    tl = min(range(4), key=lambda i: scores_s[i])
    br = max(range(4), key=lambda i: scores_s[i])
    tr = max(range(4), key=lambda i: scores_d[i])
    bl = min(range(4), key=lambda i: scores_d[i])

    return [points[tl], points[tr], points[br], points[bl]]


def euler_from_rmat(r):
    sy = math.sqrt(r[0][0] * r[0][0] + r[1][0] * r[1][0])
    singular = sy < 1e-6

    if singular:
        roll = math.atan2(-r[1][2], r[1][1])
        yaw = 0.0
    else:
        roll = math.atan2(r[2][1], r[2][2])
        yaw = math.atan2(r[1][0], r[0][0])

    pitch = math.atan2(-r[2][0], sy)
    return [math.degrees(roll), math.degrees(pitch), math.degrees(yaw)]


def project_point(pt3, rmat, tvec):
    x = rmat[0][0] * pt3[0] + rmat[0][1] * pt3[1] + rmat[0][2] * pt3[2] + tvec[0]
    y = rmat[1][0] * pt3[0] + rmat[1][1] * pt3[1] + rmat[1][2] * pt3[2] + tvec[1]
    z = rmat[2][0] * pt3[0] + rmat[2][1] * pt3[1] + rmat[2][2] * pt3[2] + tvec[2]
    if abs(z) < 1e-12:
        z = 1e-12
    u = (K[0][0] * x + K[0][2] * z) / z
    v = (K[1][1] * y + K[1][2] * z) / z
    return (int(u), int(v))


def solve_planar_pose(obj_pts, img_pts):
    # 单应矩阵 H，设 h33 = 1，未知数为 8 个。
    a = []
    b = []
    for idx in range(4):
        X, Y = obj_pts[idx]
        u, v = img_pts[idx]
        a.append([X, Y, 1.0, 0.0, 0.0, 0.0, -u * X, -u * Y])
        b.append(u)
        a.append([0.0, 0.0, 0.0, X, Y, 1.0, -v * X, -v * Y])
        b.append(v)

    h = solve_linear_system(a, b)
    if h is None:
        return None

    hmat = [
        [h[0], h[1], h[2]],
        [h[3], h[4], h[5]],
        [h[6], h[7], 1.0],
    ]

    bmat = mat3_mul(K_INV, hmat)
    b1 = [bmat[0][0], bmat[1][0], bmat[2][0]]
    b2 = [bmat[0][1], bmat[1][1], bmat[2][1]]
    b3 = [bmat[0][2], bmat[1][2], bmat[2][2]]

    scale = 1.0 / max(norm3(b1), 1e-12)
    r1 = [x * scale for x in b1]
    r2 = [x * scale for x in b2]
    tvec = [x * scale for x in b3]

    # 正交化，减少数值误差导致的歪斜
    r1 = normalize(r1)
    r2 = [r2[i] - dot(r1, r2) * r1[i] for i in range(3)]
    r2 = normalize(r2)
    r3 = normalize(cross(r1, r2))
    r2 = normalize(cross(r3, r1))

    rmat = [
        [r1[0], r2[0], r3[0]],
        [r1[1], r2[1], r3[1]],
        [r1[2], r2[2], r3[2]],
    ]

    det_r = (
        rmat[0][0] * (rmat[1][1] * rmat[2][2] - rmat[1][2] * rmat[2][1])
        - rmat[0][1] * (rmat[1][0] * rmat[2][2] - rmat[1][2] * rmat[2][0])
        + rmat[0][2] * (rmat[1][0] * rmat[2][1] - rmat[1][1] * rmat[2][0])
    )
    if det_r < 0:
        rmat = [[-v for v in row] for row in rmat]
        tvec = [-v for v in tvec]

    if tvec[2] < 0:
        rmat = [[-v for v in row] for row in rmat]
        tvec = [-v for v in tvec]

    return rmat, tvec


# ================================================================
#  卡尔曼滤波
# ================================================================

class CornerKF:
    def __init__(self):
        self.initialized = False
        self.x = 0.0
        self.y = 0.0
        self.vx = 0.0
        self.vy = 0.0
        self.px = 1.0
        self.py = 1.0
        self.q = 0.05
        self.r = 0.8

    def init(self, pt):
        self.x = float(pt[0])
        self.y = float(pt[1])
        self.vx = 0.0
        self.vy = 0.0
        self.px = 1.0
        self.py = 1.0
        self.initialized = True

    def update(self, measured):
        if not self.initialized:
            self.init(measured)
            return (float(measured[0]), float(measured[1]))

        self.x += self.vx
        self.y += self.vy

        self.px += self.q
        self.py += self.q

        kx = self.px / (self.px + self.r)
        ky = self.py / (self.py + self.r)

        mx = float(measured[0])
        my = float(measured[1])

        self.x = self.x + kx * (mx - self.x)
        self.y = self.y + ky * (my - self.y)
        self.vx = self.vx + 0.15 * (mx - self.x)
        self.vy = self.vy + 0.15 * (my - self.y)

        self.px = (1.0 - kx) * self.px
        self.py = (1.0 - ky) * self.py

        return (self.x, self.y)

    def predict_only(self):
        if not self.initialized:
            return (0.0, 0.0)
        self.x += self.vx
        self.y += self.vy
        self.px += self.q
        self.py += self.q
        return (self.x, self.y)


class PoseSmootherVec3:
    def __init__(self, n):
        self.max_n = n
        self.buf = []

    def push(self, vec3):
        self.buf.append(vec3)
        if len(self.buf) > self.max_n:
            self.buf.pop(0)

        sx = 0.0
        sy = 0.0
        sz = 0.0
        for v in self.buf:
            sx += v[0]
            sy += v[1]
            sz += v[2]
        n = float(len(self.buf))
        return (sx / n, sy / n, sz / n)


# ================================================================
#  检测与绘制
# ================================================================

def draw_label(img, text, x, y, color=(255, 255, 0), scale=2, thickness=1):
    # MicroPython 图像接口通常没有 OpenCV 那种描边文本，这里用偏移两次模拟高可读性。
    img.draw_string(x + 1, y + 1, text, color=(0, 0, 0), scale=scale, thickness=thickness + 1)
    img.draw_string(x, y, text, color=color, scale=scale, thickness=thickness)


def detect_box_corners(img):
    blobs = img.find_blobs(
        RED_THRESHOLDS,
        pixels_threshold=MIN_AREA_PIXELS,
        area_threshold=MIN_AREA_PIXELS,
        merge=True,
        margin=8,
    )

    if not blobs:
        return False, [], None

    blob = max(blobs, key=lambda b: b.pixels())
    roi = blob.rect()

    corners = None
    rects = []
    if hasattr(img, "find_rects"):
        try:
            rects = img.find_rects(roi=roi, threshold=15000)
        except TypeError:
            rects = img.find_rects(threshold=15000)

    if rects:
        best = max(rects, key=lambda r: r.magnitude())
        try:
            pts = best.corners()
            if len(pts) == 4:
                corners = [(pts[i][0], pts[i][1]) for i in range(4)]
        except Exception:
            corners = None

    if corners is None:
        x, y, w, h = roi
        corners = [
            (x, y),
            (x + w, y),
            (x + w, y + h),
            (x, y + h),
        ]

    corners = sort_corners(corners)
    return True, corners, blob


def display_frame(img):
    if not HAS_DISPLAY:
        return
    if DISPLAY_KIND == "lcd":
        lcd.display(img)
    elif DISPLAY_KIND == "display":
        display.show(img)


def draw_axes(img, rmat, tvec, origin_pt):
    origin = project_point(origin_pt, rmat, tvec)
    x_end = project_point((100.0, 0.0, 0.0), rmat, tvec)
    y_end = project_point((0.0, 100.0, 0.0), rmat, tvec)
    z_end = project_point((0.0, 0.0, 100.0), rmat, tvec)

    img.draw_arrow(origin[0], origin[1], x_end[0], x_end[1], color=(255, 0, 0), thickness=2)
    img.draw_arrow(origin[0], origin[1], y_end[0], y_end[1], color=(0, 255, 0), thickness=2)
    img.draw_arrow(origin[0], origin[1], z_end[0], z_end[1], color=(0, 0, 255), thickness=2)

    draw_label(img, "X", x_end[0] + 4, x_end[1], color=(255, 0, 0), scale=2)
    draw_label(img, "Y", y_end[0] + 4, y_end[1], color=(0, 255, 0), scale=2)
    draw_label(img, "Z", z_end[0] + 4, z_end[1], color=(0, 128, 255), scale=2)


# ================================================================
#  主程序
# ================================================================

def main():
    sensor.reset()
    sensor.set_pixformat(sensor.RGB565)
    sensor.set_framesize(sensor.VGA)
    sensor.skip_frames(time=2000)
    sensor.set_auto_gain(False)
    sensor.set_auto_whitebal(False)

    # 如果你的画面上下左右方向不对，可以按需打开下面两行。
    # sensor.set_hmirror(True)
    # sensor.set_vflip(True)

    if HAS_DISPLAY:
        try:
            if DISPLAY_KIND == "lcd":
                lcd.init()
            elif DISPLAY_KIND == "display":
                display.init()
        except Exception:
            pass

    kfs = [CornerKF(), CornerKF(), CornerKF(), CornerKF()]
    tvec_smoother = PoseSmootherVec3(SMOOTH_N)

    clock = time.clock()
    while True:
        clock.tick()
        img = sensor.snapshot()
        show = img.copy()

        detected, raw_corners, blob = detect_box_corners(img)
        smooth_corners = []
        for i in range(4):
            if detected:
                smooth_corners.append(kfs[i].update(raw_corners[i]))
            else:
                smooth_corners.append(kfs[i].predict_only())

        any_inited = any(kf.initialized for kf in kfs)
        rmat = None
        tvec = None
        euler = None
        dist = None

        if any_inited:
            for i in range(4):
                a = smooth_corners[i]
                b = smooth_corners[(i + 1) % 4]
                show.draw_line(int(a[0]), int(a[1]), int(b[0]), int(b[1]), color=(0, 220, 0), thickness=2)
                show.draw_circle(int(a[0]), int(a[1]), 4, color=(0, 0, 255), fill=True)
                draw_label(show, str(i), int(a[0]) + 5, int(a[1]) - 5, color=(255, 255, 0), scale=2)

            pose = solve_planar_pose(OBJ_PTS, smooth_corners)
            if pose is not None:
                rmat, tvec = pose
                tvec_s = tvec_smoother.push((tvec[0], tvec[1], tvec[2]))
                tvec = [tvec_s[0], tvec_s[1], tvec_s[2]]
                dist = math.sqrt(tvec[0] * tvec[0] + tvec[1] * tvec[1] + tvec[2] * tvec[2])
                euler = euler_from_rmat(rmat)

                draw_axes(show, rmat, tvec, (0.0, 0.0, 0.0))

                # 中心点十字
                center = project_point((0.0, 0.0, 0.0), rmat, tvec)
                show.draw_cross(center[0], center[1], color=(255, 255, 0), size=12, thickness=2)

                # 左上角信息面板
                show.draw_rectangle(8, 8, 360, 145, color=(0, 0, 0), fill=True)
                draw_label(show, "Distance: %.1f mm" % dist, 16, 16, color=(0, 255, 100), scale=2)
                draw_label(show, "X=%.1f Y=%.1f Z=%.1f mm" % (tvec[0], tvec[1], tvec[2]), 16, 42, color=(0, 220, 255), scale=2)
                draw_label(show, "Roll=%.1f Pitch=%.1f Yaw=%.1f" % (euler[0], euler[1], euler[2]), 16, 68, color=(255, 200, 0), scale=2)
                draw_label(show, "Detect: %s" % ("OK" if detected else "LOST - KF predict"), 16, 94, color=(0, 255, 0) if detected else (0, 128, 255), scale=2)
                draw_label(show, "FPS: %.1f" % clock.fps(), 16, 120, color=(255, 255, 255), scale=2)

                print("Dist=%.1fmm  XYZ=[%.1f, %.1f, %.1f]mm  RPY=[%.1f, %.1f, %.1f]deg" % (
                    dist, tvec[0], tvec[1], tvec[2], euler[0], euler[1], euler[2]
                ))

        if blob is not None:
            x, y, w, h = blob.rect()
            show.draw_rectangle(x, y, w, h, color=(255, 128, 0), thickness=2)
            draw_label(show, "RedBlob", x, max(0, y - 20), color=(255, 128, 0), scale=2)

        show.draw_string(8, show.height() - 20, "Press Ctrl+C to stop", color=(255, 255, 255), scale=1, thickness=1)
        display_frame(show)


try:
    main()
except KeyboardInterrupt:
    pass