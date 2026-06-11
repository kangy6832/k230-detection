#####################################################################################################
# @file         main.py
# @author       正点原子团队(ALIENTEK)
# @version      V1.0
# @date         2024-09-12
# @brief        摄像头黑色方框检测与相机坐标估计
# @license      Copyright (c) 2020-2032, 广州市星翼电子科技有限公司
#####################################################################################################
# @attention
#
# 实验平台:正点原子 K230D BOX开发板
# 在线视频:www.yuanzige.com
# 技术论坛:www.openedv.com
# 公司网址:www.alientek.com
# 购买地址:openedv.taobao.com
#
#####################################################################################################

import time, os, sys, math
from media.sensor import *  # 导入sensor模块，使用摄像头相关接口
from media.display import * # 导入display模块，使用display相关接口
from media.media import *   # 导入media模块，使用meida相关接口
from machine import UART
import struct
DISPLAY_WIDTH = 640
DISPLAY_HEIGHT = 480
RECT_THRESHOLD = 8000
MIN_RECT_AREA = 2500
MIN_ASPECT_RATIO = 0.45
MAX_ASPECT_RATIO = 1.80

MARKER_SIZE_MM = 100.0
HALF_MARKER_SIZE_MM = MARKER_SIZE_MM / 2.0

# 相机标定参数：按 1280x960 标定结果缩放到 320x240 的近似值。
# 坐标系采用常见视觉坐标系：X 向右，Y 向下，Z 向前。
FX = 394.2671618128595
FY = 394.00936353218
CX = 156.7020958462694
CY = 114.7305337502674
K1 = 0.1228141066424708
K2 = -1.061945585639066
P1 = -0.001075317168450915
P2 = -0.002785704952431605
K3 = 2.211736544154165

sensor = None
uart = UART(
    UART.UART1,
    baudrate=115200,
    bits=8,
    parity=0,
    stop=1
)

def get_right_top_corner_index(corners):
    top_two = sorted(range(4), key=lambda index: (corners[index][1], corners[index][0]))[:2]
    top_two.sort(key=lambda index: (-corners[index][0], corners[index][1]))
    return top_two[0]


def transform_point_to_camera(rmat, tvec, point):
    x = rmat[0][0] * point[0] + rmat[0][1] * point[1] + rmat[0][2] * point[2] + tvec[0]
    y = rmat[1][0] * point[0] + rmat[1][1] * point[1] + rmat[1][2] * point[2] + tvec[1]
    z = rmat[2][0] * point[0] + rmat[2][1] * point[1] + rmat[2][2] * point[2] + tvec[2]
    return (x, y, z)


def sort_corners(corners):
    center_x = sum(p[0] for p in corners) / 4.0
    center_y = sum(p[1] for p in corners) / 4.0
    ordered = sorted(corners, key=lambda p: math.atan2(p[1] - center_y, p[0] - center_x))

    start_index = 0
    best_score = ordered[0][0] + ordered[0][1]
    for index in range(1, 4):
        score = ordered[index][0] + ordered[index][1]
        if score < best_score:
            best_score = score
            start_index = index

    return ordered[start_index:] + ordered[:start_index]


def is_valid_rect(rect):
    _, _, w, h = rect.rect()
    area = w * h
    if area < MIN_RECT_AREA:
        return False
    if h == 0:
        return False

    aspect_ratio = w / h
    if aspect_ratio < MIN_ASPECT_RATIO or aspect_ratio > MAX_ASPECT_RATIO:
        return False

    return True


def select_target_rect(rects):
    best_rect = None
    best_area = 0

    for rect in rects:
        if not is_valid_rect(rect):
            continue

        _, _, w, h = rect.rect()
        area = w * h
        if area > best_area:
            best_area = area
            best_rect = rect

    return best_rect, best_area


def get_marker_world_points():
    s = HALF_MARKER_SIZE_MM
    return [(-s, -s), (s, -s), (s, s), (-s, s)]


def is_camera_calibrated():
    return FX is not None and FY is not None and CX is not None and CY is not None and FX != 0 and FY != 0


def normalize_image_points(points):
    normalized = []
    for u, v in points:
        x = (float(u) - CX) / FX
        y = (float(v) - CY) / FY
        normalized.append((x, y))
    return normalized


def undistort_normalized_points(points):
    if K1 == 0.0 and K2 == 0.0 and P1 == 0.0 and P2 == 0.0 and K3 == 0.0:
        return points

    result = []
    for xd, yd in points:

        x = xd
        y = yd
        for _ in range(5):
            r2 = x * x + y * y
            radial = 1.0 + K1 * r2 + K2 * r2 * r2 + K3 * r2 * r2 * r2
            dx = 2.0 * P1 * x * y + P2 * (r2 + 2.0 * x * x)
            dy = P1 * (r2 + 2.0 * y * y) + 2.0 * P2 * x * y
            if radial == 0.0:
                break
            x = (xd - dx) / radial
            y = (yd - dy) / radial
        result.append((x, y))
    return result


def solve_linear_system(matrix, vector):
    n = len(vector)
    a = []
    for row in range(n):
        a.append(matrix[row][:] + [vector[row]])

    for col in range(n):
        pivot = col
        max_value = abs(a[col][col])
        for row in range(col + 1, n):
            value = abs(a[row][col])
            if value > max_value:
                max_value = value
                pivot = row

        if max_value < 1e-12:
            return None

        if pivot != col:
            a[col], a[pivot] = a[pivot], a[col]

        pivot_value = a[col][col]
        for item in range(col, n + 1):
            a[col][item] /= pivot_value

        for row in range(n):
            if row == col:
                continue
            factor = a[row][col]
            if factor == 0.0:
                continue
            for item in range(col, n + 1):
                a[row][item] -= factor * a[col][item]

    return [a[row][n] for row in range(n)]


def solve_homography_4pts(world_points, image_points):
    matrix = []
    vector = []

    for index in range(4):
        X, Y = world_points[index]
        x, y = image_points[index]
        matrix.append([X, Y, 1.0, 0.0, 0.0, 0.0, -x * X, -x * Y])
        vector.append(x)
        matrix.append([0.0, 0.0, 0.0, X, Y, 1.0, -y * X, -y * Y])
        vector.append(y)

    h = solve_linear_system(matrix, vector)
    if h is None:
        return None

    return ((h[0], h[1], h[2]),
            (h[3], h[4], h[5]),
            (h[6], h[7], 1.0))


def vec_norm(v):
    return math.sqrt(v[0] * v[0] + v[1] * v[1] + v[2] * v[2])


def vec_scale(v, scale):
    return (v[0] * scale, v[1] * scale, v[2] * scale)


def vec_cross(a, b):
    return (a[1] * b[2] - a[2] * b[1],
            a[2] * b[0] - a[0] * b[2],
            a[0] * b[1] - a[1] * b[0])


def pose_from_homography(H):
    h1 = (H[0][0], H[1][0], H[2][0])
    h2 = (H[0][1], H[1][1], H[2][1])
    h3 = (H[0][2], H[1][2], H[2][2])

    norm1 = vec_norm(h1)
    norm2 = vec_norm(h2)
    if norm1 < 1e-12 or norm2 < 1e-12:
        return None

    scale = 2.0 / (norm1 + norm2)
    r1 = vec_scale(h1, scale)
    r2_raw = vec_scale(h2, scale)
    tvec = vec_scale(h3, scale)

    r3 = vec_cross(r1, r2_raw)
    norm3 = vec_norm(r3)
    if norm3 < 1e-12:
        return None
    r3 = vec_scale(r3, 1.0 / norm3)
    r2 = vec_cross(r3, r1)

    rmat = ((r1[0], r2[0], r3[0]),
            (r1[1], r2[1], r3[1]),
            (r1[2], r2[2], r3[2]))

    if tvec[2] < 0.0:
        tvec = vec_scale(tvec, -1.0)
        rmat = ((-rmat[0][0], -rmat[0][1], rmat[0][2]),
                (-rmat[1][0], -rmat[1][1], rmat[1][2]),
                (-rmat[2][0], -rmat[2][1], rmat[2][2]))

    return rmat, tvec


def estimate_marker_pose(corners):
    if not is_camera_calibrated():
        return {"valid": False, "reason": "camera calibration placeholder"}

    normalized_points = normalize_image_points(corners)
    undistorted_points = undistort_normalized_points(normalized_points)
    world_points = get_marker_world_points()
    H = solve_homography_4pts(world_points, undistorted_points)
    if H is None:
        return {"valid": False, "reason": "homography solve failed"}

    pose = pose_from_homography(H)
    if pose is None:
        return {"valid": False, "reason": "pose recover failed"}

    right_top_index = get_right_top_corner_index(corners)
    rmat, tvec = pose
    corner_cam = transform_point_to_camera(rmat, tvec, (world_points[right_top_index][0], world_points[right_top_index][1], 0.0))
    return {
        "valid": True,
        "corner_index": right_top_index,
        "corner_image": corners[right_top_index],
        "corner_cam": corner_cam,
    }


def draw_quad(img, corners):
    for index in range(4):
        p1 = corners[index]
        p2 = corners[(index + 1) % 4]
        img.draw_line(p1[0], p1[1], p2[0], p2[1], color=(255, 0, 0), thickness=2)


def draw_status(img, corners, pose):
    draw_quad(img, corners)
    corner_u, corner_v = pose["corner_image"]
    img.draw_circle(corner_u, corner_v, 6, color=(0, 255, 0), fill=True)

    x, y, z = pose["corner_cam"]
    img.draw_string(2, 2, "corner_cam_mm=({:.1f}, {:.1f}, {:.1f})".format(x, y, z), color=(255, 255, 255), scale=1)



def send_target_to_stm32(pose):
    if not pose or not pose["valid"]:
        return False

    x, y, z = pose["corner_cam"]
    packet = struct.pack(
        "<ifff",
        1,
        float(x),
        float(y),
        float(z)
    )

    uart.write(packet)
    return True


def print_detection_result(pose, sent):
    if not sent:
        return

    x, y, z = pose["corner_cam"]
    print("corner_cam_mm=({:.1f}, {:.1f}, {:.1f})".format(x, y, z))


try:
    sensor = Sensor(width=320, height=240) # 构建摄像头对象
    sensor.reset() # 复位和初始化摄像头
    sensor.set_framesize(Sensor.QVGA)   # 设置帧大小QVGA(320x240)，默认通道0
    sensor.set_pixformat(Sensor.RGB565) # 设置输出图像格式，默认通道0

    # 初始化LCD显示器，同时IDE缓冲区输出图像,显示的数据来自于sensor通道0。
    Display.init(Display.ST7701, width=DISPLAY_WIDTH, height=DISPLAY_HEIGHT, to_ide=True)
    MediaManager.init() # 初始化media资源管理器
    sensor.run()        # 启动sensor
    clock = time.clock() # 构造clock对象

    while True:
        os.exitpoint() # 检测IDE中断
        clock.tick()   # 记录开始时间（ms）
        img = sensor.snapshot() # 从通道0捕获一张图
        rects = img.find_rects(threshold=RECT_THRESHOLD)
        target_rect, _ = select_target_rect(rects)
        target_pose = None

        if target_rect:
            target_corners = sort_corners(target_rect.corners())
            target_pose = estimate_marker_pose(target_corners)
            if target_pose["valid"]:
                draw_status(img, target_corners, target_pose)

        # 显示图片
        Display.show_image(img, x=round((DISPLAY_WIDTH - sensor.width()) / 2), y=round((DISPLAY_HEIGHT - sensor.height()) / 2))
        sent = send_target_to_stm32(target_pose)
        print_detection_result(target_pose, sent)

# IDE中断释放资源代码
except KeyboardInterrupt as e:
    print("user stop: ", e)
except BaseException as e:
    print(f"Exception {e}")
finally:
    # sensor stop run
    if isinstance(sensor, Sensor):
        sensor.stop()
    # deinit display
    Display.deinit()
    os.exitpoint(os.EXITPOINT_ENABLE_SLEEP)
    time.sleep_ms(100)
    # release media buffer
    MediaManager.deinit()
