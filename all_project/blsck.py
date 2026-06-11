#####################################################################################################
# @file         main.py
# @author       正点原子团队(ALIENTEK)
# @version      V2.0
# @date         2026-06-05
# @brief        黑色灰度循线 实验 - 机械臂末端寻迹（支持90°拐角检测与圆角过渡）
# @license      Copyright (c) 2020-2032, 广州市星翼电子科技有限公司
#####################################################################################################
# @attention
#
# 实验平台:正点原子 K230D BOX开发板
# 在线视频:www.yuanzige.com
# 技术论坛:www.openedv.com
# 公司网址:www.alientek.com
# 购买地址:openedv.taobao.com                img.draw_cross(largest_blob.cx(), largest_blob.cy())

#
# V2.0 改动说明：
#   - 适用于机械臂末端寻迹，摄像头不旋转
#   - 三状态机：FOLLOWING（循线）→ CORNER（拐角过渡）→ LOST（丢失）
#   - 拐角处使用 smoothstep 插值实现圆角轨迹过渡
#   - 所有控制量通过终端打印输出
#
#####################################################################################################

import time, math, os, gc, struct
from media.sensor import *  # 导入sensor模块，使用摄像头相关接口
from media.display import * # 导入display模块，使用display相关接口
from media.media import *   # 导入media模块，使用media相关接口
from machine import UART, FPIOA

# ===================== 可调参数 =====================

# 灰度阈值：跟踪黑线。如需跟踪白线，改为 [(128, 255)]
GRAYSCALE_THRESHOLD = [(0, 64)]

# ---------- 运动参数 ----------
FOLLOW_SPEED    = 0.01     # 正常循线时的前进速度（沿运动方向的主速度）
CORNER_SPEED    = 0.03     # 拐角过渡时新方向的速度（越大拐角走得越快，可配置接口）
CENTERING_GAIN  = 0.00005  # 对中修正增益（质心偏移量 × 增益 = 修正速度）
CENTERING_FILTER_ALPHA = 0.3  # 质心低通滤波系数 (0~1)，越小越平滑但响应越慢

# ---------- 拐角参数 ----------
CORNER_DURATION  = 5      # 拐角过渡帧数（越大圆角越平缓，越小越急促，可配置接口）
CORNER_DEBOUNCE  = 5       # 连续检测到拐角多少帧才确认（防误触，越大越稳定但越迟钝）
CORNER_COOLDOWN  = 30      # 拐角完成后的冷却帧数（期间禁止检测新拐角，防止旧线段误触发方向反转）

# ---------- 丢失参数 ----------
LOST_TOLERANCE = 15        # 连续多少帧检测不到任何色块则判定为线条丢失

# ---------- 初始运动方向 ----------
# DIR_UP=0(-y), DIR_DOWN=1(+y), DIR_LEFT=2(-x), DIR_RIGHT=3(+x)
INIT_DIRECTION = 1         # 初始方向，默认向下（+y）

# ===================== 方向常量 =====================
DIR_UP    = 0   # 机械臂沿 -y 方向移动（画面中向上）
DIR_DOWN  = 1   # 机械臂沿 +y 方向移动（画面中向下）
DIR_LEFT  = 2   # 机械臂沿 -x 方向移动（画面中向左）
DIR_RIGHT = 3   # 机械臂沿 +x 方向移动（画面中向右）

# 方向名称映射，用于显示和调试
DIR_NAMES = {DIR_UP: "UP", DIR_DOWN: "DOWN", DIR_LEFT: "LEFT", DIR_RIGHT: "RIGHT"}

# ===================== 状态常量 =====================
STATE_FOLLOWING = 0   # 正常循线状态：沿当前运动方向前进，质心对中修正
STATE_CORNER    = 1   # 拐角过渡状态：旧方向速度渐减，新方向速度渐增（圆角轨迹）
STATE_LOST      = 2   # 丢失线条状态：停止运动，等待重新检测到线条

# 状态名称映射，用于显示和调试
STATE_NAMES = {STATE_FOLLOWING: "FOLLOW", STATE_CORNER: "CORNER", STATE_LOST: "LOST"}

# ===================== ROI 区域定义 =====================
# 采样图像 VGA 640×480，17个ROI按三行五列布局
# 每个元组格式：(x, y, w, h, weight)，weight 用于加权质心计算
# 布局示意：
# 640×480 图像，17个ROI按实际坐标分布

#               0            213               426              640
#          ┌────────────────┬─────────────────┬────────────────┐
#    0     │                │       1         │                │
#          │                │─────────────────│                │
#   40     │       0        │       2         │       4        │
#          │                │─────────────────│                │
#   80     │                │       3         │                │
#          │                │                 │                │
#  160     ├───┬────┬───────┼─────────────────┼───────┬────┬───┤
#          │ 5 │ 6  │   7   │                 │   9   │ 10 │11 │
#          │   │    │       │       8         │       │    │   │
#  320     ├───┴────┴───────┼─────────────────┼───────┴────┴───┤
#          │                │       13        │                │
#  400     │       12       │─────────────────│       16       │
#          │                │       14        │                │
#  440     │                │─────────────────│                │
#          │                │       15        │                │
#  480     └────────────────┴─────────────────┴────────────────┘

ROIS = [ # [ROI, weight]
        # 顶部行 (y=0~160)
        (0,   0, 213, 160, 0.1),  # idx 0  左上

        (213, 0, 213, 40, 0.3),   # idx 1  中上-上
        (213, 40, 213, 40, 0.1),  # idx 2  中上-中
        (213, 80, 213, 80, 0.2),  # idx 3  中上-下

        (426, 0, 214, 160, 0.1),  # idx 4  右上

        # 中部行 (y=160~320)
        (0, 160, 53, 160, 0.3),   # idx 5  中左-左
        (53, 160, 53, 160, 0.1),  # idx 6  中左-中
        (106, 160, 107, 160, 0.2),# idx 7  中左-右
        (213, 160, 213, 160, 0.6),# idx 8  中部-中（权重最大，核心循线区）
        (426, 160, 107, 160, 0.2),# idx 9  中右-左
        (533, 160, 53, 160, 0.1), # idx 10 中右-中
        (586, 160, 54, 160, 0.3), # idx 11 中右-右

        # 底部行 (y=320~480)
        (0, 320, 213, 160, 0.1),  # idx 12 左下

        (213, 320, 213, 80, 0.2), # idx 13 中下-上
        (213, 400, 213, 40, 0.1), # idx 14 中下-中
        (213, 440, 213, 40, 0.3), # idx 15 中下-下

        (426, 320, 214, 160, 0.1),# idx 16 右下
       ]

# ===================== 区域分组（基于运动方向动态切换）=====================
# 将17个ROI按方位分组，用于拐角检测时的区域判断
# 画面坐标系：左上角(0,0)，x向右增大，y向下增大
# 侧向区域包含拐角ROI（高灵敏度），前方区域仅用中间区域（防误触发）

def get_zones(move_dir):
    """
    根据当前运动方向返回动态区域分组。
    - 上下移动时：左右为侧向（含拐角ROI），上下为前方（仅中间列）
    - 左右移动时：上下为侧向（含拐角ROI），左右为前方（中间行+核心ROI 8）
    - center: 中心分区ROI，直线运动时这些ROI应该全部检测到线，
              若不全有线则说明线条弯向了侧方（即拐角）
    """
    if move_dir in (DIR_UP, DIR_DOWN):
        return {
            'top':    [1, 2, 3],          # 上方中间列 —— 前方
            'bottom': [13, 14, 15],       # 下方中间列 —— 前方
            'left':   [0, 5, 6, 12],      # 左侧（含拐角） —— 侧向
            'right':  [4, 10, 11, 16],    # 右侧（含拐角） —— 侧向
            'center': [1, 2, 3, 8, 13, 14, 15],  # 纵向中心列，直线上下运动应全有线
        }
    elif move_dir == DIR_LEFT:
        return {
            'top':    [0, 1, 2, 4],       # 上方（含拐角） —— 侧向
            'bottom': [12, 14, 15, 16],   # 下方（含拐角） —— 侧向
            'left':   [5, 6, 7],       # 左侧中间行+核心 —— 前方
            'right':  [9, 10, 11],        # 右侧中间行 —— 后方
            'center': [5, 6, 7, 8, 9, 10, 11],  # 横向中心行，直线左右运动应全有线
        }
    else:  # DIR_RIGHT
        return {
            'top':    [0, 1, 2, 4],       # 上方（含拐角） —— 侧向
            'bottom': [12, 14, 15, 16],   # 下方（含拐角） —— 侧向
            'left':   [5, 6, 7],          # 左侧中间行 —— 后方
            'right':  [9, 10, 11],     # 右侧中间行+核心 —— 前方
            'center': [5, 6, 7, 8, 9, 10, 11],  # 横向中心行，直线左右运动应全有线
        }


def dir_to_speed(d):
    """
    将方向常量转换为单位速度向量 [dx, dy]。

    参数:
        d: 方向常量 (DIR_UP / DIR_DOWN / DIR_LEFT / DIR_RIGHT)
    返回:
        [dx, dy]: 该方向上的单位速度向量
                  UP    = [0, -1]  (画面中y减小)
                  DOWN  = [0, +1]  (画面中y增大)
                  LEFT  = [-1, 0]  (画面中x减小)
                  RIGHT = [+1, 0]  (画面中x增大)
    """
    return {
        DIR_UP:    [0.0, -1.0],
        DIR_DOWN:  [0.0,  1.0],
        DIR_LEFT:  [-1.0, 0.0],
        DIR_RIGHT: [1.0,  0.0]
    }[d]


def set_target(state, move_dir, corner_new_dir, corner_frames,
               center_pos_x, center_pos_y):
    """
    根据当前状态计算机械臂末端的目标运动偏移量 [dx, dy]。

    三种状态下的计算方式：

    FOLLOWING:
        沿当前运动方向以 FOLLOW_SPEED 匀速前进，
        同时根据质心偏移做对中修正（偏移量 × CENTERING_GAIN）。

    CORNER:
        使用 smoothstep 插值实现圆角过渡：
        - t = min(1.0, corner_frames / CORNER_DURATION)  线性进度 0→1
        - t = t² × (3 - 2t)                              smoothstep 平滑化
        - 旧方向速度 = FOLLOW_SPEED × (1 - t)             渐减
        - 新方向速度 = CORNER_SPEED × t                   渐增
        同时叠加质心对中修正，确保过渡期间不偏离线条。

    LOST:
        停止运动，输出 [0, 0]。

    参数:
        state:          当前状态 (STATE_FOLLOWING / STATE_CORNER / STATE_LOST)
        move_dir:       当前运动方向
        corner_new_dir: 拐角后的新运动方向（仅 CORNER 状态有效）
        corner_frames:  拐角过渡已进行帧数（仅 CORNER 状态有效）
        center_pos_x:   加权质心 x 坐标
        center_pos_y:   加权质心 y 坐标
    返回:
        [dx, dy]: 目标运动偏移量
    """
    target = [0.0, 0.0]

    # 质心对中修正：质心偏离画面中心的距离 × 增益
    # 画面中心为 (320, 240)，质心偏离中心时产生修正力
    cx_corr = (center_pos_x - 320) * CENTERING_GAIN
    cy_corr = (center_pos_y - 240) * CENTERING_GAIN

    if state == STATE_FOLLOWING:
        # 正常循线：主方向匀速 + 质心对中修正
        dx, dy = dir_to_speed(move_dir)
        target[0] = dx * FOLLOW_SPEED + cx_corr
        target[1] = dy * FOLLOW_SPEED + cy_corr

    elif state == STATE_CORNER:
        # 拐角过渡：smoothstep 圆角插值
        # t 从 0 线性增长到 1，再经 smoothstep 变换使过渡更平滑
        t = min(1.0, corner_frames / CORNER_DURATION)
        t = t * t * (3 - 2 * t)  # Hermite 插值 (smoothstep)

        # 获取旧方向和新方向的单位速度向量
        old_dx, old_dy = dir_to_speed(move_dir)
        new_dx, new_dy = dir_to_speed(corner_new_dir)

        # 旧方向速度渐减，新方向速度渐增，叠加对中修正
        target[0] = old_dx * FOLLOW_SPEED * (1 - t) + new_dx * CORNER_SPEED * t + cx_corr
        target[1] = old_dy * FOLLOW_SPEED * (1 - t) + new_dy * CORNER_SPEED * t + cy_corr

    # STATE_LOST: target 保持 [0, 0]，停止运动

    return target


# ===================== 计算权重和 =====================
# 权重和用于加权质心计算，不一定为1
weight_sum = 0
for r in ROIS:
    weight_sum += r[4]  # r[4] 是该ROI的权重值


# ===================== 状态变量初始化 =====================
state = STATE_FOLLOWING          # 当前状态，初始为循线
move_dir = INIT_DIRECTION        # 当前运动方向
corner_new_dir = -1              # 拐角后的新方向（-1 表示未设定）
corner_frames = 0                # 拐角过渡已进行的帧数
lost_frames = 0                  # 连续未检测到线条的帧数
corner_detect_count = 0          # 连续检测到拐角的帧数（用于防抖）
potential_corner_dir = -1        # 防抖中暂存的拐角方向
cooldown_frames = 0              # 拐角冷却剩余帧数（>0 时禁止拐角检测，防止旧线段误触发）
prev_dir = -1                    # 上一次运动方向（拐角前的方向），用于排除旧线段所在区域
filtered_cx = 320.0              # 滤波后的质心 x，初始为画面中心
filtered_cy = 240.0              # 滤波后的质心 y，初始为画面中心

# ===================== 串口初始化 =====================
fpioa = FPIOA()
fpioa.set_function(44, FPIOA.UART2_TXD)
fpioa.set_function(45, FPIOA.UART2_RXD)
uart2 = UART(UART.UART2, baudrate=115200, bits=UART.EIGHTBITS, parity=UART.PARITY_NONE, stop=UART.STOPBITS_ONE)


try:
    sensor = Sensor(width=1280, height=960) # 构建摄像头对象
    sensor.reset() # 复位和初始化摄像头
    sensor.set_framesize(Sensor.VGA)    # 设置帧大小VGA(640x480)，默认通道0
    sensor.set_pixformat(Sensor.GRAYSCALE) # 设置输出图像格式，默认通道0

    # 初始化LCD显示器，同时IDE缓冲区输出图像,显示的数据来自于sensor通道0。
    Display.init(Display.ST7701, width=640, height=480, fps=90, to_ide=True)
    MediaManager.init() # 初始化media资源管理器
    sensor.run() # 启动sensor
    clock = time.clock() # 构造clock对象

    while True:
        os.exitpoint() # 检测IDE中断
        clock.tick()   # 记录开始时间（ms）
        img = sensor.snapshot() # 从通道0捕获一张图

        # ============ 第一步：ROI 色块检测 ============
        # 遍历所有ROI，检测黑线色块，计算加权质心，记录各ROI是否有色块
        centroid_sum_x = 0          # 加权质心 x 分量累加
        centroid_sum_y = 0          # 加权质心 y 分量累加
        active_weight_sum = 0       # 本帧实际检测到色块的ROI权重和
        roi_has_blob = [False] * len(ROIS)  # 记录每个ROI是否检测到色块

        for idx, r in enumerate(ROIS):
            # 在该ROI区域内寻找满足灰度阈值的色块
            blobs = img.find_blobs(GRAYSCALE_THRESHOLD, roi=r[0:4], merge=True)

            if blobs:
                # 找到像素最多的色块（最可能是目标线条）
                largest_blob = max(blobs, key=lambda b: b.pixels())

                # 在画面上标记色块矩形和质心十字
                img.draw_rectangle([v for v in largest_blob.rect()])
                img.draw_cross(largest_blob.cx(), largest_blob.cy())

                # 累加加权质心（位置 × 权重）
                centroid_sum_x += largest_blob.cx() * r[4]
                centroid_sum_y += largest_blob.cy() * r[4]
                active_weight_sum += r[4]

                # 标记该ROI检测到色块
                roi_has_blob[idx] = True

        # ============ 第二步：区域聚合 ============
        # 将各ROI的检测结果按方位聚合，用于拐角判断
        # 根据当前运动方向动态选择区域分组
        zones = get_zones(move_dir)
        top    = any(roi_has_blob[i] for i in zones['top'])     # 上方区域有线？
        bottom = any(roi_has_blob[i] for i in zones['bottom'])  # 下方区域有线？
        left   = any(roi_has_blob[i] for i in zones['left'])    # 左侧区域有线？
        right  = any(roi_has_blob[i] for i in zones['right'])   # 右侧区域有线？
        any_blob = any(roi_has_blob)                             # 画面中任意区域有线？

        # 中心分区完整性：直线运动时中心分区ROI应全部检测到线
        # 若不全有线，说明线条弯向了侧方（即拐角特征）
        center_count = sum(1 for i in zones['center'] if roi_has_blob[i])
        center_complete = center_count == len(zones['center'])

        # ============ 第三步：计算加权质心 ============
        # 质心位置用于对中修正，线条居中时质心接近画面中心(320, 240)
        if active_weight_sum > 0:
            center_pos_x = centroid_sum_x / active_weight_sum
            center_pos_y = centroid_sum_y / active_weight_sum
            # 一阶低通滤波：平滑质心抖动
            filtered_cx = CENTERING_FILTER_ALPHA * center_pos_x + (1 - CENTERING_FILTER_ALPHA) * filtered_cx
            filtered_cy = CENTERING_FILTER_ALPHA * center_pos_y + (1 - CENTERING_FILTER_ALPHA) * filtered_cy
        else:
            # 没检测到色块时不更新滤波值，保持上一帧结果
            center_pos_x = 320
            center_pos_y = 240

        # ============ 第四步：状态机逻辑 ============
        if state == STATE_FOLLOWING:
            # --- 循线状态 ---
            # 主要任务：沿当前方向前进 + 检测拐角 + 检测丢失

            # 冷却期递减：拐角刚完成时旧线段仍在视野中，
            # 需要等待若干帧让机器人远离旧线段后再检测新拐角
            if cooldown_frames > 0:
                cooldown_frames -= 1

            # 判断当前运动方向的"前方"是否有线
            # 根据运动方向映射到对应的画面区域：
            #   UP    → 上方区域 (ZONE_TOP)
            #   DOWN  → 下方区域 (ZONE_BOTTOM)
            #   LEFT  → 左侧区域 (ZONE_LEFT)
            #   RIGHT → 右侧区域 (ZONE_RIGHT)
            forward = {
                DIR_UP:    top,
                DIR_DOWN:  bottom,
                DIR_LEFT:  left,
                DIR_RIGHT: right
            }[move_dir]

            # 拐角检测：前方有线 + 中心分区不完整 + 侧方有线 + 对侧为空
            # 核心逻辑：拐角处线条弯向侧方，中心分区的部分ROI看不到线了
            # 对侧必须为空，防止宽线或偏移居中线同时触发两侧
            # 冷却期内跳过拐角检测，防止旧线段误触发方向反转
            detected = -1  # 检测到的拐角方向，-1 表示未检测到拐角

            if forward and not center_complete and cooldown_frames == 0:
                # 根据当前运动方向，检查左侧或右侧是否出现线条
                # prev_dir 排除旧线段所在区域，防止机器人回头走向来时的方向
                if move_dir == DIR_UP:
                    # 向上走时：前方=上区，侧方=左区/右区
                    if prev_dir != DIR_LEFT and left and not right:    detected = DIR_LEFT    # 左侧有线+右侧空 → 左拐
                    elif prev_dir != DIR_RIGHT and right and not left: detected = DIR_RIGHT   # 右侧有线+左侧空 → 右拐
                elif move_dir == DIR_DOWN:
                    # 向下走时：前方=下区，侧方=左区/右区
                    if prev_dir != DIR_LEFT and left and not right:    detected = DIR_LEFT    # 左侧有线+右侧空 → 左拐
                    elif prev_dir != DIR_RIGHT and right and not left: detected = DIR_RIGHT   # 右侧有线+左侧空 → 右拐
                elif move_dir == DIR_LEFT:
                    # 向左走时：前方=左区，侧方=上区/下区
                    if prev_dir != DIR_UP and top and not bottom:        detected = DIR_UP      # 上方有线+下方空 → 上拐
                    elif prev_dir != DIR_DOWN and bottom and not top:    detected = DIR_DOWN    # 下方有线+上方空 → 下拐
                elif move_dir == DIR_RIGHT:
                    # 向右走时：前方=右区，侧方=上区/下区
                    if prev_dir != DIR_UP and top and not bottom:        detected = DIR_UP      # 上方有线+下方空 → 上拐
                    elif prev_dir != DIR_DOWN and bottom and not top:    detected = DIR_DOWN    # 下方有线+上方空 → 下拐

            # 防抖处理：需要连续 CORNER_DEBOUNCE 帧检测到同一方向才确认拐角
            # 避免因噪声或短暂误检测导致误触发
            if detected >= 0:
                if detected == potential_corner_dir:
                    # 与上一帧检测方向一致，累加计数
                    corner_detect_count += 1
                else:
                    # 方向变化，重新开始计数
                    potential_corner_dir = detected
                    corner_detect_count = 1

                # 连续检测达到阈值，确认进入拐角过渡
                if corner_detect_count >= CORNER_DEBOUNCE:
                    state = STATE_CORNER
                    corner_new_dir = potential_corner_dir
                    corner_frames = 0
                    corner_detect_count = 0
            else:
                # 当前帧未检测到拐角，重置防抖计数
                corner_detect_count = 0
                potential_corner_dir = -1

            # 丢失检测：连续 LOST_TOLERANCE 帧无任何色块则判定丢失
            if not any_blob:
                lost_frames += 1
                if lost_frames > LOST_TOLERANCE:
                    state = STATE_LOST
            else:
                lost_frames = 0

        elif state == STATE_CORNER:
            # --- 拐角过渡状态 ---
            # 主要任务：smoothstep 过渡（旧方向渐减 + 新方向渐增）
            # corner_frames 递增，set_target 中会根据它计算插值比例

            corner_frames += 1

            # 过渡完成：达到 CORNER_DURATION 帧后，切换运动方向，回到循线
            if corner_frames >= CORNER_DURATION:
                prev_dir = move_dir         # 记录拐角前的方向，用于排除旧线段区域
                move_dir = corner_new_dir   # 更新运动方向为新方向
                state = STATE_FOLLOWING     # 回到循线状态
                corner_frames = 0
                cooldown_frames = CORNER_COOLDOWN  # 启动冷却，防止旧线段误触发

            # 丢失检测：过渡期间也可能丢失线条
            if not any_blob:
                lost_frames += 1
                if lost_frames > LOST_TOLERANCE:
                    state = STATE_LOST
            else:
                lost_frames = 0

        elif state == STATE_LOST:
            # --- 丢失状态 ---
            # 停止运动，等待重新检测到线条后恢复循线

            if any_blob:
                # 重新检测到线条，恢复循线
                state = STATE_FOLLOWING
                lost_frames = 0

        # ============ 第五步：计算控制量 ============
        # 调用 set_target 计算当前帧的目标运动偏移量
        target = set_target(state, move_dir, corner_new_dir, corner_frames,
                           filtered_cx, filtered_cy)

        # 将target向量归一化，模长始终设为0.01
        # mag = math.sqrt(target[0]**2 + target[1]**2)
        # if mag > 0:
        #     target[0] = target[0] / mag * 0.01
        #     target[1] = target[1] / mag * 0.01

        # ============ 第六步：串口发送数据 ============
        # 帧格式: AA55 + X标识(01) + X符号 + X值(uint16) + Y标识(02) + Y符号 + Y值(uint16) + 55AA
        # 每帧固定12字节
        x_sign = 0x2B if target[0] >= 0 else 0x2D  # '+' or '-'
        y_sign = 0x2B if target[1] >= 0 else 0x2D
        x_val = int(abs(target[0]) * 10000) & 0xFFFF  # 0.0100 → 100, uint16
        y_val = int(abs(target[1]) * 10000) & 0xFFFF
        frame  = b'\xAA\x55'                          # 包头
        frame += b'\x01'                               # X标识
        frame += bytes([x_sign])                       # X符号
        frame += struct.pack('<H', x_val)              # X值(小端uint16)
        frame += b'\x02'                               # Y标识
        frame += bytes([y_sign])                       # Y符号
        frame += struct.pack('<H', y_val)              # Y值(小端uint16)
        frame += b'\x55\xAA'                           # 包尾
        uart2.write(frame)

        # ============ 第七步：画面显示 ============
        # 第一行：当前状态和运动方向
        img.draw_string_advanced(0, 0, 24,
            "S:%s D:%s" % (STATE_NAMES[state], DIR_NAMES[move_dir]),
            color=(255,255,255), thickness=4)

        # 第二行：目标偏移量（X/Y 正负方向清晰显示）
        img.draw_string_advanced(0, 24, 24,
            "X:%s%.4f Y:%s%.4f" % (chr(x_sign), abs(target[0]), chr(y_sign), abs(target[1])),
            color=(255,255,255), thickness=4)

        # 第三行：中心分区完整性指示（用于调试拐角检测）
        center_color = (0,255,0) if center_complete else (255,0,0)
        img.draw_string_advanced(0, 48, 24,
            "C:%d/%d" % (center_count, len(zones['center'])),
            color=center_color, thickness=4)

        # 方向箭头：在画面右下角显示当前运动方向
        arrow_dx, arrow_dy = dir_to_speed(move_dir)
        arrow_cx, arrow_cy = 580, 440  # 箭头中心
        arrow_len = 30                  # 箭头半长度
        x1 = int(arrow_cx - arrow_dx * arrow_len)
        y1 = int(arrow_cy - arrow_dy * arrow_len)
        x2 = int(arrow_cx + arrow_dx * arrow_len)
        y2 = int(arrow_cy + arrow_dy * arrow_len)
        img.draw_arrow(x1, y1, x2, y2, color=(255,255,255), thickness=2)

        # 拐角过渡时额外显示：新方向和过渡进度
        if state == STATE_CORNER:
            img.draw_string_advanced(0, 72, 24,
                "ND:%s %d/%d" % (DIR_NAMES[corner_new_dir],
                                 corner_frames, CORNER_DURATION),
                color=(255,255,255), thickness=4)

        # 在画面上用红色圆点标记滤波后的质心位置
        img.draw_circle(int(filtered_cx), int(filtered_cy), 5, color=(255,0,0), thickness=2)
        img.draw_circle(int(filtered_cx), int(filtered_cy), 2, color=(255,0,0), thickness=-1)

        # 以绿色箭头表示对中修正速度向量（从画面中心出发，缩放显示）
        CORR_DISPLAY_SCALE = 10000
        cx_corr_disp = (filtered_cx - 320) * CENTERING_GAIN * CORR_DISPLAY_SCALE
        cy_corr_disp = (filtered_cy - 240) * CENTERING_GAIN * CORR_DISPLAY_SCALE
        if abs(cx_corr_disp) > 2 or abs(cy_corr_disp) > 2:
            img.draw_arrow(320, 240, int(320 + cx_corr_disp), int(240 + cy_corr_disp), color=(0, 255, 0), thickness=2)

        # 丢失时显示丢失帧数
        if state == STATE_LOST:
            img.draw_string_advanced(0, 72, 24,
                "LOST:%d" % lost_frames,
                color=(255,0,0), thickness=4)

        # 显示图片
        Display.show_image(img)

        # 终端打印：状态、方向、控制量、帧率
        print("S:%s D:%s X:%s%.4f Y:%s%.4f fps:%.1f" % (
            STATE_NAMES[state], DIR_NAMES[move_dir],
            chr(x_sign), abs(target[0]), chr(y_sign), abs(target[1]), clock.fps()))

# IDE中断释放资源代码
except KeyboardInterrupt as e:
    print("user stop: ", e)
except BaseException as e:
    print(f"Exception {e}")
finally:
    # 关闭串口
    try:
        uart2.deinit()
    except NameError:
        pass
    # sensor stop run
    try:
        if isinstance(sensor, Sensor):
            sensor.stop()
    except NameError:
        pass
    # deinit display
    Display.deinit()
    os.exitpoint(os.EXITPOINT_ENABLE_SLEEP)
    time.sleep_ms(100)
    # release media buffer
    MediaManager.deinit()
