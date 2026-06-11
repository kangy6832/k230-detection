#####################################################################################################
# @file         main.py
# @author       正点原子团队(ALIENTEK)
# @version      V3.0
# @date         2026-06-06
# @brief        红色目标三维坐标检测（轻量单目版）
# @license      Copyright (c) 2020-2032, 广州市星翼电子科技有限公司
#####################################################################################################
# @attention
#
# 实验平台:正点原子 K230D BOX开发板
# 当前方案: 基于红色 blob 中心点 + 已知目标真实尺寸，快速估算目标在相机坐标系下的坐标(mm)
# 说明:
# 1. 此版本优先保证速度，适合 K230 板端实时跑
# 2. 不再做 find_rects / 四角提取 / 单应矩阵 / 重投影误差
# 3. 目标尽量使用红色正方形/矩形顶面，且正对相机时精度更好
# 4. 必须按实际相机重新标定 CAMERA_MATRIX，按实际物块修改 TARGET_WIDTH_MM / TARGET_HEIGHT_MM
#
#####################################################################################################

import time, os
from media.sensor import *  # 导入sensor模块，使用摄像头相关接口
from media.display import * # 导入display模块，使用display相关接口
from media.media import *   # 导入media模块，使用meida相关接口

# ------------------------------ 运行参数 ------------------------------
# 红色LAB阈值，可按实际环境微调
RED_THRESHOLDS = [(20, 100, 20, 127, 10, 127)]

# 目标顶面真实尺寸（毫米）
# !!! 必须按你的红色物块实际尺寸修改 !!!
TARGET_WIDTH_MM = 19.0
TARGET_HEIGHT_MM = 34.0

# 内参（标定分辨率1280x960，按VGA 640x480缩放）
# 原始: fx=1587.84, fy=1589.32, cx=643.03, cy=480.87
FX = 793.92
FY = 794.66
CX = 321.51
CY = 240.43

MIN_AREA_PIXELS = 300
MERGE_MARGIN = 10
EMA_ALPHA = 0.35
CONFIRM_FRAMES = 3
PRINT_EVERY_N_FRAMES = 5
MIN_CAMERA_Z_MM = 50.0
MAX_CAMERA_Z_MM = 800.0
ASPECT_RATIO_TOL = 0.60


def estimate_pose_from_blob(blob):
    """用 blob 外接框快速估算相机坐标系位置。"""
    w = blob.w()
    h = blob.h()
    if w <= 0 or h <= 0:
        return None

    # 取宽高两个方向的测距结果做平均，减小单边抖动
    z_from_w = (FX * TARGET_WIDTH_MM) / w
    z_from_h = (FY * TARGET_HEIGHT_MM) / h
    z_mm = (z_from_w + z_from_h) * 0.5

    u = blob.cx()
    v = blob.cy()
    x_mm = ((u - CX) * z_mm) / FX
    y_mm = ((v - CY) * z_mm) / FY
    return [x_mm, y_mm, z_mm]


def is_blob_shape_ok(blob):
    w = blob.w()
    h = blob.h()
    if w <= 0 or h <= 0:
        return False
    ratio = w / h
    target_ratio = TARGET_WIDTH_MM / TARGET_HEIGHT_MM
    if target_ratio <= 0:
        return False
    return abs(ratio - target_ratio) <= ASPECT_RATIO_TOL


def draw_pose_info(img, status, pos_mm, fps_value, blob):
    s = 3  # 字体缩放倍数
    dy = 28
    y = 4
    img.draw_string(4, y, "status=%s" % status, color=(255, 255, 0), scale=s)
    y += dy
    if pos_mm is not None:
        img.draw_string(4, y, "x=%.1fmm" % pos_mm[0], color=(255, 0, 0), scale=s); y += dy
        img.draw_string(4, y, "y=%.1fmm" % pos_mm[1], color=(255, 0, 0), scale=s); y += dy
        img.draw_string(4, y, "z=%.1fmm" % pos_mm[2], color=(255, 0, 0), scale=s); y += dy
    if blob is not None:
        img.draw_string(4, y, "w=%d h=%d" % (blob.w(), blob.h()), color=(0, 255, 0), scale=s); y += dy
    img.draw_string(4, y, "fps=%.1f" % fps_value, color=(0, 255, 255), scale=s)


try:
    sensor = Sensor(width=1280, height=960) # 构建摄像头对象
    sensor.reset() # 复位和初始化摄像头
    sensor.set_framesize(Sensor.VGA)    # 设置帧大小VGA(640x480)，默认通道0
    sensor.set_pixformat(Sensor.RGB565) # 设置输出图像格式，默认通道0

    # 初始化LCD显示器，同时IDE缓冲区输出图像,显示的数据来自于sensor通道0。
    Display.init(Display.ST7701, width=640, height=480, fps=90, to_ide=True)
    MediaManager.init() # 初始化media资源管理器
    sensor.run() # 启动sensor
    clock = time.clock() # 构造clock对象

    smoothed_t = None
    ok_count = 0
    last_status = "NO_TARGET"
    frame_id = 0

    while True:
        os.exitpoint() # 检测IDE中断
        clock.tick()  # 记录开始时间（ms）
        img = sensor.snapshot() # 从通道0捕获一张图
        frame_id += 1

        status = "NO_TARGET"
        output_pos = None
        best_blob = None

        blobs = img.find_blobs(
            RED_THRESHOLDS,
            pixels_threshold=MIN_AREA_PIXELS,
            area_threshold=MIN_AREA_PIXELS,
            merge=True,
            margin=MERGE_MARGIN,
        )

        if blobs:
            best_blob = max(blobs, key=lambda b: b.pixels())
            img.draw_rectangle([v for v in best_blob.rect()], color=(255, 0, 0))
            img.draw_cross(best_blob.cx(), best_blob.cy(), color=(0, 255, 0))

            pose = estimate_pose_from_blob(best_blob)
            if pose is not None:
                z_ok = (pose[2] >= MIN_CAMERA_Z_MM) and (pose[2] <= MAX_CAMERA_Z_MM)
                shape_ok = is_blob_shape_ok(best_blob)

                if z_ok and shape_ok:
                    if smoothed_t is None:
                        smoothed_t = [pose[0], pose[1], pose[2]]
                    else:
                        smoothed_t[0] = (EMA_ALPHA * pose[0]) + ((1.0 - EMA_ALPHA) * smoothed_t[0])
                        smoothed_t[1] = (EMA_ALPHA * pose[1]) + ((1.0 - EMA_ALPHA) * smoothed_t[1])
                        smoothed_t[2] = (EMA_ALPHA * pose[2]) + ((1.0 - EMA_ALPHA) * smoothed_t[2])

                    ok_count += 1
                    output_pos = smoothed_t
                    if ok_count >= CONFIRM_FRAMES:
                        status = "OK"
                    else:
                        status = "UNRELIABLE"
                else:
                    ok_count = 0
                    smoothed_t = None
                    output_pos = pose
                    status = "UNRELIABLE"
        else:
            ok_count = 0
            smoothed_t = None

        fps_value = clock.fps()
        draw_pose_info(img, status, output_pos, fps_value, best_blob)
        Display.show_image(img)

        if (frame_id % PRINT_EVERY_N_FRAMES) == 0 or status != last_status:
            if output_pos is not None:
                print("status=%s,frame=%d,x_mm=%.2f,y_mm=%.2f,z_mm=%.2f" % (
                    status,
                    frame_id,
                    output_pos[0],
                    output_pos[1],
                    output_pos[2],
                ))
            else:
                print("status=NO_TARGET,frame=%d" % frame_id)

        last_status = status

# IDE中断释放资源代码
except KeyboardInterrupt as e:
    print("user stop: ", e)
except BaseException as e:
    print("Exception %s" % e)
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
