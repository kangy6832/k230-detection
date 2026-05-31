# K230 CanMV MicroPython 黑色边框中心线检测
# 适用：嘉楠 K230 / CanMV 固件的 MicroPython 环境
# 功能：摄像头固定在机械臂末端，检测末端是否对准黑色边框中心线，
#       并判断黑色像素在中心框的哪个方向（上/下/左/右），辅助机械臂循迹。

import time

import image
from media.sensor import *
from media.display import *
from media.media import *


# ===================== 可调参数 =====================
# 采集分辨率。QVGA 速度快，适合实时检测。
CAMERA_WIDTH = 320
CAMERA_HEIGHT = 240

# 灰度二值化阈值：灰度值在此范围内的像素视为黑色（边框）
# 如果黑色检测不到：把 GRAY_MAX 调大，例如 80/100。
# 如果背景误检为黑色：把 GRAY_MAX 调小，例如 40/50。
GRAY_MIN = 0
GRAY_MAX = 64

# 中心正方形框大小（像素），用于检测末端是否对准黑线中心线
# 黑线在画面中约 45 像素宽，默认 30 可覆盖大部分宽度
# 如果检测太严格（经常报偏离）：调大，例如 40
# 如果检测太宽松（偏离了还不报）：调小，例如 20
CENTER_BOX_SIZE = 30

# 方向检测区域大小（像素），检测黑色像素在中心框的哪个方向
# 默认与中心框一致，如果黑线延伸部分较远可适当调大
DIRECTION_BOX_SIZE = 30

# 方向检测区域中，黑色像素占比超过此阈值才认为该方向存在黑色
# 如果噪点导致误检：调大，例如 0.15/0.20
# 如果黑线检测不到：调小，例如 0.05
DIRECTION_BLACK_RATIO = 0.10

# 是否水平镜像，画面方向不对时切换。
H_MIRROR = False
V_FLIP = False


# ===================== 检测逻辑 =====================
def get_center_box_roi(img_w, img_h, box_size):
    """计算图像中心的正方形框 ROI：(x, y, w, h)"""
    cx = img_w // 2
    cy = img_h // 2
    half = box_size // 2
    x = cx - half
    y = cy - half
    return (x, y, box_size, box_size)


def get_direction_rois(img_w, img_h, center_size, direction_size):
    """
    计算中心框上下左右四个方向的检测区域 ROI。
    每个方向区域以中心框边缘为基准向外延伸。
    返回字典：{"up": roi, "down": roi, "left": roi, "right": roi}
    """
    cx = img_w // 2
    cy = img_h // 2
    chalf = center_size // 2

    # 中心框的四个边缘
    center_left = cx - chalf
    center_right = cx + chalf
    center_top = cy - chalf
    center_bottom = cy + chalf

    rois = {
        # 上方区域：与中心框同宽，向上延伸
        "up": (center_left, center_top - direction_size, direction_size, direction_size),
        # 下方区域：与中心框同宽，向下延伸
        "down": (center_left, center_bottom, direction_size, direction_size),
        # 向左区域：与中心框同高，向左延伸
        "left": (center_left - direction_size, center_top, direction_size, direction_size),
        # 向右区域：与中心框同高，向右延伸
        "right": (center_right, center_top, direction_size, direction_size),
    }
    return rois


def check_center_box_white(img, roi):
    """
    检测中心正方形框内是否存在白色像素。
    二值化后：黑色(边框)=0，白色(背景)=255。
    如果中心框内有白色像素，说明末端偏离了黑线。
    返回：True = 有白色像素（偏离），False = 全黑（对准）
    """
    # 在二值化图像中检测白色像素（灰度值 255）
    blobs = img.find_blobs([(255, 255)], roi=roi, pixels_threshold=5, merge=True)
    for blob in blobs:
        if blob.pixels() > 0:
            return True
    return False


def check_direction_black(img, roi, black_ratio):
    """
    检测指定区域内黑色像素占比是否超过阈值。
    二值化后：黑色=0，白色=255。
    通过检测白色像素，用总面积减去白色像素得到黑色像素数。
    返回：True = 黑色像素占比超过阈值，False = 未超过
    """
    w = roi[2]
    h = roi[3]
    total_pixels = w * h
    if total_pixels == 0:
        return False

    # 二值化后 find_blobs 检测白色像素（灰度值 255）
    white_blobs = img.find_blobs([(255, 255)], roi=roi, pixels_threshold=1, merge=True)
    white_pixels = 0
    for blob in white_blobs:
        white_pixels += blob.pixels()

    black_pixels = total_pixels - white_pixels
    ratio = black_pixels / total_pixels
    return ratio >= black_ratio


def draw_detection(img, center_roi, direction_rois, has_white, direction_results):
    """在图像上绘制检测结果。"""
    cx = CAMERA_WIDTH // 2
    cy = CAMERA_HEIGHT // 2

    # 绘制中心正方形框
    # 对准为绿色，偏离为红色
    center_color = (255, 0, 0) if has_white else (0, 255, 0)
    img.draw_rectangle(center_roi, color=center_color, thickness=2)

    # 绘制四个方向检测区域
    direction_colors = {
        "up": (0, 255, 0) if direction_results["up"] else (255, 0, 0),
        "down": (0, 255, 0) if direction_results["down"] else (255, 0, 0),
        "left": (0, 255, 0) if direction_results["left"] else (255, 0, 0),
        "right": (0, 255, 0) if direction_results["right"] else (255, 0, 0),
    }
    for name, roi in direction_rois.items():
        img.draw_rectangle(roi, color=direction_colors[name], thickness=1)

    # 绘制中心十字线
    img.draw_cross(cx, cy, color=center_color, size=5, thickness=1)


# ===================== K230 初始化 =====================
def init_camera():
    sensor = Sensor()
    sensor.reset()
    sensor.set_hmirror(H_MIRROR)
    sensor.set_vflip(V_FLIP)
    sensor.set_framesize(width=CAMERA_WIDTH, height=CAMERA_HEIGHT, chn=CAM_CHN_ID_0)
    sensor.set_pixformat(Sensor.GRAYSCALE, chn=CAM_CHN_ID_0)
    return sensor


def init_display():
    Display.init(Display.VIRT, width=CAMERA_WIDTH, height=CAMERA_HEIGHT, fps=30)


# ===================== 主程序 =====================
def main():
    sensor = None

    try:
        sensor = init_camera()
        init_display()
        MediaManager.init()
        sensor.run()

        clock = time.clock()

        # 预计算 ROI 区域
        center_roi = get_center_box_roi(CAMERA_WIDTH, CAMERA_HEIGHT, CENTER_BOX_SIZE)
        direction_rois = get_direction_rois(
            CAMERA_WIDTH, CAMERA_HEIGHT, CENTER_BOX_SIZE, DIRECTION_BOX_SIZE
        )

        while True:
            clock.tick()
            img = sensor.snapshot(chn=CAM_CHN_ID_0)

            # 灰度二值化：黑色像素（边框）→ 0（黑），其他 → 255（白）
            img = img.binary([(GRAY_MAX, 255)])

            # 检测中心框内是否有白色像素（是否偏离黑线）
            has_white = check_center_box_white(img, center_roi)

            # 检测四个方向是否存在黑色像素
            direction_results = {}
            for name, roi in direction_rois.items():
                direction_results[name] = check_direction_black(
                    img, roi, DIRECTION_BLACK_RATIO
                )

            # 绘制检测结果
            draw_detection(img, center_roi, direction_rois, has_white, direction_results)

            # 显示 FPS
            img.draw_string(2, 2, "FPS: %.1f" % clock.fps(), color=255, scale=1)

            # 显示检测结果
            result_str = "W:%s U:%s D:%s L:%s R:%s" % (
                "T" if has_white else "F",
                "T" if direction_results["up"] else "F",
                "T" if direction_results["down"] else "F",
                "T" if direction_results["left"] else "F",
                "T" if direction_results["right"] else "F",
            )
            img.draw_string(2, CAMERA_HEIGHT - 16, result_str, color=255, scale=1)

            # 串口输出检测结果
            print("center_white=%s up=%s down=%s left=%s right=%s fps=%.1f" % (
                has_white,
                direction_results["up"],
                direction_results["down"],
                direction_results["left"],
                direction_results["right"],
                clock.fps(),
            ))

            Display.show_image(img)

    except KeyboardInterrupt:
        print("User stopped")
    except Exception as error:
        print("Error:", error)
    finally:
        if sensor is not None:
            sensor.stop()
        Display.deinit()
        MediaManager.deinit()


main()
