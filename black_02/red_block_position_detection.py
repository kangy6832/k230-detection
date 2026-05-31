import time, os, sys
from media.sensor import *
from media.display import *
from media.media import *

# ===================== 可配置参数 =====================
# 摄像头分辨率
CAMERA_WIDTH = 320
CAMERA_HEIGHT = 240

# 红色LAB阈值 (l_lo, l_hi, a_lo, a_hi, b_lo, b_hi)
RED_THRESHOLD = (22, 37, -128, 127, -128, 127)

# 圆形框参数
INNER_RADIUS = 30      # 内圆半径（绿色）
OUTER_RADIUS = 60      # 外圆半径（红色），可独立修改，默认2倍内圆

# 深度判断阈值
INNER_RED_RATIO = 0.80  # 内圆内红色占比超过此值视为可能深度正确

# 最小blob像素数，过滤噪声
MIN_PIXELS = 100

# 显示设置
FPS_POS = (220, 5)      # FPS文字位置（右上角）
INFO_POS = (5, 5)       # 检测信息文字位置（左上角）

# 颜色定义 (RGB)
COLOR_GREEN = (0, 255, 0)
COLOR_RED = (255, 0, 0)
COLOR_BLUE = (0, 0, 255)
COLOR_WHITE = (255, 255, 255)
COLOR_YELLOW = (255, 255, 0)
# ====================================================




def check_depth(blob, inner_r, outer_r):
    """
    深度检测逻辑（基于blob大小与圆形框的关系）：
    比较blob等效半径与内外圆的关系
    返回: depth_status (True / "远了" / "近了")
    """
    # 计算blob的等效半径（假设blob近似圆形）
    blob_area = blob.area()
    blob_equiv_r = (blob_area / 3.14159) ** 0.5

    # 判断：
    # - blob等效半径 > 外圆半径 → 近了（太大，溢出）
    # - blob等效半径 <= 内圆半径，且填充率足够 → 深度正确
    # - blob等效半径 <= 内圆半径，填充率不足 → 远了（太小）
    # - 在内圆和外圆之间 → 深度正确
    if blob_equiv_r > outer_r:
        return "近了"
    elif blob_equiv_r <= inner_r:
        inner_area = 3.14159 * inner_r * inner_r
        fill_ratio = blob_area / inner_area
        if fill_ratio > INNER_RED_RATIO:
            return True
        else:
            return "远了"
    else:
        return True


def compute_3d_coord(blob, depth_status, img_w, img_h):
    """
    将深度和位置检测结果转换为三维坐标 [x, y, z]
    深度(z): True->0, 近了->-0.01, 远了->0.01
    X: -10~10->0, <-10->0.01, >10->-0.01
    Y: 同X
    """
    cx = img_w // 2
    cy = img_h // 2
    dx = blob.cx() - cx
    dy = blob.cy() - cy

    # X
    if -10 <= dx <= 10:
        x_val = 0.0
    elif dx < -10:
        x_val = 0.01
    else:
        x_val = -0.01

    # Y
    if -10 <= dy <= 10:
        y_val = 0.0
    elif dy < -10:
        y_val = 0.01
    else:
        y_val = -0.01

    # Z (深度)
    if depth_status is True:
        z_val = 0.0
    elif depth_status == "近了":
        z_val = -0.01
    else:  # "远了"
        z_val = 0.01

    return [x_val, y_val, z_val]


def draw_detection(img, blob, depth_status, fps_val, coord=None):
    """绘制检测结果"""
    cx = img.width() // 2
    cy = img.height() // 2

    # 绘制检测到的红色blob（蓝色矩形框）
    img.draw_rectangle(blob.rect(), color=COLOR_BLUE, thickness=2)

    # 在blob中心画十字标记
    img.draw_cross(blob.cx(), blob.cy(), color=COLOR_YELLOW, size=10, thickness=2)

    # 绘制内圆（绿色）和外圆（红色）
    img.draw_circle(cx, cy, INNER_RADIUS, color=COLOR_GREEN, thickness=2)
    img.draw_circle(cx, cy, OUTER_RADIUS, color=COLOR_RED, thickness=2)

    # 构建深度状态文字
    if depth_status is True:
        depth_text = "深度: 正确"
        depth_color = COLOR_GREEN
    elif depth_status == "近了":
        depth_text = "深度: 近了"
        depth_color = COLOR_RED
    else:
        depth_text = "深度: 远了"
        depth_color = COLOR_RED

    # 左上角显示检测信息
    img.draw_string(INFO_POS[0], INFO_POS[1],
                    "X:{} Y:{}".format(blob.cx(), blob.cy()),
                    color=COLOR_WHITE, scale=2)
    img.draw_string(INFO_POS[0], INFO_POS[1] + 20, depth_text,
                    color=depth_color, scale=2)

    # 显示三维坐标
    if coord is not None:
        img.draw_string(INFO_POS[0], INFO_POS[1] + 40,
                        "3D:[{},{},{}]".format(coord[0], coord[1], coord[2]),
                        color=COLOR_YELLOW, scale=2)

    # 右上角显示FPS
    img.draw_string(FPS_POS[0], FPS_POS[1], "FPS:{:.1f}".format(fps_val),
                    color=COLOR_YELLOW, scale=2)


def main():
    sensor = Sensor(width=CAMERA_WIDTH, height=CAMERA_HEIGHT)
    sensor.reset()
    sensor.set_framesize(Sensor.QVGA)
    sensor.set_pixformat(Sensor.RGB565)

    Display.init(Display.VIRT, width=CAMERA_WIDTH, height=CAMERA_HEIGHT, fps=30)
    MediaManager.init()
    sensor.run()
    clock = time.clock()

    try:
        while True:
            os.exitpoint()
            clock.tick()
            img = sensor.snapshot()

            # 检测红色blob
            blobs = img.find_blobs([RED_THRESHOLD], pixels_threshold=MIN_PIXELS)

            if blobs:
                # 取最大的红色blob
                target = max(blobs, key=lambda b: b.area())

                # 深度检测
                depth_status = check_depth(target, INNER_RADIUS, OUTER_RADIUS)

                # 计算三维坐标
                coord = compute_3d_coord(target, depth_status, img.width(), img.height())

                # 绘制
                draw_detection(img, target, depth_status, clock.fps(), coord)

                # 打印信息（终端输出）
                print("X:{} Y:{} 深度:{} 3D坐标:[{},{},{}] FPS:{:.1f}".format(
                    target.cx(), target.cy(), depth_status,
                    coord[0], coord[1], coord[2], clock.fps()))
            else:
                # 未检测到红色球体，仍然绘制圆形框
                cx = img.width() // 2
                cy = img.height() // 2
                img.draw_circle(cx, cy, INNER_RADIUS, color=COLOR_GREEN, thickness=2)
                img.draw_circle(cx, cy, OUTER_RADIUS, color=COLOR_RED, thickness=2)
                img.draw_string(INFO_POS[0], INFO_POS[1], "未检测到红色球体",
                                color=COLOR_RED, scale=2)
                img.draw_string(FPS_POS[0], FPS_POS[1], "FPS:{:.1f}".format(clock.fps()),
                                color=COLOR_YELLOW, scale=2)
                print("未检测到红色球体 FPS:{:.1f}".format(clock.fps()))

            Display.show_image(img)

    except KeyboardInterrupt:
        print("用户停止")
    except BaseException as e:
        print("异常: {}".format(e))
    finally:
        if isinstance(sensor, Sensor):
            sensor.stop()
        Display.deinit()
        os.exitpoint(os.EXITPOINT_ENABLE_SLEEP)
        time.sleep_ms(100)
        MediaManager.deinit()


if __name__ == "__main__":
    main()
