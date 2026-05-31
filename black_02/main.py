import time, os, sys
from media.sensor import *
from media.display import *
from media.media import *
from machine import Pin
from machine import FPIOA

# ===================== 按键配置 =====================
fpioa = FPIOA()
fpioa.set_function(35, FPIOA.GPIO35)  # key1
fpioa.set_function(0, FPIOA.GPIO0)    # key2
key1 = Pin(35, Pin.IN, pull=Pin.PULL_UP, drive=7)   # 按下低电平
key2 = Pin(0, Pin.IN, pull=Pin.PULL_DOWN, drive=7)  # 按下高电平

# ===================== 摄像头/显示配置 =====================
CAMERA_WIDTH = 320
CAMERA_HEIGHT = 240

# ===================== key1: 中心黑色检测参数 =====================
GRAY_MAX = 64
CENTER_BOX_SIZE = 30
DIRECTION_BOX_SIZE = 30
DIRECTION_BLACK_RATIO = 0.10

# ===================== key2: 红色块检测参数 =====================
RED_THRESHOLD = (22, 37, -128, 127, -128, 127)
INNER_RADIUS = 30
OUTER_RADIUS = 60
INNER_RED_RATIO = 0.80
MIN_PIXELS = 100

# ===================== 按键消抖 =====================
KEY_DEBOUNCE_MS = 200
last_key_tick = 0

def check_key(current_tick):
    """检测按键并返回模式切换。返回值: 0=无切换, 1=key1按下, 2=key2按下"""
    global last_key_tick
    if current_tick - last_key_tick < KEY_DEBOUNCE_MS:
        return 0
    if key1.value() == 0:
        last_key_tick = current_tick
        return 1
    if key2.value() == 1:
        last_key_tick = current_tick
        return 2
    return 0

# ===================== key1: 中心黑色检测 =====================
def get_center_box_roi(img_w, img_h, box_size):
    cx = img_w // 2
    cy = img_h // 2
    half = box_size // 2
    return (cx - half, cy - half, box_size, box_size)

def get_direction_rois(img_w, img_h, center_size, direction_size):
    cx = img_w // 2
    cy = img_h // 2
    chalf = center_size // 2
    center_left = cx - chalf
    center_right = cx + chalf
    center_top = cy - chalf
    center_bottom = cy + chalf
    return {
        "up": (center_left, center_top - direction_size, direction_size, direction_size),
        "down": (center_left, center_bottom, direction_size, direction_size),
        "left": (center_left - direction_size, center_top, direction_size, direction_size),
        "right": (center_right, center_top, direction_size, direction_size),
    }

def check_center_box_white(img, roi):
    blobs = img.find_blobs([(255, 255)], roi=roi, pixels_threshold=5, merge=True)
    for blob in blobs:
        if blob.pixels() > 0:
            return True
    return False

def check_direction_black(img, roi, black_ratio):
    w = roi[2]
    h = roi[3]
    total_pixels = w * h
    if total_pixels == 0:
        return False
    white_blobs = img.find_blobs([(255, 255)], roi=roi, pixels_threshold=1, merge=True)
    white_pixels = 0
    for blob in white_blobs:
        white_pixels += blob.pixels()
    black_pixels = total_pixels - white_pixels
    ratio = black_pixels / total_pixels
    return ratio >= black_ratio

def calculate_target_offset(has_white, direction_results):
    target = [0.0, 0.0, 0.0]
    up = direction_results["up"]
    down = direction_results["down"]
    left = direction_results["left"]
    right = direction_results["right"]
    if has_white:
        if up:
            target[1] = -0.01
        if down:
            target[1] = 0.01
        if left:
            target[0] = 0.01
        if right:
            target[0] = -0.01
    else:
        if up and down:
            target[1] = 0.01
        if left and right:
            target[0] = 0.01
        if up and right:
            target[1] = 0.01
        if up and left:
            target[0] = -0.01
        if down and left:
            target[1] = -0.01
        if down and right:
            target[0] = 0.01
    return target

def run_center_black_check(img, center_roi, direction_rois, clock):
    img_bin = img.binary([(GRAY_MAX, 255)])
    has_white = check_center_box_white(img_bin, center_roi)
    direction_results = {}
    for name, roi in direction_rois.items():
        direction_results[name] = check_direction_black(img_bin, roi, DIRECTION_BLACK_RATIO)
    target = calculate_target_offset(has_white, direction_results)
    cx = CAMERA_WIDTH // 2
    cy = CAMERA_HEIGHT // 2
    center_color = (255, 0, 0) if has_white else (0, 255, 0)
    img.draw_rectangle(center_roi, color=center_color, thickness=2)
    direction_colors = {
        "up": (0, 255, 0) if direction_results["up"] else (255, 0, 0),
        "down": (0, 255, 0) if direction_results["down"] else (255, 0, 0),
        "left": (0, 255, 0) if direction_results["left"] else (255, 0, 0),
        "right": (0, 255, 0) if direction_results["right"] else (255, 0, 0),
    }
    for name, roi in direction_rois.items():
        img.draw_rectangle(roi, color=direction_colors[name], thickness=1)
    img.draw_cross(cx, cy, color=center_color, size=5, thickness=1)
    img.draw_string(2, 2, "FPS: %.1f" % clock.fps(), color=255, scale=1)
    result_str = "W:%s U:%s D:%s L:%s R:%s" % (
        "T" if has_white else "F",
        "T" if direction_results["up"] else "F",
        "T" if direction_results["down"] else "F",
        "T" if direction_results["left"] else "F",
        "T" if direction_results["right"] else "F",
    )
    img.draw_string(2, CAMERA_HEIGHT - 30, result_str, color=255, scale=1)
    img.draw_string(2, CAMERA_HEIGHT - 16,
        "T:[%.2f,%.2f,%.2f]" % (target[0], target[1], target[2]),
        color=255, scale=1)
    print("center_white=%s up=%s down=%s left=%s right=%s target=[%.2f,%.2f,%.2f] fps=%.1f" % (
        has_white,
        direction_results["up"],
        direction_results["down"],
        direction_results["left"],
        direction_results["right"],
        target[0], target[1], target[2],
        clock.fps(),
    ))

# ===================== key2: 红色块检测 =====================
def check_depth(blob, inner_r, outer_r):
    blob_area = blob.area()
    blob_equiv_r = (blob_area / 3.14159) ** 0.5
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
    cx = img_w // 2
    cy = img_h // 2
    dx = blob.cx() - cx
    dy = blob.cy() - cy
    if -10 <= dx <= 10:
        x_val = 0.0
    elif dx < -10:
        x_val = 0.01
    else:
        x_val = -0.01
    if -10 <= dy <= 10:
        y_val = 0.0
    elif dy < -10:
        y_val = 0.01
    else:
        y_val = -0.01
    if depth_status is True:
        z_val = 0.0
    elif depth_status == "近了":
        z_val = -0.01
    else:
        z_val = 0.01
    return [x_val, y_val, z_val]

def run_red_block_detection(img, clock):
    blobs = img.find_blobs([RED_THRESHOLD], pixels_threshold=MIN_PIXELS)
    cx = img.width() // 2
    cy = img.height() // 2
    if blobs:
        target = max(blobs, key=lambda b: b.area())
        depth_status = check_depth(target, INNER_RADIUS, OUTER_RADIUS)
        coord = compute_3d_coord(target, depth_status, img.width(), img.height())
        img.draw_rectangle(target.rect(), color=(0, 0, 255), thickness=2)
        img.draw_cross(target.cx(), target.cy(), color=(255, 255, 0), size=10, thickness=2)
        img.draw_circle(cx, cy, INNER_RADIUS, color=(0, 255, 0), thickness=2)
        img.draw_circle(cx, cy, OUTER_RADIUS, color=(255, 0, 0), thickness=2)
        if depth_status is True:
            depth_text = "深度: 正确"
            depth_color = (0, 255, 0)
        elif depth_status == "近了":
            depth_text = "深度: 近了"
            depth_color = (255, 0, 0)
        else:
            depth_text = "深度: 远了"
            depth_color = (255, 0, 0)
        img.draw_string(5, 5, "X:{} Y:{}".format(target.cx(), target.cy()), color=(255, 255, 255), scale=2)
        img.draw_string(5, 25, depth_text, color=depth_color, scale=2)
        img.draw_string(5, 45, "3D:[{},{},{}]".format(coord[0], coord[1], coord[2]), color=(255, 255, 0), scale=2)
        img.draw_string(220, 5, "FPS:{:.1f}".format(clock.fps()), color=(255, 255, 0), scale=2)
        print("X:{} Y:{} 深度:{} 3D坐标:[{},{},{}] FPS:{:.1f}".format(
            target.cx(), target.cy(), depth_status,
            coord[0], coord[1], coord[2], clock.fps()))
    else:
        img.draw_circle(cx, cy, INNER_RADIUS, color=(0, 255, 0), thickness=2)
        img.draw_circle(cx, cy, OUTER_RADIUS, color=(255, 0, 0), thickness=2)
        img.draw_string(5, 5, "未检测到红色球体", color=(255, 0, 0), scale=2)
        img.draw_string(220, 5, "FPS:{:.1f}".format(clock.fps()), color=(255, 255, 0), scale=2)
        print("未检测到红色球体 FPS:{:.1f}".format(clock.fps()))

# ===================== 主程序 =====================
def main():
    sensor = Sensor(width=CAMERA_WIDTH, height=CAMERA_HEIGHT)
    sensor.reset()
    sensor.set_framesize(Sensor.QVGA)
    Display.init(Display.VIRT, width=CAMERA_WIDTH, height=CAMERA_HEIGHT, fps=30)
    MediaManager.init()
    sensor.run()
    clock = time.clock()

    # 默认模式: key1=中心黑色检测(GRAYSCALE), key2=红色块检测(RGB565)
    current_mode = 1
    sensor.set_pixformat(Sensor.GRAYSCALE)

    # 预计算 ROI
    center_roi = get_center_box_roi(CAMERA_WIDTH, CAMERA_HEIGHT, CENTER_BOX_SIZE)
    direction_rois = get_direction_rois(CAMERA_WIDTH, CAMERA_HEIGHT, CENTER_BOX_SIZE, DIRECTION_BOX_SIZE)

    try:
        while True:
            os.exitpoint()
            clock.tick()
            img = sensor.snapshot()

            # 检测按键切换模式
            key_result = check_key(time.ticks_ms())
            if key_result != 0 and key_result != current_mode:
                current_mode = key_result
                if current_mode == 1:
                    sensor.set_pixformat(Sensor.GRAYSCALE)
                    print("切换到: 中心黑色检测")
                elif current_mode == 2:
                    sensor.set_pixformat(Sensor.RGB565)
                    print("切换到: 红色块检测")

            # 根据当前模式执行检测
            if current_mode == 1:
                run_center_black_check(img, center_roi, direction_rois, clock)
                img.draw_string(CAMERA_WIDTH - 80, CAMERA_HEIGHT - 16, "[KEY1]黑线", color=255, scale=1)
            elif current_mode == 2:
                run_red_block_detection(img, clock)
                img.draw_string(CAMERA_WIDTH - 80, CAMERA_HEIGHT - 16, "[KEY2]红色", color=(255, 255, 0), scale=1)

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
