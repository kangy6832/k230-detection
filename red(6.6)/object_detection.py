

import time, os
from media.sensor import *
from media.display import *
from media.media import *
from machine import UART, FPIOA

# ============================================================
#  比赛时切换模式
# ============================================================
MODE = 2

# ============================================================
#  硬件参数
# ============================================================
FAST_PREVIEW = False   # False   True
if FAST_PREVIEW:
    PICTURE_WIDTH = 400
    PICTURE_HEIGHT = 240
else:
    PICTURE_WIDTH = 640
    PICTURE_HEIGHT = 480

DISPLAY_WIDTH = 640
DISPLAY_HEIGHT = 480
UART_BAUDRATE = 115200
SEND_EVERY_N_FRAMES = 2
DISPLAY_TO_IDE = True

# ============================================================
#  串口协议 AXYZxyzB
# ============================================================
cmd = [0, 0, 0, 0, 0, 0]   # [前, 左, 上, 后, 右, 下]
uart = None


def build_packet():
    return "A%d%d%d%d%d%dB" % (cmd[0], cmd[1], cmd[2], cmd[3], cmd[4], cmd[5])


def send_packet(frame_id=0):
    global uart
    if uart is None:
        return
    if frame_id % SEND_EVERY_N_FRAMES != 0:
        return
    try:
        uart.write(build_packet())
    except Exception:
        pass


def set_move(forward=False, backward=False,
             left=False, right=False,
             up=False, down=False):
    global cmd
    cmd = [0, 0, 0, 0, 0, 0]
    if forward:
        cmd[0] = 1
    if left:
        cmd[1] = 1
    if up:
        cmd[2] = 1
    if backward:
        cmd[3] = 1
        cmd[0] = 1
    if right:
        cmd[4] = 1
        cmd[1] = 1
    if down:
        cmd[5] = 1
        cmd[2] = 1


def set_stop():
    global cmd
    cmd = [0, 0, 0, 0, 0, 0]


# ============================================================
#  红色目标参数（必须按实机标定）
# ============================================================
END_OFFSET_X = -10
END_OFFSET_Y = 0
CENTER_X = PICTURE_WIDTH // 2 + END_OFFSET_X
CENTER_Y = PICTURE_HEIGHT // 2 + END_OFFSET_Y
DEADZONE_XY = 25
TARGET_AREA =   520000
DEADZONE_AREA = 500000
SHOW_CALIBRATION_HINT = True

# ============================================================
#  黑色方框参数
# ============================================================
BLACK_LAB = (0, 45, -128, 127, -128, 127)
BLACK_MIN_PX = 25
BLACK_MAX_PX = 180
BLACK_ASPECT_MIN = 0.75
BLACK_DENSITY_MIN = 0.08
BLACK_DENSITY_MAX = 0.45
BLACK_BLOB_MIN = 80
PIXELS_PER_CM = 8.0
TARGET_CM = 10.0
BLACK_CENTER_DEADZONE = 18
BLACK_SIZE_TOL_CM = 1.5

# ============================================================
#  红色目标参数
# ============================================================
RED_LAB = (16, 44, 14, 40, 9, 25)
RED_MIN_PX = 80
RED_MAX_PX_DETECT = 1000
RED_MAX_PX_TRACK = 3000
RED_DENSITY_MIN = 0.15
RED_BLOB_MIN = 40
RED_LOST_TOLERANCE = 3

last_red_r = None
red_lost_count = 0
red_hold_active = False


# ============================================================
#  初始化
# ============================================================
def init_sensor():
    sensor = Sensor()
    sensor.reset()
    sensor.set_framesize(width=PICTURE_WIDTH, height=PICTURE_HEIGHT, chn=CAM_CHN_ID_0)
    sensor.set_pixformat(Sensor.RGB565, chn=CAM_CHN_ID_0)
    return sensor


def init_display():
    Display.init(Display.ST7701, width=DISPLAY_WIDTH, height=DISPLAY_HEIGHT, to_ide=DISPLAY_TO_IDE)
    MediaManager.init()


def init_uart():
    global uart
    try:
        fpioa = FPIOA()
        fpioa.set_function(40, FPIOA.UART1_TXD)
        fpioa.set_function(41, FPIOA.UART1_RXD)
        uart = UART(UART.UART1, baudrate=UART_BAUDRATE,
                     bits=UART.EIGHTBITS, parity=UART.PARITY_NONE,
                     stop=UART.STOPBITS_ONE)
        print("[UART] init ok")
    except Exception as e:
        uart = None
        print("[UART] init fail:", e)


# ============================================================
#  检测函数
# ============================================================
def detect_red_object(img, max_px=RED_MAX_PX_DETECT):
    blobs = img.find_blobs([RED_LAB], pixels_threshold=RED_BLOB_MIN,
                           area_threshold=RED_BLOB_MIN, merge=True)
    candidates = []
    for blob in blobs:
        w = blob.w()
        h = blob.h()
        if not (RED_MIN_PX <= w <= max_px and RED_MIN_PX <= h <= max_px):
            continue
        area = w * h
        density = blob.pixels() / area if area > 0 else 0
        if density < RED_DENSITY_MIN:
            continue
        candidates.append({
            'cx': blob.cx(), 'cy': blob.cy(),
            'w': w, 'h': h,
            'pixels': blob.pixels(),
            'density': density
        })
    if candidates:
        candidates.sort(key=lambda x: x['pixels'], reverse=True)
        return candidates[0]
    return None


def detect_black_rect(img):
    blobs = img.find_blobs([BLACK_LAB], pixels_threshold=BLACK_BLOB_MIN,
                           area_threshold=BLACK_BLOB_MIN, merge=True)
    candidates = []
    for blob in blobs:
        w = blob.w()
        h = blob.h()
        if not (BLACK_MIN_PX <= w <= BLACK_MAX_PX and BLACK_MIN_PX <= h <= BLACK_MAX_PX):
            continue
        aspect = min(w, h) / max(w, h)
        if aspect < BLACK_ASPECT_MIN:
            continue
        area = w * h
        density = blob.pixels() / area if area > 0 else 0
        if density < BLACK_DENSITY_MIN or density > BLACK_DENSITY_MAX:
            continue

        pw = w / PIXELS_PER_CM
        ph = h / PIXELS_PER_CM
        size_err = abs(pw - TARGET_CM) + abs(ph - TARGET_CM)
        shape_err = (1.0 - aspect) * 10
        score = size_err + shape_err

        candidates.append({
            'cx': blob.cx(), 'cy': blob.cy(),
            'w': w, 'h': h,
            'density': density,
            'physical_w': pw,
            'physical_h': ph,
            'score': score
        })

    if candidates:
        candidates.sort(key=lambda x: x['score'])
        return candidates[0]
    return None


def get_stable_red_object(img, max_px=RED_MAX_PX_DETECT):
    global last_red_r, red_lost_count, red_hold_active

    red_r = detect_red_object(img, max_px=max_px)
    if red_r is not None:
        last_red_r = red_r
        red_lost_count = 0
        red_hold_active = False
        return red_r

    if last_red_r is not None and red_lost_count < RED_LOST_TOLERANCE:
        red_lost_count += 1
        red_hold_active = True
        return last_red_r

    last_red_r = None
    red_lost_count = 0
    red_hold_active = False
    return None


# ============================================================
#  控制逻辑
# ============================================================
def track_red_object(red_r):
    if red_r is None:
        return (False, False, False, False, False, False, False)

    dx = red_r['cx'] - CENTER_X
    dy = red_r['cy'] - CENTER_Y
    area = red_r['pixels']

    go_left = dx < -DEADZONE_XY
    go_right = dx > DEADZONE_XY
    go_up = dy < -DEADZONE_XY
    go_down = dy > DEADZONE_XY
    go_forward = area < (TARGET_AREA - DEADZONE_AREA)
    go_backward = area > (TARGET_AREA + DEADZONE_AREA)

    aligned = (not go_left and not go_right and
               not go_up and not go_down and
               not go_forward and not go_backward)

    return (go_forward, go_backward, go_left, go_right, go_up, go_down, aligned)


def track_black_rect(black_r):
    if black_r is None:
        return (False, False, False, False, False, False, False)

    dx = black_r['cx'] - CENTER_X
    dy = black_r['cy'] - CENTER_Y

    go_left = dx < -BLACK_CENTER_DEADZONE
    go_right = dx > BLACK_CENTER_DEADZONE
    go_up = dy < -BLACK_CENTER_DEADZONE
    go_down = dy > BLACK_CENTER_DEADZONE

    size_ok = (abs(black_r['physical_w'] - TARGET_CM) <= BLACK_SIZE_TOL_CM and
               abs(black_r['physical_h'] - TARGET_CM) <= BLACK_SIZE_TOL_CM)

    centered = (not go_left and not go_right and not go_up and not go_down)
    go_forward = centered and (not size_ok)
    aligned = centered and size_ok

    return (go_forward, False, go_left, go_right, go_up, go_down, aligned)


# ============================================================
#  绘图
# ============================================================
def draw_red(img, r):
    if r is None:
        return
    cx = int(r['cx'])
    cy = int(r['cy'])
    w = int(r['w'])
    h = int(r['h'])
    img.draw_rectangle(cx - w // 2, cy - h // 2, w, h, color=(255, 0, 0), thickness=2)
    img.draw_cross(cx, cy, color=(255, 0, 0), thickness=2)
    img.draw_string_advanced(max(0, cx - 40), max(0, cy - h // 2 - 18), 20,
                             "R %d" % r['pixels'], color=(255, 0, 0))


def draw_black(img, r):
    if r is None:
        return
    cx = int(r['cx'])
    cy = int(r['cy'])
    w = int(r['w'])
    h = int(r['h'])
    img.draw_rectangle(cx - w // 2, cy - h // 2, w, h, color=(0, 255, 0), thickness=2)
    img.draw_cross(cx, cy, color=(0, 255, 0), thickness=2)
    img.draw_string_advanced(max(0, cx - 50), max(0, cy - h // 2 - 18), 20,
                             "B %.1f %.1f" % (r['physical_w'], r['physical_h']), color=(0, 255, 0))


def draw_tracking_overlay(img, red_r):
    img.draw_cross(CENTER_X, CENTER_Y, color=(255, 255, 0), thickness=1)
    img.draw_rectangle(CENTER_X - DEADZONE_XY, CENTER_Y - DEADZONE_XY,
                       DEADZONE_XY * 2, DEADZONE_XY * 2,
                       color=(255, 255, 0), thickness=1)
    if red_r:
        dx = red_r['cx'] - CENTER_X
        dy = red_r['cy'] - CENTER_Y
        img.draw_line(CENTER_X, CENTER_Y, int(red_r['cx']), int(red_r['cy']), color=(255, 255, 0), thickness=1)
        img.draw_string_advanced(0, 0, 20,
                                 "cx:%d cy:%d a:%d" % (red_r['cx'], red_r['cy'], red_r['pixels']),
                                 color=(255, 255, 255))
        img.draw_string_advanced(0, 22, 20,
                                 "dx:%d dy:%d ta:%d" % (dx, dy, TARGET_AREA),
                                 color=(255, 255, 0))
        if red_hold_active:
            img.draw_string_advanced(0, 44, 20, "RED HOLD", color=(255, 180, 0))
        elif SHOW_CALIBRATION_HINT:
            img.draw_string_advanced(0, 44, 20, "mark CX CY AREA", color=(255, 220, 120))


# ============================================================
#  主程序
# ============================================================
def main():
    global last_red_r, red_lost_count, red_hold_active
    sensor = None
    try:
        sensor = init_sensor()
        init_display()
        init_uart()
        sensor.run()

        clock = time.clock()
        red_max_px = RED_MAX_PX_TRACK if MODE == 2 else RED_MAX_PX_DETECT
        frame_id = 0
        last_red_r = None
        red_lost_count = 0
        red_hold_active = False

        print("=" * 36)
        print("K230 official framework version")
        print("MODE:", MODE)
        print("FAST_PREVIEW:", FAST_PREVIEW)
        print("CENTER:", CENTER_X, CENTER_Y)
        print("OFFSET:", END_OFFSET_X, END_OFFSET_Y)
        print("TARGET_AREA:", TARGET_AREA)
        print("=" * 36)

        while True:
            os.exitpoint()
            clock.tick()
            frame_id += 1
            img = sensor.snapshot(chn=CAM_CHN_ID_0)

            black_r = None
            red_r = None

            if MODE == 1 or MODE == 3:
                black_r = detect_black_rect(img)
                draw_black(img, black_r)

            if MODE == 2 or MODE == 3:
                red_r = get_stable_red_object(img, max_px=red_max_px)
                draw_red(img, red_r)

            if MODE == 1:
                fwd, bwd, lft, rgt, up, down, aligned = track_black_rect(black_r)
                set_move(forward=fwd, backward=bwd, left=lft, right=rgt, up=up, down=down)

            elif MODE == 2:
                fwd, bwd, lft, rgt, up, down, aligned = track_red_object(red_r)
                set_move(forward=fwd, backward=bwd, left=lft, right=rgt, up=up, down=down)
                draw_tracking_overlay(img, red_r)

            else:
                if red_r:
                    fwd, bwd, lft, rgt, up, down, aligned = track_red_object(red_r)
                    set_move(forward=fwd, backward=bwd, left=lft, right=rgt, up=up, down=down)
                    draw_tracking_overlay(img, red_r)
                elif black_r:
                    fwd, bwd, lft, rgt, up, down, aligned = track_black_rect(black_r)
                    set_move(forward=fwd, backward=bwd, left=lft, right=rgt, up=up, down=down)
                else:
                    set_stop()

            send_packet(frame_id)

            packet = build_packet()
            fps = clock.fps()
            img.draw_string_advanced(0, PICTURE_HEIGHT - 24, 20,
                                     "FPS:%.1f %s" % (fps, packet), color=(255, 255, 255))

            if MODE == 2 and red_r:
                dx = red_r['cx'] - CENTER_X
                dy = red_r['cy'] - CENTER_Y
                area = red_r['pixels']
                xy_ok = abs(dx) <= DEADZONE_XY and abs(dy) <= DEADZONE_XY
                depth_ok = (TARGET_AREA - DEADZONE_AREA) <= area <= (TARGET_AREA + DEADZONE_AREA)
                status = "ALIGNED<=5cm" if (xy_ok and depth_ok) else "TRACKING"
                img.draw_string_advanced(0, PICTURE_HEIGHT - 48, 20, status, color=(0, 255, 255))

            x = int((DISPLAY_WIDTH - PICTURE_WIDTH) / 2)
            y = int((DISPLAY_HEIGHT - PICTURE_HEIGHT) / 2)
            Display.show_image(img, x=x, y=y)

    except KeyboardInterrupt as e:
        print("User Stop:", e)
    except Exception as e:
        print("Exception:", e)
    finally:
        set_stop()
        send_packet()
        if sensor and isinstance(sensor, Sensor):
            sensor.stop()
        Display.deinit()
        os.exitpoint(os.EXITPOINT_ENABLE_SLEEP)
        time.sleep_ms(100)
        MediaManager.deinit()


if __name__ == "__main__":
    main()
