import os
import time

from media.display import *
from media.media import *
from media.sensor import *

from camera_params import CAMERA_MATRIX
from config import (
    DRAW_DEBUG_OVERLAY,
    EMA_ALPHA,
    ENABLE_SMOOTHING,
    FRAME_SIZE,
    LINE_WIDTH_CM,
    MAX_ASPECT_RATIO_DEVIATION,
    MAX_CAMERA_Z_CM,
    MAX_RECT_AREA_PX,
    MAX_REPROJECTION_ERROR_PX,
    MAX_SIDE_RELATIVE_ERROR,
    MIN_CAMERA_Z_CM,
    MIN_MAGNITUDE,
    MIN_RECT_AREA_PX,
    PIXFORMAT,
    PRINT_EVERY_N_FRAMES,
    RECT_THRESHOLD,
    ROI,
    SQUARE_SIZE_CM,
    USE_LENS_CORR,
    USE_ROI,
    LENS_CORR_STRENGTH,
)
from geometry import (
    ema_points,
    is_square_like,
    order_corners_tl_tr_br_bl,
    solve_pose_from_square,
    square_world_to_camera_points,
)


def _sensor_framesize_from_name(name):
    return getattr(Sensor, name, Sensor.QVGA)


def _sensor_pixformat_from_name(name):
    return getattr(Sensor, name, Sensor.GRAYSCALE)


def _rect_area(rect_obj):
    return float(rect_obj.w()) * float(rect_obj.h())


def _rect_magnitude(rect_obj):
    try:
        return float(rect_obj.magnitude())
    except Exception:
        return 0.0


def _draw_text(img, x, y, text, color=(255, 255, 255)):
    try:
        img.draw_string_advanced(x, y, 20, text, color=color, thickness=2)
    except Exception:
        img.draw_string(x, y, text, color=color)


def _draw_candidate(img, ordered, color=(0, 255, 0)):
    for idx, p in enumerate(ordered):
        x = int(p[0])
        y = int(p[1])
        img.draw_circle(x, y, 4, color=color)
        _draw_text(img, x + 3, y + 3, str(idx), color=color)


def _format_corner_line(tag, point3d):
    return "%s:(%.2f,%.2f,%.2f)cm" % (tag, point3d[0], point3d[1], point3d[2])


def _pick_best_square(rects):
    best = None
    best_score = -1.0

    for r in rects:
        area = _rect_area(r)
        if area < MIN_RECT_AREA_PX or area > MAX_RECT_AREA_PX:
            continue

        mag = _rect_magnitude(r)
        if mag < MIN_MAGNITUDE:
            continue

        ordered = order_corners_tl_tr_br_bl(r.corners())
        if not is_square_like(ordered, MAX_ASPECT_RATIO_DEVIATION, MAX_SIDE_RELATIVE_ERROR):
            continue

        # Score by edge magnitude with mild area preference.
        score = mag + 0.002 * area
        if score > best_score:
            best_score = score
            best = (r, ordered, score)

    return best


def main():
    sensor = None
    smoothed_points = None
    frame_id = 0

    try:
        sensor = Sensor(width=1280, height=960)
        sensor.reset()
        sensor.set_framesize(_sensor_framesize_from_name(FRAME_SIZE))
        sensor.set_pixformat(_sensor_pixformat_from_name(PIXFORMAT))

        Display.init(Display.ST7701, width=640, height=480, fps=90, to_ide=True)
        MediaManager.init()
        sensor.run()

        clock = time.clock()

        print("Square frame detector started")
        print("Target: side=%.2fcm, line_width=%.2fcm" % (SQUARE_SIZE_CM, LINE_WIDTH_CM))

        while True:
            os.exitpoint()
            clock.tick()
            frame_id += 1

            img = sensor.snapshot()
            if USE_LENS_CORR:
                try:
                    img.lens_corr(strength=LENS_CORR_STRENGTH)
                except Exception:
                    pass

            if USE_ROI:
                rects = img.find_rects(threshold=RECT_THRESHOLD, roi=ROI)
                if DRAW_DEBUG_OVERLAY:
                    img.draw_rectangle(ROI, color=(255, 255, 0))
            else:
                rects = img.find_rects(threshold=RECT_THRESHOLD)

            best = _pick_best_square(rects)
            status = "NO_TARGET"

            if best is not None:
                _, ordered, _ = best

                rmat, tvec, reproj_err = solve_pose_from_square(
                    ordered,
                    CAMERA_MATRIX,
                    SQUARE_SIZE_CM,
                )

                if (
                    rmat is not None
                    and tvec is not None
                    and reproj_err is not None
                    and reproj_err <= MAX_REPROJECTION_ERROR_PX
                    and MIN_CAMERA_Z_CM <= tvec[2] <= MAX_CAMERA_Z_CM
                ):
                    corners_cam = square_world_to_camera_points(rmat, tvec, SQUARE_SIZE_CM)
                    if ENABLE_SMOOTHING:
                        corners_cam = ema_points(smoothed_points, corners_cam, EMA_ALPHA)
                    smoothed_points = corners_cam

                    status = "OK"
                    if DRAW_DEBUG_OVERLAY:
                        _draw_candidate(img, ordered, color=(0, 255, 0))
                        _draw_text(img, 4, 4, "status=OK err=%.2fpx" % reproj_err, color=(0, 255, 0))
                        _draw_text(img, 4, 24, _format_corner_line("TL", corners_cam[0]))
                        _draw_text(img, 4, 42, _format_corner_line("TR", corners_cam[1]))
                        _draw_text(img, 4, 60, _format_corner_line("BR", corners_cam[2]))
                        _draw_text(img, 4, 78, _format_corner_line("BL", corners_cam[3]))
                else:
                    smoothed_points = None
                    status = "UNRELIABLE"
                    if DRAW_DEBUG_OVERLAY:
                        _draw_candidate(img, ordered, color=(255, 80, 80))
                        err_text = reproj_err if reproj_err is not None else -1.0
                        _draw_text(img, 4, 4, "status=UNRELIABLE err=%.2f" % err_text, color=(255, 80, 80))
            else:
                smoothed_points = None
                if DRAW_DEBUG_OVERLAY:
                    _draw_text(img, 4, 4, "status=NO_TARGET", color=(255, 255, 0))

            if smoothed_points is not None and frame_id % PRINT_EVERY_N_FRAMES == 0:
                tl, tr, br, bl = smoothed_points
                print(
                    "%d,%s,TL(%.2f %.2f %.2f),TR(%.2f %.2f %.2f),BR(%.2f %.2f %.2f),BL(%.2f %.2f %.2f)"
                    % (
                        frame_id,
                        status,
                        tl[0],
                        tl[1],
                        tl[2],
                        tr[0],
                        tr[1],
                        tr[2],
                        br[0],
                        br[1],
                        br[2],
                        bl[0],
                        bl[1],
                        bl[2],
                    )
                )
            elif frame_id % PRINT_EVERY_N_FRAMES == 0:
                print("%d,%s" % (frame_id, status))

            Display.show_image(
                img,
                x=round((640 - sensor.width()) / 2),
                y=round((480 - sensor.height()) / 2),
            )

    except KeyboardInterrupt as e:
        print("user stop:", e)
    except BaseException as e:
        print("Exception", e)
    finally:
        if isinstance(sensor, Sensor):
            sensor.stop()
        Display.deinit()
        os.exitpoint(os.EXITPOINT_ENABLE_SLEEP)
        time.sleep_ms(100)
        MediaManager.deinit()


if __name__ == "__main__":
    main()
