"""Minimal CanMV / MicroPython photo capture application for K230."""

import os
import time

from config import (
    AUTO_CAPTURE_INTERVAL_MS,
    AUTO_PREFIX,
    CAPTURE_FRAMESIZE,
    ENABLE_AUTO_CAPTURE,
    ENABLE_KEY_CAPTURE,
    ENABLE_PREVIEW,
    JPEG_QUALITY,
    KEY0_PIN,
    KEY0_PREFIX,
    KEY1_PIN,
    KEY1_PREFIX,
    PIXFORMAT,
    PREVIEW_FRAMESIZE,
    SAVE_DIR_CANDIDATES,
)

try:
    import sensor
except ImportError:  # pragma: no cover - device module only.
    sensor = None

try:
    import display
except ImportError:  # pragma: no cover - device module only.
    display = None

try:
    from machine import Pin
except ImportError:  # pragma: no cover - device module only.
    Pin = None


def _ticks_ms():
    if hasattr(time, "ticks_ms"):
        return time.ticks_ms()
    return int(time.time() * 1000)


def _ticks_diff(current, previous):
    if hasattr(time, "ticks_diff"):
        return time.ticks_diff(current, previous)
    return current - previous


def _sleep_ms(milliseconds):
    if hasattr(time, "sleep_ms"):
        time.sleep_ms(milliseconds)
        return
    time.sleep(milliseconds / 1000.0)


def _resolve_framesize(value):
    if sensor is None:
        return value
    if isinstance(value, str):
        return getattr(sensor, value, value)
    return value


def _safe_call(target, method_name, *args, **kwargs):
    method = getattr(target, method_name, None)
    if method is None:
        return None
    return method(*args, **kwargs)


def _resolve_save_dir():
    for candidate in SAVE_DIR_CANDIDATES:
        try:
            os.makedirs(candidate)
            return candidate
        except OSError:
            continue
    raise RuntimeError("Unable to create a writable photo directory")


def _timestamp_token():
    now = time.localtime()
    return "{:04d}{:02d}{:02d}_{:02d}{:02d}{:02d}".format(
        now[0], now[1], now[2], now[3], now[4], now[5]
    )


class PhotoCaptureApp:
    """Capture photos from the sensor while keeping preview running."""

    def __init__(self):
        self.save_dir = _resolve_save_dir()
        self.sequence = 0
        self.last_auto_capture_ms = 0
        self.key0 = None
        self.key1 = None
        self.preview_framesize = _resolve_framesize(PREVIEW_FRAMESIZE)
        self.capture_framesize = _resolve_framesize(CAPTURE_FRAMESIZE)

    def setup(self):
        if sensor is None:
            raise RuntimeError("sensor module is not available")

        _safe_call(sensor, "reset")
        _safe_call(sensor, "set_pixformat", _resolve_framesize(PIXFORMAT))
        _safe_call(sensor, "set_framesize", self.preview_framesize)
        _safe_call(sensor, "skip_frames", time=2000)
        _safe_call(sensor, "skip_frames", n=10)

        if display is not None:
            self._setup_display()

        self.key0 = self._setup_key(KEY0_PIN)
        self.key1 = self._setup_key(KEY1_PIN)
        self.last_auto_capture_ms = _ticks_ms()

    def _setup_display(self):
        init = getattr(display, "init", None)
        if init is None:
            return

        attempts = [
            lambda: init(getattr(display, "ST7701", None), to_ide=True),
            lambda: init(type=getattr(display, "ST7701", None), to_ide=True),
            lambda: init(to_ide=True),
            lambda: init(),
        ]
        for attempt in attempts:
            try:
                attempt()
                return
            except Exception:
                continue

    def _setup_key(self, pin_value):
        if not ENABLE_KEY_CAPTURE or Pin is None or pin_value is None:
            return None

        attempts = []
        if isinstance(pin_value, str):
            attempts.append(lambda: Pin(pin_value, Pin.IN, Pin.PULL_UP))
            attempts.append(lambda: Pin(pin_value))
        else:
            attempts.append(lambda: Pin(pin_value, Pin.IN, Pin.PULL_UP))
            attempts.append(lambda: Pin(pin_value))

        for attempt in attempts:
            try:
                return attempt()
            except Exception:
                continue
        return None

    def _read_key_pressed(self, key):
        if key is None:
            return False
        try:
            return key.value() == 0
        except Exception:
            return False

    def _next_filename(self, prefix, suffix):
        self.sequence += 1
        filename = "{}_{}_{}.{}".format(
            _timestamp_token(), prefix, "{:03d}".format(self.sequence), suffix
        )
        return os.path.join(self.save_dir, filename)

    def _save_frame(self, frame, prefix, suffix):
        path = self._next_filename(prefix, suffix)
        if suffix.lower() == "jpg":
            frame.save(path, quality=JPEG_QUALITY)
        else:
            frame.save(path)
        return path

    def _show_preview(self, frame):
        if not ENABLE_PREVIEW or display is None:
            return

        show_image = getattr(display, "show_image", None)
        if show_image is not None:
            show_image(frame)
            return

        show = getattr(display, "show", None)
        if show is not None:
            show(frame)

    def _process_frame(self, frame):
        return frame

    def _capture_frame(self):
        if self.capture_framesize != self.preview_framesize:
            _safe_call(sensor, "set_framesize", self.capture_framesize)
            _safe_call(sensor, "skip_frames", time=200)

        frame = sensor.snapshot()

        if self.capture_framesize != self.preview_framesize:
            _safe_call(sensor, "set_framesize", self.preview_framesize)
            _safe_call(sensor, "skip_frames", time=200)

        if frame is None:
            return None
        return self._process_frame(frame)

    def capture_jpg(self, frame, prefix=AUTO_PREFIX):
        return self._save_frame(frame, prefix, "jpg")

    def capture_bmp(self, frame, prefix=AUTO_PREFIX):
        return self._save_frame(frame, prefix, "bmp")

    def run_once(self):
        frame = self._capture_frame()
        if frame is None:
            return

        self._show_preview(frame)

        if self._read_key_pressed(self.key0):
            self.capture_jpg(frame, KEY0_PREFIX)
        elif self._read_key_pressed(self.key1):
            self.capture_bmp(frame, KEY1_PREFIX)

        if ENABLE_AUTO_CAPTURE:
            now = _ticks_ms()
            if _ticks_diff(now, self.last_auto_capture_ms) >= AUTO_CAPTURE_INTERVAL_MS:
                self.capture_jpg(frame, AUTO_PREFIX)
                self.last_auto_capture_ms = now

    def run_forever(self):
        self.setup()
        while True:
            self.run_once()
            _sleep_ms(10)


def main():
    app = PhotoCaptureApp()
    app.run_forever()


if __name__ == "__main__":
    main()
