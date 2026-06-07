# CLAUDE.md

This file provides guidance to Claude Code (claude.ai/code) when working with code in this repository.

## Project Overview

**Black Line Following Detection** (黑色灰度循线) — A MicroPython embedded computer vision application for the K230D BOX development board by ALIENTEK (正点原子).

This is a **single-file project** (`main.py`) that tracks a black line on a light background using camera input and calculates a deflection angle for robotic navigation.

**No build system, package manager, or test framework.** Scripts run directly on the K230 board via CanMV firmware.

## Architecture

### Data Flow

```
Camera (VGA 640x480, Grayscale)
    ↓
ROI-based Blob Detection (3 horizontal regions)
    ↓
Centroid Calculation (weighted average)
    ↓
Deflection Angle (arctan transform)
    ↓
LCD Display + Serial Output
```

### Key Components

**1. Hardware Initialization**
- `Sensor(width=1280, height=960)` — Camera at VGA resolution
- `Display.init(Display.ST7701, ...)` — LCD output at 90 FPS
- `MediaManager.init()` — Buffer management

**2. ROI Configuration** (lines 44-48)
Three horizontal regions with weights:
- Bottom ROI (y=400): weight 0.7 — highest priority (closest to robot)
- Middle ROI (y=200): weight 0.3
- Top ROI (y=0): weight 0.1 — lowest priority (farthest from robot)

**3. Line Detection** (lines 72-84)
- `img.find_blobs(GRAYSCALE_THRESHOLD, roi=r[0:4], merge=True)` — Find black blobs
- Threshold: `[(0, 64)]` for black (use `[(128, 255)]` for white line tracking)
- Select largest blob by pixel count
- Calculate weighted centroid

**4. Angle Calculation** (lines 85-99)
```python
deflection_angle = -math.atan((center_pos - 320) / 240)
```
- Nonlinear response: stronger correction at larger offsets
- Output range: approximately -45° to +45°
- Left offset = positive angle, right offset = negative angle

### Resolution Modes

The code supports three resolutions (commented alternatives in lines 30-48):
- **QQVGA** (160x120): `center_pos - 80) / 60`
- **QVGA** (320x240): `center_pos - 160) / 120`
- **VGA** (640x480): `center_pos - 320) / 240` ← **Currently active**

## CanMV/K230 APIs Used

| Module | Purpose |
|--------|---------|
| `media.sensor` | `Sensor()`, `snapshot()`, `run()`, `stop()`, `set_framesize()`, `set_pixformat()` |
| `media.display` | `Display.init()`, `show_image()`, `deinit()` |
| `media.media` | `MediaManager.init()`, `deinit()` |
| `image` | `find_blobs()`, `draw_rectangle()`, `draw_cross()`, `draw_string_advanced()` |

## Development Workflow

1. Edit `main.py` in CanMV IDE or any editor
2. Deploy to K230 board via USB/serial connection
3. Observe LCD display for visual feedback
4. Monitor serial output for FPS and angle data
5. Adjust `GRAYSCALE_THRESHOLD` and ROI weights based on real-world conditions

## Key Configuration

**Threshold adjustment:**
- Current: `[(0, 64)]` — tracks dark black lines
- For white lines: `[(128, 255)]`

**ROI adjustment:**
- Modify the `ROIS` list (lines 44-48) to change detection regions
- Weights should typically sum to ~1.0 but don't have to (code normalizes by `weight_sum`)

**Angle inversion:**
- Negative sign in `deflection_angle = -math.atan(...)` may need adjustment depending on robot steering logic

## Notes

- Code comments and parameter documentation are in **Chinese**
- The parent directory (`k230-detection/`) contains a more comprehensive CLAUDE.md at `black_02/CLAUDE.md` with additional API reference and hardware pin configuration
- Reference examples are available in sibling directory `black_02/参考示例/`
- No unit tests or linting — testing is done by running on hardware and observing output visually
