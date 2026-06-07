# CLAUDE.md

This file provides guidance to Claude Code (claude.ai/code) when working with code in this repository.

## Project Overview

This is an **embedded computer vision project** for the **Kendryte K230D BOX** development board by ALIENTEK (正点原子). The project implements **color blob tracking** - detecting and tracking colored objects (primarily red blocks) using camera input and displaying results on an LCD screen.

**Current working directory:** `red(6.6)/` - Color block tracking experiment (色块追踪实验)

## Technical Stack

- **Language:** MicroPython (Python subset for microcontrollers)
- **Framework:** CanMV firmware (K230's MicroPython-based computer vision framework)
- **Hardware:** Kendryte K230 microcontroller with camera sensor and LCD display

## Project Structure

This is a **monorepo** containing multiple experimental variations:

```
k230-detection/
├── red(6.6)/              ← CURRENT: Color blob tracking
├── black_box_k230/        # Black square frame 3D corner detection
├── black_02/              # Multi-mode detection (black + red)
├── black(6.5)/            # Black line following detection
├── micro_pnp/             # PnP pose estimation implementation
├── blaco_box/             # OpenCV-based detection (PC version)
├── k210/                  # Face recognition on K210 board
├── red_kuai/              # Red block detection variant
├── take_photos/           # Photo capture utility
└── 双目相机/              # Binocular/stereo camera
```

## Architecture & Key APIs

### Hardware Modules
```python
from media.sensor import *      # Camera sensor interface
from media.display import *     # LCD display interface
from media.media import *       # Media buffer management
from machine import Pin         # GPIO pin control
from machine import FPIOA       # Pin function assignment
```

### Core APIs
- `Sensor(width, height)` - Initialize camera with resolution
- `sensor.snapshot()` - Capture image frame
- `img.find_blobs([threshold], pixels_threshold, area_threshold, merge, margin)` - Color blob detection
- `img.draw_rectangle()` - Draw bounding boxes
- `img.draw_cross()` - Draw crosshair at center
- `Display.init(Display.ST7701, width, height, fps)` - Initialize LCD
- `Display.show_image(img)` - Output image to LCD

### Algorithm Flow
1. Initialize camera (VGA resolution, RGB565 format) and LCD display
2. Wait for auto-exposure/auto-white-balance to stabilize (~30 frames)
3. Capture image from camera
4. If KEY0 pressed: Learn color thresholds from center 50x50 ROI using histogram percentiles
5. Use `find_blobs()` to detect color blobs matching learned thresholds
6. Draw bounding boxes and crosshair at detected blob centers
7. Display on LCD and print FPS to serial

### Hardware Configuration
- **KEY0:** GPIO34, pull-up input, active low → triggers threshold learning
- **FPIOA:** Must initialize pin function before using GPIO
- **Camera:** Configurable resolution (QQVGA/QVGA/VGA), default 640x480
- **Display:** ST7701 LCD at 640x480, 90 FPS

## Development Workflow

**No build system or package manager** - this is embedded firmware development.

1. Edit `.py` files in CanMV IDE (or any text editor)
2. Deploy to K230 board via USB/serial connection or SD card
3. Run directly on the board
4. Observe output on:
   - LCD display (visual feedback)
   - IDE virtual display
   - Serial output (FPS and debug data)

**No testing framework** - testing is done by running on hardware and observing visual output. Performance measured by FPS counter.

## Code Conventions

- Comments and documentation in **Chinese** (中文)
- Chinese variable names and print statements
- Version headers with author, date, license info
- Uses LAB color space for thresholding (L, A, B channels)
- Threshold learning uses histogram percentiles (1% and 99%)
- Proper resource cleanup in finally blocks (sensor.stop, Display.deinit, MediaManager.deinit)

## MicroPython Considerations

This is **MicroPython** for embedded systems, not standard Python:
- No `pip` or package manager
- Limited standard library
- Direct hardware access via `machine` module
- Real-time constraints
- Memory-constrained environment
- Different import structure (e.g., `from media.sensor import *`)

## Related Projects

- **micro_pnp:** Comprehensive PnP implementation with documentation
- **black_box_k230:** Full 3D corner detection with PnP pose estimation
- **black_02:** Multi-mode detection (black rectangle + red block)
- **blaco_box:** OpenCV-based PC version with YAML configuration

## Documentation Languages

- **Primary:** Chinese (中文)
- **Secondary:** English (code comments, API names)
