# 正点原子 K230D BOX 模块与函数 API 参考手册

> 来源：Kendryte 官方 CanMV K230 文档
> - 入口：https://www.kendryte.com/k230_canmv/zh/main/zh/index.html
> - API 索引：https://www.kendryte.com/k230_canmv/zh/main/zh/api/index.html
> - 整理时间：2026-06-01

---

## 目录

1. [Sensor 模块（摄像头采集）](#1-sensor-模块)
2. [Display 模块（显示输出）](#2-display-模块)
3. [MediaManager 模块（媒体缓冲管理）](#3-mediamanager-模块)
4. [Image 模块（图像处理与机器视觉）](#4-image-模块)
5. [cv_lite 模块（OpenCV 轻量加速）](#5-cv_lite-模块)
6. [machine 模块（硬件控制）](#6-machine-模块)
7. [FPIOA 模块（引脚功能映射）](#7-fpioa-模块)
8. [Pin 模块（GPIO 控制）](#8-pin-模块)
9. [UART 模块（串口通信）](#9-uart-模块)
10. [其他模块概览](#10-其他模块概览)

---

## 1. Sensor 模块

> `from media.sensor import *`

### 构造函数

```python
Sensor(id=0, width=1920, height=1080, fps=30)
```

- `id`：CSI 端口号（0–2），双目场景用 `id=0` 和 `id=1`
- `width` / `height`：最大输出分辨率
- `fps`：最大帧率

### 方法

| 方法 | 说明 |
|---|---|
| `sensor.reset()` | 复位，构造后必须调用 |
| `sensor.set_framesize(framesize, chn, alignment, crop)` | 设置输出分辨率，宽度自动对齐 16px |
| `sensor.set_pixformat(pix_format, chn)` | 设置像素格式 |
| `sensor.set_hmirror(bool)` | 水平镜像 |
| `sensor.set_vflip(bool)` | 垂直翻转 |
| `sensor.run()` | 启动输出，需在 MediaManager.init() 之后 |
| `sensor.stop()` | 停止输出，需在 MediaManager.deinit() 之前 |
| `sensor.snapshot(chn, timeout, dump_frame)` | 捕获一帧，返回 Image |
| `sensor.bind_info(x, y, chn)` | 获取绑定信息 dict（用于 Display） |
| `sensor.get_hmirror() → bool` | 获取水平镜像状态 |
| `sensor.get_vflip() → bool` | 获取垂直翻转状态 |
| `sensor.width(chn) → int` | 获取当前宽度 |
| `sensor.height(chn) → int` | 获取当前高度 |
| `sensor.get_pixformat(chn) → int` | 获取当前像素格式 |
| `sensor.get_type() → int` | 获取 sensor 类型标识 |
| `sensor.again(desired_gain)` | 获取/设置模拟增益 |
| `sensor.auto_focus(enable)` | 获取/设置自动对焦 |
| `sensor.focus_caps() → tuple` | 返回 `(isSupport, minPos, maxPos)` |
| `sensor.focus_pos(pos)` | 获取/设置对焦位置 |
| `sensor.get_exposure_time_range() → tuple` | 返回 `(max_us, min_us)`，需在 run() 后 |
| `sensor.auto_exposure(enable)` | 获取/设置自动曝光，需在 run() 前 |
| `sensor.exposure(exposure_us)` | 获取/设置曝光时间(µs)，需在 run() 后 |
| `sensor.get_again_range() → dict` | 返回 `{min, max, step}`，需在 run() 后 |
| `Sensor.list_mode(id) → tuple` | 静态方法，列出 sensor 支持的模式 |

### 像素格式常量

| 常量 | 说明 |
|---|---|
| `Sensor.RGB565` | 16 位 RGB |
| `Sensor.RGB888` | 24 位 RGB |
| `Sensor.RGBP888` | 平面 24 位 RGB |
| `Sensor.YUV420SP` | 半平面 YUV（双摄推荐） |
| `Sensor.GRAYSCALE` | 灰度 |

### 分辨率常量

`QQCIF(88×72)` `QCIF(176×144)` `CIF(352×288)` `QSIF(176×120)` `SIF(352×240)` `QQVGA(160×120)` `QVGA(320×240)` `VGA(640×480)` `HQQVGA(120×80)` `HQVGA(240×160)` `HVGA(480×320)` `B64X64` `B128X64` `B128X128` `B160X160` `B320X320` `QQVGA2(128×160)` `WVGA(720×480)` `WVGA2(752×480)` `SVGA(800×600)` `XGA(1024×768)` `WXGA(1280×768)` `SXGA(1280×1024)` `SXGAM(1280×960)` `UXGA(1600×1200)` `HD(1280×720)` `FHD(1920×1080)` `QHD(2560×1440)` `QXGA(2048×1536)` `WQXGA(2560×1600)` `WQXGA2(2592×1944)`

### 通道常量

| 常量 |
|---|
| `CAM_CHN_ID_0` / `CAM_CHN_ID_1` / `CAM_CHN_ID_2` / `CAM_CHN_ID_MAX` |

### 支持的 Sensor 芯片

| Sensor | 最大分辨率 | 最大帧率 |
|---|---|---|
| OV5647 | 2592×1944 | 10fps（最大）；90fps @ VGA |
| GC2093 | 1920×1080 | 60fps（FHD）；90fps @ 720p |
| IMX335 | 2592×1944 | 30fps |

### 双摄采集示例

```python
from media.sensor import *
from media.display import *
from media.media import *

sensor0 = Sensor(id=0)
sensor0.reset()
sensor0.set_framesize(width=960, height=540)
sensor0.set_pixformat(Sensor.YUV420SP)
bind_info = sensor0.bind_info(x=0, y=0)
Display.bind_layer(**bind_info, layer=Display.LAYER_VIDEO1)

sensor1 = Sensor(id=1)
sensor1.reset()
sensor1.set_framesize(width=960, height=540)
sensor1.set_pixformat(Sensor.YUV420SP)
bind_info = sensor1.bind_info(x=960, y=0)
Display.bind_layer(**bind_info, layer=Display.LAYER_VIDEO2)

Display.init(Display.LT9611, to_ide=True)
MediaManager.init()
sensor0.run()

while True:
    img0 = sensor0.snapshot()
    img1 = sensor1.snapshot()
```

---

## 2. Display 模块

> `from media.display import *`

### 方法

| 方法 | 说明 |
|---|---|
| `Display.init(type, width, height, fps, flag, osd_num, to_ide, quality)` | 初始化显示管线 |
| `Display.deinit()` | 反初始化 |
| `Display.inited() → bool` | 查询初始化状态 |
| `Display.config_layer(rect, pix_format, layer, alpha, flag)` | 配置显示层 |
| `Display.bind_layer(src, rect, pix_format, layer, alpha, flag)` | 绑定 sensor 输出到显示层 |
| `Display.unbind_layer(layer) → bool` | 解绑显示层 |
| `Display.disable_layer(layer)` | 禁用显示层 |
| `Display.show_image(img, x, y, layer, alpha, pixel_format, flag)` | 在 OSD 层显示 Image |
| `Display.width(layer) → int` | 获取宽度 |
| `Display.height(layer) → int` | 获取高度 |
| `Display.fps() → int` | 获取帧率 |
| `Display.writeback(enable) → bool` | 查询/设置回写 |
| `Display.writeback_dump(timeout) → object` | 从回写通道捕获一帧 |

### 显示类型常量

| 常量 | 默认分辨率 |
|---|---|
| `Display.VIRT` | 640×480@90（IDE 调试） |
| `Display.ST7701` | 800×480 |
| `Display.LT9611` | 1920×1080@30（HDMI） |
| `Display.HX8399` | 1920×1080 |
| `Display.ILI9806` | 800×480 |
| `Display.ILI9881` | 1280×800 |
| `Display.NT35516` | 960×536 |
| `Display.NT35532` | 1920×1080 |
| `Display.GC9503` | 800×480 |
| `Display.ST7102` | 640×480 |
| `Display.ST7789` | 320×240 |

### 层常量

| 常量 | 类型 |
|---|---|
| `Display.LAYER_VIDEO1` | 视频层（YUV420SP） |
| `Display.LAYER_VIDEO2` | 视频层（YUV420SP） |
| `Display.LAYER_VIDEO3` | 视频层（YUV420SP） |
| `Display.LAYER_OSD0` | OSD 层（RGB） |
| `Display.LAYER_OSD1` | OSD 层（RGB） |
| `Display.LAYER_OSD2` | OSD 层（RGB） |
| `Display.LAYER_OSD3` | OSD 层（RGB） |

### 标志常量

`FLAG_ROTATION_0/90/180/270` `FLAG_MIRROR_HOR/VER/BOTH`

---

## 3. MediaManager 模块

> `from media.media import *`

| 方法 | 说明 |
|---|---|
| `MediaManager.init()` | 初始化媒体缓冲，必须在 sensor.run() 之前 |
| `MediaManager.deinit()` | 释放缓冲，必须在 sensor.stop() 之后 |
| `MediaManager._config(config)` | 配置缓冲（内部） |
| `MediaManager.Buffer.get(size)` | 获取缓冲区 |

### 典型调用顺序

```
MediaManager.init() → sensor.run() → 主循环 → sensor.stop() → MediaManager.deinit()
```

---

## 4. Image 模块

> `import image`（CanMV 内置）

### 属性

`img.width()` `img.height()` `img.format()` `img.size()` `img.phyaddr()` `img.virtaddr()` `img.poolid()`

### 像素操作

- `img.get_pixel(x, y)`
- `img.set_pixel(x, y, pixel)`

### 格式转换

| 方法 | 说明 |
|---|---|
| `img.to_grayscale(copy=False)` | 转灰度 |
| `img.to_rgb565(copy=False)` | 转 RGB565 |
| `img.to_rainbow(copy=False)` | 转彩虹图 |
| `img.to_rgb888(x_scale, y_scale, ...)` | 转 RGB888 |
| `img.to_numpy_ref()` | 转 NumPy 数组（共享内存，用于 cv_lite） |

### 内存操作

`img.copy_from(src)` `img.copy_to(dst)` `img.copy(roi, copy_to_fb)` `img.save(path, roi, quality)` `img.clear()`

### 绘图

| 方法 | 说明 |
|---|---|
| `img.draw_line(x0,y0,x1,y1,color,thickness)` | 画线 |
| `img.draw_rectangle(x,y,w,h,color,thickness,fill)` | 画矩形 |
| `img.draw_circle(x,y,radius,color,thickness,fill)` | 画圆 |
| `img.draw_ellipse(cx,cy,rx,ry,rotation,color,thickness,fill)` | 画椭圆 |
| `img.draw_string(x,y,text,color,scale,...)` | 写文字 |
| `img.draw_cross(x,y,color,size,thickness)` | 画十字 |
| `img.draw_arrow(x0,y0,x1,y1,color,thickness)` | 画箭头 |
| `img.draw_image(image,x,y,x_scale,y_scale,mask,alpha)` | 贴图 |
| `img.draw_keypoints(keypoints,color,size,thickness,fill)` | 画关键点 |
| `img.draw_string_advanced(x,y,char_size,str,color,font)` | 高级文字（支持中文） |

### 压缩

`img.compress(quality)` `img.compressed(quality)` `img.compress_for_ide(quality)` `img.compressed_for_ide(quality)`

### 形态学操作

`img.erode(size, threshold, mask)` `img.dilate(...)` `img.open(...)` `img.close(...)` `img.top_hat(...)` `img.black_hat(...)`

### 二值化与逻辑运算

`img.binary(thresholds, invert, zero, mask)` `img.invert()` `img.b_and(image, mask)` `img.b_or(...)` `img.b_xor(...)` 等

### 算术运算

`img.add(image, mask)` `img.sub(...)` `img.mul(...)` `img.div(...)` `img.min(...)` `img.max(...)` `img.difference(...)` `img.blend(image, alpha, mask)` `img.negate()` `img.replace(...)`

### 滤波

`img.mean(size, ...)` `img.median(size, ...)` `img.mode(...)` `img.midpoint(...)` `img.morph(size, kernel, mul, add)` `img.gaussian(size, ...)` `img.laplacian(size, ...)` `img.bilateral(size, color_sigma, space_sigma, ...)`

### 池化

`img.mean_pool(x_div, y_div)` `img.mean_pooled(x_div, y_div)` `img.midpoint_pool(x_div, y_div, bias)` `img.midpoint_pooled(...)`

### 图像增强

`img.histeq(adaptive, clip_limit, mask)` `img.cartoon(...)` `img.remove_shadows(image)` `img.chrominvar()` `img.illuminvar()`

### 几何变换

`img.linpolar(reverse)` `img.logpolar(reverse)` `img.lens_corr(strength, zoom)` `img.rotation_corr(...)`

### 分析

`img.get_similarity(image)` `img.get_histogram(...)` `img.get_statistics(...)` `img.get_regression(...)`

### 特征检测（本任务重点）

| 方法 | 说明 |
|---|---|
| **`img.find_blobs(thresholds, invert, roi, x_stride, y_stride, area_threshold, pixels_threshold, merge, margin, ...)`** | **色块检测** |
| **`img.find_lines(roi, x_stride, y_stride, threshold, theta_margin, rho_margin)`** | **霍夫直线检测** |
| **`img.find_line_segments(roi, merge_distance, max_theta_difference)`** | **LSD 线段检测** |
| **`img.find_circles(roi, x_stride, y_stride, threshold, x_margin, y_margin, r_margin)`** | **霍夫圆检测** |
| **`img.find_rects(roi, threshold)`** | **四边形/矩形检测 — 本任务核心 API** |
| `img.find_qrcodes(roi)` | QR 码检测 |
| `img.find_apriltags(roi, families, fx, fy, cx, cy)` | AprilTag 检测 |
| `img.find_datamatrices(roi, effort)` | Data Matrix 检测 |
| `img.find_barcodes(roi)` | 一维条码检测 |
| `img.find_displacement(template, roi, template_roi, logpolar)` | 相位相关位移 |
| `img.find_number(roi)` | MNIST 数字识别（实验性） |
| `img.find_template(template, threshold, roi, step, search)` | NCC 模板匹配 |
| `img.find_features(cascade, threshold, scale, roi)` | Haar 级联特征检测 |
| `img.find_eye(roi)` | 瞳孔检测 |
| `img.find_keypoints(roi, threshold, normalized, scale_factor, max_keypoints, corner_detector)` | ORB 关键点提取 |
| `img.find_edges(edge_type, threshold)` | 边缘检测（CANNY/SIMPLE） |
| `img.find_hog(roi, size)` | HOG 特征提取 |

### 模块级函数

| 函数 | 说明 |
|---|---|
| `image.rgb_to_lab(rgb_tuple)` | RGB → LAB |
| `image.lab_to_rgb(lab_tuple)` | LAB → RGB |
| `image.rgb_to_grayscale(rgb_tuple)` | RGB → 灰度 |
| `image.grayscale_to_rgb(g_value)` | 灰度 → RGB |
| `image.load_descriptor(path)` | 加载描述符 |
| `image.save_descriptor(path, descriptor)` | 保存描述符 |
| `image.match_descriptor(desc0, desc1, threshold, filter_outliers)` | 匹配描述符 |

---

## 5. cv_lite 模块 — OpenCV 轻量加速

> `import cv_lite`（需 daily_build 固件）
> 文档：`cv_lite/cv_lite.html`

### 数据流模式

```python
# image → numpy → cv_lite 处理 → image
np_img = img.to_numpy_ref()
result = cv_lite.some_function(img.shape, np_img, ...)
# 如需转回 image：
out_img = image.Image(w, h, format, alloc=image.ALLOC_REF, data=np_result)
```

### Blob 检测

| 函数 | 说明 |
|---|---|
| `grayscale_find_blobs(image_shape, img_np, threshold_min, threshold_max, min_area, kernel_size)` | 灰度色块 |
| `rgb888_find_blobs(image_shape, img_np, threshold, min_area, kernel_size)` | RGB 色块，threshold = `[Rmin,Rmax,Gmin,Gmax,Bmin,Bmax]` |

### 圆检测（Hough）

| 函数 | 说明 |
|---|---|
| `grayscale_find_circles(image_shape, img_np, dp, minDist, param1, param2, minRadius, maxRadius)` | 灰度霍夫圆 |
| `rgb888_find_circles(...)` | RGB 霍夫圆 |

### 矩形检测

| 函数 | 说明 |
|---|---|
| `grayscale_find_rectangles(image_shape, img_np, canny_thresh1, canny_thresh2, approx_epsilon, area_min_ratio, max_angle_cos, gaussian_blur_size)` | 灰度矩形 |
| `grayscale_find_rectangles_with_corners(...)` | 同上，额外返回 4 个角点坐标 |
| `rgb888_find_rectangles(...)` | RGB 矩形 |
| `rgb888_find_rectangles_with_corners(...)` | RGB 矩形 + 角点 |

### 边缘检测（Canny）

- `grayscale_find_edges(image_shape, img_np, threshold1, threshold2)` → ndarray
- `rgb888_find_edges(...)` → ndarray

### 阈值/二值化

- `grayscale_threshold_binary(image_shape, img_np, thresh, maxval)` → ndarray
- `rgb888_threshold_binary(...)` → ndarray

### 曝光调整（仅 RGB888）

- `rgb888_adjust_exposure(image_shape, img_np, exposure_gain)` — gain 推荐 0.2–3.0
- `rgb888_adjust_exposure_fast(...)` — 加速版

### 白平衡（仅 RGB888）

- `rgb888_white_balance_gray_world_fast(image_shape, img_np)`
- `rgb888_white_balance_gray_world_fast_ex(image_shape, img_np, gain_clip, brightness_boost)`
- `rgb888_white_balance_white_patch(image_shape, img_np)`
- `rgb888_white_balance_white_patch_ex(image_shape, img_np, top_percent, gain_clip, brightness_boost)`

### 形态学操作（仅 RGB888）

共用参数：`(image_shape, img_np, kernel_size, iterations, threshold_value)`

| 函数 | 操作 |
|---|---|
| `rgb888_erode` | 腐蚀 |
| `rgb888_dilate` | 膨胀 |
| `rgb888_open` | 开运算 |
| `rgb888_close` | 闭运算 |
| `rgb888_tophat` | 顶帽 |
| `rgb888_blackhat` | 黑帽 |
| `rgb888_gradient` | 形态梯度 |

### 模糊（仅 RGB888）

- `rgb888_mean_blur(image_shape, img_np, kernel_size)`
- `rgb888_gaussian_blur(image_shape, img_np, kernel_size)` — kernel_size 需为奇数

### 直方图（仅 RGB888）

- `rgb888_calc_histogram(image_shape, img_np)` → 3×256 数组

### 角点检测（仅 RGB888）

- `rgb888_find_corners(image_shape, img_np, max_corners, quality_level, min_distance)` → `[x0,y0,x1,y1,...]`

### 去畸变（仅 RGB888）

- `rgb888_undistort(image_shape, img_np, camera_matrix, dist_coeffs, dist_len)`
- `rgb888_undistort_fast(...)` — 加速版
- `rgb888_undistort_new_cam_mat(...)` — 带优化相机矩阵

### PnP 距离估计（仅 RGB888）

- `rgb888_pnp_distance(image_shape, img_np, roi, camera_matrix, dist_coeffs, dist_len, roi_width_real, roi_height_real)` → 距离（cm）
- `rgb888_pnp_distance_from_corners(image_shape, img_np, camera_matrix, dist_coeffs, dist_len, obj_width_real, obj_height_real)` → `[distance, [x,y,w,h], [[x1,y1],[x2,y2],[x3,y3],[x4,y4]]]`

---

## 6. machine 模块概览

> `from machine import Pin, FPIOA, UART, I2C, SPI, PWM, Timer, WDT, RTC, TOUCH, ADC, FFT`

| 子模块 | 说明 |
|---|---|
| `machine.Pin` | GPIO 输入/输出 |
| `machine.FPIOA` | 引脚功能复用映射 |
| `machine.UART` | 串口通信（UART1/2/4） |
| `machine.I2C` | I2C 主机 |
| `machine.I2C_Slave` | I2C 从机 |
| `machine.SPI` | SPI 主机 |
| `machine.PWM` | PWM 输出 |
| `machine.ADC` | ADC 模数转换 |
| `machine.Timer` | 定时器 |
| `machine.WDT` | 看门狗 |
| `machine.RTC` | 实时时钟 |
| `machine.TOUCH` | 触摸屏 |
| `machine.FFT` | FFT 加速 |
| `machine.SPI_LCD` | SPI 液晶屏 |
| `machine.neopixel` | LED 灯带 |

---

## 7. FPIOA 模块 — 引脚功能映射

> `from machine import FPIOA`

### 构造函数

```python
fpioa = FPIOA()
```

### 方法

| 方法 | 签名 | 说明 |
|---|---|---|
| `set_function()` | `fpioa.set_function(pin, func, ie=-1, oe=-1, pu=-1, pd=-1, st=-1, ds=-1)` | 设置引脚功能 |
| `get_pin_num()` | `fpioa.get_pin_num(func)` → pin or None | 获取功能对应的引脚号 |
| `get_pin_func()` | `fpioa.get_pin_func(pin)` → func | 获取引脚当前功能 |
| `help()` | `fpioa.help([number, func=False])` | 打印引脚/功能映射 |

### 常用常量

`FPIOA.GPIO0` ~ `FPIOA.GPIO63` `FPIOA.UART0_TXD` `FPIOA.IIC0_SDA` 等

---

## 8. Pin 模块 — GPIO 控制

> `from machine import Pin`

### 构造函数

```python
Pin(index, mode, pull=Pin.PULL_NONE, value=-1, drive=7, alt=-1)
```

### 方法

| 方法 | 说明 |
|---|---|
| `pin.init(mode, pull, drive)` | 初始化引脚 |
| `pin.value([value])` | 读取/设置电平 |
| `pin.mode([mode])` | 获取/设置模式 |
| `pin.pull([pull])` | 获取/设置上下拉 |
| `pin.drive([drive])` | 获取/设置驱动强度 |
| `pin.on()` / `pin.high()` | 输出高 |
| `pin.off()` / `pin.low()` | 输出低 |
| `pin.irq(handler, trigger, priority, wake, hard, debounce)` | 使能中断 |

### 常量

- **模式**：`Pin.IN` / `Pin.OUT`
- **上下拉**：`Pin.PULL_NONE` / `Pin.PULL_UP` / `Pin.PULL_DOWN`
- **中断触发**：`Pin.IRQ_FALLING` / `Pin.IRQ_RISING` / `Pin.IRQ_LOW_LEVEL` / `Pin.IRQ_HIGH_LEVEL` / `Pin.IRQ_BOTH`
- **驱动强度**：`Pin.DRIVE_0` ~ `Pin.DRIVE_15`

---

## 9. UART 模块 — 串口通信

> `from machine import UART`

可用 UART：**UART1、UART2、UART4**（UART0/UART3 预留）

### 构造函数

```python
UART(id, baudrate=115200, bits=UART.EIGHTBITS, parity=UART.PARITY_NONE, stop=UART.STOPBITS_ONE, timeout=0)
```

### 方法

| 方法 | 签名 | 说明 |
|---|---|---|
| `init` | `uart.init(baudrate, bits, parity, stop)` | 初始化 |
| `read` | `uart.read([nbytes])` → bytes | 读取数据 |
| `readline` | `uart.readline()` → bytes | 读取一行 |
| `readinto` | `uart.readinto(buf[, nbytes])` → int | 读到缓冲区 |
| `write` | `uart.write(buf)` → int | 写入数据 |
| `deinit` | `uart.deinit()` | 关闭 |

### 常量

- **数据位**：`UART.FIVEBITS` / `SIXBITS` / `SEVENBITS` / `EIGHTBITS`
- **校验**：`UART.PARITY_NONE` / `PARITY_ODD` / `PARITY_EVEN`
- **停止位**：`UART.STOPBITS_ONE` / `STOPBITS_TWO`

---

## 10. I2C 模块

> `from machine import I2C`

### 构造函数

```python
I2C(id, freq=100000, scl=None, sda=None)
```

### 方法

| 方法 | 说明 |
|---|---|
| `init(freq, scl, sda)` | 初始化 |
| `scan()` | 扫描总线设备地址 |
| `readfrom(addr, nbytes)` | 从设备读取 |
| `writeto(addr, buf)` | 向设备写入 |
| `readfrom_mem(addr, memaddr, nbytes)` | 从设备寄存器读取 |
| `writeto_mem(addr, memaddr, buf)` | 向设备寄存器写入 |

---

## 11. SPI 模块

> `from machine import SPI`

### 构造函数

```python
SPI(id, baudrate=1000000, polarity=0, phase=0)
```

### 方法

| 方法 | 说明 |
|---|---|
| `init(baudrate, polarity, phase)` | 初始化 |
| `read(nbytes, write=0x00)` | 读取 |
| `write(buf)` | 写入 |
| `readinto(buf, write=0x00)` | 读到缓冲区 |
| `write_readinto(write_buf, read_buf)` | 同时读写 |

---

## 12. PWM 模块

> `from machine import PWM`

### 构造函数

```python
PWM(id, channel, freq=1000, duty=0)
```

### 方法

| 方法 | 说明 |
|---|---|
| `init(freq, duty)` | 初始化 |
| `freq([freq])` | 获取/设置频率 |
| `duty([duty])` | 获取/设置占空比（0~100） |
| `deinit()` | 关闭 |

---

## 13. Timer 模块

> `from machine import Timer`

### 构造函数

```python
Timer(id, mode=Timer.MODE_ONE_SHOT, period=1000, callback=None)
```

### 方法

| 方法 | 说明 |
|---|---|
| `init(mode, period, callback)` | 初始化 |
| `deinit()` | 关闭 |

### 常量

- `Timer.MODE_ONE_SHOT` — 单次
- `Timer.MODE_PERIODIC` — 周期

---

## 14. ADC 模块

> `from machine import ADC`

### 构造函数

```python
ADC(id, channel)
```

### 方法

| 方法 | 说明 |
|---|---|
| `read()` → int | 读取 ADC 值（0~4095） |
| `read_u16()` → int | 读取 16 位值 |

---

## 15. RTC 模块

> `from machine import RTC`

### 构造函数

```python
RTC()
```

### 方法

| 方法 | 说明 |
|---|---|
| `init(datetime)` | 设置日期时间 |
| `datetime()` → tuple | 获取当前时间 `(year, month, day, weekday, hour, minute, second, microsecond)` |

---

## 16. WDT 模块

> `from machine import WDT`

### 构造函数

```python
WDT(id=0, timeout=5000)
```

### 方法

| 方法 | 说明 |
|---|---|
| `feed()` | 喂狗 |

---

## 17. 其他模块

| 模块 | 导入方式 | 说明 |
|---|---|---|
| `network` | `import network` | WiFi 网络 |
| `socket` | `import socket` | 网络套接字 |
| `usb_hid` | `import usb_hid` | USB HID |
| `neopixel` | `import neopixel` | LED 灯带 |
| `SPI_LCD` | `from machine import SPI_LCD` | SPI 液晶屏 |
| `FFT` | `from machine import FFT` | FFT 加速 |
| `TOUCH` | `from machine import TOUCH` | 触摸屏 |
| `uctypes` | `import uctypes` | C 结构体访问 |
| `gc` | `import gc` | 垃圾回收 |
| `uos` | `import uos` | 文件系统 |
| `utime` | `import utime` | 时间延迟 |
| `uhashlib` | `import uhashlib` | 哈希算法 |
| `ucryptolib` | `import ucryptolib` | 加密算法 |

---

## 18. 完整模块速查表

| 分类 | 模块 | 核心功能 |
|---|---|---|
| **图像采集** | `media.sensor` | 摄像头初始化、采集、双摄 |
| **显示输出** | `media.display` | HDMI/LCD/IDE 显示、图层绑定 |
| **媒体管理** | `media.media` | 缓冲区管理 |
| **图像处理** | `image` | 绘图、滤波、特征检测、形态学 |
| **CV 加速** | `cv_lite` | 矩形/圆/角点检测、PnP、去畸变、形态学 |
| **AI 推理** | `nncase` | KPU 模型推理 |
| **AI 工具** | `aidemo` | PipeLine、Ai2d、AIBase、YOLO |
| **GPIO** | `machine.Pin` | 数字输入/输出、中断 |
| **引脚映射** | `machine.FPIOA` | 引脚功能复用 |
| **串口** | `machine.UART` | UART 通信 |
| **I2C** | `machine.I2C` | I2C 总线 |
| **SPI** | `machine.SPI` | SPI 总线 |
| **PWM** | `machine.PWM` | PWM 输出 |
| **ADC** | `machine.ADC` | 模数转换 |
| **定时器** | `machine.Timer` | 定时/回调 |
| **看门狗** | `machine.WDT` | 看门狗 |
| **RTC** | `machine.RTC` | 实时时钟 |
| **网络** | `network` / `socket` | WiFi / TCP/UDP |
| **文件系统** | `uos` | 文件/目录操作 |
| **时间** | `utime` | 延时/计时 |
| **LVGL** | `lvgl` | 图形界面库 |
| **音频** | `media.audio` | 音频输入/输出 |
| **视频编解码** | `media.vdec` / `media.venc` | 视频解码/编码 |
| **RTSP** | `media.rtsp` | RTSP 推流 |
| **电源管理** | `media.pm` | 功耗管理 |

---

*文档生成时间：2026-06-01*
*数据来源：Kendryte K230 CanMV 官方文档 https://www.kendryte.com/k230_canmv/*