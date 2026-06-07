# 双目相机检测黑色正方形框 — 前期准备与调查报告

> 目标：在 K230（CanMV MicroPython）上利用双目（双摄）相机实现对黑色正方形目标的三维空间坐标检测。
> 工作距离：10–30 cm；目标规格：边长 10 cm、线宽 0.3 cm（与 `black_box_k230` / `micro_pnp` 一致）。

---

## 1. 项目现状

仓库 `k230-detection` 下已有若干相关子项目，它们为本次双目任务提供了算法与工程基线：

| 目录 | 内容 | 与本次任务的关系 |
|---|---|---|
| `black_box_k230/` | 单目正方形角点 3D 检测（find_rects + IPPE/PnP + 亚像素优化 + EMA） | 可直接复用的目标检测、角点优化、位姿求解、坐标平滑代码 |
| `micro_pnp/` | 同功能的另一实现（支持 ROI、lens_corr、config 参数化）+ 详细设计文档 | 参考配置化架构与文档化思路 |
| `black_02/` | 多模式检测（黑矩形 + 红色色块），含按键切换与 `plan.md` | 多模式切换与目标偏移输出（`x/y/z`）的参考 |
| `center_square_black_check.py` | 灰度二值化 + 中心框 + 上下左右方向检测 | 若双目匹配成本过高，可退化为单目 + 中心线对齐逻辑 |
| `双目相机/` | **当前工作目录（空）** | 本次任务的工作区 |

关键结论：**单目 PnP 方案已经成熟**，双目方案是在此基础上增加深度感知、提升远端精度与鲁棒性。

---

## 2. K230 硬件的双摄支持（已确认）

根据 Kendryte 官方 CanMV 文档中"获取双摄像头图像并显示在 HDMI 显示器上"示例：

- K230 芯片**原生支持双摄像头同时采集**，通过 `Sensor(id=0)` 和 `Sensor(id=1)` 分别实例化。
- 两个 sensor 的通道 0 分辨率均可设为 960×540（YUV420SP），并分别绑定到 `Display.LAYER_VIDEO1` / `LAYER_VIDEO2`。
- 多摄场景仅需对其中一个 sensor 调用 `sensor0.run()` 即可启动采集。
- 每个 sensor 独立 `reset` / `set_framesize` / `set_pixformat` / `stop`。

**结论：K230 硬件上可驱动双目相机，无需额外 FPGA/桥接芯片。**

### 官方双摄示例核心代码（摘自 CanMV 文档）

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
```

> 来源：https://www.kendryte.com/k230_canmv/zh/main/zh/example/media/sensor.html

---

## 3. 双目立体视觉原理（针对本任务）

### 3.1 核心公式

```
Z = (f × B) / d
```

- `Z`：目标到相机基线的深度（mm）
- `f`：焦距（像素），来自相机标定
- `B`：基线距离（mm），两个镜头光心之间的物理距离
- `d = x_left − x_right`：视差（像素）

### 3.2 完整双目管线

1. **双目标定** — 获取左右相机内参（`fx, fy, cx, cy`）、畸变系数（`k1, k2, p1, p2`）及双目标定外参（`R, T`，其中 `T` 的 x 分量即基线 `B`）。
2. **立体校正（Rectify）** — 将左右图像重映射到同一极线平面，使对应点位于同一行，把 2D 匹配退化到 1D。
3. **立体匹配（Match）** — 计算视差图。嵌入式上推荐 **BM（Block Matching）** 或简化版 SGBM，窗口 9–15 px、视差范围按工作距离限定，以控制算力。
4. **三角测量（Triangulate）** — 用 `Q` 矩阵把视差图转为 3D 点云。
5. **目标检测** — 在左图（或右图）中用已有 `find_rects` + 正方形筛选逻辑检测黑色正方形。
6. **3D 定位** — 取目标中心/角点的视差，代入 `Z = f·B/d` 得到深度；或用 `solve_pose_from_square`（已验证的单目 PnP）与双目深度做融合，提升远端精度。

### 3.3 精度估算（典型参数）

假设 `f ≈ 400 px`、`B = 75 mm`：

| 深度 Z (mm) | 视差 d (px) | 每 1 px 误差引起的 ΔZ |
|---|---|---|
| 100 | 300 | 0.33 mm |
| 200 | 150 | 1.3 mm |
| 300 | 100 | 3.0 mm |

在 10–30 cm 工作距离内，双目深度精度**理论上可达亚毫米到毫米级**，显著优于单目 PnP 在远端的表现。

---

## 4. 两种可行技术路线

### 路线 A：双目视差 + 单目 PnP 融合（推荐）

- **左图**：运行已有的 `find_rects` + 正方形筛选 + 亚像素角点优化 + PnP 位姿求解（`black_box_k230/geometry.py` 中的 `solve_pose_from_square`）。
- **左右图**：对左图 ROI 区域做立体匹配，得到目标中心/角点的视差，换算为深度 `Z_stereo`。
- **融合**：用 `Z_stereo` 对 PnP 输出的平移向量 `t` 做尺度校正或卡尔曼滤波融合，输出最终 3D 坐标。
- **优点**：复用全部已验证的单目逻辑，双目仅提供"绝对尺度"，实现风险最低。
- **缺点**：匹配算力仍需评估。

### 路线 B：纯双目视差定位

- 左图检测目标，右图对应区域匹配，直接用视差换算 3D。
- **优点**：不依赖目标尺寸先验，通用性更强。
- **缺点**：匹配精度受纹理影响大（黑色正方形边缘特征少），需仔细调参；与现有代码复用度低。

**建议先走路线 A**，待立体匹配稳定性验证后再考虑路线 B。

---

## 5. 待完成的核心任务清单

### 5.1 硬件与驱动

- [ ] 确认具体双目相机模组型号（基线 `B`、镜头参数、接口 MIPI/USB）。
- [ ] 在 K230 上跑通官方双摄示例，验证两路图像均能正常采集。
- [ ] 选定工作分辨率（推荐 640×480 或 960×540，兼顾精度与帧率）。

### 5.2 相机标定

- [ ] 使用棋盘格标定板，分别标定左右相机内参与畸变系数（可用 OpenCV 在 PC 端完成）。
- [ ] 双目标定获取 `R, T`（基线 `B = |T_x|`）。
- [ ] 生成校正映射表 `mapx_left, mapy_left, mapx_right, mapy_right`，固化到 CanMV 脚本中。

### 5.3 立体匹配

- [ ] 在 K230 上实现或移植轻量 BM 匹配器（纯整数运算，窗口 9–11 px）。
- [ ] 限定视差搜索范围（例如 20–200 px）以降低计算量。
- [ ] 评估单帧耗时：目标 ≤ 30 ms（30 FPS 预算的一半留给检测与 PnP）。

### 5.4 目标检测复用

- [ ] 从 `black_box_k230/geometry.py` 移植：
  - `order_corners_tl_tr_br_bl`
  - `is_square_like`
  - `subpixel_corner_refine`
  - `solve_pose_from_square`
  - `square_world_to_camera_points`
  - `ema_points`
- [ ] 从 `black_box_k230/config.py` 移植可调参数（阈值、面积范围、正方形容差等）。

### 5.5 系统融合与输出

- [ ] 设计融合策略：PnP 的 `t` 与双目 `Z_stereo` 加权平均或卡尔曼滤波。
- [ ] 输出格式对齐现有项目：`(x, y, z)` 厘米或毫米，串口打印 + 画面叠加。
- [ ] 可选：ROI 机制（左图检测到目标后，右图仅对 ROI 匹配）以进一步省算力。

---

## 6. 主要风险与缓解

| 风险 | 影响 | 缓解 |
|---|---|---|
| K230 无内置立体匹配 API | 需自研或移植 BM，工作量大 | 先用 ROI 限定匹配区域，窗口与视差范围尽量小；必要时用 C 扩展 |
| 算力不足导致帧率过低 | 实时性差 | 降分辨率到 320×240 做匹配，左图保持高分辨率做检测 |
| 黑色目标纹理少，匹配困难 | 视差空洞多 | 匹配前做直方图均衡 / 拉普拉斯增强；或改用边缘图匹配 |
| 双摄同步问题 | 运动场景视差错误 | 优先全局快门模组；静态场景下可忽略 |
| 标定误差直接传导到深度 | 3D 精度下降 | 使用高精度标定板 + 多角度标定；运行时做在线校正（可选） |

---

## 7. 参考代码与文档

- **双摄驱动示例**：https://www.kendryte.com/k230_canmv/zh/main/zh/example/media/sensor.html
- **单目正方形 3D 检测**：`black_box_k230/main.py`、`black_box_k230/geometry.py`、`black_box_k230/camera_params.py`、`black_box_k230/config.py`
- **参数化实现**：`micro_pnp/main.py`、`micro_pnp/` 下 `camera_params.py`、`config.py`、`geometry.py`
- **设计文档**：`micro_pnp/正点原子K230D BOX黑色正方形目标三维空间坐标检测算法实现方案.md`
- **黑矩形方向检测**：`center_square_black_check.py`、`black_02/center_square_black_check.py`
- **多模式切换**：`black_02/main.py`

---

## 8. 下一步建议（优先级排序）

1. **跑通双摄示例** — 用官方代码确认硬件可用。
2. **完成双目标定** — 获取 `K_left, K_right, dist_left, dist_right, R, T`。
3. **实现最简立体匹配** — 在 ROI 内做 BM，验证视差合理性。
4. **移植单目检测逻辑到左图** — 确保 `find_rects + PnP` 在双目场景下仍稳定。
5. **融合双目深度** — 用视差校正 PnP 的 Z 尺度。
6. **性能优化** — 降分辨率、ROI、帧率调优。

---

*文档生成时间：2026-06-01*
