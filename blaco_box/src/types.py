from dataclasses import dataclass
from typing import Optional

import numpy as np


@dataclass
class DetectionResult:
    corners_px: np.ndarray  # shape: (4, 2), order: tl, tr, br, bl
    centerline_corners_px: np.ndarray  # shape: (4, 2)
    corners_cam_mm: np.ndarray  # shape: (4, 3)
    rvec: np.ndarray  # shape: (3, 1)
    tvec: np.ndarray  # shape: (3, 1)
    reprojection_error_px: float
    valid: bool
    reason: str


@dataclass
class FrameDebug:
    outer_corners_px: Optional[np.ndarray]
    inner_corners_px: Optional[np.ndarray]
