from typing import Dict, Any

import cv2
import numpy as np


def preprocess_frame(frame_bgr: np.ndarray, cfg: Dict[str, Any]) -> tuple[np.ndarray, np.ndarray]:
    gray = cv2.cvtColor(frame_bgr, cv2.COLOR_BGR2GRAY)

    p = cfg["preprocess"]
    denoised = cv2.bilateralFilter(
        gray,
        d=int(p["bilateral_d"]),
        sigmaColor=float(p["bilateral_sigma_color"]),
        sigmaSpace=float(p["bilateral_sigma_space"]),
    )

    _, binary = cv2.threshold(
        denoised,
        float(p["threshold_value"]),
        255,
        cv2.THRESH_BINARY_INV,
    )

    k = int(p["morph_kernel"])
    kernel = np.ones((k, k), dtype=np.uint8)
    binary = cv2.morphologyEx(binary, cv2.MORPH_OPEN, kernel)

    return gray, binary
