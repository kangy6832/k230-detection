from typing import Dict, Any

import cv2
import numpy as np


def _order_corners(corners: np.ndarray) -> np.ndarray:
    pts = corners.astype(np.float32)
    s = pts.sum(axis=1)
    d = np.diff(pts, axis=1).reshape(-1)

    tl = pts[np.argmin(s)]
    br = pts[np.argmax(s)]
    tr = pts[np.argmin(d)]
    bl = pts[np.argmax(d)]
    return np.array([tl, tr, br, bl], dtype=np.float32)


def _is_square_like(quad: np.ndarray, max_aspect_ratio_diff: float) -> bool:
    x, y, w, h = cv2.boundingRect(quad.astype(np.int32))
    if h == 0:
        return False
    ratio = w / float(h)
    return abs(ratio - 1.0) <= max_aspect_ratio_diff


def _approx_quad(contour: np.ndarray, eps_ratio: float) -> np.ndarray | None:
    epsilon = eps_ratio * cv2.arcLength(contour, True)
    approx = cv2.approxPolyDP(contour, epsilon, True)
    if len(approx) != 4 or not cv2.isContourConvex(approx):
        return None
    return approx.reshape(-1, 2).astype(np.float32)


def detect_frame_quads(binary: np.ndarray, cfg: Dict[str, Any]) -> tuple[np.ndarray | None, np.ndarray | None]:
    contours, hierarchy = cv2.findContours(binary, cv2.RETR_TREE, cv2.CHAIN_APPROX_SIMPLE)
    if hierarchy is None or len(contours) == 0:
        return None, None

    h = hierarchy[0]
    f = cfg["filter"]
    min_area_px = float(f["min_area_px"])
    eps_ratio = float(f["approx_epsilon_ratio"])
    max_ratio_diff = float(f["max_aspect_ratio_diff"])

    best_outer = None
    best_inner = None
    best_score = -1.0

    for i, cnt in enumerate(contours):
        area_outer = cv2.contourArea(cnt)
        if area_outer < min_area_px:
            continue

        child_idx = h[i][2]
        if child_idx < 0:
            continue

        approx_outer = _approx_quad(cnt, eps_ratio)
        if approx_outer is None or not _is_square_like(approx_outer, max_ratio_diff):
            continue

        child_cnt = contours[child_idx]
        area_inner = cv2.contourArea(child_cnt)
        if area_inner <= 0:
            continue

        approx_inner = _approx_quad(child_cnt, eps_ratio)
        if approx_inner is None or not _is_square_like(approx_inner, max_ratio_diff):
            continue

        # Prefer larger and clearer ring-like candidates.
        ring_ratio = min(area_inner, area_outer) / max(area_inner, area_outer)
        score = area_outer * (1.0 - ring_ratio)
        if score > best_score:
            best_score = score
            best_outer = _order_corners(approx_outer)
            best_inner = _order_corners(approx_inner)

    return best_outer, best_inner
