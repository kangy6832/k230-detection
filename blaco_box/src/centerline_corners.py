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


def centerline_from_outer_inner(
    gray: np.ndarray,
    outer_corners: np.ndarray,
    inner_corners: np.ndarray,
) -> np.ndarray:
    ordered_outer = _order_corners(outer_corners)
    ordered_inner = _order_corners(inner_corners)

    # Centerline corner estimate: midpoint between matched outer/inner corners.
    centers = (ordered_outer + ordered_inner) * 0.5

    corners = centers.reshape(-1, 1, 2).astype(np.float32)
    criteria = (cv2.TERM_CRITERIA_EPS + cv2.TERM_CRITERIA_MAX_ITER, 30, 1e-3)
    cv2.cornerSubPix(gray, corners, (5, 5), (-1, -1), criteria)
    return corners.reshape(-1, 2).astype(np.float32)
