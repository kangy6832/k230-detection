import cv2
import numpy as np


def build_object_points(side_length_mm: float) -> np.ndarray:
    s = float(side_length_mm)
    return np.array(
        [
            [0.0, 0.0, 0.0],
            [s, 0.0, 0.0],
            [s, s, 0.0],
            [0.0, s, 0.0],
        ],
        dtype=np.float32,
    )


def solve_pose_and_3d(
    image_points: np.ndarray,
    camera_matrix: np.ndarray,
    dist_coeffs: np.ndarray,
    side_length_mm: float,
) -> tuple[bool, np.ndarray | None, np.ndarray | None, np.ndarray | None, float]:
    object_points = build_object_points(side_length_mm)

    ok, rvec, tvec = cv2.solvePnP(
        object_points,
        image_points.astype(np.float32),
        camera_matrix.astype(np.float32),
        dist_coeffs.astype(np.float32),
        flags=cv2.SOLVEPNP_IPPE,
    )
    if not ok:
        return False, None, None, None, float("inf")

    rmat, _ = cv2.Rodrigues(rvec)
    corners_cam = []
    for p in object_points:
        cam_p = rmat @ p.reshape(3, 1) + tvec
        corners_cam.append(cam_p.reshape(-1))
    corners_cam = np.array(corners_cam, dtype=np.float32)

    reproj, _ = cv2.projectPoints(object_points, rvec, tvec, camera_matrix, dist_coeffs)
    reproj = reproj.reshape(-1, 2)
    err = float(np.mean(np.linalg.norm(image_points - reproj, axis=1)))

    return True, rvec, tvec, corners_cam, err
