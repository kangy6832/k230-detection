import argparse
import time
from pathlib import Path

import cv2
import numpy as np
import yaml

from src.centerline_corners import centerline_from_outer_inner
from src.pnp_solver import solve_pose_and_3d
from src.preprocess import preprocess_frame
from src.quad_detector import detect_frame_quads
from src.types import DetectionResult


def _load_yaml(path: Path) -> dict:
    with path.open("r", encoding="utf-8") as f:
        return yaml.safe_load(f)


def _draw_result(frame: np.ndarray, result: DetectionResult) -> np.ndarray:
    vis = frame.copy()

    for i, p in enumerate(result.centerline_corners_px):
        x, y = int(p[0]), int(p[1])
        cv2.circle(vis, (x, y), 5, (0, 0, 255), -1)
        cv2.putText(vis, str(i), (x + 6, y - 6), cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0, 255, 0), 1)

    status = "OK" if result.valid else f"INVALID: {result.reason}"
    cv2.putText(vis, status, (20, 30), cv2.FONT_HERSHEY_SIMPLEX, 0.8, (0, 255, 0) if result.valid else (0, 0, 255), 2)
    cv2.putText(vis, f"reproj: {result.reprojection_error_px:.3f}px", (20, 60), cv2.FONT_HERSHEY_SIMPLEX, 0.7, (255, 255, 0), 2)

    for i, p3 in enumerate(result.corners_cam_mm):
        txt = f"P{i} X:{p3[0]:.1f} Y:{p3[1]:.1f} Z:{p3[2]:.1f} mm"
        cv2.putText(vis, txt, (20, 95 + i * 25), cv2.FONT_HERSHEY_SIMPLEX, 0.6, (255, 255, 255), 2)

    return vis


def _detect_one_frame(
    frame: np.ndarray,
    camera_matrix: np.ndarray,
    dist_coeffs: np.ndarray,
    cfg: dict,
) -> DetectionResult:
    gray, binary = preprocess_frame(frame, cfg)
    outer, inner = detect_frame_quads(binary, cfg)

    if outer is None or inner is None:
        empty = np.zeros((4, 2), dtype=np.float32)
        empty3d = np.zeros((4, 3), dtype=np.float32)
        return DetectionResult(
            corners_px=empty,
            centerline_corners_px=empty,
            corners_cam_mm=empty3d,
            rvec=np.zeros((3, 1), dtype=np.float32),
            tvec=np.zeros((3, 1), dtype=np.float32),
            reprojection_error_px=float("inf"),
            valid=False,
            reason="no valid outer-inner quad",
        )

    center_corners = centerline_from_outer_inner(gray, outer, inner)

    ok, rvec, tvec, corners_cam, err = solve_pose_and_3d(
        image_points=center_corners,
        camera_matrix=camera_matrix,
        dist_coeffs=dist_coeffs,
        side_length_mm=float(cfg["square"]["side_length_mm"]),
    )

    if not ok:
        empty3d = np.zeros((4, 3), dtype=np.float32)
        return DetectionResult(
            corners_px=outer,
            centerline_corners_px=center_corners,
            corners_cam_mm=empty3d,
            rvec=np.zeros((3, 1), dtype=np.float32),
            tvec=np.zeros((3, 1), dtype=np.float32),
            reprojection_error_px=float("inf"),
            valid=False,
            reason="solvePnP failed",
        )

    max_err = float(cfg["quality"]["max_reprojection_error_px"])
    valid = err <= max_err
    reason = "ok" if valid else f"reprojection too high: {err:.3f}px"

    return DetectionResult(
        corners_px=outer,
        centerline_corners_px=center_corners,
        corners_cam_mm=corners_cam,
        rvec=rvec,
        tvec=tvec,
        reprojection_error_px=err,
        valid=valid,
        reason=reason,
    )


def main() -> None:
    parser = argparse.ArgumentParser(description="Realtime black square-frame 3D corner detector")
    parser.add_argument("--camera", default="config/camera.yaml", help="camera YAML path")
    parser.add_argument("--detector", default="config/detector.yaml", help="detector YAML path")
    parser.add_argument("--show-binary", action="store_true", help="show binary image")
    args = parser.parse_args()

    cam_cfg = _load_yaml(Path(args.camera))
    det_cfg = _load_yaml(Path(args.detector))

    camera_matrix = np.array(cam_cfg["camera_matrix"], dtype=np.float32)
    dist_coeffs = np.array(cam_cfg["dist_coeffs"], dtype=np.float32)

    rt = det_cfg["runtime"]
    cap = cv2.VideoCapture(int(rt["camera_id"]))
    cap.set(cv2.CAP_PROP_FRAME_WIDTH, int(rt["width"]))
    cap.set(cv2.CAP_PROP_FRAME_HEIGHT, int(rt["height"]))

    if not cap.isOpened():
        raise RuntimeError("Failed to open camera")

    print("Press q to quit, s to save current frame")

    while True:
        ok, frame = cap.read()
        if not ok:
            continue

        result = _detect_one_frame(frame, camera_matrix, dist_coeffs, det_cfg)
        vis = _draw_result(frame, result)
        cv2.imshow("black square frame 3D", vis)

        if args.show_binary:
            _, binary = preprocess_frame(frame, det_cfg)
            cv2.imshow("binary", binary)

        if result.valid:
            now = time.strftime("%H:%M:%S")
            p0 = result.corners_cam_mm[0]
            print(f"[{now}] reproj={result.reprojection_error_px:.3f}px P0(mm)=({p0[0]:.2f},{p0[1]:.2f},{p0[2]:.2f})")

        key = cv2.waitKey(1) & 0xFF
        if key == ord("q"):
            break
        if key == ord("s"):
            fn = f"capture_{int(time.time())}.png"
            cv2.imwrite(fn, frame)
            print(f"Saved {fn}")

    cap.release()
    cv2.destroyAllWindows()


if __name__ == "__main__":
    main()
