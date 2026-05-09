# K230 Black Square Frame 3D Corner Detection

This project detects the four corners of a black square wireframe and outputs each corner 3D coordinate in the camera coordinate frame.

## Target

- Shape: black square wireframe
- Side length: 10.0 cm
- Line width: 0.3 cm
- Output: TL, TR, BR, BL 3D coordinates in cm

## Files

- `main.py`: real-time loop on K230 CanMV
- `geometry.py`: corner ordering, filtering, homography, planar pose, reprojection check
- `camera_params.py`: camera intrinsics (replace with calibration values)
- `config.py`: runtime parameters

## Coordinate Convention

Camera coordinate frame:

- X: right
- Y: down
- Z: forward (from camera to target)

Corner order:

- TL -> TR -> BR -> BL

## Run

1. Copy all files to the board workspace (or SD card project folder).
2. Open `main.py` in CanMV IDE.
3. Click run.
4. Observe LCD/IDE overlay and serial prints.

## Important Notes

1. The current `camera_params.py` uses approximate intrinsics. Replace with your own calibration values for accurate metric output.
2. This is a monocular solution with planar target assumption.
3. `UNRELIABLE` status means the frame failed geometric or reprojection checks.

## Tuning for 25 FPS

1. Keep `FRAME_SIZE = "QVGA"` first.
2. Increase `RECT_THRESHOLD` if false positives are frequent.
3. Reduce `DRAW_DEBUG_OVERLAY` or `PRINT_EVERY_N_FRAMES` if FPS is low.
4. Enable ROI (`USE_ROI = True`) once target region is stable.

## Suggested Acceptance Checks

1. Detection stability in 30-60 cm range.
2. Reconstructed side length close to 10 cm.
3. Runtime near 25 FPS in typical indoor lighting.
