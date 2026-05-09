"""Camera intrinsic parameters.

Replace these values with real calibration results for accurate metric output.
Units are pixels.
"""

# Approximate intrinsics for QVGA center crop. Replace after calibration.
FX = 290.0
FY = 290.0
CX = 160.0
CY = 120.0

CAMERA_MATRIX = [
    [FX, 0.0, CX],
    [0.0, FY, CY],
    [0.0, 0.0, 1.0],
]
