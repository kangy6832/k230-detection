"""Runtime configuration for square frame 3D corner detection on K230 CanMV."""

# Square target definition (in centimeters)
SQUARE_SIZE_CM = 10.0
LINE_WIDTH_CM = 0.3

# Camera capture settings
# Use QVGA first for speed. Switch to Sensor.VGA for higher precision if needed.
FRAME_SIZE = "QVGA"
PIXFORMAT = "GRAYSCALE"

# Optional rectangular region-of-interest: (x, y, w, h)
USE_ROI = False
ROI = (0, 0, 320, 240)

# Rectangle detector threshold. Higher means stricter edge strength.
RECT_THRESHOLD = 7000

# Candidate filtering
MIN_RECT_AREA_PX = 900
MAX_RECT_AREA_PX = 90000
MAX_ASPECT_RATIO_DEVIATION = 0.25
MAX_SIDE_RELATIVE_ERROR = 0.18
MIN_MAGNITUDE = 3500

# Pose validity
MAX_REPROJECTION_ERROR_PX = 3.5
MIN_CAMERA_Z_CM = 8.0
MAX_CAMERA_Z_CM = 150.0

# Temporal smoothing
ENABLE_SMOOTHING = True
EMA_ALPHA = 0.45

# Display and debug
PRINT_EVERY_N_FRAMES = 5
DRAW_DEBUG_OVERLAY = True

# Lens correction in case of wide-angle distortion
USE_LENS_CORR = False
LENS_CORR_STRENGTH = 1.8
