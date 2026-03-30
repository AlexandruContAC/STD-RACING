"""
Central configuration for the ScanLine track detection project.
All tunable parameters are collected here for easy adjustment.
"""

# ---------------------------------------------------------------------------
# Camera backend: "pixy2" | "pixy2fast" | "webcam" | "mipi"
# ---------------------------------------------------------------------------
CAMERA_BACKEND = "pixy2fast"
WEBCAM_INDEX = 0  # /dev/video<N>

# ---------------------------------------------------------------------------
# MIPI camera settings for NavQPlus / i.MX8
# ---------------------------------------------------------------------------
MIPI_GSTREAMER_PIPELINE = "v4l2src device=/dev/video3 ! video/x-raw,width=640,height=480 ! videoconvert ! appsink"

# ---------------------------------------------------------------------------
# Frame dimensions (Pixy2 raw Bayer frame is fixed at 316x208)
# When using a webcam the frame is resized to these dimensions so that the
# rest of the pipeline stays resolution-agnostic.
# ---------------------------------------------------------------------------
FRAME_WIDTH  = 316   # portrait: width < height
FRAME_HEIGHT = 208   # full Pixy2 sensor height
CAMERA_CENTER_X = FRAME_WIDTH // 2 + 10  # horizontal midpoint of the frame

# Fraction of the raw frame width to trim from EACH side before resize.
# 0.0 = no crop, 0.15 = remove 15 % from left AND right (keeps centre 70 %).
# Raise this value to exclude objects at the edges of the camera view.
CROP_WIDTH_FRACTION = 0.0

# ---------------------------------------------------------------------------
# Image processing
# ---------------------------------------------------------------------------
THRESHOLD_VALUE = 42  # 0-255, pixels darker than this become white after INV

# ---------------------------------------------------------------------------
# Scan line detection
# Row Y positions (0 = top).  Lower rows are closer to the car and are
# weighted more heavily for steering.
# ---------------------------------------------------------------------------
SCAN_LINE_ROWS    = [190,185,180,175,170,165,160,155,150,145,140,135,130,125,120,115,110,105,100]
# 23 lines. Distant lines (lower Y, e.g. 60) are at the end, so we give them higher weights.
# The weights must sum to 1.0. Let's make an increasing sequence.
# Sum is 1.0. The weights increase linearly towards the distant scanlines.
SCAN_LINE_WEIGHTS = [0.010, 0.015, 0.020, 0.025, 0.030, 0.035, 0.040, 0.040, 0.040, 0.040, 0.045, 0.045, 0.045, 0.045, 0.050, 0.050, 0.050, 0.050, 0.055]  # sum = 1.0

# Maximum / minimum allowed distance (in pixels) between the left and
# right edges on a single scan row.  Rows outside this range are treated
# as invalid (crossover or noise) and their weight is redistributed.

MAX_LANE_WIDTH = 285
MIN_LANE_WIDTH = 30

# Assumed lane width in pixels.  Used for single-edge fallback: when only
# one border is visible (e.g. sharp curve), the centre is estimated as
# edge ± ASSUMED_LANE_WIDTH / 2.
ASSUMED_LANE_WIDTH = 340

# ---------------------------------------------------------------------------
# Steering
# ---------------------------------------------------------------------------
STEERING_KP = 0.0148  # proportional gain
STEERING_KI = 0.0    # integral gain   (stubbed)
STEERING_KD = 0.085 # derivative gain (stubbed)

HEADLESS_SPEED = 0.5   # default auto-mode speed when no display is active

# ---------------------------------------------------------------------------
# LIDAR (STL-27L DTOF) Configuration
# ---------------------------------------------------------------------------
LIDAR_PORT = "/dev/ttymxc2"            # UART port for the STL-27L on NavQ Plus
LIDAR_BAUDRATE = 921600                # STL-27L baud rate (8N1, no flow control)
LIDAR_BRAKE_THRESHOLD_CM = 45.0        # Emergency brake if obstacle closer than this (cm)
LIDAR_FRONT_ANGLE_RANGE = (-35.0, 35.0)  # Forward 90° cone in degrees (0° = straight ahead)

# ---------------------------------------------------------------------------
# Visualization / debug
# ---------------------------------------------------------------------------
SHOW_DEBUG_WINDOW = True
