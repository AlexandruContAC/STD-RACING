"""
Central configuration for the ScanLine track detection project.
All tunable parameters are collected here for easy adjustment.
"""

# ---------------------------------------------------------------------------
# Camera backend: "pixy2" | "webcam" | "mipi"
# ---------------------------------------------------------------------------
CAMERA_BACKEND = "webcam"
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

# Fraction of the raw frame width to trim from EACH side before resize.
# 0.0 = no crop, 0.15 = remove 15 % from left AND right (keeps centre 70 %).
# Raise this value to exclude objects at the edges of the camera view.
CROP_WIDTH_FRACTION = 0.0

# ---------------------------------------------------------------------------
# Image processing
# ---------------------------------------------------------------------------
THRESHOLD_VALUE = 40  # 0-255, pixels darker than this become white after INV

# ---------------------------------------------------------------------------
# Scan line detection
# Row Y positions (0 = top).  Lower rows are closer to the car and are
# weighted more heavily for steering.
# ---------------------------------------------------------------------------
SCAN_LINE_ROWS    = [90,110,120,130,150,170,190]
SCAN_LINE_WEIGHTS = [0.1,0.15,0.2,0.2,0.15,0.1,0.05]  # sum = 1.0

# Maximum / minimum allowed distance (in pixels) between the left and
# right edges on a single scan row.  Rows outside this range are treated
# as invalid (crossover or noise) and their weight is redistributed.

MAX_LANE_WIDTH = 260
MIN_LANE_WIDTH = 30

# Assumed lane width in pixels.  Used for single-edge fallback: when only
# one border is visible (e.g. sharp curve), the centre is estimated as
# edge ± ASSUMED_LANE_WIDTH / 2.
ASSUMED_LANE_WIDTH = 290

# ---------------------------------------------------------------------------
# Steering
# ---------------------------------------------------------------------------
STEERING_KP = 0.02   # proportional gain
STEERING_KI = 0.0    # integral gain   (stubbed)
STEERING_KD = 0.02    # derivative gain (stubbed)

# ---------------------------------------------------------------------------
# Visualization / debug
# ---------------------------------------------------------------------------
SHOW_DEBUG_WINDOW = True

# ---------------------------------------------------------------------------
# LIDAR (STL-27L DTOF) Configuration
# ---------------------------------------------------------------------------
LIDAR_PORT = "/dev/ttymxc2"            # UART port for the STL-27L on NavQ Plus
LIDAR_BAUDRATE = 921600                # STL-27L baud rate (8N1, no flow control)
LIDAR_BRAKE_THRESHOLD_CM = 10.0        # Emergency brake if obstacle closer than this (cm)
LIDAR_FRONT_ANGLE_RANGE = (-45.0, 45.0)  # Forward 90° cone in degrees (0° = straight ahead)
