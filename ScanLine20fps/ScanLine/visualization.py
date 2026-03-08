"""
Debug visualization – draws scan lines, edges, and steering on the frame.
"""

import cv2
import numpy as np

from detection.scanline import ScanResult

# Colours (BGR)
_GREEN = (0, 255, 0)
_RED = (0, 0, 255)
_CYAN = (255, 255, 0)
_YELLOW = (0, 255, 255)
_WHITE = (255, 255, 255)


def draw_debug(
    frame: np.ndarray,
    result: ScanResult,
    steering: float,
) -> np.ndarray:
    """
    Return a copy of *frame* with debug overlays:
      • horizontal scan lines (cyan)
      • left edges (red circles)
      • right edges (red circles)
      • per-row centre (green circles)
      • weighted centre (yellow vertical line)
      • steering arrow (green)
    """
    vis = frame.copy()
    h, w = vis.shape[:2]

    for row_y, left, right, center in zip(
        result.rows, result.left_edges, result.right_edges, result.centers
    ):
        # Scan line
        cv2.line(vis, (0, row_y), (w - 1, row_y), _CYAN, 1)

        # Edges
        if left is not None:
            cv2.circle(vis, (left, row_y), 4, _RED, -1)
        if right is not None:
            cv2.circle(vis, (right, row_y), 4, _RED, -1)

        # Per-row centre
        if center is not None:
            cv2.circle(vis, (int(center), row_y), 4, _GREEN, -1)

    # Weighted centre
    if result.weighted_center is not None:
        cx = int(result.weighted_center)
        cv2.line(vis, (cx, 0), (cx, h - 1), _YELLOW, 2)

    # Image centre reference
    img_cx = w // 2
    cv2.line(vis, (img_cx, 0), (img_cx, h - 1), _WHITE, 1)

    # Steering text
    direction = "RIGHT" if steering > 0 else "LEFT" if steering < 0 else "STRAIGHT"
    cv2.putText(
        vis,
        f"Steer: {steering:+.3f} ({direction})",
        (10, 20),
        cv2.FONT_HERSHEY_SIMPLEX,
        0.5,
        _GREEN,
        1,
    )

    return vis
