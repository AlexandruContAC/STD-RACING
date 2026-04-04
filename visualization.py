"""
Debug visualization – draws scan lines, edges, and steering on the frame.
"""

import cv2
import numpy as np

from detection.scanline import ScanResult
import config

# Colours (BGR)
_GREEN = (0, 255, 0)
_RED = (0, 0, 255)
_CYAN = (255, 255, 0)
_YELLOW = (0, 255, 255)
_WHITE = (255, 255, 255)
_ORANGE = (0, 165, 255)


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
      • intersection scan line (orange, when active)
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

        # MAX_LANE_WIDTH bracket (red) — shows expected lane width for debugging
        half_max = config.MAX_LANE_WIDTH // 2
        bracket_center = int(center) if center is not None else w // 2
        bracket_left = max(bracket_center - half_max, 0)
        bracket_right = min(bracket_center + half_max, w - 1)
        # Horizontal bar
        cv2.line(vis, (bracket_left, row_y), (bracket_right, row_y), _RED, 2)
        # Vertical ticks at ends
        cv2.line(vis, (bracket_left, row_y - 4), (bracket_left, row_y + 4), _RED, 2)
        cv2.line(vis, (bracket_right, row_y - 4), (bracket_right, row_y + 4), _RED, 2)

        # Edges (bright magenta so they stand out from the red bracket)
        if left is not None:
            cv2.circle(vis, (left, row_y), 4, (255, 0, 255), -1)
        if right is not None:
            cv2.circle(vis, (right, row_y), 4, (255, 0, 255), -1)

        # Per-row centre
        if center is not None:
            cv2.circle(vis, (int(center), row_y), 4, _GREEN, -1)

    # ── Intersection scan line (orange) ───────────────────────────────
    if result.intersection_row is not None:
        iy = result.intersection_row
        # Full-width orange line
        cv2.line(vis, (0, iy), (w - 1, iy), _ORANGE, 2)
        # Detected edges
        if result.intersection_left is not None:
            cv2.circle(vis, (result.intersection_left, iy), 5, _ORANGE, -1)
        if result.intersection_right is not None:
            cv2.circle(vis, (result.intersection_right, iy), 5, _ORANGE, -1)
        # Center
        if result.intersection_center is not None:
            cv2.circle(vis, (int(result.intersection_center), iy), 5, _GREEN, -1)
        # Label
        cv2.putText(vis, "INTERSECTION", (5, iy - 6),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.4, _ORANGE, 1)

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
