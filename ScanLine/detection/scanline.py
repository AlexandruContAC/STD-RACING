"""
Scan-line-based track detection.

For each configured horizontal row (scan line) in the binary image:
  1. Scan from the MIDPOINT outward toward the LEFT edge for the left border.
  2. Scan from the MIDPOINT outward toward the RIGHT edge for the right border.
  3. Compute the lane center from the detected edges.

Searching from the middle outward finds the nearest border to the car's
center first, which is more robust on sharp curves.

A weighted average across all rows yields a single track-center X that
gives more importance to the rows closer to the car (bottom of image).
"""

from __future__ import annotations

from dataclasses import dataclass, field
from typing import List, Optional

import numpy as np

import config


@dataclass
class ScanResult:
    """Holds per-row detection results and the final weighted center."""

    rows: List[int]                     # Y positions of the scan lines
    left_edges: List[Optional[int]]     # X of left border (None = not found)
    right_edges: List[Optional[int]]    # X of right border
    centers: List[Optional[float]]      # per-row center
    weighted_center: Optional[float]    # combined weighted center X
    image_width: int                    # frame width for reference
    # Intersection fallback data (None when not triggered)
    intersection_row: Optional[int] = None
    intersection_left: Optional[int] = None
    intersection_right: Optional[int] = None
    intersection_center: Optional[float] = None
    lap_finished: bool = False



class ScanLineDetector:
    """Detects NXP track borders using horizontal scan lines."""

    def __init__(
        self,
        rows: List[int] | None = None,
        weights: List[float] | None = None,
        max_lane_width: int | None = None,
        min_lane_width: int | None = None,
        assumed_lane_width: int | None = None,
    ) -> None:
        self._rows = rows or list(config.SCAN_LINE_ROWS)
        self._weights = weights or list(config.SCAN_LINE_WEIGHTS)
        self._max_lane_width = (
            max_lane_width if max_lane_width is not None
            else config.MAX_LANE_WIDTH
        )
        self._min_lane_width = (
            min_lane_width if min_lane_width is not None
            else config.MIN_LANE_WIDTH
        )
        self._half_lane = (
            (assumed_lane_width if assumed_lane_width is not None
             else config.ASSUMED_LANE_WIDTH) / 2.0
        )

        if len(self._rows) != len(self._weights):
            raise ValueError("rows and weights must have the same length")

    # ── public ────────────────────────────────────────────────────────────

    def detect(self, binary: np.ndarray) -> ScanResult:
        """
        Run scan-line detection on a binary (thresholded) image.

        Parameters
        ----------
        binary : np.ndarray
            Single-channel image where track borders are 255 (white) and
            background is 0 (black).

        Returns
        -------
        ScanResult
        """
        h, w = binary.shape[:2]
        mid = w // 2

        left_edges: List[Optional[int]] = []
        right_edges: List[Optional[int]] = []
        centers: List[Optional[float]] = []

        for row_y in self._rows:
            if row_y < 0 or row_y >= h:
                left_edges.append(None)
                right_edges.append(None)
                centers.append(None)
                continue

            scan_row = binary[row_y, :]

            # Split search: left half and right half independently
            left = self._find_edge_left_half(scan_row, mid)
            right = self._find_edge_right_half(scan_row, mid)

            left_edges.append(left)
            right_edges.append(right)

            center = self._compute_center(left, right, w)
            centers.append(center)

        # ── Majority voting for single-edge detections ────────────────────────
        only_left_count = sum(1 for l, r in zip(left_edges, right_edges) if l is not None and r is None)
        only_right_count = sum(1 for l, r in zip(left_edges, right_edges) if r is not None and l is None)

        if only_left_count > only_right_count:
            # Left side is dominant; eliminate "only right" detections
            for i in range(len(centers)):
                if right_edges[i] is not None and left_edges[i] is None:
                    centers[i] = None
        elif only_right_count > only_left_count:
            # Right side is dominant; eliminate "only left" detections
            for i in range(len(centers)):
                if left_edges[i] is not None and right_edges[i] is None:
                    centers[i] = None

        # ── Intersection fallback ──────────────────────────────────────────
        # The top 3 scan lines (rows 100, 105, 110 — last 3 in the array)
        # are the farthest from the car.  If ALL three have no valid center
        # it likely means the car is approaching an intersection.  Fall back
        # to a single scan at INTERSECTION_ROW to get a heading.
        top3_centers = centers[-3:]  # rows 110, 105, 100
        int_center: Optional[float] = None
        int_left: Optional[int] = None
        int_right: Optional[int] = None
        int_row_y: Optional[int] = None

        if all(c is None for c in top3_centers):
            int_row_y = config.INTERSECTION_ROW
            if 0 <= int_row_y < h:
                int_scan = binary[int_row_y, :]
                int_left = self._find_edge_left_half(int_scan, mid)
                int_right = self._find_edge_right_half(int_scan, mid)
                int_center = self._compute_center(
                    int_left, int_right, w
                )

        if int_center is not None:
            weighted_center = int_center
        else:
            weighted_center = self._weighted_center(centers)

        # ── Lap detection ──────────────────────────────────────────────────
        # Check closest 2 scanlines (the first two configured rows, e.g., 190, 185)
        # We search from the found white point towards the center/edges to find
        # the finish line dashes. If a second white point is found, the lap is finished.
        lap_finished = False
        for idx in range(min(2, len(self._rows))):
            row_y = self._rows[idx]
            if row_y < 0 or row_y >= h:
                continue

            scan_row = binary[row_y, :]
            left = left_edges[idx]
            right = right_edges[idx]
            jump = 30  # pixels to skip to bypass the line's own thickness

            if left is None or right is None:
                continue

            if left is not None:
                # User's logic: search from left found point to the center
                if any(scan_row[x] == 255 for x in range(left + jump, mid)):
                    lap_finished = True
                # Also search towards the left edge in case 'left' was the inner dash
                if any(scan_row[x] == 255 for x in range(left - jump, -1, -1)):
                    lap_finished = True

            if right is not None:
                # Search from right found point to the center
                if any(scan_row[x] == 255 for x in range(right - jump, mid, -1)):
                    lap_finished = True
                # Also search towards the right edge
                if any(scan_row[x] == 255 for x in range(right + jump, w)):
                    lap_finished = True



        return ScanResult(
            rows=list(self._rows),
            left_edges=left_edges,
            right_edges=right_edges,
            centers=centers,
            weighted_center=weighted_center,
            image_width=w,
            intersection_row=int_row_y if int_center is not None else None,
            intersection_left=int_left if int_center is not None else None,
            intersection_right=int_right if int_center is not None else None,
            intersection_center=int_center,
            lap_finished=lap_finished,
        )

    # ── edge finding ──────────────────────────────────────────────────────

    @staticmethod
    def _find_edge_left_half(row: np.ndarray, mid: int) -> Optional[int]:
        """Scan from the MIDPOINT outward toward the LEFT edge [mid-1 … 0] every 2 pixels.

        Returns the X position of the first white pixel found moving
        leftward from the center, or None if no white pixel exists.
        """
        left_half = row[:mid:2]
        indices = np.nonzero(left_half)[0]
        # Take the rightmost (closest to mid) white pixel
        return int(indices[-1] * 2) if len(indices) > 0 else None

    @staticmethod
    def _find_edge_right_half(row: np.ndarray, mid: int) -> Optional[int]:
        """Scan from the MIDPOINT outward toward the RIGHT edge [mid … width) every 2 pixels.

        Returns the X position of the first white pixel found moving
        rightward from the center, or None if no white pixel exists.
        """
        right_half = row[mid::2]
        indices = np.nonzero(right_half)[0]
        # Take the leftmost (closest to mid) white pixel
        return int(indices[0] * 2 + mid) if len(indices) > 0 else None

    # ── center computation ────────────────────────────────────────────────

    def _compute_center(
        self,
        left: Optional[int],
        right: Optional[int],
        image_width: int,
    ) -> Optional[float]:
        """
        Compute the lane centre for one scan row.

        Cases handled:
          1. Both edges found & gap valid (MIN ≤ gap ≤ MAX) → midpoint.
          2. Both edges found & gap < MIN → same border, single-edge fallback.
          3. Both edges found & gap > MAX → noise/intersection, row rejected.
          4. Only LEFT edge found  → left + half_lane (sharp right curve).
          5. Only RIGHT edge found → right − half_lane (sharp left curve).
          6. Neither edge found    → None (intersection/empty row, ignored).
        """
        # ── Case: both edges found ────────────────────────────────────────
        if left is not None and right is not None:
            lane_width = right - left

            # Case 1: valid lane width
            if self._min_lane_width <= lane_width <= self._max_lane_width:
                return (left + right) / 2.0

            # Case 2: gap too small — both points are on the SAME border
            # (redundancy check for sharp curves / camera distortion)
            if lane_width < self._min_lane_width:
                single_line_center = (left + right) / 2.0
                if single_line_center < (image_width / 2.0):
                    # Border is on the left half → treat as left border
                    return min(single_line_center + self._half_lane, image_width - 1)
                else:
                    # Border is on the right half → treat as right border
                    return max(single_line_center - self._half_lane, 0)

            # Case 3: gap too large — likely intersection or noise
            return None

        # ── Case 4: only left edge found (sharp right curve) ─────────────
        if left is not None and right is None:
            return min(left + self._half_lane, image_width - 1)

        # ── Case 5: only right edge found (sharp left curve) ─────────────
        if right is not None and left is None:
            return max(right - self._half_lane, 0)

        # ── Case 6: neither edge found (intersection / empty row) ────────
        return None

    # ── weighted average ──────────────────────────────────────────────────

    def _weighted_center(
        self, centers: List[Optional[float]]
    ) -> Optional[float]:
        """Compute weighted average of valid per-row centers.

        Rows where center is None (no edges, intersection, or noise) are
        excluded from the sum and their weight is NOT counted — effectively
        redistributing their influence to the remaining valid rows.
        """
        total_weight = 0.0
        weighted_sum = 0.0

        for center, weight in zip(centers, self._weights):
            if center is not None:
                weighted_sum += center * weight
                total_weight += weight

        if total_weight == 0.0:
            return None
        return weighted_sum / total_weight

