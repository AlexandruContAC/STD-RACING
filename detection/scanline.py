"""
Scan-line-based track detection.

For each configured horizontal row (scan line) in the binary image:
  1. Scan from the LEFT edge inward to the MIDPOINT for the left border.
  2. Scan from the RIGHT edge inward to the MIDPOINT for the right border.
  3. Compute the lane center from the detected edges.

This split-search ensures that when only one border is visible (e.g. sharp
curve), the other side correctly returns None — enabling single-edge fallback.

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

        weighted_center = self._weighted_center(centers)

        return ScanResult(
            rows=list(self._rows),
            left_edges=left_edges,
            right_edges=right_edges,
            centers=centers,
            weighted_center=weighted_center,
            image_width=w,
        )

    # ── edge finding ──────────────────────────────────────────────────────

    @staticmethod
    def _find_edge_left_half(row: np.ndarray, mid: int) -> Optional[int]:
        """Scan the LEFT half [0, mid) for the first white pixel (left→right).

        Returns the X position of the left border, or None if no white pixel
        exists in the left half of the row.
        """
        left_half = row[:mid]
        indices = np.nonzero(left_half)[0]
        return int(indices[0]) if len(indices) > 0 else None

    @staticmethod
    def _find_edge_right_half(row: np.ndarray, mid: int) -> Optional[int]:
        """Scan the RIGHT half [mid, width) for the last white pixel (right→left).

        Returns the X position of the right border, or None if no white pixel
        exists in the right half of the row.
        """
        right_half = row[mid:]
        indices = np.nonzero(right_half)[0]
        return int(indices[-1] + mid) if len(indices) > 0 else None

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

