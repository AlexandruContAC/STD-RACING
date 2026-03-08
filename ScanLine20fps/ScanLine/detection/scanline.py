"""
Scan-line-based track detection.

For each configured horizontal row (scan line) in the binary image:
  1. Scan from the left edge inward until a white (255) pixel is found →
     that is the LEFT border.
  2. Scan from the right edge inward until a white pixel is found → RIGHT
     border.
  3. Track center at that row = (left + right) / 2.

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

            left = self._find_edge_from_left(scan_row)
            right = self._find_edge_from_right(scan_row)

            left_edges.append(left)
            right_edges.append(right)

            center = self._compute_center(left, right, w)
            centers.append(center)

        weighted_center = self._weighted_center(centers)

        return ScanResult(
            rows=list(self._rows),
            left_edges=left_edges,
            right_edges=right_edges,
            centers=centers,
            weighted_center=weighted_center,
            image_width=w,
        )

    # ── internals ─────────────────────────────────────────────────────────

    def _compute_center(
        self,
        left: Optional[int],
        right: Optional[int],
        image_width: int,
    ) -> Optional[float]:
        """
        Compute the lane centre for one scan row.

        Cases:
          • Both edges found & gap within bounds → midpoint.
          • Only LEFT edge found  → left + half_lane (clamped to image).
          • Only RIGHT edge found → right − half_lane (clamped to image).
          • Neither edge / gap out of bounds → None.
        """
        if left is not None and right is not None and right > left:
            lane_width = right - left
            if self._min_lane_width <= lane_width <= self._max_lane_width:
                # Valid lane: gap is between MIN and MAX
                return (left + right) / 2.0
            elif lane_width < self._min_lane_width:
                # Gap is too small: left and right points likely belong to the
                # SAME dark line (e.g. thick track border).
                # Treat it as a single edge located at the midpoint of the gap.
                single_line_center = (left + right) / 2.0
                
                # Estimate if this single line is the left or right border
                # based on which half of the image it's in
                if single_line_center < (image_width / 2.0):
                    # Line is on the left half -> assume it's the left border
                    return min(single_line_center + self._half_lane, image_width - 1)
                else:
                    # Line is on the right half -> assume it's the right border
                    return max(single_line_center - self._half_lane, 0)
            else:
                # Gap > MAX_LANE_WIDTH -> reject row (crossover/noise)
                return None

        # ── Single-edge fallback (sharp curves, only one point found) ────
        if left is not None and right is None:
            return min(left + self._half_lane, image_width - 1)

        if right is not None and left is None:
            return max(right - self._half_lane, 0)

        return None

    # ── internals ─────────────────────────────────────────────────────────

    @staticmethod
    def _find_edge_from_left(row: np.ndarray) -> Optional[int]:
        """Scan left→right for the first white pixel."""
        indices = np.nonzero(row)[0]
        return int(indices[0]) if len(indices) > 0 else None

    @staticmethod
    def _find_edge_from_right(row: np.ndarray) -> Optional[int]:
        """Scan right→left for the first white pixel."""
        indices = np.nonzero(row)[0]
        return int(indices[-1]) if len(indices) > 0 else None

    def _weighted_center(
        self, centers: List[Optional[float]]
    ) -> Optional[float]:
        """Compute weighted average of valid per-row centers."""
        total_weight = 0.0
        weighted_sum = 0.0

        for center, weight in zip(centers, self._weights):
            if center is not None:
                weighted_sum += center * weight
                total_weight += weight

        if total_weight == 0.0:
            return None
        return weighted_sum / total_weight
