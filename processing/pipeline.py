"""
Image processing pipeline: grayscale conversion + binary thresholding.

The NXP track has black-bordered lanes on a lighter surface.  After
THRESH_BINARY_INV the black borders become white (255) on a black (0)
background, which makes downstream edge detection straightforward.
"""

import cv2
import numpy as np

import config


class ImageProcessor:
    """Stateless image processing helpers."""

    def __init__(self, threshold: int = config.THRESHOLD_VALUE) -> None:
        self._threshold = threshold

    def to_grayscale(self, frame: np.ndarray) -> np.ndarray:
        """Convert a BGR frame to single-channel grayscale (no-op if already gray)."""
        if frame.ndim == 2 or (frame.ndim == 3 and frame.shape[2] == 1):
            return frame  # already grayscale — skip conversion
        frame = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
        return frame

    def apply_threshold(
        self, gray: np.ndarray, threshold: int | None = None
    ) -> np.ndarray:
        """
        Apply inverse binary threshold.

        Pixels darker than *threshold* become 255 (white), everything
        else becomes 0 (black).  This isolates the dark track borders.
        """
        thresh = threshold if threshold is not None else self._threshold
        _, binary = cv2.threshold(
            gray, thresh, 255, cv2.THRESH_BINARY_INV
        )
        return binary

    def process(self, frame: np.ndarray) -> np.ndarray:
        """Convenience: grayscale → threshold in one call."""
        gray = self.to_grayscale(frame)
        return self.apply_threshold(gray)
