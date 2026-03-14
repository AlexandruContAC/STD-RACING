"""Webcam camera backend – wraps cv2.VideoCapture for desktop testing."""

import cv2
import numpy as np

from camera.base import CameraBase
import config


class WebcamCamera(CameraBase):
    """Captures frames from a standard USB webcam (or any V4L2/UVC device)."""

    def __init__(self, device_index: int = config.WEBCAM_INDEX) -> None:
        self._index = device_index
        self._cap = None

    def open(self) -> None:
        self._cap = cv2.VideoCapture(self._index)
        if not self._cap.isOpened():
            raise RuntimeError(
                f"Cannot open webcam at index {self._index}."
            )
        print(f"[Webcam] Opened /dev/video{self._index}")

    def read_frame(self) -> np.ndarray:
        ret, frame = self._cap.read()
        if not ret:
            raise RuntimeError("Failed to read frame from webcam.")

        # Center-crop horizontally to exclude objects on the sides
        if config.CROP_WIDTH_FRACTION > 0:
            h_raw, w_raw = frame.shape[:2]
            trim = int(w_raw * config.CROP_WIDTH_FRACTION)
            frame = frame[:, trim: w_raw - trim]

        # Resize to the expected pipeline dimensions
        frame = cv2.resize(
            frame, (config.FRAME_WIDTH, config.FRAME_HEIGHT)
        )
        return frame

    def close(self) -> None:
        if self._cap is not None:
            self._cap.release()
            self._cap = None
            print("[Webcam] Released.")

    def get_frame_rate(self) -> float:
        if self._cap is not None and self._cap.isOpened():
            # CV_CAP_PROP_FPS is 5
            fps = self._cap.get(cv2.CAP_PROP_FPS)
            return float(fps) if fps > 0 else 30.0
        return 0.0
