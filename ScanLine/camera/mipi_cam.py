"""MIPI camera backend – wraps cv2.VideoCapture with GStreamer for i.MX8 / NavQPlus."""

import cv2
import numpy as np

from camera.base import CameraBase
import config


class MipiCamera(CameraBase):
    """Captures frames from a MIPI camera via GStreamer."""

    def __init__(self, pipeline: str = config.MIPI_GSTREAMER_PIPELINE) -> None:
        self._pipeline = pipeline
        self._cap = None

    def open(self) -> None:
        self._cap = cv2.VideoCapture(self._pipeline, cv2.CAP_GSTREAMER)
        if not self._cap.isOpened():
            raise RuntimeError(
                f"Cannot open MIPI camera with pipeline: {self._pipeline}"
            )
        print(f"[MipiCam] Opened GStreamer pipeline: {self._pipeline}")

    def read_frame(self) -> np.ndarray:
        ret, frame = self._cap.read()
        if not ret:
            raise RuntimeError("Failed to read frame from MIPI camera.")

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
            print("[MipiCam] Released.")
