"""
Pixy2 camera backend – reads raw Bayer frames from the compiled
get_raw_frame C++ binary via subprocess stdout.

The binary outputs one _RAW_WIDTH × _RAW_HEIGHT grayscale image per
iteration, 1 byte per pixel, in a continuous stream.  We return a
standard (H, W) uint8 NumPy array – same contract as every other backend.
"""

import os
import queue
import subprocess
import threading
import time
import numpy as np

from camera.base import CameraBase

# Pixy2 raw-frame resolution constants (must match the C++ binary)
_RAW_WIDTH   = 316   # must match PIXY2_RAW_FRAME_WIDTH  in libpixyusb2.h — do not change
_RAW_HEIGHT  = 208   # must match PIXY2_RAW_FRAME_HEIGHT in libpixyusb2.h — do not change
_FRAME_BYTES = _RAW_WIDTH * _RAW_HEIGHT

# Path to the compiled get_raw_frame binary
_BINARY_PATH = os.path.join(
    os.path.dirname(os.path.dirname(os.path.abspath(__file__))),
    "get_raw_frame", "get_raw_frame",
)


class Pixy2Camera(CameraBase):
    """Captures frames from a Pixy2 camera via the get_raw_frame binary."""

    def __init__(self) -> None:
        self._process: subprocess.Popen | None = None
        self._reader_thread: threading.Thread | None = None
        self._frame_queue: queue.Queue = queue.Queue(maxsize=1)
        self._stop_event: threading.Event = threading.Event()
        self._initialised = False
        self._fps = 0.0
        # Pre-allocated read buffer – reused every frame to avoid heap churn
        self._buf = bytearray(_FRAME_BYTES)

    # ── internal ───────────────────────────────────────────────────────────

    def _reader_loop(self) -> None:
        """Background thread: reads frames from the C++ process stdout."""
        proc = self._process
        buf = self._buf
        last_time = time.perf_counter()

        view = memoryview(buf)
        while not self._stop_event.is_set():
            # Read exactly _FRAME_BYTES using a memoryview loop to handle
            # partial pipe reads without allocating new buffers each frame
            pos = 0
            while pos < _FRAME_BYTES:
                n = proc.stdout.readinto(view[pos:])
                if n == 0:  # EOF / process exited
                    self._stop_event.set()
                    return
                pos += n

            # Measure actual producer (camera) FPS here, not consumer FPS
            now = time.perf_counter()
            self._fps = 1.0 / max(now - last_time, 1e-9)
            last_time = now

            # Keep only the latest frame; drop stale ones without exceptions
            snapshot = bytes(buf)
            if self._frame_queue.full():
                try:
                    self._frame_queue.get_nowait()
                except queue.Empty:
                    pass
            self._frame_queue.put_nowait(snapshot)

    # ── public API ────────────────────────────────────────────────────────

    def open(self) -> None:
        if not os.path.isfile(_BINARY_PATH):
            raise RuntimeError(
                f"get_raw_frame binary not found at {_BINARY_PATH}. "
                "Build it first with: cd get_raw_frame && make"
            )

        self._stop_event.clear()
        self._process = subprocess.Popen(
            [_BINARY_PATH],
            stdout=subprocess.PIPE,
            stderr=subprocess.PIPE,
        )

        self._reader_thread = threading.Thread(
            target=self._reader_loop, daemon=True
        )
        self._reader_thread.start()
        self._initialised = True

        print(f"[Pixy2] get_raw_frame binary started.  Resolution: {_RAW_WIDTH}×{_RAW_HEIGHT}")

    def read_frame(self) -> np.ndarray:
        """Block until the next frame arrives and return a (H, W) uint8 array."""
        if not self._initialised:
            raise RuntimeError("Pixy2 camera is not open. Call open() first.")
        if self._stop_event.is_set():
            raise RuntimeError("Camera stream ended unexpectedly.")

        try:
            raw = self._frame_queue.get(timeout=5.0)
        except queue.Empty:
            raise TimeoutError("No frame received from Pixy2 within 5 seconds.")

        return np.frombuffer(raw, dtype=np.uint8).reshape((_RAW_HEIGHT, _RAW_WIDTH))

    def get_frame_rate(self) -> float:
        return self._fps

    def close(self) -> None:
        self._stop_event.set()
        if self._process:
            self._process.terminate()
            self._process = None
        if self._reader_thread:
            self._reader_thread.join(timeout=2)
            self._reader_thread = None
        # Drain stale frames so the queue is clean on re-open
        while not self._frame_queue.empty():
            try:
                self._frame_queue.get_nowait()
            except queue.Empty:
                break
        self._initialised = False
        print("[Pixy2] Camera closed.")
