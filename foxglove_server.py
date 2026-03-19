"""
Foxglove WebSocket streamer for ScanLine debug views.

Uses the foxglove-sdk to stream CompressedImage data
to Foxglove Studio over a WebSocket connection.

Requires: pip install foxglove-sdk
"""

import time
import cv2

from foxglove import start_server
from foxglove.channels import CompressedImageChannel
from foxglove.schemas import CompressedImage, Timestamp


class FoxgloveStreamer:
    """Streams OpenCV frames to Foxglove Studio via WebSocket."""

    def __init__(self, port: int = 8765):
        self.port = port
        self.server = None
        self.debug_channel = None
        self.binary_channel = None

    def start(self):
        """Starts the Foxglove WebSocket server."""
        self.server = start_server(host="0.0.0.0", port=self.port)
        self.debug_channel = CompressedImageChannel("/camera/debug")
        self.binary_channel = CompressedImageChannel("/camera/binary")
        print(f"[Foxglove] Server started on ws://0.0.0.0:{self.port}")
        url = self.server.app_url()
        if url:
            print(f"[Foxglove] Open in browser: {url}")

    def stop(self):
        """Stops the Foxglove server."""
        if self.server is not None:
            self.server.stop()
            print("[Foxglove] Server stopped.")

    def _cv2_to_compressed_image(self, frame):
        """Converts an OpenCV frame to a foxglove CompressedImage."""
        if frame is None:
            return None

        encode_param = [int(cv2.IMWRITE_JPEG_QUALITY), 70]
        success, encoded = cv2.imencode('.jpg', frame, encode_param)
        if not success:
            return None

        now = time.time()
        sec = int(now)
        nsec = int((now - sec) * 1e9)

        return CompressedImage(
            timestamp=Timestamp(sec=sec, nsec=nsec),
            frame_id="camera",
            data=encoded.tobytes(),
            format="jpeg",
        )

    def update_debug_frame(self, frame):
        """Sends the debug frame to /camera/debug."""
        if self.debug_channel is None:
            return
        msg = self._cv2_to_compressed_image(frame)
        if msg is not None:
            self.debug_channel.log(msg)

    def update_binary_frame(self, frame):
        """Sends the binary frame to /camera/binary."""
        if self.binary_channel is None:
            return
        msg = self._cv2_to_compressed_image(frame)
        if msg is not None:
            self.binary_channel.log(msg)
