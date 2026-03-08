#!/usr/bin/env python3
"""
ScanLine – NXP track detection with scan-line steering.

Sends cmd_vel (Twist) directly to the CANHUB-K3 over UDP using
TinyFrame-encoded nanopb protobuf — no ROS 2 bridge required.

Usage:
    python main.py --camera webcam      # desktop testing
    python main.py --camera pixy2       # on the NavQ Plus with Pixy2
    python main.py --camera webcam --no-display   # headless mode
"""

import argparse
import socket
import sys
import os

import cv2

import config
from processing.pipeline import ImageProcessor
from detection.scanline import ScanLineDetector
from steering.controller import SteeringController
from visualization import draw_debug

from synapse_tinyframe import TinyFrameBuilder, SYNAPSE_CMD_VEL_TOPIC
from synapse_msgs import encode_twist

# ── CANHUB-K3 network configuration ──────────────────────────────────────
CANHUBK3_IP = "192.0.2.1"   # CANHUB-K3 static IP (from prj.conf)
CANHUBK3_PORT = 4242         # UDP port (from udp_rx.c MY_PORT)

# ── Driving parameters ───────────────────────────────────────────────────
FORWARD_SPEED = 0.2           # constant forward speed (linear.x)


def build_camera(backend: str):
    """Factory: return the appropriate CameraBase subclass."""
    if backend == "pixy2":
        from camera.pixy2_cam import Pixy2Camera
        return Pixy2Camera()
    elif backend == "webcam":
        from camera.pixy2_cam import Pixy2Camera
        return Pixy2Camera()
        # from camera.webcam import WebcamCamera
        # return WebcamCamera()
    else:
        raise ValueError(f"Unknown camera backend: {backend}")


def main() -> None:
    parser = argparse.ArgumentParser(description="ScanLine track detector")
    parser.add_argument(
        "--camera",
        choices=["pixy2", "webcam"],
        default=config.CAMERA_BACKEND,
        help="Camera backend to use (default: %(default)s)",
    )
    parser.add_argument(
        "--no-display",
        action="store_true",
        help="Run headless (no cv2.imshow window)",
    )
    parser.add_argument(
        "--threshold",
        type=int,
        default=config.THRESHOLD_VALUE,
        help="Binary threshold value 0-255 (default: %(default)s)",
    )
    parser.add_argument(
        "--target-ip",
        default=CANHUBK3_IP,
        help="CANHUB-K3 IP address (default: %(default)s)",
    )
    parser.add_argument(
        "--target-port",
        type=int,
        default=CANHUBK3_PORT,
        help="CANHUB-K3 UDP port (default: %(default)s)",
    )
    args = parser.parse_args()

    # ── Initialise components ─────────────────────────────────────────────
    camera = build_camera(args.camera)
    processor = ImageProcessor(threshold=args.threshold)
    detector = ScanLineDetector()
    controller = SteeringController()

    display_available = os.environ.get("DISPLAY") or os.environ.get("WAYLAND_DISPLAY")
    show_display = True

    if show_display and not display_available:
        print("[ScanLine] No display detected. Switching to headless mode.")
        show_display = True

    # ── Open camera ───────────────────────────────────────────────────────
    try:
        camera.open()
    except RuntimeError as exc:
        print(f"[ERROR] {exc}", file=sys.stderr)
        sys.exit(1)

    # ── Initialize UDP socket + TinyFrame builder ─────────────────────────
    tf = TinyFrameBuilder()
    udp_sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    target_addr = (args.target_ip, args.target_port)
    print(f"[ScanLine] Sending cmd_vel to {target_addr[0]}:{target_addr[1]} via UDP/TinyFrame")

    print("[ScanLine] Pipeline running. Press Ctrl+C to quit.")
    #camera.set_lamp(True)
    try:
        while True:
            # 1. Capture
            frame = camera.read_frame()

            # 2. Process (grayscale → threshold)
            binary = processor.process(frame)

            # 3. Detect track via scan lines
            result = detector.detect(binary)

            # 4. Compute steering
            steering = controller.compute(result.weighted_center)

            # 5. Send cmd_vel over UDP/TinyFrame
            steer_value = float(steering) if steering is not None else 0.0
            payload = encode_twist(linear_x=FORWARD_SPEED, angular_z=steer_value)
            frame_bytes = tf.build_frame(SYNAPSE_CMD_VEL_TOPIC, payload)
            try:
                udp_sock.sendto(frame_bytes, target_addr)
            except OSError as e:
                print(f"\r[ScanLine] UDP send error: {e}", end="")

            # 6. Log
            print(
                f"fps={camera.get_frame_rate():.2f}  "
                f"center={result.weighted_center!s:>8s}  "
                f"steering={steering:+.4f}",
                end="\r",
            )

            #7. Visualize (optional)
            if show_display:
                debug_frame = draw_debug(frame, result, steering)
                cv2.imshow("ScanLine Debug", debug_frame)
                cv2.imshow("Binary", binary)
                if cv2.waitKey(1) & 0xFF == ord("q"):
                    break

    except KeyboardInterrupt:
        print("\n[ScanLine] Interrupted.")
    finally:
        # Send a stop command before exiting
        try:
            stop_payload = encode_twist(linear_x=0.0, angular_z=0.0)
            stop_frame = tf.build_frame(SYNAPSE_CMD_VEL_TOPIC, stop_payload)
            udp_sock.sendto(stop_frame, target_addr)
        except OSError:
            pass
        udp_sock.close()
        camera.close()
        if show_display:
            cv2.destroyAllWindows()

    print("[ScanLine] Done.")


if __name__ == "__main__":
    main()
