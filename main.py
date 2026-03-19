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
from detection.lidar import LidarSensor
from steering.controller import SteeringController
from visualization import draw_debug

from synapse_tinyframe import TinyFrameBuilder, SYNAPSE_CMD_VEL_TOPIC
from synapse_msgs import encode_twist

from foxglove_server import FoxgloveStreamer

# ── CANHUB-K3 network configuration ──────────────────────────────────────
CANHUBK3_IP = "192.0.2.1"   # CANHUB-K3 static IP (from prj.conf)
CANHUBK3_PORT = 4242         # UDP port (from udp_rx.c MY_PORT)

# ── Driving parameters ───────────────────────────────────────────────────
FORWARD_SPEED = 0.0           # constant forward speed (linear.x)

def FSD(camera, processor, detector, controller, lidar, show_display, tf, udp_sock, target_addr, foxglove_streamer=None):
    print("[ScanLine] Auto mode. Press Ctrl+C to quit.")
    speed = config.HEADLESS_SPEED if not show_display else 0.0
    obstacle_was_detected = False
    try:
        while True:
            #test cu lidar inainte de procesare de imagini pt franare mai resonsive
            # 5.0 LIDAR emergency braking check
            lidar_dist_cm = lidar.get_front_distance() / 10.0  # mm → cm
            obstacle_detected_now = lidar_dist_cm < config.LIDAR_BRAKE_THRESHOLD_CM

            if obstacle_detected_now:
                actual_speed = 0.0     # full stop (100% brake)
                steering = 0.0         # straighten wheels while braking
                status_str = f"BRAKE! dist={lidar_dist_cm:.1f}cm"
            else:
                actual_speed = speed
                status_str = f"speed={actual_speed:+.2f}"

            # 1. Capture
            frame = camera.read_frame()

            # 2. Process (grayscale → threshold)
            binary = processor.process(frame)

            # 3. Detect track via scan lines
            result = detector.detect(binary)

            # 4. Compute steering
            steering = controller.compute(result.weighted_center)

            if(steering > 0.4):
                actual_speed = 0.7
            elif(steering < -0.4):
                actual_speed = 0.7
            else:
                actual_speed = 0.9

            # 5. LIDAR emergency braking check
            lidar_dist_cm = lidar.get_front_distance() / 10.0  # mm → cm
            obstacle_detected_now = lidar_dist_cm < config.LIDAR_BRAKE_THRESHOLD_CM

            if obstacle_detected_now:
                actual_speed = 0.0     # full stop (100% brake)
                steering = 0.0         # straighten wheels while braking
                status_str = f"BRAKE! dist={lidar_dist_cm:.1f}cm"
            else:
                actual_speed = speed
                status_str = f"speed={actual_speed:+.2f}"

            # 5b. Log state transitions (only on change, not every frame)
            if obstacle_detected_now and not obstacle_was_detected:
                print(f"\n[LIDAR] OBSTACLE DETECTED at {lidar_dist_cm:.1f}cm — BRAKING!")
            elif not obstacle_detected_now and obstacle_was_detected:
                print(f"\n[LIDAR] Obstacle cleared — resuming normal driving.")

            obstacle_was_detected = obstacle_detected_now

            # 5. Send cmd_vel
            send_cmd_vel(tf, udp_sock, target_addr, steering, actual_speed)

            # 6. Log
            print(
                f"fps={camera.get_frame_rate():.2f}  "
                f"center={result.weighted_center!s:>8s}  "
                f"steering={steering:+.4f}  "
                f"[{status_str}]          ",
                end="\r",
            )

            # 7. Visualize (optional)
            debug_frame = draw_debug(frame, result, steering)
            #foxglove_streamer.update_debug_frame(debug_frame)
            #foxglove_streamer.update_binary_frame(binary)

            if show_display:
                cv2.imshow("ScanLine Debug", debug_frame)
                cv2.imshow("Binary", binary)

            # 8. Handle keyboard input (single waitKey for all key events)
            key = cv2.waitKey(1) & 0xFF
            if key == ord("q"):
                actual_speed = 0.0
                send_cmd_vel(tf, udp_sock, target_addr, steering, actual_speed)
                break

    except KeyboardInterrupt:
        print("\n[ScanLine] Ctrl+C detected. Stopping car.")
        actual_speed = 0.0
        send_cmd_vel(tf, udp_sock, target_addr, steering, actual_speed)
    

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


def send_cmd_vel(tf, udp_sock, target_addr, steering: float, forward_speed: float):
    """Send a cmd_vel message over UDP/TinyFrame."""
    steer_value = float(steering) if steering is not None else 0.0
    payload = encode_twist(linear_x=forward_speed, angular_z=steer_value)
    frame_bytes = tf.build_frame(SYNAPSE_CMD_VEL_TOPIC, payload)
    try:
        udp_sock.sendto(frame_bytes, target_addr)
    except OSError as e:
        print(f"\r[ScanLine] UDP send error: {e}", end="")


def run_manual_mode(tf, udp_sock, target_addr, camera, processor, detector):
    """Manual keyboard control via WASD keys with live camera feed for debugging."""
    steering = 0.0
    forward_speed = 0.0

    print("[Manual] WASD to drive, Q to quit. Focus the camera window!")

    while True:
        # Capture + process the camera (for debugging view only)
        frame = camera.read_frame()
        binary = processor.process(frame)
        result = detector.detect(binary)

        # Overlay HUD on the camera frame
        debug_frame = draw_debug(frame, result, steering)
        cv2.putText(debug_frame, f"MANUAL  Speed:{forward_speed:+.2f}  Steer:{steering:+.2f}",
                    (5, 20), cv2.FONT_HERSHEY_SIMPLEX, 0.45, (0, 255, 255), 1)
        cv2.putText(debug_frame, "WASD=drive  Q=quit",
                    (5, 40), cv2.FONT_HERSHEY_SIMPLEX, 0.4, (200, 200, 200), 1)
        cv2.imshow("Manual Control", debug_frame)
        cv2.imshow("Binary", binary)
        
        #foxglove_streamer.update_debug_frame(debug_frame)
        #foxglove_streamer.update_binary_frame(binary)

        key = cv2.waitKey(1) & 0xFF  # process GUI events quickly
        if key == ord("q"):
            break
        elif key == ord("w"):
            forward_speed += 0.1
        elif key == ord("s"):
            forward_speed -= 0.1
        elif key == ord("a"):
            steering += 0.1
        elif key == ord("d"):
            steering -= 0.1

        # Clamp values to reasonable range
        forward_speed = max(-1.0, min(1.0, forward_speed))
        steering = max(-1.0, min(1.0, steering))

        send_cmd_vel(tf, udp_sock, target_addr, steering, forward_speed)
        print(f"  speed={forward_speed:+.2f}  steer={steering:+.2f}", end="\r")

    # Stop the car when exiting manual mode
    send_cmd_vel(tf, udp_sock, target_addr, 0.0, 0.0)
    cv2.destroyAllWindows()


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
    parser.add_argument(
        "--lidar-port",
        default=config.LIDAR_PORT,
        help="LIDAR serial port (default: %(default)s)",
    )
    args = parser.parse_args()

    # ── Initialise components ─────────────────────────────────────────────
    camera = build_camera(args.camera)
    processor = ImageProcessor(threshold=args.threshold)
    detector = ScanLineDetector()
    controller = SteeringController()

    display_available = os.environ.get("DISPLAY") or os.environ.get("WAYLAND_DISPLAY")
    show_display = not args.no_display

    if show_display and not display_available:
        print("[ScanLine] No display detected. Switching to headless mode.")
        show_display = False

    # ── Initialise LIDAR ───────────────────────────────────────────────────
    lidar = LidarSensor(
        port=args.lidar_port,
        baudrate=config.LIDAR_BAUDRATE,
        front_angle_range=config.LIDAR_FRONT_ANGLE_RANGE,
    )
    lidar.start()

    #foxglove_streamer = FoxgloveStreamer(port=8765)
    #foxglove_streamer.start()

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

    # ── Choose mode ───────────────────────────────────────────────────────
    print("\nChoose mode:")
    print("  0 - Auto (camera + scan-line steering)")
    print("  1 - Manual (WASD keyboard control)")
    print("  2 - FULL FSD")
    mode = input("Enter mode [0/1/2]: ").strip()

    if mode == "1":
        # Manual mode — no camera needed for driving
        try:
            run_manual_mode(tf, udp_sock, target_addr, camera, processor, detector)
        except KeyboardInterrupt:
            print("\n[ScanLine] Interrupted.")
        finally:
            send_cmd_vel(tf, udp_sock, target_addr, 0.0, 0.0)
            udp_sock.close()
            camera.close()
            #foxglove_streamer.stop()
            cv2.destroyAllWindows()
        print("[ScanLine] Done.")
        return

    if mode == "2":
        FSD(camera, processor, detector, controller, lidar, show_display, tf, udp_sock, target_addr)
        return
    # ── Auto mode ─────────────────────────────────────────────────────────
    print("[ScanLine] Auto mode. Press Ctrl+C to quit.")
    speed = config.HEADLESS_SPEED if not show_display else 0.0
    obstacle_was_detected = False
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

            # 5. LIDAR emergency braking check
            lidar_dist_cm = lidar.get_front_distance() / 10.0  # mm → cm
            obstacle_detected_now = lidar_dist_cm < config.LIDAR_BRAKE_THRESHOLD_CM

            if obstacle_detected_now:
                actual_speed = 0.0     # full stop (100% brake)
                steering = 0.0         # straighten wheels while braking
                status_str = f"BRAKE! dist={lidar_dist_cm:.1f}cm"
            else:
                actual_speed = speed
                status_str = f"speed={actual_speed:+.2f}"

            # 5b. Log state transitions (only on change, not every frame)
            if obstacle_detected_now and not obstacle_was_detected:
                print(f"\n[LIDAR] OBSTACLE DETECTED at {lidar_dist_cm:.1f}cm — BRAKING!")
            elif not obstacle_detected_now and obstacle_was_detected:
                print(f"\n[LIDAR] Obstacle cleared — resuming normal driving.")

            obstacle_was_detected = obstacle_detected_now

            # 5. Send cmd_vel
            send_cmd_vel(tf, udp_sock, target_addr, steering, actual_speed)

            # 6. Log
            print(
                f"fps={camera.get_frame_rate():.2f}  "
                f"center={result.weighted_center!s:>8s}  "
                f"steering={steering:+.4f}  "
                f"[{status_str}]          ",
                end="\r",
            )

            # 7. Visualize (optional)
            debug_frame = draw_debug(frame, result, steering)
            #foxglove_streamer.update_debug_frame(debug_frame)
            #foxglove_streamer.update_binary_frame(binary)

            if show_display:
                cv2.imshow("ScanLine Debug", debug_frame)
                cv2.imshow("Binary", binary)

            # 8. Handle keyboard input (single waitKey for all key events)
            key = cv2.waitKey(1) & 0xFF
            if key == ord("q"):
                break
            elif key == ord("w"):
                speed += 0.1
            elif key == ord("s"):
                speed -= 0.1
            FORWARD_SPEED = speed

    except KeyboardInterrupt:
        print("\n[ScanLine] Interrupted by user.")
        # Ensure STOP is sent immediately before anything else closes.
        send_cmd_vel(tf, udp_sock, target_addr, 0.0, 0.0)
    finally:
        # Redundant stop to ensure we never leave motors spinning
        send_cmd_vel(tf, udp_sock, target_addr, 0.0, 0.0)
        udp_sock.close()
        camera.close()
        lidar.stop()
        #foxglove_streamer.stop()
        if show_display:
            cv2.destroyAllWindows()

    print("[ScanLine] Done.")


if __name__ == "__main__":
    main()
