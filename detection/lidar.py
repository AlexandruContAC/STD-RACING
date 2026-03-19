"""
LDRobot STL-27L DTOF LiDAR sensor driver.

Reads 360-degree scan data from the LDRobot STL-27L over UART,
parses the proprietary packet format, and exposes the minimum
distance detected within a configurable forward cone.

UART config: 921600 baud, 8 data bits, 1 stop bit, no parity, no flow control.

Packet format (47 bytes each):
  [0x54] [0x2C] [speed 2B] [start_angle 2B] [12x (dist 2B + intensity 1B)]
  [end_angle 2B] [timestamp 2B] [crc8 1B]

Compatible with the LDRobot protocol family (LD19, STL-27L, etc.).
"""

import serial
import threading
import time


class LidarSensor:
    """Threaded STL-27L LiDAR reader with forward-cone obstacle detection."""

    def __init__(self, port='/dev/ttymxc2', baudrate=921600,
                 front_angle_range=(-15.0, 15.0)):
        self.port = port
        self.baudrate = baudrate
        self.serial = None
        self.running = False
        self.thread = None

        self.front_angle_min = front_angle_range[0]
        self.front_angle_max = front_angle_range[1]

        # Latest minimum distance in the front cone (in mm), default = far away
        self.front_distance = 10000.0
        self.lock = threading.Lock()

    # ── public API ──────────────────────────────────────────────────────────

    def start(self):
        """Open the serial port and begin reading in a background thread."""
        try:
            self.serial = serial.Serial(self.port, self.baudrate, timeout=1.0)
            self.running = True
            self.thread = threading.Thread(target=self._read_loop, daemon=True)
            self.thread.start()
            print(f"[LIDAR] Started STL-27L on {self.port}")
            return True
        except Exception as e:
            print(f"[LIDAR] Failed to start on {self.port}: {e}")
            return False

    def stop(self):
        """Stop the reader thread and close the serial port."""
        self.running = False
        if self.thread:
            self.thread.join(timeout=2.0)
        if self.serial and self.serial.is_open:
            self.serial.close()
        print("[LIDAR] Stopped.")

    def get_front_distance(self):
        """Return the minimum distance (mm) in the forward cone."""
        with self.lock:
            return self.front_distance

    # ── internals ───────────────────────────────────────────────────────────

    def _normalize_angle(self, angle):
        """Normalize angle to [-180, 180)."""
        while angle >= 180.0:
            angle -= 360.0
        while angle < -180.0:
            angle += 360.0
        return angle

    def _is_in_front_cone(self, angle):
        """Check whether *angle* (degrees, 0-360 from sensor) falls in the front cone."""
        norm = self._normalize_angle(angle)
        return self.front_angle_min <= norm <= self.front_angle_max

    @staticmethod
    def _calc_crc8(data):
        """CRC-8 used by the STL-27L (polynomial 0x4D, LDRobot family)."""
        crc_table = [
            0x00, 0x4d, 0x9a, 0xd7, 0x79, 0x34, 0xe3, 0xae,
            0xf2, 0xbf, 0x68, 0x25, 0x8b, 0xc6, 0x11, 0x5c,
            0xa9, 0xe4, 0x33, 0x7e, 0xd0, 0x9d, 0x4a, 0x07,
            0x5b, 0x16, 0xc1, 0x8c, 0x22, 0x6f, 0xb8, 0xf5,
            0x1f, 0x52, 0x85, 0xc8, 0x66, 0x2b, 0xfc, 0xb1,
            0xed, 0xa0, 0x77, 0x3a, 0x94, 0xd9, 0x0e, 0x43,
            0xb6, 0xfb, 0x2c, 0x61, 0xcf, 0x82, 0x55, 0x18,
            0x44, 0x09, 0xde, 0x93, 0x3d, 0x70, 0xa7, 0xea,
            0x3e, 0x73, 0xa4, 0xe9, 0x47, 0x0a, 0xdd, 0x90,
            0xcc, 0x81, 0x56, 0x1b, 0xb5, 0xf8, 0x2f, 0x62,
            0x97, 0xda, 0x0d, 0x40, 0xee, 0xa3, 0x74, 0x39,
            0x65, 0x28, 0xff, 0xb2, 0x1c, 0x51, 0x86, 0xcb,
            0x21, 0x6c, 0xbb, 0xf6, 0x58, 0x15, 0xc2, 0x8f,
            0xd3, 0x9e, 0x49, 0x04, 0xaa, 0xe7, 0x30, 0x7d,
            0x88, 0xc5, 0x12, 0x5f, 0xf1, 0xbc, 0x6b, 0x26,
            0x7a, 0x37, 0xe0, 0xad, 0x03, 0x4e, 0x99, 0xd4,
            0x7c, 0x31, 0xe6, 0xab, 0x05, 0x48, 0x9f, 0xd2,
            0x8e, 0xc3, 0x14, 0x59, 0xf7, 0xba, 0x6d, 0x20,
            0xd5, 0x98, 0x4f, 0x02, 0xac, 0xe1, 0x36, 0x7b,
            0x27, 0x6a, 0xbd, 0xf0, 0x5e, 0x13, 0xc4, 0x89,
            0x63, 0x2e, 0xf9, 0xb4, 0x1a, 0x57, 0x80, 0xcd,
            0x91, 0xdc, 0x0b, 0x46, 0xe8, 0xa5, 0x72, 0x3f,
            0xca, 0x87, 0x50, 0x1d, 0xb3, 0xfe, 0x29, 0x64,
            0x38, 0x75, 0xa2, 0xef, 0x41, 0x0c, 0xdb, 0x96,
            0x42, 0x0f, 0xd8, 0x95, 0x3b, 0x76, 0xa1, 0xec,
            0xb0, 0xfd, 0x2a, 0x67, 0xc9, 0x84, 0x53, 0x1e,
            0xeb, 0xa6, 0x71, 0x3c, 0x92, 0xdf, 0x08, 0x45,
            0x19, 0x54, 0x83, 0xce, 0x60, 0x2d, 0xfa, 0xb7,
            0x5d, 0x10, 0xc7, 0x8a, 0x24, 0x69, 0xbe, 0xf3,
            0xaf, 0xe2, 0x35, 0x78, 0xd6, 0x9b, 0x4c, 0x01,
            0xf4, 0xb9, 0x6e, 0x23, 0x8d, 0xc0, 0x17, 0x5a,
            0x06, 0x4b, 0x9c, 0xd1, 0x7f, 0x32, 0xe5, 0xa8,
        ]
        crc = 0
        for b in data:
            crc = crc_table[crc ^ b]
        return crc

    def _read_loop(self):
        """Background thread: continuously parse STL-27L packets."""
        HEADER = 0x54
        VERLEN = 0x2C
        PACKET_SIZE = 47

        buf = bytearray()

        # Track minimum distance over one full 360-degree revolution
        scan_min_dist = 10000.0
        last_angle = 0.0

        while self.running:
            if not self.serial or not self.serial.is_open:
                break

            try:
                to_read = self.serial.in_waiting or 1
                data = self.serial.read(min(to_read, 1024))
                if not data:
                    continue

                buf.extend(data)

                while len(buf) >= PACKET_SIZE:
                    # Sync to packet header
                    if buf[0] != HEADER or buf[1] != VERLEN:
                        buf.pop(0)
                        continue

                    packet = buf[:PACKET_SIZE]

                    # Validate CRC-8 (first 46 bytes, CRC at byte 46)
                    if self._calc_crc8(packet[:46]) != packet[46]:
                        buf.pop(0)
                        continue

                    # Parse angles (in 0.01 degree units)
                    start_angle = (packet[5] << 8 | packet[4]) / 100.0
                    end_angle = (packet[43] << 8 | packet[42]) / 100.0

                    # Compute per-point angle step
                    diff = end_angle - start_angle
                    if diff < 0:
                        diff += 360.0
                    step = diff / 11.0  # 12 points → 11 intervals

                    # Extract 12 measurement points
                    for i in range(12):
                        base = 6 + i * 3
                        distance = packet[base + 1] << 8 | packet[base]
                        intensity = packet[base + 2]

                        angle = start_angle + step * i
                        if angle >= 360.0:
                            angle -= 360.0

                        # Ignore zero / noisy readings
                        if distance > 0 and intensity > 0:
                            if self._is_in_front_cone(angle):
                                if distance < scan_min_dist:
                                    scan_min_dist = distance

                    # Detect full-revolution wrap (angle decreased → new sweep)
                    if start_angle < last_angle:
                        with self.lock:
                            self.front_distance = scan_min_dist
                        scan_min_dist = 10000.0  # reset for next revolution

                    last_angle = start_angle

                    # Consume parsed packet
                    buf = buf[PACKET_SIZE:]

            except Exception as e:
                print(f"[LIDAR] Read error: {e}")
                time.sleep(0.1)