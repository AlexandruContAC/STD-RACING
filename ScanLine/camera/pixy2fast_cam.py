"""
Pixy2 FAST camera backend – reads high throughput BA81 XDATA stream
using a pure Python libusb1 implementation.
This backend enables ~60fps without relying on the compiled get_raw_frame binary.
"""

from __future__ import annotations

import struct
import time
import queue
import threading
from typing import List, Optional, Tuple

import numpy as np
import cv2
import usb.core
import usb.util
from usb.backend import libusb1

from camera.base import CameraBase

# Chirp constants
CRP_START_CODE = 0xAAAA5555
CRP_CALL = 0x80
CRP_RESPONSE = 0x40
CRP_INTRINSIC = 0x20
CRP_XDATA = 0x18
CRP_CALL_ENUMERATE = CRP_CALL | CRP_INTRINSIC | 0x00
CRP_CALL_INIT = CRP_CALL | CRP_INTRINSIC | 0x01

CRP_ARRAY = 0x80
CRP_HINT = 0x40
CRP_FLT = 0x10
CRP_NO_COPY = 0x10 | 0x20
CRP_NULLTERM_ARRAY = 0x20 | CRP_ARRAY
CRP_INT8 = 0x01
CRP_INT16 = 0x02
CRP_INT32 = 0x04
CRP_STRING = CRP_NULLTERM_ARRAY | CRP_INT8
CRP_TYPE_HINT = 0x64

PIX2_VID = 0xB1AC
PIX2_PID = 0xF000
HEADER_LEN = 12
START_CODE_LE = struct.pack("<I", CRP_START_CODE)
RAW_FOURCC_BA81 = struct.unpack("<I", b"BA81")[0]

def _align(offset: int, n: int) -> int:
    return offset if (offset & (n - 1)) == 0 else (offset & ~(n - 1)) + n

class ChirpPacket:
    def __init__(self, pkt_type: int, proc: int, payload: bytes):
        self.pkt_type = pkt_type
        self.proc = proc
        self.payload = payload

class ChirpUsbClient:
    def __init__(self, vid: int, pid: int, timeout_ms: int = 2000):
        self.vid = vid
        self.pid = pid
        self.timeout_ms = timeout_ms
        self.dev: Optional[usb.core.Device] = None
        self.ep_in: Optional[int] = None
        self.ep_out: Optional[int] = None
        self.rx_buf = bytearray()
        self.remote_hinterested = 0
        self.proc: dict[str, int] = {}

    def open(self) -> None:
        backend = libusb1.get_backend()
        self.dev = usb.core.find(idVendor=self.vid, idProduct=self.pid, backend=backend)
        if self.dev is None:
            raise RuntimeError(f"Pixy2 not found (VID=0x{self.vid:04x}, PID=0x{self.pid:04x})")

        # Reset USB device to clear any stale state from a previous crashed run
        try:
            self.dev.reset()
            time.sleep(0.1)
        except usb.core.USBError:
            pass

        try:
            self.dev.set_configuration()
        except usb.core.USBError:
            pass

        cfg = self.dev.get_active_configuration()
        intf = cfg[(1, 0)]
        intf_num = intf.bInterfaceNumber

        try:
            if self.dev.is_kernel_driver_active(intf_num):
                self.dev.detach_kernel_driver(intf_num)
        except (NotImplementedError, usb.core.USBError):
            pass

        usb.util.claim_interface(self.dev, intf_num)

        ep_in = usb.util.find_descriptor(
            intf,
            custom_match=lambda e: usb.util.endpoint_direction(e.bEndpointAddress) == usb.util.ENDPOINT_IN,
        )
        ep_out = usb.util.find_descriptor(
            intf,
            custom_match=lambda e: usb.util.endpoint_direction(e.bEndpointAddress) == usb.util.ENDPOINT_OUT,
        )
        if ep_in is None or ep_out is None:
            raise RuntimeError("Could not find Pixy2 bulk IN/OUT endpoints on interface 1")
        self.ep_in = ep_in.bEndpointAddress
        self.ep_out = ep_out.bEndpointAddress

        # Drain any stale data left in the USB IN pipe from a previous session
        self._drain_stale_data()

    def _drain_stale_data(self) -> None:
        """Read and discard any residual data in the USB IN pipe."""
        if self.dev is None or self.ep_in is None:
            return
        for _ in range(10):
            try:
                self.dev.read(self.ep_in, 0x10000, timeout=50)
            except usb.core.USBTimeoutError:
                break
            except usb.core.USBError:
                break

    def close(self) -> None:
        if self.dev is not None:
            try:
                usb.util.release_interface(self.dev, 1)
            except usb.core.USBError:
                pass
            usb.util.dispose_resources(self.dev)
        self.dev = None

    def _write(self, data: bytes) -> None:
        if self.dev is None or self.ep_out is None:
            raise RuntimeError("Device not open")
        written = self.dev.write(self.ep_out, data, timeout=self.timeout_ms)
        if written != len(data):
            raise RuntimeError(f"Short USB write: {written}/{len(data)}")

    def _read_once(self, timeout_ms: int) -> None:
        if self.dev is None or self.ep_in is None:
            raise RuntimeError("Device not open")
        data = self.dev.read(self.ep_in, 0x10000, timeout=timeout_ms)
        if data:
            self.rx_buf.extend(bytes(data))

    def _try_parse_packet(self) -> Optional[ChirpPacket]:
        while True:
            if len(self.rx_buf) < 4:
                return None

            idx = self.rx_buf.find(START_CODE_LE)
            if idx < 0:
                if len(self.rx_buf) > 3:
                    del self.rx_buf[:-3]
                return None
            if idx > 0:
                del self.rx_buf[:idx]
            if len(self.rx_buf) < HEADER_LEN:
                return None

            pkt_type = self.rx_buf[4]
            proc = struct.unpack_from("<h", self.rx_buf, 6)[0]
            payload_len = struct.unpack_from("<I", self.rx_buf, 8)[0]
            total_len = HEADER_LEN + payload_len
            if len(self.rx_buf) < total_len:
                return None

            payload = bytes(self.rx_buf[HEADER_LEN:total_len])
            del self.rx_buf[:total_len]
            return ChirpPacket(pkt_type=pkt_type, proc=proc, payload=payload)

    def recv_packet(self, timeout_ms: int) -> ChirpPacket:
        deadline = time.perf_counter() + (timeout_ms / 1000.0)
        while True:
            pkt = self._try_parse_packet()
            if pkt is not None:
                return pkt
            remaining_ms = int(max(1.0, (deadline - time.perf_counter()) * 1000.0))
            if remaining_ms <= 0:
                raise TimeoutError("Timed out waiting for Chirp packet")
            self._read_once(remaining_ms)

    @staticmethod
    def _serialize_args(args: List[Tuple[int, object]]) -> bytes:
        out = bytearray()
        i = 0
        for t, value in args:
            out.append(t & 0xFF)
            i += 1
            base_t = t & ~CRP_HINT

            if base_t == CRP_INT8:
                out.extend(struct.pack("<b", int(value)))
                i += 1
            elif base_t == CRP_INT16:
                ni = _align(i, 2)
                out.extend(b"\x00" * (ni - i))
                i = ni
                out[i - 1] = t & 0xFF
                out.extend(struct.pack("<h", int(value)))
                i += 2
            elif base_t == CRP_INT32 or t == CRP_TYPE_HINT:
                ni = _align(i, 4)
                out.extend(b"\x00" * (ni - i))
                i = ni
                out[i - 1] = t & 0xFF
                out.extend(struct.pack("<i", int(value)))
                i += 4
            elif base_t == CRP_STRING:
                data = str(value).encode("utf-8") + b"\x00"
                out.extend(data)
                i += len(data)
            elif t & CRP_ARRAY:
                elem_size = t & 0x0F
                if base_t == CRP_STRING:
                    raise ValueError("String array serialization not supported")
                raw = bytes(value)
                if len(raw) % elem_size != 0:
                    raise ValueError("Array byte length is not aligned to element size")
                count = len(raw) // elem_size
                ni = _align(i, 4)
                out.extend(b"\x00" * (ni - i))
                i = ni
                out[i - 1] = t & 0xFF
                out.extend(struct.pack("<I", count))
                i += 4
                ni = _align(i, elem_size)
                out.extend(b"\x00" * (ni - i))
                i = ni
                out.extend(raw)
                i += len(raw)
            else:
                raise ValueError(f"Unsupported Chirp arg type 0x{t:02x}")
        return bytes(out)

    @staticmethod
    def _parse_args(buf: bytes) -> List[object]:
        args: List[object] = []
        i = 0
        n = len(buf)

        while i < n:
            t = buf[i]
            i += 1
            base_t = t & ~CRP_HINT
            size = t & 0x0F

            if not (t & CRP_ARRAY):
                if base_t == CRP_STRING:
                    end = buf.find(b"\x00", i)
                    if end < 0:
                        raise ValueError("Malformed Chirp string")
                    args.append(buf[i:end].decode("utf-8", errors="replace"))
                    i = end + 1
                else:
                    if size not in (1, 2, 4):
                        raise ValueError(f"Unsupported scalar size {size}")
                    i = _align(i, size)
                    if i + size > n:
                        raise ValueError("Malformed scalar argument")
                    raw = buf[i:i + size]
                    signed = base_t in (CRP_INT8, CRP_INT16, CRP_INT32) and t != CRP_TYPE_HINT
                    args.append(int.from_bytes(raw, byteorder="little", signed=signed))
                    i += size
            else:
                if base_t == CRP_STRING:
                    end = buf.find(b"\x00", i)
                    if end < 0:
                        raise ValueError("Malformed Chirp string array")
                    args.append(buf[i:end].decode("utf-8", errors="replace"))
                    i = end + 1
                else:
                    i = _align(i, 4)
                    if i + 4 > n:
                        raise ValueError("Malformed array length")
                    length = int.from_bytes(buf[i:i + 4], byteorder="little", signed=False)
                    i += 4
                    elem_size = t & 0x0F
                    i = _align(i, elem_size)
                    total = length * elem_size
                    if i + total > n:
                        raise ValueError("Malformed array payload")
                    args.append(int(length))
                    args.append(bytes(buf[i:i + total]))
                    i += total

        return args

    @staticmethod
    def _prepend_response_int_type(payload: bytes) -> bytes:
        return bytes((CRP_INT32, 0, 0, CRP_INT32)) + payload

    def send_packet(self, pkt_type: int, proc: int, payload: bytes) -> None:
        header = struct.pack("<IBBhI", CRP_START_CODE, pkt_type, 0, int(proc), len(payload))
        self._write(header + payload)

    def call_sync(
        self,
        proc: int,
        args: List[Tuple[int, object]],
        call_type: int = CRP_CALL,
        overall_timeout_ms: int = 8000,
    ) -> List[object]:
        payload = self._serialize_args(args)
        self.send_packet(call_type, proc, payload)

        deadline = time.perf_counter() + (overall_timeout_ms / 1000.0)
        while True:
            remaining_ms = int((deadline - time.perf_counter()) * 1000.0)
            if remaining_ms <= 0:
                raise TimeoutError(f"Timed out waiting for Chirp response (proc={proc})")
            pkt = self.recv_packet(timeout_ms=min(3000, max(1, remaining_ms)))
            if pkt.pkt_type & CRP_RESPONSE:
                parsed = self._parse_args(self._prepend_response_int_type(pkt.payload))
                return parsed

    def remote_init(self, block_size: int = 64, hinterested: int = 1, retries: int = 2) -> None:
        last_exc = None
        for attempt in range(retries + 1):
            try:
                parsed = self.call_sync(
                    proc=0,
                    call_type=CRP_CALL_INIT,
                    args=[(CRP_INT16, block_size), (CRP_INT8, hinterested)],
                )
                if not parsed:
                    raise RuntimeError("Invalid init response")
                response_int = int(parsed[0])
                if response_int < 0:
                    raise RuntimeError(f"remoteInit failed with {response_int}")
                self.remote_hinterested = int(parsed[1]) if len(parsed) > 1 else 0
                return  # success
            except (usb.core.USBError, TimeoutError) as exc:
                last_exc = exc
                if attempt < retries:
                    print(f"[Pixy2Fast] remote_init attempt {attempt+1} failed: {exc}. Retrying...")
                    # Reset device and drain before retry
                    try:
                        self.dev.reset()
                        time.sleep(0.2)
                    except usb.core.USBError:
                        pass
                    self._drain_stale_data()
                    self.rx_buf.clear()
        raise RuntimeError(f"remote_init failed after {retries+1} attempts: {last_exc}")

    def get_proc(self, name: str) -> int:
        parsed = self.call_sync(
            proc=0,
            call_type=CRP_CALL_ENUMERATE,
            args=[(CRP_STRING, name), (CRP_INT16, -1)],
        )
        if not parsed:
            raise RuntimeError(f"Invalid getProc response for {name}")
        proc_id = int(parsed[0])
        if proc_id < 0:
            raise RuntimeError(f"Procedure not found: {name}")
        self.proc[name] = proc_id
        return proc_id

    def require_proc(self, name: str) -> int:
        if name in self.proc:
            return self.proc[name]
        return self.get_proc(name)


class Pixy2FastCamera(CameraBase):
    def __init__(self) -> None:
        self.client = ChirpUsbClient(PIX2_VID, PIX2_PID)
        self._reader_thread: Optional[threading.Thread] = None
        self._frame_queue: queue.Queue = queue.Queue(maxsize=1)
        self._stop_event = threading.Event()
        self._fps = 0.0
        self._width = 316
        self._height = 208
        self._initialised = False

    def _reader_loop(self) -> None:
        try:
            p_run_prog_name = self.client.require_proc("runProgName")
            ret = self.client.call_sync(p_run_prog_name, [(CRP_STRING, "video")])[0]
            if int(ret) < 0:
                print(f"[Pixy2Fast] runProgName('video') failed with {ret}")
                self._stop_event.set()
                return

            last_time = time.perf_counter()

            while not self._stop_event.is_set():
                try:
                    pkt = self.client.recv_packet(timeout_ms=1500)
                except TimeoutError:
                    continue

                if pkt.pkt_type != CRP_XDATA:
                    continue
                parsed = self.client._parse_args(pkt.payload)
                if len(parsed) < 6:
                    continue

                frame_fourcc = int(parsed[0]) & 0xFFFFFFFF
                if frame_fourcc != RAW_FOURCC_BA81:
                    continue

                frame_width = int(parsed[2])
                frame_height = int(parsed[3])
                frame_len = int(parsed[4])
                frame = parsed[5]

                if not isinstance(frame, (bytes, bytearray)) or len(frame) != frame_len:
                    continue

                # Measure actual FPS
                now = time.perf_counter()
                self._fps = 1.0 / max(now - last_time, 1e-9)
                last_time = now

                if self._frame_queue.full():
                    try:
                        self._frame_queue.get_nowait()
                    except queue.Empty:
                        pass
                self._frame_queue.put_nowait((frame, frame_width, frame_height))

        except Exception as e:
            print(f"[Pixy2Fast] Reader loop error: {e}")
            self._stop_event.set()

    def open(self) -> None:
        if self._initialised:
            return

        self.client.open()
        self.client.remote_init(block_size=64, hinterested=1)
        print("[Pixy2Fast] Chirp init OK.")

        # Try to set max FPS (61) if possible — not all firmware exports this proc
        try:
            p_cam_set_framerate = self.client.get_proc("cam_setFramerate")
            res = int(self.client.call_sync(p_cam_set_framerate, [(CRP_INT8, 61)])[0])
            if res < 0:
                print(f"[Pixy2Fast] cam_setFramerate(61) returned {res}, ignoring.")
            else:
                print("[Pixy2Fast] Camera min FPS set to 61.")
        except RuntimeError as exc:
            print(f"[Pixy2Fast] cam_setFramerate not available: {exc}")

        self._stop_event.clear()
        self._reader_thread = threading.Thread(target=self._reader_loop, daemon=True)
        self._reader_thread.start()
        self._initialised = True
        print("[Pixy2Fast] Camera opened successfully.")

    def read_frame(self) -> np.ndarray:
        if not self._initialised:
            raise RuntimeError("Pixy2Fast camera is not open. Call open() first.")
        if self._stop_event.is_set():
            raise RuntimeError("Camera stream ended unexpectedly.")

        try:
            frame, w, h = self._frame_queue.get(timeout=5.0)
            self._width = w
            self._height = h
        except queue.Empty:
            raise TimeoutError("No frame received within 5 seconds.")

        # BA81 is raw Bayer BGGR.  Demosaic to proper grayscale so the
        # image is clean instead of showing the checkerboard Bayer pattern.
        bayer = np.frombuffer(frame, dtype=np.uint8).reshape((h, w))
        gray = cv2.cvtColor(bayer, cv2.COLOR_BayerBG2GRAY)
        return gray

    def get_frame_rate(self) -> float:
        return self._fps

    def close(self) -> None:
        self._stop_event.set()
        if self._reader_thread:
            self._reader_thread.join(timeout=2)
            self._reader_thread = None
        
        while not self._frame_queue.empty():
            try:
                self._frame_queue.get_nowait()
            except queue.Empty:
                break
                
        try:
            self.client.close()
        except Exception:
            pass
        self._initialised = False
        print("[Pixy2Fast] Camera closed.")
