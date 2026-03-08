#!/usr/bin/env python3
"""
Pixy2 minimal diagnostic — mirrors libpixyusb2's exact USB init sequence.
Make sure PixyMon is CLOSED before running this.

Usage:  python3 pixy2_diag.py
"""

import struct
import sys

try:
    import usb.core
    import usb.util
except ImportError:
    print("ERROR: pyusb not installed. Run: pip install pyusb")
    sys.exit(1)

PIXY2_VID = 0xB1AC
PIXY2_PID = 0xF000
SYNC = 0xC1AE
TYPE_GET_VERSION = 0x0E


def try_approach(dev, label, setup_fn):
    """Try a USB init approach and send getVersion."""
    print(f"\n── Approach: {label} ──")
    try:
        setup_fn(dev)
    except Exception as e:
        print(f"  ✗ Setup failed: {e}")
        return False

    packet = struct.pack("<HBB", SYNC, TYPE_GET_VERSION, 0)
    try:
        written = dev.write(0x02, packet, timeout=2000)
        print(f"  ✓ Wrote {written} bytes")
    except usb.core.USBError as e:
        print(f"  ✗ Write failed: {e}")
        return False

    try:
        data = bytes(dev.read(0x82, 256, timeout=3000))
        print(f"  ✓ Read {len(data)} bytes: {data.hex()}")
        if len(data) >= 4:
            sync, ptype, length = struct.unpack("<HBB", data[:4])
            print(f"    sync=0x{sync:04X} type=0x{ptype:02X} length={length}")
        return True
    except usb.core.USBTimeoutError:
        print(f"  ✗ Read timed out")
        return False
    except usb.core.USBError as e:
        print(f"  ✗ Read error: {e}")
        return False


def main():
    print("=" * 60)
    print("  Pixy2 Minimal Diagnostic")
    print("  (Make sure PixyMon is CLOSED first!)")
    print("=" * 60)

    dev = usb.core.find(idVendor=PIXY2_VID, idProduct=PIXY2_PID)
    if dev is None:
        print("\n✗ Pixy2 NOT FOUND.")
        sys.exit(1)
    print(f"\n✓ Found: {dev.manufacturer} {dev.product} (Serial: {dev.serial_number})")

    # ── Approach 1: Exact libpixyusb2 sequence (simplest) ────────────
    def approach_1(d):
        d.set_configuration(1)
        usb.util.claim_interface(d, 1)
        print("  ✓ set_configuration(1) + claim_interface(1)")

    if try_approach(dev, "libpixyusb2 (set_config + claim)", approach_1):
        print("\n✓✓✓ Approach 1 WORKED!")
        usb.util.dispose_resources(dev)
        return

    usb.util.dispose_resources(dev)

    # ── Approach 2: auto-detach kernel driver ────────────────────────
    dev = usb.core.find(idVendor=PIXY2_VID, idProduct=PIXY2_PID)

    def approach_2(d):
        # Let libusb handle kernel driver automatically
        if hasattr(d, 'set_auto_detach_kernel_driver'):
            d.set_auto_detach_kernel_driver(True)
            print("  ✓ auto_detach enabled")
        d.set_configuration(1)
        usb.util.claim_interface(d, 1)
        print("  ✓ set_configuration(1) + claim_interface(1)")

    if try_approach(dev, "auto-detach + claim", approach_2):
        print("\n✓✓✓ Approach 2 WORKED!")
        usb.util.dispose_resources(dev)
        return

    usb.util.dispose_resources(dev)

    # ── Approach 3: reset + detach all + claim ───────────────────────
    dev = usb.core.find(idVendor=PIXY2_VID, idProduct=PIXY2_PID)

    def approach_3(d):
        # Detach kernel drivers first, then configure
        for i in range(3):
            try:
                if d.is_kernel_driver_active(i):
                    d.detach_kernel_driver(i)
                    print(f"  ✓ Detached kernel driver from interface {i}")
            except Exception:
                pass
        d.set_configuration(1)
        usb.util.claim_interface(d, 1)
        print("  ✓ detach_all + set_configuration(1) + claim_interface(1)")

    if try_approach(dev, "detach-first + config + claim", approach_3):
        print("\n✓✓✓ Approach 3 WORKED!")
        usb.util.dispose_resources(dev)
        return

    usb.util.dispose_resources(dev)

    # ── Approach 4: Interface 0 instead of 1 ─────────────────────────
    dev = usb.core.find(idVendor=PIXY2_VID, idProduct=PIXY2_PID)

    def approach_4(d):
        d.set_configuration(1)
        usb.util.claim_interface(d, 0)
        print("  ✓ set_configuration(1) + claim_interface(0)")

    # Use interrupt endpoint 0x81 instead of bulk
    print(f"\n── Approach: Interface 0 with interrupt endpoint 0x81 ──")
    try:
        approach_4(dev)
        packet = struct.pack("<HBB", SYNC, TYPE_GET_VERSION, 0)
        # Try writing to the interrupt endpoint on interface 0
        written = dev.write(0x02, packet, timeout=2000)
        print(f"  ✓ Wrote {written} bytes")
        data = bytes(dev.read(0x81, 256, timeout=3000))
        print(f"  ✓ Read {len(data)} bytes: {data.hex()}")
    except usb.core.USBError as e:
        print(f"  ✗ Failed: {e}")

    usb.util.dispose_resources(dev)

    print("\n" + "=" * 60)
    print("  All approaches failed.")
    print("  Please run: lsusb -v -d b1ac:f000 2>/dev/null | head -80")
    print("  and paste the output.")
    print("=" * 60)


if __name__ == "__main__":
    main()
