"""
Synthetic-image tests for the ScanLine pipeline.

Generates a fake NXP track image and verifies that the processing,
detection, and steering modules produce correct results.
"""

import sys
import os

# Ensure the project root is on the path
sys.path.insert(0, os.path.dirname(os.path.dirname(os.path.abspath(__file__))))

import numpy as np
import cv2

from processing.pipeline import ImageProcessor
from detection.scanline import ScanLineDetector
from steering.controller import SteeringController


# ── Helpers ──────────────────────────────────────────────────────────────

def make_track_image(
    width: int = 316,
    height: int = 208,
    left_x: int = 100,
    right_x: int = 216,
    border_thickness: int = 6,
) -> np.ndarray:
    """
    Create a synthetic BGR image with two black vertical borders
    simulating an NXP track on a white background.
    """
    img = np.ones((height, width, 3), dtype=np.uint8) * 220  # light grey

    # Left border (black)
    cv2.line(img, (left_x, 0), (left_x, height - 1), (0, 0, 0), border_thickness)
    # Right border (black)
    cv2.line(img, (right_x, 0), (right_x, height - 1), (0, 0, 0), border_thickness)

    return img


def make_off_center_track(
    width: int = 316,
    height: int = 208,
    left_x: int = 30,
    right_x: int = 146,
    border_thickness: int = 6,
) -> np.ndarray:
    """Track shifted to the left side of the image."""
    return make_track_image(width, height, left_x, right_x, border_thickness)


# ── Tests ────────────────────────────────────────────────────────────────

def test_grayscale_conversion():
    """ImageProcessor.to_grayscale should return a single-channel image."""
    proc = ImageProcessor()
    img = make_track_image()
    gray = proc.to_grayscale(img)

    assert gray.ndim == 2, f"Expected 2D array, got {gray.ndim}D"
    assert gray.shape == (208, 316), f"Shape mismatch: {gray.shape}"
    print("  ✓ test_grayscale_conversion")


def test_threshold_output():
    """apply_threshold should produce a binary image with only 0 and 255."""
    proc = ImageProcessor(threshold=60)
    img = make_track_image()
    gray = proc.to_grayscale(img)
    binary = proc.apply_threshold(gray)

    unique = set(np.unique(binary))
    assert unique <= {0, 255}, f"Expected {{0, 255}}, got {unique}"
    assert binary.shape == gray.shape
    print("  ✓ test_threshold_output")


def test_pipeline_end_to_end():
    """process() should produce a valid binary image."""
    proc = ImageProcessor(threshold=60)
    img = make_track_image()
    binary = proc.process(img)

    assert binary.ndim == 2
    assert set(np.unique(binary)) <= {0, 255}
    print("  ✓ test_pipeline_end_to_end")


def test_scanline_centred_track():
    """Centred track should yield a weighted centre near the image centre."""
    proc = ImageProcessor(threshold=60)
    detector = ScanLineDetector(
        rows=[100, 120, 140, 160, 180],
        weights=[0.05, 0.10, 0.15, 0.30, 0.40],
    )

    img = make_track_image(left_x=100, right_x=216)
    binary = proc.process(img)
    result = detector.detect(binary)

    expected_center = (100 + 216) / 2.0  # 158.0
    image_center = 316 / 2.0  # 158.0

    assert result.weighted_center is not None, "No track detected!"
    error = abs(result.weighted_center - expected_center)
    assert error < 15, f"Centre off by {error:.1f}px (expected ~{expected_center})"
    print(f"  ✓ test_scanline_centred_track  (centre={result.weighted_center:.1f})")


def test_scanline_off_centre_track():
    """Off-centre track should yield a centre away from the image centre."""
    proc = ImageProcessor(threshold=60)
    detector = ScanLineDetector(
        rows=[100, 120, 140, 160, 180],
        weights=[0.05, 0.10, 0.15, 0.30, 0.40],
    )

    img = make_off_center_track(left_x=30, right_x=146)
    binary = proc.process(img)
    result = detector.detect(binary)

    expected_center = (30 + 146) / 2.0  # 88.0
    assert result.weighted_center is not None, "No track detected!"
    error = abs(result.weighted_center - expected_center)
    assert error < 15, f"Centre off by {error:.1f}px"
    print(f"  ✓ test_scanline_off_centre_track  (centre={result.weighted_center:.1f})")


def test_steering_centred():
    """Centred track should produce near-zero steering."""
    proc = ImageProcessor(threshold=60)
    detector = ScanLineDetector(
        rows=[100, 120, 140, 160, 180],
        weights=[0.05, 0.10, 0.15, 0.30, 0.40],
    )
    controller = SteeringController(image_width=316, kp=0.01)

    img = make_track_image(left_x=100, right_x=216)
    binary = proc.process(img)
    result = detector.detect(binary)
    steering = controller.compute(result.weighted_center)

    assert abs(steering) < 0.1, f"Steering should be ~0 for centred track, got {steering}"
    print(f"  ✓ test_steering_centred  (steering={steering:+.4f})")


def test_steering_off_centre():
    """Off-centre track should produce a non-zero steering correction."""
    proc = ImageProcessor(threshold=60)
    detector = ScanLineDetector(
        rows=[100, 120, 140, 160, 180],
        weights=[0.05, 0.10, 0.15, 0.30, 0.40],
    )
    controller = SteeringController(image_width=316, kp=0.01)

    img = make_off_center_track(left_x=30, right_x=146)
    binary = proc.process(img)
    result = detector.detect(binary)
    steering = controller.compute(result.weighted_center)

    # Track is to the left → error is positive → steering should be positive
    assert steering > 0.1, f"Expected positive steering, got {steering}"
    print(f"  ✓ test_steering_off_centre  (steering={steering:+.4f})")


# ── Runner ───────────────────────────────────────────────────────────────

def main():
    print("=" * 60)
    print("  ScanLine Pipeline Tests")
    print("=" * 60)

    tests = [
        test_grayscale_conversion,
        test_threshold_output,
        test_pipeline_end_to_end,
        test_scanline_centred_track,
        test_scanline_off_centre_track,
        test_steering_centred,
        test_steering_off_centre,
    ]

    passed = 0
    failed = 0
    for test in tests:
        try:
            test()
            passed += 1
        except AssertionError as e:
            print(f"  ✗ {test.__name__}: {e}")
            failed += 1
        except Exception as e:
            print(f"  ✗ {test.__name__}: UNEXPECTED ERROR: {e}")
            failed += 1

    print("=" * 60)
    print(f"  Results: {passed} passed, {failed} failed")
    print("=" * 60)

    return 0 if failed == 0 else 1


if __name__ == "__main__":
    sys.exit(main())
