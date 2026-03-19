# ScanLine — NXP Track Detection with Scan Line Steering

A modular Python pipeline that captures frames from a **Pixy2** camera (or a webcam for testing), converts to grayscale, applies a binary threshold, detects the NXP track borders using **scan lines**, and outputs a steering value.

## Architecture

```
┌────────────┐    ┌────────────────┐    ┌──────────────────┐    ┌────────────┐
│   Camera    │───▶│   Processing   │───▶│    Detection      │───▶│  Steering  │
│  (capture)  │    │ gray+threshold │    │  (scan lines)     │    │ controller │
└────────────┘    └────────────────┘    └──────────────────┘    └────────────┘
      │                                         │                       │
      └─────────────────────┬───────────────────┘                       │
                            ▼                                           │
                    ┌───────────────┐                                   │
                    │ Visualization │◀──────────────────────────────────┘
                    │  (debug view) │
                    └───────────────┘
```

## Project Structure

```
ScanLine/
├── main.py                  # Entry point
├── config.py                # All tunable parameters
├── requirements.txt
├── camera/
│   ├── base.py              # Abstract camera interface
│   ├── pixy2_cam.py         # Pixy2 USB backend (pyusb)
│   └── webcam.py            # Webcam backend (OpenCV)
├── processing/
│   └── pipeline.py          # Grayscale + threshold
├── detection/
│   └── scanline.py          # Scan line track detector
├── steering/
│   └── controller.py        # Proportional steering
├── visualization.py         # Debug overlay drawing
└── tests/
    └── test_pipeline.py     # Synthetic image tests
```

## Setup

```bash
cd ~/ScanLine
pip install -r requirements.txt
```

> **Note:** For Pixy2 USB access on Linux you may need a udev rule or run with `sudo`.

## Usage

### Desktop testing (webcam)
```bash
python main.py --camera webcam
```

### NavQ Plus with Pixy2
```bash
python main.py --camera pixy2
```

### Headless mode (no display window)
```bash
python main.py --camera webcam --no-display
```

### Custom threshold
```bash
python main.py --camera webcam --threshold 80
```

Press **q** to quit when the debug window is active.

## Configuration

Edit `config.py` to tune:

| Parameter | Default | Description |
|---|---|---|
| `THRESHOLD_VALUE` | 60 | Grayscale threshold (0-255) |
| `SCAN_LINE_ROWS` | `[100,120,140,160,180]` | Y-row positions for scan lines |
| `SCAN_LINE_WEIGHTS` | `[0.05,0.10,0.15,0.30,0.40]` | Per-row weight (bottom = more) |
| `STEERING_KP` | 0.01 | Proportional steering gain |

## How the Scan Line Method Works

1. The binary image (black borders → white after threshold inversion) is scanned at multiple horizontal rows.
2. For each row, the algorithm scans **inward from both edges** to find the first white pixel (track border).
3. The **center** between left and right borders is computed per row.
4. A **weighted average** across all rows gives the final track center, with closer (lower) rows weighted more.
5. A proportional controller converts the offset between image center and track center into a steering value in `[-1.0, 1.0]`.
