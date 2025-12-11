# AI Sentry - Automated Laser Turret

An autonomous person-tracking laser turret using a XIAO ESP32S3 Sense with YOLOv8 computer vision on a MacBook.

## Overview

This project creates a pan-tilt turret with a laser pointer that automatically detects and tracks people. The ESP32S3 streams camera frames to a MacBook via USB serial, which runs YOLOv8 person detection and sends servo control commands back.

```
┌─────────────────────┐        USB Serial         ┌─────────────────────┐
│   XIAO ESP32S3      │◄─────────────────────────►│      MacBook        │
│   Sense             │                           │                     │
│                     │   Request + Commands ───► │  - YOLOv8 (MPS/GPU) │
│   - OV2640 Camera   │                           │  - PD Control       │
│   - Pan/Tilt Servos │   ◄─── JPEG Frame         │  - Target Cycling   │
│   - Laser Module    │                           │  - Live Viz Window  │
└─────────────────────┘                           └─────────────────────┘
```

**Key Features:**
- Real-time person detection with YOLOv8
- Low-latency request-response frame protocol
- PD (Proportional-Derivative) control to prevent overshoot
- Multi-target cycling (tracks multiple people)
- Apple Silicon GPU acceleration (MPS)
- Live visualization with debug info

## Hardware Requirements

### Components

| Component | Description |
|-----------|-------------|
| XIAO ESP32S3 Sense | Microcontroller with built-in OV2640 camera |
| 2x SG90 Servos | Pan and tilt movement |
| Laser Module | 5V laser pointer (< 5mW for safety) |
| Pan-Tilt Bracket | Mounting bracket for servos |
| USB-C Cable | Data + power (not charge-only!) |

### Wiring Diagram

```
XIAO ESP32S3 Sense
        │
        ├── D0 (GPIO1) ──► Pan Servo Signal (orange/yellow)
        ├── D1 (GPIO2) ──► Tilt Servo Signal (orange/yellow)
        ├── D2 (GPIO3) ──► Laser Signal (or transistor gate)
        ├── 5V ──────────► Servo VCC (red) + Laser VCC
        └── GND ─────────► Servo GND (brown) + Laser GND
```

**Important:** The camera module must be attached to the XIAO ESP32S3 Sense expansion board.

### Servo Orientation

- **Pan Servo (D0):** Increasing angle → moves RIGHT
- **Tilt Servo (D1):** Increasing angle → moves DOWN

The software automatically compensates if your camera is mounted upside down.

## Software Setup

### Prerequisites

- macOS with Apple Silicon (M1/M2/M3) recommended for GPU acceleration
- Python 3.9+
- PlatformIO (for ESP32 firmware)

### 1. Install PlatformIO

```bash
# Using pipx (recommended)
brew install pipx
pipx install platformio

# Or using pip
pip install platformio
```

### 2. Upload ESP32 Firmware

```bash
cd esp32_turret

# Put XIAO in bootloader mode:
# 1. Hold BOOT button
# 2. Press and release RESET while holding BOOT
# 3. Release BOOT

pio run --target upload
```

The laser will flash **twice** when ready.

### 3. Setup Python Environment

```bash
cd turret_controller
python3 -m venv venv
source venv/bin/activate
pip install -r requirements.txt
```

### 4. Run the Controller

```bash
python main.py
```

A visualization window will open showing the camera feed with detection overlays.

## Usage

### Basic Commands

```bash
# Run with auto-detected settings
python main.py

# List available serial ports
python main.py --list-ports

# Specify serial port manually
python main.py --port /dev/cu.usbmodem2101

# Test with MacBook webcam (no ESP32 needed)
python main.py --webcam
```

### Command Line Options

| Option | Default | Description |
|--------|---------|-------------|
| `--port, -p` | auto | Serial port |
| `--baud, -b` | 921600 | Baud rate |
| `--kp` | 0.2 | Proportional gain (lower = smoother) |
| `--threshold, -t` | 30.0 | Lock-on threshold in pixels |
| `--cycle, -c` | 3.0 | Target switch interval (seconds) |
| `--no-auto-fire` | false | Disable automatic laser firing |
| `--mock` | false | Use mock detector (no YOLO) |
| `--webcam` | false | Use MacBook webcam instead |

### Tuning Tips

```bash
# Smoother tracking (less overshoot)
python main.py --kp 0.15

# Tighter lock required before firing
python main.py --threshold 20

# Faster target switching
python main.py --cycle 2.0

# Disable laser for testing
python main.py --no-auto-fire
```

### Keyboard Controls

| Key | Action |
|-----|--------|
| Q / ESC | Quit application |

## Test Scripts

### Test Laser Hardware

```bash
python test_laser.py
```

Blinks the laser 5 times to verify wiring is correct.

### Test Serial Communication

```bash
python test_serial.py
```

Reads raw serial data to verify camera is streaming.

## System Architecture

### Communication Protocol

The system uses a **request-response model** for minimal latency:

1. Python sends: `<P:pan,T:tilt,L:laser,F:1>` (command + frame request)
2. ESP32 captures fresh frame and sends: `[0xAA55AA55][length][JPEG data]`
3. Python processes frame, sends next command
4. Repeat

This ensures each frame is captured **on-demand**, eliminating buffering delays.

### Control System

**PD (Proportional-Derivative) Control:**

```
error = target_position - frame_center
d_error = error - previous_error
output = Kp * error + Kd * d_error
```

- **P term:** Moves toward target
- **D term:** Dampens movement to prevent overshoot from latency

### Target Management

When multiple people are detected:
1. Sorted by bounding box area (largest/closest first)
2. Current target tracked for `cycle_interval` seconds
3. Automatically switches to next person
4. Immediate switch if current target is lost

### Laser Firing Logic

The laser fires when:
1. A target is detected
2. Aim error is below `threshold` pixels (locked)
3. `auto_fire` is enabled (default: true)

## File Structure

```
ai-sentry/
├── esp32_turret/
│   ├── platformio.ini          # PlatformIO config
│   └── src/
│       └── main.cpp            # ESP32 firmware
│
├── turret_controller/
│   ├── requirements.txt        # Python dependencies
│   ├── main.py                 # Main application
│   ├── serial_comm.py          # Serial + frame protocol
│   ├── detector.py             # YOLOv8 detection (MPS support)
│   ├── targeting.py            # Multi-target management
│   ├── controller.py           # PD servo controller
│   ├── visualizer.py           # OpenCV visualization
│   ├── test_laser.py           # Laser hardware test
│   └── test_serial.py          # Serial communication test
│
├── README.md
└── LICENSE
```

## Troubleshooting

### "Waiting for frames..." (no video)

1. **Press RESET** on the XIAO ESP32S3
2. Wait for **2 laser blinks** (indicates ready)
3. Run Python script immediately after

If laser blinks **slowly** (every 1 second): Camera failed to initialize. Check that camera module is firmly attached.

### Corrupt JPEG / Glitchy Video

The system auto-recovers from corrupt frames. If it persists:
- Try lower baud rate: `python main.py --baud 115200`
- Check USB cable is data-capable (not charge-only)

### Servo Overshooting / Oscillating

```bash
# Lower the proportional gain
python main.py --kp 0.1

# Or increase lock threshold
python main.py --threshold 40
```

### Laser Not Firing

1. Run `python test_laser.py` to verify hardware
2. Check wiring: Laser signal → D2 (GPIO3)
3. Verify "LOCKED" shows in visualization window
4. Check `--no-auto-fire` is not set

### Low FPS

- MPS (Apple GPU) should auto-enable on Apple Silicon
- Check with: `python -c "import torch; print(torch.backends.mps.is_available())"`
- Ensure good lighting for faster detection

### Upload Failed (ESP32)

1. Put in bootloader mode: Hold BOOT → Press RESET → Release BOOT
2. Try again: `pio run --target upload`

## Safety Warning

⚠️ **LASER SAFETY:**
- Use only low-power lasers (< 5mW, Class 2)
- **NEVER** point at eyes or reflective surfaces
- Consider using a visible LED for testing
- Supervise operation at all times

## Dependencies

**ESP32 (PlatformIO):**
- espressif32 platform
- ESP32Servo library

**Python:**
- pyserial - Serial communication
- opencv-python - Image processing
- ultralytics - YOLOv8 detection
- numpy - Array operations
- torch - MPS/GPU acceleration

## License

MIT License - See [LICENSE](LICENSE) for details.
