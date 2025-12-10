# AI Sentry - Automated Laser Turret

An autonomous person-tracking laser turret system using a XIAO ESP32S3 Sense with computer vision processing on a MacBook.

## Overview

This project creates a pan-tilt turret with a laser pointer that automatically detects and tracks people using YOLOv8 person detection. The ESP32S3 streams camera frames to a MacBook for processing, which then sends servo control commands back to aim the turret.

```
┌─────────────────────┐         Serial USB          ┌─────────────────────┐
│   XIAO ESP32S3      │◄───────────────────────────►│      MacBook        │
│   Sense             │                             │                     │
│                     │      JPEG frames ────►      │  - YOLOv8 Detection │
│   - OV2640 Camera   │                             │  - Target Tracking  │
│   - Pan/Tilt Servos │      ◄──── Commands         │  - Servo Control    │
│   - Laser Module    │                             │  - Visualization    │
└─────────────────────┘                             └─────────────────────┘
```

## Hardware Setup

### Components

- **XIAO ESP32S3 Sense** (with built-in OV2640 camera)
- **2x Servo Motors** (SG90 or similar for pan-tilt)
- **Laser Module** (5V, low-power for safety)
- **Pan-Tilt Bracket** (3D printed or purchased)
- **USB-C Cable** (for power and data)

### Wiring

| Component | XIAO Pin | Notes |
|-----------|----------|-------|
| Pan Servo | D0 (GPIO1) | Signal wire (orange/yellow) |
| Tilt Servo | D1 (GPIO2) | Signal wire (orange/yellow) |
| Laser | D2 (GPIO3) | Signal or transistor gate |
| Servo VCC | 5V | Red wire (both servos) |
| Servo GND | GND | Brown/black wire |
| Laser VCC | 5V | Or through transistor |
| Laser GND | GND | |

### Servo Orientation

- **Pan Servo (D0)**: Increasing angle moves RIGHT
- **Tilt Servo (D1)**: Increasing angle moves DOWN

## Software Setup

### ESP32S3 Firmware

1. Install [PlatformIO](https://platformio.org/) in VS Code

2. Open the `esp32_turret` folder in VS Code

3. Build and upload:
   ```bash
   cd esp32_turret
   pio run --target upload
   ```

4. The laser will flash briefly when ready

### MacBook Controller

1. Create a Python virtual environment:
   ```bash
   cd turret_controller
   python3 -m venv venv
   source venv/bin/activate
   ```

2. Install dependencies:
   ```bash
   pip install -r requirements.txt
   ```

3. Run the controller:
   ```bash
   python main.py
   ```

## Usage

### Basic Operation

```bash
# Auto-detect serial port and run
python main.py

# Specify serial port
python main.py --port /dev/cu.usbmodem*

# List available ports
python main.py --list-ports
```

### Command Line Options

| Option | Default | Description |
|--------|---------|-------------|
| `--port, -p` | auto | Serial port |
| `--baud, -b` | 2000000 | Baud rate |
| `--mock` | false | Use mock detector (no YOLO) |
| `--cycle, -c` | 3.0 | Target switch interval (seconds) |
| `--threshold, -t` | 20.0 | Lock-on threshold (pixels) |
| `--kp` | 0.3 | Proportional gain |
| `--no-auto-fire` | false | Disable automatic laser |
| `--dwell` | 0.5 | Lock time before firing (seconds) |
| `--webcam` | false | Use MacBook webcam for testing |

### Testing Without Hardware

Test the system using your MacBook's webcam:
```bash
python main.py --webcam
```

### Keyboard Controls

- **Q** or **ESC**: Quit the application

## System Architecture

### Data Flow

1. **ESP32S3** captures JPEG frame from OV2640 camera
2. **Serial** transmits frame to MacBook (JPEG start/end markers)
3. **Detector** runs YOLOv8 person detection
4. **Target Manager** selects current target, handles cycling
5. **Controller** calculates servo angles using proportional control
6. **Serial** sends command `<P:pan,T:tilt,L:laser>` to ESP32
7. **ESP32S3** moves servos and controls laser
8. **Visualizer** displays annotated frame with debug info

### Control System

The system uses closed-loop visual servoing with proportional control:

```
Error = TargetPosition - FrameCenter
ServoDelta = Kp × Error × DegreesPerPixel
NewAngle = CurrentAngle + ServoDelta
```

- **Deadzone**: No adjustment if error < threshold (prevents jitter)
- **Dwell Time**: Laser only fires after stable lock

### Target Cycling

When multiple people are detected:
1. Targets are sorted by bounding box area (largest first)
2. System tracks current target for `cycle_interval` seconds
3. Automatically switches to next target
4. If current target is lost, immediately switches

## File Structure

```
ai-sentry/
├── esp32_turret/
│   ├── platformio.ini        # PlatformIO configuration
│   └── src/
│       └── main.cpp          # ESP32 firmware
├── turret_controller/
│   ├── requirements.txt      # Python dependencies
│   ├── main.py               # Main application entry point
│   ├── serial_comm.py        # Serial communication
│   ├── detector.py           # YOLOv8 person detection
│   ├── targeting.py          # Multi-target management
│   ├── controller.py         # Servo control logic
│   └── visualizer.py         # OpenCV visualization
├── LICENSE
└── README.md
```

## Troubleshooting

### Serial Connection Issues

```bash
# List available ports
python main.py --list-ports

# Check if ESP32 is detected
ls /dev/cu.usb*
```

### Low Frame Rate

- Reduce JPEG quality in ESP32 firmware (increase `jpeg_quality` value)
- Ensure USB cable supports data transfer (not charge-only)
- Try lower baud rate: `--baud 921600`

### Servo Jitter

- Increase deadzone: `--threshold 30`
- Decrease Kp: `--kp 0.2`
- Check servo power supply (use external 5V if needed)

### Detection Issues

- Ensure good lighting
- Test with `--webcam` to verify detector works
- Try mock detector: `--mock`

## Safety Notes

⚠️ **Laser Safety**:
- Use low-power laser (< 5mW, Class 2)
- Never point at eyes or reflective surfaces
- Consider using visible LED instead of laser for testing

## License

MIT License - See LICENSE file for details
