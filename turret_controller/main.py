#!/usr/bin/env python3
"""
Laser Turret Controller - Main Application

Integrates all components:
- Serial communication with ESP32S3
- YOLOv8 person detection
- Multi-target tracking with cycling
- Proportional servo control
- Real-time visualization
"""

import argparse
import time
import sys
import os
from typing import Optional

import cv2
import numpy as np

# Suppress OpenCV JPEG decode warnings
os.environ["OPENCV_LOG_LEVEL"] = "SILENT"

from serial_comm import SerialComm
from detector import PersonDetector, create_detector, Detection
from targeting import TargetManager, TargetState
from controller import ServoController, ControllerOutput
from visualizer import Visualizer


class LaserTurret:
    """
    Main laser turret controller class.

    Coordinates all subsystems for autonomous person tracking and targeting.
    """

    def __init__(
        self,
        serial_port: Optional[str] = None,
        baud_rate: int = 2000000,
        use_mock_detector: bool = False,
        cycle_interval: float = 3.0,
        lock_threshold: float = 20.0,
        kp: float = 0.3,
        auto_fire: bool = True,
        dwell_time: float = 0.5
    ):
        """
        Initialize the laser turret system.

        Args:
            serial_port: Serial port to use (auto-detect if None).
            baud_rate: Serial baud rate.
            use_mock_detector: Use mock detector instead of YOLO.
            cycle_interval: Seconds between target switches.
            lock_threshold: Pixel error for lock-on.
            kp: Proportional gain for servo control.
            auto_fire: Automatically fire laser when locked.
            dwell_time: Seconds to stay locked before firing.
        """
        self.auto_fire = auto_fire
        self.dwell_time = dwell_time
        self.lock_start_time: Optional[float] = None

        # Frame dimensions (will be updated from actual frames)
        self.frame_width = 320
        self.frame_height = 240

        # Initialize subsystems
        print("Initializing laser turret system...")

        # Serial communication
        print("  Connecting to ESP32S3...")
        self.serial = SerialComm(port=serial_port, baud_rate=baud_rate)
        if not self.serial.is_connected():
            print("  WARNING: Serial not connected. Running in simulation mode.")

        # Person detector
        print("  Loading person detector...")
        self.detector = create_detector(use_mock=use_mock_detector)

        # Target manager
        self.target_manager = TargetManager(
            cycle_interval=cycle_interval,
            lock_threshold=lock_threshold,
            neutral_position=(self.frame_width // 2, self.frame_height // 2)
        )

        # Servo controller with PD control to handle latency
        self.servo_controller = ServoController(
            frame_size=(self.frame_width, self.frame_height),
            kp=kp,
            kd=0.4,  # Derivative gain to prevent overshoot from latency
            deadzone=lock_threshold
        )

        # Visualizer
        self.visualizer = Visualizer("Laser Turret - Press Q to quit")

        # State tracking
        self.running = False
        self.laser_on = False
        self.frame_count = 0
        self.fps = 0.0
        self.fps_update_time = time.time()
        self.fps_frame_count = 0
        self.corrupt_frame_count = 0

        print("  Initialization complete!")

    def process_frame(self, jpeg_data: bytes) -> bool:
        """
        Process a single frame from the camera.

        Args:
            jpeg_data: Raw JPEG image data.

        Returns:
            True to continue, False to stop.
        """
        # Decode JPEG
        nparr = np.frombuffer(jpeg_data, np.uint8)
        frame = cv2.imdecode(nparr, cv2.IMREAD_COLOR)

        if frame is None:
            self.corrupt_frame_count += 1
            # If too many corrupt frames, reset the serial buffer
            if self.corrupt_frame_count >= 10:
                print(f"Too many corrupt frames ({self.corrupt_frame_count}), resetting buffer...")
                self.serial.buffer.clear()
                if self.serial.serial:
                    self.serial.serial.reset_input_buffer()
                self.corrupt_frame_count = 0
            return True  # Skip invalid/corrupt frame
        
        # Valid frame received, reset corrupt counter
        self.corrupt_frame_count = 0
        
        # Flip image (camera is mounted upside down) - 180 degree rotation
        frame = cv2.flip(frame, -1)
        
        # Run person detection on the correctly oriented frame
        detections = self.detector.detect(frame)

        # Update frame dimensions if changed
        h, w = frame.shape[:2]
        if w != self.frame_width or h != self.frame_height:
            self.frame_width = w
            self.frame_height = h
            self.servo_controller.update_frame_size(w, h)
            self.target_manager.neutral_position = (w // 2, h // 2)

        # Get target position
        target_position = self.target_manager.get_target_position()

        # Invert only X because of how camera/servo are mounted
        # Pan (X): needs inversion due to 180° image flip
        # Tilt (Y): does NOT need inversion
        corrected_target = (
            self.frame_width - target_position[0],   # Invert X for pan
            target_position[1]                        # Keep Y as-is for tilt
        )
        
        # Calculate servo control with corrected position
        control_output = self.servo_controller.compute(corrected_target)

        # Update target state with current error
        error = (control_output.pan_error, control_output.tilt_error)
        target_state = self.target_manager.update(detections, error)

        # Determine laser state
        self.laser_on = self._should_fire(target_state)

        # Debug: print when laser state changes
        if self.laser_on:
            print(f"FIRING! is_locked={target_state.is_locked}, target={target_state.current_target is not None}")

        # Send commands to ESP32 and request next frame
        self.serial.send_command(
            pan=control_output.pan_angle,
            tilt=control_output.tilt_angle,
            laser=self.laser_on,
            request_frame=True  # Request next frame with command
        )

        # Update FPS counter
        self._update_fps()

        # Draw visualization
        display = self.visualizer.draw_frame(
            frame=frame,
            detections=detections,
            target_state=target_state,
            servo_angles=(control_output.pan_angle, control_output.tilt_angle),
            error=error,
            laser_on=self.laser_on,
            fps=self.fps
        )

        # Show frame and check for quit
        return self.visualizer.show(display)

    def _should_fire(self, target_state: TargetState) -> bool:
        """Determine if laser should fire - fires when locked on target."""
        if not self.auto_fire:
            return False

        # Fire immediately when locked (reticle is close to target center)
        return target_state.is_locked and target_state.current_target is not None

    def _update_fps(self):
        """Update FPS calculation."""
        self.fps_frame_count += 1
        current_time = time.time()
        elapsed = current_time - self.fps_update_time

        if elapsed >= 1.0:
            self.fps = self.fps_frame_count / elapsed
            self.fps_frame_count = 0
            self.fps_update_time = current_time

    def run(self):
        """Main run loop - process frames until stopped."""
        print("\nStarting laser turret...")
        print("Press 'Q' in the window or Ctrl+C to stop.\n")

        self.running = True
        no_frame_count = 0
        last_status_time = time.time()

        # Clear any stale data that accumulated during initialization
        if self.serial.is_connected():
            self.serial.buffer.clear()
            if self.serial.serial:
                self.serial.serial.reset_input_buffer()
            print("Serial buffer cleared, waiting for fresh frames...")

        try:
            # Request first frame
            if self.serial.is_connected():
                self.serial.request_frame()
                time.sleep(0.05)  # Give ESP32 time to capture and send

            while self.running:
                if self.serial.is_connected():
                    # Read frame from serial
                    jpeg_data = self.serial.read_frame()
                    if jpeg_data:
                        no_frame_count = 0
                        if not self.process_frame(jpeg_data):
                            break
                        # Note: process_frame sends command with request_frame=True
                    else:
                        no_frame_count += 1
                        # Print status every 2 seconds if no frames
                        if time.time() - last_status_time > 2.0:
                            print(f"Waiting for frames... (buffer: {len(self.serial.buffer)} bytes)")
                            last_status_time = time.time()
                            # Re-request frame in case it was missed
                            self.serial.request_frame()
                        time.sleep(0.005)  # Small delay when no frame
                else:
                    # Simulation mode - use webcam
                    self._run_simulation()
                    break

        except KeyboardInterrupt:
            print("\nInterrupted by user")
        finally:
            self.stop()

    def _run_simulation(self):
        """Run in simulation mode using webcam."""
        print("Running in simulation mode (using webcam)")

        cap = cv2.VideoCapture(0)
        if not cap.isOpened():
            print("Could not open webcam")
            return

        try:
            while self.running:
                ret, frame = cap.read()
                if not ret:
                    break

                # Resize to expected dimensions
                frame = cv2.resize(frame, (self.frame_width, self.frame_height))

                # Encode as JPEG
                _, jpeg_data = cv2.imencode('.jpg', frame)

                if not self.process_frame(jpeg_data.tobytes()):
                    break

        finally:
            cap.release()

    def stop(self):
        """Stop the turret and cleanup."""
        self.running = False

        # Turn off laser
        self.laser_on = False
        if self.serial.is_connected():
            self.serial.send_command(
                pan=90, tilt=90, laser=False
            )

        # Close serial
        self.serial.disconnect()

        # Close visualization
        self.visualizer.close()

        print("Laser turret stopped.")


def main():
    """Main entry point."""
    parser = argparse.ArgumentParser(
        description="Laser Turret Controller - Autonomous person tracking"
    )

    parser.add_argument(
        "--port", "-p",
        type=str,
        default=None,
        help="Serial port (auto-detect if not specified)"
    )

    parser.add_argument(
        "--baud", "-b",
        type=int,
        default=921600,
        help="Serial baud rate (default: 921600, use 2000000 for higher fps but less stability)"
    )

    parser.add_argument(
        "--mock",
        action="store_true",
        help="Use mock detector (no YOLO)"
    )

    parser.add_argument(
        "--cycle", "-c",
        type=float,
        default=3.0,
        help="Target cycle interval in seconds (default: 3.0)"
    )

    parser.add_argument(
        "--threshold", "-t",
        type=float,
        default=30.0,
        help="Lock-on threshold in pixels (default: 30.0, higher = easier to lock)"
    )

    parser.add_argument(
        "--kp",
        type=float,
        default=0.2,
        help="Proportional gain (default: 0.2, lower = smoother but slower)"
    )

    parser.add_argument(
        "--no-auto-fire",
        action="store_true",
        help="Disable automatic laser firing"
    )

    parser.add_argument(
        "--dwell",
        type=float,
        default=0.5,
        help="Dwell time before firing in seconds (default: 0.5)"
    )

    parser.add_argument(
        "--webcam",
        action="store_true",
        help="Use webcam instead of ESP32 camera"
    )

    parser.add_argument(
        "--list-ports",
        action="store_true",
        help="List available serial ports and exit"
    )

    args = parser.parse_args()

    # List ports if requested
    if args.list_ports:
        from serial_comm import SerialComm
        comm = SerialComm.__new__(SerialComm)
        ports = comm.list_ports() if hasattr(comm, 'list_ports') else []
        print("Available serial ports:")
        import serial.tools.list_ports
        for port in serial.tools.list_ports.comports():
            print(f"  {port.device}: {port.description}")
        return

    # Force simulation mode if webcam requested
    serial_port = None if args.webcam else args.port

    # Create and run turret
    turret = LaserTurret(
        serial_port=serial_port,
        baud_rate=args.baud,
        use_mock_detector=args.mock,
        cycle_interval=args.cycle,
        lock_threshold=args.threshold,
        kp=args.kp,
        auto_fire=not args.no_auto_fire,
        dwell_time=args.dwell
    )

    turret.run()


if __name__ == "__main__":
    main()

