"""
Serial Communication Module for Laser Turret

Handles serial communication with the ESP32S3, including:
- JPEG frame parsing from the serial stream
- Sending servo and laser commands
"""

import serial
import serial.tools.list_ports
from typing import Optional, Tuple
import time


class SerialComm:
    """Handles serial communication with the ESP32S3 turret controller."""

    # Frame protocol markers
    FRAME_START = bytes([0xAA, 0x55, 0xAA, 0x55])
    FRAME_HEADER_SIZE = 8  # 4 bytes marker + 4 bytes length

    def __init__(self, port: Optional[str] = None, baud_rate: int = 2000000):
        """
        Initialize serial communication.

        Args:
            port: Serial port name (e.g., '/dev/cu.usbmodem*'). Auto-detect if None.
            baud_rate: Baud rate for serial communication.
        """
        self.baud_rate = baud_rate
        self.serial: Optional[serial.Serial] = None
        self.buffer = bytearray()

        if port is None:
            port = self._auto_detect_port()

        if port:
            self.connect(port)

    def _auto_detect_port(self) -> Optional[str]:
        """Auto-detect the ESP32S3 serial port."""
        ports = serial.tools.list_ports.comports()

        for port in ports:
            # Look for common ESP32 identifiers
            desc_lower = port.description.lower()
            if any(x in desc_lower for x in ['esp32', 'usb', 'serial', 'usbmodem']):
                print(f"Auto-detected port: {port.device} ({port.description})")
                return port.device

        # On macOS, look for /dev/cu.usbmodem*
        for port in ports:
            if 'usbmodem' in port.device or 'usbserial' in port.device:
                print(f"Auto-detected port: {port.device}")
                return port.device

        print("Could not auto-detect serial port. Available ports:")
        for port in ports:
            print(f"  {port.device}: {port.description}")

        return None

    def connect(self, port: str) -> bool:
        """
        Connect to the specified serial port.

        Args:
            port: Serial port name.

        Returns:
            True if connection successful, False otherwise.
        """
        try:
            self.serial = serial.Serial(
                port=port,
                baudrate=self.baud_rate,
                timeout=0.1,
                write_timeout=1.0
            )
            # Clear any pending data
            time.sleep(0.5)
            self.serial.reset_input_buffer()
            self.serial.reset_output_buffer()
            print(f"Connected to {port} at {self.baud_rate} baud")
            return True
        except serial.SerialException as e:
            print(f"Failed to connect to {port}: {e}")
            return False

    def disconnect(self):
        """Disconnect from the serial port."""
        if self.serial and self.serial.is_open:
            self.serial.close()
            print("Serial connection closed")

    def is_connected(self) -> bool:
        """Check if serial connection is active."""
        return self.serial is not None and self.serial.is_open

    def read_frame(self) -> Optional[bytes]:
        """
        Read a single JPEG frame from the serial stream using length-prefix protocol.
        
        Protocol: [4 bytes: 0xAA55AA55 marker][4 bytes: length (LE)][JPEG data]

        Returns:
            JPEG frame data as bytes, or None if no complete frame available.
        """
        if not self.is_connected():
            return None

        # Read available data into buffer
        try:
            available = self.serial.in_waiting
            if available > 0:
                data = self.serial.read(available)
                self.buffer.extend(data)
            elif len(self.buffer) < 1000:
                # If buffer is small, do a small blocking read
                data = self.serial.read(1024)
                if data:
                    self.buffer.extend(data)
        except serial.SerialException as e:
            print(f"Serial read error: {e}")
            return None

        # Need at least header size to process
        if len(self.buffer) < self.FRAME_HEADER_SIZE:
            return None

        # Look for frame start marker
        start_idx = self.buffer.find(self.FRAME_START)
        if start_idx == -1:
            # No start marker found, keep last 3 bytes (in case marker is split)
            if len(self.buffer) > 3:
                self.buffer = self.buffer[-3:]
            return None

        # Discard data before start marker
        if start_idx > 0:
            self.buffer = self.buffer[start_idx:]

        # Need full header
        if len(self.buffer) < self.FRAME_HEADER_SIZE:
            return None

        # Extract frame length (little endian, bytes 4-7)
        frame_len = (
            self.buffer[4] |
            (self.buffer[5] << 8) |
            (self.buffer[6] << 16) |
            (self.buffer[7] << 24)
        )

        # Sanity check on frame length
        if frame_len <= 0 or frame_len > 500000:  # Max 500KB
            # Invalid length, skip this marker and try to find next
            self.buffer = self.buffer[4:]
            return None

        # Check if we have the complete frame
        total_size = self.FRAME_HEADER_SIZE + frame_len
        if len(self.buffer) < total_size:
            # Not enough data yet
            return None

        # Extract the JPEG frame
        frame = bytes(self.buffer[self.FRAME_HEADER_SIZE:total_size])
        self.buffer = self.buffer[total_size:]

        # If buffer is getting too large, we're falling behind - clear it
        # This drops old frames to stay current
        if len(self.buffer) > 50000:  # More than ~5 frames backed up
            print(f"Dropping {len(self.buffer)} bytes of backed up frames")
            self.buffer.clear()
            if self.serial:
                self.serial.reset_input_buffer()

        return frame

    def send_command(self, pan: int, tilt: int, laser: bool, request_frame: bool = True) -> bool:
        """
        Send servo and laser commands to the ESP32S3.

        Args:
            pan: Pan servo angle (0-180).
            tilt: Tilt servo angle (0-180).
            laser: Laser state (True = on, False = off).
            request_frame: Whether to request a new frame.

        Returns:
            True if command sent successfully, False otherwise.
        """
        if not self.is_connected():
            return False

        # Format: <P:angle,T:angle,L:0/1,F:0/1>
        cmd = f"<P:{pan},T:{tilt},L:{1 if laser else 0},F:{1 if request_frame else 0}>"

        try:
            self.serial.write(cmd.encode('ascii'))
            return True
        except serial.SerialException as e:
            print(f"Failed to send command: {e}")
            return False
    
    def request_frame(self) -> bool:
        """Request a new frame from ESP32."""
        if not self.is_connected():
            return False
        try:
            self.serial.write(b"<F:1>")
            return True
        except serial.SerialException as e:
            print(f"Failed to request frame: {e}")
            return False

    def list_ports(self) -> list:
        """List all available serial ports."""
        ports = serial.tools.list_ports.comports()
        return [(p.device, p.description) for p in ports]


def find_esp32_port() -> Optional[str]:
    """Utility function to find ESP32 port."""
    comm = SerialComm.__new__(SerialComm)
    return comm._auto_detect_port()


if __name__ == "__main__":
    # Test serial communication
    print("Available serial ports:")
    ports = serial.tools.list_ports.comports()
    for port in ports:
        print(f"  {port.device}: {port.description}")

    print("\nAttempting auto-detection...")
    comm = SerialComm()

    if comm.is_connected():
        print("\nReading frames...")
        frame_count = 0
        start_time = time.time()

        try:
            while frame_count < 10:
                frame = comm.read_frame()
                if frame:
                    frame_count += 1
                    print(f"Frame {frame_count}: {len(frame)} bytes")

                    # Send a test command
                    comm.send_command(90, 90, False)

            elapsed = time.time() - start_time
            print(f"\nReceived {frame_count} frames in {elapsed:.2f}s ({frame_count/elapsed:.1f} fps)")

        except KeyboardInterrupt:
            pass

        comm.disconnect()

