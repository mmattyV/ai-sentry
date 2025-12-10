#!/usr/bin/env python3
"""
Simple serial test script to debug ESP32 communication.
"""

import serial
import serial.tools.list_ports
import time
import sys


def find_port():
    """Find ESP32 port."""
    ports = serial.tools.list_ports.comports()
    for port in ports:
        if 'usbmodem' in port.device or 'usbserial' in port.device:
            return port.device
    return None


def main():
    port = find_port()
    if not port:
        print("No ESP32 port found!")
        print("Available ports:")
        for p in serial.tools.list_ports.comports():
            print(f"  {p.device}: {p.description}")
        return

    print(f"Connecting to {port}...")
    
    # Try different baud rates
    baud_rates = [2000000, 921600, 115200]
    
    for baud in baud_rates:
        print(f"\nTrying baud rate: {baud}")
        try:
            ser = serial.Serial(port, baud, timeout=1)
            time.sleep(0.5)
            ser.reset_input_buffer()
            
            print("Reading for 5 seconds...")
            start = time.time()
            total_bytes = 0
            jpeg_starts = 0
            jpeg_ends = 0
            
            buffer = bytearray()
            
            while time.time() - start < 5:
                data = ser.read(1024)
                if data:
                    total_bytes += len(data)
                    buffer.extend(data)
                    
                    # Count JPEG markers
                    jpeg_starts += data.count(b'\xff\xd8')
                    jpeg_ends += data.count(b'\xff\xd9')
                    
                    # Show first few bytes if text
                    if len(buffer) < 500:
                        try:
                            text = data.decode('utf-8', errors='ignore')
                            if text.strip():
                                print(f"Text: {text.strip()}")
                        except:
                            pass
            
            ser.close()
            
            print(f"\nResults at {baud} baud:")
            print(f"  Total bytes received: {total_bytes}")
            print(f"  JPEG start markers (FFD8): {jpeg_starts}")
            print(f"  JPEG end markers (FFD9): {jpeg_ends}")
            print(f"  Estimated frames: {min(jpeg_starts, jpeg_ends)}")
            
            if total_bytes > 0:
                print(f"\nFirst 100 bytes (hex): {buffer[:100].hex()}")
                
                # Try to find first JPEG
                start_idx = buffer.find(b'\xff\xd8')
                if start_idx >= 0:
                    print(f"\nFirst JPEG start at byte: {start_idx}")
                    end_idx = buffer.find(b'\xff\xd9', start_idx)
                    if end_idx >= 0:
                        frame_size = end_idx - start_idx + 2
                        print(f"First frame size: {frame_size} bytes")
            
            if jpeg_starts > 0:
                print(f"\n✓ Camera appears to be working at {baud} baud!")
                return
                
        except Exception as e:
            print(f"Error: {e}")
            continue
    
    print("\n✗ No JPEG data received at any baud rate.")
    print("Check that:")
    print("  1. Camera module is properly attached to XIAO")
    print("  2. ESP32 firmware is uploaded correctly")
    print("  3. Press RESET button on XIAO after upload")


if __name__ == "__main__":
    main()

