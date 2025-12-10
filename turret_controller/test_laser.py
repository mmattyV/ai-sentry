#!/usr/bin/env python3
"""
Test script to blink the laser and verify hardware works.
"""

import serial
import serial.tools.list_ports
import time


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
        return

    print(f"Connecting to {port}...")
    ser = serial.Serial(port, 921600, timeout=1)
    time.sleep(2)  # Wait for ESP32 to be ready
    ser.reset_input_buffer()
    
    print("\n=== LASER BLINK TEST ===")
    print("Watch the laser - it should blink 5 times")
    print("Press Ctrl+C to stop\n")

    try:
        for i in range(5):
            # Laser ON
            cmd = f"<P:90,T:90,L:1,F:0>"
            ser.write(cmd.encode())
            print(f"  Blink {i+1}: LASER ON")
            time.sleep(0.5)
            
            # Laser OFF
            cmd = f"<P:90,T:90,L:0,F:0>"
            ser.write(cmd.encode())
            print(f"  Blink {i+1}: LASER OFF")
            time.sleep(0.5)
        
        print("\n=== TEST COMPLETE ===")
        print("Did you see the laser blink 5 times?")
        print("  YES -> Laser hardware works! Issue is in firing logic.")
        print("  NO  -> Check laser wiring to pin D2 (GPIO3)")
        
        # Leave laser off
        ser.write(b"<P:90,T:90,L:0,F:0>")
        
    except KeyboardInterrupt:
        print("\nStopped by user")
        ser.write(b"<P:90,T:90,L:0,F:0>")
    
    ser.close()


if __name__ == "__main__":
    main()

