#!/usr/bin/env python3
"""Simple NSH serial test - talk to PX4 board via USB CDC ACM."""

import serial
import sys
import time

PORT = sys.argv[1] if len(sys.argv) > 1 else "/dev/ttyACM0"
BAUD = 115200
TIMEOUT = 3

def main():
    print(f"Opening {PORT} at {BAUD} baud...")
    try:
        ser = serial.Serial(PORT, BAUD, timeout=TIMEOUT)
    except Exception as e:
        print(f"Failed to open port: {e}")
        return 1

    # Drain any existing data
    time.sleep(0.2)
    old = ser.read(ser.in_waiting or 0)
    if old:
        print(f"Drained {len(old)} bytes: {old[:200]}")

    # Send a few newlines to wake up NSH prompt
    print("Sending CR/LF to wake NSH...")
    for _ in range(3):
        ser.write(b"\r\n")
        time.sleep(0.3)

    resp = ser.read(ser.in_waiting or 0)
    print(f"After wake: {repr(resp)}")

    # Try sending 'ver all' command
    cmd = b"ver all\r\n"
    print(f"\nSending: {cmd}")
    ser.write(cmd)
    time.sleep(2)

    resp = b""
    while ser.in_waiting:
        chunk = ser.read(ser.in_waiting)
        resp += chunk
        time.sleep(0.1)

    print(f"Response ({len(resp)} bytes):")
    if resp:
        try:
            print(resp.decode("utf-8", errors="replace"))
        except:
            print(repr(resp))
    else:
        print("(no response)")

    # Try a few more commands
    for cmd_str in ["param show SYS_USB_AUTO", "top once", "dmesg"]:
        cmd = f"{cmd_str}\r\n".encode()
        print(f"\n--- Sending: {cmd_str} ---")
        ser.write(cmd)
        time.sleep(2)

        resp = b""
        while ser.in_waiting:
            chunk = ser.read(ser.in_waiting)
            resp += chunk
            time.sleep(0.1)

        if resp:
            try:
                print(resp.decode("utf-8", errors="replace"))
            except:
                print(repr(resp))
        else:
            print("(no response)")

    ser.close()
    print("\nDone.")
    return 0

if __name__ == "__main__":
    sys.exit(main())
