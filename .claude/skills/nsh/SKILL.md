---
name: nsh
description: Run an NSH command on a connected PX4 board via USB serial
argument-hint: <nsh-command>
---

# NSH Command

Connect to a PX4 board's NuttX Shell (NSH) via USB serial and run a command.

## Prerequisites

- Board must be connected via USB with `/dev/ttyACM0` present
- Board must have `SYS_USB_AUTO=1` (auto-detect mode) for direct NSH
- Python `serial` module must be available (conda env `px4`)

## Command Execution

Use the MCP PX4 server's NSH client if available, otherwise use direct serial:

### Method 1: Direct Serial (preferred)

PX4's cdcacm_autostart requires **3 consecutive carriage returns** (`\r\r\r`, i.e. `0x0D 0x0D 0x0D`) to trigger NSH in auto-detect mode (`SYS_USB_AUTO=1`).

```python
import serial, time

ser = serial.Serial('/dev/ttyACM0', 57600, timeout=2)
time.sleep(0.5)

# Send 3 carriage returns to trigger NSH auto-detect
ser.write(b'\r\r\r')
time.sleep(2)

# Read NSH banner
output = ser.read(ser.in_waiting or 4096)
print(output.decode('utf-8', errors='replace'))

# Send command (terminate with \r)
command = "$ARGUMENTS"
ser.write(f'{command}\r'.encode())
time.sleep(3)

# Read response
response = ser.read(ser.in_waiting or 8192)
print(response.decode('utf-8', errors='replace'))
ser.close()
```

### Method 2: MAVLink Shell (if SYS_USB_AUTO=2)

If the board is running MAVLink on USB (default `SYS_USB_AUTO=2`), use pymavlink
with `SERIAL_CONTROL` messages (devnum=10) to access the shell. See `Tools/mavlink_shell.py`.

## Common NSH Commands

| Command | Description |
|---------|-------------|
| `ver all` | Show firmware version and board info |
| `top once` | Show running tasks and CPU usage (single snapshot) |
| `param show <name>` | Show a parameter value |
| `param set <name> <value>` | Set a parameter |
| `param reset_all` | Reset all parameters to defaults |
| `param save` | Save parameters to flash |
| `dmesg` | Show kernel/driver messages |
| `listener <topic>` | Listen to a uORB topic |
| `<module> status` | Show module status |
| `<module> start` | Start a module |
| `<module> stop` | Stop a module |
| `free` | Show memory usage |
| `ps` | Show running processes |
| `reboot` | Reboot the board |

## Steps

1. Verify `/dev/ttyACM0` exists
2. If `$ARGUMENTS` is empty, ask user what command to run or suggest `ver all`
3. Try connecting and sending `\r\r\r` to trigger NSH
4. Send the command terminated with `\r`
5. Report the command output or any connection issues

## Troubleshooting

- **No /dev/ttyACM***: Board not connected or USB not initialized. Run `/board-status`
- **No response to `\r\r\r`**: `SYS_USB_AUTO` may be set to 2 (MAVLink). Use MAVLink shell method or set `SYS_USB_AUTO=1` and reboot
- **Binary garbage**: Port is in MAVLink mode. Use `Tools/mavlink_shell.py` instead
- **Permission denied**: User may not be in `dialout` group. Run `sudo usermod -aG dialout $USER`
