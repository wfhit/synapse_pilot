---
name: board-status
description: Show connected PX4 board information and status
---

# Board Status

Check which PX4 boards are connected via USB and report their status.

## Steps

1. List USB devices and filter for known PX4 boards:

```bash
lsusb | grep -iE "matek|1b8c|cuav|3163|px4|stm|26ac"
```

2. Check for serial devices:

```bash
ls -la /dev/ttyACM* /dev/ttyUSB* 2>/dev/null
```

3. Get detailed USB device info:

```bash
for dev in /dev/ttyACM*; do
  udevadm info -q property -n "$dev" 2>/dev/null | grep -E "ID_VENDOR|ID_MODEL|ID_SERIAL"
done
```

## Board Identification

| VID:PID | Board | Mode |
|---------|-------|------|
| `1b8c:004b` | NXT-Dual | Bootloader |
| `1b8c:0036` | NXT-Dual | Application |
| `3163:004c` | CUAV X7Pro / X7Plus-WL | Application |
| `26ac:0011` | PX4 FMU v5 | Application |

## Output Format

Report a summary table:

```
Connected Boards:
  Port          VID:PID      Board              Mode
  /dev/ttyACM0  1b8c:0036    NXT-Dual           Application
  /dev/ttyACM1  3163:004c    CUAV X7Plus-WL     Application
```

If no boards are found, report that and suggest checking USB cable connections.

## Additional Diagnostics

If a board is detected by lsusb but no /dev/ttyACM* appears, check:

```bash
dmesg | tail -20 | grep -i -E "usb|cdc|acm|tty"
```

Report any USB errors or driver issues found in dmesg.
