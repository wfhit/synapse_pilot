---
name: board-status
description: Show connected PX4 board information and status
---

# Board Status

Check which PX4 boards are connected via USB, identify them, and query firmware version.

## Steps

1. List USB devices and filter for known PX4 boards:

```bash
lsusb | grep -iE "1b8c|3163|3162|3643"
```

2. Check for serial devices:

```bash
ls -la /dev/ttyACM* /dev/ttyUSB* 2>/dev/null
```

3. Get detailed USB device info to map each `/dev/ttyACM*` to a VID:PID:

```bash
for dev in /dev/ttyACM*; do
  udevadm info -q property -n "$dev" 2>/dev/null | grep -E "ID_VENDOR_ID|ID_MODEL_ID|ID_SERIAL"
done
```

4. For each board in **application mode**, query firmware version via MCP `nsh_command`:
   - Send `ver all` with a 10s timeout on the board's ACM port
   - Extract: firmware version, board type, git hash
   - If NSH doesn't respond, report "NSH not responding" for that board

## Board Identification

| VID:PID | Board | Mode |
|---------|-------|------|
| `3162:004b` | NXT-Dual (nxt-front / nxt-rear) | Bootloader |
| `1b8c:0036` | NXT-Dual (nxt-front / nxt-rear) | Application |
| `3163:004c` | CUAV X7Plus-WL (cuav-wl) | Bootloader |
| `3163:004d` | CUAV X7Plus-WL (cuav-wl) | Application |
| `3643:001d` | Holybro V6X-RT-WL (holybro) | Bootloader |
| `3643:001e` | Holybro V6X-RT-WL (holybro) | Application |

**Note:** NXT-Dual front and rear share the same VID:PID. Distinguish them by
running `ver all` — the board name in the firmware version string differs.

## Output Format

```
Connected Boards:
  Port          VID:PID      Board              Mode         Firmware
  /dev/ttyACM0  1b8c:0036    NXT-Dual           Application  v0.1.0-dirty (abc1234)
  /dev/ttyACM1  3163:004c    CUAV X7Plus-WL     Application  v0.1.0 (def5678)
```

If no boards are found, report that and suggest checking USB cable connections.

## Additional Diagnostics

If a board is detected by lsusb but no `/dev/ttyACM*` appears, check:

```bash
dmesg | tail -20 | grep -i -E "usb|cdc|acm|tty"
```

Report any USB errors or driver issues found in dmesg.
