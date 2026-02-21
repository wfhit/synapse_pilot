---
name: upload
description: Upload firmware to a connected PX4 board via USB
argument-hint: [board-shortname]
---

# Upload Firmware

Upload a built firmware `.px4` file to a connected board via USB bootloader.

## Board Detection

Identify the connected board by checking USB devices:

```bash
lsusb | grep -iE "1b8c|3163"
ls -la /dev/ttyACM* 2>/dev/null
```

Known VID:PID mappings:

| VID:PID | Board |
|---------|-------|
| `1b8c:0036` | NXT-Dual (nxt-front / nxt-rear) — app mode |
| `1b8c:004b` | NXT-Dual (nxt-front / nxt-rear) — bootloader mode |
| `3163:004c` | CUAV X7Plus-WL (cuav-wl) |

## Firmware File Location

Find the firmware file based on the board target:

| Board | Firmware Path |
|-------|---------------|
| `nxt-front` | `build/wheel_loader_nxt-dual-wl-front_default/*.px4` |
| `nxt-rear` | `build/wheel_loader_nxt-dual-wl-rear_default/*.px4` |
| `cuav-wl` | `build/wheel_loader_cuav-x7plus-wl_default/*.px4` |
| `holybro` | `build/wheel_loader_holybro-v6xrt-wl_default/*.px4` |

If `$ARGUMENTS` specifies a board shortname, use it. Otherwise, auto-detect from the connected board's VID:PID.

## Upload Command

```bash
python3 Tools/px_uploader.py --port /dev/ttyACM0 <firmware-path>
```

The uploader automatically sends a reboot-to-bootloader command over MAVLink/serial, then programs via the bootloader protocol.

## Steps

1. Log: `[upload] Scanning USB for connected wheel loader boards...`
2. Check which board is connected via USB (lsusb + /dev/ttyACM*); log detected board and VID:PID
3. **Check NSH is responsive** before uploading — use the MCP NSH tool or `nsh_client.py` to send a `ver all` command with a 10s timeout:
   - If NSH responds: log `[upload] NSH responsive — proceeding with upload`
   - If NSH times out or fails: log `[upload] ✗ NSH not responding — please repower the board, then run /upload again` and **stop** (do not attempt upload)
4. Resolve the firmware `.px4` file path (from argument or auto-detect); log: `[upload] Firmware: <path> (<size> bytes)`
5. Verify the `.px4` file exists; if not, log `[upload] ✗ Firmware not found — run /build <board> first` and stop
6. Log: `[upload] Starting upload to <port> at <timestamp>...`
7. Run `px_uploader.py` with a 2-minute timeout; stream its output so erase/program progress is visible
8. Log each phase as it appears in uploader output: `[upload] Erasing...`, `[upload] Programming... <N>%`, `[upload] Verifying...`
9. On completion: Log `[upload] ✓ Upload complete — elapsed <duration>s` or `[upload] ✗ Upload failed — exit code <N>`
10. Wait 10 seconds, then check if USB re-enumerates

## Post-Upload Verification

After the board reboots, check:
```bash
sleep 10
lsusb | grep -iE "1b8c|3163"
ls -la /dev/ttyACM* 2>/dev/null
```

Log: `[upload] Board re-enumerated as <VID:PID> on <port>` or `[upload] ✗ Board did not re-enumerate — check USB cable`
