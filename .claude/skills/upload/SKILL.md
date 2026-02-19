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
lsusb | grep -iE "matek|1b8c|cuav|3163|px4|stm"
ls -la /dev/ttyACM* 2>/dev/null
```

Known VID:PID mappings:

| VID:PID | Board |
|---------|-------|
| `1b8c:0036` | NXT-Dual (app mode) |
| `1b8c:004b` | NXT-Dual (bootloader mode) |
| `3163:004c` | CUAV X7Pro / CUAV X7Plus-WL |
| `26ac:*` | PX4 generic |

## Firmware File Location

Find the firmware file based on the board target:

| Board | Firmware Path |
|-------|---------------|
| `nxt-front` | `build/wheel_loader_nxt-dual-wl-front_default/*.px4` |
| `nxt-rear` | `build/wheel_loader_nxt-dual-wl-rear_default/*.px4` |
| `cuav-wl` | `build/wheel_loader_cuav-x7plus-wl_default/*.px4` |
| `holybro` | `build/wheel_loader_holybro-v6xrt-wl_default/*.px4` |
| `nxt-dual` | `build/hkust_nxt-dual_default/*.px4` |
| `cuav-x7pro` | `build/cuav_x7pro_default/*.px4` |
| `cuav-nora` | `build/cuav_nora_default/*.px4` |

If `$ARGUMENTS` specifies a board shortname, use it. Otherwise, auto-detect from the connected board's VID:PID.

## Upload Command

```bash
python3 Tools/px_uploader.py --port /dev/ttyACM0 <firmware-path>
```

The uploader automatically sends a reboot-to-bootloader command over MAVLink/serial, then programs via the bootloader protocol.

## Steps

1. Check which board is connected via USB (lsusb + /dev/ttyACM*)
2. Resolve the firmware `.px4` file path (from argument or auto-detect)
3. Verify the `.px4` file exists; if not, suggest running `/build` first
4. Run `px_uploader.py` with a 2-minute timeout
5. After upload completes, wait 10 seconds and check if USB re-enumerates
6. Report success or failure

## Post-Upload Verification

After the board reboots, check:
```bash
sleep 10
lsusb | grep -iE "matek|1b8c|cuav|3163"
ls -la /dev/ttyACM* 2>/dev/null
```

Report whether the board re-enumerated and which VID:PID was detected.
