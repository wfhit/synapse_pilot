---
name: board-diff
description: Compare configuration between two board variants
argument-hint: <board1> <board2>
---

# Board Config Diff

Compare the configuration files between two board variants to understand differences.

## Board Path Mapping

| Shortname | Board Path |
|-----------|------------|
| `nxt-front` | `boards/wheel_loader/nxt-dual-wl-front` |
| `nxt-rear` | `boards/wheel_loader/nxt-dual-wl-rear` |
| `cuav-wl` | `boards/wheel_loader/cuav-x7plus-wl` |
| `holybro` | `boards/wheel_loader/holybro-v6xrt-wl` |

Parse `$ARGUMENTS` to extract two board shortnames. If only one is given, ask for the second. If none given, suggest common comparisons.

## Files to Compare

For each board pair, compare these files (if they exist in both):

1. **Board config header**: `src/board_config.h`
   - GPIO definitions, USB config, hardware defines
2. **NuttX defconfig**: `nuttx-config/nsh/defconfig`
   - Peripheral enables, UART config, USB settings
3. **Init defaults**: `init/rc.board_defaults`
   - Parameter defaults
4. **Init extras**: `init/rc.board_extras`
   - Module startup sequence
5. **Build config**: `default.px4board`
   - Enabled modules and drivers
6. **Firmware prototype**: `firmware.prototype`
   - Board ID, flash size

## Steps

1. Resolve both board shortnames to paths
2. For each file pair, run `diff` and show meaningful differences
3. Highlight key differences:
   - Different board_id values
   - Different USB VID:PID
   - GPIO pin assignments that differ
   - Modules enabled in one but not the other
   - Parameter defaults that differ
   - UART/peripheral configuration differences

## Output Format

Present a structured summary:

```
=== Board Comparison: nxt-front vs nxt-rear ===

Board ID:        1013 vs 1013 (same)
USB PID:         0x0036 vs 0x0037

GPIO Differences:
  - DRV8701_RIGHT_DIR: PE13 vs PE13 (same)
  - BOOM_CONTROL:      N/A vs PD14

Module Differences:
  nxt-front only: tilt_control
  nxt-rear only:  boom_control, load_lamp_controller

Parameter Differences:
  TILT_CTRL_EN:   1 vs 0
  BOOM_CTRL_EN:   0 vs 1
```

Then show the full diffs for each file if the user wants details.
