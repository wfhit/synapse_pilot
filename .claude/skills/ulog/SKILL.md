---
name: ulog
description: Enable ulog, capture a log from SD card, download it, and analyze the result
argument-hint: [topic-filter | "download" | "analyze <file>"]
---

# ULog Debug Workflow

End-to-end ulog debugging: enable logging, capture to SD card, download the `.ulg` file, and analyze it with pyulog.

## Argument Modes

| Argument | Action |
|----------|--------|
| _(empty)_ | Full workflow: enable logging → prompt user to arm/run → download latest log → analyze |
| `download` | Download the latest `.ulg` from the board only |
| `analyze <file>` | Analyze a local `.ulg` file only (skip board interaction) |
| `topics <t1,t2,...>` | Enable logging then filter analysis to specific topics |

---

## Step 1 — Enable Logging

Connect to NSH (see `/nsh` skill) and configure logging parameters:

```
param set SDLOG_MODE 2        # 2 = always log while running (not just when armed)
param set SDLOG_PROFILE 3     # 1=default topics, 2=estimator replay, bitmask OR them
param save
```

Key `SDLOG_MODE` values:
| Value | Meaning |
|-------|---------|
| 0 | Disabled |
| 1 | Log when armed |
| 2 | Log always (from start) |
| 3 | Log from boot |

Key `SDLOG_PROFILE` bits (bitmask):
| Bit | Topics |
|-----|--------|
| 0 (1) | Default set |
| 1 (2) | Estimator replay |
| 2 (4) | Thermal calibration |
| 3 (8) | System identification |
| 4 (16) | High-rate |
| 5 (32) | Debug |

After setting params, reboot to apply:

```
reboot
```

Verify logger is running after reboot:

```
logger status
```

Expected output contains `running` and a log file path like `/fs/microsd/log/YYYY-MM-DD/HH_MM_SS.ulg`.

---

## Step 2 — Capture Log

Instruct the user:

```
[ulog] Logging is active. Reproduce the behavior you want to debug, then come back.
[ulog] When ready, run: /ulog download
```

If running the full workflow, wait for user confirmation before proceeding to download.

---

## Step 3 — Download Log via MAVLink FTP

Find the latest log file on the board via NSH:

```
ls /fs/microsd/log
ls /fs/microsd/log/<latest-date-dir>
```

Then download using pymavlink's `mavftp`:

```python
#!/usr/bin/env python3
"""Download latest ULog from PX4 board via MAVLink FTP."""
import subprocess, sys, os, glob, time

port = "/dev/ttyACM0"
baud = 57600
out_dir = "/tmp/px4_ulogs"
os.makedirs(out_dir, exist_ok=True)

# Use mavftp to list and download
result = subprocess.run(
    ["python3", "-m", "pymavlink.tools.mavftp",
     "--master", f"serial:{port}:{baud}",
     "list", "/fs/microsd/log"],
    capture_output=True, text=True, timeout=30
)
print(result.stdout)
```

Alternatively, use `mavlink ftp get` directly from NSH if pymavlink is unavailable:

```
# On NSH — copy log to a known path first if needed
ls /fs/microsd/log
```

Then use QGroundControl or `Tools/mavlink_shell.py` with FTP commands.

**Preferred download method** — run this Python script via Bash:

```python
#!/usr/bin/env python3
"""
Download latest .ulg from PX4 board.
Requires: pip install pymavlink
"""
import os, sys, time, glob
from pymavlink import mavutil

PORT = "/dev/ttyACM0"
BAUD = 57600
OUT_DIR = "/tmp/px4_ulogs"
os.makedirs(OUT_DIR, exist_ok=True)

print(f"[ulog] Connecting to {PORT}...")
mav = mavutil.mavlink_connection(f"serial:{PORT}:{BAUD}", autoreconnect=True)
mav.wait_heartbeat(timeout=10)
print(f"[ulog] Heartbeat from sysid={mav.target_system}")

ftp = mavutil.mavftp(mav)

# List log directories
dirs = ftp.list("/fs/microsd/log")
if not dirs:
    print("[ulog] No log directories found on SD card")
    sys.exit(1)

# Pick latest date directory
date_dirs = sorted([d for d in dirs if d.startswith("2")], reverse=True)
latest_dir = f"/fs/microsd/log/{date_dirs[0]}"
print(f"[ulog] Latest log dir: {latest_dir}")

files = ftp.list(latest_dir)
ulg_files = sorted([f for f in files if f.endswith(".ulg")], reverse=True)
if not ulg_files:
    print("[ulog] No .ulg files found")
    sys.exit(1)

remote_path = f"{latest_dir}/{ulg_files[0]}"
local_path = os.path.join(OUT_DIR, ulg_files[0])
print(f"[ulog] Downloading {remote_path} -> {local_path}")

ftp.get(remote_path, local_path)
print(f"[ulog] Download complete: {local_path}")
```

Save as `/tmp/px4_download_ulog.py` and run:

```bash
python3 /tmp/px4_download_ulog.py
```

---

## Step 4 — Analyze with pyulog

Install pyulog if needed:

```bash
pip install pyulog
```

### 4a — File Info

```bash
ulog_info <file.ulg>
```

Shows: log duration, dropouts, logged topics, message counts.

### 4b — Extract to CSV

```bash
ulog_extract <file.ulg> -o /tmp/px4_csv/
```

Creates one CSV per topic, e.g. `vehicle_status_0.csv`, `sensor_accel_0.csv`.

### 4c — Show Parameters

```bash
ulog_params <file.ulg>
```

### 4d — Show Log Messages

```bash
ulog_messages <file.ulg>
```

Shows all `PX4_INFO/WARN/ERR` messages with timestamps.

### 4e — Python Analysis Script

For deeper analysis, run this inline:

```python
#!/usr/bin/env python3
"""Analyze a PX4 ULog file and print a summary."""
import sys
from pyulog import ULog
from pyulog.core import ULog

if len(sys.argv) < 2:
    print("Usage: python3 analyze_ulog.py <file.ulg> [topic1 topic2 ...]")
    sys.exit(1)

path = sys.argv[1]
filter_topics = sys.argv[2:] if len(sys.argv) > 2 else None

print(f"[ulog] Loading {path}...")
log = ULog(path, filter_topics)

# Header
print(f"\n=== ULog Summary ===")
print(f"Duration : {log.last_timestamp / 1e6:.1f}s")
print(f"Dropouts : {len(log.dropouts)} ({sum(d.duration for d in log.dropouts)/1000:.0f}ms total)")

# Parameters
print(f"\n--- Parameters ({len(log.initial_parameters)}) ---")
for k, v in sorted(log.initial_parameters.items()):
    print(f"  {k} = {v}")

# Topics
print(f"\n--- Logged Topics ---")
for d in sorted(log.data_list, key=lambda x: x.name):
    count = len(d.data['timestamp'])
    duration_s = (d.data['timestamp'][-1] - d.data['timestamp'][0]) / 1e6 if count > 1 else 0
    rate = count / duration_s if duration_s > 0 else 0
    print(f"  {d.name}[{d.multi_id}]  {count} msgs  ~{rate:.0f} Hz  fields: {list(d.data.keys())}")

# Log messages
print(f"\n--- Log Messages ---")
for m in log.logged_messages:
    level = {0: 'EMERG', 1: 'ALERT', 2: 'CRIT', 3: 'ERR',
             4: 'WARN', 5: 'NOTICE', 6: 'INFO', 7: 'DEBUG'}.get(m.log_level, '?')
    ts = m.timestamp / 1e6
    print(f"  [{ts:8.3f}s] {level:6s}  {m.message}")
```

Save as `/tmp/analyze_ulog.py` and run:

```bash
python3 /tmp/analyze_ulog.py <file.ulg> [topic1 topic2 ...]
```

---

## Full Workflow Steps

1. Check board is connected: `ls /dev/ttyACM*`
2. Connect via NSH and set `SDLOG_MODE=2`, `SDLOG_PROFILE=1`, save params, reboot
3. Verify `logger status` shows running
4. Prompt user to reproduce the issue
5. Download latest `.ulg` via MAVLink FTP to `/tmp/px4_ulogs/`
6. Run `ulog_info <file>` for quick summary
7. Run `ulog_messages <file>` to show all log messages
8. Run the Python analysis script for full topic/parameter dump
9. If `$ARGUMENTS` contains specific topic names, filter the CSV extract to those topics

## Troubleshooting

| Problem | Fix |
|---------|-----|
| `logger status` shows not running | Check SD card is inserted; check `SDLOG_MODE != 0` |
| No SD card detected | Run `ls /fs/microsd` on NSH — if empty, SD card missing or corrupt |
| FTP download fails | Use QGroundControl Log Download, or remove SD card and copy directly |
| `pyulog` not found | `pip install pyulog` or `conda install -c conda-forge pyulog` |
| Log file is 0 bytes | Logger started but stopped immediately — check `dmesg` for SD errors |
| Dropouts in log | CPU overload or SD card too slow — reduce `SDLOG_PROFILE` bits |
