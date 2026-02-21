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
| _(empty)_ | Full workflow: enable logging, prompt user to run, download latest log, analyze |
| `download` | Download the latest `.ulg` from the board only |
| `analyze <file>` | Analyze a local `.ulg` file only (skip board interaction) |
| `topics <t1,t2,...>` | Enable logging then filter analysis to specific topics |

## Step 1 — Enable Logging

Use MCP `nsh_command` to configure logging parameters:

```
param set SDLOG_MODE 2
param set SDLOG_PROFILE 3
param save
reboot
```

Key `SDLOG_MODE` values: 0=disabled, 1=armed only, 2=always, 3=from boot.

`SDLOG_PROFILE` is a bitmask: 1=default, 2=estimator replay, 4=thermal cal, 8=sysid, 16=high-rate, 32=debug.

After reboot, verify with `logger status` — should show `running` and a log path.

## Step 2 — Capture

Tell the user:
```
[ulog] Logging is active. Reproduce the behavior you want to debug, then come back.
[ulog] When ready, run: /ulog download
```

## Step 3 — Download Log

Find the latest log on the board via NSH:

```
ls /fs/microsd/log
ls /fs/microsd/log/<latest-date-dir>
```

Download using MAVLink FTP via pymavlink:

```bash
python3 -m pymavlink.tools.mavftp --master serial:/dev/ttyACM0:57600 \
  get /fs/microsd/log/<date>/<file>.ulg /tmp/px4_ulogs/<file>.ulg
```

If pymavlink FTP isn't available, remove the SD card and copy directly.

## Step 4 — Analyze

Install pyulog if needed: `pip install pyulog`

```bash
# Quick summary (duration, dropouts, topics, message counts)
ulog_info <file.ulg>

# Extract topics to CSV
ulog_extract <file.ulg> -o /tmp/px4_csv/

# Show parameters
ulog_params <file.ulg>

# Show PX4_INFO/WARN/ERR log messages with timestamps
ulog_messages <file.ulg>
```

For deeper analysis, use pyulog's Python API:

```python
from pyulog import ULog
log = ULog("file.ulg")
print(f"Duration: {log.last_timestamp / 1e6:.1f}s")
print(f"Dropouts: {len(log.dropouts)}")
for d in log.data_list:
    count = len(d.data['timestamp'])
    print(f"  {d.name}[{d.multi_id}]: {count} msgs")
```

## Troubleshooting

| Problem | Fix |
|---------|-----|
| `logger status` not running | Check SD card inserted; `SDLOG_MODE != 0` |
| No SD card detected | `ls /fs/microsd` — if empty, SD missing or corrupt |
| FTP download fails | Use QGroundControl Log Download, or remove SD card |
| `pyulog` not found | `pip install pyulog` |
| Log file is 0 bytes | Check `dmesg` for SD errors |
| Dropouts in log | CPU overload or slow SD — reduce `SDLOG_PROFILE` bits |
