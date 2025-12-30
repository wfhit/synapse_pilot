# WK2132 UART Test Utility

## Overview

The WK2132 UART test utility provides comprehensive testing capabilities for the WK2132 I2C-to-UART bridge used in wheel loader serial communication systems. This command-line tool offers multiple test modes to validate UART functionality, data integrity, and performance.

## Features

- **Basic connectivity testing** - Verify device opens and basic I/O works
- **Loopback testing** - Hardware-based data integrity validation
- **Echo testing** - External device communication verification
- **Pattern transmission** - Continuous data streaming for analysis
- **Configurable parameters** - Baud rate, data format, test duration
- **Verbose diagnostics** - Detailed error reporting and progress tracking

## Usage

```bash
wk2132_test <command> [options]
```

### Commands

| Command | Description |
|---------|-------------|
| `basic` | Run basic connectivity test |
| `loopback` | Run loopback test (requires TX-RX jumper) |
| `echo` | Run echo test (requires external echo device) |
| `pattern` | Send test pattern |
| `continuous` | Continuous transmission test |

### Options

| Option | Description | Default | Range |
|--------|-------------|---------|-------|
| `-d <device>` | Serial device path | `/dev/ttyS10` | - |
| `-b <baud>` | Baud rate | `9600` | 1200-921600 |
| `-t <seconds>` | Test duration | `10` | 1-3600 |
| `-p <pattern>` | Test pattern | `0` | 0-3 |
| `-v` | Verbose output | `false` | - |
| `-c` | Continuous mode | `false` | - |
| `-D <bits>` | Data bits | `8` | 5-8 |
| `-S <bits>` | Stop bits | `1` | 1-2 |
| `-P` | Enable even parity | `false` | - |

### Test Patterns

| Pattern | Description | Data |
|---------|-------------|------|
| `0` | Incremental | `0x00, 0x01, 0x02, ..., 0xFF, 0x00...` |
| `1` | Alternating 0x55 | `0x55, 0x55, 0x55...` (binary: 01010101) |
| `2` | Alternating 0xAA | `0xAA, 0xAA, 0xAA...` (binary: 10101010) |
| `3` | Random | Pseudo-random data |

## Test Examples

### 1. Basic Connectivity Test

Test basic UART functionality and device accessibility.

```bash
# Default test
wk2132_test basic

# Custom device and baud rate
wk2132_test basic -d /dev/ttyS10 -b 115200

# With verbose output
wk2132_test basic -d /dev/ttyS10 -b 38400 -v
```

**Expected Output:**
```
INFO  [wk2132_test] Starting basic connectivity test on /dev/ttyS10 at 9600 baud
INFO  [wk2132_test] Successfully wrote 21 bytes
INFO  [wk2132_test] No data received (timeout)
INFO  [wk2132_test] Basic connectivity test completed
```

### 2. Loopback Test

Requires physical jumper wire connecting TX to RX pins.

```bash
# Basic loopback test
wk2132_test loopback -d /dev/ttyS10 -b 9600 -t 10

# High-speed test with verbose output
wk2132_test loopback -d /dev/ttyS10 -b 115200 -t 30 -p 0 -v

# Pattern-based test
wk2132_test loopback -d /dev/ttyS10 -b 38400 -t 15 -p 1

# Maximum speed test
wk2132_test loopback -d /dev/ttyS10 -b 921600 -t 5 -p 3 -v
```

**Hardware Setup:**
- Connect TX pin to RX pin with jumper wire
- Ensure proper ground connection

**Expected Output (Success):**
```
INFO  [wk2132_test] Starting loopback test on /dev/ttyS10 (ensure TX-RX are connected)
INFO  [wk2132_test] Running loopback test for 10 seconds...
INFO  [wk2132_test] Completed 100 tests, 0 errors
INFO  [wk2132_test] Completed 200 tests, 0 errors
INFO  [wk2132_test] Loopback test completed: 847 tests, 0 errors (100.00% success rate)
```

**Expected Output (Failure):**
```
INFO  [wk2132_test] Starting loopback test on /dev/ttyS10 (ensure TX-RX are connected)
INFO  [wk2132_test] Running loopback test for 10 seconds...
ERROR [wk2132_test] Read size mismatch: expected 15, got 0
ERROR [wk2132_test] Read size mismatch: expected 8, got 0
INFO  [wk2132_test] Loopback test completed: 245 tests, 245 errors (0.00% success rate)
```

### 3. Echo Test

Requires external device that echoes back received data.

```bash
# Basic echo test
wk2132_test echo -d /dev/ttyS10 -b 9600

# High-speed echo test
wk2132_test echo -d /dev/ttyS10 -b 115200

# Custom serial format
wk2132_test echo -d /dev/ttyS10 -b 38400 -D 7 -S 1 -P
```

**Hardware Setup:**
1. Connect WK2132 UART to PC via USB-to-Serial adapter
2. Open terminal program (PuTTY, minicom, screen) on PC
3. Configure terminal to echo received characters
4. Run the test

**Expected Output:**
```
INFO  [wk2132_test] Starting echo test on /dev/ttyS10 (requires external echo device)
INFO  [wk2132_test] Sending 4 test messages...
INFO  [wk2132_test] Sent: Hello WK2132

INFO  [wk2132_test] Echo: Hello WK2132

INFO  [wk2132_test] Sent: Echo Test 123

INFO  [wk2132_test] Echo: Echo Test 123

INFO  [wk2132_test] Echo test completed
```

### 4. Pattern Test

Continuous data transmission for oscilloscope/analyzer testing.

```bash
# Incremental pattern
wk2132_test pattern -d /dev/ttyS10 -b 9600 -p 0 -t 10

# Alternating 0x55 pattern (good for clock recovery)
wk2132_test pattern -d /dev/ttyS10 -b 115200 -p 1 -t 30

# Alternating 0xAA pattern
wk2132_test pattern -d /dev/ttyS10 -b 38400 -p 2 -t 5

# Random pattern (stress test)
wk2132_test pattern -d /dev/ttyS10 -b 921600 -p 3 -t 60
```

**Expected Output:**
```
INFO  [wk2132_test] Starting pattern test on /dev/ttyS10
INFO  [wk2132_test] Sending pattern 1 for 10 seconds...
INFO  [wk2132_test] Pattern test completed: 64000 bytes sent
```

### 5. Continuous Test

Runs pattern test indefinitely until interrupted.

```bash
# Continuous incremental pattern
wk2132_test continuous -d /dev/ttyS10 -b 115200 -p 0

# Continuous high-speed 0x55 pattern
wk2132_test continuous -d /dev/ttyS10 -b 921600 -p 1
```

**Note:** Use Ctrl+C to stop continuous tests.

## Advanced Configuration Examples

### Custom Serial Formats

```bash
# 7 data bits, 2 stop bits, even parity
wk2132_test basic -d /dev/ttyS10 -b 9600 -D 7 -S 2 -P

# 5 data bits, 1 stop bit, no parity
wk2132_test loopback -d /dev/ttyS10 -b 19200 -D 5 -S 1 -t 30

# 8 data bits, 2 stop bits, no parity
wk2132_test pattern -d /dev/ttyS10 -b 460800 -D 8 -S 2 -p 2 -t 60
```

### Performance Testing

```bash
# High-speed loopback stress test
wk2132_test loopback -d /dev/ttyS10 -b 921600 -t 300 -v

# Long-duration pattern test
wk2132_test pattern -d /dev/ttyS10 -b 230400 -p 3 -t 3600

# Comprehensive format test
for baud in 9600 38400 115200 460800 921600; do
    wk2132_test loopback -d /dev/ttyS10 -b $baud -t 10 -v
done
```

## Troubleshooting

### Common Issues

1. **Device not found**
   ```bash
   # Check if device exists
   ls -l /dev/ttyS*

   # Test with different device
   wk2132_test basic -d /dev/ttyS11
   ```

2. **Permission denied**
   ```bash
   # Check device permissions
   ls -l /dev/ttyS10

   # May need to run as root or add user to dialout group
   ```

3. **Loopback test failures**
   ```bash
   # Verify hardware connection
   wk2132_test basic -d /dev/ttyS10 -v

   # Try lower baud rate
   wk2132_test loopback -d /dev/ttyS10 -b 9600 -t 5 -v
   ```

4. **Inconsistent results**
   ```bash
   # Test at different speeds
   for speed in 9600 38400 115200; do
       wk2132_test loopback -d /dev/ttyS10 -b $speed -t 5
   done
   ```

### Diagnostic Commands

```bash
# Quick connectivity check
wk2132_test basic -d /dev/ttyS10

# Comprehensive loopback test
wk2132_test loopback -d /dev/ttyS10 -b 115200 -t 60 -v

# Signal quality test (use with oscilloscope)
wk2132_test pattern -d /dev/ttyS10 -b 115200 -p 1 -t 30
```

## Hardware Requirements

### Loopback Test Setup
```
WK2132 UART
┌─────────────┐
│ TX ●────●RX │  ← Jumper wire connection
│             │
│         GND │
└─────────────┘
```

### Echo Test Setup
```
WK2132 UART          USB-Serial Adapter          PC
┌─────────────┐      ┌─────────────────┐      ┌─────────┐
│ TX ●────────┼──────┼● RX             │      │Terminal │
│ RX ●────────┼──────┼● TX    USB ●────┼──────┼● Echo   │
│ GND●────────┼──────┼● GND            │      │  Mode   │
└─────────────┘      └─────────────────┘      └─────────┘
```

## Error Codes

| Return Code | Description |
|-------------|-------------|
| `0` | Success |
| `1` | Invalid arguments or usage |
| `-1` | Test failure (device error, data mismatch, etc.) |

## Development Notes

### File Structure
```
src/systemcmds/wk2132_test/
├── CMakeLists.txt      # Build configuration
├── Kconfig            # Configuration options
├── wk2132_test.cpp    # Main implementation
└── README.md          # This documentation
```

### Test Algorithm Details

The loopback test uses the following algorithm:
1. Generate random-sized data packets (1-32 bytes)
2. Fill with selected test pattern
3. Write to UART TX
4. Wait 10ms for loopback propagation
5. Read expected number of bytes from UART RX
6. Compare transmitted vs received data
7. Track errors and success rate

### Performance Characteristics

| Baud Rate | Theoretical Throughput | Typical Test Rate |
|-----------|----------------------|-------------------|
| 9600 | 960 bytes/s | ~850 bytes/s |
| 38400 | 3840 bytes/s | ~3400 bytes/s |
| 115200 | 11520 bytes/s | ~10200 bytes/s |
| 921600 | 92160 bytes/s | ~81500 bytes/s |

## Integration

This test utility is integrated into the PX4 wheel loader firmware and available via the NSH (NuttShell) command line. It's automatically built when `CONFIG_SYSTEMCMDS_WK2132_TEST=y` is enabled in the board configuration.

## License

Copyright (c) 2025 PX4 Development Team. All rights reserved.

This utility is part of the PX4 autopilot project and is licensed under the BSD 3-Clause License.
