---
name: test-uart
description: Test UART connectivity between two boards
argument-hint: <board1:port> <board2:port>
---

# Cross-Board UART Connectivity Test

Test bidirectional UART connectivity between two connected boards using `gen_serial_test`.

## Prerequisites

- Both boards connected via USB (`/dev/ttyACM0`, `/dev/ttyACM1`)
- `gen_serial_test` module enabled on both boards (`CONFIG_SYSTEMCMDS_GEN_SERIAL_TEST=y`)
- Physical UART cable connecting the two ports under test

## Board / Port Reference

| Board | Short | Typical test ports |
|-------|-------|--------------------|
| cuav-wl | cuav | `/dev/ttyS10` (WK2132 ch0, Rear NXT), `/dev/ttyS11` (WK2132 ch1, Front NXT) |
| nxt-front | nxt-f | `/dev/ttyS1` (USART1, uORB proxy port) |
| nxt-rear | nxt-r | `/dev/ttyS2` (USART1, uORB proxy port) |

## Steps

1. Identify connected boards:
   ```bash
   lsusb | grep -iE "1b8c|3162|3163"
   ls -la /dev/ttyACM* 2>/dev/null
   ```
   Map each `/dev/ttyACM*` to a board using VID:PID (`1b8c:0036` = NXT app, `3162:004b` = NXT bootloader, `3163:004c` = CUAV).

2. Parse `$ARGUMENTS` for two endpoints in format `<board>:<port>`. Examples:
   - `nxt-f:ttyS1 cuav:ttyS11`
   - `cuav:ttyS10 cuav:ttyS11` (loopback between WK2132 channels)

   If arguments are missing, ask the user which ports to test.

3. Determine which `/dev/ttyACM*` corresponds to each board (from step 1).

4. **Test direction A→B**: Use MCP `nsh_command` on board A's ACM port:
   ```
   gen_serial_test send -d /dev/<portA> -b 115200 -t 10 &
   ```
   Then on board B's ACM port:
   ```
   gen_serial_test receive -d /dev/<portB> -b 115200 -t 12
   ```
   Wait for receive to complete. Report bytes received.

5. **Test direction B→A**: Reverse the roles and repeat.

6. Report results:
   ```
   [test-uart] Results:
     A→B (<boardA>:<portA> → <boardB>:<portB>): <N> bytes received ✓ / 0 bytes ✗
     B→A (<boardB>:<portB> → <boardA>:<portA>): <N> bytes received ✓ / 0 bytes ✗
   ```

## Troubleshooting

| Symptom | Likely cause |
|---------|-------------|
| 0 bytes both directions | No cable, wrong ports, or baud mismatch |
| 0 bytes one direction only | TX/RX swap on one end, or broken wire |
| Low byte count | Baud mismatch or noise on long cable |
| `gen_serial_test: command not found` | Module not enabled in `.px4board` |
