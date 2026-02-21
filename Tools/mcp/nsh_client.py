"""NSH serial communication library.

Adapted from Tools/HIL/{run_nsh_cmd.py, monitor_firmware_upload.py, run_tests.py}.
Returns results and raises exceptions instead of calling sys.exit().

NOTE: Some STM32H7 boards (e.g. CUAV X7Plus-WL) have a USB CDC ACM bug where
the first byte of each USB bulk OUT transfer is delayed and delivered after the
rest of the packet.  All command writes are prefixed with a space to absorb this
(NSH ignores leading whitespace).  See commit message for the repro details.
"""

import time
from dataclasses import dataclass

import serial


BAUDRATE = 57600
SERIAL_TIMEOUT = 1  # seconds
INTER_BYTE_TIMEOUT = 1  # seconds


class NshError(Exception):
    """General NSH communication error."""


class NshTimeoutError(NshError):
    """Timeout waiting for NSH response."""


@dataclass
class NshResult:
    output: str
    return_code: int


def _open_serial(port: str) -> serial.Serial:
    return serial.Serial(
        port=port,
        baudrate=BAUDRATE,
        bytesize=serial.EIGHTBITS,
        parity=serial.PARITY_NONE,
        stopbits=serial.STOPBITS_ONE,
        timeout=SERIAL_TIMEOUT,
        xonxoff=False,
        rtscts=False,
        dsrdtr=False,
        inter_byte_timeout=INTER_BYTE_TIMEOUT,
    )


def _drain_pending(ser: serial.Serial, settle: float = 0.3) -> None:
    """Read and discard all pending serial data until quiet for *settle* seconds.

    After sending \\r\\r\\r, NSH may still be emitting prompt lines and ANSI
    escape sequences.  This helper keeps reading until no new data arrives
    for *settle* seconds, ensuring the board-side input buffer is idle before
    the next command is written.
    """
    saved_timeout = ser.timeout
    ser.timeout = 0.05  # short non-blocking reads for draining
    try:
        deadline = time.monotonic() + settle
        while time.monotonic() < deadline:
            chunk = ser.read(256)
            if chunk:
                # Got data — reset the quiet-period timer
                deadline = time.monotonic() + settle
            else:
                time.sleep(0.01)
    finally:
        ser.timeout = saved_timeout
    ser.reset_input_buffer()


def _wait_for_prompt(ser: serial.Serial, timeout: float = 30) -> str:
    """Send carriage returns until we see 'nsh>'. Returns output collected while waiting.

    PX4's cdcacm_autostart scans for 3 consecutive carriage returns (0x0D)
    to trigger NSH when SYS_USB_AUTO=1 (auto-detect mode).
    """
    output_lines = []
    timeout_start = time.monotonic()

    # 3 consecutive \r bytes trigger NSH in auto-detect mode
    ser.write(b"\r\r\r")

    while True:
        line = ser.readline().decode("ascii", errors="ignore")

        if len(line) > 0:
            output_lines.append(line)
            if "nsh>" in line:
                # Drain any remaining prompt output (from the
                # multiple \r's) so stale bytes don't interfere
                # with the next command.
                _drain_pending(ser)
                return "".join(output_lines)
        else:
            if time.monotonic() > timeout_start + timeout:
                raise NshTimeoutError(
                    "Timeout waiting for NSH prompt."
                )
            ser.write(b"\r\r\r")


def run_nsh_command(
    port: str,
    command: str,
    timeout: float = 600,
    ignore_errors: bool = False,
) -> NshResult:
    """Run an NSH command and return the result.

    Based on Tools/HIL/run_nsh_cmd.py:42 do_nsh_cmd().
    """
    ser = _open_serial(port)
    try:
        _wait_for_prompt(ser)

        # Send a single \r to get one clean prompt, then drain it.
        # This guarantees no stale bytes are in-flight on USB before
        # the real command is written.
        ser.write(b"\r")
        _drain_pending(ser, settle=0.2)

        success_cmd = "cmd succeeded!"
        # Leading space absorbs the STM32H7 USB CDC ACM first-byte-delay
        # bug — NSH ignores leading whitespace.
        serial_cmd = ' {0}; echo "{1}"; echo "{2}";\r'.format(
            command, success_cmd, success_cmd
        )
        ser.write(serial_cmd.encode("ascii"))

        # Wait for command echo
        output_lines = []
        echo_timeout_start = time.monotonic()
        echo_timeout = 5

        while True:
            line = ser.readline().decode("ascii", errors="ignore")

            if len(line) > 0:
                if command in line:
                    break
                elif (
                    line.startswith(success_cmd)
                    and len(line) <= len(success_cmd) + 2
                ):
                    # Missed the echo, but command ran and succeeded
                    output_lines.append(line)
                    return NshResult(
                        output="".join(output_lines), return_code=0
                    )
                else:
                    output_lines.append(line)
            else:
                if time.monotonic() > echo_timeout_start + echo_timeout:
                    break

        # Collect command output
        timeout_start = time.monotonic()
        return_code = 0

        while True:
            line = ser.readline().decode("ascii", errors="ignore")

            if len(line) > 0:
                if success_cmd in line:
                    return NshResult(
                        output="".join(output_lines),
                        return_code=return_code,
                    )
                else:
                    if "ERROR " in line and not ignore_errors:
                        return_code = -1

                    output_lines.append(line)

                    if "nsh>" in line or "NuttShell (NSH)" in line:
                        return NshResult(
                            output="".join(output_lines), return_code=1
                        )
            else:
                if time.monotonic() > timeout_start + timeout:
                    return NshResult(
                        output="".join(output_lines)
                        + "\n[Timeout after {0}s]".format(int(timeout)),
                        return_code=-1,
                    )
    finally:
        ser.close()


def monitor_boot(port: str, timeout: float = 180) -> NshResult:
    """Monitor board boot and return the boot log.

    Based on Tools/HIL/monitor_firmware_upload.py:42.
    """
    ser = _open_serial(port)
    # Use longer read timeout for boot monitoring
    ser.timeout = 3
    try:
        output_lines = []
        timeout_start = time.monotonic()
        timeout_newline = time.monotonic()

        while True:
            line = ser.readline().decode("ascii", errors="ignore")

            if len(line) > 0:
                output_lines.append(line)

                if "NuttShell (NSH)" in line or "nsh>" in line:
                    return NshResult(
                        output="".join(output_lines), return_code=0
                    )
            else:
                if time.monotonic() > timeout_start + timeout:
                    return NshResult(
                        output="".join(output_lines)
                        + "\n[Timeout after {0}s]".format(int(timeout)),
                        return_code=-1,
                    )

                # Send carriage returns every 10 seconds to prod the board
                if time.monotonic() - timeout_newline > 10:
                    timeout_newline = time.monotonic()
                    ser.write(b"\r\r\r")
    finally:
        ser.close()


def run_hil_test(
    port: str, test_name: str, timeout: float = 300
) -> NshResult:
    """Run a HIL test and return the result.

    Based on Tools/HIL/run_tests.py:45 do_test().
    """
    ser = _open_serial(port)
    # Use longer read timeout for test monitoring
    ser.timeout = 3
    try:
        _wait_for_prompt(ser)

        ser.write(b"\r")
        _drain_pending(ser, settle=0.2)

        # Leading space absorbs the STM32H7 USB CDC ACM first-byte-delay bug
        cmd = "tests " + test_name
        ser.write(" {0}\r".format(cmd).encode("ascii"))

        # Wait for command echo
        output_lines = []
        echo_timeout_start = time.monotonic()
        echo_timeout = 2

        while True:
            line = ser.readline().decode("ascii", errors="ignore")

            if len(line) > 0:
                if cmd in line:
                    break
            else:
                if time.monotonic() > echo_timeout_start + echo_timeout:
                    break

        # Collect test output, wait for PASSED or FAILED
        timeout_start = time.monotonic()
        timeout_newline = timeout_start

        while True:
            line = ser.readline().decode("ascii", errors="ignore")

            if len(line) > 0:
                output_lines.append(line)

                if test_name + " PASSED" in line:
                    return NshResult(
                        output="".join(output_lines), return_code=0
                    )
                elif test_name + " FAILED" in line:
                    return NshResult(
                        output="".join(output_lines), return_code=1
                    )
            else:
                if time.monotonic() > timeout_start + timeout:
                    return NshResult(
                        output="".join(output_lines)
                        + "\n[Timeout after {0}s]".format(int(timeout)),
                        return_code=-1,
                    )

                # Send carriage return every 30 seconds to keep connection alive
                if time.monotonic() - timeout_newline > 30:
                    ser.write(b"\r")
                    timeout_newline = time.monotonic()
    finally:
        ser.close()
