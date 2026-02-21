# USB CDC ACM First-Byte-Delay Bug Analysis

Date: 2026-02-21
Boards: CUAV X7Plus-WL vs NXT-Dual-WL-Front

## Symptom

On the CUAV X7Plus-WL board, the first byte of each USB bulk OUT transfer is delayed
and delivered after the rest of the packet. For example, sending `ver all\r` results
in the board receiving `er all` followed by `v` arriving late.

The NXT-Dual-WL-Front board does NOT exhibit this bug.

## Root Cause Analysis

### 1. TTY_SIGINT / TTY_SIGTSTP (Primary suspect)

CUAV had `CONFIG_TTY_SIGINT=y` and `CONFIG_TTY_SIGTSTP=y` enabled.
NXT did not.

When enabled, NuttX's TTY layer intercepts every incoming byte in the serial receive
path (`uart_recvchars`) and checks whether it matches Ctrl+C (0x03) or Ctrl+Z (0x1A).
This per-byte scanning happens inside the CDC ACM receive interrupt handler before
data reaches the application-level read buffer.

When a USB bulk OUT packet arrives, the OTG ISR reads it into the CDC ACM request
buffer as a whole packet. The TTY layer then processes bytes one at a time. The SIGINT
check on the first byte can trigger a code path that flushes the line discipline state,
causing the first byte to be buffered separately from the rest.

PX4 doesn't use Ctrl+C/Ctrl+Z signal handling. Module shutdown uses `<module> stop`.

**Fix:** Disabled both options.

### 2. IOB Buffer Pool Exhaustion (Contributing factor)

CUAV used NuttX default IOB pool (typically 8 buffers).
NXT explicitly configured 24 buffers.

IOBs are NuttX's internal packet buffers for passing data between driver layers.
On CUAV, 6 active UART ports compete for IOBs alongside USB CDC ACM. With only
8 buffers, a burst of UART activity (e.g., GPS NMEA + VLA proxy packet arriving
simultaneously) can temporarily exhaust the pool. When CDC ACM tries to allocate
an IOB for an incoming USB packet and the pool is empty, partial delivery can occur.

**Fix:** Added `CONFIG_MM_IOB=y`, `CONFIG_IOB_NBUFFERS=24`, `CONFIG_IOB_NCHAINS=24`.

### 3. AHB Bus Contention from DMA (Contributing factor)

CUAV had ~14 active DMA channels (11 UART + 3 SPI).
NXT had ~3 (2 UART + 1 SPI).

Each DMA channel generates AHB bus transactions that compete with CPU access to the
USB OTG RXFIFO. The STM32H7's DMA controllers share the AHB bus matrix with the CPU.

Low-speed ports (UART7 debug console at 115200, UART8 spare at 57600) don't benefit
meaningfully from DMA. Removing their 4 DMA channels reduces total from ~11 to ~7,
cutting AHB contention by ~36%.

**Fix:** Disabled DMA on UART7 and UART8 (CUAV).

### 4. Oversized UART Buffers in DMA-capable SRAM (Minor)

All UART buffers were 3000 bytes on both boards, regardless of baud rate.
These buffers are allocated from D2 SRAM (288 KB), which is shared with all DMA
peripherals and the USB OTG FIFO.

6 UARTs x 2 buffers x 3000 bytes = 36 KB of DMA-capable memory consumed.

**Fix:** Right-sized buffers based on actual throughput needs:
- UART4 (GPS2 38400): 1500 RX / 600 TX (GPS NMEA sentences + minimal TX)
- UART7 (console 115200): 600 RX / 1200 TX (interactive, TX larger for output)
- UART8 (spare 57600): 600 / 600
- NXT UART8 (console 115200): 600 RX / 1200 TX

Saves ~12 KB of D2 SRAM.

## Detailed Configuration Comparison

### USB CDC ACM (identical)

| Config | CUAV | NXT |
|---|---|---|
| CDCACM_RXBUFSIZE | 600 | 600 |
| CDCACM_TXBUFSIZE | 12000 | 12000 |
| CDCACM_IFLOWCONTROL | y | y |
| CDCACM_CONSOLE | y | y |
| OTG_ID_GPIO_DISABLE | y | y |
| USBDEV_BUSPOWERED | y | y |
| USBDEV_MAXPOWER | 500 | 500 |

### USB Descriptors (different)

| Config | CUAV | NXT |
|---|---|---|
| CDCACM_VENDORID | 0x3163 (CUAV) | 0x1B8C (Matek) |
| CDCACM_PRODUCTID | 0x004c | 0x0036 |
| CDCACM_PRODUCTSTR | "PX4 CUAV X7Plus-WL" | "HKUST UAV NxtPX4" |

### UART / DMA (major difference)

CUAV - 6 UARTs enabled:

| Port | Baud | DMA | RX buf | TX buf | Usage |
|---|---|---|---|---|---|
| USART1 | 38400 | RX+TX | 3000 | 3000 | GPS1 |
| USART2 | 921600 | RX+TX | 3000 | 3000 | VLA proxy |
| UART4 | 38400 | RX+TX | 1500* | 600* | GPS2 |
| USART6 | 57600 | none | 3000 | 3000 | RC input |
| UART7 | 115200 | none* | 600* | 1200* | debug console |
| UART8 | 57600 | none* | 600* | 600* | spare |

\* = changed from original 3000/3000 with DMA

NXT - 2 UARTs enabled:

| Port | Baud | DMA | RX buf | TX buf | Usage |
|---|---|---|---|---|---|
| USART1 | 921600 | RX+TX | 3000 | 3000 | uORB proxy |
| UART8 | 115200 | none | 600* | 1200* | console |

\* = changed from original 3000/3000

### TTY Signal Handling

| Config | CUAV (before) | CUAV (after) | NXT |
|---|---|---|---|
| TTY_SIGINT | y | not set | not set |
| TTY_SIGTSTP | y | not set | not set |

### IOB Buffers

| Config | CUAV (before) | CUAV (after) | NXT |
|---|---|---|---|
| MM_IOB | default | y | y |
| IOB_NBUFFERS | default (~8) | 24 | 24 |
| IOB_NCHAINS | default (~8) | 24 | 24 |

### Other Differences (not related to bug)

| Config | CUAV | NXT |
|---|---|---|
| Chip | STM32H743XI | STM32H743VI |
| VBUS pin | PA9 (dedicated) | PD0 (GPIO) |
| HS_USB_EN | PH15 | absent |
| I2C buses | 4 | 2 |
| SPI DMA | SPI1+SPI4+SPI6 (1024B each) | SPI2 (4096B) |
| SPI_DMATHRESHOLD | not set | 8 |
| Timers | 7 (TIM1,3,4,5,8,12,15) | 5 (TIM1,2,3,4,8) |
| ADC | ADC1,3 | ADC1,2,3 |
| Flash | RAMTRON emulated | W25 QSPI |
| LIBC_FLOATINGPOINT | no | yes |
| DEBUG_MEMFAULT | no | yes |
| WQUEUE_NOTIFIER | no | yes |
| FLASH_OVERRIDE_I | no | yes |
| USE_LEGACY_PINMAP | no | yes |

## Workaround (still in place)

`Tools/mcp/nsh_client.py` prefixes all command writes with a space byte as a
sacrificial first byte. NSH ignores leading whitespace, so the space is absorbed
by the delay and the actual command arrives intact. This workaround remains in
place as defense-in-depth even after the defconfig fixes.

## Verification Plan

1. Build and flash CUAV with updated defconfig
2. Test NSH commands via USB CDC ACM without the nsh_client.py workaround
3. Verify first byte is no longer delayed
4. If fixed, the nsh_client.py space-prefix workaround can optionally be kept
   for robustness against other boards that may have similar issues
