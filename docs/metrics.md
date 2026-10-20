# Measurements

All numbers come from the firmware itself (`perf`, `tasks`, `status`, command output).

## Conditions

| Item | Value |
|---|---|
| Board | BlackPill STM32F401CCU6 |
| Core clock | 84 MHz from the 25 MHz crystal through the PLL (an earlier run on the 16 MHz internal oscillator is shown where it matters) |
| SPI1 (flash) | 5.25 MHz, mode 0 |
| I2C1 (sensors) | 100 kHz |
| UART2 | 115200 8N1 |
| Build | Debug, `-O0 -g3` (no optimisation) |

## Stage 5.0: baseline, before DMA and the scheduler

The first column was measured on the 16 MHz internal oscillator by mistake: the clock tree was not configured yet, which is also why the SPI clock was 1 MHz. It is kept because it shows what the clock fix is worth.

| Metric | 16 MHz, SPI 1 MHz | 84 MHz, SPI 5.25 MHz |
|---|---|---|
| Main loop pass, idle average | 38.5 us | 7.6 us |
| Main loop pass, idle maximum | 3.8 ms | 3.2 ms |
| Main loop pass, maximum at start-up | 31.9 ms | 32.0 ms |
| Longest I2C transfer, at start-up | 2494 us | 2467 us |
| Longest I2C transfer, steady state | 877 us | 847 us |
| Loop stall during `flash read 0 256` | 116 ms | 107.8 ms |
| Loop stall during sector erase | 101 ms | 100.0 ms |
| Full flash scan (16 MB) | 345 s | 65.8 s |

What the baseline shows:

- The UART output is the main source of stalls: about 1220 characters at 115200 baud take about 106 ms, and the loop waited for all of them.
- Sector erase blocked the loop for about 100 ms.
- I2C stays under 1 ms in steady state and is bounded by the 100 kHz bus. It is deliberately left blocking.

## Stage 4: flash driver (84 MHz)

| Metric | Value |
|---|---|
| Sector erase (4 KB) | 93 - 101 ms |
| Page programming, 600 bytes over 4 pages | 7 ms |
| Read-back verify, 600 bytes | 17 ms |
| Log record size | 32 bytes, 128 records per sector, 2048 records in total |
| Log clear, one sector | 101 ms |

## After stages 5a - 5c: UART TX DMA, UART RX DMA, scheduler

| Metric | Before (5.0) | After |
|---|---|---|
| Stall during `flash read 0 256` | 107.8 ms (longest loop pass, `perf`) | 2.0 ms (longest task, `shell`, `tasks`), other tasks late by at most 1 ms |
| Stall during sector erase | 100.0 ms | 96.4 ms, other tasks late by 95 - 96 ms (removed in 5d) |
| Full flash scan | 65.8 s | 66.2 s (+0.6 %, scheduler overhead) |
| UART transmit | blocking | DMA, ring buffer peak 1202 of 2048 bytes, `waits=0`, `dropped=0`, `errors=0` |
| UART receive | interrupt per byte | circular DMA with IDLE detection, `rx_overflow=0` |
| Task schedule | everything on every pass | shell 1 ms, sensor 2 ms, button 5 ms, scan every pass |

Task statistics at idle (`tasks`):

| Task | Period | Average | Maximum | Latest late |
|---|---|---|---|---|
| shell | 1 ms | 1.6 - 1.9 us | 142 us | 0 - 1 ms |
| sensor | 2 ms | 7.3 - 8.2 us | 1254 us | 0 - 1 ms |
| button | 5 ms | 2.8 us | 2 us | 0 - 1 ms |

Notes:

- The loop-pass maximum (`perf`) and the task maximum (`tasks`) are different metrics. A direct `perf` run after `flash read 0 256` on the new firmware is still to be recorded.
- A 1 ms task with 1 ms tick resolution often shows `resync` when another task runs longer than 1 ms (653 during a flash scan). This costs no data: the UART receive buffer holds 512 bytes, and 2 ms of input is about 23 bytes.
- The UART receive test with long pasted input (stage 5b) was not measured before the change, so there is no loss comparison. Only the interrupt count can be compared: `events` is now much lower than `bytes`.

## After stage 5d: background erase

| Metric | Before (5c) | After |
|---|---|---|
| Longest task during `flash erase <sector>` | 96.4 ms | - |
| Other tasks late during `flash erase <sector>` | 95 - 96 ms | - |
| Chip erase duration | - | - |