# TCD1304 advanced firmware protocol

Firmware for the STM32F401RE/Nucleo and TCD1304 linear CCD. This variant extends the `sep19-TCD1304-uart` firmware with a falling-edge external trigger and trigger delay. It keeps the 12-byte command layout and raw spectrum payload, but uses UART2 at **500000 baud**; the Sep 19 firmware is configured for **115200 baud** and is not wire-compatible with the current `SpectrumPlotter` connection settings.

## Serial settings and framing

- UART2, 8 data bits, no parity, 1 stop bit (8N1), no flow control.
- Host to device commands are exactly 12 bytes. Bytes are indexed from zero.
- The command sync bytes are ASCII `E`, `R` (`0x45 0x52`). Firmware DMA receives a circular 12-byte buffer and searches for this sync, including a sync split across the buffer boundary.
- Device to host spectrum data is exactly `3694 * 2 = 7388` bytes: 3694 little-endian unsigned 16-bit ADC samples, with no header, checksum, or terminator. The host must read a complete fixed-size payload before issuing/handling the next capture.
- ADC values are sent as captured/averaged; the wire representation is low byte followed by high byte. Values are nominally 12-bit.

## Command layout

| Bytes | Capture/configuration command | Trigger configuration (`mode = 2`) |
|---|---|---|
| 0..1 | `45 52` (`ER`) | `45 52` (`ER`) |
| 2..5 | SH period, unsigned 32-bit big-endian | Reserved; current firmware ignores these bytes |
| 6..9 | ICG period, unsigned 32-bit big-endian | byte 6 selects trigger mode; bytes 7..9 ignored |
| 10 | Capture mode: `0` one shot, `1` continuous | `2` |
| 11 | Average count: `1..15`; zero or `>=16` means 1 | `0` |

The trigger configuration has a firmware-specific interpretation of bytes 4..6: bytes 4..5 are an unsigned 16-bit delay in microseconds, big-endian; byte 6 is the trigger selector. Selector `0` arms the external falling-edge trigger on PC13, while selector `2` starts without waiting for an external edge. The current C# application sends the delay in bytes 4..5 and uses selectors `0` and `2`.

The trigger configuration command does not start a capture. Send a separate capture command to set the integration timing and begin acquisition. For externally triggered acquisition, PC13 must first be observed inactive/high; its subsequent falling edge starts the sequence after the configured delay. Trigger delay is limited to 0..65535 us.

## Capture behavior

On a capture command, the firmware flushes the CCD, applies SH and ICG timing, then acquires. In one-shot mode it stops the ICG sequence after the completed spectrum and transmits one payload. Continuous mode (`1`) continues acquisition and transmits each completed spectrum. Averaging accumulates the requested number of integrations and emits their integer average as one payload. The C# application currently requests mode `0` and average count `0` (one integration, one-shot).

## Differences from `sep19-TCD1304-uart`

- UART baud rate changed from 115200 to 500000 to match `SpectrumPlotter`.
- Trigger input moved from the earlier polled PA15 path to PC13 with EXTI falling-edge handling and a microsecond delay timer.
- Trigger configuration now interprets only bytes 4..5 as delay; the older implementation copied bytes 2..5 as a 32-bit value.
- New transmission helper supports the short `[g3gg0.de]` connection probe and variable DMA lengths. The spectrum payload format and command length remain unchanged.

## Build and license

The PlatformIO project targets `nucleo_f401re` with the STM32 Standard Peripheral Library. The firmware is based on Esben Rossel's TCD1304 driver; see <https://tcd1304.wordpress.com/>. Source and binaries retain the FreeBSD license terms in the source headers.
