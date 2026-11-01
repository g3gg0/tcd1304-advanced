# TCD1304 advanced firmware protocol

Firmware for the STM32F401RE/Nucleo and TCD1304 linear CCD. This variant extends the `sep19-TCD1304-uart` firmware with a falling-edge external trigger and trigger delay. It keeps the 12-byte command layout and raw spectrum payload, but uses UART2 at **500000 baud**; the Sep 19 firmware is configured for **115200 baud** and is not wire-compatible with the current `SpectrumPlotter` connection settings.

## Serial settings and framing

- UART2, 8 data bits, no parity, 1 stop bit (8N1), no flow control.
- Host to device commands are exactly 12 bytes. Bytes are indexed from zero.
- The command sync bytes are ASCII `E`, `R` (`0x45 0x52`). Firmware DMA receives a circular 12-byte buffer and searches for this sync, including a sync split across the buffer boundary.
- Default/legacy device to host spectrum data is exactly `3694 * 2 = 7388` bytes: 3694 little-endian unsigned 16-bit ADC samples, with no header, checksum, or terminator. The host must read a complete fixed-size payload before issuing/handling the next capture. Negotiated downsampling/encoding changes this format as specified below.
- ADC values are sent as captured/averaged; the wire representation is low byte followed by high byte. Values are nominally 12-bit.

## Command layout

| Bytes | Capture/configuration command | Trigger configuration (`mode = 2`) |
|---|---|---|
| 0..1 | `45 52` (`ER`) | `45 52` (`ER`) |
| 2..5 | SH period, unsigned 32-bit big-endian | bytes 2..3 reserved; bytes 4..5 delay in µs, unsigned 16-bit big-endian |
| 6..9 | ICG period, unsigned 32-bit big-endian | byte 6 selects trigger mode/extension; bytes 7..9 reserved except for the build-information and downsample extensions |
| 10 | Capture mode: `0` one shot, `1` continuous | `2` |
| 11 | Average count: `1..15`; zero or `>=16` means 1 | `0` for trigger configuration; `1` for the extended identity query below |

The trigger configuration has a firmware-specific interpretation of bytes 4..6: bytes 4..5 are an unsigned 16-bit delay in microseconds, big-endian; byte 6 is the trigger selector. Selector `0` arms the external falling-edge trigger on PC13, while selector `2` starts without waiting for an external edge. The current C# application sends the delay in bytes 4..5 and uses selectors `0` and `2`.

The trigger configuration command does not start a capture. Send a separate capture command to set the integration timing and begin acquisition. For externally triggered acquisition, PC13 must first be observed inactive/high; its subsequent falling edge starts the sequence after the configured delay. Trigger delay is limited to 0..65535 us.

## Identity and build information (extension v1)

The original identity query remains exactly compatible with existing C# hosts:

```text
Host:   45 52 00 00 00 00 FF 00 01 00 02 00
Device: 5B 67 33 67 67 30 2E 64 65 5D       ASCII [g3gg0.de]
```

After verifying this identity, a host may send a second, explicitly marked query:

```text
Host: 45 52 00 00 00 00 FF 42 49 31 02 01
                            B  I  1
```

This uses selector `FF`, bytes 7..9 `BI1`, mode byte 10 `02`, and revision byte 11 `01`. New firmware responds as follows:

| Offset | Length | Content |
|---|---|---|
| 0 | 10 | ASCII `[g3gg0.v1]` (no NUL) |
| 10 | 2 | Payload length `N`, unsigned 16-bit **big-endian**, 1..1024 |
| 12 | N | UTF-8 JSON build metadata, no trailing NUL/newline |

Example JSON (actual values are generated at build time):

```json
{"firmware":"tcd1304-advanced","protocol":1,"version":"1.1.0","built_at":"2026-11-01T12:34:56Z","build_timezone":"UTC","commit":"9376e02864f1fc524f94398520c686b4e46f92af","describe":"9376e02","branch":"master","dirty":true,"board":"nucleo_f401re","mcu":"stm32f401ret6","environment":"nucleo_f411re","compiler":"7.2.1","cpu_hz":84000000,"ccd_clock_hz":2000000}
```

`firmware` and `protocol` identify the metadata schema. `version` is the configured firmware release version, while `commit` is the full Git HEAD ID. `dirty` records tracked changes **or untracked files** at build time; it is `null` when Git state is unavailable. `built_at` is an ISO 8601 UTC timestamp for PlatformIO builds. `cpu_hz` and `ccd_clock_hz` describe configured clocks, not hardware timing measurements. Unknown optional metadata is `null`.

Older advanced firmware safely interprets the second query as an ordinary identity request and responds with `[g3gg0.de]` only. The web host then continues with legacy identity verification and explicitly marks build information unavailable. Hosts must not assume a length field follows this legacy response. Neither query starts an acquisition. New firmware also leaves trigger settings unchanged when answering identity queries.

Only request build information before acquisition or between complete frames: metadata and spectra share the same unframed UART transport. Validate the response magic, length and JSON schema before sending a capture command. Receiving an extended response with an invalid length, unsupported schema, invalid UTF-8 or incomplete JSON must fail the connection rather than treating metadata bytes as spectrum data.

### Generating build metadata

`platformio.ini` runs `tools/build_info.py` before compilation. The helper creates `.pio/build/<environment>/generated/build_info.generated.h`, which is embedded by `src/build_info.c`. Generated metadata remains outside the tracked source tree. `custom_firmware_version` controls the release version. Git is optional: source archives build with an unknown commit and working-tree state.

`SOURCE_DATE_EPOCH`, if set, supplies the UTC build timestamp for reproducible builds. Otherwise each build records the current UTC time. Non-PlatformIO builds use compiler date/time with an explicitly unspecified timezone and unknown Git provenance.

Build and generator tests:

```text
pio run -e nucleo_f411re
python -m unittest discover -s tests -v
```

The existing environment name `nucleo_f411re` is retained for compatibility; its configured board is **nucleo_f401re**.

## Capture behavior

On a capture command, the firmware flushes the CCD, applies SH and ICG timing, then acquires. In one-shot mode it stops the ICG sequence after the completed spectrum and transmits one payload. Continuous mode (`1`) continues acquisition and transmits each completed spectrum. Averaging accumulates the requested number of integrations and emits their integer average as one payload. The C# application currently requests mode `0` and average count `0` (one integration, one-shot).

## Differences from `sep19-TCD1304-uart`

- UART baud rate changed from 115200 to 500000 to match `SpectrumPlotter`.
- Trigger input moved from the earlier polled PA15 path to PC13 with EXTI falling-edge handling and a microsecond delay timer.
- Trigger configuration now interprets only bytes 4..5 as delay; the older implementation copied bytes 2..5 as a 32-bit value.
- New transmission helper supports the short `[g3gg0.de]` connection probe and variable DMA lengths. The spectrum payload format and command length remain unchanged.

## Build and license

GitHub Actions builds every newly pushed commit on `master` (including multiple commits in one push) and publishes a prerelease tagged `build-<full-commit-sha>`. Only successful builds are published. The first push of a new branch builds its tip; pull requests build without publishing. Pushing `v*` tags still publishes regular releases. Each commit prerelease contains `tcd1304-advanced.bin` and `manifest.json` with the target board, full commit, binary size and SHA-256 checksum. Re-running a published build does not overwrite existing assets.

The workflow also maintains an immutable firmware archive on the `gh-pages` branch and deploys it using GitHub Pages. Enable **Settings → Pages → Source: GitHub Actions** in this firmware repository. SpectrumPlotterWeb loads `https://g3gg0.github.io/tcd1304-advanced/catalog.json` and `builds/<commit>/firmware.bin` directly in the browser, avoiding release-download CORS restrictions. Previous builds remain in the archive; `manifest.json` accompanies each binary. The Pages publication runs after the push's build jobs succeed. Releases created before this archive was introduced are not automatically backfilled.

The PlatformIO project targets `nucleo_f401re` with the STM32 Standard Peripheral Library. The PlatformIO environment is named `nucleo_f411re` for historical reasons, but it builds for the **F401RE**, not the F411RE.

### Local build

Install GNU Make and Python 3 on Windows or Linux, then run in this directory:

```sh
make
```

The Makefile uses a project-local PlatformIO environment if present, otherwise the standard PlatformIO IDE installation in `~/.platformio/penv` (or `%USERPROFILE%\.platformio\penv` on Windows), then `pio` or `platformio` on `PATH`. If none exists, it automatically installs PlatformIO Core 6.2.0 in `.pio/venv-linux` or `.pio/venv-win32`. That fallback also stores the downloaded STM32 platform and compiler in `.pio/core-linux` or `.pio/core-win32`, without installing anything globally. These directories are ignored by Git and kept by `make clean`.

Windows and WSL use separate environments, even when sharing the same project directory. On Ubuntu/Debian, the automatic installation requires `python3-venv`; the first build requires internet access.

If needed, specify the PlatformIO executable explicitly, for example in Windows PowerShell:

```powershell
make "PIO=C:\Users\yourname\.platformio\penv\Scripts\pio.exe"
```

The default Python command is `python` on Windows and `python3` on Linux. Override it with `make PYTHON=python3.12` or `make "PYTHON=C:\path\to\python.exe"` if needed.

PlatformIO may download the STM32 platform and compiler on the first build. The outputs are:

- `.pio/build/nucleo_f411re/firmware.hex` — copy this to the `NODE_F401RE` drive to program the board.
- `.pio/build/nucleo_f411re/firmware.bin`
- `.pio/build/nucleo_f411re/firmware.elf`

Run `make clean` to remove the PlatformIO build output. GitHub Actions uses the same Makefile and publishes the firmware files as a build artifact.

The firmware is based on Esben Rossel's TCD1304 driver; see <https://tcd1304.wordpress.com/>. Source and binaries retain the FreeBSD license terms in the source headers.

## Downsampling and sample encoding (firmware 1.2.0)

Discover capabilities through the existing `FF / BI1` build-information query. The JSON adds:

```json
{"features":["downsample-v1","sample-encoding-v1"],"max_downsample":64,"pixels":3694,"adc_bits":12}
```

The metadata envelope remains protocol 1. **Never send `FE` to firmware that does not advertise `downsample-v1`**: old firmware interprets it as a trigger selector. Encoding other than raw16 additionally requires `sample-encoding-v1`. Version strings alone are not capability detection. Existing C# clients retain their full raw frames without changes.

### Configure the next one-shot

Send exactly 12 bytes, wait for and validate its 8-byte acknowledgement, then send an ordinary capture command. No capture starts from this configuration alone. Do not send commands during acquisition/transfer.

| Offset | Meaning |
|---|---|
| 0..1 | ASCII `ER` (`45 52`) |
| 2..3 | Stride N, uint16 big-endian, 1..64 |
| 4..5 | Trigger delay, uint16 big-endian, microseconds |
| 6 | `FE` extension selector |
| 7 | Trigger: `00` external falling edge PC13, `02` immediate |
| 8..9 | ASCII `D1` (`44 31`) |
| 10 | `02` configuration mode |
| 11 | Low nibble revision `1`; bits 4..5 encoding ID; bits 6..7 zero |

Encoding IDs: 0=raw16, 1=log10, 2=packed12, 3=log8. Byte 11 is respectively `01`, `11`, `21`, `31` hexadecimal. Firmware rejects all other revision/flag bits, invalid strides and invalid triggers without mutating configuration.

Example: stride 15, immediate trigger, zero delay, logarithmic 8-bit:

```text
Host:   45 52 00 0F 00 00 FE 02 44 31 02 31
Device: 44 53 31 00 00 0F 00 F7
```

Acknowledgement: bytes 0..1 ASCII `DS`; byte 2 echoes the revision/encoding byte; byte 3 status (0 success, 1 invalid); bytes 4..5 accepted stride; bytes 6..7 sample count `ceil(3694/N)`, both uint16 big-endian. An invalid request has zero stride/count. Host must validate the complete acknowledgement before capture; on mismatch, disconnect instead of assuming stream alignment.

The accepted stride/encoding is consumed by the next capture and immediately resets to N=1/raw16 for subsequent commands. Continuous mode always uses full raw16 frames. An ordinary trigger command or identity/build query also resets pending settings. Hosts therefore configure `FE` before **every** optimized one-shot. Legacy trigger configuration keeps its existing no-ack behavior.

### Frame layout

N=1 with raw16 retains the original 7388-byte response with no header. All other combinations use a **16-byte header**, followed by exactly `ceil(sample_count * bits_per_sample / 8)` bytes:

| Offset | Meaning |
|---|---|
| 0..3 | ASCII `DSF1` |
| 4..5 | Big-endian uint16: encoding ID in bits 14..15, stride in bits 0..13 |
| 6..7 | Sample count, uint16 big-endian |
| 8..9 | Minimum ADC count over the full 3694-pixel frame |
| 10..11 | Maximum ADC count over the full frame |
| 12..13 | Full-frame count of saturated samples (`==4095`) |
| 14..15 | Full-frame count of invalid samples (`>4095`) |

All header integers are big-endian. Payload samples correspond to original pixels `0,N,2N,... <3694`, with no synthetic endpoint. Dark subtraction and wavelength mapping must use these original pixel indices. Statistics are computed **before** decimation/encoding, so omitted/clipped pixels still cause an ADC error. Existing hardware averaging precedes these statistics and can hide clipping in an individual integration, as before.

Raw16 payload is uint16 little-endian. Packed12/log10/log8 payload is a continuous **least-significant-bit first** stream: sample 0 occupies the lowest bits of byte 0; subsequent sample bits fill following positions/bytes without per-sample padding. Unused high bits in the final byte are zero. Packed12 preserves all ADC bits exactly. Invalid ADC values are clamped to 4095 only for packed/log payloads; the original full-frame invalid count remains in the header.

For logarithmic formats with B=10 or B=8, unsigned integer code Q and reconstructed count V are:

```text
Q = floor(log(1 + ADC) / log(4096) * (2^B - 1) + 0.5)
V = floor(exp(Q / (2^B - 1) * log(4096)) - 1 + 0.5)
```

Both endpoints (0 and 4095) are preserved. Log10 gives approximately 0.41% half-step error in `(ADC+1)`; log8 approximately 1.65%, plus integer rounding. Near full scale these can be roughly 17 / 67 ADC counts. Log values can reconstruct to 4095 even when the original was below saturation: **use header statistics for exposure assessment**, never the decoded peak. Quantization occurs before browser dark subtraction, so weak dark-corrected residuals on a large baseline can have larger relative errors. Calibration batches always request N=1/raw16.

Firmware uses generated flash lookup tables (`tools/generate_log_encoding.py`) rather than per-pixel logarithms. In-place decimation and packing reuse the ADC buffer after the completed one-shot, without a second frame-sized RAM allocation. The full sensor is still sampled at the existing clock/timing; this mode reduces UART traffic only.

### Resolution, bandwidth and limits

The web app requests `N = min(max_downsample, ceil(3694 / plot_resolution))` automatically, with an independently selected sample encoding. Defaults: 12-bit lossless, plot target 1850 (N=2, 1847 actual samples). Targets 1024/512/256/128/64 select N=4/8/15/29/58. Changes apply to the next frame, keeping the currently received frame's geometry/encoding fixed. Legacy firmware remains full raw16 with browser-side plot reduction.

At 500000 baud/8N1, payload transfer time is `bytes * 10 / 500000` seconds. These examples include a frame header where required, excluding configuration/ack commands, exposure, readout, flush and host latency:

| Mode | Samples | Bytes/frame | UART time |
|---|---:|---:|---:|
| Full raw16 | 3694 | 7388 | 147.76 ms |
| Full packed12 | 3694 | 5557 | 111.14 ms |
| Full log10 | 3694 | 4634 | 92.68 ms |
| Full log8 | 3694 | 3710 | 74.20 ms |
| Target 256, N=15, raw16 | 247 | 510 | 10.20 ms |
| Target 256, N=15, packed12 | 247 | 387 | 7.74 ms |
| Target 256, N=15, log10 | 247 | 325 | 6.50 ms |
| Target 256, N=15, log8 | 247 | 263 | 5.26 ms |

These are calculated serial times, **not measured frame rates or a 50-FPS guarantee**. The current repeated-one-shot implementation retains its CCD flush/readout/exposure delays. Nth-sample decimation can miss narrow spectral peaks; select full resolution for line identification and precise measurements. There is still no payload CRC or sequence number; header validation detects geometry/framing errors but cannot detect arbitrary payload corruption.
