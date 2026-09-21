# ESP32-S3 TRMNL refresh smoke test

Standalone firmware using the local epd-waveshare source. Runs one sequence per
boot: normal full, fast full, partial, then panel sleep. No Wi-Fi or MCU deep sleep.
USB Serial/JTAG reports refresh durations, BUSY observations, and TEST DONE or a
panic. No automatic retries. BUSY polling is bounded by a 30-second SPI deadline.

EE04 wiring: SCLK 7, MOSI 9, CS 44, DC 10, RESET 38, BUSY 4 (active low),
panel power enable 43 (active high). USB logging avoids UART/GPIO43 conflicts.

From this directory, using the installed Espressif environment:

```powershell
. C:/Users/Manuel/export-esp.ps1
.\build-x64.cmd
espflash flash --chip esp32s3 --port COM11 --non-interactive --monitor --skip-update-check target/xtensa-esp32s3-none-elf/release/esp32s3-refresh
```

Use Espressif Rust release 1.93.0.0 for host x86_64-pc-windows-msvc
(on Windows ARM64 this runs under x64 emulation). Cargo may download missing
dependencies. The lockfile starts from the neighboring ESP32-S3 firmware versions
to avoid newer dependencies requiring Rust 1.95. `build-x64.cmd` loads the
installed Visual Studio 2026 x64 environment and runs
`cargo +esp build --release --target xtensa-esp32s3-none-elf`.
Flashing replaces the application on the connected board.

Expected image sequence (four seconds between stages):
1. White screen with black border and a solid upper-left rectangle.
2. Same image with vertical bars added in the upper right.
3. Same image with a lower-center black rectangle containing a white stripe.

BUSY/timing logs establish command completion, not visual quality. Inspect the
screen for ghosting, contrast and preservation of content outside the partial
rectangle. Resetting the MCU starts another sequence; leave it running after DONE
when only one run is desired.
