# CLAUDE.md

Guidance for Claude Code working in this repository.

## What this is

A fork of [SensorsIot/NTP-Server-with-GPS](https://github.com/SensorsIot/NTP-Server-with-GPS): DIY NTP servers disciplined by a GNSS receiver's PPS output. Active work is on `PPS_Newapproach_V4`, a rewrite that turns a WT32-ETH01 (ESP32 + LAN8720 Ethernet) and a Quescan M10050 (u-blox M10) receiver into a stratum 1 NTP server, aiming for low response latency (< 1 ms) and low jitter (< 0.2 ms). The other sketches are from the upstream project and are kept for reference.

## Layout

| Path | Notes |
| --- | --- |
| `ESP32/ESP32-Ethernet/PPS_Newapproach_V4` | **Main sketch.** Simple, low-jitter NTP server for a WT32-ETH01 with a PPS GPS module |
| `ESP32/ESP32-Ethernet/PPS_Newaproach_V3` | Previous version (note the upstream spelling "Newaproach") |
| `ESP32/ESP32-C3` | Upstream WiFi sketches for ESP32-C3 (`NMEASerial`, `tinyNTPServerPPS`, `tinyNTPServerTime`) |
| `Raspberry Pi` | Upstream chrony setup guide (PDF only) |
| `docs` | Documentation. [`docs/PPS_Newapproach_V4-design.md`](docs/PPS_Newapproach_V4-design.md) explains how V4 works |
| `.devcontainer` | Debian + `arduino-cli` dev container |

`credentials.h` files are gitignored. V4 does not include its `credentials.h`.

## V4 architecture

Everything is in one file, `PPS_Newapproach_V4.ino`. The ESP32 system clock is **not** used. Served time is computed from `esp_timer` as `ntpSec + (t − edgeUs) × (1 − rate)`, using a `timescale` struct (anchor PPS edge, its UTC second, and the timer's frequency error) guarded by the `tsMux` spinlock.

| Context | Function | Job |
| --- | --- | --- |
| Interrupt (GPIO2 rising) | `PPS_ISR` | Saves the low 32 bits of `esp_timer_get_time()` into `lastPPSMicros` (one word, so atomic) |
| FreeRTOS task `GPS` (core 1, prio 2, every 10 ms) | `gpsTask` → `readGPSTime` → `updateTimescale` | Parses NMEA with TinyGPSPlus and pairs the time with the latest PPS edge. Rejects edges whose spacing implies more than `MAX_PPM`, keeps an exponential average of the frequency error (`rateQ32`, `RATE_SHIFT`), and publishes the anchor |
| lwIP tcpip thread | `onNtpPacket` | Raw lwIP `udp_recv` callback. Reads `esp_timer` first, copies `timescale`, converts with `toNtp()`, rewrites the request `pbuf` in place, reads `esp_timer` again for the transmit time and sends with `udp_sendto` |
| Arduino `loop()` | `loop` | `ArduinoOTA.handle()` every 100 ms (hostname `gpsclock`, mDNS disabled to keep the tcpip thread quiet, so upload by IP) |

NTP reply behavior:
- Only answers packets that are at least 48 bytes and in mode 3 (client), once `timescale.valid` is set (two GNSS syncs agree).
- Copies VN from the request.
- Reports LI=3 and stratum 16 once the anchor is older than `HOLDOVER_MS`, but keeps extrapolating at the learned rate. Otherwise reports LI=0 and stratum 1.

Startup (`setup`): fix the CPU at 240 MHz, wait 10 s for the GNSS module, start UART1 and the PPS ISR, then `initNetwork` (PHY power-cycle, DHCP with a 30 s timeout), `initOTA`, `initRawNTP`, and start `gpsTask`. Init failures call `fatalError()`, which restarts the device.

## Working on V4

- **Keep the NTP callback minimal.** `onNtpPacket` runs in the lwIP thread, so any work added there delays every packet and adds jitter. The `esp_timer_get_time()` call must stay first. Don't replace the raw API with `WiFiUDP`/`AsyncUDP` or a polled task.
- **Calling raw lwIP from outside the tcpip thread** requires `LOCK_TCPIP_CORE()` / `UNLOCK_TCPIP_CORE()`, as in `initRawNTP`.
- **Shared state.** Single-word globals (`lastPPSMicros`) rely on atomic 32-bit access. `timescale` has 64-bit fields, so always read and write it under `tsMux`, holding the lock only to copy it.
- **Don't reintroduce the system clock.** No `settimeofday`, `adjtime` or `gettimeofday` in the timing path. They take a mutex and add the step/slew behaviour this design removed. If something needs wall-clock time, derive it from `timescale` with `toNtp()`.
- **Keep the timing maths integer-only.** The ESP32 FPU is single-precision only, so `double` is emulated in software.
- **The PPS/NMEA pairing guard** (`1 ms < delta < 900 ms`) assumes NMEA for second N arrives a few hundred ms after PPS edge N.
- **`CORRECTION_US`** is an empirical, measured constant. Don't change it without new measurements. The procedure is in the "Tuning" section of the design doc.
- **The hardware is fixed.** The code targets one setup only (WT32-ETH01 with a Quescan M10050 at default settings: 38400 baud, 1 Hz PPS), and the pins and settings are `#define`s at the top of the file. `STATUS_LED_PIN` (GPIO33) is a placeholder because the board has no LED.
- **The OTA password** (`uploadota`) is hard-coded.
- **Keep the design doc current.** When V4 behavior changes, update `docs/PPS_Newapproach_V4-design.md` to match, especially its "Known limitations" list.

Open TODOs (README): add a watchdog.

## Building

There is no test suite. Testing is done on hardware by comparing against a trusted NTP reference (see the "Tuning" section of the design doc). The timescale maths (`usecToNtpFrac`, `toNtp`, `updateTimescale`) is plain C++, so it can be checked on the host by copying those functions into a small `g++` harness with `portENTER_CRITICAL`/`portEXIT_CRITICAL` stubbed out.

The `.ino` uses CRLF line endings. Strip the `\r` before matching lines with `sed`/`grep` anchors.

The dev container image ships `arduino-cli` without the ESP32 core or libraries, so install them once per container. V4 has been compiled with esp32 core 3.3.12 and TinyGPSPlus 1.0.3. `ETH.h`, `ArduinoOTA` and `ESPmDNS` come with the core.

```sh
URL=https://espressif.github.io/arduino-esp32/package_esp32_index.json
arduino-cli core update-index --additional-urls $URL
arduino-cli core install esp32:esp32 --additional-urls $URL
arduino-cli lib install TinyGPSPlus
arduino-cli compile --fqbn esp32:esp32:wt32-eth01 --warnings all ESP32/ESP32-Ethernet/PPS_Newapproach_V4
```

The only expected warning is an implicit fall-through inside TinyGPSPlus.
