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

Everything is in one file, `PPS_Newapproach_V4.ino`. Four execution contexts share the ESP32 system clock and a few `volatile` 32-bit globals, with no locks:

| Context | Function | Job |
| --- | --- | --- |
| Interrupt (GPIO2 rising) | `PPS_ISR` | Saves `micros()` into `lastPPSMicros` |
| FreeRTOS task `GPS` (core 1, prio 2, every 10 ms) | `gpsTask` → `readGPSTime` | Parses NMEA with TinyGPSPlus, pairs the time with the latest PPS edge, then slews (`adjtime`) the system clock if the offset is under 1 s or steps it (`settimeofday`) otherwise. Publishes `lastSyncSec`/`lastSyncFrac` (NTP format, network byte order), `lastSyncMillis` and `everSynced` |
| lwIP tcpip thread | `onNtpPacket` | Raw lwIP `udp_recv` callback. Calls `gettimeofday` first, then rewrites the request `pbuf` in place and sends it back with `udp_sendto` |
| Arduino `loop()` | `loop` | `ArduinoOTA.handle()` every 100 ms (hostname `gpsclock`) |

NTP reply behavior:
- Only answers packets that are at least 48 bytes, in mode 3 (client), and arrive after the first GNSS sync (`everSynced`).
- Copies VN from the request.
- Reports LI=3 and stratum 16 once the last sync is older than `HOLDOVER_MS`. Otherwise reports LI=0 and stratum 1.

Startup (`setup`): fix the CPU at 240 MHz, wait 10 s for the GNSS module, start UART1 and the PPS ISR, then `initNetwork` (PHY power-cycle, DHCP with a 30 s timeout), `initOTA`, `initRawNTP`, and start `gpsTask`. Init failures call `fatalError()`, which restarts the device.

## Working on V4

- **Keep the NTP callback minimal.** `onNtpPacket` runs in the lwIP thread, so any work added there delays every packet and adds jitter. The `gettimeofday` call must stay first. Don't replace the raw API with `WiFiUDP`/`AsyncUDP` or a polled task.
- **Calling raw lwIP from outside the tcpip thread** requires `LOCK_TCPIP_CORE()` / `UNLOCK_TCPIP_CORE()`, as in `initRawNTP`.
- **Shared state is lock-free.** It works only because each global is a single aligned 32-bit word. Keep new shared state the same way, or add explicit synchronization.
- **Prefer slewing to stepping.** Avoid changes that make the clock jump or run backwards during normal operation.
- **The PPS/NMEA pairing guard** (`1 ms < delta < 900 ms`) assumes NMEA for second N arrives a few hundred ms after PPS edge N.
- **`CORRECTION_US`** (currently 545 µs) is an empirical, measured constant. Don't change it without new measurements. See the README testing notes.
- **The hardware is fixed.** The code targets one setup only (WT32-ETH01 with a Quescan M10050 at default settings: 38400 baud, 1 Hz PPS), and the pins and settings are `#define`s at the top of the file. `STATUS_LED_PIN` (GPIO33) is a placeholder because the board has no LED.
- **The OTA password** (`uploadota`) is hard-coded.
- **Keep the design doc current.** When V4 behavior changes, update `docs/PPS_Newapproach_V4-design.md` to match, especially its "Known limitations" list. Some items in that list (always claiming sync, stepping when the clock is ahead, no holdover) are already fixed in the current code.

Open TODOs (README): add a watchdog.

## Building

There is no test suite. Testing is done on hardware by comparing against a trusted NTP reference (see the README).

The dev container ships `arduino-cli` without the ESP32 core or libraries. To compile V4 you need the `esp32:esp32` core and the `TinyGPSPlus` library. `ETH.h`, `ArduinoOTA` and `ESPmDNS` come with the core. Once they are installed, a likely compile command is the following (untested here):

```sh
arduino-cli compile --fqbn esp32:esp32:wt32-eth01 ESP32/ESP32-Ethernet/PPS_Newapproach_V4
```
