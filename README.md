# NTP Server with GPS

## Changes from forked source

Take a look at [`ESP32/ESP32-Ethernet/PPS_Newapproach_V4/PPS_Newapproach_V4.ino`](ESP32/ESP32-Ethernet/PPS_Newapproach_V4/PPS_Newapproach_V4.ino) (and the detailed design notes in [`docs/PPS_Newapproach_V4-design.md`](docs/PPS_Newapproach_V4-design.md)). This is rewritten code assisted by Gemini 4 Neon and Claude Opus 5.5 with testing by myself.

* **Low latency:** Uses lwIP's raw UDP callback directly in the `tcpip` thread, taking the receive timestamp on arrival and the transmit timestamp just before sending. In testing with OPNsense, delay is < 1 ms (~0.68 ms).
* **Low jitter & hardware PPS capture:** Serves time from the ESP32's free-running microsecond timer (`esp_timer`), anchored at each PPS rising edge latched in hardware via the ESP32's 80 MHz MCPWM capture peripheral (removing PPS ISR latency jitter) and corrected for the crystal's measured frequency error, so the clock never jumps or slews. In testing with OPNsense, jitter is ~0.07 ms.
* **Stratum 1 protocol & holdover behavior:** Sets NTP server header fields (`precision`, `rootDelay`, and holdover-scaled `rootDispersion` growing at 15 ppm per RFC 5905), guards `pbuf` lengths, stays silent until two GNSS syncs agree, and transitions to LI=3 / stratum 16 after 60 s without a valid sync.
* **GNSS fix & edge validation:** Parses date, time, and fix status (`'A'`/`'V'`) atomically from RMC (`$GNRMC`/`$GPRMC`), requires valid date/time and at least 4 satellites (from GGA), rejects outlier PPS edges against the rate-predicted spacing, and requires corroboration before trusting a re-seeded anchor.
* **u-blox M10 startup config (RAM-only):** Sends a non-destructive `UBX-CFG-VALSET` at boot to set Stationary dynamic mode, disable unused NMEA sentences (keeping only RMC and GGA), and output 1 Hz / 100 ms PPS pulses only when GNSS-locked.
* **Watchdog & Arduino OTA:** Subscribes the GPS task to the ESP-IDF task watchdog, and enables Arduino OTA (pausing NTP replies, PPS capture, and the GPS task watchdog during flash writes).
* **Fixed hardware target:** Configured specifically for a WT32-ETH01 module with a Quescan M10050 (u-blox M10) GNSS receiver at 38400 baud and 1 Hz PPS.

### Flashing over OTA

mDNS is disabled to keep the network stack quiet, so the board won't show up as a network port in the Arduino IDE. To flash the firmware you'll need to export the compiled binary (Sketch > Export Compiled Binary) and run `espota` yourself from the command line:

```
%LOCALAPPDATA%\Arduino15\packages\esp32\hardware\esp32\[version]\tools>espota.exe -i [hardware ip] -p 3232 -f [path]\PPS_Newapproach_V4.ino.bin -a "[password]"
```

The password is the one set with `ArduinoOTA.setPassword()` in the sketch.

## Original readme

[![MIT License](https://img.shields.io/badge/License-MIT-yellow.svg)](https://opensource.org/licenses/MIT)
[![GitHub stars](https://img.shields.io/github/stars/SensorsIot/NTP-Server-with-GPS)](https://github.com/SensorsIot/NTP-Server-with-GPS/stargazers)
[![YouTube](https://img.shields.io/badge/YouTube-Video-red?logo=youtube)](https://youtu.be/BGb2t5FT-zw)

![ESP32](https://img.shields.io/badge/ESP32-Supported-green?logo=espressif)
![Raspberry Pi](https://img.shields.io/badge/Raspberry%20Pi-Supported-C51A4A?logo=raspberrypi)
![GPS](https://img.shields.io/badge/GPS-PPS-blue?logo=satellite)

Build your own precision NTP time server using GPS with PPS (Pulse Per Second) signal.

## 📺 Video Tutorials

| Platform | Video |
|----------|-------|
| Raspberry Pi | https://youtu.be/RKRN4p0gobk |
| ESP32 | https://youtu.be/BGb2t5FT-zw |

## 📁 Project Structure

```
├── ESP32/
│   ├── ESP32-C3/          # ESP32-C3 with WiFi
│   │   ├── NMEASerial/
│   │   ├── tinyNTPServerPPS/
│   │   └── tinyNTPServerTime/
│   └── ESP32-Ethernet/    # ESP32 with Ethernet (WT32-ETH01)
│       └── PPS_Newaproach_V3/
└── Raspberry Pi/          # Raspberry Pi setup
```

## ⚙️ Configuration

### 📶 WiFi Credentials (ESP32-C3)

Create a file named `credentials.h` in the sketch folder with your WiFi settings:

```cpp
#ifndef CREDENTIALS_H
#define CREDENTIALS_H

const char* ssid = "your-wifi-name";
const char* password = "your-wifi-password";

#endif
```

### 🔌 Pin Definitions (ESP32-C3)

| Function | GPIO |
|----------|------|
| GPS RX | GPIO 20 |
| GPS TX | GPIO 21 |
| PPS | GPIO 2 |

### 🌐 Ethernet Settings (WT32-ETH01)

Make sure the `ETH.begin()` parameters are in the correct order:

```cpp
ETH.begin(ethAddr, powerPin, mdcPin, mdioPin, phyType, clkMode);
```

## 📦 Dependencies

Install using the Arduino Library Manager:

| Library | Description |
|---------|-------------|
| [TinyGPSPlus](https://github.com/mikalhart/TinyGPSPlus) | GPS NMEA parsing |
| [U8g2](https://github.com/olikraus/u8g2) | OLED display |
