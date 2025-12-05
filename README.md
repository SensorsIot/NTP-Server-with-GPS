# NTP Server with GPS

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
