#include <WiFi.h>
#include <WiFiUdp.h>
#include <TinyGPSPlus.h>
#include <time.h>
#include <EEPROM.h>
#include <freertos/FreeRTOS.h>
#include <freertos/task.h>
#include <freertos/semphr.h>
#include <Wire.h>
#include <U8g2lib.h>
#include "credentials.h"  // defines ssid, password

// —————— Debug Macros ——————
#define DEBUG_STANDARD        1
#define DEBUG_TIME_CRITICAL   0

#if DEBUG_STANDARD
  #define DEBUG_PRINT(x)    Serial.print(x)
  #define DEBUG_PRINTLN(x)  Serial.println(x)
#else
  #define DEBUG_PRINT(x)
  #define DEBUG_PRINTLN(x)
#endif

#if DEBUG_TIME_CRITICAL
  #define DEBUG_CRITICAL_PRINT(x)    Serial.print(x)
  #define DEBUG_CRITICAL_PRINTLN(x)  Serial.println(x)
#else
  #define DEBUG_CRITICAL_PRINT(x)
  #define DEBUG_CRITICAL_PRINTLN(x)
#endif

// —————— Pin Definitions ——————
#define GPS_RX_PIN    20    // ESP32-C3 RX1 ← GPS TX
#define GPS_TX_PIN    21    // ESP32-C3 TX1 → GPS RX (optional)
#define PPS_PIN        2    // Pulse-per-second input

// —————— Globals ——————
TinyGPSPlus   gps;
HardwareSerial gpsSerial(1);
WiFiUDP       udp;
SemaphoreHandle_t gpsMutex;

volatile bool PPSsignal    = false;
volatile bool PPSavailable = false;

#define GPSBaud           9600
#define CORRECTION_FACTOR 0

// —————— OLED Setup ——————
const int xOffset = 30;  // center 72px window horizontally
const int yOffset = 25;  // center 40px window vertically
U8G2_SSD1306_128X64_NONAME_F_HW_I2C u8g2(
  U8G2_R0, U8X8_PIN_NONE,
  /* SCL=*/ 6, /* SDA=*/ 5
);

// Draw current time and date on the OLED
void displayTime() {
  struct timeval tv;
  gettimeofday(&tv, NULL);
  time_t now = tv.tv_sec;
  struct tm timeinfo;
  localtime_r(&now, &timeinfo);

  char line1[16], line2[16];
  snprintf(line1, sizeof(line1), "%02d:%02d:%02d",
           timeinfo.tm_hour,
           timeinfo.tm_min,
           timeinfo.tm_sec);
  snprintf(line2, sizeof(line2), "%02d-%02d-%02d",
           timeinfo.tm_year % 100,
           timeinfo.tm_mon + 1,
           timeinfo.tm_mday);

  uint8_t fh      = u8g2.getMaxCharHeight(); // ~10px
  uint8_t spacing = 2;
  uint8_t lineH   = fh + spacing;
  int     x       = xOffset + 2;

  u8g2.clearBuffer();
  u8g2.setCursor(x, yOffset + 0 * lineH);
  u8g2.print(line1);
  u8g2.setCursor(x, yOffset + 1 * lineH);
  u8g2.print(line2);
  u8g2.sendBuffer();
}

// —————— Wi-Fi + NTP Init ——————
void initNetwork() {
  WiFi.mode(WIFI_STA);
  WiFi.begin(ssid, password);
  unsigned long t0 = millis();
  while (WiFi.status() != WL_CONNECTED && millis() - t0 < 10000) {
    DEBUG_PRINTLN("Connecting to WiFi...");
    delay(500);
  }
  if (WiFi.status() == WL_CONNECTED) {
    DEBUG_PRINT("WiFi IP: "); DEBUG_PRINTLN(WiFi.localIP());
  } else {
    DEBUG_PRINTLN("WiFi failed");
  }
  udp.begin(123);  // NTP port
}

void printMacAddress() {
  String mac = WiFi.macAddress();
  DEBUG_PRINT("WiFi MAC: "); DEBUG_PRINTLN(mac);
}

// —————— PPS Setup ——————
void IRAM_ATTR PPS_ISR() { PPSsignal = true; }

void initPPS() {
  pinMode(PPS_PIN, INPUT);
  unsigned long t0 = millis();
  while (millis() - t0 < 1500) {
    if (digitalRead(PPS_PIN) == HIGH) {
      PPSavailable = true;
      break;
    }
    delay(1);
  }
  if (PPSavailable) {
    DEBUG_PRINTLN("PPS detected");
    attachInterrupt(digitalPinToInterrupt(PPS_PIN), PPS_ISR, RISING);
  } else {
    DEBUG_PRINTLN("No PPS");
  }
}

// —————— GPS Init & Time Sync ——————
void initGPS() {
  gpsSerial.begin(GPSBaud, SERIAL_8N1, GPS_RX_PIN, GPS_TX_PIN);
  unsigned long t0 = millis();
  while (millis() - t0 < 30000) {
    while (gpsSerial.available()) {
      gps.encode(gpsSerial.read());
      if (gps.time.isValid() && gps.date.isValid()) {
        DEBUG_PRINT("GPS time acquired: ");
        if (gps.time.hour() < 10) DEBUG_PRINT('0');
        DEBUG_PRINT(gps.time.hour()); DEBUG_PRINT(':');
        if (gps.time.minute() < 10) DEBUG_PRINT('0');
        DEBUG_PRINT(gps.time.minute()); DEBUG_PRINT(':');
        if (gps.time.second() < 10) DEBUG_PRINT('0');
        DEBUG_PRINTLN(gps.time.second());
        return;
      }
    }
    delay(100);
  }
  DEBUG_PRINTLN("GPS time fail");
}

void readGPSTime() {
  while (gpsSerial.available()) {
    gps.encode(gpsSerial.read());
  }
  if (gps.time.isValid() && gps.date.isValid()) {
    struct tm ti = {0};
    ti.tm_year = gps.date.year() - 1900;
    ti.tm_mon  = gps.date.month() - 1;
    ti.tm_mday = gps.date.day();
    ti.tm_hour = gps.time.hour();
    ti.tm_min  = gps.time.minute();
    ti.tm_sec  = gps.time.second();
    time_t unixTime = mktime(&ti) + 1;
    if (PPSavailable) {
      while (!PPSsignal) {}
      PPSsignal = false;
    }
    struct timeval tv = { .tv_sec = unixTime, .tv_usec = CORRECTION_FACTOR };
    settimeofday(&tv, NULL);
    DEBUG_CRITICAL_PRINTLN("GPS time updated");
  }
}

// —————— NTP Handler ——————
void handleNTPRequest() {
  int size = udp.parsePacket();
  if (size >= 48) {
    uint8_t req[48], resp[48];
    udp.read(req, 48);

    // 1) Decode the client’s Transmit Timestamp (T₁) from bytes 40..47
    uint32_t T1_sec  = (uint32_t)req[40]<<24 | (uint32_t)req[41]<<16 | (uint32_t)req[42]<<8 | (uint32_t)req[43];
    uint32_t T1_frac = (uint32_t)req[44]<<24 | (uint32_t)req[45]<<16 | (uint32_t)req[46]<<8 | (uint32_t)req[47];

    // 2) Record server receipt time (T₂)
    struct timeval tv2;
    gettimeofday(&tv2, NULL);
    uint32_t T2_sec  = tv2.tv_sec  + 2208988800UL;
    uint32_t T2_frac = (uint32_t)( (double)tv2.tv_usec * (4294967296.0/1e6) );

    // Build the response header (unchanged)…
    resp[0] = 0x24; resp[1] = 1; resp[2] = req[2]; resp[3] = (uint8_t)-6;
    memset(&resp[4], 0, 8);
    resp[12]='L'; resp[13]='O'; resp[14]='C'; resp[15]='L';

    // Reference Timestamp = T₂
    for (int i = 0; i < 4; i++) resp[16+i] = (T2_sec  >> (24-8*i)) & 0xFF;
    for (int i = 0; i < 4; i++) resp[20+i] = (T2_frac >> (24-8*i)) & 0xFF;

    // Originate Timestamp = client’s T₁
    memcpy(&resp[24], &req[40], 8);

    // 3) Record server transmit time (T₃) immediately before sending
    struct timeval tv3;
    gettimeofday(&tv3, NULL);
    uint32_t T3_sec  = tv3.tv_sec  + 2208988800UL;
    uint32_t T3_frac = (uint32_t)( (double)tv3.tv_usec * (4294967296.0/1e6) );
    for (int i = 0; i < 4; i++) resp[32+i] = (T3_sec  >> (24-8*i)) & 0xFF;
    for (int i = 0; i < 4; i++) resp[36+i] = (T3_frac >> (24-8*i)) & 0xFF;
    for (int i = 0; i < 4; i++) resp[40+i] = (T3_sec  >> (24-8*i)) & 0xFF;
    for (int i = 0; i < 4; i++) resp[44+i] = (T3_frac >> (24-8*i)) & 0xFF;

    // 4) Send the packet
    udp.beginPacket(udp.remoteIP(), udp.remotePort());
    udp.write(resp, 48);
    udp.endPacket();

    // 5) Log all three timestamps to Serial
    Serial.printf("T1(client orig): %10u.%08u\n", T1_sec, T1_frac);
    Serial.printf("T2(server recv): %10u.%08u\n", T2_sec, T2_frac);
    Serial.printf("T3(server send): %10u.%08u\n\n", T3_sec, T3_frac);
  }
}


// —————— Tasks ——————
void gpsTask(void *param) {
  // Initialize GPS UART & OLED once
  gpsSerial.begin(GPSBaud, SERIAL_8N1, GPS_RX_PIN, GPS_TX_PIN);
  u8g2.begin();
  u8g2.setContrast(255);
  u8g2.setBusClock(400000);
  u8g2.setFont(u8g2_font_6x10_tr);
  u8g2.setFontPosTop();

  for (;;) {
    readGPSTime();
    displayTime();              // show synced time instead of lat/lon
    vTaskDelay(pdMS_TO_TICKS(500));
  }
}

void ntpTask(void *param) {
  for (;;) {
    handleNTPRequest();
    vTaskDelay(pdMS_TO_TICKS(100));
  }
}

void setup() {
  Serial.begin(115200);
  initNetwork();
  printMacAddress();
  initPPS();
  initGPS();

  gpsMutex = xSemaphoreCreateMutex();
  xTaskCreatePinnedToCore(gpsTask, "GPSTask", 4096, NULL, 1, NULL, 0);
  xTaskCreatePinnedToCore(ntpTask, "NTPTask", 4096, NULL, 1, NULL, 0);
}

void loop() {
  // everything handled in tasks
}
