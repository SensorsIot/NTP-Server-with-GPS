#include <WiFi.h>
#include <WiFiUdp.h>
#include <TinyGPSPlus.h>
#include <time.h>
#include <freertos/FreeRTOS.h>
#include <freertos/task.h>
#include <freertos/semphr.h>
#include <Wire.h>
#include <U8g2lib.h>
#include "credentials.h"

// Pin Definitions
#define GPS_RX_PIN 20
#define GPS_TX_PIN 21
#define PPS_PIN 2

// Global Variables
TinyGPSPlus gps;
HardwareSerial gpsSerial(1);
WiFiUDP udp;
SemaphoreHandle_t gpsMutex;

volatile bool PPSsignal = false;
volatile uint32_t last_pps_micros_isr = 0;

volatile uint32_t last_sync_unix_sec = 0;
volatile uint32_t last_sync_micros_at_pps = 0;

volatile uint32_t latest_gps_unix_second = 0;
volatile bool latest_gps_second_valid = false;

volatile bool PPSavailable = false;

#define GPSBaud 9600
#define CORRECTION_FACTOR 0

static const uint32_t NTP_OFFSET = 2208988800UL;

struct PreciseTime {
  uint32_t seconds;
  uint32_t microseconds;
};

struct PreciseDateTime {
  struct tm timeinfo;
  uint32_t microseconds;
};

// OLED Setup
U8G2_SSD1306_72X40_ER_F_HW_I2C u8g2(
  /* Rotation */ U8G2_R0,
  /* CS */ U8X8_PIN_NONE,
  /* SCL=*/6,
  /* SDA=*/5);


// Function Prototypes
void initNetwork();
void printMacAddress();
void initPPS();
void IRAM_ATTR handlePPSRisingEdge();
void initGPS();
void processGPS();
void syncTimeWithGPS();
void handleNTPRequest();
uint32_t dateToEpoch(uint8_t d, uint8_t m, uint16_t y);
void centerText(const char* text, int y);


PreciseTime getPreciseTime(uint32_t current_micros, uint32_t last_sync_sec, uint32_t last_sync_micros_at_pps) {
  PreciseTime pt;

  // Utilize PPS-aligned calculation only if PPS is available and a synchronization point exists.
  if (PPSavailable && last_sync_micros_at_pps != 0) {
    uint64_t elapsed_micros_64;

    if (current_micros >= last_sync_micros_at_pps) {
      elapsed_micros_64 = (uint64_t)current_micros - last_sync_micros_at_pps;
    } else {
      // Handle micros() counter rollover.
      elapsed_micros_64 = (uint64_t)(0xFFFFFFFF - last_sync_micros_at_pps) + current_micros + 1;
    }

    pt.seconds = last_sync_sec + (uint32_t)(elapsed_micros_64 / 1000000);
    pt.microseconds = (uint32_t)(elapsed_micros_64 % 1000000);
  } else {
    // Fallback to coarse system time if PPS is not available or not yet synchronized.
    struct timeval tv_current;
    gettimeofday(&tv_current, NULL);
    pt.seconds = (uint32_t)tv_current.tv_sec;
    pt.microseconds = (uint32_t)tv_current.tv_usec;
  }
  return pt;
}

PreciseDateTime getPreciseDateTime() {
  PreciseDateTime pdt;
  pdt.microseconds = 0;
  memset(&pdt.timeinfo, 0, sizeof(pdt.timeinfo));

  // Safely acquire mutex to read synchronized time values.
  if (xSemaphoreTake(gpsMutex, pdMS_TO_TICKS(50)) == pdTRUE) {
    PreciseTime pt = getPreciseTime(micros(), last_sync_unix_sec, last_sync_micros_at_pps);

    // Cast `pt.seconds` to `time_t` for `gmtime_r` to ensure correct interpretation.
    time_t current_unix_time = (time_t)pt.seconds;
    gmtime_r(&current_unix_time, &pdt.timeinfo);

    pdt.microseconds = pt.microseconds;
    xSemaphoreGive(gpsMutex);
  } else {
    // If mutex acquisition fails, fall back to system's `gettimeofday()`.
    Serial.println("[getPreciseDateTime] Warning: Could not take GPS mutex. Using system clock fallback or stale values.");
    struct timeval tv_fallback;
    gettimeofday(&tv_fallback, NULL);
    time_t fallback_unix_time = (time_t)tv_fallback.tv_sec;
    gmtime_r(&fallback_unix_time, &pdt.timeinfo);
    pdt.microseconds = (uint32_t)tv_fallback.tv_usec;
  }
  return pdt;
}

// FreeRTOS Tasks

void gpsTask(void *param) {
  Serial.println("[gpsTask] Starting GPS task...");
  for (;;) {
    // Periodically process GPS data and synchronize time.
    if (xSemaphoreTake(gpsMutex, pdMS_TO_TICKS(100)) == pdTRUE) {
      processGPS();
      syncTimeWithGPS();
      xSemaphoreGive(gpsMutex);
    } else {
      Serial.println("[gpsTask] Warning: Could not take GPS mutex, skipping GPS/sync cycle.");
    }
    vTaskDelay(pdMS_TO_TICKS(5));
  }
}

// Helper function to center text on the display
// Helper function to center text on the display
void centerText(const char* text, int y) {
  int textWidth = u8g2.getStrWidth(text);
  int xPos = (u8g2.getDisplayWidth() - textWidth) / 2;
  u8g2.setCursor(xPos, y);
  u8g2.print(text);
}

void displayTask(void *param) {
  Serial.println("[displayTask] Starting display task...");
  for (;;) {
    PreciseDateTime current_pdt = getPreciseDateTime();

    char time_str[10]; // Sufficient for HH:MM:SS
    char date_str[15]; // Sufficient for YY-MM-DD

    // Format time (HH:MM:SS) - removed "UTC"
    snprintf(time_str, sizeof(time_str), "%02d:%02d:%02d",
             current_pdt.timeinfo.tm_hour, current_pdt.timeinfo.tm_min,
             current_pdt.timeinfo.tm_sec);
    // Full time with microseconds for Serial output (still useful for debugging)
    Serial.printf("[displayTask] Current Time: %02d:%02d:%02d.%06lu UTC\n",
                  current_pdt.timeinfo.tm_hour, current_pdt.timeinfo.tm_min,
                  current_pdt.timeinfo.tm_sec, current_pdt.microseconds);

    // Format date (YY-MM-DD)
    snprintf(date_str, sizeof(date_str), "%02d-%02d-%02d",
             current_pdt.timeinfo.tm_year % 100, current_pdt.timeinfo.tm_mon + 1,
             current_pdt.timeinfo.tm_mday);
    Serial.printf("[displayTask] Current Date: %02d-%02d-%02d\n",
                  current_pdt.timeinfo.tm_year % 100, current_pdt.timeinfo.tm_mon + 1,
                  current_pdt.timeinfo.tm_mday);

    // Clear the display buffer
    u8g2.clearBuffer();

    // Draw a rounded rectangle border around the entire display
    u8g2.drawRFrame(0, 0, u8g2.getDisplayWidth(), u8g2.getDisplayHeight(), 5);

    // Calculate vertical positions to center two lines of text within the border
    int max_line_height = u8g2.getMaxCharHeight();
    int total_text_height = max_line_height * 2 + 2; // Two lines plus a small gap

    int yPos_time = (u8g2.getDisplayHeight() - total_text_height) / 2 + 2;
    int yPos_date = yPos_time + max_line_height + 2;

    // Draw the time and date, centered
    centerText(time_str, yPos_time);
    centerText(date_str, yPos_date);

    // Send buffer to the display
    u8g2.sendBuffer();

    vTaskDelay(pdMS_TO_TICKS(500)); // Update display and serial every 500ms
  }
}

// NTP Request Handling
void handleNTPRequest() {
  int size = udp.parsePacket();
  if (size >= 48) {
    uint8_t req[48];
    uint8_t resp[48];

    udp.read(req, 48);
    uint32_t T2_raw_micros = micros();

    memset(resp, 0, 48);

    resp[0] = (req[0] & 0x38) | 0x04;
    resp[1] = 1; // Stratum 1
    resp[2] = 6; // Poll Interval
    resp[3] = -20; // Precision

    resp[12] = 'G';
    resp[13] = 'P';
    resp[14] = 'S';
    resp[15] = ' '; // Reference Identifier

    // Acquire mutex to get synchronized time for NTP timestamps.
    PreciseTime t0_precise, t2_precise, t3_precise;
    if (xSemaphoreTake(gpsMutex, pdMS_TO_TICKS(10)) == pdTRUE) {
      t0_precise = getPreciseTime(last_sync_micros_at_pps, last_sync_unix_sec, last_sync_micros_at_pps);
      t2_precise = getPreciseTime(T2_raw_micros, last_sync_unix_sec, last_sync_micros_at_pps);
      uint32_t T3_raw_micros_temp = micros();
      t3_precise = getPreciseTime(T3_raw_micros_temp, last_sync_unix_sec, last_sync_micros_at_pps);
      xSemaphoreGive(gpsMutex);
    } else {
      Serial.println("[NTPTask] Warning: Could not get mutex for NTP timestamps. Using stale values or failing.");
      t0_precise.seconds = last_sync_unix_sec;
      t0_precise.microseconds = 0;
      t2_precise = getPreciseTime(T2_raw_micros, last_sync_unix_sec, last_sync_micros_at_pps);
      t3_precise = getPreciseTime(micros(), last_sync_unix_sec, last_sync_micros_at_pps);
    }

    // Populate NTP response packet with calculated timestamps.
    uint32_t T0_sec = t0_precise.seconds + NTP_OFFSET;
    uint32_t T0_frac = (uint32_t)((double)t0_precise.microseconds * (4294967296.0 / 1e6));
    resp[16] = (T0_sec >> 24) & 0xFF; resp[17] = (T0_sec >> 16) & 0xFF; resp[18] = (T0_sec >> 8) & 0xFF; resp[19] = T0_sec & 0xFF;
    resp[20] = (T0_frac >> 24) & 0xFF; resp[21] = (T0_frac >> 16) & 0xFF; resp[22] = (T0_frac >> 8) & 0xFF; resp[23] = T0_frac & 0xFF;

    memcpy(&resp[24], &req[40], 8); // Copy client's T1 to Originate Timestamp

    uint32_t T2_sec = t2_precise.seconds + NTP_OFFSET;
    uint32_t T2_frac = (uint32_t)((double)t2_precise.microseconds * (4294967296.0 / 1e6));
    resp[32] = (T2_sec >> 24) & 0xFF; resp[33] = (T2_sec >> 16) & 0xFF; resp[34] = (T2_sec >> 8) & 0xFF; resp[35] = T2_sec & 0xFF;
    resp[36] = (T2_frac >> 24) & 0xFF; resp[37] = (T2_frac >> 16) & 0xFF; resp[38] = (T2_frac >> 8) & 0xFF; resp[39] = T2_frac & 0xFF;

    uint32_t T3_sec = t3_precise.seconds + NTP_OFFSET;
    uint32_t T3_frac = (uint32_t)((double)t3_precise.microseconds * (4294967296.0 / 1e6));
    resp[40] = (T3_sec >> 24) & 0xFF; resp[41] = (T3_sec >> 16) & 0xFF; resp[42] = (T3_sec >> 8) & 0xFF; resp[43] = T3_sec & 0xFF;
    resp[44] = (T3_frac >> 24) & 0xFF; resp[45] = (T3_frac >> 16) & 0xFF; resp[46] = (T3_frac >> 8) & 0xFF; resp[47] = T3_frac & 0xFF;

    udp.beginPacket(udp.remoteIP(), udp.remotePort());
    udp.write(resp, 48);
    udp.endPacket();

    char buf_t2[32], buf_t3[32];
    time_t t2_unix = t2_precise.seconds;
    time_t t3_unix = t3_precise.seconds;
    strftime(buf_t2, sizeof(buf_t2), "%H:%M:%S", gmtime(&t2_unix));
    strftime(buf_t3, sizeof(buf_t3), "%H:%M:%S", gmtime(&t3_unix));
    Serial.printf("[NTPTask] NTP Req. T2: %s.%06lu UTC, T3: %s.%06lu UTC\n", buf_t2, (unsigned long)t2_precise.microseconds, buf_t3, (unsigned long)t3_precise.microseconds);
  }
}

void ntpTask(void *param) {
  Serial.println("[NTPTask] Starting NTP task...");
  for (;;) {
    handleNTPRequest();
    vTaskDelay(pdMS_TO_TICKS(10));
  }
}

// Network Initialization
void initNetwork() {
  Serial.println("[initNetwork] Initializing network...");
  WiFi.mode(WIFI_STA);
  WiFi.begin(ssid, password);
  unsigned long t0 = millis();
  Serial.println("[initNetwork] Connecting to WiFi...");
  while (WiFi.status() != WL_CONNECTED && millis() - t0 < 20000) {
    delay(500);
    u8g2.clearBuffer();
    centerText("Connecting", u8g2.getDisplayHeight() / 2 - u8g2.getMaxCharHeight() - 2);
    centerText(ssid, u8g2.getDisplayHeight() / 2 + 2);
    u8g2.sendBuffer();
  }
  if (WiFi.status() == WL_CONNECTED) {
    Serial.print("[initNetwork] WiFi IP: ");
    Serial.println(WiFi.localIP());
    u8g2.clearBuffer();
    centerText("Connected!", u8g2.getDisplayHeight() / 2 - u8g2.getMaxCharHeight() - 2);
    char ip_str[20];
    WiFi.localIP().toString().toCharArray(ip_str, sizeof(ip_str));
    centerText(ip_str, u8g2.getDisplayHeight() / 2 + 2);
    u8g2.sendBuffer();
    delay(2000);
    Serial.println("[initNetwork] WiFi connected.");
  } else {
    Serial.println("[initNetwork] WiFi failed to connect!");
    u8g2.clearBuffer();
    centerText("WiFi Failed!", u8g2.getDisplayHeight() / 2 - u8g2.getMaxCharHeight() / 2);
    u8g2.sendBuffer();
  }
  udp.begin(123);
  Serial.println("[initNetwork] NTP server started on port 123");
}

void printMacAddress() {
  Serial.print("[printMacAddress] WiFi MAC: ");
  Serial.println(WiFi.macAddress());
}

// PPS Detection & ISR
void IRAM_ATTR handlePPSRisingEdge() {
  PPSsignal = true;
  last_pps_micros_isr = micros();
}

void initPPS() {
  pinMode(PPS_PIN, INPUT_PULLDOWN);
  unsigned long t0 = millis();
  bool detected = false;
  Serial.print("[initPPS] Checking for PPS signal on pin "); Serial.print(PPS_PIN); Serial.println("...");
  while (millis() - t0 < 2000) {
    if (digitalRead(PPS_PIN) == HIGH) {
      detected = true;
      break;
    }
    delay(1);
  }
  if (detected) {
    attachInterrupt(digitalPinToInterrupt(PPS_PIN), handlePPSRisingEdge, RISING);
    PPSavailable = true;
    Serial.println("[initPPS] PPS signal detected. Rising edge interrupt attached.");
  } else {
    PPSavailable = false;
    Serial.println("[initPPS] No PPS signal detected or pin is not connected.");
    Serial.println("[initPPS] Will rely solely on GPS NMEA time (less accurate).");
  }
}

// GPS Initialization & Time Sync
void initGPS() {
  gpsSerial.begin(GPSBaud, SERIAL_8N1, GPS_RX_PIN, GPS_TX_PIN);
  unsigned long t0 = millis();
  Serial.println("[initGPS] Waiting for initial GPS fix...");
  while (millis() - t0 < 60000) {
    while (gpsSerial.available()) {
      gps.encode(gpsSerial.read());
      if (gps.time.isValid() && gps.date.isValid()) {
        Serial.printf("[initGPS] Initial GPS fix acquired. Time: %02d:%02d:%02d\n", gps.time.hour(), gps.time.minute(), gps.time.second());
        Serial.printf("[initGPS] Date: %02d/%02d/%04d\n", gps.date.day(), gps.date.month(), gps.date.year());

        uint8_t d = gps.date.day();
        uint8_t m = gps.date.month();
        uint16_t y = gps.date.year();
        uint32_t days = dateToEpoch(d, m, y);
        latest_gps_unix_second = days * 86400UL + gps.time.hour() * 3600UL + gps.time.minute() * 60UL + gps.time.second();
        latest_gps_second_valid = true;

        if (xSemaphoreTake(gpsMutex, pdMS_TO_TICKS(5000)) == pdTRUE) {
          last_sync_unix_sec = latest_gps_unix_second;
          xSemaphoreGive(gpsMutex);
        } else {
          Serial.println("[initGPS] Warning: Failed to acquire mutex for initial time set after fix.");
        }

        struct timeval tv = { .tv_sec = (time_t)latest_gps_unix_second, .tv_usec = CORRECTION_FACTOR };
        settimeofday(&tv, NULL);
        struct timeval tv_check;
        gettimeofday(&tv_check, NULL);
        Serial.printf("[initGPS] System time initialized via NMEA. Set to %lu.%06lu\n", (unsigned long)tv_check.tv_sec, (unsigned long)tv_check.tv_usec);
        return;
      }
    }
    delay(100);
  }
  Serial.println("[initGPS] Initial GPS time acquisition failed (no fix within timeout).");
}

void processGPS() {
  while (gpsSerial.available()) {
    gps.encode(gpsSerial.read());
  }

  if (gps.time.isValid() && gps.date.isValid()) {
    uint8_t d = gps.date.day();
    uint8_t m = gps.date.month();
    uint16_t y = gps.date.year();
    uint32_t days = dateToEpoch(d, m, y);
    latest_gps_unix_second = days * 86400UL + gps.time.hour() * 3600UL + gps.time.minute() * 60UL + gps.time.second();
    latest_gps_second_valid = true;
  } else {
    latest_gps_second_valid = false;
  }
}

void syncTimeWithGPS() {
  // This function is called within a mutex-protected section in `gpsTask`.
  // It handles synchronization logic based on PPS and GPS data.

  if (PPSsignal && PPSavailable && latest_gps_second_valid) {
    PPSsignal = false;

    // Update the precise synchronization point using GPS NMEA and PPS.
    last_sync_unix_sec = latest_gps_unix_second + 1;
    last_sync_micros_at_pps = last_pps_micros_isr;

    // Set the system's time for OS-level time functions.
    struct timeval tv = { .tv_sec = (time_t)last_sync_unix_sec, .tv_usec = 0 };
    settimeofday(&tv, NULL);

    Serial.printf("[syncTimeWithGPS] GPS+PPS Sync: Unix Sec: %lu, Micros at PPS: %lu\n", last_sync_unix_sec, last_sync_micros_at_pps);
  } else if (PPSsignal && PPSavailable && !latest_gps_second_valid) {
    // If PPS is present but GPS NMEA is lost, increment Unix time based on PPS only.
    PPSsignal = false;
    Serial.println("[syncTimeWithGPS] PPS available but GPS fix lost. Cannot sync time precisely with NMEA.");
    if (last_sync_unix_sec != 0) {
      last_sync_unix_sec++;
      last_sync_micros_at_pps = last_pps_micros_isr;
      struct timeval tv = { .tv_sec = (time_t)last_sync_unix_sec, .tv_usec = 0 };
      settimeofday(&tv, NULL);
      Serial.printf("[syncTimeWithGPS] PPS only (NMEA lost): Unix Sec: %lu, Micros at PPS: %lu\n", last_sync_unix_sec, last_sync_micros_at_pps);
    }
  } else if (!PPSavailable && latest_gps_second_valid) {
    // If PPS is unavailable, synchronize solely based on GPS NMEA data.
    struct timeval tv = { .tv_sec = (time_t)latest_gps_unix_second, .tv_usec = CORRECTION_FACTOR };
    settimeofday(&tv, NULL);
    struct timeval tv_check;
    gettimeofday(&tv_check, NULL);

    // Update synchronization point using current micros() for NMEA-only sync.
    last_sync_unix_sec = tv_check.tv_sec;
    last_sync_micros_at_pps = micros();
    Serial.printf("[syncTimeWithGPS] NMEA Only Sync: Unix Sec: %lu.%06lu\n", last_sync_unix_sec, tv_check.tv_usec);
  }
}

uint32_t dateToEpoch(uint8_t d, uint8_t m, uint16_t y) {
  int Y = y;
  int M = m;
  if (M < 3) {
    Y--;
    M += 12;
  }
  uint32_t era = Y / 400;
  uint32_t yoe = Y - era * 400;
  uint32_t doy = (153 * (M - 3) + 2) / 5 + d - 1;
  uint32_t doe = yoe * 365 + yoe / 4 - yoe / 100 + doy;
  return era * 146097 + doe - 719468;
}

// Arduino Setup and Loop

void setup() {
  Serial.begin(115200);
  Serial.println("\n[setup] Starting setup...");
  Serial.printf("[DEBUG] micros() at start of setup: %lu\n", micros());

  // Initialize FreeRTOS mutex for thread-safe access to GPS time data.
  gpsMutex = xSemaphoreCreateMutex();
  if (gpsMutex != NULL) {
    Serial.println("[setup] GPS Mutex created successfully.");
  } else {
    Serial.println("[setup] ERROR: Failed to create GPS Mutex! This is critical and will cause crashes.");
    while (true);
  }

  // Initialize OLED display.
  u8g2.begin();
  u8g2.setContrast(50); // Set brightness based on your example
  u8g2.setBusClock(400000);
  u8g2.setFont(u8g2_font_6x10_tr);
  u8g2.setFontPosTop();
  Serial.println("[setup] OLED initialized.");

  // Display welcome message with border
  u8g2.clearBuffer();
  u8g2.drawRFrame(0, 0, u8g2.getDisplayWidth(), u8g2.getDisplayHeight(), 5); // ADDED THIS LINE FOR THE WELCOME BORDER
  centerText("NTP", u8g2.getDisplayHeight() / 2 - u8g2.getMaxCharHeight() - 2);
  centerText("Server", u8g2.getDisplayHeight() / 2 + 2);
  u8g2.sendBuffer();
  delay(1500); // Show welcome message for a bit

  // Configure and connect to Wi-Fi, and start UDP listener for NTP.
  initNetwork();
  printMacAddress();

  // Detect PPS signal presence and attach interrupt if found.
  initPPS();

  Serial.printf("[DEBUG] micros() before initGPS loop: %lu\n", micros());
  // Initialize GPS serial communication and obtain initial time fix.
  initGPS();

  // Create FreeRTOS tasks for GPS processing, NTP serving, and display updates.
  xTaskCreatePinnedToCore(
    gpsTask,
    "GPSTask",
    4096,
    NULL,
    configMAX_PRIORITIES - 1, // Highest priority for critical time sync
    NULL,
    0);
  Serial.println("[setup] GPSTask created.");

  xTaskCreatePinnedToCore(
    ntpTask,
    "NTPTask",
    4096,
    NULL,
    configMAX_PRIORITIES - 2, // High priority for responsive NTP service
    NULL,
    0);
  Serial.println("[setup] NTPTask created.");

  xTaskCreatePinnedToCore(
    displayTask,
    "DisplayTask",
    2048,
    NULL,
    configMAX_PRIORITIES - 3, // Lower priority for display updates
    NULL,
    0);
  Serial.println("[setup] DisplayTask created.");

  Serial.println("[setup] Setup complete. Tasks are running.");
}

void loop() {
  // This loop is intentionally empty as all primary logic is handled by FreeRTOS tasks.
  // The default loop task is deleted to save resources.
  vTaskDelete(NULL);
}