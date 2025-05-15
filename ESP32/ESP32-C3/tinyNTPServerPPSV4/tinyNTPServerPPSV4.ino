#include <WiFi.h>
#include <WiFiUdp.h>
#include <TinyGPSPlus.h>
#include <time.h>
#include <EEPROM.h> // Not used in this version, but kept from original
#include <freertos/FreeRTOS.h>
#include <freertos/task.h>
#include <freertos/semphr.h>
#include <Wire.h>
#include <U8g2lib.h>
#include "credentials.h" // defines ssid, password

// —————— Pin Definitions ——————
#define GPS_RX_PIN    20    // ESP32-C3 RX1 ← GPS TX
#define GPS_TX_PIN    21    // ESP32-C3 TX1 → GPS RX (optional, not strictly needed for reading)
#define PPS_PIN       2     // Pulse-per-second input (expect external pull-down)

// —————— Globals ——————
TinyGPSPlus      gps;
HardwareSerial gpsSerial(1); // Use Serial port 1 for GPS
WiFiUDP          udp;
SemaphoreHandle_t gpsMutex; // Mutex to protect GPS data access

volatile bool    PPSsignal    = false; // Flag set by PPS ISR
volatile uint32_t last_pps_micros = 0; // Timestamp of the last PPS pulse (rising edge) in microseconds

// Global variables to store the time when the system clock was last updated by GPS/PPS
// Note: last_sync_unix_usec will now store the micros() value at the last PPS pulse
volatile uint32_t last_sync_unix_sec = 0; // Unix epoch seconds corresponding to last_pps_micros
volatile uint32_t last_sync_micros_at_pps = 0; // micros() timestamp at the last PPS pulse

// Store the latest valid Unix second obtained from GPS NMEA data
volatile uint32_t latest_gps_unix_second = 0;
volatile bool     latest_gps_second_valid = false; // Flag to indicate if latest_gps_unix_second is valid

// Flag to indicate if PPS signal was detected during initialization
volatile bool     PPSavailable = false; // Ensure this is declared globally

#define GPSBaud           9600
#define CORRECTION_FACTOR 0 // Microsecond correction offset if needed (can be tuned)

// NTP epoch offset (seconds from 1900 to 1970)
static const uint32_t NTP_OFFSET = 2208988800UL;

// Structure to hold precise time (seconds and microseconds)
struct PreciseTime {
    uint32_t seconds;
    uint32_t microseconds;
};

// Structure to hold precise date and time components
struct PreciseDateTime {
    struct tm timeinfo;
    uint32_t microseconds;
};

// —————— OLED Setup ——————
// Adjust these offsets based on your display and desired position
const int xOffset = 30;
const int yOffset = 25;
// U8g2 Constructor: U8G2_SSD1306_128X64_NONAME_F_HW_I2C
// Parameters: rotation, reset, clock, data
U8G2_SSD1306_128X64_NONAME_F_HW_I2C u8g2(
  U8G2_R0, U8X8_PIN_NONE, // No reset pin
  /* SCL=*/ 6, /* SDA=*/ 5 // I2C pins for ESP32-C3 (check your board's pinout)
);

// —————— Function Prototypes ——————
void initNetwork();
void printMacAddress();
void initPPS();
void IRAM_ATTR handlePPSRisingEdge(); // ISR for PPS rising edge
void initGPS();
void processGPS(); // Function to read and process GPS data
void syncTimeWithGPS(); // Function to synchronize time using GPS/PPS
void handleNTPRequest(); // Moved definition below
void displayTime();
uint32_t dateToEpoch(uint8_t d, uint8_t m, uint16_t y); // Helper function for date conversion

/**
 * @brief Calculates the current precise time (Unix epoch seconds and microseconds)
 * based on the last known PPS synchronization point and the current micros() value.
 *
 * This function leverages the high-resolution micros() timer, synchronized to the
 * PPS rising edge, to determine the current time with microsecond precision.
 * It accounts for micros() rollover.
 *
 * @param current_micros The current value of the micros() timer.
 * @param last_sync_sec The Unix epoch second at the moment of the last PPS synchronization.
 * @param last_sync_micros The micros() timestamp at the moment of the last PPS synchronization.
 * @return PreciseTime A struct containing the calculated current Unix epoch seconds and microseconds.
 */
PreciseTime getPreciseTime(uint32_t current_micros, uint32_t last_sync_sec, uint32_t last_sync_micros) {
    PreciseTime pt;
    uint32_t elapsed_micros = 0;

    if (current_micros >= last_sync_micros) {
        elapsed_micros = current_micros - last_sync_micros;
    } else {
        // Handle micros() rollover
        elapsed_micros = (0xFFFFFFFF - last_sync_micros) + current_micros + 1;
    }

    pt.seconds = last_sync_sec + (elapsed_micros / 1000000);
    pt.microseconds = elapsed_micros % 1000000;

    return pt;
}

/**
 * @brief Calculates the current precise date and time components (UTC).
 *
 * This function uses getPreciseTime to get the current Unix epoch time with
 * microsecond precision and then converts the seconds part into a tm structure
 * for date and time components (hour, minute, second, day, month, year).
 *
 * @return PreciseDateTime A struct containing the date/time components in a tm struct
 * and the microsecond part.
 */
PreciseDateTime getPreciseDateTime() {
    PreciseDateTime pdt;
    PreciseTime pt = getPreciseTime(micros(), last_sync_unix_sec, last_sync_micros_at_pps);

    // Convert seconds to UTC time structure
    gmtime_r((time_t*)&pt.seconds, &pdt.timeinfo);
    pdt.microseconds = pt.microseconds;

    return pdt;
}


// —————— FreeRTOS Tasks ——————

// Task to read GPS data and update the display and synchronize time
void gpsTask(void *param) {
  for(;;) { // Infinite loop for the task
    // Try to take the mutex before accessing shared GPS data
    if(xSemaphoreTake(gpsMutex, pdMS_TO_TICKS(100)) == pdTRUE) { // Wait up to 100ms
      // Process incoming GPS serial data
      processGPS();

      // Attempt to synchronize time if PPS signal was detected
      syncTimeWithGPS();

      // Update the display with the current system time
      displayTime();

      xSemaphoreGive(gpsMutex); // Release the mutex
    } else {
      // Could add a debug print if mutex is not available, but usually not necessary
    }
    // Adjust delay for how often you want to check GPS and update display/time
    // A shorter delay means more frequent checks and potential time updates.
    vTaskDelay(pdMS_TO_TICKS(10)); // Check GPS and update time/display frequently
  }
}

// —————— NTP Request Handling ——————
// Handles incoming NTP requests and sends a response
void handleNTPRequest() {
  int size = udp.parsePacket(); // Check if a UDP packet is available
  if (size >= 48) { // NTP packets are typically 48 bytes
    uint8_t req[48]; // Buffer to hold the incoming request
    uint8_t resp[48]; // Buffer to hold the outgoing response

    udp.read(req, 48); // Read the incoming packet into the buffer

    // Capture Server Receive Timestamp (T2) as early as possible using micros()
    uint32_t T2_raw_micros = micros();

    // --- Fill Response Buffer ---
    memset(resp, 0, 48); // Initialize response buffer with zeros

    // Byte 0: LI (Leap Indicator) | VN (Version Number) | Mode (NTP Mode)
    // LI = 0 (No leap second adjustment)
    // VN = Use the version from the client request (bits 3-5 of req[0])
    // Mode = 4 (Server)
    resp[0] = (req[0] & 0x38) | 0x04; // Keep client's version, set mode to 4

    // Byte 1: Stratum
    // Stratum = 1 (Primary reference clock - synchronized directly to GPS/PPS)
    resp[1] = 1;

    // Byte 2: Poll Interval (log2 of max polling interval in seconds)
    // e.g., 6 for 64 seconds (2^6)
    resp[2] = 6; // Can adjust this based on desired polling frequency

    // Byte 3: Precision (log2 of precision in seconds)
    // e.g., -20 for ~1 microsecond precision (2^-20)
    resp[3] = -20; // Reasonable precision for GPS/PPS source

    // Bytes 4-7: Root Delay (Fixed point 16.16 format)
    // Delay from the primary reference clock. For Stratum 1, this is typically 0.
    // Bytes 8-11: Root Dispersion (Fixed point 16.16 format)
    // Dispersion relative to the primary reference clock. For Stratum 1, this is typically 0.
    // These are already 0 due to memset.

    // Bytes 12-15: Reference Identifier (4 ASCII characters)
    // For Stratum 1, this identifies the reference source (e.g., 'GPS', 'PPS', 'ATOM').
    resp[12] = 'G'; resp[13] = 'P'; resp[14] = 'S'; resp[15] = ' '; // Indicate GPS source

    // Bytes 16-23: Reference Timestamp (T0)
    // Time when the system clock was last set or corrected by the reference source.
    // Use the last time the system clock's second was updated by GPS/PPS (last_sync_unix_sec)
    // and the micros() timestamp at that moment (last_sync_micros_at_pps).
    // Convert Unix epoch time (seconds since 1970) to NTP epoch time (seconds since 1900).
    uint32_t T0_sec = last_sync_unix_sec + NTP_OFFSET;
    // Convert microseconds from last_sync_micros_at_pps (relative to the start of the second)
    // to NTP fractional seconds (2^32 fractions per second).
    // Note: last_sync_micros_at_pps is the absolute micros() value, we need the fractional part.
    // The fractional part is the time elapsed since the start of the second (which is last_sync_micros_at_pps itself).
    uint32_t T0_frac = (uint32_t)((double)(last_sync_micros_at_pps % 1000000) * (4294967296.0 / 1e6));


    // Write T0 (seconds and fractional seconds) into the response buffer (Big-endian)
    resp[16] = (T0_sec >> 24) & 0xFF;
    resp[17] = (T0_sec >> 16) & 0xFF;
    resp[18] = (T0_sec >> 8) & 0xFF;
    resp[19] = T0_sec & 0xFF;
    resp[20] = (T0_frac >> 24) & 0xFF;
    resp[21] = (T0_frac >> 16) & 0xFF;
    resp[22] = (T0_frac >> 8) & 0xFF;
    resp[23] = T0_frac & 0xFF;

    // Bytes 24-31: Originate Timestamp (T1')
    // This is the server's copy of the client's Transmit Timestamp (T1) from the request.
    // T1 is located at bytes 40-47 in the client's request.
    // Copy the 8 bytes of T1 from the request (req[40-47]) to the response (resp[24-31]).
    memcpy(&resp[24], &req[40], 8);

    // Bytes 32-39: Receive Timestamp (T2)
    // Calculate T2 using the precise time function
    PreciseTime t2_precise = getPreciseTime(T2_raw_micros, last_sync_unix_sec, last_sync_micros_at_pps);

    // Add NTP_OFFSET to get NTP epoch second
    uint32_t T2_sec = t2_precise.seconds + NTP_OFFSET;
    uint32_t T2_usec = t2_precise.microseconds;
    uint32_t T2_frac = (uint32_t)((double)T2_usec * (4294967296.0 / 1e6));

    // Write T2 (seconds and fractional seconds) into the response buffer (Big-endian)
    resp[32] = (T2_sec >> 24) & 0xFF;
    resp[33] = (T2_sec >> 16) & 0xFF;
    resp[34] = (T2_sec >> 8) & 0xFF;
    resp[35] = T2_sec & 0xFF;
    resp[36] = (T2_frac >> 24) & 0xFF;
    resp[37] = (T2_frac >> 16) & 0xFF;
    resp[38] = (T2_frac >> 8) & 0xFF;
    resp[39] = T2_frac & 0xFF;

    // Bytes 40-47: Transmit Timestamp (T3)
    // Capture the current time just before sending the packet using micros().
    uint32_t T3_raw_micros = micros();

    // Calculate T3 using the precise time function
    PreciseTime t3_precise = getPreciseTime(T3_raw_micros, last_sync_unix_sec, last_sync_micros_at_pps);

    // Add NTP_OFFSET to get NTP epoch second
    uint32_t T3_sec = t3_precise.seconds + NTP_OFFSET;
    uint32_t T3_usec = t3_precise.microseconds;
    uint32_t T3_frac = (uint32_t)((double)T3_usec * (4294967296.0 / 1e6));

    // Write T3 (seconds and fractional seconds) into the response buffer (Big-endian)
    resp[40] = (T3_sec >> 24) & 0xFF;
    resp[41] = (T3_sec >> 16) & 0xFF;
    resp[42] = (T3_sec >> 8) & 0xFF;
    resp[43] = T3_sec & 0xFF;
    resp[44] = (T3_frac >> 24) & 0xFF;
    resp[45] = (T3_frac >> 16) & 0xFF;
    resp[46] = (T3_frac >> 8) & 0xFF;
    resp[47] = T3_frac & 0xFF;

    // --- Send the response packet ---
    // Begin sending packet to the client's IP and port
    udp.beginPacket(udp.remoteIP(), udp.remotePort());
    udp.write(resp, 48); // Write the 48-byte response buffer
    udp.endPacket(); // Finish and send the packet

    // Only print T2 and T3 for debugging NTP timing
    char buf_t2[32], buf_t3[32];
    time_t t2_unix = t2_precise.seconds; // Use Unix epoch for gmtime
    time_t t3_unix = t3_precise.seconds; // Use Unix epoch for gmtime
    strftime(buf_t2, sizeof(buf_t2), "%H:%M:%S", gmtime(&t2_unix));
    strftime(buf_t3, sizeof(buf_t3), "%H:%M:%S", gmtime(&t3_unix));
    Serial.printf("NTP Request Handled. T2: %s.%06lu UTC, T3: %s.%06lu UTC\n", buf_t2, (unsigned long)T2_usec, buf_t3, (unsigned long)T3_usec);
  }
}


// Task to handle incoming NTP requests
void ntpTask(void *param) {
  for(;;) { // Infinite loop for the task
    handleNTPRequest(); // Check for and handle NTP requests
    vTaskDelay(pdMS_TO_TICKS(1)); // Short delay to yield and prevent watchdog timer issues
  }
}


// —————— Setup ——————
void setup() {
  Serial.begin(115200); // Initialize serial for debugging

  // Initialize OLED display
  u8g2.begin();
  u8g2.setContrast(255); // Max contrast
  u8g2.setBusClock(400000); // Fast I2C
  u8g2.setFont(u8g2_font_6x10_tr); // Set font
  u8g2.setFontPosTop(); // Set font position

  initNetwork();
  printMacAddress();
  initPPS(); // Initialize PPS interrupt
  initGPS(); // Initialize GPS serial and wait for first fix

  // Create mutex for protecting access to shared resources (like GPS data, though less critical with tasks)
  gpsMutex = xSemaphoreCreateMutex();

  // Create FreeRTOS tasks
  // ESP32-C3 has only core 0, so pinning is less critical but good practice
  xTaskCreatePinnedToCore(
    gpsTask,       // Task function
    "GPSTask",     // Task name
    4096,          // Stack size (bytes)
    NULL,          // Parameter to pass
    configMAX_PRIORITIES - 2, // Priority (lower than NTP)
    NULL,          // Task handle
    0              // Core to run on
  );

  xTaskCreatePinnedToCore(
    ntpTask,       // Task function
    "NTPTask",     // Task name
    4096,          // Stack size (bytes)
    NULL,          // Parameter to pass
    configMAX_PRIORITIES - 1, // Priority (higher than GPS)
    NULL,          // Task handle
    0              // Core to run on
  );
}

void loop() {
  // This loop function is part of the default Arduino sketch structure.
  // Since we are using FreeRTOS tasks to handle everything, this loop
  // is not needed and its task is deleted immediately after setup completes.
  vTaskDelete(NULL); // Delete the loop task to free up resources
}

// —————— Network Initialization ——————
void initNetwork() {
  WiFi.mode(WIFI_STA); // Set WiFi to Station mode
  WiFi.begin(ssid, password); // Connect to WiFi
  unsigned long t0 = millis();
  Serial.println("Connecting to WiFi..."); // Use Serial directly
  while (WiFi.status() != WL_CONNECTED && millis() - t0 < 20000) { // Wait up to 20 seconds
    delay(500);
    // Display connection status on OLED
    u8g2.clearBuffer();
    u8g2.setCursor(xOffset, yOffset);
    u8g2.print("Connecting...");
    u8g2.setCursor(xOffset, yOffset + 12);
    u8g2.print(ssid);
    u8g2.sendBuffer();
  }
  if (WiFi.status() == WL_CONNECTED) {
    Serial.print("WiFi IP: "); Serial.println(WiFi.localIP()); // Use Serial directly
    // Display IP on OLED
    u8g2.clearBuffer();
    u8g2.setCursor(xOffset, yOffset);
    u8g2.print("Connected!");
    u8g2.setCursor(xOffset, yOffset + 12);
    u8g2.print(WiFi.localIP());
    u8g2.sendBuffer();
    delay(2000); // Show IP for a few seconds
  } else {
    Serial.println("WiFi failed"); // Use Serial directly
    // Display failure on OLED
    u8g2.clearBuffer();
    u8g2.setCursor(xOffset, yOffset);
    u8g2.print("WiFi Failed!");
    u8g2.sendBuffer();
    // Keep trying to connect in the background or restart? For now, just proceed.
  }
  udp.begin(123); // Start UDP listener on NTP port 123
  Serial.println("NTP server started on port 123"); // Use Serial directly
}

void printMacAddress() {
  Serial.print("WiFi MAC: "); Serial.println(WiFi.macAddress()); // Use Serial directly
}

// —————— PPS Detection & ISR ——————
// Interrupt Service Routine for the rising edge of the Pulse-Per-Second signal
void IRAM_ATTR handlePPSRisingEdge() {
  PPSsignal = true; // Set the flag to indicate a PPS pulse occurred
  last_pps_micros = micros(); // Capture the microsecond timestamp of the pulse
}

// Initialize PPS pin and detect if a signal is present
void initPPS() {
  pinMode(PPS_PIN, INPUT_PULLDOWN); // Configure PPS pin with internal pull-down
  unsigned long t0 = millis();
  bool detected = false;
  // Check for a rising edge within a short period to confirm PPS is connected
  Serial.print("Checking for PPS signal on pin "); Serial.print(PPS_PIN); Serial.println("..."); // Use Serial directly
  while (millis() - t0 < 2000) { // Check for up to 2 seconds
    if (digitalRead(PPS_PIN) == HIGH) {
      detected = true;
      break;
    }
    delay(1); // Small delay to allow other things to run
  }
  if (detected) {
    // Attach interrupt on the rising edge of the PPS signal
    attachInterrupt(digitalPinToInterrupt(PPS_PIN), handlePPSRisingEdge, RISING);
    PPSavailable = true; // Mark PPS as available
    Serial.println("PPS signal detected. Rising edge interrupt attached."); // Use Serial directly
  } else {
    // If no PPS detected at startup, assume it's not available
    PPSavailable = false;
    Serial.println("No PPS signal detected or pin is not connected."); // Use Serial directly
    Serial.println("Will rely solely on GPS NMEA time (less accurate)."); // Use Serial directly
  }
}

// —————— GPS Initialization & Time Sync ——————
// Initialize GPS serial and wait for the first valid time/date fix
void initGPS() {
  gpsSerial.begin(GPSBaud, SERIAL_8N1, GPS_RX_PIN, GPS_TX_PIN); // Start GPS serial communication
  unsigned long t0 = millis();
  Serial.println("Waiting for initial GPS fix..."); // Use Serial directly
  // Wait for a valid date and time from GPS, up to 60 seconds
  while (millis() - t0 < 60000) {
    while (gpsSerial.available()) {
      gps.encode(gpsSerial.read()); // Feed GPS data to TinyGPSPlus
      // Check if both time and date are valid
      if (gps.time.isValid() && gps.date.isValid()) {
        Serial.print("Initial GPS fix acquired. Time: "); // Use Serial directly
        Serial.print(gps.time.hour()); Serial.print(":"); // Use Serial directly
        Serial.print(gps.time.minute()); Serial.print(":"); // Use Serial directly
        Serial.println(gps.time.second()); // Use Serial directly
        Serial.print("Date: "); // Use Serial directly
        Serial.print(gps.date.day()); Serial.print("/"); // Use Serial directly
        Serial.print(gps.date.month()); Serial.print("/"); // Use Serial directly
        Serial.println(gps.date.year()); // Use Serial directly

        // Calculate and store the initial Unix epoch second from NMEA
        uint8_t d = gps.date.day();
        uint8_t m = gps.date.month();
        uint16_t y = gps.date.year();
        uint32_t days = dateToEpoch(d, m, y);
        latest_gps_unix_second = days*86400UL + gps.time.hour()*3600UL + gps.time.minute()*60UL + gps.time.second();
        latest_gps_second_valid = true; // Mark the stored second as valid

        // Initial time sync based on the first valid NMEA data
        // We still set the system time here, but mainly for the second part.
        struct timeval tv = { .tv_sec = (time_t)latest_gps_unix_second, .tv_usec = CORRECTION_FACTOR };
        settimeofday(&tv, NULL);
        struct timeval tv_check;
        gettimeofday(&tv_check, NULL);
        last_sync_unix_sec = tv_check.tv_sec;
        last_sync_micros_at_pps = micros(); // Capture micros at this initial sync point

        Serial.printf("System time initialized via NMEA. Set to %lu.%06lu\n", last_sync_unix_sec, tv_check.tv_usec); // Use Serial directly
        Serial.printf("Time check after initial settimeofday: %lu.%06lu\n", tv_check.tv_sec, tv_check.tv_usec); // Use Serial directly

        return; // Exit the function once initial sync is attempted
      }
    }
    delay(100); // Short delay to avoid blocking
  }
  Serial.println("Initial GPS time acquisition failed (no fix within timeout)."); // Use Serial directly
  // The system time will not be set accurately without a GPS fix.
}

// Process incoming GPS serial data
void processGPS() {
    while (gpsSerial.available()) {
        gps.encode(gpsSerial.read());
    }

    // If we have a valid time and date, update the latest stored second
    if (gps.time.isValid() && gps.date.isValid()) {
        uint8_t d = gps.date.day();
        uint8_t m = gps.date.month();
        uint16_t y = gps.date.year();
        uint32_t days = dateToEpoch(d, m, y);
        latest_gps_unix_second = days*86400UL + gps.time.hour()*3600UL + gps.time.minute()*60UL + gps.time.second();
        latest_gps_second_valid = true; // Mark the stored second as valid
    } else {
        latest_gps_second_valid = false; // Invalidate if fix is lost
    }
}

// Synchronize the system time using the latest GPS data and PPS (if available)
void syncTimeWithGPS() {
    // Only attempt to sync if PPS signal was received AND we have valid GPS second data
    if (PPSsignal && PPSavailable && latest_gps_second_valid) {
        // Reset the flag immediately to avoid processing the same pulse multiple times
        PPSsignal = false;

        // Update the last synchronized second by incrementing the latest NMEA second
        // This assumes the NMEA second corresponds to the previous second relative to the PPS pulse
        last_sync_unix_sec = latest_gps_unix_second + 1;
        last_sync_micros_at_pps = last_pps_micros; // Use the micros captured by the ISR for the start of this second

        // Optional: We can still call settimeofday with the second part
        // to keep the system clock somewhat aligned, but the microsecond
        // precision for NTP responses will come from micros() relative to last_sync_micros_at_pps.
        struct timeval tv = { .tv_sec = (time_t)last_sync_unix_sec, .tv_usec = 0 }; // Set microseconds to 0
        settimeofday(&tv, NULL);

        // Optional: Debug print to confirm time was set (can be removed later)
        // struct timeval tv_check;
        // gettimeofday(&tv_check, NULL);
        // Serial.printf("System time updated via GPS+PPS. Set second to %lu. Micros at PPS: %lu\n", last_sync_unix_sec, last_sync_micros_at_pps);
        // Serial.printf("Time check after settimeofday: %lu.%06lu\n", tv_check.tv_sec, tv_check.tv_usec);
    } else if (PPSavailable && !latest_gps_second_valid) {
        // If PPS is available but GPS fix is lost, indicate this
        // Serial.println("PPS available but GPS fix lost. Cannot sync time precisely.");
        // In this state, we could potentially just increment the last known second
        // based on PPS pulses, but this is less reliable without NMEA data.
        // For now, we won't update last_sync_unix_sec until NMEA is valid again.
        if (PPSsignal) { // Still reset the signal if it fired
             PPSsignal = false;
        }
    } else if (!PPSavailable && latest_gps_second_valid) {
        // If PPS is not available but GPS fix is valid, update time based on NMEA only
        // This prevents the clock from stopping if PPS is not connected/lost.
        // This will be less precise.
        struct timeval tv = { .tv_sec = (time_t)latest_gps_unix_second, .tv_usec = CORRECTION_FACTOR };
        settimeofday(&tv, NULL);
        struct timeval tv_check;
        gettimeofday(&tv_check, NULL);
        last_sync_unix_sec = tv_check.tv_sec;
        last_sync_micros_at_pps = micros(); // Capture micros at this less precise sync point
        // Serial.printf("System time updated via NMEA only (PPS not available). Set to %lu.%06lu\n", last_sync_unix_sec, tv_check.tv_usec);
        // Serial.printf("Time check after settimeofday (NMEA only): %lu.%06lu\n", tv_check.tv_sec, tv_check.tv_usec);
    }
}


// Convert date (day, month, year) to days since Unix epoch (Jan 1, 1970)
// This is a standard algorithm (Fliegel-Van Flandern or similar)
uint32_t dateToEpoch(uint8_t d, uint8_t m, uint16_t y) {
  // Adjust month and year for calculation
  int Y = y;
  int M = m;
  if (M < 3) {
    Y--;
    M += 12;
  }
  // Calculate days since 0000-03-01
  uint32_t era = Y / 400;
  uint32_t yoe = Y - era * 400;
  uint32_t doy = (153*(M-3)+2)/5 + d - 1;
  uint32_t doe = yoe*365 + yoe/4 - yoe/100 + doy;
  // Convert to days since 1970-01-01
  return era*146097 + doe - 719468;
}

// —————— OLED Display ——————
// Display current system time on the OLED
void displayTime() {
  // Get the current time using the precise time function
  PreciseDateTime current_pdt = getPreciseDateTime();

  char line1[20], line2[20]; // Buffers for display lines

  // Format time (HH:MM:SS UTC) - Include microseconds for display
  snprintf(line1, sizeof(line1), "%02d:%02d:%02d.%03lu UTC", current_pdt.timeinfo.tm_hour, current_pdt.timeinfo.tm_min, current_pdt.timeinfo.tm_sec, current_pdt.microseconds / 1000); // Display milliseconds
  // Format date (YY-MM-DD)
  snprintf(line2, sizeof(line2), "%02d-%02d-%02d", current_pdt.timeinfo.tm_year % 100, current_pdt.timeinfo.tm_mon + 1, current_pdt.timeinfo.tm_mday);

  // Clear the display buffer and write the lines
  u8g2.clearBuffer();
  u8g2.setCursor(xOffset, yOffset);
  u8g2.print(line1);
  u8g2.setCursor(xOffset, yOffset + 12); // Move down for the next line
  u8g2.print(line2);
  u8g2.sendBuffer(); // Send buffer to the display
}
