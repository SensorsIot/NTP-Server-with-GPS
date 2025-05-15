#include <ETH.h>
#include <WiFi.h>           // For WiFi fallback and control
#include <WiFiUdp.h>
#include <TinyGPSPlus.h>
#include <time.h>
#include <EEPROM.h>
#include <freertos/FreeRTOS.h>
#include <freertos/task.h>
#include <freertos/semphr.h>
#include "credentials.h"    // Contains ssid and password definitions

// Debugging levels
#define DEBUG_STANDARD 1
#define DEBUG_TIME_CRITICAL 0

// Debugging Macros
#if DEBUG_STANDARD
  #define DEBUG_PRINT(x) Serial.print(x)
  #define DEBUG_PRINTLN(x) Serial.println(x)
#else
  #define DEBUG_PRINT(x)
  #define DEBUG_PRINTLN(x)
#endif

#if DEBUG_TIME_CRITICAL
  #define DEBUG_CRITICAL_PRINT(x) Serial.print(x)
  #define DEBUG_CRITICAL_PRINTLN(x) Serial.println(x)
#else
  #define DEBUG_CRITICAL_PRINT(x)
  #define DEBUG_CRITICAL_PRINTLN(x)
#endif

// Global Variables
TinyGPSPlus gps;
SemaphoreHandle_t gpsMutex;
WiFiUDP udp;

volatile bool PPSsignal = false;
volatile bool PPSavailable = false;

HardwareSerial gpsSerial(1);

// Baud rate for GPS and correction factor for time update
#define GPSBaud 9600
#define CORRECTION_FACTOR 0

#define EEPROM_SIZE 1

// Board configuration structure including PPS and GPS pins.
struct BoardConfig {
  const char *boardName;
  eth_phy_type_t phyType;
  int ethAddr;
  int powerPin;    // Use -1 if no Ethernet power control is needed.
  int mdcPin;
  int mdioPin;
  eth_clock_mode_t clkMode;
  int rxPin;       // GPS RX pin
  int txPin;       // GPS TX pin
  int ppsPin;      // PPS signal pin
};

// Define board configurations for different boards.
BoardConfig boardConfigs[] = {
  // Board 0: LilyGo PoE configuration (Ethernet present).
  { "LilyGo PoE", ETH_PHY_LAN8720, 0, 5, 23, 18, ETH_CLOCK_GPIO17_OUT, 15, 4, 14 },
  // Board 1: WT32-eth01 configuration (Ethernet present).
  { "WT32-eth01", ETH_PHY_LAN8720, 1, 16, 23, 18, ETH_CLOCK_GPIO0_IN, 15, 4, 2 },
  // Board 2: WiFi Only configuration (no Ethernet hardware).
  { "WiFi Only", (eth_phy_type_t)0, 0, -1, -1, -1, ETH_CLOCK_GPIO17_OUT, 15, 4, 2 }
};
const int numBoards = sizeof(boardConfigs) / sizeof(boardConfigs[0]);

// Global variable to store the selected board configuration.
BoardConfig currentBoardConfig;

// Global flag indicating if WiFi is used (fallback or WiFi-only configuration).
bool useWiFi = false;

// Decide which configuration to use (here using EEPROM to cycle through boards).
void selectBoardConfig() {
  if (!EEPROM.begin(EEPROM_SIZE)) {
    DEBUG_PRINTLN("Failed to initialize EEPROM.");
  }
  int lastTested = EEPROM.read(0);
  int currentBoard = (lastTested + 1) % numBoards;
  currentBoardConfig = boardConfigs[currentBoard];
  DEBUG_PRINT("Selected board: ");
  DEBUG_PRINTLN(currentBoardConfig.boardName);

  // Print the actual pins from the configuration.
  DEBUG_PRINT("RX pin: ");
  DEBUG_PRINTLN(currentBoardConfig.rxPin);
  DEBUG_PRINT("TX pin: ");
  DEBUG_PRINTLN(currentBoardConfig.txPin);
  DEBUG_PRINT("PPS pin: ");
  DEBUG_PRINTLN(currentBoardConfig.ppsPin);

  // Save the configuration index back to EEPROM.
  EEPROM.write(0, currentBoard);
  EEPROM.commit();
}

// Network initialization: if powerPin is valid, attempt Ethernet initialization;
// otherwise (or if Ethernet fails), use WiFi.
void initNetwork() {
  selectBoardConfig();

  // If the board is WiFi-only (powerPin == -1), skip Ethernet initialization.
  if (currentBoardConfig.powerPin == -1) {
    DEBUG_PRINTLN("WiFi Only configuration selected. Skipping Ethernet.");
    useWiFi = true;
    WiFi.mode(WIFI_STA);
    WiFi.begin(ssid, password);
    unsigned long wifiStartTime = millis();
    const unsigned long wifiTimeout = 10000; // 10 second timeout
    while (WiFi.status() != WL_CONNECTED && (millis() - wifiStartTime < wifiTimeout)) {
      DEBUG_PRINTLN("Connecting to WiFi...");
      delay(500);
    }
    if (WiFi.status() == WL_CONNECTED) {
      DEBUG_PRINT("WiFi connected! IP address: ");
      DEBUG_PRINTLN(WiFi.localIP());
    } else {
      DEBUG_PRINTLN("WiFi connection failed.");
    }
  } else {
    // For Ethernet-enabled boards, power-cycle the Ethernet chip if needed.
    pinMode(currentBoardConfig.powerPin, OUTPUT);
    digitalWrite(currentBoardConfig.powerPin, LOW);
    delay(100);
    digitalWrite(currentBoardConfig.powerPin, HIGH);
    delay(100);

    DEBUG_PRINT("Initializing Ethernet for ");
    DEBUG_PRINTLN(currentBoardConfig.boardName);
    bool ethStarted = ETH.begin(currentBoardConfig.phyType,
                                currentBoardConfig.ethAddr,
                                currentBoardConfig.mdcPin,
                                currentBoardConfig.mdioPin,
                                currentBoardConfig.powerPin,
                                currentBoardConfig.clkMode);
    bool ethInitialized = false;
    if (ethStarted) {
      DEBUG_PRINT("Ethernet initialized successfully for ");
      DEBUG_PRINTLN(currentBoardConfig.boardName);
      unsigned long startTime = millis();
      const unsigned long ethTimeout = 10000; // 10 second timeout
      while (ETH.localIP() == IPAddress(0, 0, 0, 0) && (millis() - startTime < ethTimeout)) {
        DEBUG_PRINTLN("Waiting for Ethernet IP address...");
        delay(500);
      }
      if (ETH.localIP() != IPAddress(0, 0, 0, 0)) {
        DEBUG_PRINT("Ethernet connected! IP Address: ");
        DEBUG_PRINTLN(ETH.localIP().toString().c_str());
        ethInitialized = true;
      } else {
        DEBUG_PRINTLN("Ethernet IP not obtained. Falling back to WiFi.");
      }
    } else {
      DEBUG_PRINT("Ethernet initialization failed for ");
      DEBUG_PRINTLN(currentBoardConfig.boardName);
    }
    if (ethInitialized) {
      DEBUG_PRINTLN("Disabling WiFi radio.");
      WiFi.disconnect(true);
      WiFi.mode(WIFI_OFF);
    } else {
      // Fallback to WiFi if Ethernet fails.
      useWiFi = true;
      DEBUG_PRINTLN("Initializing WiFi as fallback...");
      WiFi.mode(WIFI_STA);
      WiFi.begin(ssid, password);
      unsigned long wifiStartTime = millis();
      const unsigned long wifiTimeout = 10000; // 10 second timeout
      while (WiFi.status() != WL_CONNECTED && (millis() - wifiStartTime < wifiTimeout)) {
        DEBUG_PRINTLN("Connecting to WiFi...");
        delay(500);
      }
      if (WiFi.status() == WL_CONNECTED) {
        DEBUG_PRINT("WiFi connected! IP address: ");
        DEBUG_PRINTLN(WiFi.localIP());
      } else {
        DEBUG_PRINTLN("WiFi connection failed.");
      }
    }
  }
  udp.begin(123); // Start UDP on port 123 (used for NTP)
}

void printMacAddress() {
  char macStr[18]; // 17 characters + null terminator
  if (!useWiFi) {
    // Ethernet is enabled: get MAC from ETH
    uint8_t mac[6];
    ETH.macAddress(mac);
    sprintf(macStr, "%02X:%02X:%02X:%02X:%02X:%02X", 
            mac[0], mac[1], mac[2], mac[3], mac[4], mac[5]);
    DEBUG_PRINT("Ethernet MAC Address: ");
    DEBUG_PRINTLN(macStr);
  } else {
    // WiFi fallback: get MAC from WiFi
    String mac = WiFi.macAddress();
    DEBUG_PRINT("WiFi MAC Address: ");
    DEBUG_PRINTLN(mac);
  }
}


// PPS Interrupt Service Routine
void IRAM_ATTR PPS_ISR() {
  PPSsignal = true;
}

void initPPS() {
  // Set the PPS pin as a normal input.
  pinMode(currentBoardConfig.ppsPin, INPUT);
  // Print the actual PPS pin used.
  DEBUG_PRINT("Using PPS pin: ");
  DEBUG_PRINTLN(currentBoardConfig.ppsPin);
  
  DEBUG_PRINTLN("Checking PPS signal...");

  unsigned long startTime = millis();
  PPSavailable = false;
  
  // Increase detection window to 1500 ms, sampling every 1 ms.
  while (millis() - startTime < 1500) {
    if (digitalRead(currentBoardConfig.ppsPin) == HIGH) {
      PPSavailable = true;
      break;
    }
    delay(1);
  }
  
  if (PPSavailable) {
    DEBUG_PRINTLN("PPS signal detected.");
    attachInterrupt(digitalPinToInterrupt(currentBoardConfig.ppsPin), PPS_ISR, RISING);
    DEBUG_PRINTLN("PPS initialized. Waiting for signal...");
  } else {
    DEBUG_PRINTLN("PPS signal not detected! Continuing without PPS synchronization.");
  }
}

void initGPS() {
  DEBUG_PRINTLN("Initializing GPS...");
  // Use the RX/TX pins from the current board configuration.
  DEBUG_PRINT("Using GPS RX pin: ");
  DEBUG_PRINTLN(currentBoardConfig.rxPin);
  DEBUG_PRINT("Using GPS TX pin: ");
  DEBUG_PRINTLN(currentBoardConfig.txPin);
  
  gpsSerial.begin(GPSBaud, SERIAL_8N1, currentBoardConfig.rxPin, currentBoardConfig.txPin);
  unsigned long startTime = millis();
  while (millis() - startTime < 30000) {
    while (gpsSerial.available()) {
      gps.encode(gpsSerial.read());
      if (gps.time.isValid()) {
        DEBUG_PRINT("GPS time acquired: ");
        if (gps.time.hour() < 10) DEBUG_PRINT("0");
        DEBUG_PRINT(gps.time.hour());
        DEBUG_PRINT(":");
        if (gps.time.minute() < 10) DEBUG_PRINT("0");
        DEBUG_PRINT(gps.time.minute());
        DEBUG_PRINT(":");
        if (gps.time.second() < 10) DEBUG_PRINT("0");
        DEBUG_PRINTLN(gps.time.second());
        return;
      }
    }
    delay(100);
  }
  DEBUG_PRINTLN("Failed to acquire valid GPS time!");
}

void readGPSTime() {
  while (gpsSerial.available()) {
    gps.encode(gpsSerial.read());
  }
  if (gps.time.isValid() && gps.date.isValid()) {
    struct tm timeinfo = { 0 };
    timeinfo.tm_year = gps.date.year() - 1900;
    timeinfo.tm_mon = gps.date.month() - 1;
    timeinfo.tm_mday = gps.date.day();
    timeinfo.tm_hour = gps.time.hour();
    timeinfo.tm_min = gps.time.minute();
    timeinfo.tm_sec = gps.time.second();
    time_t unixTime = mktime(&timeinfo) + 1;
    // If PPS is available, wait for the next PPS signal for precise synchronization.
    if (PPSavailable) {
      while (!PPSsignal) {}
      PPSsignal = false;
    }
    struct timeval tv;
    tv.tv_sec = unixTime;
    tv.tv_usec = CORRECTION_FACTOR;
    settimeofday(&tv, NULL);
    DEBUG_CRITICAL_PRINTLN("GPS Time Updated");
  }
}

void handleNTPRequest() {
  int packetSize = udp.parsePacket();
  if (packetSize >= 48) {  // NTP packets are 48 bytes long
    uint8_t requestBuffer[48];
    uint8_t responseBuffer[48];
    udp.read(requestBuffer, 48);
    struct timeval tv;
    gettimeofday(&tv, NULL);
    uint32_t recvSec = tv.tv_sec + 2208988800UL;
    uint32_t recvFrac = (uint32_t)((double)tv.tv_usec * (4294967296.0 / 1000000.0));
    responseBuffer[0] = 0x24;              // LI=0, VN=4, Mode=4 (server)
    responseBuffer[1] = 1;                 // Stratum 1
    responseBuffer[2] = requestBuffer[2];  // Poll interval from request
    responseBuffer[3] = -6;                // Precision (example)
    // Clear Root Delay & Root Dispersion
    for (int i = 4; i < 12; i++) {
      responseBuffer[i] = 0;
    }
    // Reference Identifier ("LOCL")
    responseBuffer[12] = 'L';
    responseBuffer[13] = 'O';
    responseBuffer[14] = 'C';
    responseBuffer[15] = 'L';
    // Reference Timestamp
    responseBuffer[16] = (recvSec >> 24) & 0xFF;
    responseBuffer[17] = (recvSec >> 16) & 0xFF;
    responseBuffer[18] = (recvSec >> 8) & 0xFF;
    responseBuffer[19] = recvSec & 0xFF;
    responseBuffer[20] = (recvFrac >> 24) & 0xFF;
    responseBuffer[21] = (recvFrac >> 16) & 0xFF;
    responseBuffer[22] = (recvFrac >> 8) & 0xFF;
    responseBuffer[23] = recvFrac & 0xFF;
    // Originate Timestamp: copy client's Transmit Timestamp (offset 40)
    for (int i = 0; i < 8; i++) {
      responseBuffer[24 + i] = requestBuffer[40 + i];
    }
    // Receive Timestamp
    responseBuffer[32] = (recvSec >> 24) & 0xFF;
    responseBuffer[33] = (recvSec >> 16) & 0xFF;
    responseBuffer[34] = (recvSec >> 8) & 0xFF;
    responseBuffer[35] = recvSec & 0xFF;
    responseBuffer[36] = (recvFrac >> 24) & 0xFF;
    responseBuffer[37] = (recvFrac >> 16) & 0xFF;
    responseBuffer[38] = (recvFrac >> 8) & 0xFF;
    responseBuffer[39] = recvFrac & 0xFF;
    // Transmit Timestamp (recorded just before sending)
    gettimeofday(&tv, NULL);
    uint32_t txSec = tv.tv_sec + 2208988800UL;
    uint32_t txFrac = (uint32_t)((double)tv.tv_usec * (4294967296.0 / 1000000.0));
    responseBuffer[40] = (txSec >> 24) & 0xFF;
    responseBuffer[41] = (txSec >> 16) & 0xFF;
    responseBuffer[42] = (txSec >> 8) & 0xFF;
    responseBuffer[43] = txSec & 0xFF;
    responseBuffer[44] = (txFrac >> 24) & 0xFF;
    responseBuffer[45] = (txFrac >> 16) & 0xFF;
    responseBuffer[46] = (txFrac >> 8) & 0xFF;
    responseBuffer[47] = txFrac & 0xFF;
    udp.beginPacket(udp.remoteIP(), udp.remotePort());
    udp.write(responseBuffer, 48);
    udp.endPacket();
    DEBUG_PRINTLN("NTP response sent.");
  }
}

void gpsTask(void *parameter) {
  while (1) {
    readGPSTime();
    vTaskDelay(pdMS_TO_TICKS(10));
  }
}

void ntpTask(void *parameter) {
  while (1) {
    handleNTPRequest();
    vTaskDelay(pdMS_TO_TICKS(100));
  }
}

void setup() {
  Serial.begin(115200);
  initNetwork();   // Use Ethernet if available; otherwise, fall back to WiFi.
  printMacAddress();  // Print the actual MAC address used.
  initPPS();
  initGPS();
  gpsMutex = xSemaphoreCreateMutex();
  xTaskCreatePinnedToCore(gpsTask, "GPSTask", 4096, NULL, 1, NULL, 0);
  xTaskCreatePinnedToCore(ntpTask, "NTPTask", 4096, NULL, 1, NULL, 0);
}

void loop() {
  // The tasks handle GPS and NTP functionality.
}
