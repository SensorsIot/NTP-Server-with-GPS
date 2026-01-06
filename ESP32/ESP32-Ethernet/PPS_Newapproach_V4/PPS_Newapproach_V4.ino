#include <ETH.h>
#include <TinyGPSPlus.h>
#include <time.h>
#include <lwip/def.h>
#include <lwip/udp.h> 
#include <freertos/FreeRTOS.h>
#include <freertos/task.h>
#include <lwip/tcpip.h>
#include <ArduinoOTA.h>
#include <ESPmDNS.h>

// --- CONFIGURATION ---
// this code assumes that we're always using a WT32-ETH01 module
//
// Pins:
#define GPS_RX_PIN       15
#define GPS_TX_PIN       4
#define PPS_PIN          2
#define ETH_ADDR         1
#define ETH_POWER_PIN    16
#define ETH_MDC_PIN      23
#define ETH_MDIO_PIN     18
#define ETH_TYPE         ETH_PHY_LAN8720
#define ETH_CLK_MODE     ETH_CLOCK_GPIO0_IN
#define STATUS_LED_PIN   33           // TODO - there isn't a status PIN on the board so just map to GPIO33 for now

// Settings
#define GPS_BAUD         38400        // for a QUESCAN M10050 M10 GNNS receiver in default configuration (38400 baud, 1hz PPS)
#define CORRECTION_US    1325         // this value may need tuning in your own setup
#define NTP_EPOCH_OFFSET 2208988800UL
#define NTP_PORT         123

// --- GLOBALS ---
TinyGPSPlus gps;
HardwareSerial gpsSerial(1);
volatile uint32_t lastPPSMicros = 0;
// We use volatile to ensure the LwIP callback sees the latest atomic write
volatile uint32_t lastSyncSec = 0;
volatile uint32_t lastSyncFrac = 0;

// Raw LwIP Control Block
struct udp_pcb *ntp_pcb;

// ntp packet
typedef struct {
  uint8_t li_vn_mode; 
  uint8_t stratum; 
  uint8_t poll; 
  uint8_t precision;
  uint32_t rootDelay; 
  uint32_t rootDispersion; 
  uint32_t refId;
  uint32_t refTm_s; 
  uint32_t refTm_f;
  uint32_t origTm_s; 
  uint32_t origTm_f;
  uint32_t rxTm_s; 
  uint32_t rxTm_f;
  uint32_t txTm_s; 
  uint32_t txTm_f;
} ntp_packet_t;

// --- ERROR RECOVERY ---
void fatalError(const char* msg) {
  Serial.print("FATAL: "); Serial.println(msg);
  Serial.println("Restarting system in 2 seconds...");
  delay(2000); 
  ESP.restart();
}

// --- ISR ---
void IRAM_ATTR PPS_ISR() {
  lastPPSMicros = micros();
}

// --- RAW LwIP CALLBACK (Running in TCP/IP Thread) ---
// This function runs immediately when a packet arrives, bypassing the scheduler.
// It reduces jitter as we don't have to wait (an inconsistent amount of time) for the scheduler to run our task.
void onNtpPacket(void *arg, struct udp_pcb *pcb, struct pbuf *p, const ip_addr_t *addr, u16_t port) {
  if (p == NULL) return;

  // 1. TIMESTAMP IMMEDIATELY (Critical Path)
  struct timeval tv;
  gettimeofday(&tv, NULL);

  // 2. Validate Packet Size (NTP is 48 bytes)
  if (p->tot_len >= sizeof(ntp_packet_t)) {
    // Access payload directly (Zero Copy)
    ntp_packet_t *pkt = (ntp_packet_t *)p->payload;

    // 3. Prepare Timestamps
    uint32_t s = htonl(tv.tv_sec + NTP_EPOCH_OFFSET);
    uint32_t f = htonl((uint32_t)((double)tv.tv_usec * 4294.967296)); 

    // 4. Update Packet in Place
    pkt->li_vn_mode = 0x24; // Server, Stratum 1
    pkt->stratum = 1;
    pkt->refId = htonl(0x47505300); // "GPS"
    
    // Copy reference time (atomic read from globals)
    pkt->refTm_s = lastSyncSec;
    pkt->refTm_f = lastSyncFrac;
    
    // Originate = Client's Transmit Time
    pkt->origTm_s = pkt->txTm_s; 
    pkt->origTm_f = pkt->txTm_f;
    
    // RX and TX times are effectively "now"
    pkt->rxTm_s = s; pkt->rxTm_f = f;
    pkt->txTm_s = s; pkt->txTm_f = f;

    // 5. Send Response (Reflect the buffer back)
    udp_sendto(pcb, p, addr, port);
  }

  // 6. Free the buffer (LwIP requires this)
  pbuf_free(p);
}

// --- INIT ---
void initNetwork() {
  // power cycle ethernet
  pinMode(ETH_POWER_PIN, OUTPUT);
  digitalWrite(ETH_POWER_PIN, LOW); 
  delay(100);
  digitalWrite(ETH_POWER_PIN, HIGH); 
  delay(100);
  
  if (!ETH.begin(ETH_TYPE, ETH_ADDR, ETH_MDC_PIN, ETH_MDIO_PIN, ETH_POWER_PIN, ETH_CLK_MODE)) {
    fatalError("Ethernet PHY failed to start");
  }

  unsigned long start = millis();
  while (ETH.localIP() == IPAddress(0,0,0,0)) { 
    if (millis() - start > 30000) fatalError("DHCP Timeout");
    delay(500); 
  }
  Serial.print("IP: "); 
  Serial.println(ETH.localIP());
}

void initRawNTP() {
 // CRITICAL: Lock the TCP/IP core before calling Raw API functions
  LOCK_TCPIP_CORE();
  
   // Create a new UDP Control Block
  ntp_pcb = udp_new();
  if (!ntp_pcb) { 
    fatalError("Could not create UDP PCB");
  }

  // Bind to port 123
  if (udp_bind(ntp_pcb, IP_ADDR_ANY, NTP_PORT) != ERR_OK) {
    fatalError("Could not bind UDP PCB");
  }

  // Register the callback function
  udp_recv(ntp_pcb, onNtpPacket, NULL);
  
  // CRITICAL: Unlock when done
  UNLOCK_TCPIP_CORE();
  
  Serial.println("Raw NTP Callback registered.");
}

void initOTA() {
  ArduinoOTA.setHostname("gpsclock");
  ArduinoOTA.setPassword("uploadota");
  ArduinoOTA.onStart([]() {
    Serial.println("Start updating...");
    // Stop interrupts to prevent crashes during flash write
    detachInterrupt(digitalPinToInterrupt(PPS_PIN));
  });
  ArduinoOTA.onEnd([]() { 
    Serial.println("\nEnd"); 
  });
  ArduinoOTA.onProgress([](unsigned int progress, unsigned int total) {
    Serial.printf("Progress: %u%%\r", (progress / (total / 100)));
  });
  ArduinoOTA.onError([](ota_error_t error) { 
    ESP.restart(); 
  });
  ArduinoOTA.begin();
}

// --- TASKS ---
void readGPSTime() {
  uint32_t localPPS = lastPPSMicros;
  uint32_t nowMicros = micros();
  static int previousSecond = -1;

  while (gpsSerial.available()) gps.encode(gpsSerial.read());

  if (gps.time.isUpdated() && gps.time.isValid()) {
    int currentSecond = gps.time.second();
    
    if (currentSecond == previousSecond) { 
      // we've already seen a NMEA message for this second, don't process any more updates
      return; 
    }

    previousSecond = currentSecond;

    // need to allow time for the clock to slew if we have previously asked for an adjustment.
    if (currentSecond % 5 != 0) {
      return;
    }

    uint32_t delta = nowMicros - localPPS;
    if (delta > 900000 || delta < 1000) { 
      // "one-second" guard: GPS sentences usually arrive 200-500ms AFTER the PPS pulse.
      // If microsecondsSincePulse is very large (e.g., > 900ms), we are likely looking at 
      // an old PPS pulse from the PREVIOUS second. If it's very small (e.g., < 10ms), the
      // PPS for the NEXT second might have just fired before we parsed the data for the
      // current second. If we detect either of these situations, skip processing. This
      // means we only process when in a reasonable time window from the PPS pulse.
      return; 
    }

    static bool ledState = false;
    digitalWrite(STATUS_LED_PIN, ledState = !ledState);

    struct tm t = {0};
    t.tm_year = gps.date.year() - 1900;
    t.tm_mon  = gps.date.month() - 1;
    t.tm_mday = gps.date.day();
    t.tm_hour = gps.time.hour();
    t.tm_min  = gps.time.minute();
    t.tm_sec  = gps.time.second();
    
    time_t gpsSecs = mktime(&t);
    struct timeval now;
    gettimeofday(&now, NULL);

    struct timeval adj;
    adj.tv_sec  = gpsSecs - now.tv_sec;
    adj.tv_usec = (delta + CORRECTION_US) - now.tv_usec;

    // ensure that tv_usec is under 1000000
    adj.tv_sec  += adj.tv_usec / 1000000;
    adj.tv_usec %= 1000000;
    if (adj.tv_usec < 0) { 
      adj.tv_usec += 1000000; adj.tv_sec--; 
    }

    // decide if we do a hard set because the difference between GNSS and system
    // clock is too high, or we slew the clock because the difference was low.
    if (abs(adj.tv_sec) >= 2) {
       struct timeval tv = { .tv_sec = gpsSecs, .tv_usec = delta + CORRECTION_US };
       tv.tv_sec += tv.tv_usec / 1000000; tv.tv_usec %= 1000000;
       settimeofday(&tv, NULL);
    } else {
       adjtime(&adj, NULL);
    }

    lastSyncSec  = htonl(gpsSecs + NTP_EPOCH_OFFSET);
    lastSyncFrac = htonl((uint32_t)((double)(delta + CORRECTION_US) * 4294.967296));
  }
}

void gpsTask(void *p) { 
  while(1) { 
    readGPSTime(); 
    vTaskDelay(pdMS_TO_TICKS(10)); 
  } 
}

void setup() {
  // force max frequency so the cpu doesn't autoscale
  setCpuFrequencyMhz(240);
  Serial.begin(115200);

  Serial.println("----- start: delay for gps startup -----");
  delay(10000);

  Serial.println("----- start: gps init -----");
  pinMode(STATUS_LED_PIN, OUTPUT);
  gpsSerial.begin(GPS_BAUD, SERIAL_8N1, GPS_RX_PIN, GPS_TX_PIN);
  pinMode(PPS_PIN, INPUT);
  attachInterrupt(digitalPinToInterrupt(PPS_PIN), PPS_ISR, RISING);
  
  Serial.println("----- start: network init -----");
  initNetwork();

  Serial.println("----- start: ota init -----");
  initOTA();
  
  Serial.println("----- start: ntp callback init -----");
  initRawNTP();

  Serial.println("----- start: start gps task -----");
  xTaskCreatePinnedToCore(gpsTask, "GPS", 4096, NULL, 2, NULL, 1);

  Serial.println("----- start: complete -----");
}

void loop() { 
  ArduinoOTA.handle(); 
  delay(100); 
}