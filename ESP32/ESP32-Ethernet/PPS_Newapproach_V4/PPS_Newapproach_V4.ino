#include <ETH.h>
#include <TinyGPSPlus.h>
#include <time.h>
#include <esp_timer.h>
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
#define CORRECTION_US    0          // this value may need tuning in your own setup
#define NTP_EPOCH_OFFSET 2208988800UL
#define NTP_PORT         123
#define HOLDOVER_MS      60000        // report unsynchronised (LI=3) if no GNSS sync for this long
#define MAX_PPM          100          // reject PPS edges whose spacing implies a clock error larger than this
#define RATE_SHIFT       4            // frequency estimate averages over about 2^RATE_SHIFT syncs

// --- GLOBALS ---
TinyGPSPlus gps;
HardwareSerial gpsSerial(1);
volatile uint32_t lastPPSMicros = 0;   // low 32 bits of esp_timer at the latest PPS edge (one word, so the write is atomic)

// The served time is computed from esp_timer, anchored at the last accepted PPS edge and corrected
// for esp_timer's frequency error. gpsTask writes it and the NTP callback reads it, both under tsMux.
typedef struct {
  int64_t  edgeUs;    // esp_timer time of the anchor PPS edge, minus CORRECTION_US
  uint32_t ntpSec;    // NTP seconds that start at that edge
  int64_t  rateQ32;   // esp_timer frequency error as a fraction x 2^32, positive when it runs fast
  bool     valid;     // false until two GNSS syncs agree, so we never serve the boot time or a glitched edge
} timescale_t;
timescale_t timescale = {0, 0, 0, false};
portMUX_TYPE tsMux = portMUX_INITIALIZER_UNLOCKED;

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

// --- HELPERS ---
// usec * 2^32 / 10^6 without floating point (the ESP32 FPU is single precision, so double maths is emulated)
static inline uint32_t usecToNtpFrac(uint32_t usec) {
  return (uint32_t)(((uint64_t)usec * 281474977ULL) >> 16);
}

// convert an esp_timer reading into NTP seconds and fraction using a copy of the timescale
static inline void toNtp(const timescale_t &ts, int64_t t, uint32_t &sec, uint32_t &frac) {
  int64_t elapsed = t - ts.edgeUs;              // esp_timer us since the anchor edge
  elapsed -= (elapsed * ts.rateQ32) >> 32;      // remove esp_timer's frequency error
  sec  = ts.ntpSec + (uint32_t)(elapsed / 1000000);
  frac = usecToNtpFrac((uint32_t)(elapsed % 1000000));
}

// --- ISR ---
void IRAM_ATTR PPS_ISR() {
  lastPPSMicros = (uint32_t)esp_timer_get_time();
}

// --- RAW LwIP CALLBACK (Running in TCP/IP Thread) ---
// This function runs immediately when a packet arrives, bypassing the scheduler.
// It reduces jitter as we don't have to wait (an inconsistent amount of time) for the scheduler to run our task.
void onNtpPacket(void *arg, struct udp_pcb *pcb, struct pbuf *p, const ip_addr_t *addr, u16_t port) {
  if (p == NULL) return;

  // 1. TIMESTAMP IMMEDIATELY (Critical Path)
  int64_t rxUs = esp_timer_get_time();

  // take a consistent copy of the timescale (the lock is only held for the copy)
  timescale_t ts;
  portENTER_CRITICAL(&tsMux);
  ts = timescale;
  portEXIT_CRITICAL(&tsMux);

  // 2. Validate Packet Size (NTP is 48 bytes), only answer client requests (mode 3) so we
  // never reply to another server's reply, and stay silent until the timescale has been set
  // from GNSS so we never serve the boot time.
  ntp_packet_t *pkt = (ntp_packet_t *)p->payload;
  if (ts.valid && p->tot_len >= sizeof(ntp_packet_t) && (pkt->li_vn_mode & 0x07) == 3) {
    // 3. Update Packet in Place
    // LI=0 while GNSS is fresh, LI=3 (unsynchronised) once we've been in holdover too long.
    // VN is copied from the request, mode 4 = server.
    bool stale = (rxUs - ts.edgeUs) > (int64_t)HOLDOVER_MS * 1000;
    uint8_t li = stale ? 3 : 0;
    uint8_t vn = (pkt->li_vn_mode >> 3) & 0x07;
    pkt->li_vn_mode = (li << 6) | (vn << 3) | 4;
    pkt->stratum = stale ? 16 : 1;   // 16 = unsynchronised
    pkt->refId = htonl(0x47505300); // "GPS"
    
    // Reference time = the PPS edge the timescale is anchored to
    pkt->refTm_s = htonl(ts.ntpSec);
    pkt->refTm_f = 0;

    // Originate = Client's Transmit Time
    pkt->origTm_s = pkt->txTm_s;
    pkt->origTm_f = pkt->txTm_f;

    // RX = the timestamp taken on arrival
    uint32_t s, f;
    toNtp(ts, rxUs, s, f);
    pkt->rxTm_s = htonl(s);
    pkt->rxTm_f = htonl(f);

    // 4. TX = read the clock again as late as possible so our processing time is
    // reported to the client instead of being counted as network delay
    toNtp(ts, esp_timer_get_time(), s, f);
    pkt->txTm_s = htonl(s);
    pkt->txTm_f = htonl(f);

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
  // no mDNS responder: it adds traffic to the lwIP tcpip thread that the NTP callback runs in. Upload by IP.
  ArduinoOTA.setMdnsEnabled(false);
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
// Anchor the timescale at PPS edge 'edgeUs' (full esp_timer time), which starts NTP second 'ntpSec',
// and learn esp_timer's frequency error from the spacing of successive edges.
void updateTimescale(uint32_t ntpSec, int64_t edgeUs) {
  static int64_t prevEdgeUs = 0;
  static uint32_t prevSec = 0;
  static int64_t rateQ32 = 0;
  static bool haveRate = false;
  static int rejects = 0;

  if (prevSec != 0) {
    int64_t secs = (int32_t)(ntpSec - prevSec);                 // seconds since the previous anchor
    int64_t gainUs = (edgeUs - prevEdgeUs) - secs * 1000000;     // how far esp_timer ran ahead over them
    if (secs <= 0 || llabs(gainUs) > secs * MAX_PPM) {
      // the edge spacing doesn't fit a real crystal: a glitch on the PPS line, or the NMEA paired with
      // the wrong edge or date. Keep serving from the previous anchor, unless this keeps happening, in
      // which case it was the previous anchor that was wrong.
      if (++rejects < 3) return;
    } else {
      int64_t sample = gainUs * 4294967296LL / (secs * 1000000);
      rateQ32 = haveRate ? rateQ32 + ((sample - rateQ32) >> RATE_SHIFT) : sample;
      haveRate = true;
    }
  }
  rejects = 0;
  prevEdgeUs = edgeUs;
  prevSec = ntpSec;

  portENTER_CRITICAL(&tsMux);
  timescale.edgeUs = edgeUs - CORRECTION_US;
  timescale.ntpSec = ntpSec;
  timescale.rateQ32 = rateQ32;
  timescale.valid = haveRate;
  portEXIT_CRITICAL(&tsMux);
}

void readGPSTime() {
  // snapshot the latest PPS edge and the time now before draining the UART, so an edge that
  // fires while we parse can't replace the one for the second being decoded
  uint32_t localPPS = lastPPSMicros;
  int64_t nowUs = esp_timer_get_time();
  static int previousSecond = -1;

  while (gpsSerial.available()) gps.encode(gpsSerial.read());

  if (gps.time.isUpdated() && gps.time.isValid()) {
    int currentSecond = gps.time.second();
    
    if (currentSecond == previousSecond) { 
      // we've already seen a NMEA message for this second, don't process any more updates
      return; 
    }

    previousSecond = currentSecond;

    uint32_t delta = (uint32_t)nowUs - localPPS;
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

    // edge N happened delta us before nowUs (localPPS is the low 32 bits of its esp_timer time)
    updateTimescale(gpsSecs + NTP_EPOCH_OFFSET, nowUs - delta);
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