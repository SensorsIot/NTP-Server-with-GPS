#include <ETH.h>
#include <TinyGPSPlus.h>
#include <time.h>
#include <esp_timer.h>
#include <esp_task_wdt.h>
#include <driver/mcpwm_cap.h>
#include <lwip/def.h>
#include <lwip/udp.h> 
#include <freertos/FreeRTOS.h>
#include <freertos/task.h>
#include <lwip/tcpip.h>
#include <ArduinoOTA.h>

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

// Settings
#define GPS_BAUD         38400        // for a QUESCAN M10050 M10 GNNS receiver in default configuration (38400 baud, 1hz PPS)
#define CORRECTION_US    130          // this value may need tuning in your own setup
#define NTP_EPOCH_OFFSET 2208988800UL
#define NTP_PORT         123
#define HOLDOVER_MS      60000        // report unsynchronised (LI=3) if no GNSS sync for this long
#define MAX_PPM            100        // reject PPS edges whose spacing implies a clock error larger than this
#define MAX_EDGE_JITTER_US 15         // max allowed PPS residual vs rate-predicted edge (base, in us)
#define MAX_DRIFT_PPM      5          // additional allowed residual per second of interval (covers temp drift in holdover)
#define MIN_SATS           4          // minimum satellites in use (from GGA) required to accept a GNSS sync
#define MIN_VALID_YEAR     2025       // reject uninitialized/RTC-default GNSS dates before this year
#define RATE_SHIFT         4          // frequency estimate averages over about 2^RATE_SHIFT syncs
#define NTP_PRECISION      (-20)      // log2 seconds (~0.95 us), matching esp_timer's 1 us resolution
#define ROOT_DISP_BASE_US  50         // baseline root dispersion in us when freshly synced
#define ROOT_DISP_PPM      15         // holdover dispersion growth rate in ppm (RFC 5905 PHI)

// --- GLOBALS ---
TinyGPSPlus gps;
TinyGPSCustom gnrmcStatus(gps, "GNRMC", 2);  // RMC fix status ('A' = active, 'V' = void)
TinyGPSCustom gprmcStatus(gps, "GPRMC", 2);
HardwareSerial gpsSerial(1);

// MCPWM hardware capture state: channel 0 latches PPS rising edges in hardware at 80 MHz APB
// (12.5 ns, zero ISR latency jitter); channel 1 is software-triggered to correlate MCPWM ticks
// with esp_timer's microsecond timebase.
mcpwm_cap_channel_handle_t ppsCapChan  = NULL;
mcpwm_cap_channel_handle_t softCapChan = NULL;
uint32_t capResHz = 80000000;          // actual MCPWM capture tick rate (queried at init)
volatile uint32_t lastPPSCap     = 0;  // hardware-latched MCPWM count at the latest PPS edge
volatile uint32_t lastPPSRoughUs = 0;  // coarse esp_timer time of that edge (guards 53.7 s cap wrap)
volatile bool     havePPSCap     = false;
volatile uint32_t lastSoftCap    = 0;  // MCPWM count latched by software catch
volatile bool     softCapDone    = false;
portMUX_TYPE      capMux         = portMUX_INITIALIZER_UNLOCKED;

TaskHandle_t  gpsTaskHandle = NULL;
volatile bool otaInProgress = false;

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

// ntp packet
typedef struct {
  uint8_t li_vn_mode; 
  uint8_t stratum; 
  uint8_t poll; 
  int8_t  precision;
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

// root dispersion in NTP short format (16.16 unsigned fixed-point seconds, 1 LSB = 1/65536 s ~ 15.26 us)
static inline uint32_t calcRootDispersion(int64_t ageUs) {
  static constexpr uint64_t BASE_SHORT = ((uint64_t)ROOT_DISP_BASE_US * 65536ULL + 999999ULL) / 1000000ULL;
  static constexpr uint64_t RATE_Q32   = ((uint64_t)ROOT_DISP_PPM * 65536ULL * (1ULL << 32) + 999999999999ULL) / 1000000000000ULL;
  uint64_t disp = BASE_SHORT;
  if (ageUs > 0) {
    disp += ((uint64_t)ageUs * RATE_Q32 + (1ULL << 31)) >> 32;
  }
  return (disp > 0xFFFFFFFFULL) ? 0xFFFFFFFFUL : (uint32_t)disp;
}

// convert an esp_timer reading into NTP seconds and fraction using a copy of the timescale
static inline void toNtp(const timescale_t &ts, int64_t t, uint32_t &sec, uint32_t &frac) {
  int64_t elapsed = t - ts.edgeUs;              // esp_timer us since the anchor edge
  elapsed -= (elapsed * ts.rateQ32) >> 32;      // remove esp_timer's frequency error
  sec  = ts.ntpSec + (uint32_t)(elapsed / 1000000);
  frac = usecToNtpFrac((uint32_t)(elapsed % 1000000));
}

// --- MCPWM CAPTURE ISRs ---
static bool IRAM_ATTR onPpsCapture(mcpwm_cap_channel_handle_t cap_chan,
                                   const mcpwm_capture_event_data_t *edata,
                                   void *user_ctx) {
  lastPPSCap = edata->cap_value;
  lastPPSRoughUs = (uint32_t)esp_timer_get_time();
  havePPSCap = true;
  return false;
}

static bool IRAM_ATTR onSoftCapture(mcpwm_cap_channel_handle_t cap_chan,
                                    const mcpwm_capture_event_data_t *edata,
                                    void *user_ctx) {
  lastSoftCap = edata->cap_value;
  softCapDone = true;
  return false;
}

// Correlate esp_timer and the MCPWM capture counter within a short critical section so a
// hardware-latched PPS count can be mapped into esp_timer's microsecond timebase.
static bool snapshotClocks(int64_t &nowUs, uint32_t &nowCap) {
  for (int attempt = 0; attempt < 4; attempt++) {
    softCapDone = false;
    portENTER_CRITICAL(&capMux);
    int64_t t1 = esp_timer_get_time();
    esp_err_t err = mcpwm_capture_channel_trigger_soft_catch(softCapChan);
    int64_t t2 = esp_timer_get_time();
    portEXIT_CRITICAL(&capMux);
    if (err != ESP_OK) return false;

    int spins = 0;
    while (!softCapDone && spins < 1000) {
      spins++;
    }
    if (softCapDone && (t2 - t1) <= 6) {
      nowUs = t1 + (t2 - t1) / 2;
      nowCap = lastSoftCap;
      return true;
    }
  }
  return false;
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

  // 2. Validate Packet Size (NTP is 48 bytes, contiguous in the first pbuf), only answer
  // client requests (mode 3) so we never reply to another server's reply, and stay silent
  // until the timescale has been set from GNSS so we never serve the boot time.
  if (ts.valid && p->len >= sizeof(ntp_packet_t)) {
    ntp_packet_t *pkt = (ntp_packet_t *)p->payload;
    if ((pkt->li_vn_mode & 0x07) == 3) {
      // Trim any trailing bytes (e.g. client extension fields / MAC) so we always send a 48-byte reply
      if (p->tot_len > sizeof(ntp_packet_t)) {
        pbuf_realloc(p, sizeof(ntp_packet_t));
      }

      // 3. Update Packet in Place
      // LI=0 while GNSS is fresh, LI=3 (unsynchronised) once we've been in holdover too long.
      // VN and poll are copied from the request, mode 4 = server.
      int64_t ageUs = rxUs - ts.edgeUs;
      bool stale = ageUs > (int64_t)HOLDOVER_MS * 1000;
      uint8_t li = stale ? 3 : 0;
      uint8_t vn = (pkt->li_vn_mode >> 3) & 0x07;
      pkt->li_vn_mode = (li << 6) | (vn << 3) | 4;
      pkt->stratum = stale ? 16 : 1;   // 16 = unsynchronised
      pkt->precision = NTP_PRECISION;
      pkt->rootDelay = 0;
      pkt->rootDispersion = htonl(calcRootDispersion(ageUs));
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
  }

  // 6. Free the buffer (LwIP requires this)
  pbuf_free(p);
}

// --- INIT ---
void initPpsCapture() {
  mcpwm_cap_timer_handle_t ppsCapTimer = NULL;
  mcpwm_capture_timer_config_t timerCfg = {};
  timerCfg.group_id = 0;
  timerCfg.clk_src = MCPWM_CAPTURE_CLK_SRC_DEFAULT;
  if (mcpwm_new_capture_timer(&timerCfg, &ppsCapTimer) != ESP_OK) {
    fatalError("Could not create MCPWM capture timer");
  }
  if (mcpwm_capture_timer_get_resolution(ppsCapTimer, &capResHz) != ESP_OK || capResHz == 0) {
    fatalError("Could not query MCPWM capture resolution");
  }

  // Channel 0: hardware rising-edge capture on PPS_PIN (GPIO2)
  mcpwm_capture_channel_config_t ppsChanCfg = {};
  ppsChanCfg.gpio_num = PPS_PIN;
  ppsChanCfg.prescale = 1;
  ppsChanCfg.flags.pos_edge = true;
  if (mcpwm_new_capture_channel(ppsCapTimer, &ppsChanCfg, &ppsCapChan) != ESP_OK) {
    fatalError("Could not create MCPWM PPS capture channel");
  }
  mcpwm_capture_event_callbacks_t ppsCbs = {};
  ppsCbs.on_cap = onPpsCapture;
  if (mcpwm_capture_channel_register_event_callbacks(ppsCapChan, &ppsCbs, NULL) != ESP_OK) {
    fatalError("Could not register MCPWM PPS callback");
  }
  if (mcpwm_capture_channel_enable(ppsCapChan) != ESP_OK) {
    fatalError("Could not enable MCPWM PPS channel");
  }

  // Channel 1: software-only capture channel (no GPIO) to correlate MCPWM ticks with esp_timer
  mcpwm_capture_channel_config_t softChanCfg = {};
  softChanCfg.gpio_num = -1;
  softChanCfg.prescale = 1;
  softChanCfg.flags.pos_edge = true;
  if (mcpwm_new_capture_channel(ppsCapTimer, &softChanCfg, &softCapChan) != ESP_OK) {
    fatalError("Could not create MCPWM soft capture channel");
  }
  mcpwm_capture_event_callbacks_t softCbs = {};
  softCbs.on_cap = onSoftCapture;
  if (mcpwm_capture_channel_register_event_callbacks(softCapChan, &softCbs, NULL) != ESP_OK) {
    fatalError("Could not register MCPWM soft callback");
  }
  if (mcpwm_capture_channel_enable(softCapChan) != ESP_OK) {
    fatalError("Could not enable MCPWM soft channel");
  }

  if (mcpwm_capture_timer_enable(ppsCapTimer) != ESP_OK ||
      mcpwm_capture_timer_start(ppsCapTimer) != ESP_OK) {
    fatalError("Could not start MCPWM capture timer");
  }
}

// Configure a u-blox M10 over UART1 (RAM layer only, so settings are non-destructive and
// reset cleanly on power-cycle): Stationary mode, keep only RMC + GGA NMEA sentences, and
// emit 1 Hz / 100 ms PPS on TP1 only when locked to GNSS time.
void initUbloxM10() {
  uint8_t buf[128];
  size_t len = 0;

  // UBX header: sync (0xB5, 0x62), class CFG (0x06), id VALSET (0x8A), length placeholder
  buf[len++] = 0xB5;
  buf[len++] = 0x62;
  buf[len++] = 0x06;
  buf[len++] = 0x8A;
  buf[len++] = 0x00;
  buf[len++] = 0x00;
  // CFG-VALSET header: version 0, layers = RAM only (0x01), reserved (2 bytes)
  buf[len++] = 0x00;
  buf[len++] = 0x01;
  buf[len++] = 0x00;
  buf[len++] = 0x00;

  auto addKey = [&](uint32_t key) {
    buf[len++] = (uint8_t)(key & 0xFF);
    buf[len++] = (uint8_t)((key >> 8) & 0xFF);
    buf[len++] = (uint8_t)((key >> 16) & 0xFF);
    buf[len++] = (uint8_t)((key >> 24) & 0xFF);
  };
  auto addU1 = [&](uint32_t key, uint8_t val) {
    addKey(key);
    buf[len++] = val;
  };
  auto addU4 = [&](uint32_t key, uint32_t val) {
    addKey(key);
    buf[len++] = (uint8_t)(val & 0xFF);
    buf[len++] = (uint8_t)((val >> 8) & 0xFF);
    buf[len++] = (uint8_t)((val >> 16) & 0xFF);
    buf[len++] = (uint8_t)((val >> 24) & 0xFF);
  };

  // Dynamic model: 2 = Stationary (tighter timing for a fixed antenna)
  addU1(0x20110021, 2);       // CFG-NAVSPG-DYNMODEL

  // NMEA output on UART1: keep RMC (atomic time/date/status) and GGA (satellites), disable the rest
  addU1(0x209100AC, 1);       // CFG-MSGOUT-NMEA_ID_RMC_UART1
  addU1(0x209100BB, 1);       // CFG-MSGOUT-NMEA_ID_GGA_UART1
  addU1(0x209100CA, 0);       // CFG-MSGOUT-NMEA_ID_GLL_UART1
  addU1(0x209100C0, 0);       // CFG-MSGOUT-NMEA_ID_GSA_UART1
  addU1(0x209100C5, 0);       // CFG-MSGOUT-NMEA_ID_GSV_UART1
  addU1(0x209100B1, 0);       // CFG-MSGOUT-NMEA_ID_VTG_UART1

  // Time pulse TP1: 1 Hz / 100 ms pulse when GNSS-locked, 0 us (no pulse) when unlocked
  addU1(0x10050007, 1);       // CFG-TP-TP1_ENA
  addU1(0x10050008, 1);       // CFG-TP-SYNC_GNSS_TP1
  addU1(0x10050009, 1);       // CFG-TP-USE_LOCKED_TP1
  addU1(0x20050023, 0);       // CFG-TP-PULSE_DEF (0 = PERIOD)
  addU1(0x20050030, 1);       // CFG-TP-PULSE_LENGTH_DEF (1 = LENGTH in us)
  addU4(0x40050002, 1000000); // CFG-TP-PERIOD_TP1 (1 s)
  addU4(0x40050003, 1000000); // CFG-TP-PERIOD_LOCK_TP1 (1 s)
  addU4(0x40050004, 0);       // CFG-TP-LEN_TP1 (0 us = off when unlocked)
  addU4(0x40050005, 100000);  // CFG-TP-LEN_LOCK_TP1 (100 ms when locked)

  uint16_t payloadLen = (uint16_t)(len - 6);
  buf[4] = (uint8_t)(payloadLen & 0xFF);
  buf[5] = (uint8_t)((payloadLen >> 8) & 0xFF);

  uint8_t ckA = 0, ckB = 0;
  for (size_t i = 2; i < len; i++) {
    ckA += buf[i];
    ckB += ckA;
  }
  buf[len++] = ckA;
  buf[len++] = ckB;

  gpsSerial.write(buf, len);
  gpsSerial.flush();
  delay(50);
}

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
  struct udp_pcb *ntp_pcb = udp_new();
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
    // Stop serving NTP time, halt GPS updates, and disable PPS capture + WDT during flash writes
    otaInProgress = true;
    portENTER_CRITICAL(&tsMux);
    timescale.valid = false;
    portEXIT_CRITICAL(&tsMux);
    if (ppsCapChan) {
      mcpwm_capture_channel_disable(ppsCapChan);
    }
    if (gpsTaskHandle) {
      esp_task_wdt_delete(gpsTaskHandle);
    }
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

  if (prevSec == 0) {
    // First sync: store as candidate anchor and wait for a second sync to corroborate it.
    prevEdgeUs = edgeUs;
    prevSec = ntpSec;
    return;
  }

  int64_t secs = (int32_t)(ntpSec - prevSec);                 // seconds since the previous anchor
  int64_t gainUs = (edgeUs - prevEdgeUs) - secs * 1000000LL;  // how far esp_timer ran ahead over them
  bool bad = (secs <= 0 || llabs(gainUs) > secs * MAX_PPM);
  if (!bad && haveRate) {
    // Once frequency error is known, reject edges that deviate too far from the predicted spacing
    // (e.g. a delayed PPS ISR) instead of baking that jitter into the next second's anchor.
    int64_t expectedGainUs = ((secs * 1000000LL) * rateQ32 + (1LL << 31)) >> 32;
    int64_t maxResidualUs = MAX_EDGE_JITTER_US + secs * MAX_DRIFT_PPM;
    if (llabs(gainUs - expectedGainUs) > maxResidualUs) {
      bad = true;
    }
  }

  if (bad) {
    // Keep serving from the previous anchor. If 3 syncs in a row fail, assume the previous
    // anchor (or learned rate) was bad and re-seed without publishing yet, so the next sync
    // must corroborate the new candidate before clients see it.
    if (++rejects >= 3) {
      rejects = 0;
      prevEdgeUs = edgeUs;
      prevSec = ntpSec;
      haveRate = false;
    }
    return;
  }

  int64_t sample = gainUs * 4294967296LL / (secs * 1000000LL);
  rateQ32 = haveRate ? rateQ32 + ((sample - rateQ32) >> RATE_SHIFT) : sample;
  haveRate = true;
  rejects = 0;
  prevEdgeUs = edgeUs;
  prevSec = ntpSec;

  portENTER_CRITICAL(&tsMux);
  timescale.edgeUs = edgeUs - CORRECTION_US;
  timescale.ntpSec = ntpSec;
  timescale.rateQ32 = rateQ32;
  timescale.valid = true;
  portEXIT_CRITICAL(&tsMux);
}

void readGPSTime() {
  // snapshot the latest hardware-latched PPS edge before draining the UART, so an edge that
  // fires while we parse can't replace the one for the second being decoded
  bool localHavePPS = havePPSCap;
  uint32_t localPPSCap = lastPPSCap;
  uint32_t localPPSRoughUs = lastPPSRoughUs;
  if (lastPPSCap != localPPSCap) {
    localPPSCap = lastPPSCap;
    localPPSRoughUs = lastPPSRoughUs;
    localHavePPS = havePPSCap;
  }
  int64_t preDrainUs = esp_timer_get_time();

  while (gpsSerial.available()) gps.encode(gpsSerial.read());

  // Only trigger on RMC ($GNRMC / $GPRMC), which carries date, time, and fix status ('A'/'V')
  // atomically in one sentence — avoiding the midnight GGA-before-RMC day-behind race.
  bool rmcUpdated = false;
  bool rmcActive = false;
  if (gnrmcStatus.isUpdated()) {
    rmcUpdated = true;
    rmcActive = (gnrmcStatus.value()[0] == 'A');
  }
  if (gprmcStatus.isUpdated()) {
    rmcUpdated = true;
    rmcActive = rmcActive || (gprmcStatus.value()[0] == 'A');
  }

  if (!rmcUpdated || !rmcActive || !localHavePPS) {
    return;
  }

  if (!gps.date.isValid() || !gps.time.isValid() || gps.date.year() < MIN_VALID_YEAR) {
    return;
  }

  if (!gps.satellites.isValid() || gps.satellites.age() > 2500 || gps.satellites.value() < MIN_SATS) {
    return;
  }

  uint32_t preDelta = (uint32_t)preDrainUs - localPPSRoughUs;
  if (preDelta > 900000 || preDelta < 1000) {
    // "one-second" guard: GPS sentences usually arrive 50-500ms AFTER the PPS pulse.
    // If delta is very large (> 900ms), we are likely looking at an old PPS pulse from
    // the PREVIOUS second. If it's very small (< 1ms), the PPS for the NEXT second might
    // have just fired before we parsed the data for the current second. Skip processing
    // outside this window.
    return;
  }

  int64_t nowUs = 0;
  uint32_t nowCap = 0;
  if (!snapshotClocks(nowUs, nowCap)) {
    return;
  }

  // Convert the hardware-latched 80 MHz tick delta into microseconds and cross-check against
  // roughDelta (71.6 min wrap) so the 53.7 s 32-bit capture counter wrap can never fool us.
  uint32_t roughDelta = (uint32_t)nowUs - localPPSRoughUs;
  uint32_t deltaTicks = nowCap - localPPSCap;
  uint32_t deltaUs = (uint32_t)(((uint64_t)deltaTicks * 1000000ULL + capResHz / 2) / capResHz);
  if (deltaUs > 900000 || deltaUs < 1000 || llabs((int64_t)deltaUs - (int64_t)roughDelta) > 5000) {
    return;
  }

  struct tm t = {0};
  t.tm_year = gps.date.year() - 1900;
  t.tm_mon  = gps.date.month() - 1;
  t.tm_mday = gps.date.day();
  t.tm_hour = gps.time.hour();
  t.tm_min  = gps.time.minute();
  t.tm_sec  = gps.time.second();

  time_t gpsSecs = mktime(&t);
  if (gpsSecs < 0) {
    return;
  }

  // edge N happened deltaUs before nowUs (measured from the hardware-latched MCPWM capture count)
  updateTimescale(gpsSecs + NTP_EPOCH_OFFSET, nowUs - deltaUs);
}

void gpsTask(void *p) {
  esp_task_wdt_add(NULL);
  while (1) {
    if (!otaInProgress) {
      esp_task_wdt_reset();
      readGPSTime();
    }
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
  gpsSerial.begin(GPS_BAUD, SERIAL_8N1, GPS_RX_PIN, GPS_TX_PIN);
  initUbloxM10();
  initPpsCapture();
  
  Serial.println("----- start: network init -----");
  initNetwork();

  Serial.println("----- start: ota init -----");
  initOTA();
  
  Serial.println("----- start: ntp callback init -----");
  initRawNTP();

  Serial.println("----- start: start gps task -----");
  xTaskCreatePinnedToCore(gpsTask, "GPS", 4096, NULL, 2, &gpsTaskHandle, 1);

  Serial.println("----- start: complete -----");
}

void loop() { 
  ArduinoOTA.handle(); 
  delay(100); 
}