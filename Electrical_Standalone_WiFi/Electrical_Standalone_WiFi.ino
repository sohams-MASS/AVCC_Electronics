// =====================================================================
//  SAUMEL ELECTRICAL — WI-FI GUI (standalone)
//
//  Single-board test rig: this ESP32 hosts its own AP and serves a web
//  UI that lets you set per-channel enable / frequency / pulse / voltage
//  for all 12 channels independently. No master, no ESP-NOW.
//
//  AP:        SSID "Saumel_Electrical"  password "12345678"
//  URL:       http://192.168.4.1
//
//  Architecture mirrors Electrical_Receiver_V2:
//    • Core 1, prio 15: waveformTask owns DAC writes and the scheduler
//    • Core 1, prio 1:  loop() runs WebServer.handleClient()
//    • Updates cross from loop() → waveformTask via a FreeRTOS queue
//
//  Each channel runs independently with its own period_us and pulse_us;
//  on Apply, all enabled channels re-arm phase-aligned at "now + grace".
// =====================================================================

#include <Arduino.h>
#include <WiFi.h>
// Default WEBSERVER_MAX_POST_ARGS=32 is too small (12 ch × 4 fields = 48).
#define WEBSERVER_MAX_POST_ARGS 64
#include <WebServer.h>
#include <soc/gpio_struct.h>
#include <freertos/FreeRTOS.h>
#include <freertos/task.h>
#include <freertos/queue.h>

// Forward declaration so Arduino's auto-prototype generator compiles.
struct ChanState;

// ---------- PINS (verbatim from Electrical_Receiver_V2) ----------
static const int WR1 = 23;
static const int WR2 = 25;
static const int WR3 = 26;

static const int DB7 = 2;
static const int DB6 = 4;
static const int DB5 = 5;
static const int DB4 = 12;
static const int DB3 = 13;
static const int DB2 = 14;
static const int DB1 = 15;
static const int DB0 = 16;

static const int DAC_SEL1 = 18;
static const int DAC_SEL0 = 17;

static const uint8_t NCH = 12;

// ---------- DAC CODES ----------
static const uint8_t CODE_NEU = 0x80;  //   0V (DAC midpoint)

// ---------- GUARDRAILS ----------
static const float    MIN_FREQ_HZ   = 0.01f;
static const float    MAX_FREQ_HZ   = 10.0f;
static const float    MIN_AMP_V     = 0.0f;
static const float    MAX_AMP_V     = 15.0f;
static const uint32_t MIN_PULSE_US  = 50;
static const uint32_t APPLY_GRACE_US = 200000;  // 200 ms before first edge

// Map a signed voltage in [-15, +15] V to an 8-bit DAC code centered at 0x80.
static inline uint8_t voltsToCode(float v) {
  if (v < -MAX_AMP_V) v = -MAX_AMP_V;
  if (v > +MAX_AMP_V) v = +MAX_AMP_V;
  float code = ((v + MAX_AMP_V) / (2.0f * MAX_AMP_V)) * 255.0f;
  int   c    = (int)lroundf(code);
  if (c < 0) c = 0; if (c > 255) c = 255;
  return (uint8_t)c;
}

// ---------- WIFI / WEB ----------
static const uint8_t WIFI_CH = 1;
const char* AP_SSID = "Saumel_Electrical";
const char* AP_PASS = "12345678";
WebServer server(80);

// =====================================================================
//  CONFIG  (loop() only)
// =====================================================================
static bool     cfgEnable[NCH];
static float    cfgFreqHz[NCH];
static uint32_t cfgPulseUs[NCH];
static float    cfgAmpV[NCH];          // 0..15 V (mirrored around neutral)
static bool     runningAny = false;   // last applied state had any channel enabled

// =====================================================================
//  CROSS-TASK MESSAGES
// =====================================================================
enum ApplyKind : uint8_t { APPLY_RUN = 1, APPLY_STOP_ALL = 2 };

struct ApplyMsg {
  ApplyKind kind;
  bool      enabled[NCH];
  uint32_t  period_us[NCH];
  uint32_t  pulse_us[NCH];
  uint8_t   pos_code[NCH];
  uint8_t   neg_code[NCH];
};

static QueueHandle_t applyQueue = nullptr;
static const int APPLY_QUEUE_DEPTH = 4;

// =====================================================================
//  LOW-LEVEL DAC BUS
// =====================================================================
static inline void setBus8(uint8_t v) {
  uint32_t set = 0, clr = 0;
  if (v & 0x01) set |= (1UL << DB0); else clr |= (1UL << DB0);
  if (v & 0x02) set |= (1UL << DB1); else clr |= (1UL << DB1);
  if (v & 0x04) set |= (1UL << DB2); else clr |= (1UL << DB2);
  if (v & 0x08) set |= (1UL << DB3); else clr |= (1UL << DB3);
  if (v & 0x10) set |= (1UL << DB4); else clr |= (1UL << DB4);
  if (v & 0x20) set |= (1UL << DB5); else clr |= (1UL << DB5);
  if (v & 0x40) set |= (1UL << DB6); else clr |= (1UL << DB6);
  if (v & 0x80) set |= (1UL << DB7); else clr |= (1UL << DB7);
  GPIO.out_w1tc = clr;
  GPIO.out_w1ts = set;
}

static inline void channelToChipAndDac(uint8_t ch1based,
                                        int &wrPin, uint8_t &dacSel) {
  uint8_t idx     = ch1based - 1;
  uint8_t chipIdx = idx / 4;
  dacSel = idx % 4;
  wrPin  = (chipIdx == 0) ? WR1 : (chipIdx == 1) ? WR2 : WR3;
}

static inline void setDacSelect(uint8_t dacSel) {
  if (dacSel & 0x01) GPIO.out_w1ts = (1UL << DAC_SEL0);
  else               GPIO.out_w1tc = (1UL << DAC_SEL0);
  if (dacSel & 0x02) GPIO.out_w1ts = (1UL << DAC_SEL1);
  else               GPIO.out_w1tc = (1UL << DAC_SEL1);
}

static inline void strobeWR(int wrPin) {
  GPIO.out_w1tc = (1UL << wrPin);
  delayMicroseconds(2);
  GPIO.out_w1ts = (1UL << wrPin);
}

static inline uint8_t mapLogicalToPhysical(uint8_t ch1based) {
  switch (ch1based) {
    case 2:  return 3;
    case 3:  return 2;
    case 6:  return 7;
    case 7:  return 6;
    case 10: return 11;
    case 11: return 10;
    default: return ch1based;
  }
}

static inline void writeChannelCode(uint8_t ch1based, uint8_t code) {
  uint8_t phys = mapLogicalToPhysical(ch1based);
  int wrPin; uint8_t dacSel;
  channelToChipAndDac(phys, wrPin, dacSel);

  setBus8(code);
  delayMicroseconds(1);
  setDacSelect(dacSel);
  delayMicroseconds(1);
  strobeWR(wrPin);
}

// =====================================================================
//  WAVEFORM SCHEDULER STATE  (waveformTask only)
// =====================================================================
enum Phase : uint8_t { PH_IDLE = 0, PH_POS_WAIT, PH_POS_PULSE, PH_NEG_WAIT, PH_NEG_PULSE };

struct ChanState {
  Phase    phase;
  uint32_t period_us;
  uint32_t pulse_us;
  uint32_t nextEdge_us;
  uint8_t  pos_code;
  uint8_t  neg_code;
};

static ChanState chs[NCH];

static inline uint32_t nowUs() { return (uint32_t)micros(); }

static inline bool due(uint32_t t, uint32_t deadline) {
  return (int32_t)(t - deadline) >= 0;
}

static inline void clampTiming(uint32_t &period_us, uint32_t &pulse_us) {
  if (pulse_us < MIN_PULSE_US) pulse_us = MIN_PULSE_US;
  uint32_t half = period_us / 2;
  if (half == 0) half = 1;
  if (pulse_us > half) pulse_us = half;
}

static inline void parkAllNeutral() {
  for (uint8_t i = 0; i < NCH; i++) {
    chs[i].phase = PH_IDLE;
    writeChannelCode(i + 1, CODE_NEU);
  }
}

static void advanceChannel(uint8_t i) {
  ChanState &c = chs[i];
  uint32_t half = c.period_us / 2;
  if (half == 0) half = 1;

  switch (c.phase) {
    case PH_POS_WAIT:
      writeChannelCode(i + 1, c.pos_code);
      c.phase        = PH_POS_PULSE;
      c.nextEdge_us += c.pulse_us;
      break;
    case PH_POS_PULSE:
      writeChannelCode(i + 1, CODE_NEU);
      c.phase        = PH_NEG_WAIT;
      c.nextEdge_us += (half > c.pulse_us) ? (half - c.pulse_us) : 0;
      break;
    case PH_NEG_WAIT:
      writeChannelCode(i + 1, c.neg_code);
      c.phase        = PH_NEG_PULSE;
      c.nextEdge_us += c.pulse_us;
      break;
    case PH_NEG_PULSE:
      writeChannelCode(i + 1, CODE_NEU);
      c.phase        = PH_POS_WAIT;
      c.nextEdge_us += (half > c.pulse_us) ? (half - c.pulse_us) : 0;
      break;
    case PH_IDLE:
      break;
  }
}

static void applyMessage(const ApplyMsg &m) {
  // First park everything at neutral so a transition between configs
  // never leaves a channel mid-pulse at full amplitude.
  parkAllNeutral();

  if (m.kind == APPLY_STOP_ALL) return;

  uint32_t startAt = nowUs() + APPLY_GRACE_US;
  for (uint8_t i = 0; i < NCH; i++) {
    if (!m.enabled[i]) continue;
    chs[i].period_us   = m.period_us[i];
    chs[i].pulse_us    = m.pulse_us[i];
    clampTiming(chs[i].period_us, chs[i].pulse_us);
    chs[i].pos_code    = m.pos_code[i];
    chs[i].neg_code    = m.neg_code[i];
    chs[i].phase       = PH_POS_WAIT;
    chs[i].nextEdge_us = startAt;
  }
}

// Returns smallest gap (µs) from `now` to the next scheduled edge.
// UINT32_MAX if no channel is active.
static inline uint32_t usToNextEvent(uint32_t now) {
  uint32_t minGap = UINT32_MAX;
  for (uint8_t i = 0; i < NCH; i++) {
    if (chs[i].phase == PH_IDLE) continue;
    int32_t gap = (int32_t)(chs[i].nextEdge_us - now);
    if (gap <= 0) return 0;
    if ((uint32_t)gap < minGap) minGap = (uint32_t)gap;
  }
  return minGap;
}

// =====================================================================
//  WAVEFORM TASK  (core 1, prio 15)
// =====================================================================
static void waveformTask(void *) {
  const int MAX_EVENTS_PER_BURST = 200;

  while (true) {
    // 1. Drain pending apply messages — newest wins.
    ApplyMsg m;
    while (xQueueReceive(applyQueue, &m, 0) == pdTRUE) {
      applyMessage(m);
    }

    // 2. Run the per-channel scheduler.
    for (int events = 0; events < MAX_EVENTS_PER_BURST; events++) {
      uint32_t t = nowUs();
      int8_t fired = -1;
      for (uint8_t i = 0; i < NCH; i++) {
        if (chs[i].phase == PH_IDLE) continue;
        if (due(t, chs[i].nextEdge_us)) {
          advanceChannel(i);
          fired = i;
          break;
        }
      }
      if (fired < 0) break;
    }

    // 3. Adaptive yield (mirrors the receiver's strategy).
    uint32_t gap = usToNextEvent(nowUs());
    if (gap == UINT32_MAX) {
      vTaskDelay(pdMS_TO_TICKS(10));
    } else if (gap > 5000) {
      uint32_t sleepMs = (gap - 2000) / 1000;
      if (sleepMs > 0) vTaskDelay(pdMS_TO_TICKS(sleepMs));
    } else if (gap > 2000) {
      vTaskDelay(pdMS_TO_TICKS(1));
    } else if (gap > 500) {
      taskYIELD();
    }
    // gap ≤ 500 µs: spin-wait, no yield.
  }
}

// =====================================================================
//  WEB UI  (streamed HTML — no String accumulation in heap)
// =====================================================================
static const char* PAGE_HEAD =
  "<!DOCTYPE html><html><head><meta charset='utf-8'>"
  "<meta name='viewport' content='width=device-width,initial-scale=1'>"
  "<title>Saumel Electrical (standalone)</title>"
  "<style>"
  "body{font-family:system-ui,sans-serif;background:#111;color:#eee;margin:14px}"
  "h1{font-size:18px;margin:0 0 12px}"
  "table{border-collapse:collapse;width:100%;max-width:560px}"
  "th,td{padding:6px 8px;border-bottom:1px solid #333;text-align:left}"
  "th{font-weight:600;background:#1c1c1c}"
  "input[type=number]{width:96px;background:#222;color:#eee;border:1px solid #333;padding:4px;border-radius:3px}"
  ".ch{text-align:center;font-weight:700;width:40px}"
  ".btn{display:inline-block;margin:14px 6px 0 0;padding:8px 14px;border-radius:4px;border:0;font-weight:600;cursor:pointer}"
  ".apply{background:#2266dd;color:#fff}"
  ".stop{background:#aa3333;color:#fff}"
  ".status{margin-top:14px;font-size:13px;color:#9af}"
  "</style></head><body>";

static void sendChannelRow(int i) {
  char buf[480];
  int ch = i + 1;
  const char* chk = cfgEnable[i] ? " checked" : "";
  snprintf(buf, sizeof(buf),
    "<tr>"
      "<td class='ch'>%d</td>"
      "<td><input type='checkbox' name='en%d' value='1'%s></td>"
      "<td><input type='number' step='0.01' min='%.2f' max='%.2f' name='f%d' value='%.2f'></td>"
      "<td><input type='number' step='1' min='%lu' name='p%d' value='%lu'></td>"
      "<td><input type='number' step='0.1' min='%.1f' max='%.1f' name='a%d' value='%.1f'></td>"
    "</tr>",
    ch,
    ch, chk,
    MIN_FREQ_HZ, MAX_FREQ_HZ, ch, cfgFreqHz[i],
    (unsigned long)MIN_PULSE_US, ch, (unsigned long)cfgPulseUs[i],
    MIN_AMP_V, MAX_AMP_V, ch, cfgAmpV[i]
  );
  server.sendContent(buf);
}

static void handleRoot() {
  server.setContentLength(CONTENT_LENGTH_UNKNOWN);
  server.send(200, "text/html", "");
  server.sendContent(PAGE_HEAD);

  char hdr[320];
  snprintf(hdr, sizeof(hdr),
    "<h1>Saumel Electrical &mdash; standalone (%s)</h1>"
    "<form method='POST' action='/apply'>"
    "<table><thead><tr>"
    "<th>Ch</th><th>On</th><th>Hz</th><th>Pulse (&mu;s)</th><th>Amp (V)</th>"
    "</tr></thead><tbody>",
    runningAny ? "running" : "stopped");
  server.sendContent(hdr);

  for (int i = 0; i < NCH; i++) sendChannelRow(i);

  server.sendContent(
    "</tbody></table>"
    "<button class='btn apply' type='submit'>Apply</button>"
    "<button class='btn stop'  type='submit' formaction='/stop'>Stop All</button>"
    "</form>"
    "<div class='status'>"
    "Bipolar pulse: +V for pulse_us &rarr; 0V &rarr; &minus;V for pulse_us &rarr; 0V (V is per-channel amplitude)."
    " All enabled channels re-arm phase-aligned on Apply."
    "</div></body></html>");
  server.client().stop();
}

static float argFloatOrDefault(const String &k, float def) {
  if (!server.hasArg(k)) return def;
  return server.arg(k).toFloat();
}

static uint32_t argU32OrDefault(const String &k, uint32_t def) {
  if (!server.hasArg(k)) return def;
  long v = server.arg(k).toInt();
  if (v < 0) v = 0;
  return (uint32_t)v;
}

static void handleApply() {
  ApplyMsg m{};
  m.kind = APPLY_RUN;
  bool anyEnabled = false;

  for (int i = 0; i < NCH; i++) {
    String chS = String(i + 1);
    bool en  = server.hasArg("en" + chS);
    float f  = argFloatOrDefault("f"  + chS, cfgFreqHz[i]);
    uint32_t p = argU32OrDefault("p"  + chS, cfgPulseUs[i]);
    float a  = argFloatOrDefault("a"  + chS, cfgAmpV[i]);

    if (f < MIN_FREQ_HZ)  f = MIN_FREQ_HZ;
    if (f > MAX_FREQ_HZ)  f = MAX_FREQ_HZ;
    if (p < MIN_PULSE_US) p = MIN_PULSE_US;
    if (a < MIN_AMP_V)    a = MIN_AMP_V;
    if (a > MAX_AMP_V)    a = MAX_AMP_V;

    cfgEnable[i]  = en;
    cfgFreqHz[i]  = f;
    cfgPulseUs[i] = p;
    cfgAmpV[i]    = a;

    uint32_t period_us = (uint32_t)lroundf(1.0e6f / f);
    uint32_t pulse_us  = p;
    // local clamp so the snapshot pushed to waveformTask is already valid
    uint32_t half = period_us / 2;
    if (half > 0 && pulse_us > half) pulse_us = half;

    m.enabled[i]   = en;
    m.period_us[i] = period_us;
    m.pulse_us[i]  = pulse_us;
    m.pos_code[i]  = voltsToCode(+a);
    m.neg_code[i]  = voltsToCode(-a);
    if (en) anyEnabled = true;
  }
  runningAny = anyEnabled;

  xQueueSend(applyQueue, &m, 0);

  server.sendHeader("Location", "/");
  server.send(303, "text/plain", "");
}

static void handleStop() {
  ApplyMsg m{};
  m.kind = APPLY_STOP_ALL;
  xQueueSend(applyQueue, &m, 0);
  runningAny = false;

  server.sendHeader("Location", "/");
  server.send(303, "text/plain", "");
}

static void handleNotFound() {
  server.send(404, "text/plain", "Not found");
}

// =====================================================================
//  SETUP / LOOP
// =====================================================================
void setup() {
  Serial.begin(115200);
  delay(150);
  Serial.println();
  Serial.println("Saumel Electrical standalone (Wi-Fi GUI)");

  // Defaults: all enabled, 0.5 Hz, ch1=2000 µs ascending +1000 µs, ±15 V.
  for (int i = 0; i < NCH; i++) {
    cfgEnable[i]  = true;
    cfgFreqHz[i]  = 0.5f;
    cfgPulseUs[i] = 2000UL + (uint32_t)i * 1000UL;
    cfgAmpV[i]    = MAX_AMP_V;
  }

  // GPIO init.
  const int allPins[] = {
    DB7, DB6, DB5, DB4, DB3, DB2, DB1, DB0,
    DAC_SEL1, DAC_SEL0,
    WR1, WR2, WR3
  };
  for (int p : allPins) pinMode(p, OUTPUT);
  digitalWrite(WR1, HIGH);
  digitalWrite(WR2, HIGH);
  digitalWrite(WR3, HIGH);

  // Park every DAC at neutral before anything else can drive them.
  for (uint8_t i = 0; i < NCH; i++) {
    chs[i].phase = PH_IDLE;
    writeChannelCode(i + 1, CODE_NEU);
  }

  // FreeRTOS queue for cross-task config updates.
  applyQueue = xQueueCreate(APPLY_QUEUE_DEPTH, sizeof(ApplyMsg));
  if (!applyQueue) {
    Serial.println("FATAL: queue creation failed");
    while (true) vTaskDelay(1000);
  }

  // Wi-Fi AP.
  WiFi.mode(WIFI_AP);
  WiFi.setSleep(false);
  bool apOk = WiFi.softAP(AP_SSID, AP_PASS, WIFI_CH, false, 4);
  Serial.printf("softAP: %s\n", apOk ? "OK" : "FAIL");
  Serial.print("AP IP: "); Serial.println(WiFi.softAPIP());

  // Web routes.
  server.on("/",       HTTP_GET,  handleRoot);
  server.on("/apply",  HTTP_POST, handleApply);
  server.on("/stop",   HTTP_POST, handleStop);
  server.onNotFound(handleNotFound);
  server.begin();
  Serial.printf("Web UI ready: connect to '%s' (pw: %s) → http://192.168.4.1\n",
                AP_SSID, AP_PASS);

  // Detach core 1's IDLE WDT — waveformTask spin-waits at high prio.
  disableCore1WDT();

  // Spawn waveform task.
  xTaskCreatePinnedToCore(
    waveformTask, "waveform",
    4096, nullptr,
    15, nullptr,
    1
  );

  Serial.println("ready.");
}

void loop() {
  server.handleClient();
}
