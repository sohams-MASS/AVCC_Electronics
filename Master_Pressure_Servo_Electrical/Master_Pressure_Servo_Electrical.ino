// ===========================
//  AVCC MASTER (NO-STRING-HEAP WEB UI + ACK/STATUS PANEL)
//  ESP32 Arduino Core 2.0.14 compatible
//
//  Key fixes for "heap keeps going down / page stops loading":
//   - No giant HTML String assembly
//   - Avoid String(...) allocations in sendContent (use char buffers)
//   - Explicitly close client connection after response
//   - favicon + notFound handlers
//
//  Protocol:
//   - Master -> Slaves: Msg (CMD_SET/CMD_START/CMD_STOP/CMD_KEEPALIVE)
//   - Slaves -> Master: AckMsg (0xA1) + optional StatusMsg (0xA2 @ ~2Hz)
//
// ===========================

#include <esp_now.h>
#include <WiFi.h>
#include <esp_wifi.h>
// MUST be defined before WebServer.h is included.
// Default is 32 — far too small for our form (12 channels × 7 params = 84+ fields).
// Without this, params beyond the 32nd are silently dropped, meaning channels 5-12
// never update on Apply, and the "mode" button value (Apply+Start / Stop) is NEVER
// received (it sits at position ~87 in the POST body).
#define WEBSERVER_MAX_POST_ARGS 100
#include <WebServer.h>
#include <math.h>
#include <Preferences.h>

// ====================== TYPE DEFINITIONS ======================
// These must appear before any function that uses them as a parameter or
// return type. The Arduino IDE auto-generates forward function prototypes
// and inserts them near the top of the file; if SlaveIdx / MasterState are
// defined further down, those prototypes fail to compile.
enum MasterState : uint8_t {
  MS_IDLE,          // waiting for user input
  MS_WAIT_ACKS,     // sent SETs (and maybe STARTs), waiting for ACK deadline
  MS_RUNNING        // all channels active, sending keepalives
};

enum SlaveIdx : int { S_ELEC=0, S_PRES=1, S_MOTO=2 };

// ====================== SLAVE MACs ======================
// Use BICEP_MAC_Address sketch to read each device's MAC, then update here.
//uint8_t mac_elec[]  = {0x68, 0xFE, 0x71, 0x0D, 0x80, 0xB0}; // Electrical slave
//uint8_t mac_press[] = {0x68, 0xFE, 0x71, 0x0C, 0xB9, 0x50}; // Pressure slave
//uint8_t mac_motor[] = {0x68, 0xFE, 0x71, 0x0C, 0xB7, 0xF8}; // Servo/Motor slave

uint8_t mac_elec[]  = {0x80, 0xF3, 0xDA, 0x41, 0x68, 0x10}; // Electrical slave80:F3:DA:41:68:10
uint8_t mac_press[] = {0x68, 0xFE, 0x71, 0x0C, 0xB9, 0x50}; // Pressure slave
uint8_t mac_motor[] = {0x68, 0xFE, 0x71, 0x0C, 0xB7, 0xF8}; // Servo/Motor slave





// ====================== WIFI / AP ======================
static const uint8_t WIFI_CH = 1;
const char* AP_SSID = "AVCC_Master 5";
const char* AP_PASS = "12345678";
WebServer server(80);

// ====================== NVS ======================
Preferences prefs;
static const char* NVS_NS      = "avcc";
static const char* NVS_KEY_RID = "rid";   // persisted run_id high-water mark

// ====================== ESPNOW ======================
static const uint32_t KEEPALIVE_PERIOD_MS = 150; // ~6 Hz (reduced from 20 Hz to avoid congestion with 3 slaves)
static uint32_t lastKeepMs = 0;

enum Cmd : uint8_t { CMD_SET=1, CMD_START=2, CMD_STOP=3, CMD_KEEPALIVE=4 };

// START target flags:
static const uint8_t FLG_TGT_ELEC = 0x02;
static const uint8_t FLG_TGT_PRES = 0x04;
static const uint8_t FLG_TGT_MOTO = 0x08;

// SET flags:
static const uint8_t FLG_SET_MIRROR = 0x01;

// ====================== MESSAGE ======================
typedef struct __attribute__((packed)) {
  uint8_t  cmd;
  uint8_t  ch;            // 1..12, or 0 = all
  uint8_t  flags;
  uint8_t  neutral_code;

  uint32_t run_id;
  uint32_t seq;
  uint32_t period_us;
  uint32_t pulse_us;

  uint8_t  pos_code;
  uint8_t  neg_code;
  uint8_t  rsv0;
  uint8_t  rsv1;

  uint32_t start_delay_us;
  uint32_t phase_offset_us;
  uint32_t duration_ms;   // pressure duration (ms); motor ignores or can use later
} Msg;

static const int NCH = 12;
static const uint8_t NEUTRAL_CODE = 0x80;

// ====================== SYNC TUNING ======================
static const uint32_t START_DELAY_US_DEFAULT = 600000; // 600 ms

// ====================== UI TOGGLES ======================
static bool debugEnabled = false;
static bool bootAutoStart = false;
static bool bootAutoStarted = false;
static const uint32_t BOOT_GRACE_MS = 3000; // give slaves time to join before auto-start

// ====================== PER-CHANNEL SETTINGS ======================
float    chFreqHz[NCH];
uint32_t chPulseUs[NCH];
float    chAmpV[NCH];
bool     chEnable[NCH];

// per-channel pressure settings
uint32_t chPressDurMs[NCH];         // pressure duration (ms)
uint32_t chPressPhaseUs[NCH];       // pressure phase offset (us)
uint32_t chMotorPhaseUs[NCH];       // motor phase offset (us)

// sequencing (global monotonically increases)
uint32_t seq_ch[NCH];

static volatile uint32_t run_id  = 1;  // volatile: read in callback (onEspNowRecv)
static volatile bool     running = false;

// ====================== MASTER STATE MACHINE ======================
// MasterState is defined near the top of the file (before any function
// that uses it) to satisfy the Arduino auto-prototype generator.

enum RequestType : uint8_t { REQ_APPLY_ONLY, REQ_APPLY_START, REQ_STOP };

struct PendingRequest {
  RequestType type;
  bool valid = false;
};

static MasterState   masterState       = MS_IDLE;
static PendingRequest pendingReq       = { REQ_STOP, false };
static bool          applyIncludedStart = false;

// ====================== ELECTRICAL LIVENESS ======================
// In MS_RUNNING, Electrical's StatusMsg arrives at ~1 Hz. If it goes
// silent for longer than this, mirror its local 500 ms heartbeat
// failsafe at the master so the user sees IDLE state and is prompted.
static const uint32_t ELEC_SILENT_TIMEOUT_MS = 2000;

// ====================== AUTO-RETRANSMIT ======================
static const uint32_t RETRANSMIT_DELAY_MS = 600;
static const int      MAX_RETRANSMITS     = 3;
static int            retransmitCount     = 0;
static uint32_t       waitAckDeadline     = 0;
static uint16_t       unackedBitmap[3]    = {0, 0, 0}; // per-slave bitmask of unacked channels

// snapshot buffers from web form
float    nextFreqHz[NCH];
uint32_t nextPulseUs[NCH];
float    nextAmpV[NCH];
bool     nextEnable[NCH];

uint32_t nextPressDurMs[NCH];
uint32_t nextPressPhaseUs[NCH];
uint32_t nextMotorPhaseUs[NCH];

// dirty flag (RAM differs from saved flash snapshot)
static bool dirtySettings = false;

// ====================== ACK / STATUS (RX) ======================
static const uint8_t ACK_TYPE    = 0xA1;
static const uint8_t STATUS_TYPE = 0xA2;

typedef struct __attribute__((packed)) {
  uint8_t  type;     // 0xA1
  uint8_t  role;     // 1=Electrical, 2=Pressure, 3=Motor
  uint8_t  cmd;      // CMD_SET/CMD_START/CMD_STOP
  uint8_t  ch;       // 1..12 (0=all)
  uint32_t run_id;
  uint32_t seq;
  uint8_t  ok;
  uint8_t  rsv[3];
} AckMsg;

// Receiver status heartbeat (sent by slaves ~2 Hz)
typedef struct __attribute__((packed)) {
  uint8_t  type;       // 0xA2
  uint8_t  role;       // 1=Electrical, 2=Pressure, 3=Motor
  uint8_t  rsv0;
  uint8_t  rsv1;

  uint32_t millis_now;      // slave millis()
  uint32_t run_id;
  uint32_t last_seq_max;    // slave's last seen seq (optional)
  uint16_t hb_age_ms;       // slave's keepalive age
  uint16_t free_heap;       // slave ESP.getFreeHeap()
} StatusMsg;

struct SlaveHealth {
  bool     seen = false;
  uint32_t lastSeenMs = 0;
  uint32_t lastRunId  = 0;
  uint32_t lastSeqMax = 0;
  uint8_t  lastRole   = 0;

  uint32_t ackSeqCh[NCH] = {0};      // last acked seq per channel
  uint32_t expSeqCh[NCH] = {0};      // last expected seq per channel (for this slave)

  uint16_t lastHbAgeMs   = 0;
  uint16_t lastFreeHeap  = 0;
  uint32_t lastStatusSlaveMs = 0;
};

// SlaveIdx is defined near the top of the file (auto-prototype safe).
SlaveHealth H[3];

// ====================== UTILITIES ======================
static inline uint8_t clampU8(int v){ if(v<0) return 0; if(v>255) return 255; return (uint8_t)v; }

static inline uint8_t voltsToCode(float v){
  if (v < -15.0f) v = -15.0f;
  if (v > +15.0f) v = +15.0f;
  float code = ((v + 15.0f) / 30.0f) * 255.0f;
  return clampU8((int)lroundf(code));
}

static inline uint8_t ampToPosCode(float A){
  if (A < 0.0f) A = 0.0f;
  if (A > 15.0f) A = 15.0f;
  return voltsToCode(+A);
}

static inline uint32_t hzToPeriodUs(float hz){
  if (hz < 0.01f) hz = 0.01f;
  if (hz > 10.0f) hz = 10.0f;
  return (uint32_t)lroundf(1e6f / hz);
}

// ====================== ESPNOW SEND ======================
static inline void logSendErr(esp_err_t e){
  if (!debugEnabled) return;
  Serial.printf("esp_now_send failed: e=%d (0x%08X)\n", (int)e, (unsigned int)e);
}

static inline void sendOne(const uint8_t *mac, const Msg &m){
  esp_err_t e = esp_now_send(mac, (const uint8_t*)&m, sizeof(Msg));
  if (e != ESP_OK) logSendErr(e);
}

static inline void sendToSlave(SlaveIdx s, const Msg &m){
  if (s == S_ELEC) sendOne(mac_elec, m);
  if (s == S_PRES) sendOne(mac_press, m);
  if (s == S_MOTO) sendOne(mac_motor, m);
}

// Send SET to all three slaves. ESP-NOW queue depth is 10; we only queue 3 packets
// per call (one per slave), so no inter-packet delay is needed.
static inline void sendSetToAll(const Msg &m){
  sendOne(mac_elec,  m);
  sendOne(mac_press, m);
  sendOne(mac_motor, m);
  yield();
}

static inline Msg makeKeep(){
  Msg m{};
  m.cmd = CMD_KEEPALIVE;
  m.ch  = 0;
  m.run_id = run_id;
  return m;
}

static inline Msg makeStop(){
  Msg m{};
  m.cmd    = CMD_STOP;
  m.run_id = run_id;
  return m;
}

static inline Msg makeSet(uint8_t ch1based, uint32_t seq,
                          uint32_t period_us, uint32_t pulse_us,
                          uint8_t neutral, uint8_t pos, uint8_t neg, bool mirror){
  Msg m{};
  m.cmd = CMD_SET;
  m.ch  = ch1based;
  m.flags = mirror ? FLG_SET_MIRROR : 0x00;
  m.neutral_code = neutral;
  m.run_id = run_id;
  m.seq = seq;
  m.period_us = period_us;
  m.pulse_us  = pulse_us;
  m.pos_code  = pos;
  m.neg_code  = neg;
  return m;
}

static inline Msg makeStart(uint8_t ch1based, uint32_t seq,
                            uint8_t tgtFlags,
                            uint32_t start_delay_us,
                            uint32_t phase_offset_us,
                            uint32_t duration_ms){
  Msg m{};
  m.cmd = CMD_START;
  m.ch  = ch1based;
  m.flags = tgtFlags;
  m.neutral_code = NEUTRAL_CODE;
  m.run_id = run_id;
  m.seq = seq;
  m.start_delay_us  = start_delay_us;
  m.phase_offset_us = phase_offset_us;
  m.duration_ms     = duration_ms;
  return m;
}

static void stopAllNow(){
  running = false;
  retransmitCount   = 0;
  waitAckDeadline   = 0;
  memset(unackedBitmap, 0, sizeof(unackedBitmap));
  Msg st = makeStop();
  sendToSlave(S_ELEC, st);
  sendToSlave(S_PRES, st);
  sendToSlave(S_MOTO, st);
  if (debugEnabled) Serial.println("STOP sent");
}

// ====================== PEERS (AP interface — stable on Core 3.x) ======================
// On ESP-IDF 5.x the STA interface has no active connection, so sending
// ESP-NOW on it can destabilise the radio and tear down the soft-AP.
// Routing peers through WIFI_IF_AP keeps the AP interface (and its channel)
// in sole control of the radio.  channel=0 = "use current channel".
static void addPeer(const uint8_t *mac){
  esp_now_peer_info_t p{};
  memcpy(p.peer_addr, mac, 6);
  p.channel = 0;
  p.encrypt = false;
  p.ifidx   = WIFI_IF_AP;
  esp_err_t e = esp_now_add_peer(&p);
  if (e != ESP_OK && e != ESP_ERR_ESPNOW_EXIST) {
    Serial.printf("add_peer FAIL e=%d\n", (int)e);
  }
}

// ====================== RX CALLBACK ======================
static inline int whichSlave(const uint8_t* mac){
  if (!memcmp(mac, mac_elec,  6)) return S_ELEC;
  if (!memcmp(mac, mac_press, 6)) return S_PRES;
  if (!memcmp(mac, mac_motor, 6)) return S_MOTO;
  return -1;
}

static void onEspNowRecv(const esp_now_recv_info_t *info, const uint8_t *data, int len){
  const uint8_t *mac = info->src_addr;
  int s = whichSlave(mac);
  if (s < 0) return;

  if (len == (int)sizeof(AckMsg)) {
    AckMsg a{};
    memcpy(&a, data, sizeof(a));
    if (a.type != ACK_TYPE) return;

    H[s].seen = true;
    H[s].lastSeenMs = millis();
    H[s].lastRunId  = a.run_id;
    H[s].lastRole   = a.role;
    if (a.seq > H[s].lastSeqMax) H[s].lastSeqMax = a.seq;

    if (a.ch >= 1 && a.ch <= NCH) {
      int c = (int)a.ch - 1;
      if (a.seq > H[s].ackSeqCh[c]) H[s].ackSeqCh[c] = a.seq;
    }
    return;
  }

  if (len == (int)sizeof(StatusMsg)) {
    StatusMsg st{};
    memcpy(&st, data, sizeof(st));
    if (st.type != STATUS_TYPE) return;

    H[s].seen = true;
    H[s].lastSeenMs = millis();
    H[s].lastRunId  = st.run_id;
    H[s].lastRole   = st.role;
    if (st.last_seq_max > H[s].lastSeqMax) H[s].lastSeqMax = st.last_seq_max;

    H[s].lastHbAgeMs = st.hb_age_ms;
    H[s].lastFreeHeap = st.free_heap;
    H[s].lastStatusSlaveMs = st.millis_now;
    return;
  }
}

// ====================== NVS: DEFAULTS / LOAD / SAVE ======================
static void setFactoryDefaultsToRAM(){
  debugEnabled  = false;
  bootAutoStart = false;
  bootAutoStarted = false;

  for (int i=0;i<NCH;i++){
    chEnable[i] = true;
    chFreqHz[i] = 0.5f;
    chPulseUs[i]= 2000;
    chAmpV[i]   = 15.0f;

    // Default pressure duration = 30*11000 us = 330 ms
    chPressDurMs[i]   = 330;
    chPressPhaseUs[i] = 0;
    chMotorPhaseUs[i] = 0;

    nextEnable[i] = chEnable[i];
    nextFreqHz[i] = chFreqHz[i];
    nextPulseUs[i]= chPulseUs[i];
    nextAmpV[i]   = chAmpV[i];

    nextPressDurMs[i]   = chPressDurMs[i];
    nextPressPhaseUs[i] = chPressPhaseUs[i];
    nextMotorPhaseUs[i] = chMotorPhaseUs[i];
  }

  dirtySettings = true;
}

static void loadSettingsFromFlashToRAM(){
  prefs.begin(NVS_NS, true);

  debugEnabled  = prefs.getBool("dbg", false);
  bootAutoStart = prefs.getBool("boot", false);

  for (int i=0;i<NCH;i++){
    char k[16];

    snprintf(k,sizeof(k),"en%02d", i); chEnable[i] = prefs.getBool(k, true);
    snprintf(k,sizeof(k),"f%02d",  i); chFreqHz[i] = prefs.getFloat(k, 0.5f);
    snprintf(k,sizeof(k),"pu%02d", i); chPulseUs[i]= prefs.getUInt(k, 2000);
    snprintf(k,sizeof(k),"a%02d",  i); chAmpV[i]   = prefs.getFloat(k, 15.0f);

    snprintf(k,sizeof(k),"pd%02d", i); chPressDurMs[i]   = prefs.getUInt(k, 330);
    snprintf(k,sizeof(k),"pp%02d", i); chPressPhaseUs[i] = prefs.getUInt(k, 0);
    snprintf(k,sizeof(k),"mp%02d", i); chMotorPhaseUs[i] = prefs.getUInt(k, 0);
  }

  prefs.end();

  for (int i=0;i<NCH;i++){
    nextEnable[i] = chEnable[i];
    nextFreqHz[i] = chFreqHz[i];
    nextPulseUs[i]= chPulseUs[i];
    nextAmpV[i]   = chAmpV[i];

    nextPressDurMs[i]   = chPressDurMs[i];
    nextPressPhaseUs[i] = chPressPhaseUs[i];
    nextMotorPhaseUs[i] = chMotorPhaseUs[i];
  }

  dirtySettings = false;
}

static void saveSettingsToFlashFromRAM(){
  prefs.begin(NVS_NS, false);

  prefs.putBool("dbg",  debugEnabled);
  prefs.putBool("boot", bootAutoStart);

  for (int i=0;i<NCH;i++){
    char k[16];

    snprintf(k,sizeof(k),"en%02d", i); prefs.putBool(k,  chEnable[i]);
    snprintf(k,sizeof(k),"f%02d",  i); prefs.putFloat(k, chFreqHz[i]);
    snprintf(k,sizeof(k),"pu%02d", i); prefs.putUInt(k,  chPulseUs[i]);
    snprintf(k,sizeof(k),"a%02d",  i); prefs.putFloat(k, chAmpV[i]);

    snprintf(k,sizeof(k),"pd%02d", i); prefs.putUInt(k,  chPressDurMs[i]);
    snprintf(k,sizeof(k),"pp%02d", i); prefs.putUInt(k,  chPressPhaseUs[i]);
    snprintf(k,sizeof(k),"mp%02d", i); prefs.putUInt(k,  chMotorPhaseUs[i]);
  }

  prefs.end();
  dirtySettings = false;

  if (debugEnabled) Serial.println("Saved settings to flash");
}

// ---------- run_id persistence ----------
// Why: slaves only reset their per-channel lastSeq[] when run_id changes.
// If the master reboots and starts a new run with the *same* run_id, the
// slave's stale lastSeq rejects every new packet → run silently fails to
// arm and slave's 500 ms failsafe stops it. Persisting run_id and bumping
// it on every boot + every new run guarantees strict monotonicity across
// master reboots.
static void saveRunIdToFlash(uint32_t v){
  prefs.begin(NVS_NS, false);
  prefs.putUInt(NVS_KEY_RID, v);
  prefs.end();
}

static void initRunIdFromFlash(){
  prefs.begin(NVS_NS, true);
  uint32_t stored = prefs.getUInt(NVS_KEY_RID, 0);
  prefs.end();
  run_id = stored + 1;
  saveRunIdToFlash(run_id);
}

// ====================== WEB UI HELPERS ======================
static float argFloatOrDefault(const String& k, float def){
  if(!server.hasArg(k)) return def;
  return server.arg(k).toFloat();
}

static uint32_t argU32OrDefault(const String& k, uint32_t def){
  if(!server.hasArg(k)) return def;
  long v = server.arg(k).toInt();
  if (v < 0) v = 0;
  return (uint32_t)v;
}

static int pendingCountForSlave(SlaveIdx s){
  int pending = 0;
  for (int i=0;i<NCH;i++){
    if (!chEnable[i]) continue;
    if (H[s].ackSeqCh[i] < H[s].expSeqCh[i]) pending++;
  }
  return pending;
}

static const char* roleNameC(uint8_t r){
  if (r == 1) return "Electrical";
  if (r == 2) return "Pressure";
  if (r == 3) return "Motor";
  return "?";
}

static const char* healthTagC(SlaveIdx s){
  if (!H[s].seen) return "DOWN";
  uint32_t age = millis() - H[s].lastSeenMs;
  // Slaves send STATUS at 1 Hz + ACKs on every command.
  // 2500ms = 2.5 periods: fine for a quiet (no-command) session.
  // 6000ms = 6 periods: still alive but very slow / intermittent.
  if (age < 2500) return "OK";
  if (age < 6000) return "WARN";
  return "DOWN";
}

static const char* healthClassC(const char* tag){
  if (!strcmp(tag,"OK")) return "ok";
  if (!strcmp(tag,"WARN")) return "warn";
  return "down";
}

static const char* stateNameC(MasterState s){
  switch (s) {
    case MS_IDLE:      return "IDLE";
    case MS_WAIT_ACKS: return "WAIT_ACKS";
    case MS_RUNNING:   return "RUNNING";
    default:           return "?";
  }
}

// ====================== STREAMED HTML (LOW HEAP) ======================
static void sendRow(int i){
  int ch = i + 1;
  char buf[760];
  const char* chk = chEnable[i] ? " checked" : "";

  // NOTE: using snprintf to a stack buffer then sendContent(buf) (NO String)
  snprintf(buf, sizeof(buf),
    "<tr>"
      "<td style='text-align:center;font-weight:700'>%d</td>"
      "<td style='text-align:center'><input type='checkbox' name='en%d' value='1'%s></td>"
      "<td><input name='f%d'  type='number' step='0.01' min='0.01' max='10' value='%.2f'></td>"
      "<td><input name='p%d'  type='number' step='1' min='50' value='%lu'></td>"
      "<td><input name='a%d'  type='number' step='0.1' min='0' max='15' value='%.1f'></td>"
      "<td><input name='pd%d' type='number' step='1' min='0' value='%lu'></td>"
      "<td><input name='pph%d' type='number' step='1' min='0' value='%lu'></td>"
      "<td><input name='mph%d' type='number' step='1' min='0' value='%lu'></td>"
    "</tr>",
    ch,
    ch, chk,
    ch, chFreqHz[i],
    ch, (unsigned long)chPulseUs[i],
    ch, chAmpV[i],
    ch, (unsigned long)chPressDurMs[i],
    ch, (unsigned long)chPressPhaseUs[i],
    ch, (unsigned long)chMotorPhaseUs[i]
  );
  server.sendContent(buf);
}

static void sendSlaveRowClean(int idx, const char* name){
  const char* tag = healthTagC((SlaveIdx)idx);
  const char* cls = healthClassC(tag);
  uint32_t age = H[idx].seen ? (millis() - H[idx].lastSeenMs) : 0;

  char lastSeen[32];
  if (!H[idx].seen) snprintf(lastSeen, sizeof(lastSeen), "never");
  else snprintf(lastSeen, sizeof(lastSeen), "%lu ms", (unsigned long)age);

  char rbuf[620];
  snprintf(rbuf, sizeof(rbuf),
    "<tr><td>%s</td>"
    "<td><span class='%s'>%s</span></td>"
    "<td>%s</td>"
    "<td>%s</td>"
    "<td>%lu</td><td>%lu</td><td>%d</td>"
    "<td>%u</td><td>%u</td></tr>",
    name,
    cls, tag,
    lastSeen,
    roleNameC(H[idx].lastRole),
    (unsigned long)H[idx].lastRunId,
    (unsigned long)H[idx].lastSeqMax,
    pendingCountForSlave((SlaveIdx)idx),
    (unsigned)H[idx].lastHbAgeMs,
    (unsigned)H[idx].lastFreeHeap
  );
  server.sendContent(rbuf);
}

static void sendPage(){
  server.sendHeader("Cache-Control", "no-store");
  server.sendHeader("Connection", "close");
  server.setContentLength(CONTENT_LENGTH_UNKNOWN);
  server.send(200, "text/html", ""); // start chunked response

  server.sendContent(
    "<!doctype html><html><head><meta name='viewport' content='width=device-width,initial-scale=1'>"
    "<title>AVCC Control</title><style>"
    "body{font-family:Arial;margin:18px}.card{max-width:1450px;border:1px solid #ddd;border-radius:14px;padding:14px}"
    "table{border-collapse:collapse;width:100%}th,td{border-bottom:1px solid #eee;padding:8px}"
    "th{background:#fafafa;text-align:left}input{width:100%;padding:8px;font-size:14px;box-sizing:border-box}"
    ".btnrow{display:flex;gap:10px;flex-wrap:wrap;margin-top:12px}"
    "button{padding:10px 14px;font-size:15px;border-radius:10px;border:1px solid #ccc;background:#fff;cursor:pointer}"
    ".primary{background:#f6f6f6}.muted{color:#666;font-size:13px;margin:6px 0}"
    ".tog{display:flex;align-items:center;gap:8px;margin:8px 0}"
    ".warn{color:#b45309;font-weight:700}.ok{color:#16a34a;font-weight:700}.down{color:#dc2626;font-weight:700}"
    ".danger{border-color:#ef4444;background:#fff5f5}"
    "</style></head><body><div class='card'>"
    "<h2>AVCC Master 12 Channel Control</h2>"
    "<div class='muted'>Wi-Fi: <b>AVCC_Master</b>  URL: <b>http://192.168.4.1</b></div>"
  );

  char head[720];
  snprintf(head, sizeof(head),
    "<div class='muted'>State: <b>%s</b>  run_id: %lu  running: %s</div>"
    "<div class='muted'>Start delay: <b>%lu us</b></div>"
    "<div class='muted'>Flash saved: <b>%s</b></div>"
    "<div class='muted'>Master heap: <b>%u</b>  maxAlloc: <b>%u</b></div>",
    stateNameC(masterState),
    (unsigned long)run_id,
    running ? "YES" : "NO",
    (unsigned long)START_DELAY_US_DEFAULT,
    dirtySettings ? "<span class='warn'>NO (unsaved changes)</span>" : "YES",
    (unsigned)ESP.getFreeHeap(),
    (unsigned)ESP.getMaxAllocHeap()
  );
  server.sendContent(head);

  server.sendContent(
    "<h3>Status</h3><table>"
    "<tr><th>Slave</th><th>Health</th><th>Last seen</th><th>Role</th><th>run_id</th>"
    "<th>last seq</th><th>pending chans</th><th>hb age (ms)</th><th>heap</th></tr>"
  );

  sendSlaveRowClean(S_ELEC, "Slave 1");
  sendSlaveRowClean(S_PRES, "Slave 2");
  sendSlaveRowClean(S_MOTO, "Slave 3");

  server.sendContent("</table>");

  server.sendContent("<form method='POST' action='/apply'>");

  server.sendContent("<div class='tog'><input type='checkbox' name='dbg' value='1'");
  if (debugEnabled) server.sendContent(" checked");
  server.sendContent("><span>Debug (more Serial output)</span></div>");

  server.sendContent("<div class='tog'><input type='checkbox' name='boot' value='1'");
  if (bootAutoStart) server.sendContent(" checked");
  server.sendContent("><span>Boot Auto-Start (after grace period)</span></div>");

  server.sendContent(
    "<table><tr>"
    "<th style='width:55px'>CH</th><th style='width:55px'>EN</th>"
    "<th>Freq (Hz)</th><th>Pulse (&micro;s)</th><th>Amp (V)</th>"
    "<th>Pressure duration (ms)</th><th>Pressure phase offset (us)</th><th>Motor phase offset (us)</th>"
    "</tr>"
  );

  for (int i=0;i<NCH;i++) sendRow(i);
  server.sendContent("</table>");

  server.sendContent(
    "<div class='btnrow'>"
    "<button class='primary' name='mode' value='apply'>Apply (RAM only)</button>"
    "<button class='primary' name='mode' value='start'>Apply + Start (RAM only)</button>"
    "<button name='mode' value='stop'>Stop</button>"
    "</div></form>"
  );

  server.sendContent(
    "<div class='btnrow'>"
    "<form method='POST' action='/save' style='margin:0'><button class='primary' type='submit'>Save settings to flash</button></form>"
    "<form method='POST' action='/load' style='margin:0'><button class='primary' type='submit'>Load saved settings</button></form>"
    "<form method='POST' action='/factory' style='margin:0'><button class='primary danger' type='submit'>Factory defaults (RAM only)</button></form>"
    "<form method='POST' action='/factory_save' style='margin:0'><button class='primary danger' type='submit'>Factory defaults + Save</button></form>"
    "</div>"
  );

  server.sendContent("</div></body></html>");

  // terminate chunked response + force socket close
  server.sendContent("");
  delay(1);
  server.client().stop();
}

// ====================== WEB HANDLERS ======================
static void handleRoot(){ sendPage(); }

static void handleApply(){
  String mode = server.arg("mode");

  debugEnabled  = server.hasArg("dbg");
  bootAutoStart = server.hasArg("boot");
  if (!bootAutoStart) bootAutoStarted = false;

  for (int i=0;i<NCH;i++){
    int ch=i+1;
    nextEnable[i]  = server.hasArg("en"+String(ch));
    nextFreqHz[i]  = argFloatOrDefault("f"+String(ch), chFreqHz[i]);
    nextPulseUs[i] = argU32OrDefault("p"+String(ch), chPulseUs[i]);
    nextAmpV[i]    = argFloatOrDefault("a"+String(ch), chAmpV[i]);

    nextPressDurMs[i]   = argU32OrDefault("pd"+String(ch),  chPressDurMs[i]);
    nextPressPhaseUs[i] = argU32OrDefault("pph"+String(ch), chPressPhaseUs[i]);
    nextMotorPhaseUs[i] = argU32OrDefault("mph"+String(ch), chMotorPhaseUs[i]);

    if (nextFreqHz[i] < 0.01f) nextFreqHz[i] = 0.01f;
    if (nextFreqHz[i] > 10.0f) nextFreqHz[i] = 10.0f;

    float half_us = (1e6f / nextFreqHz[i]) * 0.5f;
    if ((float)nextPulseUs[i] > half_us) nextPulseUs[i] = (uint32_t)floorf(half_us);
    if (nextPulseUs[i] < 50) nextPulseUs[i] = 50;  // match slave MIN_PULSE_US

    if (nextAmpV[i] < 0.0f) nextAmpV[i] = 0.0f;
    if (nextAmpV[i] > 15.0f) nextAmpV[i] = 15.0f;
  }

  if (mode == "stop") {
    pendingReq.type = REQ_STOP;
  } else if (mode == "start") {
    pendingReq.type = REQ_APPLY_START;
  } else {
    pendingReq.type = REQ_APPLY_ONLY;
  }
  pendingReq.valid = true;

  server.sendHeader("Connection", "close");
  server.sendHeader("Location", "/");
  server.send(303);
  server.client().stop();
}

static void handleSave(){
  saveSettingsToFlashFromRAM();
  server.sendHeader("Connection", "close");
  server.sendHeader("Location", "/");
  server.send(303);
  server.client().stop();
}

static void handleLoad(){
  loadSettingsFromFlashToRAM();
  pendingReq.valid = false; masterState = MS_IDLE;
  server.sendHeader("Connection", "close");
  server.sendHeader("Location", "/");
  server.send(303);
  server.client().stop();
}

static void handleFactory(){
  setFactoryDefaultsToRAM();
  pendingReq.valid = false; masterState = MS_IDLE;
  server.sendHeader("Connection", "close");
  server.sendHeader("Location", "/");
  server.send(303);
  server.client().stop();
}

static void handleFactorySave(){
  setFactoryDefaultsToRAM();
  saveSettingsToFlashFromRAM();
  pendingReq.valid = false; masterState = MS_IDLE;
  server.sendHeader("Connection", "close");
  server.sendHeader("Location", "/");
  server.send(303);
  server.client().stop();
}

static void handleFavicon(){
  server.sendHeader("Connection", "close");
  server.send(204);
  server.client().stop();
}

static void handleNotFound(){
  server.sendHeader("Connection", "close");
  server.send(404, "text/plain", "Not found");
  server.client().stop();
}

// ====================== STATE MACHINE HELPERS ======================

// Commit web form snapshot into active channel settings
static void commitSnapshot(){
  for (int i=0;i<NCH;i++){
    chEnable[i] = nextEnable[i];
    chFreqHz[i] = nextFreqHz[i];
    chPulseUs[i]= nextPulseUs[i];
    chAmpV[i]   = nextAmpV[i];

    chPressDurMs[i]   = nextPressDurMs[i];
    chPressPhaseUs[i] = nextPressPhaseUs[i];
    chMotorPhaseUs[i] = nextMotorPhaseUs[i];
  }
  dirtySettings = true;
}

// Send SET for all 12 channels in one burst (replaces one-per-loop applyOneStep)
static void burstAllSets(){
  memset(unackedBitmap, 0, sizeof(unackedBitmap));

  for (int i=0;i<NCH;i++){
    uint8_t ch = (uint8_t)(i+1);
    seq_ch[i]++;

    if (chEnable[i]){
      uint32_t period_us = hzToPeriodUs(chFreqHz[i]);
      uint32_t pulse_us  = chPulseUs[i];
      uint8_t pos        = ampToPosCode(chAmpV[i]);

      Msg mset = makeSet(ch, seq_ch[i], period_us, pulse_us, NEUTRAL_CODE, pos, 0x00, true);

      H[S_ELEC].expSeqCh[i] = seq_ch[i];
      H[S_PRES].expSeqCh[i] = seq_ch[i];
      H[S_MOTO].expSeqCh[i] = seq_ch[i];

      sendSetToAll(mset);

      // Mark all 3 slaves as expecting ack for this channel
      unackedBitmap[S_ELEC] |= (1u << i);
      unackedBitmap[S_PRES] |= (1u << i);
      unackedBitmap[S_MOTO] |= (1u << i);

      if (debugEnabled) {
        Serial.printf("SET ch=%u seq=%lu freq=%.2f pulse=%lu amp=%.1f\n",
                      ch, (unsigned long)seq_ch[i], chFreqHz[i], (unsigned long)chPulseUs[i], chAmpV[i]);
      }
    }
    yield(); // let ESP-NOW queue drain after each channel's 3 packets
  }
}

// Send START for all enabled channels to each slave
static void burstAllStarts(){
  int started = 0;
  for (int k=0;k<NCH;k++){
    if (!chEnable[k]) continue;
    uint8_t chc = (uint8_t)(k+1);

    // Electrical START
    seq_ch[k]++;
    Msg stE = makeStart(chc, seq_ch[k], FLG_TGT_ELEC, START_DELAY_US_DEFAULT, 0, 0);
    H[S_ELEC].expSeqCh[k] = seq_ch[k];
    sendToSlave(S_ELEC, stE);

    // Pressure START (uses duration + phase)
    seq_ch[k]++;
    Msg stP = makeStart(chc, seq_ch[k], FLG_TGT_PRES, START_DELAY_US_DEFAULT, chPressPhaseUs[k], chPressDurMs[k]);
    H[S_PRES].expSeqCh[k] = seq_ch[k];
    sendToSlave(S_PRES, stP);

    // Motor START (uses motor phase)
    seq_ch[k]++;
    Msg stM = makeStart(chc, seq_ch[k], FLG_TGT_MOTO, START_DELAY_US_DEFAULT, chMotorPhaseUs[k], 0);
    H[S_MOTO].expSeqCh[k] = seq_ch[k];
    sendToSlave(S_MOTO, stM);

    started++;
    yield(); // let ESP-NOW queue drain after each channel's 3 packets
  }

  lastKeepMs = millis();
  Serial.printf("START sent (%d channels)\n", started);
}

// Recompute unacked bitmaps from H[].ackSeqCh vs expSeqCh. Returns true if any unacked.
static bool recomputeUnackedBitmaps(){
  memset(unackedBitmap, 0, sizeof(unackedBitmap));
  bool any = false;
  for (int s=0; s<3; s++){
    for (int i=0;i<NCH;i++){
      if (!chEnable[i]) continue;
      if (H[s].ackSeqCh[i] < H[s].expSeqCh[i]){
        unackedBitmap[s] |= (1u << i);
        any = true;
      }
    }
  }
  return any;
}

// Re-send SET only for channels that haven't been acked, per slave
static void retransmitUnacked(){
  for (int i=0;i<NCH;i++){
    if (!chEnable[i]) continue;

    // Check if ANY slave is missing ack for this channel
    bool anyMissing = false;
    for (int s=0; s<3; s++){
      if (unackedBitmap[s] & (1u << i)) anyMissing = true;
    }
    if (!anyMissing) continue;

    seq_ch[i]++;
    uint32_t period_us = hzToPeriodUs(chFreqHz[i]);
    uint32_t pulse_us  = chPulseUs[i];
    uint8_t pos        = ampToPosCode(chAmpV[i]);
    Msg mset = makeSet((uint8_t)(i+1), seq_ch[i], period_us, pulse_us, NEUTRAL_CODE, pos, 0x00, true);

    // Send only to slaves that haven't acked
    for (int s=0; s<3; s++){
      if (unackedBitmap[s] & (1u << i)){
        H[s].expSeqCh[i] = seq_ch[i];
        sendToSlave((SlaveIdx)s, mset);
      }
    }
    yield();
  }

  if (debugEnabled) {
    Serial.printf("Retransmit #%d (elec=%d pres=%d moto=%d)\n",
                  retransmitCount,
                  pendingCountForSlave(S_ELEC),
                  pendingCountForSlave(S_PRES),
                  pendingCountForSlave(S_MOTO));
  }
}

// Send keepalive if due
static void sendKeepaliveIfDue(){
  uint32_t now = millis();
  if (now - lastKeepMs >= KEEPALIVE_PERIOD_MS){
    lastKeepMs = now;
    Msg k = makeKeep();
    sendToSlave(S_ELEC, k);
    sendToSlave(S_PRES, k);
    sendToSlave(S_MOTO, k);
  }
}

// ====================== SETUP / LOOP ======================
void setup(){
  Serial.begin(115200);
  delay(150);

  for (int i=0;i<NCH;i++) seq_ch[i]=1;

  loadSettingsFromFlashToRAM();
  initRunIdFromFlash();

  // IMPORTANT: AP-only mode (not AP_STA). STA mode destabilizes the radio on ESP-IDF 5.x
  // and causes the soft-AP to disappear. Slaves will use STA mode to receive/send ESP-NOW.
  WiFi.mode(WIFI_AP);
  WiFi.setSleep(false);
  esp_wifi_set_ps(WIFI_PS_NONE);

  bool apOk = WiFi.softAP(AP_SSID, AP_PASS, WIFI_CH, false, 4);
  Serial.printf("softAP: %s\n", apOk ? "OK" : "FAIL");
  Serial.print("AP IP: "); Serial.println(WiFi.softAPIP());

  // On Core 3.x / ESP-IDF 5.x, softAP() already locks the channel.
  // An extra esp_wifi_set_channel() can fight the AP — removed.
  delay(30);

  server.on("/", HTTP_GET, handleRoot);
  server.on("/apply", HTTP_POST, handleApply);
  server.on("/save", HTTP_POST, handleSave);
  server.on("/load", HTTP_POST, handleLoad);
  server.on("/factory", HTTP_POST, handleFactory);
  server.on("/factory_save", HTTP_POST, handleFactorySave);
  server.on("/favicon.ico", HTTP_GET, handleFavicon);
  server.onNotFound(handleNotFound);
  server.begin();
  Serial.println("Web UI ready: http://192.168.4.1");

  if (esp_now_init() != ESP_OK){
    Serial.println("ESP-NOW init FAIL");
    return;
  }

  // Master can receive ACKs + STATUS
  esp_now_register_recv_cb(onEspNowRecv);

  // Add peers
  addPeer(mac_elec);
  addPeer(mac_press);
  addPeer(mac_motor);

  masterState    = MS_IDLE;
  pendingReq.valid = false;
  lastKeepMs     = millis();
}

void loop(){
  server.handleClient();

  // Boot auto-start: fires at most once, and only if the user hasn't
  // already kicked off (or stopped) a run within the grace window.
  if (bootAutoStart && !bootAutoStarted && millis() > BOOT_GRACE_MS){
    bootAutoStarted = true;
    if (masterState == MS_IDLE && !pendingReq.valid && !running) {
      pendingReq.type  = REQ_APPLY_START;
      pendingReq.valid = true;
    }
  }

  // Electrical liveness watchdog (only meaningful while RUNNING).
  // We require H[S_ELEC].seen so a slow first-status doesn't false-trip.
  if (masterState == MS_RUNNING && running && H[S_ELEC].seen) {
    uint32_t age = millis() - H[S_ELEC].lastSeenMs;
    if (age > ELEC_SILENT_TIMEOUT_MS) {
      Serial.printf("Electrical slave silent (%lu ms) — aborting run\n",
                    (unsigned long)age);
      stopAllNow();
      masterState = MS_IDLE;
    }
  }

  // ====================== MASTER STATE MACHINE ======================
  switch (masterState) {

    case MS_IDLE:
    case MS_RUNNING:
      if (pendingReq.valid) {
        RequestType rt = pendingReq.type;
        pendingReq.valid = false;

        if (rt == REQ_STOP) {
          stopAllNow();
          masterState = MS_IDLE;
        } else {
          commitSnapshot();
          applyIncludedStart = (rt == REQ_APPLY_START);
          if (applyIncludedStart) {
            run_id++;
            saveRunIdToFlash(run_id);
            running = true;
          }
          burstAllSets();
          if (applyIncludedStart) burstAllStarts();
          retransmitCount = 0;
          waitAckDeadline = millis() + RETRANSMIT_DELAY_MS;
          masterState = MS_WAIT_ACKS;
        }
      }
      if (masterState == MS_RUNNING) sendKeepaliveIfDue();
      break;

    case MS_WAIT_ACKS:
      // Allow STOP to interrupt waiting
      if (pendingReq.valid && pendingReq.type == REQ_STOP) {
        pendingReq.valid = false;
        stopAllNow();
        masterState = MS_IDLE;
        break;
      }

      if ((int32_t)(millis() - waitAckDeadline) >= 0) {
        if (!recomputeUnackedBitmaps()) {
          // All acked — done
          masterState = applyIncludedStart ? MS_RUNNING : MS_IDLE;
        } else if (retransmitCount < MAX_RETRANSMITS) {
          retransmitCount++;
          retransmitUnacked();
          waitAckDeadline = millis() + RETRANSMIT_DELAY_MS;
          // stay in MS_WAIT_ACKS
        } else {
          // Max retries exceeded.
          // Why: if Electrical's SET never landed but we transition to
          // RUNNING, START would arm channels with stale/default config
          // (full +amplitude). Pressure/Servo can degrade gracefully, but
          // an Electrical mismatch is unsafe — abort the whole run.
          if (unackedBitmap[S_ELEC] != 0) {
            Serial.println("Max retransmits with Electrical unacked — aborting run");
            stopAllNow();
            masterState = MS_IDLE;
          } else {
            if (debugEnabled)
              Serial.println("Max retransmits reached, proceeding (Electrical OK)");
            masterState = applyIncludedStart ? MS_RUNNING : MS_IDLE;
          }
        }
      }

      // Still send keepalives while waiting if running
      if (running) sendKeepaliveIfDue();
      break;
  }
}
