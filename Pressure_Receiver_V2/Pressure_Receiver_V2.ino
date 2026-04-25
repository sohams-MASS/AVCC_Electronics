// ===========================
//  PRESSURE SLAVE / RECEIVER (ADAPTED: ACK + 2 Hz STATUS + SEQ GUARD)
//  ESP32 Arduino Core 2.0.14 compatible
//
//  Based on YOUR "Pressure Slave V3 Msg" code, adapted to:
//   - Send ACK (0xA1) for SET / START / STOP
//   - Send STATUS (0xA2) at 2 Hz so master UI can show contact / heap / hb age / last seq
//   - Enforce per-channel seq monotonicity (ignore stale packets)
//   - Respect target flag for START (FLG_TGT_PRES = 0x04) when present
//   - Keep V1 backward-compat decode (no start_delay/phase/duration), but still runs
//   - Still fires TWICE per electrical period (one pulse each half-cycle)
//
// ===========================

#include <esp_now.h>
#include <WiFi.h>
#include <esp_wifi.h>
#include <freertos/FreeRTOS.h>
#include <freertos/queue.h>

// ---------- WIFI / ESPNOW ----------
static const uint8_t WIFI_CH = 1;   // MUST match master channel

// ---------- PINS (CH1..CH12) ----------
static const int pressurePins[12] = {
  19, 14, 15, 23, 25, 26, 0, 2, 4, 5, 12, 13
};
static const uint8_t NCH = 12;

// ---------- GUARDRAILS ----------
static const uint32_t MIN_PULSE_US  = 50;
static const uint32_t MIN_PERIOD_US = 100000;    // 10 Hz max (period refers to FULL cycle)
static const uint32_t MAX_PERIOD_US = 30000000;  // 30 s max

// ---------- HEARTBEAT FAILSAFE ----------
static const uint32_t HEARTBEAT_TIMEOUT_MS = 500; // 500ms: ~3 missed keepalives at master's 150ms rate
static volatile uint32_t lastHeartbeatMs = 0;
static volatile uint32_t currentRunId = 0;
static volatile bool runActive = false;

// ---------- STATUS TX (1 Hz) ----------
static const uint32_t STATUS_PERIOD_MS = 1000; // 1 Hz (reduced from 2 Hz to halve uplink traffic)
static uint32_t lastStatusMs = 0;
struct ChanState;
// ---------- PROTOCOL ----------
enum Cmd : uint8_t { CMD_SET=1, CMD_START=2, CMD_STOP=3, CMD_KEEPALIVE=4 };

// Target flags (match master)
static const uint8_t FLG_TGT_PRES = 0x04;

// ---------- MSG (V3, matches updated master) ----------
typedef struct __attribute__((packed)) {
  uint8_t  cmd;
  uint8_t  ch;            // 1..12, or 0 = all
  uint8_t  flags;
  uint8_t  neutral_code;

  uint32_t run_id;
  uint32_t seq;
  uint32_t period_us;     // FULL cycle period (same as electrical)
  uint32_t pulse_us;      // pulse width

  uint8_t  pos_code;
  uint8_t  neg_code;
  uint8_t  rsv0;
  uint8_t  rsv1;

  uint32_t start_delay_us;
  uint32_t phase_offset_us;
  uint32_t duration_ms;   // 0 = forever
} Msg;

// ---------- Back-compat V1 (older size, no V3 fields) ----------
typedef struct __attribute__((packed)) {
  uint8_t  cmd;
  uint8_t  ch;
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
} MsgV1;

// ---------- ACK + STATUS TO MASTER ----------
static const uint8_t ACK_TYPE    = 0xA1;
static const uint8_t STATUS_TYPE = 0xA2;
static const uint8_t ROLE_PRES   = 2;

typedef struct __attribute__((packed)) {
  uint8_t  type;     // 0xA1
  uint8_t  role;     // 2
  uint8_t  cmd;      // CMD_SET/CMD_START/CMD_STOP
  uint8_t  ch;       // 1..12 (0=all/stop)
  uint32_t run_id;
  uint32_t seq;
  uint8_t  ok;
  uint8_t  rsv[3];
} AckMsg;

typedef struct __attribute__((packed)) {
  uint8_t  type;          // 0xA2
  uint8_t  role;          // 2
  uint8_t  rsv0;
  uint8_t  rsv1;

  uint32_t millis_now;    // slave millis()
  uint32_t run_id;
  uint32_t last_seq_max;  // max seq seen by this slave
  uint16_t hb_age_ms;     // keepalive age (ms), 0 if not active
  uint16_t free_heap;     // ESP.getFreeHeap() clipped to 16-bit
} StatusMsg;

static uint8_t lastMasterMac[6] = {0};
static bool masterPeerAdded = false;

// Send-failure backoff: after 3 consecutive send errors, force peer re-registration.
static uint8_t sendFailCount = 0;
static inline void trackSendResult(esp_err_t e){
  if (e == ESP_OK){
    sendFailCount = 0;
  } else {
    if (++sendFailCount >= 3){
      masterPeerAdded = false; // will re-add on next ensureMasterPeer()
      sendFailCount = 0;
    }
  }
}

static inline void ensureMasterPeer(){
  if (masterPeerAdded) return;
  if (memcmp(lastMasterMac, "\0\0\0\0\0\0", 6) == 0) return;

  // Remove stale entry first
  esp_now_del_peer(lastMasterMac);

  esp_now_peer_info_t p{};
  memcpy(p.peer_addr, lastMasterMac, 6);
  p.channel = WIFI_CH;
  p.encrypt = false;
  p.ifidx   = WIFI_IF_STA;

  esp_err_t e = esp_now_add_peer(&p);
  if (e == ESP_OK || e == ESP_ERR_ESPNOW_EXIST) masterPeerAdded = true;
}

static inline void sendAck(uint8_t cmd, uint8_t ch, uint32_t run_id, uint32_t seq){
  ensureMasterPeer();
  if (!masterPeerAdded) return;

  AckMsg a{};
  a.type  = ACK_TYPE;
  a.role  = ROLE_PRES;
  a.cmd   = cmd;
  a.ch    = ch;
  a.run_id = run_id;
  a.seq    = seq;
  a.ok     = 1;

  trackSendResult(esp_now_send(lastMasterMac, (uint8_t*)&a, sizeof(a)));
}

static uint32_t lastSeqMaxSeen = 0;

static inline void sendStatus(){
  ensureMasterPeer();
  if (!masterPeerAdded) return;

  StatusMsg st{};
  st.type = STATUS_TYPE;
  st.role = ROLE_PRES;
  st.millis_now = millis();
  st.run_id = currentRunId;
  st.last_seq_max = lastSeqMaxSeen;

  uint32_t age = 0;
  if (runActive) age = millis() - lastHeartbeatMs;
  if (age > 65535) age = 65535;
  st.hb_age_ms = (uint16_t)age;

  uint32_t fh = ESP.getFreeHeap();
  if (fh > 65535) fh = 65535;
  st.free_heap = (uint16_t)fh;

  trackSendResult(esp_now_send(lastMasterMac, (uint8_t*)&st, sizeof(st)));
}

// ---------- MESSAGE QUEUE (replaces single-message latch) ----------
struct RxPacket {
  uint8_t mac[6];
  Msg     msg;
};
static QueueHandle_t msgQueue = nullptr;
static const int MSG_QUEUE_DEPTH = 16;

static inline uint32_t nowUs(){ return (uint32_t)micros(); }
static inline bool due(uint32_t t, uint32_t deadline){ return (int32_t)(t - deadline) >= 0; }

// ---------- PER-CHANNEL STATE ----------
enum PressChState : uint8_t {
  PCH_IDLE,       // channel disabled or not started
  PCH_PULSE_ON,   // pulse active (output HIGH)
  PCH_PULSE_OFF   // in off-portion of half-cycle (output LOW)
};

struct ChanState {
  PressChState chState = PCH_IDLE;

  uint32_t period_us = 2000000; // FULL period (default 0.5 Hz)
  uint32_t pulse_us  = 2000;

  // scheduling (edge-based)
  uint32_t nextEdge_us = 0;

  // duration
  uint32_t stopAt_ms = 0; // 0 = never
};

static ChanState chs[NCH];
static uint32_t lastSeq[NCH] = {0};

// ---------- LOW-LEVEL IO ----------
static inline void writePressure(uint8_t ch1based, bool on){
  uint8_t idx = ch1based - 1;
  if (idx >= NCH) return;
  digitalWrite(pressurePins[idx], on ? HIGH : LOW);
}

// ---------- SAFETY ----------
static inline void safeStopAll(){
  for (uint8_t i=0;i<NCH;i++){
    chs[i].chState = PCH_IDLE;
    chs[i].stopAt_ms = 0;
    writePressure(i+1, false);
  }
  runActive = false;
}

// ---------- GUARDRAILS (pulse must fit in HALF period) ----------
static inline void clampTiming(ChanState &c){
  if (c.period_us < MIN_PERIOD_US) c.period_us = MIN_PERIOD_US;
  if (c.period_us > MAX_PERIOD_US) c.period_us = MAX_PERIOD_US;

  if (c.pulse_us < MIN_PULSE_US) c.pulse_us = MIN_PULSE_US;

  uint32_t half_us = c.period_us / 2;
  if (half_us == 0) half_us = 1;

  if (c.pulse_us > half_us) c.pulse_us = half_us;
}

// ---------- SEQ GUARD ----------
static inline bool acceptSeq(uint8_t idx0, const Msg &m){
  if (m.seq <= lastSeq[idx0]) return false;
  lastSeq[idx0] = m.seq;
  if (m.seq > lastSeqMaxSeen) lastSeqMaxSeen = m.seq;
  return true;
}

// ---------- APPLY MSG ----------
static inline void applySetToChannel(uint8_t idx0, const Msg &m){
  if (!acceptSeq(idx0, m)) return;

  ChanState &c = chs[idx0];
  if (m.period_us > 0) c.period_us = m.period_us;
  if (m.pulse_us  > 0) c.pulse_us  = m.pulse_us;
  clampTiming(c);

  sendAck(CMD_SET, (uint8_t)(idx0+1), m.run_id, m.seq);
}

static inline void applyStartToChannel(uint8_t idx0, const Msg &m){
  if (!acceptSeq(idx0, m)) return;

  ChanState &c = chs[idx0];
  clampTiming(c);

  c.chState = PCH_PULSE_OFF;

  // Sync start: start_delay + phase
  uint32_t t = nowUs();
  c.nextEdge_us = t + m.start_delay_us + m.phase_offset_us;

  // Duration
  if (m.duration_ms > 0) c.stopAt_ms = millis() + m.duration_ms;
  else                  c.stopAt_ms = 0;

  // Ensure output LOW until first edge
  writePressure(idx0 + 1, false);

  sendAck(CMD_START, (uint8_t)(idx0+1), m.run_id, m.seq);
}

static inline void dispatchMsg(const Msg &m){
  if (m.cmd == CMD_KEEPALIVE){
    if (runActive && (m.run_id == currentRunId)) lastHeartbeatMs = millis();
    return;
  }

  if (m.cmd == CMD_STOP){
    safeStopAll();
    // STOP is not per-channel; still ack once so master UI sees it.
    sendAck(CMD_STOP, 0, m.run_id, m.seq);
    return;
  }

  if (m.cmd == CMD_START){
    if (m.flags != 0 && ((m.flags & FLG_TGT_PRES) == 0)) return;

    // New run_id means master (re)started. Reset seq guards so fresh
    // sequence numbers are accepted and not stale-rejected.
    if (m.run_id != currentRunId) {
      memset(lastSeq, 0, sizeof(lastSeq));
      lastSeqMaxSeen = 0;
    }
    currentRunId = m.run_id;
    runActive = true;
    lastHeartbeatMs = millis();
  }

  if (m.cmd == CMD_SET || m.cmd == CMD_START){
    if (m.ch == 0){
      for (uint8_t i=0;i<NCH;i++){
        if (m.cmd == CMD_SET)   applySetToChannel(i, m);
        if (m.cmd == CMD_START) applyStartToChannel(i, m);
      }
    } else if (m.ch >= 1 && m.ch <= NCH){
      uint8_t idx0 = m.ch - 1;
      if (m.cmd == CMD_SET)   applySetToChannel(idx0, m);
      if (m.cmd == CMD_START) applyStartToChannel(idx0, m);
    }
  }
}

// ---------- ESPNOW RECV CALLBACK ----------
void OnDataRecv(const esp_now_recv_info_t *info, const uint8_t *incomingData, int len){
  Msg m{};
  if (len == (int)sizeof(MsgV1)){
    MsgV1 v1;
    memcpy(&v1, incomingData, sizeof(MsgV1));
    memcpy(&m, &v1, sizeof(MsgV1));
    m.start_delay_us  = 0;
    m.phase_offset_us = 0;
    m.duration_ms     = 0;
  } else if (len == (int)sizeof(Msg)){
    memcpy(&m, incomingData, sizeof(Msg));
  } else {
    return;
  }

  RxPacket pkt;
  memcpy(pkt.mac, info->src_addr, 6);
  pkt.msg = m;
  xQueueSend(msgQueue, &pkt, 0); // non-blocking; drop if full
}

// ---------- STRICT PRIORITY EVENT LOOP ----------
static inline int pickNextDue(uint32_t t){
  int best = -1;
  uint32_t bestDeadline = 0;

  for (int i=0;i<NCH;i++){
    ChanState &c = chs[i];
    if (c.chState == PCH_IDLE) continue;
    if (!due(t, c.nextEdge_us)) continue;

    if (best < 0){ best = i; bestDeadline = c.nextEdge_us; }
    else{
      if ((int32_t)(c.nextEdge_us - bestDeadline) < 0 ||
          (c.nextEdge_us == bestDeadline && i < best)){
        best = i;
        bestDeadline = c.nextEdge_us;
      }
    }
  }
  return best;
}

static inline void advanceOneEvent(uint8_t idx0){
  ChanState &c = chs[idx0];
  // clampTiming already enforced in applySetToChannel / applyStartToChannel

  uint32_t half_us = c.period_us / 2;
  if (half_us == 0) half_us = 1;

  switch (c.chState) {
    case PCH_PULSE_OFF:
      // Rising edge: start pulse (every half-cycle)
      writePressure(idx0 + 1, true);
      c.chState = PCH_PULSE_ON;
      c.nextEdge_us += c.pulse_us;
      break;
    case PCH_PULSE_ON:
      // Falling edge: end pulse, schedule next half-cycle boundary
      writePressure(idx0 + 1, false);
      c.chState = PCH_PULSE_OFF;
      c.nextEdge_us += (half_us > c.pulse_us) ? (half_us - c.pulse_us) : 0;
      break;
    case PCH_IDLE:
      break; // should not happen (pickNextDue filters idle)
  }
}

void setup(){
  Serial.begin(115200);

  // GPIO init
  for (int i=0;i<NCH;i++){
    pinMode(pressurePins[i], OUTPUT);
    digitalWrite(pressurePins[i], LOW);
  }

  // WiFi + channel lock
  WiFi.mode(WIFI_STA);
  WiFi.setSleep(false);
  esp_wifi_set_channel(WIFI_CH, WIFI_SECOND_CHAN_NONE);

  Serial.print("Pressure Slave MAC: ");
  Serial.println(WiFi.macAddress());
  Serial.printf("Pressure WiFi CH forced to %u\n", WIFI_CH);

  // Defaults
  for (uint8_t i=0;i<NCH;i++){
    chs[i].chState = PCH_IDLE;
    chs[i].period_us = 2000000;
    chs[i].pulse_us  = 2000;
    chs[i].nextEdge_us = 0;
    chs[i].stopAt_ms = 0;
    clampTiming(chs[i]);
    lastSeq[i] = 0;
  }
  lastSeqMaxSeen = 0;

  // Create message queue before ESP-NOW init
  msgQueue = xQueueCreate(MSG_QUEUE_DEPTH, sizeof(RxPacket));
  if (!msgQueue){
    Serial.println("FATAL: queue creation failed");
    while (true) delay(1000);
  }

  if (esp_now_init() != ESP_OK){
    Serial.println("ESP-NOW init FAIL");
    return;
  }
  esp_now_register_recv_cb(OnDataRecv);

  lastHeartbeatMs = millis();
  runActive = false;
  lastStatusMs = millis();

  Serial.println("Pressure slave READY (ACK + 1 Hz STATUS, 500ms HB timeout, 2 pulses per period).");
}

void loop(){
  // Drain all queued messages (processes entire burst, not just one)
  RxPacket pkt;
  while (xQueueReceive(msgQueue, &pkt, 0) == pdTRUE) {
    memcpy(lastMasterMac, pkt.mac, 6);
    dispatchMsg(pkt.msg);
    // Throttle to ~1 ms per packet so the ESP-NOW send queue (10 deep)
    // can drain between back-to-back acks during the Apply+Start burst.
    vTaskDelay(1);
  }

  // Duration auto-stop per channel
  uint32_t nowms = millis();
  for (uint8_t i=0;i<NCH;i++){
    if (chs[i].chState == PCH_IDLE) continue;
    if (chs[i].stopAt_ms != 0 && (int32_t)(nowms - chs[i].stopAt_ms) >= 0){
      chs[i].chState = PCH_IDLE;
      chs[i].stopAt_ms = 0;
      writePressure(i+1, false);
    }
  }

  // Heartbeat failsafe
  if (runActive){
    uint32_t age = millis() - lastHeartbeatMs;
    if (age > HEARTBEAT_TIMEOUT_MS){
      Serial.println("Heartbeat timeout -> safe stop");
      safeStopAll();
    }
  }

  // 1 Hz status ping to master (for UI contact)
  if ((uint32_t)(millis() - lastStatusMs) >= STATUS_PERIOD_MS){
    lastStatusMs = millis();
    sendStatus();
  }

  // Generate pulses (strict priority)
  const int MAX_EVENTS_PER_LOOP = 400;
  int events = 0;

  while (events < MAX_EVENTS_PER_LOOP){
    uint32_t t = nowUs();
    int idx = pickNextDue(t);
    if (idx < 0) break;
    advanceOneEvent((uint8_t)idx);
    events++;
  }
}
