// ===========================
//  SERVO SLAVE / RECEIVER (V3 + ACK + 2 Hz STATUS + 2 Hz HARD CAP)
//  ESP32 Arduino Core 2.0.14 compatible
//
//  REQUIREMENTS (per your request):
//   - KEEP SERVO WAVEFORM UNCHANGED (DEG_MIN/DEG_MAX, UP/DOWN step timing)
//   - HARD LIMIT: if channel freq > 2 Hz (period_us < 500000), DO NOT enable servo on that channel
//   - V3 Msg compatible (start_delay_us + phase_offset_us + duration_ms)
//   - Non-blocking scheduler (cycle boundaries + step scheduler)
//   - Heartbeat failsafe (all channels to DEG_MIN)
//   - Adds:
//        * ACK (0xA1) back to master for SET/START/STOP
//        * STATUS (0xA2) @ 2 Hz so master UI can show "in contact", hb age, heap, last seq
//   - Includes per-channel SEQ guard (ignore stale packets)
//
//  NOTE: Master must add/parse STATUS_TYPE=0xA2 if you want UI updates.
// ===========================

#include <esp_now.h>
#include <WiFi.h>
#include <esp_wifi.h>
#include <Servo.h>
#include <freertos/FreeRTOS.h>
#include <freertos/queue.h>

// ---------- WIFI / ESPNOW ----------
static const uint8_t WIFI_CH = 1;

// ---------- SERVO PINS (CH1..CH12) ----------
static const int servoPins[12] = {
  19, 14, 15, 23, 25, 26, 0, 2, 4, 5, 12, 13
};
static const uint8_t NCH = 12;

// ---------- HEARTBEAT ----------
static const uint32_t HEARTBEAT_TIMEOUT_MS = 500; // 500ms: ~3 missed keepalives at master's 150ms rate
static volatile uint32_t lastHeartbeatMs = 0;
static volatile uint32_t currentRunId = 0;
static volatile bool runActive = false;

// ---------- STATUS TX (1 Hz) ----------
static const uint32_t STATUS_PERIOD_MS = 1000; // 1 Hz (reduced from 2 Hz to halve uplink traffic)
static uint32_t lastStatusMs = 0;

// ---------- PROTOCOL ----------
enum Cmd : uint8_t { CMD_SET=1, CMD_START=2, CMD_STOP=3, CMD_KEEPALIVE=4 };
struct ChanState;
// Target flag (match master)
static const uint8_t FLG_TGT_MOTO = 0x08; // your master uses 0x08 for motor/servo start

// V3 message
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

  uint32_t start_delay_us;
  uint32_t phase_offset_us;
  uint32_t duration_ms;
} Msg;

// Back-compat (older size)
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

// ---------- MESSAGE QUEUE (replaces single-message latch) ----------
struct RxPacket {
  uint8_t mac[6];
  Msg     msg;
};
static QueueHandle_t msgQueue = nullptr;
static const int MSG_QUEUE_DEPTH = 16;

// ---------- TIME ----------
static inline uint32_t nowUs(){ return (uint32_t)micros(); }
static inline bool due(uint32_t t, uint32_t deadline){ return (int32_t)(t - deadline) >= 0; }

// ---------- SERVO WAVEFORM (UNCHANGED) ----------
static const int DEG_MIN = 30;
static const int DEG_MAX = 60;
static const uint32_t UP_STEP_US   = 11000;
static const uint32_t DOWN_STEP_US = 3667;

// ---------- HARD LIMIT ----------
static const uint32_t SERVO_MIN_PERIOD_US = 500000; // 2 Hz max (period >= 500ms)

// ---------- ACK + STATUS TO MASTER ----------
static const uint8_t ACK_TYPE    = 0xA1;
static const uint8_t STATUS_TYPE = 0xA2;
static const uint8_t ROLE_SERVO  = 3; // treat as role 3 (motor/servo) for UI consistency

typedef struct __attribute__((packed)) {
  uint8_t  type;
  uint8_t  role;
  uint8_t  cmd;
  uint8_t  ch;
  uint32_t run_id;
  uint32_t seq;
  uint8_t  ok;
  uint8_t  rsv[3];
} AckMsg;

typedef struct __attribute__((packed)) {
  uint8_t  type;          // 0xA2
  uint8_t  role;          // 3
  uint8_t  rsv0;
  uint8_t  rsv1;

  uint32_t millis_now;
  uint32_t run_id;
  uint32_t last_seq_max;
  uint16_t hb_age_ms;     // capped at 65535ms in master UI unless you upgrade to uint32_t
  uint16_t free_heap;     // capped at 65535
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

static inline void sendAck(uint8_t cmd, uint8_t ch, uint32_t run_id, uint32_t seq, uint8_t ok = 1){
  if (!masterPeerAdded) return;

  AckMsg a{};
  a.type = ACK_TYPE;
  a.role = ROLE_SERVO;
  a.cmd  = cmd;
  a.ch   = ch;
  a.run_id = run_id;
  a.seq    = seq;
  a.ok     = ok;
  trackSendResult(esp_now_send(lastMasterMac, (uint8_t*)&a, sizeof(a)));
}

static uint32_t lastSeqMaxSeen = 0;

static inline void sendStatus(){
  ensureMasterPeer();
  if (!masterPeerAdded) return;

  StatusMsg st{};
  st.type = STATUS_TYPE;
  st.role = ROLE_SERVO;
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

// ---------- PER-CHANNEL STATE ----------
enum MotionStage : uint8_t { ST_IDLE=0, ST_UP=1, ST_DOWN=2 };

struct ChanState {
  bool enabled = false;

  uint32_t period_us = 2000000;

  // sync
  uint32_t nextCycle_us = 0;
  uint32_t stopAt_ms = 0;

  // motion
  MotionStage st = ST_IDLE;
  int posDeg = DEG_MIN;
  uint32_t nextStep_us = 0;
};

static ChanState chs[NCH];
static uint32_t lastSeqCh[NCH] = {0};
static Servo sv[NCH];

// ---------- SAFETY ----------
static inline void safeStopAll(){
  for (uint8_t i=0;i<NCH;i++){
    chs[i].enabled = false;
    chs[i].st = ST_IDLE;
    chs[i].nextStep_us = 0;
    chs[i].stopAt_ms = 0;
    sv[i].write(DEG_MIN);
  }
  runActive = false;
}

// ---------- SEQ GUARD ----------
static inline bool acceptSeq(uint8_t idx0, const Msg &m){
  if (m.seq <= lastSeqCh[idx0]) return false;
  lastSeqCh[idx0] = m.seq;
  if (m.seq > lastSeqMaxSeen) lastSeqMaxSeen = m.seq;
  return true;
}

// ---------- APPLY MSG ----------
static inline void applySetToChannel(uint8_t idx0, const Msg &m){
  if (!acceptSeq(idx0, m)) return;
  if (m.period_us > 0) chs[idx0].period_us = m.period_us;

  // Optional: send ACK for set
  sendAck(CMD_SET, (uint8_t)(idx0+1), m.run_id, m.seq);
}

static inline void disableChannelHardCap(uint8_t idx0){
  ChanState &c = chs[idx0];
  c.enabled = false;
  c.st = ST_IDLE;
  c.nextStep_us = 0;
  c.stopAt_ms = 0;
  sv[idx0].write(DEG_MIN);
}

static inline void applyStartToChannel(uint8_t idx0, const Msg &m){
  if (!acceptSeq(idx0, m)) return;

  ChanState &c = chs[idx0];

  // HARD CAP: reject >2 Hz (period < 500ms)
  if (c.period_us < SERVO_MIN_PERIOD_US){
    disableChannelHardCap(idx0);
    Serial.printf("SERVO ch=%u DISABLED (freq > 2 Hz, period=%lu us)\n",
                  (unsigned)(idx0+1), (unsigned long)c.period_us);
    ensureMasterPeer();
    sendAck(CMD_START, (uint8_t)(idx0+1), m.run_id, m.seq, 0); // ok=0: rejected
    return;
  }

  c.enabled = true;
  c.st = ST_IDLE;
  c.nextStep_us = 0;

  uint32_t t = nowUs();
  c.nextCycle_us = t + m.start_delay_us + m.phase_offset_us;

  if (m.duration_ms > 0) c.stopAt_ms = millis() + m.duration_ms;
  else                  c.stopAt_ms = 0;

  sv[idx0].write(DEG_MIN);

  sendAck(CMD_START, (uint8_t)(idx0+1), m.run_id, m.seq);
}

static inline void dispatchMsg(const Msg &m){
  if (m.cmd == CMD_KEEPALIVE){
    if (runActive && m.run_id == currentRunId) lastHeartbeatMs = millis();
    return;
  }

  if (m.cmd == CMD_STOP){
    safeStopAll();
    sendAck(CMD_STOP, 0, m.run_id, m.seq);
    return;
  }

  if (m.cmd == CMD_START){
    if (m.flags != 0 && ((m.flags & FLG_TGT_MOTO) == 0)) return;

    // New run_id means master (re)started. Reset seq guards so fresh
    // sequence numbers are accepted and not stale-rejected.
    if (m.run_id != currentRunId) {
      memset(lastSeqCh, 0, sizeof(lastSeqCh));
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
      uint8_t i = m.ch - 1;
      if (m.cmd == CMD_SET)   applySetToChannel(i, m);
      if (m.cmd == CMD_START) applyStartToChannel(i, m);
    }
  }
}

// ---------- ESPNOW CALLBACK ----------
void OnDataRecv(const esp_now_recv_info_t *info, const uint8_t *data, int len){
  Msg m{};
  if (len == (int)sizeof(MsgV1)){
    MsgV1 v1; memcpy(&v1, data, sizeof(v1));
    memcpy(&m, &v1, sizeof(v1));
    m.start_delay_us  = 0;
    m.phase_offset_us = 0;
    m.duration_ms     = 0;
  } else if (len == (int)sizeof(Msg)){
    memcpy(&m, data, sizeof(Msg));
  } else {
    return;
  }

  RxPacket pkt;
  memcpy(pkt.mac, info->src_addr, 6);
  pkt.msg = m;
  xQueueSend(msgQueue, &pkt, 0); // non-blocking; drop if full
}

// ---------- MOTION (UNCHANGED) ----------
static inline void startMotion(ChanState &c, uint32_t t){
  c.st = ST_UP;
  c.posDeg = DEG_MIN;
  c.nextStep_us = t;
}

static inline void stepMotion(uint8_t i, uint32_t t){
  ChanState &c = chs[i];
  if (!due(t, c.nextStep_us)) return;

  if (c.st == ST_UP){
    sv[i].write(c.posDeg);
    if (c.posDeg >= DEG_MAX){
      c.st = ST_DOWN;
      c.nextStep_us += DOWN_STEP_US;
    } else {
      c.posDeg++;
      c.nextStep_us += UP_STEP_US;
    }
  } else if (c.st == ST_DOWN){
    sv[i].write(c.posDeg);
    if (c.posDeg <= DEG_MIN){
      c.st = ST_IDLE;
      c.nextStep_us = 0;
    } else {
      c.posDeg--;
      c.nextStep_us += DOWN_STEP_US;
    }
  }
}

static inline int pickNextDue(uint32_t t){
  for (int i=0;i<NCH;i++){
    if (chs[i].enabled && chs[i].st != ST_IDLE && due(t, chs[i].nextStep_us))
      return i;
  }
  return -1;
}

// ---------- SETUP ----------
void setup(){
  Serial.begin(115200);

  for (int i=0;i<NCH;i++){
    sv[i].attach(servoPins[i]);
    sv[i].write(DEG_MIN);
    chs[i].enabled = false;
    chs[i].period_us = 2000000;
    chs[i].st = ST_IDLE;
    chs[i].nextStep_us = 0;
    chs[i].stopAt_ms = 0;
    lastSeqCh[i] = 0;
  }
  lastSeqMaxSeen = 0;

  WiFi.mode(WIFI_STA);
  WiFi.setSleep(false);
  esp_wifi_set_channel(WIFI_CH, WIFI_SECOND_CHAN_NONE);

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

  Serial.println("Servo slave READY (waveform unchanged, 2 Hz hard cap, ACK + 1 Hz STATUS, 500ms HB timeout).");
}

// ---------- LOOP ----------
void loop(){
  // Drain all queued messages (processes entire burst, not just one)
  RxPacket pkt;
  while (xQueueReceive(msgQueue, &pkt, 0) == pdTRUE) {
    memcpy(lastMasterMac, pkt.mac, 6);
    dispatchMsg(pkt.msg);
  }

  // duration handling
  uint32_t nowms = millis();
  for (uint8_t i=0;i<NCH;i++){
    if (chs[i].enabled && chs[i].stopAt_ms && (int32_t)(nowms - chs[i].stopAt_ms) >= 0){
      disableChannelHardCap(i);
    }
  }

  // heartbeat failsafe
  if (runActive && (millis() - lastHeartbeatMs > HEARTBEAT_TIMEOUT_MS)){
    Serial.println("Heartbeat timeout -> servo safe stop");
    safeStopAll();
  }

  // 1 Hz status ping
  if ((uint32_t)(millis() - lastStatusMs) >= STATUS_PERIOD_MS){
    lastStatusMs = millis();
    sendStatus();
  }

  uint32_t t = nowUs();

  // cycle boundaries (one motion per electrical cycle)
  for (uint8_t i=0;i<NCH;i++){
    ChanState &c = chs[i];
    if (!c.enabled) continue;

    if (due(t, c.nextCycle_us)){
      while (due(t, c.nextCycle_us)) c.nextCycle_us += c.period_us;
      if (c.st == ST_IDLE) startMotion(c, c.nextCycle_us - c.period_us);
    }
  }

  const int MAX_EVENTS = 120;
  int ev = 0;
  while (ev < MAX_EVENTS){
    uint32_t nowt = nowUs();
    int idx = pickNextDue(nowt);
    if (idx < 0) break;
    stepMotion((uint8_t)idx, nowt);
    ev++;
  }
}
