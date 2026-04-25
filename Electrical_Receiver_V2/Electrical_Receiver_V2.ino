// ===========================
//  ELECTRICAL SLAVE / RECEIVER  (ACK + 1 Hz STATUS)
//  ESP32 Arduino Core 2.0.14 / ESP-IDF 5.x
//
//  Sub-millisecond pulse architecture
//  ───────────────────────────────────
//  Core 0: WiFi / ESP-NOW system tasks (priority ~23) — untouched
//  Core 1: waveformTask (priority 15) + Arduino loop() (priority 1)
//
//  waveformTask owns all GPIO writes and the pulse scheduler.
//  It spin-waits when the next edge is < 500 µs away so the timing
//  loop is never interrupted by ESP-NOW I/O.  When idle (next edge
//  > 2 ms away) it yields, giving loop() time to handle messages,
//  send ACKs, and refresh the 1 Hz status heartbeat.
//
//  loop() owns all ESP-NOW calls (send/receive) and config parsing.
//  It pushes updates into updateQueue (FIFO, non-blocking) for the
//  waveform task, and sets a single volatile flag for STOP requests.
// ===========================

#include <esp_now.h>
#include <WiFi.h>
#include <esp_wifi.h>
#include <soc/gpio_struct.h>
#include <freertos/FreeRTOS.h>
#include <freertos/task.h>
#include <freertos/queue.h>

// Forward declarations so Arduino's auto-generated function prototypes
// (inserted near the top of the .ino) compile. The full definitions
// appear further down; passing by reference only needs the name in scope.
struct ChanState;
struct ChanUpdate;

// ---------- WIFI / ESPNOW ----------
static const uint8_t WIFI_CH = 1;

// ---------- PINS ----------
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

// ---------- DEFAULT CODES ----------
static const uint8_t DEF_POS = 0xFF;
static const uint8_t DEF_NEU = 0x80;
static const uint8_t DEF_NEG = 0x00;

// ---------- GUARDRAILS ----------
static const uint32_t MIN_PULSE_US  = 50;
static const uint32_t MIN_PERIOD_US = 100000;
static const uint32_t MAX_PERIOD_US = 30000000;

// ---------- HEARTBEAT FAILSAFE ----------
static const uint32_t HEARTBEAT_TIMEOUT_MS = 500;
static uint32_t lastHeartbeatMs = 0;   // written+read by loop() only
static volatile uint32_t currentRunId = 0;
static volatile bool     runActive    = false;

// ---------- STOP REQUEST  (loop → waveformTask) ----------
// Set by loop() on CMD_STOP or heartbeat timeout.
// Cleared by waveformTask after safe-stopping all channels.
static volatile bool stopRequested = false;

// ---------- PROTOCOL ----------
enum Cmd : uint8_t { CMD_SET=1, CMD_START=2, CMD_STOP=3, CMD_KEEPALIVE=4 };
static const uint8_t FLG_MIRROR   = 0x01;
static const uint8_t FLG_TGT_ELEC = 0x02;

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
  uint32_t duration_ms;   // ignored by electrical
} Msg;

// ---------- ACK TO MASTER ----------
static const uint8_t ACK_TYPE  = 0xA1;
static const uint8_t ROLE_ELEC = 1;

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

// ---------- STATUS TO MASTER ----------
static const uint8_t  STATUS_TYPE      = 0xA2;
static const uint32_t STATUS_PERIOD_MS = 1000;  // 1 Hz
static uint32_t lastStatusMs = 0;               // loop() only

typedef struct __attribute__((packed)) {
  uint8_t  type;
  uint8_t  role;
  uint8_t  rsv0;
  uint8_t  rsv1;
  uint32_t millis_now;
  uint32_t run_id;
  uint32_t last_seq_max;
  uint16_t hb_age_ms;
  uint16_t free_heap;
} StatusMsg;

// ---------- MASTER PEER TRACKING  (loop() only) ----------
static uint8_t lastMasterMac[6] = {0};
static bool    masterPeerAdded  = false;
static uint8_t sendFailCount    = 0;

static inline void trackSendResult(esp_err_t e) {
  if (e == ESP_OK) {
    sendFailCount = 0;
  } else {
    if (++sendFailCount >= 3) {
      masterPeerAdded = false;
      sendFailCount   = 0;
    }
  }
}

static inline void ensureMasterPeer() {
  if (masterPeerAdded) return;
  if (memcmp(lastMasterMac, "\0\0\0\0\0\0", 6) == 0) return;
  esp_now_del_peer(lastMasterMac);
  esp_now_peer_info_t p{};
  memcpy(p.peer_addr, lastMasterMac, 6);
  p.channel = WIFI_CH;
  p.encrypt = false;
  p.ifidx   = WIFI_IF_STA;
  esp_err_t e = esp_now_add_peer(&p);
  if (e == ESP_OK || e == ESP_ERR_ESPNOW_EXIST) masterPeerAdded = true;
}

// sendAck() is ONLY called from loop() — no esp_now_send in waveformTask.
static inline void sendAck(uint8_t cmd, uint8_t ch,
                            uint32_t run_id, uint32_t seq) {
  ensureMasterPeer();
  if (!masterPeerAdded) return;
  AckMsg a{};
  a.type   = ACK_TYPE; a.role = ROLE_ELEC;
  a.cmd    = cmd;      a.ch   = ch;
  a.run_id = run_id;   a.seq  = seq;
  a.ok     = 1;
  trackSendResult(esp_now_send(lastMasterMac, (uint8_t*)&a, sizeof(a)));
}

// ============================================================
//  INTER-TASK QUEUES
// ============================================================
//
//  msgQueue    : ESP-NOW callback → loop()
//                Carries RxPacket (MAC + Msg).  Depth 16.
//
//  updateQueue : loop() → waveformTask
//                Carries ChanUpdate (channel config/start).
//                Depth 36 (headroom for 12×SET + 12×START + 12 spare).
// ============================================================

// Wrapper so the MAC address travels through the queue safely
struct RxPacket {
  uint8_t mac[6];
  Msg     msg;
};

// Channel update sent from loop() to waveformTask
struct ChanUpdate {
  enum Type : uint8_t { CONFIG = 1, START = 2 } type;
  uint8_t  ch;               // 0-based channel index
  // CONFIG fields (always fully populated from commCfg shadow):
  uint32_t period_us;
  uint32_t pulse_us;
  uint8_t  neutral_code;
  uint8_t  pos_code;
  uint8_t  neg_code;
  bool     mirrorAroundNeutral;
  // START field:
  uint32_t nextEdge_us;      // pre-computed by loop() at dispatch time
};

static QueueHandle_t msgQueue    = nullptr;  // RxPacket
static QueueHandle_t updateQueue = nullptr;  // ChanUpdate

static const int MSG_QUEUE_DEPTH    = 16;
static const int UPDATE_QUEUE_DEPTH = NCH * 3;  // 36

// ============================================================
//  PER-CHANNEL STATE  — owned exclusively by waveformTask
//  (after setup() finishes and the task is created)
// ============================================================
enum ElecChState : uint8_t {
  ECH_IDLE,       // channel disabled
  ECH_POS_WAIT,   // waiting for positive pulse edge
  ECH_POS_PULSE,  // positive pulse active (DAC = pos_code)
  ECH_NEG_WAIT,   // waiting for negative pulse edge
  ECH_NEG_PULSE   // negative pulse active (DAC = neg_code)
};

struct ChanState {
  ElecChState chState = ECH_IDLE;
  uint32_t period_us = 2000000;
  uint32_t pulse_us  = 2000;
  uint8_t  neutral_code = DEF_NEU;
  uint8_t  pos_code     = DEF_POS;
  uint8_t  neg_code     = DEF_NEG;
  bool     mirrorAroundNeutral = false;
  uint32_t nextEdge_us = 0;
};

static ChanState chs[NCH];   // waveformTask eyes only

// ============================================================
//  CONFIG SHADOW  — owned exclusively by loop() / comm task
//  Tracks the last-applied configuration per channel so that a
//  START update can carry a full config snapshot without reading
//  chs[] (which belongs to waveformTask).
// ============================================================
struct ChanConfig {
  uint32_t period_us    = 2000000;
  uint32_t pulse_us     = 2000;
  uint8_t  neutral_code = DEF_NEU;
  uint8_t  pos_code     = DEF_POS;
  uint8_t  neg_code     = DEF_NEG;
  bool     mirrorAroundNeutral = false;
};

static ChanConfig commCfg[NCH];   // loop() only

// Sequence guards and high-water mark — loop() only
static uint32_t         lastSeq[NCH] = {0};
static volatile uint32_t lastSeqMax  = 0;   // read by loop() in sendStatus

// ============================================================
//  LOW-LEVEL HELPERS
// ============================================================
static inline uint32_t nowUs()       { return (uint32_t)micros(); }
static inline uint8_t  clamp8(int v) {
  if (v < 0) return 0; if (v > 255) return 255; return (uint8_t)v;
}

// Direct GPIO register writes: 2 writes replace 8 digitalWrite() calls.
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
  GPIO.out_w1tc = (1UL << wrPin);  // WR LOW
  delayMicroseconds(2);            // DAC latch hold time — do not reduce
  GPIO.out_w1ts = (1UL << wrPin);  // WR HIGH
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
  delayMicroseconds(1);     // data setup time — do not reduce

  setDacSelect(dacSel);
  delayMicroseconds(1);     // select setup time — do not reduce

  strobeWR(wrPin);
}

// ---------- GUARDRAILS ----------
static inline void clampTiming(uint32_t &period_us, uint32_t &pulse_us) {
  if (period_us < MIN_PERIOD_US) period_us = MIN_PERIOD_US;
  if (period_us > MAX_PERIOD_US) period_us = MAX_PERIOD_US;
  if (pulse_us  < MIN_PULSE_US)  pulse_us  = MIN_PULSE_US;
  uint32_t half = period_us / 2;
  if (half == 0) half = 1;
  if (pulse_us > half) pulse_us = half;
}

// Overload for ChanState
static inline void clampTiming(ChanState &c) {
  clampTiming(c.period_us, c.pulse_us);
}

// ============================================================
//  SAFE STOP  —  called ONLY from waveformTask
// ============================================================
static inline void safeStopAll() {
  for (uint8_t i = 0; i < NCH; i++) {
    chs[i].chState = ECH_IDLE;
    writeChannelCode(i + 1, chs[i].neutral_code);
  }
  runActive = false;
}

// ============================================================
//  SEQUENCE GUARD  —  loop() only
// ============================================================
static inline bool acceptSeq(uint8_t idx0, const Msg &m) {
  if (m.seq <= lastSeq[idx0]) return false;
  lastSeq[idx0] = m.seq;
  if (m.seq > lastSeqMax) lastSeqMax = m.seq;
  return true;
}

// ============================================================
//  MESSAGE DISPATCH  —  runs in loop() only
//  All esp_now_send() calls happen here; waveformTask never calls
//  esp_now_send(), keeping its timing loop free of WiFi stack delays.
// ============================================================
static void dispatchMsg(const Msg &m) {

  // ── KEEPALIVE ──────────────────────────────────────────────
  if (m.cmd == CMD_KEEPALIVE) {
    if (runActive && (m.run_id == currentRunId))
      lastHeartbeatMs = millis();
    return;
  }

  // ── STOP ───────────────────────────────────────────────────
  if (m.cmd == CMD_STOP) {
    stopRequested = true;   // waveformTask will call safeStopAll()
    runActive = false;      // suppress repeat timeout triggers in loop()
    sendAck(CMD_STOP, 0, m.run_id, m.seq);
    return;
  }

  // ── START / SET ────────────────────────────────────────────
  if (m.cmd == CMD_START) {
    if ((m.flags & FLG_TGT_ELEC) == 0) return;   // not for us

    if (m.run_id != currentRunId) {
      memset(lastSeq, 0, sizeof(lastSeq));
      lastSeqMax   = 0;
    }
    currentRunId    = m.run_id;
    runActive       = true;
    lastHeartbeatMs = millis();
  }

  if (m.cmd != CMD_SET && m.cmd != CMD_START) return;

  uint8_t lo = (m.ch == 0) ? 0       : (uint8_t)(m.ch - 1);
  uint8_t hi = (m.ch == 0) ? (uint8_t)NCH : m.ch;

  for (uint8_t i = lo; i < hi; i++) {
    if (!acceptSeq(i, m)) continue;

    ChanConfig &cfg = commCfg[i];

    if (m.cmd == CMD_SET) {
      // Update config shadow
      if (m.period_us > 0) cfg.period_us = m.period_us;
      if (m.pulse_us  > 0) cfg.pulse_us  = m.pulse_us;
      cfg.neutral_code       = m.neutral_code;
      cfg.pos_code           = m.pos_code;
      cfg.mirrorAroundNeutral = (m.flags & FLG_MIRROR) != 0;
      cfg.neg_code = cfg.mirrorAroundNeutral
                     ? clamp8(2 * (int)m.neutral_code - (int)m.pos_code)
                     : m.neg_code;
      clampTiming(cfg.period_us, cfg.pulse_us);

      // Push full config snapshot to waveformTask
      ChanUpdate upd{};
      upd.type             = ChanUpdate::CONFIG;
      upd.ch               = i;
      upd.period_us        = cfg.period_us;
      upd.pulse_us         = cfg.pulse_us;
      upd.neutral_code     = cfg.neutral_code;
      upd.pos_code         = cfg.pos_code;
      upd.neg_code         = cfg.neg_code;
      upd.mirrorAroundNeutral = cfg.mirrorAroundNeutral;
      if (xQueueSend(updateQueue, &upd, 0) != pdTRUE) {
        Serial.println("WARN: updateQueue full (CONFIG dropped)");
      }
      sendAck(CMD_SET, (uint8_t)(i + 1), m.run_id, m.seq);

    } else {  // CMD_START
      // nextEdge_us is computed here (comm-task time) so the waveform task
      // applies it without any additional time-of-flight error.
      ChanUpdate upd{};
      upd.type         = ChanUpdate::START;
      upd.ch           = i;
      upd.period_us    = cfg.period_us;   // pass config for safety
      upd.pulse_us     = cfg.pulse_us;
      upd.neutral_code = cfg.neutral_code;
      upd.pos_code     = cfg.pos_code;
      upd.neg_code     = cfg.neg_code;
      upd.mirrorAroundNeutral = cfg.mirrorAroundNeutral;
      upd.nextEdge_us  = nowUs() + m.start_delay_us + m.phase_offset_us;
      if (xQueueSend(updateQueue, &upd, 0) != pdTRUE) {
        Serial.println("WARN: updateQueue full (START dropped)");
      }
      sendAck(CMD_START, (uint8_t)(i + 1), m.run_id, m.seq);
    }
  }
}

// ============================================================
//  STATUS SEND  —  loop() only
// ============================================================
static inline void sendStatus() {
  ensureMasterPeer();
  if (!masterPeerAdded) return;

  uint32_t nowMs = millis();
  uint32_t hbAge = nowMs - lastHeartbeatMs;
  if (hbAge > 65535) hbAge = 65535;

  uint32_t heap = ESP.getFreeHeap();
  if (heap > 65535) heap = 65535;

  StatusMsg s{};
  s.type         = STATUS_TYPE;
  s.role         = ROLE_ELEC;
  s.millis_now   = nowMs;
  s.run_id       = currentRunId;
  s.last_seq_max = lastSeqMax;
  s.hb_age_ms    = (uint16_t)hbAge;
  s.free_heap    = (uint16_t)heap;

  trackSendResult(esp_now_send(lastMasterMac, (uint8_t*)&s, sizeof(s)));
}

// ============================================================
//  ESP-NOW RECEIVE CALLBACK
//  Runs in the WiFi system task on core 0.
//  Only queues the packet — no processing here.
// ============================================================
void OnDataRecv(const esp_now_recv_info_t *info,
                const uint8_t *incomingData, int len) {
  if (len != (int)sizeof(Msg)) return;

  RxPacket pkt;
  memcpy(pkt.mac, info->src_addr, 6);
  memcpy(&pkt.msg, incomingData, sizeof(Msg));

  // Non-blocking send; if the queue is full, the oldest packet is
  // already stale — drop silently.
  xQueueSend(msgQueue, &pkt, 0);
}

// ============================================================
//  WAVEFORM SCHEDULER HELPERS  —  waveformTask only
// ============================================================
static inline bool due(uint32_t t, uint32_t deadline) {
  return (int32_t)(t - deadline) >= 0;
}

static inline int pickNextDue(uint32_t t) {
  int      best         = -1;
  uint32_t bestDeadline = 0;
  for (int i = 0; i < NCH; i++) {
    if (chs[i].chState == ECH_IDLE) continue;
    if (!due(t, chs[i].nextEdge_us)) continue;
    if (best < 0 ||
        (int32_t)(chs[i].nextEdge_us - bestDeadline) < 0 ||
        (chs[i].nextEdge_us == bestDeadline && i < best)) {
      best         = i;
      bestDeadline = chs[i].nextEdge_us;
    }
  }
  return best;
}

// Returns microseconds until the nearest scheduled edge (UINT32_MAX if none).
static inline uint32_t usToNextEvent() {
  uint32_t t      = nowUs();
  uint32_t minGap = UINT32_MAX;
  for (int i = 0; i < NCH; i++) {
    if (chs[i].chState == ECH_IDLE) continue;
    int32_t gap = (int32_t)(chs[i].nextEdge_us - t);
    if (gap <= 0) return 0;
    if ((uint32_t)gap < minGap) minGap = (uint32_t)gap;
  }
  return minGap;
}

static inline void advanceOneEvent(uint8_t idx0) {
  ChanState &c    = chs[idx0];
  uint32_t   half = c.period_us / 2;
  if (half == 0) half = 1;

  switch (c.chState) {
    case ECH_POS_WAIT:
      writeChannelCode(idx0 + 1, c.pos_code);
      c.chState      = ECH_POS_PULSE;
      c.nextEdge_us += c.pulse_us;
      break;
    case ECH_POS_PULSE:
      writeChannelCode(idx0 + 1, c.neutral_code);
      c.chState      = ECH_NEG_WAIT;
      c.nextEdge_us += (half > c.pulse_us) ? (half - c.pulse_us) : 0;
      break;
    case ECH_NEG_WAIT:
      writeChannelCode(idx0 + 1, c.neg_code);
      c.chState      = ECH_NEG_PULSE;
      c.nextEdge_us += c.pulse_us;
      break;
    case ECH_NEG_PULSE:
      writeChannelCode(idx0 + 1, c.neutral_code);
      c.chState      = ECH_POS_WAIT;
      c.nextEdge_us += (half > c.pulse_us) ? (half - c.pulse_us) : 0;
      break;
    case ECH_IDLE:
      break; // should not happen (pickNextDue filters idle)
  }
}

// Apply a config or start update from the comm task.
static inline void applyUpdate(const ChanUpdate &upd) {
  ChanState &c = chs[upd.ch];

  // Always refresh config fields from the snapshot (full overwrite).
  c.period_us          = upd.period_us;
  c.pulse_us           = upd.pulse_us;
  c.neutral_code       = upd.neutral_code;
  c.pos_code           = upd.pos_code;
  c.neg_code           = upd.neg_code;
  c.mirrorAroundNeutral = upd.mirrorAroundNeutral;
  clampTiming(c);

  if (upd.type == ChanUpdate::START) {
    c.chState     = ECH_POS_WAIT;
    c.nextEdge_us = upd.nextEdge_us;
    writeChannelCode(upd.ch + 1, c.neutral_code);  // park at neutral
  }
  // CONFIG: runtime state (chState, nextEdge) unchanged
}

// ============================================================
//  WAVEFORM TASK  —  core 1, priority 15
//
//  Yield strategy keeps the timing loop tight while still giving
//  loop() regular CPU slices to handle messages and status sends.
//
//    gap < 500 µs  → spin-wait (no yield) — critical sub-ms window
//    gap 500 µs–2 ms → taskYIELD()       — very short reschedule
//    gap 2–5 ms    → vTaskDelay(1 tick)  — 1 ms sleep
//    gap > 5 ms    → sleep (gap–2ms)/ms  — long idle
//    no channels   → vTaskDelay(10 ticks) — deep idle
// ============================================================
static void waveformTask(void *) {
  const int MAX_EVENTS_PER_BURST = 400;

  while (true) {
    // ── 1. Apply pending config/start updates from loop() ──────
    ChanUpdate upd;
    while (xQueueReceive(updateQueue, &upd, 0) == pdTRUE) {
      applyUpdate(upd);
    }

    // ── 2. Handle STOP request from loop() ─────────────────────
    if (stopRequested) {
      safeStopAll();
      stopRequested = false;
    }

    // ── 3. Waveform scheduler burst ────────────────────────────
    // stopRequested is checked at the top of every event so a STOP
    // (from CMD_STOP or heartbeat timeout) cannot be queued behind up
    // to MAX_EVENTS_PER_BURST in-flight pulses.
    for (int events = 0; events < MAX_EVENTS_PER_BURST; events++) {
      if (stopRequested) break;
      int idx = pickNextDue(nowUs());
      if (idx < 0) break;
      advanceOneEvent((uint8_t)idx);
    }

    // ── 4. Adaptive yield ──────────────────────────────────────
    uint32_t gap = usToNextEvent();

    if (gap == UINT32_MAX) {
      // No active channels — deep idle, loop() runs freely
      vTaskDelay(pdMS_TO_TICKS(10));
    } else if (gap > 5000) {
      // More than 5 ms away — sleep for most of it, wake ~2 ms early
      uint32_t sleepMs = (gap - 2000) / 1000;
      if (sleepMs > 0) vTaskDelay(pdMS_TO_TICKS(sleepMs));
    } else if (gap > 2000) {
      // 2–5 ms — yield one FreeRTOS tick (~1 ms)
      vTaskDelay(pdMS_TO_TICKS(1));
    } else if (gap > 500) {
      // 500 µs – 2 ms — brief reschedule, let loop() handle a message
      taskYIELD();
    }
    // gap ≤ 500 µs: spin-wait, no yield — sub-ms critical window
  }
}

// ============================================================
//  SETUP
// ============================================================
void setup() {
  Serial.begin(115200);

  WiFi.mode(WIFI_STA);
  WiFi.setSleep(false);
  esp_wifi_set_ps(WIFI_PS_NONE);
  esp_wifi_set_channel(WIFI_CH, WIFI_SECOND_CHAN_NONE);

  // GPIO init
  const int allPins[] = {
    DB7, DB6, DB5, DB4, DB3, DB2, DB1, DB0,
    DAC_SEL1, DAC_SEL0,
    WR1, WR2, WR3
  };
  for (int p : allPins) pinMode(p, OUTPUT);

  // WR pins idle HIGH (active LOW strobe)
  digitalWrite(WR1, HIGH);
  digitalWrite(WR2, HIGH);
  digitalWrite(WR3, HIGH);

  // Initialise channel state and park DACs at neutral
  uint32_t t0 = nowUs();
  for (uint8_t i = 0; i < NCH; i++) {
    chs[i]     = ChanState{};
    commCfg[i] = ChanConfig{};
    chs[i].nextEdge_us = t0 + 1000000UL;  // don't fire immediately
    clampTiming(chs[i]);
    lastSeq[i] = 0;
    writeChannelCode(i + 1, DEF_NEU);
  }

  // Create inter-task queues
  msgQueue    = xQueueCreate(MSG_QUEUE_DEPTH,    sizeof(RxPacket));
  updateQueue = xQueueCreate(UPDATE_QUEUE_DEPTH, sizeof(ChanUpdate));
  if (!msgQueue || !updateQueue) {
    Serial.println("FATAL: queue creation failed");
    while (true) vTaskDelay(1000);
  }

  // Initialise ESP-NOW
  if (esp_now_init() != ESP_OK) {
    Serial.println("FATAL: ESP-NOW init failed");
    while (true) vTaskDelay(1000);
  }
  esp_now_register_recv_cb(OnDataRecv);

  lastHeartbeatMs = millis();
  lastStatusMs    = millis();
  lastSeqMax      = 0;

  // Detach core 1's IDLE task from the task watchdog. waveformTask runs
  // on core 1 at priority 15 with up to ~500 µs spin-waits, intentionally
  // starving IDLE-1. If a future build config registers IDLE-1 with the
  // TWDT (some Arduino-ESP32 / ESP-IDF combinations), that starvation
  // would panic the chip mid-run. Removing it pre-empts that risk.
  disableCore1WDT();

  // Spawn waveform task: core 1, priority 15 (above loop() at 1,
  // isolated from WiFi system tasks which run on core 0 at ~23).
  xTaskCreatePinnedToCore(
    waveformTask,   // function
    "waveform",     // name (visible in task dump)
    4096,           // stack bytes — tight loop, no deep calls
    nullptr,        // parameter
    15,             // priority
    nullptr,        // handle (not needed)
    1               // core 1  ← same as loop(), away from WiFi on core 0
  );

  Serial.println(
    "Electrical slave READY  "
    "(waveformTask core1@pri15, loop core1@pri1, "
    "sub-ms spin-wait enabled)"
  );
}

// ============================================================
//  LOOP  (comm task)  —  core 1, priority 1
//
//  All ESP-NOW I/O (receive, ACK, status) lives here.
//  waveformTask preempts this whenever it becomes ready.
// ============================================================
void loop() {
  // ── 1. Drain incoming message queue ────────────────────────
  RxPacket pkt;
  while (xQueueReceive(msgQueue, &pkt, 0) == pdTRUE) {
    memcpy(lastMasterMac, pkt.mac, 6);
    dispatchMsg(pkt.msg);
  }

  // ── 2. 1 Hz status heartbeat to master ─────────────────────
  uint32_t nowMs = millis();
  if ((uint32_t)(nowMs - lastStatusMs) >= STATUS_PERIOD_MS) {
    lastStatusMs = nowMs;
    sendStatus();
  }

  // ── 3. Heartbeat failsafe ───────────────────────────────────
  // Uses stopRequested flag (not direct safeStopAll) because safeStopAll
  // writes to GPIOs owned by waveformTask on the same core at priority 15.
  // Pressure/Servo slaves call safeStopAll() directly since they're single-threaded.
  if (runActive) {
    uint32_t age = millis() - lastHeartbeatMs;
    if (age > HEARTBEAT_TIMEOUT_MS) {
      Serial.println("Heartbeat timeout -> safe stop");
      runActive     = false;   // prevent repeated triggers
      stopRequested = true;    // waveformTask will call safeStopAll()
    }
  }

  // No explicit vTaskDelay here — waveformTask's yielding strategy
  // already gives loop() regular CPU slices between pulse events.
  // loop() runs whenever waveformTask calls vTaskDelay / taskYIELD.
}
