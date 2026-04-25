// =====================================================================
//  ELECTRICAL STANDALONE TEST
//
//  Drives the 12-channel bipolar DAC bus directly — no ESP-NOW, no Wi-Fi,
//  no master. Use this to verify the analog electrical front end is
//  wired and pulsing correctly before bringing the wireless layer up.
//
//  Pattern (every channel runs continuously):
//    • Frequency:  0.5 Hz   (2-second period)
//    • Bipolar:    +15V pulse → 0V → −15V pulse → 0V, repeating
//    • Pulse widths (per channel):
//        ch1  = 2000 µs
//        ch2  = 3000 µs
//        ch3  = 4000 µs
//          ⋮
//        ch12 = 13000 µs    (each channel +1000 µs vs the previous)
//    • All 12 channels phase-aligned: their rising edges fire together.
//
//  Pin map, DAC select, WR strobe timing, and the 2/3/6/7/10/11 channel
//  remap are taken verbatim from Electrical_Receiver_V2.ino so the
//  observed waveforms are directly comparable to the wireless build.
// =====================================================================

#include <Arduino.h>
#include <soc/gpio_struct.h>

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

// ---------- DAC CODES (8-bit, ±15V span) ----------
static const uint8_t CODE_POS = 0xFF;  // +15V
static const uint8_t CODE_NEU = 0x80;  //   0V (idle)
static const uint8_t CODE_NEG = 0x00;  // −15V

// ---------- PATTERN ----------
static const uint32_t PERIOD_US     = 2000000;  // 0.5 Hz
static const uint32_t BASE_PULSE_US = 2000;     // ch1 pulse width
static const uint32_t PULSE_STEP_US = 1000;     // step per subsequent channel

// =====================================================================
//  LOW-LEVEL DAC BUS (verbatim from Electrical_Receiver_V2)
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
  GPIO.out_w1tc = (1UL << wrPin);  // WR LOW
  delayMicroseconds(2);            // DAC latch hold time — do not reduce
  GPIO.out_w1ts = (1UL << wrPin);  // WR HIGH
}

// Logical channels 2↔3, 6↔7, 10↔11 are physically swapped on the board.
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
  delayMicroseconds(1);     // data setup
  setDacSelect(dacSel);
  delayMicroseconds(1);     // select setup
  strobeWR(wrPin);
}

// =====================================================================
//  WAVEFORM SCHEDULER
// =====================================================================
enum Phase : uint8_t { POS_WAIT = 0, POS_PULSE, NEG_WAIT, NEG_PULSE };

struct ChState {
  Phase    phase;
  uint32_t pulse_us;
  uint32_t nextEdge_us;
};

static ChState chs[NCH];

static inline bool due(uint32_t now, uint32_t deadline) {
  return (int32_t)(now - deadline) >= 0;
}

static void advanceChannel(uint8_t i) {
  ChState &c = chs[i];
  const uint32_t half = PERIOD_US / 2;

  switch (c.phase) {
    case POS_WAIT:
      writeChannelCode(i + 1, CODE_POS);
      c.phase        = POS_PULSE;
      c.nextEdge_us += c.pulse_us;
      break;
    case POS_PULSE:
      writeChannelCode(i + 1, CODE_NEU);
      c.phase        = NEG_WAIT;
      c.nextEdge_us += (half - c.pulse_us);
      break;
    case NEG_WAIT:
      writeChannelCode(i + 1, CODE_NEG);
      c.phase        = NEG_PULSE;
      c.nextEdge_us += c.pulse_us;
      break;
    case NEG_PULSE:
      writeChannelCode(i + 1, CODE_NEU);
      c.phase        = POS_WAIT;
      c.nextEdge_us += (half - c.pulse_us);
      break;
  }
}

// =====================================================================
//  SETUP / LOOP
// =====================================================================
void setup() {
  Serial.begin(115200);
  delay(200);
  Serial.println();
  Serial.println("Electrical standalone test");
  Serial.printf("  period:    %lu us  (%.2f Hz)\n",
                (unsigned long)PERIOD_US, 1e6f / (float)PERIOD_US);
  Serial.printf("  ch1 pulse: %lu us, step +%lu us per channel\n",
                (unsigned long)BASE_PULSE_US, (unsigned long)PULSE_STEP_US);

  // Configure all bus + control pins as outputs.
  const int allPins[] = {
    DB7, DB6, DB5, DB4, DB3, DB2, DB1, DB0,
    DAC_SEL1, DAC_SEL0,
    WR1, WR2, WR3
  };
  for (int p : allPins) pinMode(p, OUTPUT);

  // WR pins idle HIGH (active-low strobe).
  digitalWrite(WR1, HIGH);
  digitalWrite(WR2, HIGH);
  digitalWrite(WR3, HIGH);

  // Park every DAC at neutral before doing anything time-critical.
  for (uint8_t i = 0; i < NCH; i++) {
    writeChannelCode(i + 1, CODE_NEU);
  }

  // Stage all 12 channels to fire their first +pulse together, with a
  // 500 ms grace period so the very first edge isn't racing setup().
  uint32_t t0 = (uint32_t)micros() + 500000;
  for (uint8_t i = 0; i < NCH; i++) {
    chs[i].phase       = POS_WAIT;
    chs[i].pulse_us    = BASE_PULSE_US + (uint32_t)i * PULSE_STEP_US;
    chs[i].nextEdge_us = t0;
    Serial.printf("  ch%2u pulse = %lu us\n",
                  (unsigned)(i + 1), (unsigned long)chs[i].pulse_us);
  }

  Serial.println("running...");
}

void loop() {
  uint32_t now = (uint32_t)micros();
  for (uint8_t i = 0; i < NCH; i++) {
    while (due(now, chs[i].nextEdge_us)) {
      advanceChannel(i);
      now = (uint32_t)micros();
    }
  }
}
