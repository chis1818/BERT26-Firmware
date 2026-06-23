#include <Arduino.h>
#include <FlexCAN_T4.h>

// Enum at top — Arduino auto-prototype generation requires types before any function signatures
enum BmsState {
  STATE_INIT,
  STATE_WAIT_SDC,
  STATE_PRECHARGE,
  STATE_CLOSE_POSITIVE,
  STATE_RUNNING,
  STATE_FAULT
};

// -----------------------------------------------------------------------
// Calibration (Chan B linear fit, applied to both channels)
//   V = (ADC - CAL_INTERCEPT) / CAL_SLOPE
static constexpr float CAL_SLOPE     = 5.2957f;
static constexpr float CAL_INTERCEPT = 1.83f;

// -----------------------------------------------------------------------
// Precharge parameters — FSAE 2026 EV.5.6
//   RC = 1200 Ω × 440 µF = 528 ms
//   t_90 = -RC·ln(0.1) ≈ 1216 ms
//   EV.5.6.2 max = max(2×1216ms, 1216ms+500ms) = 2432 ms → round up to 2500 ms
static constexpr float    PRECHARGE_THRESHOLD  = 0.90f;  // 90% of battery voltage
static constexpr uint32_t PRECHARGE_TIMEOUT_MS = 2500;   // EV.5.6.2
static constexpr uint32_t PRECHARGE_MIN_MS     = 200;    // too-fast fault threshold
static constexpr uint32_t INIT_DELAY_MS        = 3000;   // boot stabilisation before pin 41 rises

// RUNNING-state IMD vs iso-amp cross-check
static constexpr float DISAGREE_ABS_V = 10.0f; // absolute disagreement threshold [V]
static constexpr float DISAGREE_PCT   = 0.05f; // relative threshold (5%)

// Minimum sensible battery voltage — below this during precharge is a sensor fault
static constexpr float CHAN_A_MIN_V = 10.0f;

// Poll intervals
static constexpr uint32_t PRECHARGE_POLL_MS    = 2;
static constexpr uint32_t RUNNING_POLL_MS      = 200;
static constexpr uint32_t IMD_POLL_MS          = 100;  // 10 Hz — maximum per iso175 spec
static constexpr uint32_t CLOSE_SETTLE_MS      = 100; // wait after closing positive AIR before opening precharge relay

// -----------------------------------------------------------------------
// Pins
#define PIN_CHAN_A        14  // A0 — battery-side isolated amp
#define PIN_CHAN_B        15  // A1 — inverter-side isolated amp
#define PIN_NEG_SIGNAL    20  // Rising edge = negative contactor closed (from SDC)
#define PIN_PRECHARGE     24  // Precharge relay low-side driver
#define PIN_POS_AIR       25  // Positive AIR low-side driver
#define PIN_BMS_STATUS    41  // BMS → SDC: HIGH = nominal, LOW = fault

// -----------------------------------------------------------------------
// CAN / Bender iso175 (redundant voltage only — isolation monitoring is VCU's job)
FlexCAN_T4<CAN3, RX_SIZE_256, TX_SIZE_16> Can3;
static constexpr uint32_t ID_IMD_REQUEST  = 0x22;
static constexpr uint32_t ID_IMD_RESPONSE = 0x23;
static constexpr uint8_t  IDX_VOLTAGE_HV  = 0x5E; // HV_System: (raw-32128)×0.05 V, LE

// -----------------------------------------------------------------------
// State

BmsState g_state = STATE_INIT;
volatile bool g_negContactorClosed = false; // set by ISR

// IMD voltage (updated by CAN callback)
volatile bool  g_imd_valid   = false;
volatile float g_imd_voltage = 0.0f;

// Timers
elapsedMillis initTimer;
elapsedMillis prechargeTimer;
elapsedMillis voltPollTimer;
elapsedMillis imdPollTimer;
elapsedMillis closeSettleTimer;

// -----------------------------------------------------------------------
// ISR — negative contactor closed signal

void pin20ISR() {
  g_negContactorClosed = true;
}

// -----------------------------------------------------------------------
// Helpers

static inline uint16_t u16le(uint8_t lo, uint8_t hi) {
  return (uint16_t)lo | ((uint16_t)hi << 8);
}

static float decodeIMDVoltage(uint16_t raw) {
  return ((float)raw - 32128.0f) * 0.05f;
}

static float adcToVolts(int adc) {
  return ((float)adc - CAL_INTERCEPT) / CAL_SLOPE;
}

// Averaged ADC read converted to volts
static float readVoltage(int pin, int samples = 16) {
  long sum = 0;
  for (int i = 0; i < samples; i++) sum += analogRead(pin);
  return adcToVolts((int)(sum / samples));
}

void sendGet(uint8_t index) {
  CAN_message_t tx{};
  tx.id     = ID_IMD_REQUEST;
  tx.len    = 8;
  tx.buf[0] = index;
  for (int i = 1; i < 8; i++) tx.buf[i] = 0xFF;
  Can3.write(tx);
}

// Latching fault — requires power cycle to clear
void enterFault(const char* reason) {
  digitalWrite(PIN_PRECHARGE,  LOW);
  digitalWrite(PIN_POS_AIR,    LOW);
  digitalWrite(PIN_BMS_STATUS, LOW); // open SDC → AIRs de-energize
  g_state = STATE_FAULT;
  Serial.print("[FAULT] ");
  Serial.println(reason);
  Serial.println("Power cycle required to reset.");
}

// -----------------------------------------------------------------------
// CAN callback — only listen for IMD voltage response

void canSniff(const CAN_message_t &msg) {
  if (msg.id == ID_IMD_RESPONSE && msg.buf[0] == IDX_VOLTAGE_HV) {
    uint16_t raw = u16le(msg.buf[1], msg.buf[2]);
    if (raw == 0xFFFF) {
      g_imd_valid = false;
    } else {
      g_imd_voltage = decodeIMDVoltage(raw);
      g_imd_valid   = true;
    }
  }
  // All other frames ignored — isolation monitoring is the VCU's job
}

// -----------------------------------------------------------------------
// State handlers

void handleInit() {
  if (initTimer >= INIT_DELAY_MS) {
    digitalWrite(PIN_BMS_STATUS, HIGH); // BMS nominal — SDC may now close negative AIR
    g_state = STATE_WAIT_SDC;
    Serial.println("[BMS] INIT done -> WAIT_SDC | Pin 41 HIGH");
  }
}

void handleWaitSDC() {
  if (g_negContactorClosed) {
    static bool settling = false;
    static elapsedMillis settleTimer;
    if (!settling) {
      settling = true;
      settleTimer = 0;
      Serial.println("[BMS] Neg contactor closed — settling 1000ms before precharge...");
    }
    if (settleTimer >= 1000) {
      settling = false;
      digitalWrite(PIN_PRECHARGE, HIGH);
      prechargeTimer = 0;
      voltPollTimer  = 0;
      imdPollTimer   = 0;
      g_state = STATE_PRECHARGE;
      Serial.println("[BMS] -> PRECHARGE | Pin 24 HIGH");
    }
  }
}

void handlePrecharge() {
  // Poll IMD voltage every IMD_POLL_MS
  if (imdPollTimer >= IMD_POLL_MS) {
    imdPollTimer = 0;
    sendGet(IDX_VOLTAGE_HV);
  }

  if (voltPollTimer < PRECHARGE_POLL_MS) return;
  voltPollTimer = 0;

  float v_batt    = readVoltage(PIN_CHAN_A, 16);
  uint32_t elapsedMs = prechargeTimer;

  // Fault: battery voltage implausibly low — sensor or wiring problem
  if (v_batt < CHAN_A_MIN_V) {
    enterFault("Chan A (battery) < 10V during precharge — check sensor/wiring");
    return;
  }

  // Fault: took too long (EV.5.6.2)
  if (elapsedMs > PRECHARGE_TIMEOUT_MS) {
    enterFault("Precharge timeout >1s (FSAE EV.5.6.2) — open/bad precharge resistor?");
    return;
  }

  // Wait for a valid IMD reading before evaluating the threshold
  if (!g_imd_valid) {
    Serial.printf("[PRECHARGE] %5lu ms | Batt=%6.2f V | IMD=pending\n", elapsedMs, v_batt);
    return;
  }

  float v_inv = g_imd_voltage;
  Serial.printf("[PRECHARGE] %5lu ms | Batt=%6.2f V | IMD=%6.2f V | %.1f%%\n",
                elapsedMs, v_batt, v_inv,
                (v_batt > 0.1f) ? (v_inv / v_batt * 100.0f) : 0.0f);

  // Check 90% threshold
  if (v_inv >= PRECHARGE_THRESHOLD * v_batt) {
    // Fault: reached 90% suspiciously fast — bypassed resistor or shorted cap
    if (elapsedMs < PRECHARGE_MIN_MS) {
      enterFault("Precharge too fast <50ms — bypassed resistor or shorted capacitor");
      return;
    }
    // Success
    digitalWrite(PIN_POS_AIR, HIGH);
    closeSettleTimer = 0;
    g_state = STATE_CLOSE_POSITIVE;
    Serial.printf("[BMS] Precharge complete in %lu ms (%.1f%%) -> CLOSE_POSITIVE | Pin 25 HIGH\n",
                  elapsedMs, v_inv / v_batt * 100.0f);
  }
}

void handleClosePositive() {
  // After settle time, open precharge relay — relay no longer carries load
  if (closeSettleTimer >= CLOSE_SETTLE_MS) {
    digitalWrite(PIN_PRECHARGE, LOW);
    voltPollTimer = 0;
    imdPollTimer  = 0;
    g_state = STATE_RUNNING;
    Serial.println("[BMS] Pin 24 LOW (precharge relay open) -> RUNNING");
  }
}

void handleRunning() {
  // Voltage readout — Chan A (battery) vs IMD (inverter/output side)
  if (voltPollTimer >= RUNNING_POLL_MS) {
    voltPollTimer = 0;

    float v_batt = readVoltage(PIN_CHAN_A, 16);

    if (g_imd_valid) {
      float imd       = g_imd_voltage;
      float threshold = fmaxf(DISAGREE_ABS_V, DISAGREE_PCT * imd);
      float delta     = fabsf(v_batt - imd);

      Serial.printf("[RUNNING] Batt=%6.2f V | IMD=%6.2f V | delta=%.2f V\n",
                    v_batt, imd, delta);

      if (delta > threshold) {
        Serial.printf("[BMS] Disagreement: Batt=%.1fV IMD=%.1fV delta=%.1fV threshold=%.1fV\n",
                      v_batt, imd, delta, threshold);
        enterFault("Battery iso-amp and IMD voltage disagree beyond threshold");
        return;
      }
    } else {
      Serial.printf("[RUNNING] Batt=%6.2f V | IMD=pending\n", v_batt);
    }
  }

  // IMD voltage poll
  if (imdPollTimer >= IMD_POLL_MS) {
    imdPollTimer = 0;
    sendGet(IDX_VOLTAGE_HV);
  }
}

// -----------------------------------------------------------------------

void setup() {
  Serial.begin(115200);
  while (!Serial && millis() < 2000) {}
  Serial.println("BMS Precharge Firmware — Teensy 4.1");
  Serial.println("FSAE 2026 EV.5.6 | PM100DX 440uF | 390 Ohm | 90% threshold | 1s timeout");

  // Outputs — safe-low on boot
  pinMode(PIN_BMS_STATUS, OUTPUT); digitalWrite(PIN_BMS_STATUS, LOW);
  pinMode(PIN_PRECHARGE,  OUTPUT); digitalWrite(PIN_PRECHARGE,  LOW);
  pinMode(PIN_POS_AIR,    OUTPUT); digitalWrite(PIN_POS_AIR,    LOW);

  // Inputs
  analogReadResolution(12);
  pinMode(PIN_CHAN_A,     INPUT);
  pinMode(PIN_CHAN_B,     INPUT);
  pinMode(PIN_NEG_SIGNAL, INPUT);
  attachInterrupt(digitalPinToInterrupt(PIN_NEG_SIGNAL), pin20ISR, RISING);

  // CAN3 — Bender iso175
  Can3.setRX(31);
  Can3.setTX(30);
  Can3.begin();
  Can3.setBaudRate(500000);
  Can3.setMaxMB(16);
  Can3.enableFIFO();
  Can3.enableFIFOInterrupt();
  Can3.onReceive(canSniff);

  initTimer = 0;
  g_state   = STATE_INIT;
  Serial.println("[BMS] STATE_INIT — waiting 3s before signaling nominal to SDC...");
}

void loop() {
  Can3.events();

  switch (g_state) {
    case STATE_INIT:           handleInit();           break;
    case STATE_WAIT_SDC:       handleWaitSDC();        break;
    case STATE_PRECHARGE:      handlePrecharge();      break;
    case STATE_CLOSE_POSITIVE: handleClosePositive();  break;
    case STATE_RUNNING:        handleRunning();        break;
    case STATE_FAULT:          /* latched — power cycle to reset */ break;
  }
}
