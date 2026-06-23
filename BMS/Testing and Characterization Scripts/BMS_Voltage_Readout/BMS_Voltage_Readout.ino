#include <Arduino.h>
#include <FlexCAN_T4.h>

// --- Calibration from characterization (Chan B fit, applied to both channels) ---
// V = (ADC - INTERCEPT) / SLOPE
static constexpr float CAL_SLOPE     = 5.2957f;
static constexpr float CAL_INTERCEPT = 1.83f;

// --- Pins ---
#define PIN_41    41   // Negative contactor — HIGH after 3 s
#define PIN_ISO_A 14   // A0
#define PIN_ISO_B 15   // A1

// --- Bender iso175 CAN ---
FlexCAN_T4<CAN3, RX_SIZE_256, TX_SIZE_16> Can3;
static constexpr uint32_t ID_IMD_REQUEST  = 0x22;
static constexpr uint32_t ID_IMD_RESPONSE = 0x23;
static constexpr uint8_t  IDX_VOLTAGE_HV  = 0x5E; // (raw-32128)*0.05 V, LE DataWord

volatile bool  g_imd_valid   = false;
volatile float g_imd_voltage = 0.0f;

elapsedMillis pollTimer;
elapsedMillis printTimer;
static constexpr uint32_t POLL_MS  = 300;  // IMD GET rate
static constexpr uint32_t PRINT_MS = 500;  // Serial print rate

// -----------------------------------------------------------------------

static inline uint16_t u16le(uint8_t lo, uint8_t hi) {
  return (uint16_t)lo | ((uint16_t)hi << 8);
}

static float decodeIMDVoltage(uint16_t raw) {
  return ((float)raw - 32128.0f) * 0.05f;
}

static float adcToVolts(int adc) {
  return ((float)adc - CAL_INTERCEPT) / CAL_SLOPE;
}

void sendGet(uint8_t index) {
  CAN_message_t tx{};
  tx.id     = ID_IMD_REQUEST;
  tx.len    = 8;
  tx.buf[0] = index;
  for (int i = 1; i < 8; i++) tx.buf[i] = 0xFF;
  Can3.write(tx);
}

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
}

// -----------------------------------------------------------------------

void setup() {
  Serial.begin(115200);
  while (!Serial && millis() < 2000) {}
  Serial.println("BMS Voltage Readout -- Teensy 4.1");
  Serial.printf("Calibration: V = (ADC - %.2f) / %.4f\n", CAL_INTERCEPT, CAL_SLOPE);
  Serial.println("  IMD_V (V) | Chan_A (V) | Chan_B (V) | ADC_A | ADC_B");

  analogReadResolution(12);
  pinMode(PIN_ISO_A, INPUT);
  pinMode(PIN_ISO_B, INPUT);

  Can3.setRX(31);
  Can3.setTX(30);
  Can3.begin();
  Can3.setBaudRate(500000);
  Can3.setMaxMB(16);
  Can3.enableFIFO();
  Can3.enableFIFOInterrupt();
  Can3.onReceive(canSniff);

  // Pin 41: goes HIGH after 3 seconds
  pinMode(PIN_41, OUTPUT);
  digitalWrite(PIN_41, LOW);
  delay(3000);
  digitalWrite(PIN_41, HIGH);
  Serial.println("Negative contactor closed.");
}

void loop() {
  Can3.events();

  if (pollTimer >= POLL_MS) {
    pollTimer = 0;
    sendGet(IDX_VOLTAGE_HV);
  }

  if (printTimer >= PRINT_MS) {
    printTimer = 0;

    // Average 64 reads for stability
    long sum_a = 0, sum_b = 0;
    for (int i = 0; i < 64; i++) {
      sum_a += analogRead(PIN_ISO_A);
      sum_b += analogRead(PIN_ISO_B);
    }
    int adc_a = (int)(sum_a / 64);
    int adc_b = (int)(sum_b / 64);

    float v_a   = adcToVolts(adc_a);
    float v_b   = adcToVolts(adc_b);
    float v_imd = g_imd_valid ? g_imd_voltage : -999.0f;

    Serial.printf("%9.2f | %10.2f | %10.2f | %5d | %5d\n",
                  v_imd, v_a, v_b, adc_a, adc_b);
  }
}
