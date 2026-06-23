#include <Arduino.h>
#include <FlexCAN_T4.h>

// Structs must live before any function definitions so Arduino's
// auto-generated prototypes can reference them without errors.
struct Sample { float imd_v; int adc_a; int adc_b; };
struct LinFit  { double slope, intercept, r2; int n; };

// -----------------------------------------------------------------------
// Pinout
#define PIN_41     41   // Negative contactor — HIGH after 3 s
#define PIN_ISO_A  14   // A0 — isolated amp channel A
#define PIN_ISO_B  15   // A1 — isolated amp channel B

// --- Bender iso175 CAN ---
FlexCAN_T4<CAN3, RX_SIZE_256, TX_SIZE_16> Can3;
static constexpr uint32_t ID_IMD_REQUEST  = 0x22;
static constexpr uint32_t ID_IMD_RESPONSE = 0x23;
static constexpr uint8_t  IDX_VOLTAGE_HV  = 0x5E; // HV_System: (raw-32128)*0.05 V, LE DataWord

// Latest IMD voltage — updated from CAN callback
volatile bool  g_imd_valid   = false;
volatile float g_imd_voltage = 0.0f;

// --- Sample storage ---
static constexpr int MAX_SAMPLES = 60;
static Sample samples[MAX_SAMPLES];
static int    sampleCount = 0;

elapsedMillis pollTimer;
static constexpr uint32_t POLL_MS = 300;

// -----------------------------------------------------------------------
// Helpers

static inline uint16_t u16le(uint8_t lo, uint8_t hi) {
  return (uint16_t)lo | ((uint16_t)hi << 8);
}

static float decodeIMDVoltage(uint16_t raw) {
  return ((float)raw - 32128.0f) * 0.05f;
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
// Linear least-squares: ADC = slope * V_imd + intercept

LinFit computeFit(int channel) {
  int n = sampleCount;
  LinFit f = {0, 0, 0, n};
  if (n < 2) return f;

  double sx = 0, sy = 0, sxy = 0, sxx = 0, syy = 0;
  for (int i = 0; i < n; i++) {
    double x = samples[i].imd_v;
    double y = (channel == 0) ? samples[i].adc_a : samples[i].adc_b;
    sx += x; sy += y; sxy += x * y; sxx += x * x; syy += y * y;
  }
  double denom = n * sxx - sx * sx;
  if (denom == 0.0) return f;
  f.slope     = (n * sxy - sx * sy) / denom;
  f.intercept = (sy - f.slope * sx) / n;

  double ss_res = 0, ss_tot = 0, mean_y = sy / n;
  for (int i = 0; i < n; i++) {
    double y     = (channel == 0) ? samples[i].adc_a : samples[i].adc_b;
    double y_hat = f.slope * samples[i].imd_v + f.intercept;
    ss_res += (y - y_hat) * (y - y_hat);
    ss_tot += (y - mean_y) * (y - mean_y);
  }
  f.r2 = (ss_tot > 0) ? (1.0 - ss_res / ss_tot) : 1.0;
  return f;
}

void printFit() {
  if (sampleCount < 2) { Serial.println("Need >= 2 samples for fit."); return; }
  LinFit fa = computeFit(0);
  LinFit fb = computeFit(1);
  Serial.println("\n=== Linear Fit  (ADC = slope * V_imd + intercept) ===");
  Serial.printf("  Chan A (pin 14): slope=%8.4f  intercept=%8.2f  R2=%.6f  n=%d\n",
                fa.slope, fa.intercept, fa.r2, fa.n);
  Serial.printf("  Chan B (pin 15): slope=%8.4f  intercept=%8.2f  R2=%.6f  n=%d\n",
                fb.slope, fb.intercept, fb.r2, fb.n);
  Serial.println("=== Inverse  (V = (ADC - intercept) / slope) ===");
  Serial.printf("  Chan A: V = (ADC - %.2f) / %.4f\n", fa.intercept, fa.slope);
  Serial.printf("  Chan B: V = (ADC - %.2f) / %.4f\n", fb.intercept, fb.slope);
}

void printSamples() {
  Serial.println("\n--- Samples ---");
  Serial.println("  # |   IMD_V  | ADC_A | ADC_B");
  for (int i = 0; i < sampleCount; i++) {
    Serial.printf("%3d | %8.2f | %5d | %5d\n",
                  i + 1, samples[i].imd_v, samples[i].adc_a, samples[i].adc_b);
  }
}

void takeSample() {
  if (!g_imd_valid) { Serial.println("IMD voltage not ready yet."); return; }
  if (sampleCount >= MAX_SAMPLES) { Serial.println("Buffer full — clear with [c]."); return; }

  long sum_a = 0, sum_b = 0;
  for (int i = 0; i < 64; i++) {
    sum_a += analogRead(PIN_ISO_A);
    sum_b += analogRead(PIN_ISO_B);
  }
  int   adc_a = (int)(sum_a / 64);
  int   adc_b = (int)(sum_b / 64);
  float v     = g_imd_voltage;

  samples[sampleCount++] = { v, adc_a, adc_b };
  Serial.printf("\nSample %d: IMD=%.2f V  |  ADC_A=%d  |  ADC_B=%d\n",
                sampleCount, v, adc_a, adc_b);
  printFit();
}

// -----------------------------------------------------------------------

void setup() {
  Serial.begin(115200);
  while (!Serial && millis() < 2000) {}
  Serial.println("ISO Amp Characterization -- Teensy 4.1");
  Serial.println("Set supply voltage, then:");
  Serial.println("  [s] take sample  [p] print samples  [c] clear  [f] print fit");

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

  if (Serial.available()) {
    char c = Serial.read();
    switch (c) {
      case 's': case 'S': takeSample();                                   break;
      case 'p': case 'P': printSamples();                                 break;
      case 'c': case 'C': sampleCount = 0; Serial.println("Cleared.");   break;
      case 'f': case 'F': printFit();                                     break;
    }
  }
}
