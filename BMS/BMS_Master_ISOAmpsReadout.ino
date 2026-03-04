// Teensy 4.1: Read analog pins 14 and 15 and stream over UART (Serial)
//
// Scaling: V_adc = V_high * GAIN
// => V_high = V_adc / GAIN

#include <Arduino.h>

static const int TB_PIN = 14;   // A14 on Teensy 4.1
static const int HV_PIN = 15;   // A15 on Teensy 4.1

static const float VREF = 3.3f;          // Teensy 4.1 ADC reference (typical)
static const float GAIN = 0.00428776f;   // your divider/conditioning gain

// Choose ADC resolution and matching full-scale count
static const int ADC_BITS = 12;          // 12-bit default (0..4095)
static const float ADC_MAX = (1 << ADC_BITS) - 1; // 4095 for 12-bit

// Output rate
static const uint32_t SAMPLE_PERIOD_MS = 50; // 20 Hz

void setup() {
  Serial.begin(115200);
  while (!Serial && millis() < 2000) {
    // Wait briefly for Serial Monitor (optional)
  }

  analogReadResolution(ADC_BITS);

  // Optional: averaging helps noise
  analogReadAveraging(16);

  // Optional: set ADC conversion/sampling speed (Teensy-specific)
  // analogReadRes is enough for many cases; keep it simple unless needed.

  Serial.println("ms,raw14,v_adc14,v_high14,raw15,v_adc15,v_high15");
}

void loop() {
  static uint32_t last_ms = 0;
  uint32_t now = millis();
  if (now - last_ms < SAMPLE_PERIOD_MS) return;
  last_ms = now;

  // Read raw ADC counts
  int raw14 = analogRead(TB_PIN);
  int raw15 = analogRead(HV_PIN);

  // Convert to ADC pin voltage
  float v_adc14 = (raw14 * VREF) / ADC_MAX;
  float v_adc15 = (raw15 * VREF) / ADC_MAX;

  // Convert to high voltage (pre-divider / pre-gain)
  float v_high14 = (v_adc14 / GAIN)-95; // added DC offest of 95
  float v_high15 = (v_adc15 / GAIN)-95;

  // Optional clamp / sanity (since you expect up to ~410 V)
  // if (v_high14 < 0) v_high14 = 0;
  // if (v_high15 < 0) v_high15 = 0;

  // Broadcast as CSV
  Serial.print(now);
  Serial.print(",");
  Serial.print(raw14);
  Serial.print(",");
  Serial.print(v_adc14, 6);
  Serial.print(",");
  Serial.print(v_high14, 3);
  Serial.print(",");
  Serial.print(raw15);
  Serial.print(",");
  Serial.print(v_adc15, 6);
  Serial.print(",");
  Serial.println(v_high15, 3);
}