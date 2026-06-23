// ============================================================
// Energy Meter CAN -> USB Decoder  (Teensy 4.1)
// ------------------------------------------------------------
// Sits on the main vehicle CAN bus, listens ONLY for the FSAE
// competition Energy Meter's messages, fully decodes them into
// human-readable text, and prints that over the USB serial (UART).
//
// CAN settings are taken from BMS_Full.ino. The Energy Meter runs at
// 500 kbps (EM_User_Manual_2026, p.9 "CAN Baud Rate: 500k"), which
// matches the BMS_Full *vehicle* bus:
//     CAN3, pins 30(TX)/31(RX), 500000 baud, 11-bit standard frames.
// (BMS_Full's other bus, CAN1, is the 250 kbps charger bus — wrong
//  baud — so CAN3 is the channel selected here.)
//
// Because the bridge shares the main vehicle bus with the BMS, IMD and
// everything else, hardware acceptance filters are configured so the
// FIFO only ever delivers the three Energy-Meter-transmitted IDs.
//
// All Energy Meter signals are little-endian (Intel), per the manual.
// ============================================================
#include <Arduino.h>
#include <FlexCAN_T4.h>

// ============================================================
// CAN BUS — copied from BMS_Full.ino CanVehicle (the 500 kbps bus)
// ============================================================
FlexCAN_T4<CAN3, RX_SIZE_256, TX_SIZE_16> CanEM;
static constexpr uint32_t EM_CAN_BAUD = 500000;   // matches Energy Meter
static constexpr uint8_t  EM_CAN_TX   = 30;       // CAN3 TX (BMS_Full)
static constexpr uint8_t  EM_CAN_RX   = 31;       // CAN3 RX (BMS_Full)

// ============================================================
// OUTPUT — USB serial only
// ============================================================
static constexpr uint32_t USB_BAUD = 115200;

// ============================================================
// ENERGY METER MESSAGE IDs (11-bit) — EM_User_Manual_2026 p.10-11
// These are the only IDs the Energy Meter TRANSMITS, so they are the
// only frames we accept off the shared vehicle bus.
// ============================================================
static constexpr uint32_t ID_EM_MEASUREMENT = 0x10D; // 20 ms:  Current[A] f32, Voltage[V] f32
static constexpr uint32_t ID_EM_STATUS      = 0x40D; // 100 ms: flags + Energy[Whr] f32
static constexpr uint32_t ID_EM_TEMPERATURE = 0x60D; // 250 ms: muxed temperatures

// Up to 32 temperature sensors (mux 0..6, 5 temps per frame; mux6 carries 2)
static constexpr uint8_t EM_MAX_TEMPS = 32;
static float   g_temps[EM_MAX_TEMPS];
static bool    g_temp_valid[EM_MAX_TEMPS];
static uint8_t g_temp_num_sensors = 0;
static float   g_temp_min = NAN, g_temp_max = NAN;

// Latest measurement (0x10D, 20 ms), folded in until a fresh temperature
// sweep (0x60D, 250 ms) completes and the combined block is printed.
static float g_current = NAN, g_voltage = NAN;

// ============================================================
// DECODE HELPERS — all little-endian per manual
// ============================================================
static float le_float(const uint8_t* p) {
  // Teensy 4.x is little-endian, so a straight copy matches Intel byte order.
  float f;
  memcpy(&f, p, sizeof(f));
  return f;
}

// Output cadence: the temperature sweep (0x60D) is the slowest source of
// fresh data (~250 ms), so the monitor is printed once per sweep period.
// This keeps measurement and temperature matched at one rate and is robust
// regardless of how many sensors the meter reports (no reliance on a
// specific mux frame arriving).
static constexpr uint32_t MONITOR_PERIOD_MS = 250;
static uint32_t g_last_print_ms = 0;

// ------------------------------------------------------------
// Print the latest measurement together with the most recent temperature
// set. Called on the monitor period so both are reported at the same
// cadence, using the freshest values folded in by the decoders.
// ------------------------------------------------------------
static void printMonitor() {
  float power_kW = g_current * g_voltage / 1000.0f;
  Serial.printf("[EM MEASUREMENT]  Current = %8.3f A   Voltage = %8.3f V   Power = %7.3f kW\n",
                g_current, g_voltage, power_kW);
  Serial.printf("[EM TEMPERATURE]  Sensors = %u   Min = %.1f C   Max = %.1f C\n",
                g_temp_num_sensors, g_temp_min, g_temp_max);
  Serial.print("                  ");
  for (uint8_t i = 0; i < EM_MAX_TEMPS; i++) {
    if (!g_temp_valid[i]) continue;
    Serial.printf("T%-2u=%.1fC  ", i, g_temps[i]);
    if ((i % 8) == 7) Serial.print("\n                  ");
  }
  Serial.println();
}

// ------------------------------------------------------------
// 0x10D Measurement: Current[A] float @0-3, Voltage[V] float @4-7
// Folded in at the 20 ms frame rate; printed at the slower sweep rate.
// ------------------------------------------------------------
static void decodeMeasurement(const CAN_message_t& m) {
  if (m.len < 8) return;
  g_current = le_float(&m.buf[0]);
  g_voltage = le_float(&m.buf[4]);
}

// ------------------------------------------------------------
// 0x40D Status: byte0 bits 0..3 = flags, Energy[Whr] float @4-7
// ------------------------------------------------------------
static void decodeStatus(const CAN_message_t& m) {
  if (m.len < 8) return;
  uint8_t flags          = m.buf[0];
  bool violation         = flags & 0x01;
  bool logging           = flags & 0x02;
  bool fault_active      = flags & 0x04;
  bool fault_prev_active = flags & 0x08;
  float energy_Whr       = le_float(&m.buf[4]);
  Serial.printf("[EM STATUS]       Violation = %-3s   Logging = %-3s   FaultActive = %-3s   FaultPrev = %-3s   Energy = %.2f Whr\n",
                violation ? "YES" : "no",
                logging ? "YES" : "no",
                fault_active ? "YES" : "no",
                fault_prev_active ? "YES" : "no",
                energy_Whr);
}

// ------------------------------------------------------------
// 0x60D Temperature (multiplexed): byte0 bits0-2 = mux index.
//   mux 0: bits3-7 = num sensors; byte1 = min*0.5; byte2 = max*0.5;
//          bytes3-7 = Temp[0..4]
//   mux N: bytes3-7 = Temp[N*5 .. N*5+4]
//   All temps: degC = raw * 0.5 (unsigned byte)
//
// One full temperature sweep arrives across several muxed frames. We fold
// each frame into g_temps[] and, once the final frame of a sweep is in,
// emit the combined monitor block — this is the slowest source of fresh
// data, so it sets the matched output rate for measurement + temperature.
// ------------------------------------------------------------
static void decodeTemperature(const CAN_message_t& m) {
  if (m.len < 8) return;
  uint8_t mux = m.buf[0] & 0x07;

  if (mux == 0) {
    g_temp_num_sensors = m.buf[0] >> 3;
    g_temp_min = m.buf[1] * 0.5f;
    g_temp_max = m.buf[2] * 0.5f;
  }

  uint8_t base = mux * 5;
  for (uint8_t i = 0; i < 5; i++) {
    uint8_t idx = base + i;
    if (idx >= EM_MAX_TEMPS) break;
    g_temps[idx]      = m.buf[3 + i] * 0.5f;
    g_temp_valid[idx] = true;
  }
}

// ============================================================
// CAN RX CALLBACK — only EM IDs reach here thanks to the FIFO filters
// ============================================================
void onEMFrame(const CAN_message_t& m) {
  switch (m.id) {
    case ID_EM_MEASUREMENT: decodeMeasurement(m);  break;
    case ID_EM_STATUS:      decodeStatus(m);        break;
    case ID_EM_TEMPERATURE: decodeTemperature(m);   break;
    default: /* filtered out — should not occur */  break;
  }
}

// ============================================================
// SETUP
// ============================================================
void setup() {
  Serial.begin(USB_BAUD);
  Serial.println("[EM-BRIDGE] Energy Meter CAN -> USB decoder — Teensy 4.1");

  for (uint8_t i = 0; i < EM_MAX_TEMPS; i++) {
    g_temps[i] = NAN;
    g_temp_valid[i] = false;
  }

  // CAN3 @ 500 kbps — same configuration as BMS_Full's CanVehicle bus.
  CanEM.setTX(EM_CAN_TX);
  CanEM.setRX(EM_CAN_RX);
  CanEM.begin();
  CanEM.setBaudRate(EM_CAN_BAUD);
  CanEM.setMaxMB(16);
  CanEM.enableFIFO();
  CanEM.enableFIFOInterrupt();

  // Acceptance filters: reject everything on the shared vehicle bus, then
  // admit ONLY the three Energy-Meter-transmitted 11-bit IDs.
  CanEM.setFIFOFilter(REJECT_ALL);
  CanEM.setFIFOFilter(0, ID_EM_MEASUREMENT, STD);
  CanEM.setFIFOFilter(1, ID_EM_STATUS,      STD);
  CanEM.setFIFOFilter(2, ID_EM_TEMPERATURE, STD);

  CanEM.onReceive(onEMFrame);

  Serial.printf("[EM-BRIDGE] CAN3 @ %lu baud (TX%u/RX%u) — listening for EM IDs 0x%X, 0x%X, 0x%X only\n",
                (unsigned long)EM_CAN_BAUD, EM_CAN_TX, EM_CAN_RX,
                ID_EM_MEASUREMENT, ID_EM_STATUS, ID_EM_TEMPERATURE);
}

// ============================================================
// LOOP
// ============================================================
void loop() {
  CanEM.events();   // service FIFO; onEMFrame() folds in the latest data

  // Print measurement + temperature together at the matched monitor rate.
  if (millis() - g_last_print_ms >= MONITOR_PERIOD_MS) {
    g_last_print_ms = millis();
    printMonitor();
  }
}
