#include <Arduino.h>
#include <FlexCAN_T4.h>

// Teensy 4.1 pinout (same as IMD_Serial_Test):
//   CTX (CAN TX) = pin 30
//   CRX (CAN RX) = pin 31
FlexCAN_T4<CAN3, RX_SIZE_256, TX_SIZE_16> Can3;

// --- Bender iso175 CAN IDs (Bender-Standard defaults) ---
static constexpr uint32_t ID_IMD_INFO_GENERAL = 0x37;
static constexpr uint32_t ID_IMD_REQUEST      = 0x22;
static constexpr uint32_t ID_IMD_RESPONSE     = 0x23;

// --- GET indices (spec D00415_03) ---
static constexpr uint8_t IDX_VOLTAGE_HV_SYSTEM   = 0x5E; // GET: HV system voltage, DataWord LE, unit 0.05 V, offset 32128
static constexpr uint8_t IDX_VOLTAGE_MODE        = 0x64; // GET: 0xFD=AC, 0xFE=DC (default DC)

// Voltage decoding per spec:
//   raw = DataWord (uint16, little-endian), 65535 = SNV
//   voltage_V = (raw - 32128) * 0.05
static float decodeVoltage(uint16_t raw) {
  return ((float)raw - 32128.0f) * 0.05f;
}

// Little-endian 16-bit from buf[n], buf[n+1]
static inline uint16_t u16le(uint8_t lo, uint8_t hi) {
  return (uint16_t)lo | ((uint16_t)hi << 8);
}

// -----------------------------------------------------------------------

void sendGet(uint8_t index) {
  CAN_message_t tx{};
  tx.id     = ID_IMD_REQUEST;
  tx.len    = 8;
  tx.buf[0] = index;
  for (int i = 1; i < 8; i++) tx.buf[i] = 0xFF;
  Can3.write(tx);
}

// -----------------------------------------------------------------------

// IMD_Info_General (0x37) — isolation + status, always active
void handleInfoGeneral(const CAN_message_t &msg) {
  uint16_t r_iso = u16le(msg.buf[0], msg.buf[1]);
  uint8_t  stat  = msg.buf[2];
  uint8_t  warn  = msg.buf[4];

  Serial.print("[0x37] R_iso=");
  if (r_iso == 0xFFFF) Serial.print("SNV");
  else { Serial.print(r_iso); Serial.print(" kOhm"); }

  Serial.print(" | status=0x"); Serial.print(stat, HEX);
  Serial.print(" | warn=0x");   Serial.println(warn, HEX);
}

// IMD_Response (0x23) — responses to our GET requests
void handleResponse(const CAN_message_t &msg) {
  uint8_t index = msg.buf[0];

  if (index == IDX_VOLTAGE_HV_SYSTEM) {
    uint16_t raw = u16le(msg.buf[1], msg.buf[2]);
    Serial.print("[GET 0x5E] HV_System=");
    if (raw == 0xFFFF) Serial.println("SNV");
    else { Serial.print(decodeVoltage(raw), 2); Serial.println(" V"); }

  } else if (index == IDX_VOLTAGE_MODE) {
    Serial.print("[GET 0x64] Voltage_Mode=");
    if      (msg.buf[1] == 0xFD) Serial.println("AC");
    else if (msg.buf[1] == 0xFE) Serial.println("DC");
    else { Serial.print("0x"); Serial.println(msg.buf[1], HEX); }
  }
}

// -----------------------------------------------------------------------

void canSniff(const CAN_message_t &msg) {
  if      (msg.id == ID_IMD_INFO_GENERAL) handleInfoGeneral(msg);
  else if (msg.id == ID_IMD_RESPONSE)     handleResponse(msg);
}

// -----------------------------------------------------------------------

elapsedMillis reqTimer;
static constexpr uint32_t REQ_PERIOD_MS = 500; // poll at 2 Hz

void setup() {
  Serial.begin(115200);
  while (!Serial && millis() < 2000) {}

  Serial.println("IMD Bus Voltage — Teensy 4.1 / CAN3 (pins 30/31)");

  Can3.setRX(31);
  Can3.setTX(30);
  Can3.begin();
  Can3.setBaudRate(500000);
  Can3.setMaxMB(16);
  Can3.enableFIFO();
  Can3.enableFIFOInterrupt();
  Can3.onReceive(canSniff);

  // Verify voltage mode is DC on startup
  delay(500);
  sendGet(IDX_VOLTAGE_MODE);
  Serial.println("Requested Voltage_Mode (expect DC).");
}

void loop() {
  Can3.events();

  if (reqTimer >= REQ_PERIOD_MS) {
    reqTimer = 0;
    sendGet(IDX_VOLTAGE_HV_SYSTEM);
  }
}
