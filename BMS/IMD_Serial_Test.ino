#include <Arduino.h>
#include <FlexCAN_T4.h>

// Teensy 4.1 pins per your setup:
//   CTX (CAN TX) = pin 30
//   CRX (CAN RX) = pin 31
//
// On Teensy 4.1, pins 30/31 map to CAN3 in FlexCAN_T4.
FlexCAN_T4<CAN3, RX_SIZE_256, TX_SIZE_16> Can3;

// --- iso175 (Bender-Standard) default CAN IDs ---
static constexpr uint32_t ID_IMD_INFO_GENERAL = 0x37; // Tx from IMD every 100ms (default)
static constexpr uint32_t ID_IMD_REQUEST      = 0x22; // Request to IMD (default)
static constexpr uint32_t ID_IMD_RESPONSE     = 0x23; // Response from IMD (default)

// --- GET indices (Bender-Standard CAN spec) ---
static constexpr uint8_t IDX_R_ISO_CORRECTED  = 0x4C; // GET: corrected isolation value [kΩ] (DataWord)
static constexpr uint8_t IDX_R_ISO_ORIGINAL   = 0x4E; // GET: original isolation value  [kΩ] (DataWord)
static constexpr uint8_t IDX_R_ISO_STATUS     = 0x44; // GET: R_iso_status
static constexpr uint8_t IDX_DEVICE_ACTIVITY  = 0x68; // GET: Status: Device_Activity
static constexpr uint8_t IDX_WARN_ALARMS      = 0x6C; // GET: Status: Warnings_and_Alarms (bitfield)

// Request pacing note: spec says max request rate <= 10 Hz and wait for response before re-sending.
elapsedMillis reqTimer;
static constexpr uint32_t REQ_PERIOD_MS = 500;  // 2 Hz => safely under 10 Hz

// Simple helper: little-endian 16-bit from two bytes
static inline uint16_t u16le(uint8_t lo, uint8_t hi) {
  return (uint16_t)lo | ((uint16_t)hi << 8);
}

// Print bitfield meaning (Warnings_and_Alarms)
void printWarningsAlarmsBits(uint16_t bits) {
  // Bits per iso175 CAN spec:
  // 0 Device error, 1 HV_pos conn fail, 2 HV_neg conn fail, 3 Earth conn fail,
  // 4 Iso alarm, 5 Iso warning, 6 Iso outdated, 7 Unbalance alarm,
  // 8 Undervoltage, 9 Unsafe to start, 10 Earthlift open
  Serial.print("Warnings/Alarms bits: 0x");
  Serial.print(bits, HEX);

  auto show = [&](uint8_t b, const char* name) {
    if (bits & (1u << b)) {
      Serial.print(" [");
      Serial.print(name);
      Serial.print("]");
    }
  };

  show(0,  "DeviceError");
  show(1,  "HV+ConnFail");
  show(2,  "HV-ConnFail");
  show(3,  "EarthConnFail");
  show(4,  "IsoAlarm");
  show(5,  "IsoWarning");
  show(6,  "IsoOutdated");
  show(7,  "UnbalanceAlarm");
  show(8,  "Undervoltage");
  show(9,  "UnsafeToStart");
  show(10, "EarthliftOpen");
  Serial.println();
}

// Decode the cyclic IMD_Info_General (0x37)
void handleInfoGeneral(const CAN_message_t &msg) {
  // Per spec table:
  // Byte0-1: Isolation R_iso_corrected (neg tolerance shifted) (DataWord)
  // Byte2:   R_iso_status
  // Byte3:   Isolation Measurement counter
  // Byte4:   Status Warnings and alarms (likely bitfield; in cyclic message it's a byte, but treat as LSBs)
  // Byte5:   Status Device activity
  //
  // Note: Some fields are also available as GET indices; this cyclic frame is just a quick comms sanity check.
  uint16_t r_iso_corr_kohm = u16le(msg.buf[0], msg.buf[1]);
  uint8_t  r_iso_status    = msg.buf[2];
  uint8_t  meas_ctr        = msg.buf[3];
  uint8_t  warn_alarms_lsb  = msg.buf[4];
  uint8_t  dev_activity    = msg.buf[5];

  Serial.print("[0x37 IMD_Info_General] R_iso_corrected=");
  if (r_iso_corr_kohm == 0xFFFF) Serial.print("SNV");
  else {
    Serial.print(r_iso_corr_kohm);
    Serial.print(" kOhm");
  }

  Serial.print(" | R_iso_status=0x");
  Serial.print(r_iso_status, HEX);

  Serial.print(" | measCtr=");
  Serial.print(meas_ctr);

  Serial.print(" | warnLSB=0x");
  Serial.print(warn_alarms_lsb, HEX);

  Serial.print(" | devActivity=");
  Serial.println(dev_activity);
}

// Send a GET request (IMD_Request) for one index.
// The IMD allows variable DLC; we’ll send 8 bytes and fill unused with 0xFF (per spec).
void sendGet(uint8_t index) {
  CAN_message_t tx{};
  tx.id  = ID_IMD_REQUEST;
  tx.len = 8;
  tx.buf[0] = index;
  for (int i = 1; i < 8; i++) tx.buf[i] = 0xFF;
  Can3.write(tx);
}

// Handle IMD_Response (0x23)
void handleResponse(const CAN_message_t &msg) {
  uint8_t index = msg.buf[0];

  Serial.print("[0x23 IMD_Response] index=0x");
  Serial.print(index, HEX);
  Serial.print(" data:");

  for (int i = 1; i < 8; i++) {
    Serial.print(" ");
    if (msg.buf[i] < 0x10) Serial.print("0");
    Serial.print(msg.buf[i], HEX);
  }
  Serial.println();

  // Decode a few common ones:
  if (index == IDX_R_ISO_CORRECTED) {
    uint16_t v = u16le(msg.buf[1], msg.buf[2]);
    Serial.print("  -> R_iso_corrected = ");
    if (v == 0xFFFF) Serial.println("SNV");
    else {
      Serial.print(v);
      Serial.println(" kOhm");
    }
  } else if (index == IDX_R_ISO_ORIGINAL) {
    uint16_t v = u16le(msg.buf[1], msg.buf[2]);
    Serial.print("  -> R_iso_original  = ");
    if (v == 0xFFFF) Serial.println("SNV");
    else {
      Serial.print(v);
      Serial.println(" kOhm");
    }
  } else if (index == IDX_R_ISO_STATUS) {
    Serial.print("  -> R_iso_status = 0x");
    Serial.println(msg.buf[1], HEX);
    // 0xFC estimated at startup, 0xFD first measured at startup, 0xFE normal operation (per spec)
  } else if (index == IDX_DEVICE_ACTIVITY) {
    Serial.print("  -> Device_Activity = ");
    Serial.println(msg.buf[1]); // 0 init, 1 normal, 2 self test (per spec)
  } else if (index == IDX_WARN_ALARMS) {
    // Spec defines this as a bitfield; many implementations pack it into 2 bytes (LSB-first).
    // If your IMD returns only 1 byte meaningful, this still works for bits 0..7.
    uint16_t bits = u16le(msg.buf[1], msg.buf[2]);
    printWarningsAlarmsBits(bits);
  }
}

// FlexCAN_T4 receive callback
void canSniff(const CAN_message_t &msg) {
  if (msg.id == ID_IMD_INFO_GENERAL) {
    handleInfoGeneral(msg);
  } else if (msg.id == ID_IMD_RESPONSE) {
    handleResponse(msg);
  } else {
    // Uncomment if you want to see all frames:
    // Serial.printf("[CAN] id=0x%X len=%d\n", msg.id, msg.len);
  }
}

void setup() {
  Serial.begin(115200);
  while (!Serial && millis() < 2000) {}

  Serial.println("Teensy 4.1 iso175 CAN test starting...");

  Can3.setRX(31);           // CRX
  Can3.setTX(30);           // CTX
  Can3.begin();
  Can3.setBaudRate(500000); // iso175 standard config is typically 500 kbit/s

  // Optional but helpful:
  Can3.setMaxMB(16);
  Can3.enableFIFO();
  Can3.enableFIFOInterrupt();
  Can3.onReceive(canSniff);

  Serial.println("CAN3 started @ 500k. Listening for 0x37 (IMD_Info_General)...");
  Serial.println("Also sending periodic GET requests to 0x22 and expecting 0x23 responses.");
}

void loop() {
  // Required when using FIFO + interrupts in FlexCAN_T4
  Can3.events();

  // Periodic GET requests (safe rate)
  if (reqTimer >= REQ_PERIOD_MS) {
    reqTimer = 0;

    // Ask for a few useful values
    sendGet(IDX_DEVICE_ACTIVITY);
    sendGet(IDX_R_ISO_STATUS);
    sendGet(IDX_R_ISO_CORRECTED);
    sendGet(IDX_R_ISO_ORIGINAL);  // <-- added
    sendGet(IDX_WARN_ALARMS);
  }
}