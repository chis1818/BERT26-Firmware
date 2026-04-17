/* BMS_Teensy_CAN_to_ECU_DEBUG.ino
   - Teensy 4.1
   - Uses FlexCAN_T4.h
   - Reads IMD data on CAN3
   - Sends compact status frame to ECU
   - Prints IMD reads and ECU transmissions to Serial for debugging
*/

#include <Arduino.h>
#include <FlexCAN_T4.h>

// Teensy 4.1 CAN3 on pins 30/31
FlexCAN_T4<CAN3, RX_SIZE_256, TX_SIZE_16> Can3;

// ---------------- CAN IDs ----------------
static constexpr uint32_t ID_IMD_INFO_GENERAL = 0x37;   // periodic IMD message
static constexpr uint32_t ID_IMD_REQUEST      = 0x22;   // GET request to IMD
static constexpr uint32_t ID_IMD_RESPONSE     = 0x23;   // GET response from IMD
static constexpr uint32_t BMS_TO_ECU_ID       = 0x100;  // BMS -> ECU status message

// --------------- GET indices --------------
static constexpr uint8_t IDX_R_ISO_CORRECTED  = 0x4C;
static constexpr uint8_t IDX_R_ISO_ORIGINAL   = 0x4E;
static constexpr uint8_t IDX_R_ISO_STATUS     = 0x44;
static constexpr uint8_t IDX_DEVICE_ACTIVITY  = 0x68;
static constexpr uint8_t IDX_WARN_ALARMS      = 0x6C;

// --------------- Timing -------------------
elapsedMillis sendTimer;
elapsedMillis reqTimer;

static constexpr uint32_t SEND_PERIOD_MS = 100;   // BMS -> ECU every 100 ms
static constexpr uint32_t REQ_PERIOD_MS  = 500;   // IMD GET requests every 500 ms

// --------------- State --------------------
volatile uint16_t latestWarnAlarms = 0;
volatile uint8_t  latestDeviceActivity = 0;
volatile uint16_t latestRIsoCorrected = 0xFFFF;
volatile uint16_t latestRIsoOriginal  = 0xFFFF;
volatile uint8_t  latestRIsoStatus    = 0xFF;

volatile bool haveImdData = false;

// --------------- Helpers ------------------
static inline uint16_t u16le(uint8_t lo, uint8_t hi) {
  return (uint16_t)lo | ((uint16_t)hi << 8);
}

static inline uint8_t composeFlags(uint16_t warnbits) {
  // bit0 = any fault/warning present
  return (warnbits != 0) ? 0x01 : 0x00;
}

void printWarningsAlarmsBits(uint16_t bits) {
  Serial.print("    Warnings/Alarms bits = 0x");
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

void printRIsoValue(const char* label, uint16_t value) {
  Serial.print("    ");
  Serial.print(label);
  Serial.print(" = ");
  if (value == 0xFFFF) {
    Serial.println("SNV");
  } else {
    Serial.print(value);
    Serial.println(" kOhm");
  }
}

// --------------- IMD Requests -------------
void sendGet(uint8_t index) {
  CAN_message_t tx{};
  tx.id  = ID_IMD_REQUEST;
  tx.len = 8;
  tx.buf[0] = index;
  for (int i = 1; i < 8; i++) tx.buf[i] = 0xFF;

  Can3.write(tx);

  Serial.print("[BMS -> IMD] Sent GET request, index 0x");
  Serial.println(index, HEX);
}

// --------------- IMD Decoding -------------
void handleInfoGeneral(const CAN_message_t &msg) {
  uint16_t r_iso_corr_kohm = u16le(msg.buf[0], msg.buf[1]);
  uint8_t  r_iso_status    = msg.buf[2];
  uint8_t  meas_ctr        = msg.buf[3];
  uint8_t  warn_alarms_lsb = msg.buf[4];
  uint8_t  dev_activity    = msg.buf[5];

  latestRIsoCorrected  = r_iso_corr_kohm;
  latestRIsoStatus     = r_iso_status;
  latestWarnAlarms     = warn_alarms_lsb; // periodic frame mainly gives LSB
  latestDeviceActivity = dev_activity;
  haveImdData = true;

  Serial.println("[IMD -> BMS] IMD_Info_General (0x37)");
  printRIsoValue("R_iso_corrected", r_iso_corr_kohm);

  Serial.print("    R_iso_status = 0x");
  Serial.println(r_iso_status, HEX);

  Serial.print("    Measurement Counter = ");
  Serial.println(meas_ctr);

  Serial.print("    Device Activity = ");
  Serial.println(dev_activity);

  printWarningsAlarmsBits(latestWarnAlarms);
}

void handleResponse(const CAN_message_t &msg) {
  uint8_t index = msg.buf[0];

  Serial.print("[IMD -> BMS] IMD_Response (0x23), index 0x");
  Serial.println(index, HEX);

  Serial.print("    Raw data:");
  for (int i = 1; i < msg.len; i++) {
    Serial.print(" ");
    if (msg.buf[i] < 0x10) Serial.print("0");
    Serial.print(msg.buf[i], HEX);
  }
  Serial.println();

  if (index == IDX_R_ISO_CORRECTED) {
    latestRIsoCorrected = u16le(msg.buf[1], msg.buf[2]);
    printRIsoValue("R_iso_corrected", latestRIsoCorrected);
    haveImdData = true;
  }
  else if (index == IDX_R_ISO_ORIGINAL) {
    latestRIsoOriginal = u16le(msg.buf[1], msg.buf[2]);
    printRIsoValue("R_iso_original", latestRIsoOriginal);
    haveImdData = true;
  }
  else if (index == IDX_R_ISO_STATUS) {
    latestRIsoStatus = msg.buf[1];
    Serial.print("    R_iso_status = 0x");
    Serial.println(latestRIsoStatus, HEX);
    haveImdData = true;
  }
  else if (index == IDX_DEVICE_ACTIVITY) {
    latestDeviceActivity = msg.buf[1];
    Serial.print("    Device Activity = ");
    Serial.println(latestDeviceActivity);
    haveImdData = true;
  }
  else if (index == IDX_WARN_ALARMS) {
    latestWarnAlarms = u16le(msg.buf[1], msg.buf[2]);
    printWarningsAlarmsBits(latestWarnAlarms);
    haveImdData = true;
  }
}

// --------------- ECU Send -----------------
void sendStatusToEcu() {
  CAN_message_t tx{};
  tx.id = BMS_TO_ECU_ID;
  tx.len = 8;

  uint8_t flags = composeFlags(latestWarnAlarms);

  tx.buf[0] = flags;
  tx.buf[1] = (uint8_t)(latestWarnAlarms & 0xFF);
  tx.buf[2] = (uint8_t)((latestWarnAlarms >> 8) & 0xFF);
  tx.buf[3] = latestDeviceActivity;
  tx.buf[4] = (uint8_t)(latestRIsoCorrected & 0xFF);
  tx.buf[5] = (uint8_t)((latestRIsoCorrected >> 8) & 0xFF);
  tx.buf[6] = latestRIsoStatus;
  tx.buf[7] = 0xFF;

  Can3.write(tx);

  Serial.println("[BMS -> ECU] Sent status frame");
  Serial.print("    CAN ID = 0x");
  Serial.println(BMS_TO_ECU_ID, HEX);

  Serial.print("    flags = 0x");
  Serial.println(flags, HEX);

  Serial.print("    latestWarnAlarms = 0x");
  Serial.println(latestWarnAlarms, HEX);

  Serial.print("    latestDeviceActivity = ");
  Serial.println(latestDeviceActivity);

  printRIsoValue("latestR_iso_corrected", latestRIsoCorrected);

  Serial.print("    latestR_iso_status = 0x");
  Serial.println(latestRIsoStatus, HEX);

  Serial.print("    TX bytes: ");
  for (int i = 0; i < tx.len; i++) {
    if (tx.buf[i] < 0x10) Serial.print("0");
    Serial.print(tx.buf[i], HEX);
    Serial.print(" ");
  }
  Serial.println();
  Serial.println();
}

// --------------- CAN Callback -------------
void canRx(const CAN_message_t &msg) {
  if (msg.id == ID_IMD_INFO_GENERAL) {
    handleInfoGeneral(msg);
  }
  else if (msg.id == ID_IMD_RESPONSE) {
    handleResponse(msg);
  }
  else {
    // Uncomment to print every CAN frame seen:
    /*
    Serial.print("[CAN] id=0x");
    Serial.print(msg.id, HEX);
    Serial.print(" len=");
    Serial.println(msg.len);
    */
  }
}

void setup() {
  Serial.begin(115200);
  while (!Serial && millis() < 2000) {}

  Serial.println("BMS Teensy IMD debug starting...");

  Can3.setRX(31);
  Can3.setTX(30);
  Can3.begin();
  Can3.setBaudRate(500000);

  Can3.setMaxMB(16);
  Can3.enableFIFO();
  Can3.enableFIFOInterrupt();
  Can3.onReceive(canRx);

  Serial.println("CAN3 started @ 500k");
  Serial.println("Listening for IMD messages and forwarding status to ECU");
  Serial.println();
}

void loop() {
  Can3.events();

  // Periodically request useful IMD values
  if (reqTimer >= REQ_PERIOD_MS) {
    reqTimer = 0;
    sendGet(IDX_DEVICE_ACTIVITY);
    sendGet(IDX_R_ISO_STATUS);
    sendGet(IDX_R_ISO_CORRECTED);
    sendGet(IDX_R_ISO_ORIGINAL);
    sendGet(IDX_WARN_ALARMS);
    Serial.println();
  }

  // Periodically send summary to ECU
  if (sendTimer >= SEND_PERIOD_MS) {
    sendTimer = 0;
    if (haveImdData) {
      sendStatusToEcu();
    } else {
      Serial.println("[BMS -> ECU] No IMD data yet, nothing meaningful to send");
      Serial.println();
    }
  }
}