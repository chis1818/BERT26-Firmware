/*
  Teensy 4.0 <-> TI BQ796xx (1x bq79600 + N-1 x bq79616)

  Stack model:
    - Device 0: bq79600 base interface
    - Devices 1..(N-1): bq79616

  Added in this version:
    - Cell balancing support for bq79616
    - Raw TX frame logging
    - Raw RX frame logging
    - Write + readback verification helpers
    - BAL_STAT polling
    - USB command: "bal" to start a simple balancing test
    - USB command: "off" to send shutdown tone

  Thermistor wiring (same on each bq79616):
    MUX0: EN GPIO4, OUT GPIO5
    MUX1: EN GPIO6, OUT GPIO7
    A0/A1/A2 for both muxes: GPIO1 / GPIO2 / GPIO3
*/

#include <Arduino.h>
#include <math.h>

// ---------------- User config ----------------

// Total devices INCLUDING the bq79600 at id0.
// Example: 1x bq79600 + 6x bq79616 => NUM_TOTAL_DEV = 7
static const uint8_t NUM_TOTAL_DEV = 2;

static const uint8_t FIRST_BQ79616_ID = 1;

// Print interval (ms) for VCELL + DIETEMP
static const uint32_t PRINT_INTERVAL_MS = 1500;

// NTC scanning control
static const bool     ENABLE_NTC_SCANS     = true;
static const uint32_t NTC_SCAN_INTERVAL_MS = 5000;

// Frame logging control
static const bool LOG_ALL_FRAMES      = false; // very verbose if true
static const bool LOG_BALANCE_FRAMES  = true;  // recommended true

// Balancing test control
static const bool    ENABLE_BAL_TEST_AT_BOOT = true;
static const uint8_t BAL_TEST_DEVICE_ID      = 1;    // choose a bq79616
static const uint8_t BAL_TEST_CELL           = 2;    // cell to balance
static const uint8_t BAL_TEST_TIMER_CODE     = 0x04; // 0x01=10s, 0x02=30s, 0x03=60s
static const uint8_t BAL_TEST_DUTY_CODE      = 0x00; // 0x0=5s, 0x1=10s, 0x2=30s, ...
static const bool    BAL_TEST_AUTO_MODE      = true;
static const bool    BAL_TEST_FLTSTOP_EN     = false;
static const bool    BAL_TEST_OTCB_EN        = false;

// ---------------- Hardware ----------------

static const uint32_t UART_BAUD   = 1000000;
static const uint32_t USB_WAIT_MS = 8000;

HardwareSerial &BMS = Serial1;
const int LED_PIN = 13;

// ---------------- Registers ----------------

#define ACTIVE_CELL      0x0003
#define ADC_CONF1        0x0007
#define GPIO_CONF1       0x000E
#define GPIO_CONF2       0x000F
#define GPIO_CONF3       0x0010
#define GPIO_CONF4       0x0011

#define DIR0_ADDR        0x0306
#define COMM_CTRL        0x0308
#define CONTROL1         0x0309
#define CONTROL2         0x030A
#define ADC_CTRL1        0x030D

#define OTP_ECC_DATAIN1  0x0343
#define OTP_ECC_DATAIN8  0x034A

// ---- Cell balancing registers ----
#define CB_CELL16_CTRL   0x0318
#define VMB_DONE_THRESH  0x0328
#define BAL_TIME         0x0329
#define VCB_DONE_THRESH  0x032A
#define OTCB_THRESH      0x032B
#define BAL_CTRL1        0x032E
#define BAL_CTRL2        0x032F
#define BAL_STAT         0x052B

#define PARTID           0x0500
#define DEV_STAT         0x052C

#define VCELL16_HI       0x0568
#define TSREF_HI         0x058C

#define GPIO1_HI         0x058E
#define GPIO2_HI         0x0590
#define GPIO3_HI         0x0592
#define GPIO4_HI         0x0594
#define GPIO5_HI         0x0596
#define GPIO6_HI         0x0598
#define GPIO7_HI         0x059A
#define GPIO8_HI         0x059C

#define DIETEMP1_HI      0x05AE

static inline uint16_t gpio_hi_reg(uint8_t gpio) {
  return (uint16_t)(GPIO1_HI + 2u * (uint16_t)(gpio - 1u));
}

// ---------------- GPIO mode encodings ----------------

static const uint8_t GPIO_MODE_DISABLE  = 0b000;
static const uint8_t GPIO_MODE_ADC_ONLY = 0b010;
static const uint8_t GPIO_MODE_OUT_HIGH = 0b100;
static const uint8_t GPIO_MODE_OUT_LOW  = 0b101;

// ---------------- NTC params ----------------

static const float NTC_R25   = 10000.0f;
static const float NTC_BETA  = 3380.0f;
static const float T0_KELVIN = 273.15f + 25.0f;

// ---------------- Frame types ----------------

enum : uint8_t { FRMWRT_SGL_W = 0x90, FRMWRT_STK_W = 0xB0, FRMWRT_ALL_W = 0xD0 };
enum : uint8_t { FRMWRT_SGL_R = 0x00, FRMWRT_STK_R = 0x20, FRMWRT_ALL_R = 0x40 };

// ---------------- BAL_CTRL2 bits ----------------

static const uint8_t BAL2_AUTO_BAL    = 0x01;
static const uint8_t BAL2_BAL_GO      = 0x02;
static const uint8_t BAL2_OTCB_EN     = 0x10;
static const uint8_t BAL2_FLTSTOP_EN  = 0x20;
static const uint8_t BAL2_CB_PAUSE    = 0x40;

// ---------------- BAL_STAT bits ----------------

static const uint8_t BALSTAT_CB_DONE        = 0x01;
static const uint8_t BALSTAT_MB_DONE        = 0x02;
static const uint8_t BALSTAT_ABORTFLT       = 0x04;
static const uint8_t BALSTAT_CB_RUN         = 0x08;
static const uint8_t BALSTAT_MB_RUN         = 0x10;
static const uint8_t BALSTAT_CB_INPAUSE     = 0x20;
static const uint8_t BALSTAT_OT_PAUSE_DET   = 0x40;
static const uint8_t BALSTAT_INVALID_CBCONF = 0x80;

// ---------------- Small helpers ----------------

static inline uint8_t top_id() {
  return (NUM_TOTAL_DEV == 0) ? 0 : (uint8_t)(NUM_TOTAL_DEV - 1);
}

static bool is_bq79616_id(uint8_t id) {
  return (id >= FIRST_BQ79616_ID) && (id < NUM_TOTAL_DEV);
}

// ---------------- CRC16 IBM/Modbus (0xA001), LSB-first ----------------

static uint16_t crc16_ibm(const uint8_t *data, size_t len) {
  uint16_t crc = 0xFFFF;
  for (size_t i = 0; i < len; ++i) {
    crc ^= data[i];
    for (int b = 0; b < 8; ++b) {
      crc = (crc & 1) ? (crc >> 1) ^ 0xA001 : (crc >> 1);
    }
  }
  return crc;
}

// ---------------- Raw frame builders ----------------

static size_t buildWriteFrameBytes(uint8_t wrType, uint8_t devAddr, uint16_t reg,
                                   const uint8_t *data, uint8_t n,
                                   uint8_t *out, size_t outMax) {
  if (!out) return 0;

  const size_t needed = (size_t)(1 + ((wrType == FRMWRT_SGL_W) ? 1 : 0) + 2 + n + 2);
  if (outMax < needed) return 0;

  const uint8_t hdr = wrType + (n - 1);
  size_t k = 0;

  out[k++] = 0x80 | hdr;
  if (wrType == FRMWRT_SGL_W) out[k++] = devAddr;
  out[k++] = (uint8_t)(reg >> 8);
  out[k++] = (uint8_t)(reg & 0xFF);

  for (uint8_t i = 0; i < n; ++i) {
    out[k++] = data[i];
  }

  uint16_t c = crc16_ibm(out, k);
  out[k++] = (uint8_t)(c & 0xFF);
  out[k++] = (uint8_t)(c >> 8);

  return k;
}

static size_t buildReadFrameBytes(uint8_t rdType, uint8_t devAddr, uint16_t reg,
                                  uint8_t nBytes,
                                  uint8_t *out, size_t outMax) {
  if (!out) return 0;

  const size_t needed = (size_t)(1 + ((rdType == FRMWRT_SGL_R) ? 1 : 0) + 2 + 1 + 2);
  if (outMax < needed) return 0;

  size_t k = 0;

  out[k++] = 0x80 | rdType;
  if (rdType == FRMWRT_SGL_R) out[k++] = devAddr;
  out[k++] = (uint8_t)(reg >> 8);
  out[k++] = (uint8_t)(reg & 0xFF);
  out[k++] = (uint8_t)(nBytes - 1);

  uint16_t c = crc16_ibm(out, k);
  out[k++] = (uint8_t)(c & 0xFF);
  out[k++] = (uint8_t)(c >> 8);

  return k;
}

static void printHexLine(const char *prefix, const uint8_t *buf, size_t n) {
  Serial.print(prefix);
  for (size_t i = 0; i < n; ++i) {
    Serial.printf("%02X", buf[i]);
    if (i + 1 < n) Serial.print(" ");
  }
  Serial.println();
}

static void dumpHex(const uint8_t *p, size_t n) {
  for (size_t i = 0; i < n; ++i) {
    Serial.printf("%02X%s", p[i], (i + 1 < n) ? " " : "\n");
  }
}

static void drainBMSRx(void) {
  while (BMS.available()) {
    (void)BMS.read();
  }
}

static size_t sniffReply(uint8_t *dst, size_t maxLen, uint32_t timeout_ms) {
  uint32_t t0 = millis();
  size_t got = 0;

  while ((millis() - t0) < timeout_ms && got < maxLen) {
    while (BMS.available() && got < maxLen) {
      dst[got++] = (uint8_t)BMS.read();
    }
  }
  return got;
}

// ---------------- UART framing ----------------

static void writeFrame(uint8_t wrType, uint8_t devAddr, uint16_t reg, const uint8_t *data, uint8_t n) {
  uint8_t buf[32] = {0};
  size_t len = buildWriteFrameBytes(wrType, devAddr, reg, data, n, buf, sizeof(buf));
  if (len == 0) return;

  if (LOG_ALL_FRAMES) {
    Serial.printf("[TX-W] type=0x%02X dev=%u reg=0x%04X n=%u\n", wrType, devAddr, reg, n);
    printHexLine("  TX: ", buf, len);
  }

  BMS.write(buf, len);
  BMS.flush();
}

static void readFrame(uint8_t rdType, uint8_t devAddr, uint16_t reg, uint8_t nBytes) {
  uint8_t buf[16] = {0};
  size_t len = buildReadFrameBytes(rdType, devAddr, reg, nBytes, buf, sizeof(buf));
  if (len == 0) return;

  if (LOG_ALL_FRAMES) {
    Serial.printf("[TX-R] type=0x%02X dev=%u reg=0x%04X n=%u\n", rdType, devAddr, reg, nBytes);
    printHexLine("  TX: ", buf, len);
  }

  BMS.write(buf, len);
  BMS.flush();
}

static bool readExact(uint8_t *dst, size_t n, uint32_t to_ms) {
  uint32_t t0 = millis();
  size_t got = 0;

  while (got < n && (millis() - t0) < to_ms) {
    while (BMS.available() && got < n) {
      dst[got++] = (uint8_t)BMS.read();
    }
    if (got < n) delay(0);
  }
  return got == n;
}

// ---------------- TX1 as GPIO for tones ----------------

static void uartPing_us(uint32_t low_us) {
  BMS.end();
  pinMode(1, OUTPUT);
  digitalWrite(1, LOW);
  delayMicroseconds(low_us);
  digitalWrite(1, HIGH);
  BMS.begin(UART_BAUD, SERIAL_8N1);
  delayMicroseconds(100);
}

static void bq_hwrst() { uartPing_us(36000); delay(100); }
static void bq_wake()  { uartPing_us(2500); }
static void bq_sta()   { uartPing_us(250);  }

// ---------------- Conversions ----------------

static float raw_to_V(uint16_t raw) { return (int16_t)raw * 190.73e-6f; }
static float raw_to_C(int16_t raw)  { return 0.025f * (float)raw; }

// ---------------- Logged read/write helpers ----------------

static bool write_u8_logged(uint8_t addr, uint16_t reg, uint8_t val, const char *label, bool forceLog = false) {
  uint8_t tx[16] = {0};
  uint8_t rx[32] = {0};

  size_t txLen = buildWriteFrameBytes(FRMWRT_SGL_W, addr, reg, &val, 1, tx, sizeof(tx));
  if (txLen == 0) return false;

  if (LOG_ALL_FRAMES || forceLog) {
    Serial.printf("[WR:%s] dev=%u reg=0x%04X val=0x%02X\n", label, addr, reg, val);
    printHexLine("  TX: ", tx, txLen);
  }

  drainBMSRx();
  BMS.write(tx, txLen);
  BMS.flush();

  size_t got = sniffReply(rx, sizeof(rx), 10);
  if ((LOG_ALL_FRAMES || forceLog)) {
    if (got > 0) printHexLine("  RX: ", rx, got);
    else Serial.println("  RX: <none>");
  }

  return true;
}

static bool read_u8_logged(uint8_t addr, uint16_t reg, uint8_t &val, const char *label, bool forceLog = false) {
  uint8_t tx[16] = {0};
  uint8_t rx[7]  = {0};

  size_t txLen = buildReadFrameBytes(FRMWRT_SGL_R, addr, reg, 1, tx, sizeof(tx));
  if (txLen == 0) return false;

  if (LOG_ALL_FRAMES || forceLog) {
    Serial.printf("[RD:%s] dev=%u reg=0x%04X n=1\n", label, addr, reg);
    printHexLine("  TX: ", tx, txLen);
  }

  drainBMSRx();
  BMS.write(tx, txLen);
  BMS.flush();

  if (!readExact(rx, sizeof(rx), 150)) {
    if (LOG_ALL_FRAMES || forceLog) Serial.println("  RX: <timeout>");
    return false;
  }

  if (LOG_ALL_FRAMES || forceLog) {
    printHexLine("  RX: ", rx, sizeof(rx));
  }

  val = rx[4];
  return true;
}

static bool read_u16_logged(uint8_t addr, uint16_t reg, uint16_t &val, const char *label, bool forceLog = false) {
  uint8_t tx[16] = {0};
  uint8_t rx[8]  = {0};

  size_t txLen = buildReadFrameBytes(FRMWRT_SGL_R, addr, reg, 2, tx, sizeof(tx));
  if (txLen == 0) return false;

  if (LOG_ALL_FRAMES || forceLog) {
    Serial.printf("[RD:%s] dev=%u reg=0x%04X n=2\n", label, addr, reg);
    printHexLine("  TX: ", tx, txLen);
  }

  drainBMSRx();
  BMS.write(tx, txLen);
  BMS.flush();

  if (!readExact(rx, sizeof(rx), 150)) {
    if (LOG_ALL_FRAMES || forceLog) Serial.println("  RX: <timeout>");
    return false;
  }

  if (LOG_ALL_FRAMES || forceLog) {
    printHexLine("  RX: ", rx, sizeof(rx));
  }

  val = ((uint16_t)rx[4] << 8) | rx[5];
  return true;
}

static bool write_verify_u8(uint8_t addr, uint16_t reg, uint8_t val, const char *label, bool forceLog = false) {
  if (!write_u8_logged(addr, reg, val, label, forceLog)) return false;

  uint8_t rb = 0;
  if (!read_u8_logged(addr, reg, rb, label, forceLog)) {
    Serial.printf("  VERIFY: readback failed for %s\n", label);
    return false;
  }

  if (LOG_ALL_FRAMES || forceLog) {
    Serial.printf("  VERIFY: wrote 0x%02X, readback 0x%02X -> %s\n",
                  val, rb, (rb == val) ? "OK" : "MISMATCH");
  }

  return (rb == val);
}

// ---------------- Read helpers ----------------

static bool read_u16(uint8_t addr, uint16_t reg_hi, uint16_t &val) {
  return read_u16_logged(addr, reg_hi, val, "U16", false);
}

static bool read_u8(uint8_t addr, uint16_t reg, uint8_t &val) {
  return read_u8_logged(addr, reg, val, "U8", false);
}

static bool read_vcell_id(uint8_t id, float outV[16]) {
  uint8_t tx[16]      = {0};
  uint8_t rx[32 + 6]  = {0};

  size_t txLen = buildReadFrameBytes(FRMWRT_SGL_R, id, VCELL16_HI, 32, tx, sizeof(tx));
  if (txLen == 0) return false;

  if (LOG_ALL_FRAMES) {
    Serial.printf("[RD:VCELL] dev=%u reg=0x%04X n=32\n", id, VCELL16_HI);
    printHexLine("  TX: ", tx, txLen);
  }

  drainBMSRx();
  BMS.write(tx, txLen);
  BMS.flush();

  if (!readExact(rx, sizeof(rx), 300)) {
    if (LOG_ALL_FRAMES) Serial.println("  RX: <timeout>");
    return false;
  }

  if (LOG_ALL_FRAMES) {
    printHexLine("  RX: ", rx, sizeof(rx));
  }

  for (int i = 0; i < 16; ++i) {
    uint16_t raw = ((uint16_t)rx[4 + i * 2] << 8) | rx[4 + i * 2 + 1];
    int cell = 16 - i;
    outV[cell - 1] = raw_to_V(raw);
  }

  return true;
}

static bool read_die_temp_id(uint8_t id, float &tC1) {
  uint16_t r1 = 0;
  if (!read_u16_logged(id, DIETEMP1_HI, r1, "DIETEMP1", false)) return false;
  tC1 = raw_to_C((int16_t)r1);
  return true;
}

static bool read_id_status_id(uint8_t id) {
  uint16_t pid = 0;
  if (read_u16_logged(id, PARTID, pid, "PARTID", true)) {
    Serial.printf("[id%d] PARTID=0x%04X\n", id, pid);
  } else {
    Serial.printf("[id%d] PARTID timeout\n", id);
  }

  uint8_t st = 0;
  if (read_u8_logged(id, DEV_STAT, st, "DEV_STAT", true)) {
    Serial.printf("[st%d] DEV_STAT=0x%02X\n", id, st);
    return true;
  } else {
    Serial.printf("[st%d] DEV_STAT timeout\n", id);
    return false;
  }
}

// ---------------- ADC start (broadcast) ----------------

static void adc_start_all() {
  { uint8_t v = 0x0A; writeFrame(FRMWRT_ALL_W, 0, ACTIVE_CELL, &v, 1); }
  { uint8_t v = 0x02; writeFrame(FRMWRT_ALL_W, 0, ADC_CONF1,  &v, 1); }
  { uint8_t v = 0x0E; writeFrame(FRMWRT_ALL_W, 0, ADC_CTRL1,  &v, 1); }
  delayMicroseconds(40000);
}

// ---------------- Auto-address ----------------

static void autoAddress_fixed() {
  for (uint16_t r = OTP_ECC_DATAIN1; r <= OTP_ECC_DATAIN8; ++r) {
    uint8_t z = 0x00;
    writeFrame(FRMWRT_STK_W, 0, r, &z, 1);
  }

  { uint8_t c1 = 0x01; writeFrame(FRMWRT_ALL_W, 0, CONTROL1, &c1, 1); }

  for (uint8_t a = 0; a < NUM_TOTAL_DEV; ++a) {
    writeFrame(FRMWRT_ALL_W, 0, DIR0_ADDR, &a, 1);
  }

  { uint8_t v = 0x02; writeFrame(FRMWRT_ALL_W, 0, COMM_CTRL, &v, 1); }
  { uint8_t v = 0x03; writeFrame(FRMWRT_SGL_W, top_id(), COMM_CTRL, &v, 1); }

  for (uint16_t r = OTP_ECC_DATAIN1; r <= OTP_ECC_DATAIN8; ++r) {
    uint8_t z = 0x00;
    writeFrame(FRMWRT_STK_W, 0, r, &z, 1);
  }
}

// ---------------- TSREF + NTC ----------------

static bool read_tsref_code_id(uint8_t id, int16_t &code) {
  uint16_t v = 0;
  if (!read_u16(id, TSREF_HI, v)) return false;
  code = (int16_t)v;
  if (code == (int16_t)0x8000 || code <= 0) return false;
  return true;
}

static bool read_gpio_code_id(uint8_t id, uint8_t gpio, int16_t &code) {
  if (gpio < 1 || gpio > 8) return false;

  uint16_t reg = gpio_hi_reg(gpio);
  uint16_t v = 0;
  if (!read_u16(id, reg, v)) return false;

  code = (int16_t)v;
  if (code == (int16_t)0x8000) return false;
  return true;
}

static bool ntc_from_adc_codes(int16_t gpioCode, int16_t tsrefCode, float &tC) {
  if (tsrefCode <= 0) return false;

  float ratio = (float)gpioCode / (float)tsrefCode;
  if (ratio <= 0.0f || ratio >= 0.999f) return false;

  const float Rpull = NTC_R25;
  float Rntc = Rpull * ratio / (1.0f - ratio);

  float lnR   = logf(Rntc / NTC_R25);
  float invT  = (1.0f / T0_KELVIN) + (lnR / NTC_BETA);
  float Tkelv = 1.0f / invT;
  tC = Tkelv - 273.15f;
  return true;
}

static bool enable_tsref_device(uint8_t id) {
  uint8_t c2_old = 0;
  if (!read_u8_logged(id, CONTROL2, c2_old, "CONTROL2", true)) {
    Serial.printf("[tsref] read CONTROL2 failed for id%d\n", id);
    return false;
  }

  uint8_t c2_new = c2_old | 0x01;
  if (!write_verify_u8(id, CONTROL2, c2_new, "CONTROL2", true)) {
    Serial.printf("[tsref] write/verify failed for id%d\n", id);
    return false;
  }

  delayMicroseconds(2000);
  return true;
}

// Set mux channel on a given bq79616 device
static void set_mux_channel(uint8_t devId, uint8_t muxIndex, uint8_t chan) {
  chan &= 0x7;

  bool a0 = chan & 0x1;
  bool a1 = chan & 0x2;
  bool a2 = chan & 0x4;

  uint8_t mode_g1 = a0 ? GPIO_MODE_OUT_HIGH : GPIO_MODE_OUT_LOW;
  uint8_t mode_g2 = a1 ? GPIO_MODE_OUT_HIGH : GPIO_MODE_OUT_LOW;
  uint8_t mode_g3 = a2 ? GPIO_MODE_OUT_HIGH : GPIO_MODE_OUT_LOW;

  uint8_t mode_g4;
  uint8_t mode_g6;

  if (muxIndex == 0) {
    mode_g4 = GPIO_MODE_OUT_HIGH;
    mode_g6 = GPIO_MODE_DISABLE;
  } else {
    mode_g4 = GPIO_MODE_DISABLE;
    mode_g6 = GPIO_MODE_OUT_HIGH;
  }

  uint8_t mode_g5 = GPIO_MODE_ADC_ONLY;
  uint8_t mode_g7 = GPIO_MODE_ADC_ONLY;
  uint8_t mode_g8 = GPIO_MODE_DISABLE;

  uint8_t conf1 = (uint8_t)((mode_g2 << 3) | mode_g1);
  uint8_t conf2 = (uint8_t)((mode_g4 << 3) | mode_g3);
  uint8_t conf3 = (uint8_t)((mode_g6 << 3) | mode_g5);
  uint8_t conf4 = (uint8_t)((mode_g8 << 3) | mode_g7);

  writeFrame(FRMWRT_SGL_W, devId, GPIO_CONF1, &conf1, 1);
  writeFrame(FRMWRT_SGL_W, devId, GPIO_CONF2, &conf2, 1);
  writeFrame(FRMWRT_SGL_W, devId, GPIO_CONF3, &conf3, 1);
  writeFrame(FRMWRT_SGL_W, devId, GPIO_CONF4, &conf4, 1);

  delayMicroseconds(2500);
  adc_start_all();
}

static void read_all_ntc_device(uint8_t devId) {
  int16_t tsrefCode = 0;
  if (!read_tsref_code_id(devId, tsrefCode)) {
    Serial.printf("[ntc:id%u] TSREF read failed\n", devId);
    return;
  }

  for (uint8_t cell = 1; cell <= 16; ++cell) {
    uint8_t muxIndex = (cell <= 8) ? 0 : 1;
    uint8_t chan     = (uint8_t)((cell - 1) & 0x7);

    set_mux_channel(devId, muxIndex, chan);

    uint8_t senseGpio = (muxIndex == 0) ? 5 : 7;
    int16_t gpioCode  = 0;

    if (!read_gpio_code_id(devId, senseGpio, gpioCode)) {
      Serial.printf("[ntc:id%u] Cell%02u: GPIO%u read failed\n", devId, cell, senseGpio);
      continue;
    }

    float tC = 0.0f;
    if (!ntc_from_adc_codes(gpioCode, tsrefCode, tC)) {
      Serial.printf("[ntc:id%u] Cell%02u: conversion failed (GPIO=%d, TSREF=%d)\n",
                    devId, cell, (int)gpioCode, (int)tsrefCode);
      continue;
    }

    Serial.printf("[ntc:id%u] Cell%02u: %.2f C\n", devId, cell, tC);
  }
}

// ---------------- Cell balancing helpers ----------------

// Maps cell number 1..16 to register 0x0327..0x0318
static uint16_t cb_cell_reg(uint8_t cell) {
  if (cell < 1 || cell > 16) return 0;
  return (uint16_t)(CB_CELL16_CTRL + (16u - cell));
}

static void cb_print_status(uint8_t balStat) {
  Serial.printf("[BAL_STAT] 0x%02X\n", balStat);
  Serial.printf("  INVALID_CBCONF : %u\n", (balStat & BALSTAT_INVALID_CBCONF) ? 1 : 0);
  Serial.printf("  OT_PAUSE_DET   : %u\n", (balStat & BALSTAT_OT_PAUSE_DET)   ? 1 : 0);
  Serial.printf("  CB_INPAUSE     : %u\n", (balStat & BALSTAT_CB_INPAUSE)     ? 1 : 0);
  Serial.printf("  MB_RUN         : %u\n", (balStat & BALSTAT_MB_RUN)         ? 1 : 0);
  Serial.printf("  CB_RUN         : %u\n", (balStat & BALSTAT_CB_RUN)         ? 1 : 0);
  Serial.printf("  ABORTFLT       : %u\n", (balStat & BALSTAT_ABORTFLT)       ? 1 : 0);
  Serial.printf("  MB_DONE        : %u\n", (balStat & BALSTAT_MB_DONE)        ? 1 : 0);
  Serial.printf("  CB_DONE        : %u\n", (balStat & BALSTAT_CB_DONE)        ? 1 : 0);
}

static bool cb_read_status(uint8_t devAddr, uint8_t &balStat) {
  return read_u8_logged(devAddr, BAL_STAT, balStat, "BAL_STAT", LOG_BALANCE_FRAMES);
}

static bool cb_set_timer(uint8_t devAddr, uint8_t cell, uint8_t timeCode) {
  uint16_t reg = cb_cell_reg(cell);
  if (reg == 0) return false;

  char label[24];
  snprintf(label, sizeof(label), "CB_CELL%u_CTRL", cell);

  uint8_t v = (uint8_t)(timeCode & 0x1F);
  return write_verify_u8(devAddr, reg, v, label, LOG_BALANCE_FRAMES);
}

static bool cb_set_vcb_done_thresh(uint8_t devAddr, uint8_t thrCode) {
  uint8_t v = (uint8_t)(thrCode & 0x3F);
  return write_verify_u8(devAddr, VCB_DONE_THRESH, v, "VCB_DONE_THRESH", LOG_BALANCE_FRAMES);
}

static bool cb_set_ctrl1(uint8_t devAddr, uint8_t dutyCode) {
  uint8_t v = (uint8_t)(dutyCode & 0x07);
  return write_verify_u8(devAddr, BAL_CTRL1, v, "BAL_CTRL1", LOG_BALANCE_FRAMES);
}

static bool cb_start(uint8_t devAddr,
                     bool autoBal,
                     bool fltStopEn,
                     bool otcbEn) {
  uint8_t v = 0x00;

  if (autoBal)   v |= BAL2_AUTO_BAL;
  v |= BAL2_BAL_GO; // self-clearing
  if (otcbEn)    v |= BAL2_OTCB_EN;
  if (fltStopEn) v |= BAL2_FLTSTOP_EN;

  if (!write_u8_logged(devAddr, BAL_CTRL2, v, "BAL_CTRL2", LOG_BALANCE_FRAMES)) return false;

  uint8_t rb = 0;
  if (!read_u8_logged(devAddr, BAL_CTRL2, rb, "BAL_CTRL2", LOG_BALANCE_FRAMES)) return false;

  Serial.printf("  NOTE: BAL_GO is self-clearing, BAL_CTRL2 readback = 0x%02X\n", rb);
  return true;
}

static bool cb_test_one_cell(uint8_t devAddr, uint8_t cell) {
  if (!is_bq79616_id(devAddr)) {
    Serial.printf("[bal] id%u is not a bq79616\n", devAddr);
    return false;
  }

  Serial.printf("\n[bal] Configure dev=%u cell=%u\n", devAddr, cell);

  // Disable voltage-based stop for this first test
  if (!cb_set_vcb_done_thresh(devAddr, 0x00)) return false;

  // Set odd/even duty-cycle timing used by AUTO_BAL
  if (!cb_set_ctrl1(devAddr, BAL_TEST_DUTY_CODE)) return false;

  // Arm one channel with a non-zero timer
  if (!cb_set_timer(devAddr, cell, BAL_TEST_TIMER_CODE)) return false;

  // Start balancing
  if (!cb_start(devAddr, BAL_TEST_AUTO_MODE, BAL_TEST_FLTSTOP_EN, BAL_TEST_OTCB_EN)) return false;

  uint8_t st = 0;
  if (!cb_read_status(devAddr, st)) return false;
  cb_print_status(st);

  return true;
}

// ---------------- USB commands ----------------

static void bq_send_off_tone() {
  uint8_t v = 0x40; // CONTROL1 bit6
  writeFrame(FRMWRT_ALL_W, 0, CONTROL1, &v, 1);
  Serial.println("[cmd] OFF: CONTROL1[SEND_SHUTDOWN]=1 -> SHUTDOWN tone sent up the stack");
}

static void handle_usb_command() {
  static char buf[32];
  static uint8_t idx = 0;

  while (Serial.available()) {
    char c = (char)Serial.read();

    if (c == '\r' || c == '\n') {
      if (idx > 0) {
        buf[idx] = '\0';
        idx = 0;

        if (strcmp(buf, "off") == 0) {
          bq_send_off_tone();
        } else if (strcmp(buf, "bal") == 0) {
          if (!cb_test_one_cell(BAL_TEST_DEVICE_ID, BAL_TEST_CELL)) {
            Serial.println("[cmd] bal failed");
          }
        } else if (strcmp(buf, "balstat") == 0) {
          uint8_t st = 0;
          if (cb_read_status(BAL_TEST_DEVICE_ID, st)) cb_print_status(st);
          else Serial.println("[cmd] balstat read failed");
        } else {
          Serial.print("[cmd] Unknown command: ");
          Serial.println(buf);
        }
      }
    } else {
      if (idx < sizeof(buf) - 1) {
        buf[idx++] = c;
      }
    }
  }
}

// ---------------- Per-device print ----------------

static void print_vcells_and_temp_for_device(uint8_t id) {
  float v[16];

  if (read_vcell_id(id, v)) {
    Serial.printf("[vcell:id%u] C1..C16 (V)\n", id);
    for (int i = 0; i < 16; ++i) {
      Serial.printf("  C%02d: %.5f\n", i + 1, v[i]);
    }
  } else {
    Serial.printf("[vcell:id%u] read failed\n", id);
  }

  float tC = 0.0f;
  if (read_die_temp_id(id, tC)) {
    Serial.printf("[temp:id%u] DIETEMP1: %.2f C\n", id, tC);
  } else {
    Serial.printf("[temp:id%u] DIETEMP read failed\n", id);
  }

  if (id == BAL_TEST_DEVICE_ID) {
    uint8_t st = 0;
    if (cb_read_status(id, st)) {
      cb_print_status(st);
    } else {
      Serial.printf("[bal:id%u] BAL_STAT read failed\n", id);
    }
  }
}

static void print_stack_all_bq79616() {
  for (uint8_t id = FIRST_BQ79616_ID; id < NUM_TOTAL_DEV; ++id) {
    print_vcells_and_temp_for_device(id);
  }
}

// ---------------- Init sequence ----------------

static bool bq_init_sequence() {
  BMS.begin(UART_BAUD, SERIAL_8N1);
  Serial.println("[boot] UART1 @ 1,000,000 8N1");
  delay(2000);

  Serial.println("[boot] HWRST 36 ms...");
  bq_hwrst();
  delay(6);

  Serial.println("[boot] WAKE x2 + StA...");
  bq_wake(); delay(12);
  bq_wake(); delay(12);
  bq_sta();  delay(2);

  Serial.printf("[boot] AutoAddress (N=%u)...\n", NUM_TOTAL_DEV);
  autoAddress_fixed();

  delayMicroseconds(4000);

  if (!read_id_status_id(top_id())) return false;

  Serial.println("[adc] Start conversions (ALL)...");
  adc_start_all();

  // Enable TSREF on all bq79616 devices
  if (ENABLE_NTC_SCANS && NUM_TOTAL_DEV > FIRST_BQ79616_ID) {
    for (uint8_t id = FIRST_BQ79616_ID; id < NUM_TOTAL_DEV; ++id) {
      if (!enable_tsref_device(id)) {
        Serial.printf("[tsref] enable failed on id%u (NTC may fail)\n", id);
      }
    }
  }

  return true;
}

// ---------------- Arduino setup/loop ----------------

static uint32_t lastPrintMs = 0;
static uint32_t lastNtcMs   = 0;

void setup() {
  pinMode(LED_PIN, OUTPUT);
  digitalWrite(LED_PIN, LOW);

  Serial.begin(115200);
  uint32_t t0 = millis();
  while (!Serial && (millis() - t0) < USB_WAIT_MS) {}

  Serial.println("\n[boot] Teensy 4.0 <-> bq796xx (N-DEVICE STACK)");
  Serial.printf("[boot] NUM_TOTAL_DEV=%u (id0=bq79600, ids %u..%u=bq79616)\n",
                NUM_TOTAL_DEV,
                FIRST_BQ79616_ID,
                (NUM_TOTAL_DEV == 0) ? 0 : (uint8_t)(NUM_TOTAL_DEV - 1));

  while (!bq_init_sequence()) {
    Serial.println("[boot] init failed; retrying...");
    delay(200);
  }

  lastPrintMs = millis();
  lastNtcMs   = millis();

  // Optional balancing test at boot
  if (ENABLE_BAL_TEST_AT_BOOT) {
    if (!cb_test_one_cell(BAL_TEST_DEVICE_ID, BAL_TEST_CELL)) {
      Serial.println("[bal] boot balancing test failed");
    }
  }

  // Immediate first print + optional first NTC scan
  print_stack_all_bq79616();

  if (ENABLE_NTC_SCANS) {
    for (uint8_t id = FIRST_BQ79616_ID; id < NUM_TOTAL_DEV; ++id) {
      Serial.printf("[ntc:id%u] --- NTC scan start ---\n", id);
      read_all_ntc_device(id);
      Serial.printf("[ntc:id%u] --- NTC scan end ---\n", id);
    }
  }

  Serial.println("[cmd] Type 'bal' to start balance test, 'balstat' to read BAL_STAT, 'off' to send shutdown tone");
}

void loop() {
  handle_usb_command();

  uint32_t now = millis();

  if (now - lastPrintMs >= PRINT_INTERVAL_MS) {
    lastPrintMs = now;
    print_stack_all_bq79616();
  }

  if (ENABLE_NTC_SCANS && (now - lastNtcMs >= NTC_SCAN_INTERVAL_MS)) {
    lastNtcMs = now;

    for (uint8_t id = FIRST_BQ79616_ID; id < NUM_TOTAL_DEV; ++id) {
      Serial.printf("[ntc:id%u] --- NTC scan start ---\n", id);
      read_all_ntc_device(id);
      Serial.printf("[ntc:id%u] --- NTC scan end ---\n", id);
    }
  }

  delay(5);
}
