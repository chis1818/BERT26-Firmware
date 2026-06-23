/*
  ============================================================
  Teensy 4.0 <-> BQ79600 (base) + N x BQ79616 (stack)
  ============================================================

  ONLY LINE TO CHANGE:
    NUM_STACK_DEVS = number of BQ79616s in the stack

  All BQ79616s have dual-TMUX NTC mux boards wired as:
    GPIO1 = A0, GPIO2 = A1, GPIO3 = A2  (mux address)
    GPIO4 = MUX0 EN,  GPIO5 = MUX0 OUT  (cells  1-8)
    GPIO6 = MUX1 EN,  GPIO7 = MUX1 OUT  (cells 9-16)

  Loop behaviour:
    - Voltages read every loop iteration (~fast)
    - Temperatures read every TEMP_INTERVAL_MS (~10 s)
    - Pack stats printed every loop using most recent data

  Output:
    [vcell:idX] V: <C1..C16>
    [temp:idX]  T: <T1..T16>
    [pack:volt] min max spread avg std
    [pack:temp] min max spread avg std
    [timing] loop=Xms
  ============================================================
*/

#include <Arduino.h>
#include <math.h>

// ============================================================
// USER CONFIG
// ============================================================
static const uint8_t  NUM_STACK_DEVS    = 6;
static const uint32_t TEMP_INTERVAL_MS  = 10000;  // temp read period (ms)

// ============================================================
// DERIVED
// ============================================================
static const uint8_t  NUM_DEV    = NUM_STACK_DEVS + 1;
static const uint8_t  ID_BASE    = 0;
static const uint8_t  ID_TOP     = NUM_STACK_DEVS;
static const uint16_t PACK_CELLS = (uint16_t)NUM_STACK_DEVS * 16u;

// ============================================================
// STATS STRUCT  (declared before any use)
// ============================================================
struct Stats { float mn, mx, spread, avg, std; };

// ============================================================
// PACK DATA  (most recent readings, persisted between loops)
// ============================================================
static float g_volts[NUM_STACK_DEVS * 16];   // [device*16 + cell]
static float g_temps[NUM_STACK_DEVS * 16];   // NaN = not yet read

// ============================================================
// HARDWARE
// ============================================================
static const uint32_t UART_BAUD   = 1000000;
static const uint32_t USB_WAIT_MS = 8000;
HardwareSerial& BMS = Serial1;
const int LED_PIN = 13;

// ============================================================
// REGISTERS
// ============================================================
#define ACTIVE_CELL     0x0003
#define ADC_CONF1       0x0007
#define GPIO_CONF1      0x000E
#define GPIO_CONF2      0x000F
#define GPIO_CONF3      0x0010
#define GPIO_CONF4      0x0011
#define DIR0_ADDR       0x0306
#define COMM_CTRL       0x0308
#define CONTROL1        0x0309
#define CONTROL2        0x030A
#define ADC_CTRL1       0x030D
#define COMM_TIMEOUT_CONF 0x0019
#define OTP_ECC_DATAIN1 0x0343
#define OTP_ECC_DATAIN8 0x034A
#define PARTID          0x0500
#define DEV_STAT        0x052C
#define VCELL16_HI      0x0568
#define TSREF_HI        0x058C
#define GPIO1_HI        0x058E

static inline uint16_t gpio_reg(uint8_t gpio) {
  return (uint16_t)(GPIO1_HI + 2u * (uint16_t)(gpio - 1u));
}

// GPIO mode constants
#define GM_OFF  0b000
#define GM_ADC  0b010
#define GM_HI   0b100
#define GM_LO   0b101

// ============================================================
// FRAME TYPES
// ============================================================
enum : uint8_t { FRMWRT_SGL_W = 0x90, FRMWRT_STK_W = 0xB0, FRMWRT_ALL_W = 0xD0 };
enum : uint8_t { FRMWRT_SGL_R = 0x00, FRMWRT_STK_R = 0x20, FRMWRT_ALL_R = 0x40 };

// ============================================================
// CRC
// ============================================================
static uint16_t crc16(const uint8_t* d, size_t n) {
  uint16_t crc = 0xFFFF;
  for (size_t i = 0; i < n; i++) {
    crc ^= d[i];
    for (int b = 0; b < 8; b++) crc = (crc & 1) ? (crc >> 1) ^ 0xA001 : (crc >> 1);
  }
  return crc;
}

// ============================================================
// UART
// ============================================================
static void tx(uint8_t wrType, uint8_t addr, uint16_t reg,
               const uint8_t* data, uint8_t n) {
  uint8_t buf[16];
  uint8_t k = 0;
  buf[k++] = 0x80 | (wrType + (n - 1));
  if (wrType == FRMWRT_SGL_W) buf[k++] = addr;
  buf[k++] = reg >> 8;
  buf[k++] = reg & 0xFF;
  for (uint8_t i = 0; i < n; i++) buf[k++] = data[i];
  uint16_t c = crc16(buf, k);
  buf[k++] = c & 0xFF;
  buf[k++] = c >> 8;
  BMS.write(buf, k);
  BMS.flush();
}

static void rx_req(uint8_t rdType, uint8_t addr, uint16_t reg, uint8_t nBytes) {
  uint8_t buf[8];
  uint8_t k = 0;
  buf[k++] = 0x80 | rdType;
  if (rdType == FRMWRT_SGL_R) buf[k++] = addr;
  buf[k++] = reg >> 8;
  buf[k++] = reg & 0xFF;
  buf[k++] = nBytes - 1;
  uint16_t c = crc16(buf, k);
  buf[k++] = c & 0xFF;
  buf[k++] = c >> 8;
  BMS.write(buf, k);
  BMS.flush();
}

// Wait for exactly n bytes, bail after to_ms
static bool rx_wait(uint8_t* dst, size_t n, uint32_t to_ms) {
  uint32_t t0 = millis();
  size_t got = 0;
  while (got < n) {
    while (BMS.available() && got < n) dst[got++] = BMS.read();
    if (got == n) return true;
    if (millis() - t0 >= to_ms) return false;
  }
  return true;
}

// Read 1 byte from a device register
static bool reg_read8(uint8_t id, uint16_t reg, uint8_t& val) {
  uint8_t buf[7] = {};
  rx_req(FRMWRT_SGL_R, id, reg, 1);
  if (!rx_wait(buf, 7, 30)) return false;
  val = buf[4];
  return true;
}

// Read 2 bytes from a device register
static bool reg_read16(uint8_t id, uint16_t reg, uint16_t& val) {
  uint8_t buf[8] = {};
  rx_req(FRMWRT_SGL_R, id, reg, 2);
  if (!rx_wait(buf, 8, 30)) return false;
  val = ((uint16_t)buf[4] << 8) | buf[5];
  return true;
}

// Write 1 byte, broadcast
static void reg_write_all(uint16_t reg, uint8_t val) {
  tx(FRMWRT_ALL_W, 0, reg, &val, 1);
}

// Write 1 byte, single device
static void reg_write_sgl(uint8_t id, uint16_t reg, uint8_t val) {
  tx(FRMWRT_SGL_W, id, reg, &val, 1);
}

// ============================================================
// BUS TONES
// ============================================================
static void ping(uint32_t us) {
  BMS.end();
  pinMode(1, OUTPUT);
  digitalWrite(1, LOW);
  delayMicroseconds(us);
  digitalWrite(1, HIGH);
  BMS.begin(UART_BAUD, SERIAL_8N1);
  delayMicroseconds(200);
}

static void bq_hwrst() { ping(36000); delay(100); }
static void bq_wake()  { ping(2500); }
static void bq_sta()   { ping(250); }

// ============================================================
// AUTO ADDRESS
// ============================================================
static void auto_address() {
  // DLL sync
  for (uint16_t r = OTP_ECC_DATAIN1; r <= OTP_ECC_DATAIN8; r++) {
    uint8_t z = 0;
    tx(FRMWRT_STK_W, 0, r, &z, 1);
  }
  // Enable addr write on all devices
  reg_write_all(CONTROL1, 0x01);
  // Assign addresses 0..NUM_DEV-1
  for (uint8_t a = 0; a < NUM_DEV; a++)
    reg_write_all(DIR0_ADDR, a);
  // Set comm roles
  reg_write_all(COMM_CTRL, 0x02);              // all = STACK
  reg_write_sgl(ID_TOP,  COMM_CTRL, 0x03);    // top = TOP
  reg_write_sgl(ID_BASE, COMM_CTRL, 0x00);    // base = BASE
}

// ============================================================
// ADC INIT  (call once, leaves ADC running continuously)
// ============================================================
static void adc_init() {
  reg_write_all(ACTIVE_CELL, 0x0A);   // enable cells
  reg_write_all(ADC_CONF1,   0x02);   // LPF on
  reg_write_all(ADC_CTRL1,   0x0E);   // MAIN_GO | CONT
  delay(50);                           // first conversion settle
}

// ============================================================
// TSREF ENABLE
// ============================================================
static void enable_tsref(uint8_t id) {
  uint8_t v = 0;
  reg_read8(id, CONTROL2, v);
  reg_write_sgl(id, CONTROL2, v | 0x01);
  delay(3);
}

// ============================================================
// VOLTAGE READ  — fills dst[0..15] = C1..C16
// ============================================================
static bool read_voltages(uint8_t id, float dst[16]) {
  uint8_t buf[38] = {};
  rx_req(FRMWRT_SGL_R, id, VCELL16_HI, 32);
  if (!rx_wait(buf, 38, 60)) return false;
  // Registers come back C16..C1; reverse to C1..C16
  for (int i = 0; i < 16; i++) {
    uint16_t raw = ((uint16_t)buf[4 + i * 2] << 8) | buf[4 + i * 2 + 1];
    dst[15 - i] = (int16_t)raw * 190.73e-6f;
  }
  return true;
}

// ============================================================
// NTC HELPERS
// ============================================================
static const float NTC_R25  = 10000.0f;
static const float NTC_BETA = 3380.0f;
static const float T0_K     = 298.15f;   // 25 + 273.15
static const float RPULL    = 10000.0f;
static const float CORR_A   = -5.7e-6f;
static const float CORR_B   =  0.846f;
static const float CORR_C   =  180.0f;

static float ntc_to_c(float Rmeas) {
  float R = CORR_A * Rmeas * Rmeas + CORR_B * Rmeas + CORR_C;
  if (R <= 0.0f) return NAN;
  float invT = (1.0f / T0_K) + logf(R / NTC_R25) / NTC_BETA;
  if (invT <= 0.0f) return NAN;
  return 1.0f / invT - 273.15f;
}

// Set mux address + EN for one channel on device id.
// muxBank 0 = GPIO4/5 (cells 1-8), bank 1 = GPIO6/7 (cells 9-16).
// Does NOT touch ADC registers.
static void mux_set(uint8_t id, uint8_t bank, uint8_t chan) {
  chan &= 0x7;
  uint8_t a0 = (chan & 1) ? GM_HI : GM_LO;
  uint8_t a1 = (chan & 2) ? GM_HI : GM_LO;
  uint8_t a2 = (chan & 4) ? GM_HI : GM_LO;
  uint8_t en0 = (bank == 0) ? GM_HI : GM_OFF;
  uint8_t en1 = (bank == 1) ? GM_HI : GM_OFF;
  reg_write_sgl(id, GPIO_CONF1, (uint8_t)((a1 << 3) | a0));
  reg_write_sgl(id, GPIO_CONF2, (uint8_t)((en0 << 3) | a2));
  reg_write_sgl(id, GPIO_CONF3, (uint8_t)((en1 << 3) | GM_ADC));  // GPIO5=ADC, GPIO6=en1
  reg_write_sgl(id, GPIO_CONF4, (uint8_t)((GM_OFF << 3) | GM_ADC)); // GPIO8=off, GPIO7=ADC
}

// Read TSREF and one GPIO ADC code.
// Returns false if either is invalid.
static bool mux_read(uint8_t id, uint8_t gpioNum, float& tC) {
  uint16_t tsref_raw = 0, gpio_raw = 0;
  if (!reg_read16(id, TSREF_HI,        tsref_raw)) return false;
  if (!reg_read16(id, gpio_reg(gpioNum), gpio_raw)) return false;

  int16_t tsref = (int16_t)tsref_raw;
  int16_t code  = (int16_t)gpio_raw;

  if (tsref <= 0 || tsref == (int16_t)0x8000) return false;
  if (code  <= 0 || code  == (int16_t)0x8000) return false;

  float ratio = (float)code / (float)tsref;
  if (ratio <= 0.001f || ratio >= 0.999f) return false;

  float Rmeas = RPULL * ratio / (1.0f - ratio);
  tC = ntc_to_c(Rmeas);
  return !isnan(tC);
}

// Read all 16 NTC channels on one device.
// Settle time per channel: 4 ms (enough for ~20 ADC round-robins at 200 us each).
// Bank switchover gets an extra 4 ms.
static void read_temps(uint8_t id, float dst[16]) {
  const uint32_t SETTLE_US = 4000;

  for (uint8_t cell = 0; cell < 16; cell++) {
    uint8_t bank    = (cell < 8) ? 0 : 1;
    uint8_t chan    = cell & 0x7;
    uint8_t gpioOut = (bank == 0) ? 5 : 7;

    mux_set(id, bank, chan);

    // Extra settle on bank 0 first channel and bank switchover
    if (cell == 0 || cell == 8) delayMicroseconds(SETTLE_US);
    delayMicroseconds(SETTLE_US);

    float tC = NAN;
    mux_read(id, gpioOut, tC);
    dst[cell] = tC;
  }
}

// ============================================================
// STATS
// ============================================================
static uint16_t calc_stats(const float* arr, uint16_t len, Stats& s) {
  s = {NAN, NAN, NAN, NAN, NAN};
  uint16_t n = 0;
  double sum = 0;
  for (uint16_t i = 0; i < len; i++) {
    if (isnan(arr[i])) continue;
    if (n == 0 || arr[i] < s.mn) s.mn = arr[i];
    if (n == 0 || arr[i] > s.mx) s.mx = arr[i];
    sum += arr[i]; n++;
  }
  if (!n) return 0;
  s.avg = (float)(sum / n);
  s.spread = s.mx - s.mn;
  double ss = 0;
  for (uint16_t i = 0; i < len; i++) {
    if (isnan(arr[i])) continue;
    double d = arr[i] - s.avg; ss += d * d;
  }
  s.std = (float)sqrt(ss / n);
  return n;
}

// ============================================================
// PRINT HELPERS
// ============================================================
static void print_volts(uint8_t id, const float v[16]) {
  Serial.printf("[vcell:id%d] V:", id);
  for (int i = 0; i < 16; i++) Serial.printf(" %.5f", v[i]);
  Serial.println();
}

static void print_temps_line(uint8_t id, const float t[16]) {
  Serial.printf("[temp:id%d]  T:", id);
  for (int i = 0; i < 16; i++) {
    if (isnan(t[i])) Serial.print(" ---");
    else              Serial.printf(" %.2f", t[i]);
  }
  Serial.println();
}

static void print_pack_stats() {
  Stats vs, ts;
  uint16_t vn = calc_stats(g_volts, PACK_CELLS, vs);
  uint16_t tn = calc_stats(g_temps, PACK_CELLS, ts);
  Serial.println("--- Pack ---");
  if (vn)
    Serial.printf("[pack:volt] min=%.5f max=%.5f spread=%.5f avg=%.5f std=%.5f V (%u)\n",
                  vs.mn, vs.mx, vs.spread, vs.avg, vs.std, vn);
  if (tn)
    Serial.printf("[pack:temp] min=%.2f max=%.2f spread=%.2f avg=%.2f std=%.2f C (%u)\n",
                  ts.mn, ts.mx, ts.spread, ts.avg, ts.std, tn);
}

// ============================================================
// INIT SEQUENCE
// ============================================================
static void bq_init() {
  BMS.begin(UART_BAUD, SERIAL_8N1);
  delay(500);

  bq_hwrst();
  delay(10);
  bq_wake(); delay(15);
  bq_wake(); delay(15);
  bq_sta();  delay(5);

  auto_address();
  delay(5);

  // Verify stack devices
  for (uint8_t s = 0; s < NUM_STACK_DEVS; s++) {
    uint8_t id = s + 1;
    uint16_t pid = 0;
    if (reg_read16(id, PARTID, pid)) {
      Serial.printf("[init] id%d PARTID=0x%04X\n", id, pid);
    } else {
      Serial.printf("[init] id%d no response — restarting\n", id);
      bq_init();  // recurse once
      return;
    }
  }

  // Enable TSREF on all stack devices
  for (uint8_t s = 0; s < NUM_STACK_DEVS; s++) enable_tsref(s + 1);

  adc_init();

  // Auto-shutdown after 1 min of no valid comms: write then verify each stack device.
  // CTL_ACT=1 (SHUTDOWN), CTL_TIME=100b (1 min), CTS_TIME=000 disabled → 0x0C
  for (uint8_t s = 0; s < NUM_STACK_DEVS; s++) {
    uint8_t id = s + 1;
    reg_write_sgl(id, COMM_TIMEOUT_CONF, 0x0C);
    delay(2);
    uint8_t rb = 0;
    if (reg_read8(id, COMM_TIMEOUT_CONF, rb)) {
      Serial.printf("[init] id%d COMM_TIMEOUT_CONF=0x%02X %s\n",
                    id, rb, rb == 0x0C ? "OK" : "MISMATCH");
    } else {
      Serial.printf("[init] id%d COMM_TIMEOUT_CONF read FAIL\n", id);
    }
  }

  // Pre-fill temp array with NaN so stats skip until first real read
  for (uint16_t i = 0; i < PACK_CELLS; i++) g_temps[i] = NAN;

  Serial.println("[init] done");
}

// ============================================================
// SETUP
// ============================================================
void setup() {
  pinMode(LED_PIN, OUTPUT);
  Serial.begin(115200);
  uint32_t t0 = millis();
  while (!Serial && millis() - t0 < USB_WAIT_MS) {}
  Serial.printf("\n[boot] BQ79600 + %d x BQ79616  (%u cells)\n",
                NUM_STACK_DEVS, PACK_CELLS);
  bq_init();
}

// ============================================================
// LOOP
// ============================================================
void loop() {
  static uint32_t last_temp_ms = 0;
  uint32_t now = millis();
  uint32_t t0  = now;
delay(5000);
  // ---- VOLTAGES: every iteration ----
  bool all_v_ok = true;
  for (uint8_t s = 0; s < NUM_STACK_DEVS; s++) {
    uint8_t id = s + 1;
    float* vSlot = g_volts + s * 16;
    if (read_voltages(id, vSlot)) {
      print_volts(id, vSlot);
    } else {
      Serial.printf("[vcell:id%d] FAIL\n", id);
      all_v_ok = false;
    }
  }

  // ---- TEMPERATURES: every TEMP_INTERVAL_MS ----
  if (now - last_temp_ms >= TEMP_INTERVAL_MS) {
    last_temp_ms = now;
    for (uint8_t s = 0; s < NUM_STACK_DEVS; s++) {
      uint8_t id = s + 1;
      float* tSlot = g_temps + s * 16;
      read_temps(id, tSlot);
      print_temps_line(id, tSlot);
    }
    // Re-arm ADC for normal voltage reading after mux sweep
    adc_init();
  }

  // ---- PACK STATS: every iteration (uses latest data) ----
  print_pack_stats();

  Serial.printf("[timing] loop=%lums\n", millis() - t0);
}
