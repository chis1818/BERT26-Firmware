/*
  Teensy 4.0 <-> TI BQ79616 (TWO-DEVICE STACK: VCELL + DIE TEMP)
  Flow (NO assumptions changed; adds HWRST at boot):
    HWRST 36 ms
    WAKE x2 (12 ms apart) -> StA
    DLL touch (optional log)
    AutoAddress(2):
      - write 0x0343..0x034A (DLL touch)
      - CONTROL1 = 0x01
      - DIR0_ADDR = 0x00, 0x01  (broadcast loop)
      - COMM_CTRL = 0x02 (ALL=STACK)
      - COMM_CTRL(base:ID0)=0x00 , COMM_CTRL(top:ID1)=0x03
    wait 4 ms
    Start ADC (broadcast)
    Read per-device:
      - VCELL16_HI.. (addr 0) -> C1..C16 (V)
      - DIETEMP1 / DIETEMP2 raw -> °C (0.025 °C/LSB) and print

  UART1 = 1,000,000 8N1
  Tones on TX1 (pin 1) for HWRST/WAKE/StA
*/

#include <Arduino.h>

static const uint32_t UART_BAUD   = 1000000;
static const uint32_t USB_WAIT_MS = 8000;
HardwareSerial &BMS = Serial1;
const int LED_PIN = 13;

static const uint8_t NUM_DEV = 2;    // ********* two devices *********

// ----- A0/B0 register subset -----
#define ACTIVE_CELL      0x0003
#define ADC_CONF1        0x0007
#define TX_HOLD_OFF      0x001A
#define DIR0_ADDR        0x0306
#define COMM_CTRL        0x0308
#define CONTROL1         0x0309
#define ADC_CTRL1        0x030D
#define OTP_ECC_DATAIN1  0x0343
#define OTP_ECC_DATAIN8  0x034A
#define PARTID           0x0500   // 2 bytes
#define DEV_STAT         0x052C   // 1 byte
#define VCELL16_HI       0x0568   // 32 bytes (C16..C1)
#define TSREF_HI         0x058C   // 2 bytes (optional)
#define DIETEMP1_HI      0x05AE   // 2 bytes (signed)
#define DIETEMP2_HI      0x05B0   // 2 bytes (signed)
#include <math.h>   // <-- add this at top if not already present

// --- extra control & GPIO registers ---
#define CONTROL2        0x030A

#define GPIO_CONF1      0x000E
#define GPIO_CONF2      0x000F
#define GPIO_CONF3      0x0010
#define GPIO_CONF4      0x0011

// Main-ADC result registers
#define GPIO1_HI       0x058E
#define GPIO2_HI       0x0590
#define GPIO3_HI       0x0592
#define GPIO4_HI       0x0594
#define GPIO5_HI       0x0596
#define GPIO6_HI       0x0598
#define GPIO7_HI       0x059A
#define GPIO8_HI       0x059C
// GPIO2_HI = GPIO1_HI + 2, etc.
static inline uint16_t gpio_hi_reg(uint8_t gpio) {
  return (uint16_t)(GPIO1_HI + 2u * (uint16_t)(gpio - 1u));
}

// --- GPIO mode encodings from datasheet ---
static const uint8_t GPIO_MODE_DISABLE    = 0b000;
static const uint8_t GPIO_MODE_ADC_OTUT   = 0b001;
static const uint8_t GPIO_MODE_ADC_ONLY   = 0b010;
static const uint8_t GPIO_MODE_DIGITAL_IN = 0b011;
static const uint8_t GPIO_MODE_OUT_HIGH   = 0b100;
static const uint8_t GPIO_MODE_OUT_LOW    = 0b101;
static const uint8_t GPIO_MODE_ADC_WPU    = 0b110;
static const uint8_t GPIO_MODE_ADC_WPD    = 0b111;

// NTC params
static const float NTC_R25   = 10000.0f;   // 10k at 25°C
static const float NTC_BETA  = 3380.0f;    // B25/50 = 3380K
static const float T0_KELVIN = 273.15f + 25.0f;

// frame types
enum : uint8_t { FRMWRT_SGL_W = 0x90, FRMWRT_STK_W = 0xB0, FRMWRT_ALL_W = 0xD0 };
enum : uint8_t { FRMWRT_SGL_R = 0x00, FRMWRT_STK_R = 0x20, FRMWRT_ALL_R = 0x40 };

// CRC16 IBM/Modbus (0xA001), LSB-first  (unchanged)
static uint16_t crc16_ibm(const uint8_t *data, size_t len) {
  uint16_t crc = 0xFFFF;
  for (size_t i=0;i<len;++i) {
    crc ^= data[i];
    for (int b=0; b<8; ++b) crc = (crc & 1) ? (crc >> 1) ^ 0xA001 : (crc >> 1);
  }
  return crc; // low byte first
}

// ---- USB Serial command handler (type "off" + Enter in Serial Monitor) ----
static void handle_usb_command() {
  static char buf[16];
  static uint8_t idx = 0;

  while (Serial.available()) {
    char c = (char)Serial.read();

    if (c == '\r' || c == '\n') {
      if (idx > 0) {
        buf[idx] = '\0';
        idx = 0;

        if (strcmp(buf, "off") == 0) {
          bq_send_off_tone();
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

// ---- UART helpers (same framing) ----
static void writeFrame(uint8_t wrType, uint8_t devAddr, uint16_t reg, const uint8_t *data, uint8_t n) {
  const uint8_t hdr = wrType + (n - 1);
  uint8_t buf[1 + 1 + 2 + 8 + 2]; size_t k=0;
  buf[k++] = 0x80 | hdr;
  if (wrType == FRMWRT_SGL_W) buf[k++] = devAddr;
  buf[k++] = (uint8_t)(reg>>8); buf[k++] = (uint8_t)reg;
  for (uint8_t i=0;i<n;++i) buf[k++] = data[i];
  uint16_t c = crc16_ibm(buf, k); buf[k++] = (uint8_t)c; buf[k++] = (uint8_t)(c>>8);
  BMS.write(buf, k); BMS.flush();
}

static void readFrame(uint8_t rdType, uint8_t devAddr, uint16_t reg, uint8_t nBytes) {
  uint8_t buf[1 + 1 + 2 + 1 + 2]; size_t k=0;
  buf[k++] = 0x80 | rdType;
  if (rdType == FRMWRT_SGL_R) buf[k++] = devAddr;
  buf[k++] = (uint8_t)(reg>>8); buf[k++] = (uint8_t)reg;
  buf[k++] = nBytes - 1;
  uint16_t c = crc16_ibm(buf, k); buf[k++] = (uint8_t)c; buf[k++] = (uint8_t)(c>>8);
  BMS.write(buf, k); BMS.flush();
}

static bool readExact(uint8_t *dst, size_t n, uint32_t to_ms) {
  uint32_t t0 = millis(); size_t got=0;
  while (got<n && (millis()-t0)<to_ms) {
    while (BMS.available() && got<n) dst[got++] = (uint8_t)BMS.read();
    if (got<n) delay(0);
  }
  return got==n;
}

static void dumpHex(const uint8_t *p, size_t n) {
  for (size_t i=0;i<n;++i) Serial.printf("%02X%s", p[i], (i+1<n)?" ":"\n");
}

// ---- TX1 as GPIO for pings ----
static void uartPing_us(uint32_t low_us) {
  BMS.end();
  pinMode(1, OUTPUT);
  digitalWrite(1, LOW);
  delayMicroseconds(low_us);
  digitalWrite(1, HIGH);
  BMS.begin(UART_BAUD, SERIAL_8N1);
  delayMicroseconds(100);
}
static void bq_hwrst(){ 
  uartPing_us(36000); 
  delay(100);}  // ******** added ********
static void bq_wake(){  uartPing_us(2500);  }
static void bq_sta(){   uartPing_us(250);   }
// ---- user command: send SHUTDOWN "OFF" tone via CONTROL1[SEND_SHUTDOWN] ----
static void bq_send_off_tone() {
  // CONTROL1 bits (addr 0x0309):
  // bit6 = SEND_SHUTDOWN; we set only this bit, others = 0
  uint8_t v = 0x40;  // 0b0100_0000

  // Broadcast write so every device sends a SHUTDOWN tone to the next device up the stack
  writeFrame(FRMWRT_ALL_W, 0, CONTROL1, &v, 1);

  Serial.println("[cmd] OFF: CONTROL1[SEND_SHUTDOWN]=1 -> SHUTDOWN tone sent up the stack");
}


static void autoAddress_fixed(void) {
  // Step 1: dummy write ECC_DATA1..8 (broadcast) to sync DLL (write direction)
 

  // Step 1: Send wake tone
  {
    uint8_t c1;
    // Broadcast read isn't available yet; if you can’t read, assume default and just set the bit carefully.
    // If single-read of base is possible, do it; otherwise set only the bit:
    c1 = 0b00100000; // assumes bit0 = ADDR_WR; safer: read-modify-write per device once you have addresses
    writeFrame(FRMWRT_SGL_W, 0, CONTROL1, &c1, 1);
  }
  

   for (uint16_t r = OTP_ECC_DATAIN1; r <= OTP_ECC_DATAIN8; ++r) {
    uint8_t z = 0x00;
    writeFrame(FRMWRT_STK_W, 0, r, &z, 1);
  }
    // Step 2: enable auto-addressing: CONTROL1[ADDR_WR]=1 for all (read-modify-write!)
  {
    uint8_t c1;
    // Broadcast read isn't available yet; if you can’t read, assume default and just set the bit carefully.
    // If single-read of base is possible, do it; otherwise set only the bit:
    c1 = 0b00000001; // assumes bit0 = ADDR_WR; safer: read-modify-write per device once you have addresses
    writeFrame(FRMWRT_ALL_W, 0, CONTROL1, &c1, 1);
  }

  // Step 3: send addresses in ASCENDING order (base must see 0x00 first)
  for (uint8_t a = 0; a < NUM_DEV; ++a) {
    writeFrame(FRMWRT_ALL_W, 0, DIR0_ADDR, &a, 1);
    // small inter-frame gap can help if timing is tight
  }

  // (Optional) Step 5: dummy read ECC_DATA1..8 (broadcast) to sync DLL (read direction)
  // ... your broadcast read here if you have a helper

  // Step 4: configure COMM roles
  // Optionally broadcast "stack" first, then fix base and top:
  { uint8_t v = 0x02; writeFrame(FRMWRT_ALL_W, 0, COMM_CTRL, &v, 1); } // STACK_DEV=1, TOP_STACK=0
  // { uint8_t v = 0x00; writeFrame(FRMWRT_SGL_W, 0x00, COMM_CTRL, &v, 1); } // base (id0): BASE
  { uint8_t v = 0x03; writeFrame(FRMWRT_SGL_W, (NUM_DEV-1), COMM_CTRL, &v, 1); } // top: TOP
   for (uint16_t r = OTP_ECC_DATAIN1; r <= OTP_ECC_DATAIN8; ++r) {
    uint8_t z = 0x00;
    writeFrame(FRMWRT_STK_W, 0, r, &z, 1);
  }

  // (Recommended) Read back DIR0_ADDR via broadcast read to verify the sequence 0..NUM_DEV-1
}

// ---- ADC start (broadcast; applies to both) ----
static void adc_start_all() {
  { uint8_t v=0x0A; writeFrame(FRMWRT_ALL_W, 0, ACTIVE_CELL, &v, 1); } // enable all cells
  { uint8_t v=0x02; writeFrame(FRMWRT_ALL_W, 0, ADC_CONF1,  &v, 1); }  // LPF on
  { uint8_t v=0x0E; writeFrame(FRMWRT_ALL_W, 0, ADC_CTRL1,  &v, 1); }  // MAIN_GO | CONT
  delayMicroseconds(40000); // let first conversion complete
}

// ---- conversions ----
static float raw_to_V(uint16_t raw) { return (int16_t)raw * 190.73e-6f; } // 190.73 µV/LSB
static float raw_to_C(int16_t raw)   { return 0.025f * (float)raw; }      // 0.025 °C/LSB (signed)

// ---- SGL helpers ----
static bool read_u16(uint8_t addr, uint16_t reg_hi, uint16_t &val) {
  uint8_t rx[8]={0};
  readFrame(FRMWRT_SGL_R, addr, reg_hi, 2);
  if (!readExact(rx, sizeof(rx), 200)) return false;
  val = ((uint16_t)rx[4] << 8) | rx[5];
  return true;
}

static bool read_vcell_id(uint8_t id, float outV[16]) {
  uint8_t rx[32+6]={0};
  readFrame(FRMWRT_SGL_R, id, VCELL16_HI, 32);
  if (!readExact(rx, sizeof(rx), 300)) return false;
  for (int i=0;i<16;++i) {
    uint16_t raw = ((uint16_t)rx[4+i*2]<<8) | rx[4+i*2+1]; // C16..C1
    int cell = 16 - i; outV[cell-1] = raw_to_V(raw);       // -> C1..C16
  }
  return true;
}

static bool read_die_temp_id(uint8_t id, float &tC1) {
  uint16_t r1=0;
  bool ok1 = read_u16(id, DIETEMP1_HI, r1);
  if (!ok1) return false;
  tC1 = raw_to_C((int16_t)r1);
  return true;
}

static bool read_id_status_id(uint8_t id) {
  uint8_t rx8[8]={0}; readFrame(FRMWRT_SGL_R, id, PARTID, 2);
  if (readExact(rx8, sizeof(rx8), 150)) {
    uint16_t pid = (rx8[4]<<8)|rx8[5];
    Serial.printf("[id%d] PARTID=0x%04X  raw: ", id, pid); dumpHex(rx8, sizeof(rx8));
  } else Serial.printf("[id%d] PARTID timeout\n", id);

  uint8_t rx7[7]={0}; readFrame(FRMWRT_SGL_R, id, DEV_STAT, 1);
  if (readExact(rx7, sizeof(rx7), 150)) {
    Serial.printf("[st%d] DEV_STAT=0x%02X  raw: ", id, rx7[4]), dumpHex(rx7, sizeof(rx7));
    return true;}
  else {
    Serial.printf("[st%d] DEV_STAT timeout\n", id);
    return false;}
}
// --- raw Main-ADC reads (TSREF + GPIO) --------------------

static bool read_tsref_code_id(uint8_t id, int16_t &code) {
  uint16_t v = 0;
  if (!read_u16(id, TSREF_HI, v)) return false;
  code = (int16_t)v;
  // 0x8000 means "default / inactive" per datasheet
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

// --- NTC math: ADC codes -> resistance -> °C -------------

static bool ntc_from_adc_codes(int16_t gpioCode, int16_t tsrefCode, float &tC) {
  if (tsrefCode <= 0) return false;

  // ratiometric: (GPIO / TSREF) = Rntc / (Rntc + Rpull)
  float ratio = (float)gpioCode / (float)tsrefCode;
  if (ratio <= 0.0f || ratio >= 0.999f) return false;

  const float Rpull = NTC_R25;          // 10k pull-up to TSREF
  float Rntc = Rpull * ratio / (1.0f - ratio);

  // Steinhart–Hart (single-parameter beta form)
  float lnR   = logf(Rntc / NTC_R25);
  float invT  = (1.0f / T0_KELVIN) + (lnR / NTC_BETA);
  float Tkelv = 1.0f / invT;
  tC = Tkelv - 273.15f;
  return true;
}

// --- GPIO configuration for the top device thermistor mux ---

// Top device = ID1
// MUX0: EN on GPIO4, output on GPIO5
// MUX1: EN on GPIO6, output on GPIO7
// A0/A1/A2 for both muxes: GPIO1 / GPIO2 / GPIO3

static void set_top_mux_channel(uint8_t muxIndex, uint8_t chan) {
  // muxIndex: 0 = first TMUX (GPIO5 output), 1 = second (GPIO7 output)
  // chan: 0..7 (thermistor index within that TMUX)

  chan &= 0x7;

  // A0..A2 bits from chan (0..7)
  bool a0 = chan & 0x1;
  bool a1 = chan & 0x2;
  bool a2 = chan & 0x4;

  // GPIO1/2/3 drive A0/A1/A2 (OUT_HIGH / OUT_LOW)
  uint8_t mode_g1 = a0 ? GPIO_MODE_OUT_HIGH : GPIO_MODE_OUT_LOW;
  uint8_t mode_g2 = a1 ? GPIO_MODE_OUT_HIGH : GPIO_MODE_OUT_LOW;
  uint8_t mode_g3 = a2 ? GPIO_MODE_OUT_HIGH : GPIO_MODE_OUT_LOW;

  // EN lines
  uint8_t mode_g4; // EN for MUX0
  uint8_t mode_g6; // EN for MUX1

  if (muxIndex == 0) {
    mode_g4 = GPIO_MODE_OUT_HIGH;   // enable MUX0
    mode_g6 = GPIO_MODE_DISABLE;    // let pulldown keep EN2 low
  } else {
    mode_g4 = GPIO_MODE_DISABLE;    // EN1 low via pulldown
    mode_g6 = GPIO_MODE_OUT_HIGH;   // enable MUX1
  }

  // Sense pins: always ADC-only
  uint8_t mode_g5 = GPIO_MODE_ADC_ONLY;  // MUX0 output
  uint8_t mode_g7 = GPIO_MODE_ADC_ONLY;  // MUX1 output
  uint8_t mode_g8 = GPIO_MODE_DISABLE;   // unused

  // Pack into GPIO_CONFx registers.
  // NOTE: for CONF2/3/4 the upper GPIO field is in bits [5:3], not [7:5].
  uint8_t conf1 = (uint8_t)((mode_g2 << 3) | (mode_g1));      // GPIO2[2:0], GPIO1[2:0]
  uint8_t conf2 = (uint8_t)((mode_g4 << 3) | (mode_g3));      // GPIO4[2:0], GPIO3[2:0]
  uint8_t conf3 = (uint8_t)((mode_g6 << 3) | (mode_g5));      // GPIO6[2:0], GPIO5[2:0]
  uint8_t conf4 = (uint8_t)((mode_g8 << 3) | (mode_g7));      // GPIO8[2:0], GPIO7[2:0]

  writeFrame(FRMWRT_SGL_W, 1, GPIO_CONF1, &conf1, 1);
  writeFrame(FRMWRT_SGL_W, 1, GPIO_CONF2, &conf2, 1);
  writeFrame(FRMWRT_SGL_W, 1, GPIO_CONF3, &conf3, 1);
  writeFrame(FRMWRT_SGL_W, 1, GPIO_CONF4, &conf4, 1);

  // allow MUX + ADC round robin to settle
  delayMicroseconds(2500);   // > 8 round-robins (~1.6ms) with margin
  adc_start_all();
}

// --- high-level NTC read for all 16 cells on top device ---

static void read_all_ntc_top() {
  int16_t tsrefCode = 0;
  if (!read_tsref_code_id(1, tsrefCode)) {
    Serial.println("[ntc:id1] TSREF read failed");
    return;
  }

  for (uint8_t cell = 1; cell <= 16; ++cell) {
    uint8_t muxIndex = (cell <= 8) ? 0 : 1;
    uint8_t chan     = (uint8_t)((cell - 1) & 0x7); // 0..7 within that mux

    set_top_mux_channel(muxIndex, chan);

    uint8_t senseGpio = (muxIndex == 0) ? 5 : 7;
    int16_t gpioCode  = 0;

    if (!read_gpio_code_id(1, senseGpio, gpioCode)) {
      Serial.printf("[ntc:id1] Cell%02u: GPIO%u read failed (raw=0x%04X)\n",
                    cell, senseGpio, (uint16_t)gpioCode);
      continue;
    }

    float tC = 0.0f;
    if (!ntc_from_adc_codes(gpioCode, tsrefCode, tC)) {
      Serial.printf("[ntc:id1] Cell%02u: NTC conversion failed (GPIO=%d, TSREF=%d)\n",
                    cell, (int)gpioCode, (int)tsrefCode);
      continue;
    }

    Serial.printf("[ntc:id1] Cell%02u: %.2f °C\n", cell, tC);
  }
}
static bool read_u8(uint8_t addr, uint16_t reg, uint8_t &val) {
  uint8_t rx[7] = {0};
  readFrame(FRMWRT_SGL_R, addr, reg, 1);
  if (!readExact(rx, sizeof(rx), 150)) return false;
  val = rx[4];
  return true;
}

// ---- enable TSREF on a specific device ID ----
static bool enable_tsref_device(uint8_t id) {
  uint8_t c2_old = 0;

  if (!read_u8(id, CONTROL2, c2_old)) {
    Serial.printf("[tsref] read CONTROL2 failed for id%d\n", id);
    return false;
  }

  uint8_t c2_new = c2_old | 0x01;  // set TSREF_EN (bit0)

  // single-device write to this stack address
  writeFrame(FRMWRT_SGL_W, id, CONTROL2, &c2_new, 1);

  // TSREF startup: datasheet says wait ≥1.35 ms
  delayMicroseconds(2000);

  uint8_t c2_verify = 0;
  if (!read_u8(id, CONTROL2, c2_verify)) {
    Serial.printf("[tsref] verify CONTROL2 failed for id%d\n", id);
    return false;
  }

  Serial.printf(
    "[tsref] CONTROL2 id%d: old=0x%02X new=0x%02X readback=0x%02X\n",
    id, c2_old, c2_new, c2_verify
  );

  if (!(c2_verify & 0x01)) {
    Serial.printf("[tsref] WARNING: TSREF_EN bit still 0 for id%d\n", id);
    return false;
  }

  return true;
}

void setup() {
  pinMode(LED_PIN, OUTPUT); digitalWrite(LED_PIN, LOW);
  Serial.begin(115200);
  uint32_t t0 = millis(); while (!Serial && (millis()-t0) < USB_WAIT_MS) {}
  Serial.println("\n[boot] Teensy 4.0 <-> bq79616 (TWO-DEVICE STACK)");

  BQ_Init:
  BMS.begin(UART_BAUD, SERIAL_8N1);
  Serial.println("[boot] UART1 @ 1,000,000 8N1");
  delay(2000);

  // ******** HWRST at boot (affects whole stack) ********
  Serial.println("[boot] HWRST 36 ms...");
  bq_hwrst();
  delay(6); // brief settle

  // WAKE x2 + StA (your timing)
  Serial.println("[boot] WAKE x2 + StA...");
  bq_wake(); delay(12);
  bq_wake(); delay(12);
  bq_sta();  delay(2);

  // Address two devices; roles: base(id0), top(id1)
  Serial.println("[boot] AutoAddress (two, exact)...");
  autoAddress_fixed();
  
  
  delayMicroseconds(1500); // TSREF startup per datasheet (~1.35 ms)

  // Settle
  delayMicroseconds(4000);

  // Prove both are talking (optional)
  if(!read_id_status_id(1)){
    goto BQ_Init;
  }
  
  

  // Start ADC on both (broadcast)
  Serial.println("[adc] Start conversions (both)...");
  adc_start_all();

  // Read & print VCELL for both devices
  float v0[16], v1[16];
  if (read_vcell_id(0, v0)) {
    Serial.println("[vcell:id0] C1..C16 (V)");  
    for (int i=0;i<16;++i) Serial.printf("  C%02d: %.5f\n", i+1, v0[i]);
  } else {
    Serial.println("[vcell:id0] read failed");
  }

  if (read_vcell_id(1, v1)) {
    Serial.println("[vcell:id1] C1..C16 (V)");
    for (int i=0;i<16;++i) Serial.printf("  C%02d: %.5f\n", i+1, v1[i]);
  } else {
    Serial.println("[vcell:id1] read failed");
  }

  // Read die temps for both
  float t10=0,  t11=0;
  if (read_die_temp_id(0, t10)) {
    Serial.printf("[temp:id0] DIETEMP1: %.2f °C\n", t10 );
  } else Serial.println("[temp:id0] DIETEMP read failed");

  if (read_die_temp_id(1, t11)) {
    Serial.printf("[temp:id1] DIETEMP1: %.2f °C\n", t11);
  } else Serial.println("[temp:id1] DIETEMP read failed");

  if (!enable_tsref_device(1)) {
    Serial.println("[tsref] Failed to enable TSREF on top device (id1)");
  }
}

void loop() {
    handle_usb_command();
  float  v1[16];


  if (read_vcell_id(1, v1)) {
    Serial.println("[vcell:id1] C1..C16 (V)");
    for (int i=0;i<16;++i) Serial.printf("  C%02d: %.5f\n", i+1, v1[i]);
  } else {
    Serial.println("[vcell:id1] read failed");
  }

  // Read die temps for both
  float t11=0;
  
  if (read_die_temp_id(1, t11)) {
    Serial.printf("[temp:id1] DIETEMP1: %.2f °C\n", t11);
  } else Serial.println("[temp:id1] DIETEMP read failed");
    // Read 16 thermistors on top device via GPIO muxes
  read_all_ntc_top();

  delay(1500);  // or whatever rate you like

}
