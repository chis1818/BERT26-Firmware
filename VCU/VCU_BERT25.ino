// Teensy 4.x dashboard – EMA pedals + inverter cmd (11-bit) + iso175 decode + Serial logger
#include <ST7796_t3.h>
#include <XPT2046_Touchscreen.h>
#include <FlexCAN_T4.h>
#include <SPI.h>

/* TFT & touch pins */
#define TFT_MISO  12
#define TFT_MOSI  11
#define TFT_SCK   13
#define TFT_DC     8
#define TFT_CS    10
#define TFT_RST    9
#define CS_PIN     7
#define TIRQ_PIN   6

/* Display & Touch objects */
ST7796_t3            tft(TFT_CS, TFT_DC, TFT_RST);
XPT2046_Touchscreen  ts(CS_PIN, TIRQ_PIN);

/* CAN */
FlexCAN_T4<CAN1, RX_SIZE_64, TX_SIZE_16> Can0;
CAN_message_t rx, tx;

/* Inverter (11-bit) */
static constexpr uint32_t CMD_ID        = 0x0C0;
static constexpr uint32_t RPM_FRAME_ID  = 0x0A5;
static constexpr uint32_t CAN_BAUD      = 500000;
static constexpr uint16_t TORQUE_SCALE  = 10;      // 0.1 N·m / LSB
static constexpr uint16_t CMD_PERIOD_MS = 10;      // 100 Hz
static constexpr float    MPH_PER_RPM   = 0.0185472f;

/* IMD J1939 PGNs */
static constexpr uint32_t PGN_INFO_GENERAL    = 65281; // 0x00FF01
static constexpr uint32_t PGN_INFO_ISO_DETAIL = 65282; // 0x00FF02
static constexpr uint32_t PGN_INFO_VOLTAGE    = 65283; // 0x00FF03

/* EMA */
static constexpr float EMA_ALPHA = 0.4f;
float filtRightPedal = 0.0f, filtLeftPedal = 0.0f;

/* Live data */
volatile float   packVoltage=0, packCurrent=0, powerKW=0;
volatile uint8_t stateOfChargePct=0, batteryTempC=0;
volatile int16_t motorRPM=0;
int   speedMph   = 0;
float desiredNm  = 0.0f;
char  drive_mode = 'N';   // 'D','N','R'

/* iso175 decoded (SNV defaults) */
// 65281
volatile uint16_t imd_riso_corr_kohm = 0xFFFF;
volatile uint8_t  imd_riso_status     = 0xFF;
volatile uint16_t imd_warn_alarms     = 0x0000;
volatile uint8_t  imd_device_activity = 0xFF;
// 65282
volatile uint16_t imd_riso_neg_kohm   = 0xFFFF;
volatile uint16_t imd_riso_pos_kohm   = 0xFFFF;
volatile uint16_t imd_riso_orig_kohm  = 0xFFFF;
volatile uint8_t  imd_quality_pct     = 0xFF;
// 65283 (scale/offset)
static   constexpr int16_t IMD_VOLT_OFFSET = 32128;
static   constexpr float   IMD_VOLT_SCALE  = 0.05f;
volatile int16_t  imd_hv_sys_raw      = IMD_VOLT_OFFSET;
volatile int16_t  imd_hv_neg_e_raw    = IMD_VOLT_OFFSET;
volatile int16_t  imd_hv_pos_e_raw    = IMD_VOLT_OFFSET;

/* Helpers */
char  checkIfTouched(int x,int y);
void  updateScreen(int spd,int pwr,int soc,int battT,char gear);
void  drawIMDPanel();

/* --- J1939 helpers --- */
static inline uint32_t j1939_pgn_from_id(uint32_t id29) { return (id29 >> 8) & 0x3FFFFu; }
static inline uint8_t  j1939_sa_from_id (uint32_t id29) { return id29 & 0xFF; }
static inline uint8_t  j1939_prio_from_id(uint32_t id29) { return (id29 >> 26) & 0x7; }
static inline uint16_t u16_le(uint8_t lo, uint8_t hi) { return (uint16_t)(lo | (uint16_t(hi) << 8)); }
static inline float imd_word_to_volts(uint16_t w) { return (float)((int32_t)w - (int32_t)IMD_VOLT_OFFSET) * IMD_VOLT_SCALE; }

/* --- Serial logger (pretty) --- */
void print_hex_byte(uint8_t b){ if(b<16) Serial.print('0'); Serial.print(b, HEX); }
void dump_payload_hex(const CAN_message_t& m){
  Serial.print(" [");
  for (int i=0;i<m.len;i++){ print_hex_byte(m.buf[i]); if(i< m.len-1) Serial.print(' '); }
  Serial.print("] ");
}
const char* riso_status_str(uint8_t s){
  switch(s){ case 0xFC: return "startup(est)"; case 0xFD: return "startup(first)"; case 0xFE: return "normal"; default: return "SNV"; }
}
const char* dev_activity_str(uint8_t s){
  switch(s){ case 0: return "Init"; case 1: return "Normal"; case 2: return "Self-test"; default: return "SNV"; }
}
void print_warn_bits(uint16_t f){
  Serial.print("Flags{");
  if(f & (1u<<0))  Serial.print("DEV_ERR,");
  if(f & (1u<<1))  Serial.print("HV+_FAIL,");
  if(f & (1u<<2))  Serial.print("HV-_FAIL,");
  if(f & (1u<<3))  Serial.print("EARTH_FAIL,");
  if(f & (1u<<4))  Serial.print("ISO_ALARM,");
  if(f & (1u<<5))  Serial.print("ISO_WARN,");
  if(f & (1u<<6))  Serial.print("ISO_OUTDATED,");
  if(f & (1u<<7))  Serial.print("UNBAL_ALARM,");
  if(f & (1u<<8))  Serial.print("UNDERVOLT,");
  if(f & (1u<<9))  Serial.print("UNSAFE_START,");
  if(f & (1u<<10)) Serial.print("EARTHLIFT_OPEN,");
  Serial.print("}");
}

/* Optional throttle so Serial isn’t flooded (per PGN, ~10 Hz) */
uint32_t lastPrint65281=0, lastPrint65282=0, lastPrint65283=0;
bool throttle_print(uint32_t& tstamp, uint32_t period_ms){ uint32_t now=millis(); if(now - tstamp >= period_ms){ tstamp=now; return true;} return false; }

/* ─────────  SETUP  ───────── */
void setup(){
  Serial.begin(115200);
  while(!Serial && millis()<1000){}  // small wait for USB serial
  Serial.println("\nIMD logger ready (listening only; no requests)");

  Can0.begin();
  Can0.setBaudRate(CAN_BAUD);

  // keep your 11-bit inverter TX
  tx.id = CMD_ID; tx.len = 8; tx.flags.extended = 0; memset(tx.buf, 0, 8);

  ts.begin(); ts.setRotation(1);
  tft.init(320,480); tft.setRotation(135); tft.fillScreen(ST7735_BLACK);
  // updateScreen(0,0,0,0,0);
  drawIMDPanel();
}

/* ─────────  LOOP  ───────── */
void loop(){
  // Pedals (EMA) -----------------------------------
  int rawRight = analogRead(A13);
  int rawLeft  = analogRead(A12);
  static bool firstEMA = true;
  if (firstEMA) { filtRightPedal = rawRight; filtLeftPedal = rawLeft; firstEMA = false; }
  filtRightPedal = EMA_ALPHA * rawRight + (1.0f - EMA_ALPHA) * filtRightPedal;
  filtLeftPedal  = EMA_ALPHA * rawLeft  + (1.0f - EMA_ALPHA) * filtLeftPedal;
  int RightAccelPedal = int(roundf(filtRightPedal));
  int LeftAccelPedal  = int(roundf(filtLeftPedal));
  if (LeftAccelPedal  <= 880) LeftAccelPedal  = 880;
  if (LeftAccelPedal  >= 1023) LeftAccelPedal = 1023;
  if (RightAccelPedal <= 563) RightAccelPedal = 563;
  if (RightAccelPedal >= 613) RightAccelPedal = 613;
  int pedaltravel_APPS2 = map(LeftAccelPedal,  880, 1023, 0, 100);
  desiredNm = map(pedaltravel_APPS2, 0, 100, 0, 73);

  // CAN Rx -----------------------------------------
  while (Can0.read(rx)) {
    if (rx.flags.extended) {
      // J1939 (29-bit)
      const uint32_t pgn  = j1939_pgn_from_id(rx.id);
      const uint8_t  sa   = j1939_sa_from_id(rx.id);
      const uint8_t  prio = j1939_prio_from_id(rx.id);

      // --- Serial: raw header + payload (hex) ---
      if ((pgn==PGN_INFO_GENERAL && throttle_print(lastPrint65281,100)) ||
          (pgn==PGN_INFO_ISO_DETAIL && throttle_print(lastPrint65282,200)) ||
          (pgn==PGN_INFO_VOLTAGE    && throttle_print(lastPrint65283,200))) {
        Serial.print("RX 29b ID=0x"); Serial.print(rx.id, HEX);
        Serial.print("  prio="); Serial.print(prio);
        Serial.print("  PGN=0x"); Serial.print(pgn, HEX);
        Serial.print("  SA=0x");  Serial.print(sa, HEX);
        dump_payload_hex(rx);
        Serial.println();
      }

      // --- Decode & print human-readable ---
      if (pgn == PGN_INFO_GENERAL && rx.len >= 7) {
        uint16_t riso = u16_le(rx.buf[0], rx.buf[1]);          // kΩ (65535 = SNV)
        uint8_t  rstat = rx.buf[2];                            // 0xFC..0xFE, 0xFF=SNV
        uint16_t flags = u16_le(rx.buf[4], rx.buf[5]);         // warnings/alarms
        uint8_t  act   = rx.buf[6];                            // device activity
        if (riso != 0xFFFF) imd_riso_corr_kohm = riso;
        imd_riso_status = rstat; imd_warn_alarms = flags; imd_device_activity = act;

        Serial.print("  -> 65281 Info_General: Riso(corr)=");
        if (riso==0xFFFF) Serial.print("SNV"); else Serial.print(riso);
        Serial.print(" kOhm, Status="); Serial.print(riso_status_str(rstat));
        Serial.print(", Activity="); Serial.print(dev_activity_str(act));
        Serial.print(", "); print_warn_bits(flags);
        Serial.println();
        continue;
      }
      if (pgn == PGN_INFO_ISO_DETAIL && rx.len >= 8) {
        uint16_t rneg = u16_le(rx.buf[0], rx.buf[1]);          // kΩ
        uint16_t rpos = u16_le(rx.buf[2], rx.buf[3]);          // kΩ
        uint16_t rorg = u16_le(rx.buf[4], rx.buf[5]);          // kΩ
        uint8_t  qual = rx.buf[7];                             // %
        if (rneg != 0xFFFF) imd_riso_neg_kohm  = rneg;
        if (rpos != 0xFFFF) imd_riso_pos_kohm  = rpos;
        if (rorg != 0xFFFF) imd_riso_orig_kohm = rorg;
        imd_quality_pct = qual;

        Serial.print("  -> 65282 IsoDetail: Riso-=");
        if (rneg==0xFFFF) Serial.print("SNV"); else Serial.print(rneg);
        Serial.print(" kOhm, Riso+=");
        if (rpos==0xFFFF) Serial.print("SNV"); else Serial.print(rpos);
        Serial.print(" kOhm, Riso(orig)=");
        if (rorg==0xFFFF) Serial.print("SNV"); else Serial.print(rorg);
        Serial.print(" kOhm, Quality=");
        if (qual==0xFF) Serial.print("SNV"); else Serial.print(qual);
        Serial.println("%");
        continue;
      }
      if (pgn == PGN_INFO_VOLTAGE && rx.len >= 7) {
        imd_hv_sys_raw   = (int16_t)u16_le(rx.buf[0], rx.buf[1]);
        imd_hv_neg_e_raw = (int16_t)u16_le(rx.buf[2], rx.buf[3]);
        imd_hv_pos_e_raw = (int16_t)u16_le(rx.buf[4], rx.buf[5]);
        float v_sys = imd_word_to_volts((uint16_t)imd_hv_sys_raw);
        float v_ne  = imd_word_to_volts((uint16_t)imd_hv_neg_e_raw);
        float v_pe  = imd_word_to_volts((uint16_t)imd_hv_pos_e_raw);

        Serial.print("  -> 65283 Voltage: Vsys=");
        Serial.print(v_sys,1); Serial.print(" V, V-2E=");
        Serial.print(v_ne,1);  Serial.print(" V, V+2E=");
        Serial.print(v_pe,1);  Serial.println(" V");
        continue;
      }
      // ignore other extended frames
    } else {
      // 11-bit frames (your BMS/RPM)
      const uint32_t id = rx.id;
      if (id == 0x6B0) {
        packCurrent=(int16_t)((rx.buf[0]<<8)|rx.buf[1])*0.1f;
        packVoltage=((rx.buf[2]<<8)|rx.buf[3])*0.1f;
        stateOfChargePct=uint8_t(rx.buf[5]*0.5f);
      }
      else if (id == 0x6B1) {
        batteryTempC=rx.buf[4];
      }
      else if (id == RPM_FRAME_ID && rx.len >= 4) {
        motorRPM=(int16_t)(rx.buf[2]|(rx.buf[3]<<8));
      }
    }
  }

  powerKW=(packVoltage*packCurrent)/1000.0f;
  speedMph=(int)roundf(motorRPM * MPH_PER_RPM);

  // Heartbeat to inverter (no change) -----------------------------
  static uint32_t lastCmd = 0; static bool firstFrame = true;
  if (millis() - lastCmd >= CMD_PERIOD_MS) {
    lastCmd = millis();
    bool active = (drive_mode == 'D' || drive_mode == 'R');
    float tqCmdNm = active ? fabs(desiredNm) : 0.0f;
    int16_t tqRaw = (int16_t)roundf(tqCmdNm * TORQUE_SCALE);
    tx.buf[0] = tqRaw & 0xFF;
    tx.buf[1] = (tqRaw >> 8) & 0xFF;
    tx.buf[4] = (drive_mode == 'R') ? 0 : 1;
    tx.buf[5] = firstFrame ? 0x00 : (active ? 0x01 : 0x00);
    firstFrame = false;
    Can0.write(tx);
  }

  // Touch & screen -----------------------------------------------
  TS_Point p=ts.getPoint(); char newMode=checkIfTouched(p.x,p.y); if(newMode) drive_mode=newMode;

  static uint32_t lastDraw=0; 
  if(millis()-lastDraw>=400){
    // updateScreen((int)desiredNm, int(roundf(powerKW)), stateOfChargePct, batteryTempC, drive_mode);
    drawIMDPanel();
    lastDraw=millis();
  }
}

/* Touch */
char checkIfTouched(int x,int y){ const int y_min=2650,y_max=3500;
  if(x> 950&&x<1550&&y>y_min&&y<y_max) return 'D';
  if(x>1900&&x<2500&&y>y_min&&y<y_max) return 'N';
  if(x>2900&&x<3500&&y>y_min&&y<y_max) return 'R';
  return 0;
}

/* Your existing UI code (unchanged, trimmed) */
void updateScreen(int speed, int power, int battery_percent, int battery_temp, char drive_mode){
  tft.setTextSize(6);
  tft.setCursor(tft.width() / 4 + 20, tft.height() / 2 - 30);
  tft.setTextColor(ST7735_WHITE, ST7735_BLACK);
  char speed_str[12]; snprintf(speed_str, sizeof(speed_str), "%d mph ", speed);
  tft.println(speed_str);

  tft.setCursor(tft.width() * 0.05, tft.height() * 0.05);
  tft.setTextSize(2);
  char power_str[18];
  snprintf(power_str, sizeof(power_str), "Power:%d kW ", power);
  tft.print(power_str);

  tft.fillRect(tft.width() * 0.6, tft.height() * 0.05, 120, 15, ST7735_BLACK);
  tft.fillRoundRect(tft.width() * 0.85, tft.height() * 0.05, tft.width() * 0.1, 20, 4, ST7735_WHITE);
  tft.fillRoundRect(tft.width() * 0.85 + tft.width() * 0.1 + 2, tft.height() * 0.05 + 4, 4, 12, 1, ST7735_WHITE);
  battery_percent = constrain(battery_percent, 0, 100);
  uint16_t barColor = (battery_percent < 15) ? ST7735_RED : (battery_percent < 40) ? ST7735_YELLOW : ST7735_GREEN;
  int barWidth = constrain(40 * battery_percent / 100, 3, 40);
  tft.fillRoundRect(tft.width() * 0.85 + 4, tft.height() * 0.05 + 4, barWidth, 12, 1, barColor);
  tft.setCursor(tft.width() * 0.75 - 6, tft.height() * 0.05);
  tft.setTextSize(2);
  tft.printf("%d%%", battery_percent);

  tft.setCursor(tft.width() * 0.05, tft.height() * 0.11);
  tft.setTextSize(2);
  tft.print("Highest Cell Temp: ");
  tft.printf("%d", battery_temp);
  tft.setTextSize(1); tft.print(" o");
  tft.setTextSize(2); tft.print("C");

  tft.setTextSize(5);
  tft.setCursor(tft.width() / 4, tft.height() * 0.7);
  tft.setTextColor((drive_mode == 'D') ? ST7735_RED : ST7735_WHITE); tft.print("D");
  tft.setCursor(tft.width() / 2, tft.height() * 0.7);
  tft.setTextColor((drive_mode == 'N') ? ST7735_RED : ST7735_WHITE); tft.print("N");
  tft.setCursor(tft.width() * 0.75, tft.height() * 0.7);
  tft.setTextColor((drive_mode == 'R') ? ST7735_RED : ST7735_WHITE); tft.print("R");
  tft.setTextColor(ST7735_WHITE);
}

/* IMD panel (uses the decoded globals) */
static inline bool flag(uint16_t f, uint8_t bit){ return (f & (1u<<bit)) != 0; }
void drawIMDPanel(){
  int x = tft.width() * 0.05;
  int y = tft.height() * 0.17;
  int w = tft.width() * 0.9;
  int h = tft.height() * 0.45;
  tft.fillRect(x, y, w, h, ST7735_BLACK);

  // Severity header color
  bool f_iso_alarm   = flag(imd_warn_alarms, 4);
  bool f_iso_warn    = flag(imd_warn_alarms, 5);
  bool f_dev_error   = flag(imd_warn_alarms, 0);
  bool f_unsafe      = flag(imd_warn_alarms, 9);
  uint16_t headColor = f_iso_alarm || f_unsafe || f_dev_error ? ST7735_RED : (f_iso_warn ? ST7735_YELLOW : ST7735_WHITE);

  tft.setCursor(x, y); tft.setTextSize(2); tft.setTextColor(headColor); tft.print("IMD (iso175)"); tft.setTextColor(ST7735_WHITE);

  y += 24; tft.setCursor(x, y);
  tft.print("Riso(corr): "); if (imd_riso_corr_kohm==0xFFFF) tft.print("-- kOhm"); else tft.printf("%u kOhm",(unsigned)imd_riso_corr_kohm);
  tft.setCursor(x + w*0.55, y); tft.print("Status: "); 
  switch(imd_riso_status){ case 0xFC: tft.print("startup(est)"); break; case 0xFD: tft.print("startup(first)"); break; case 0xFE: tft.print("normal"); break; default: tft.print("SNV"); }

  y += 20; tft.setCursor(x, y); tft.print("Device: ");
  switch(imd_device_activity){ case 0: tft.print("Initialization"); break; case 1: tft.print("Normal"); break; case 2: tft.print("Self-test"); break; default: tft.print("SNV"); }

  y += 22; tft.setCursor(x, y);
  tft.print("Riso- : "); if (imd_riso_neg_kohm==0xFFFF) tft.print("--"); else tft.printf("%u",(unsigned)imd_riso_neg_kohm);
  tft.print(" kOhm   Riso+ : "); if (imd_riso_pos_kohm==0xFFFF) tft.print("--"); else tft.printf("%u",(unsigned)imd_riso_pos_kohm); tft.print(" kOhm");

  y += 20; tft.setCursor(x, y);
  tft.print("Riso(orig): "); if (imd_riso_orig_kohm==0xFFFF) tft.print("-- kOhm"); else tft.printf("%u kOhm",(unsigned)imd_riso_orig_kohm);
  tft.print("   Qual: "); if (imd_quality_pct==0xFF) tft.print("--%"); else tft.printf("%u%%",(unsigned)imd_quality_pct);

  float v_sys = imd_word_to_volts((uint16_t)imd_hv_sys_raw);
  float v_ne  = imd_word_to_volts((uint16_t)imd_hv_neg_e_raw);
  float v_pe  = imd_word_to_volts((uint16_t)imd_hv_pos_e_raw);
  y += 20; tft.setCursor(x, y); tft.printf("Vsys: %.1fV   V-2E: %.1fV   V+2E: %.1fV", v_sys, v_ne, v_pe);

  y += 24; tft.setCursor(x, y); tft.print("Flags: ");
  y += 18; tft.setCursor(x, y);
  tft.setTextColor(flag(imd_warn_alarms,5)?ST7735_YELLOW:ST7735_WHITE); tft.print("ISO_WARN  ");
  tft.setTextColor(flag(imd_warn_alarms,4)?ST7735_RED   :ST7735_WHITE); tft.print("ISO_ALARM  ");
  tft.setTextColor(flag(imd_warn_alarms,9)?ST7735_RED   :ST7735_WHITE); tft.print("UNSAFE_START  ");
  y += 18; tft.setCursor(x, y);
  tft.setTextColor(flag(imd_warn_alarms,0)?ST7735_RED   :ST7735_WHITE); tft.print("DEV_ERR  ");
  tft.setTextColor(flag(imd_warn_alarms,8)?ST7735_RED   :ST7735_WHITE); tft.print("UNDERVOLT  ");
  tft.setTextColor(flag(imd_warn_alarms,6)?ST7735_YELLOW:ST7735_WHITE); tft.print("ISO_OUTDATED  ");
  y += 18; tft.setCursor(x, y);
  tft.setTextColor(flag(imd_warn_alarms,1)?ST7735_YELLOW:ST7735_WHITE); tft.print("HV+_FAIL  ");
  tft.setTextColor(flag(imd_warn_alarms,2)?ST7735_YELLOW:ST7735_WHITE); tft.print("HV-_FAIL  ");
  tft.setTextColor(flag(imd_warn_alarms,3)?ST7735_YELLOW:ST7735_WHITE); tft.print("EARTH_FAIL  ");
  y += 18; tft.setCursor(x, y);
  tft.setTextColor(flag(imd_warn_alarms,10)?ST7735_YELLOW:ST7735_WHITE); tft.print("EARTHLIFT_OPEN");
  tft.setTextColor(ST7735_WHITE);
}
