// Requires: ESP32-TWAI-CAN by handmade-intelligence (Library Manager → search "ESP32-TWAI-CAN")
#include <WiFi.h>
#include <WebServer.h>
#include <DNSServer.h>
#include <ESP32-TWAI-CAN.hpp>

// ── CAN3 hardware (per CAN_MESSAGES.md: TX=43, RX=44, 500 kbps, vehicle bus) ──
#define CAN_TX_PIN          44
#define CAN_RX_PIN          43
#define CAN_SPEED           TWAI_SPEED_500KBPS

// ── CAN message IDs ───────────────────────────────────────────────────────────
#define ID_BMS_STATUS       0x310          // 11-bit standard, 500 ms
#define ID_BMS_TEMP         0x311          // 11-bit standard, 500 ms
#define ID_CELL_REQUEST     0x312          // ESP32 → BMS: request cell dump
#define ID_CELL_DUMP        0x313          // BMS → ESP32: cell dump response
#define ID_CHARGER          0x18FF50E5UL   // 29-bit extended, 1 s

// ── Cell dump config ──────────────────────────────────────────────────────────
#define CELL_COUNT          96    // 6 ICs × 16 cells
#define DUMP_FRAMES_V       32    // voltage:  32 frames × 3 cells = 96 cells
#define DUMP_FRAMES_T       14    // temp:     14 frames × 7 cells = 98 slots (last 2 = padding)
#define CELL_DUMP_INTERVAL  5000  // alternate voltage/temp request every 5 s

static uint16_t cell_mv[CELL_COUNT];        // mV; 0xFFFF = not yet received / invalid
static int8_t   cell_temp_c[CELL_COUNT];    // °C decoded; INT8_MIN = not yet received / no sensor
static bool     cell_v_valid    = false;
static bool     cell_t_valid    = false;
static uint8_t  active_dump     = 0;        // 1 = voltage frames incoming, 2 = temp frames incoming
static uint32_t dump_start_ms   = 0;        // millis() when active_dump was last (re)requested

// ── Scalar signals ────────────────────────────────────────────────────────────
enum {
  SIG_BMS_FLAGS,       // 0x310 byte 0  — bit0=fault, bit1=contactors, bit2=charge
  SIG_CELL_V_MIN,      // 0x310 bytes 1-2  mV → V
  SIG_CELL_V_MAX,      // 0x310 bytes 3-4  mV → V
  SIG_CELL_V_AVG,      // 0x310 bytes 5-6  mV → V
  SIG_SOC,             // 0x310 byte 7   %
  SIG_TEMP_MIN,        // 0x311 byte 0   °C (-1 = no sensor)
  SIG_TEMP_MAX,        // 0x311 byte 1   °C
  SIG_TEMP_AVG,        // 0x311 byte 2   °C
  SIG_FAULT_CODE,      // 0x311 byte 3   (see CAN_MESSAGES.md for codes)
  SIG_PACK_CURRENT,    // 0x311 bytes 4-5  A (negative = regen/charging)
  SIG_CHARGER_VOLTAGE, // 0x18FF50E5 bytes 0-1  V
  SIG_CHARGER_CURRENT, // 0x18FF50E5 bytes 2-3  A
  SIG_CHARGER_FLAGS,   // 0x18FF50E5 byte 4  — bit0=HW fault, bit1=temp, bit2=AC, bit3=no AC, bit4=timeout
  SIGNAL_COUNT
};

const char* signal_names[SIGNAL_COUNT] = {
  "BmsFlags",
  "CellVMin",
  "CellVMax",
  "CellVAvg",
  "SOC",
  "TempMin",
  "TempMax",
  "TempAvg",
  "FaultCode",
  "PackCurrent",
  "ChargerVoltage",
  "ChargerCurrent",
  "ChargerFlags"
};

double signals[SIGNAL_COUNT];

// ── Wi-Fi AP ──────────────────────────────────────────────────────────────────
const char* SSID     = "BERT.26";
const char* PASSWORD = "BERT2026";
const byte  DNS_PORT = 53;

DNSServer dnsServer;
WebServer server(80);
IPAddress local_ip(192, 168, 1, 1);
IPAddress gateway(192, 168, 1, 1);
IPAddress subnet(255, 255, 255, 0);

extern const char* base_html;
extern const char* config_html;

// ── CAN TX: request a cell dump (type 1 = voltage, type 2 = temperature) ─────
void requestCellDump(uint8_t type) {
  active_dump = type;
  dump_start_ms = millis();
  CanFrame frame = {};
  frame.identifier       = ID_CELL_REQUEST;
  frame.extd             = 0;
  frame.data_length_code = 1;
  frame.data[0]          = type;
  ESP32Can.writeFrame(frame);
}

// ── CAN RX: decode incoming frames ───────────────────────────────────────────
void processCANMessage(long id, bool extended, uint8_t* d, int len) {

  if (!extended) {
    switch ((uint16_t)id) {

      case ID_BMS_STATUS:  // 0x310 — flags, min/max/avg cell voltage, SOC
        if (len < 8) break;
        signals[SIG_BMS_FLAGS]  = d[0];
        signals[SIG_CELL_V_MIN] = (uint16_t)(d[1] << 8 | d[2]) * 0.001;
        signals[SIG_CELL_V_MAX] = (uint16_t)(d[3] << 8 | d[4]) * 0.001;
        signals[SIG_CELL_V_AVG] = (uint16_t)(d[5] << 8 | d[6]) * 0.001;
        signals[SIG_SOC]        = d[7];
        break;

      case ID_BMS_TEMP:  // 0x311 — temps, fault code, pack current
        if (len < 6) break;
        // temp bytes use +40 °C offset; 0xFF means no sensor
        signals[SIG_TEMP_MIN]     = (d[0] != 0xFF) ? (int)d[0] - 40 : -1;
        signals[SIG_TEMP_MAX]     = (d[1] != 0xFF) ? (int)d[1] - 40 : -1;
        signals[SIG_TEMP_AVG]     = (d[2] != 0xFF) ? (int)d[2] - 40 : -1;
        signals[SIG_FAULT_CODE]   = d[3];
        // pack_current: int16 with 3000 bias, 0.1 A/bit  →  A = (raw - 3000) × 0.1
        signals[SIG_PACK_CURRENT] = ((int16_t)((uint16_t)d[4] << 8 | d[5]) - 3000) * 0.1;
        break;

      case ID_CELL_DUMP:  // 0x313 — voltage OR temperature dump response
        if (len < 2) break;
        if (active_dump == 1) {
          // Voltage dump: 32 frames, bytes 1-6 = 3 × uint16 mV, frame_index × 3 = base cell
          if (len < 7) break;
          uint8_t fi = d[0];
          if (fi >= DUMP_FRAMES_V) break;
          int base = fi * 3;
          for (int c = 0; c < 3 && (base + c) < CELL_COUNT; c++) {
            cell_mv[base + c] = (uint16_t)d[1 + c * 2] << 8 | d[2 + c * 2];
          }
          if (fi == DUMP_FRAMES_V - 1) cell_v_valid = true;
        } else if (active_dump == 2) {
          // Temp dump: 14 frames, bytes 1-7 = 7 × uint8 (°C + 40 offset), frame_index × 7 = base cell
          uint8_t fi = d[0];
          if (fi >= DUMP_FRAMES_T) break;
          int base = fi * 7;
          for (int c = 0; c < 7 && (base + c) < CELL_COUNT; c++) {
            uint8_t raw = d[1 + c];
            cell_temp_c[base + c] = (raw != 0xFF) ? (int8_t)(raw - 40) : INT8_MIN;
          }
          if (fi == DUMP_FRAMES_T - 1) cell_t_valid = true;
        }
        break;
    }
    return;
  }

  // Extended frame
  if ((unsigned long)id == ID_CHARGER) {  // 0x18FF50E5 — charger status
    if (len < 5) return;
    signals[SIG_CHARGER_VOLTAGE] = (uint16_t)(d[0] << 8 | d[1]) * 0.1;
    signals[SIG_CHARGER_CURRENT] = (uint16_t)(d[2] << 8 | d[3]) * 0.1;
    signals[SIG_CHARGER_FLAGS]   = d[4];
  }
}

// ── CAN TX: Set SOC (0x314) and current calibration point (0x315) ────────────
void sendSOC(float soc_pct) {
  uint16_t val = (uint16_t)constrain(soc_pct * 10.0f, 0.0f, 1000.0f);
  CanFrame f = {};
  f.identifier = 0x314; f.extd = 0; f.data_length_code = 2;
  f.data[0] = (uint8_t)(val >> 8);
  f.data[1] = (uint8_t)(val & 0xFF);
  ESP32Can.writeFrame(f);
}

void sendCalPoint(uint8_t point, float amps) {
  int16_t cur = (int16_t)constrain(amps * 10.0f, -32768.0f, 32767.0f);
  CanFrame f = {};
  f.identifier = 0x315; f.extd = 0; f.data_length_code = 3;
  f.data[0] = point;
  f.data[1] = (uint8_t)((uint16_t)cur >> 8);
  f.data[2] = (uint8_t)((uint16_t)cur & 0xFF);
  ESP32Can.writeFrame(f);
}

// ── HTTP helpers ──────────────────────────────────────────────────────────────
void sendLargeHTML(const char* data) {
  server.setContentLength(CONTENT_LENGTH_UNKNOWN);
  server.send(200, "text/html", "");
  const size_t chunkSize = 16184;
  size_t length = strlen(data);
  for (size_t i = 0; i < length; i += chunkSize) {
    size_t n = (length - i < chunkSize) ? length - i : chunkSize;
    server.sendContent(data + i, n);
  }
  server.sendContent("");
}

void HTTP_handle_root()   { sendLargeHTML(base_html); }
void HTTP_handle_config() { sendLargeHTML(config_html); }

void HTTP_handle_set_soc() {
  String s = server.arg("soc");
  float v = s.toFloat();
  if (s.length() == 0 || v < 0.0f || v > 100.0f) {
    server.send(400, "application/json", "{\"ok\":false,\"error\":\"range\"}");
    return;
  }
  sendSOC(v);
  server.send(200, "application/json", "{\"ok\":true}");
}

void HTTP_handle_set_cal() {
  String ps = server.arg("point");
  String cs = server.arg("current");
  uint8_t pt = (uint8_t)ps.toInt();
  if (pt < 1 || pt > 2 || cs.length() == 0) {
    server.send(400, "application/json", "{\"ok\":false,\"error\":\"args\"}");
    return;
  }
  sendCalPoint(pt, cs.toFloat());
  server.send(200, "application/json", "{\"ok\":true}");
}

void HTTP_handle_data() {
  server.setContentLength(CONTENT_LENGTH_UNKNOWN);
  server.send(200, "application/json", "");

  // Scalar signals
  String json = "{\"timeSent\":" + String(millis()) + ",";
  for (int i = 0; i < SIGNAL_COUNT; i++) {
    json += "\"" + String(signal_names[i]) + "\":" + String(signals[i], 3) + ",";
  }

  // Cell voltages (mV); empty array until first complete voltage dump
  json += "\"cellVoltages\":[";
  server.sendContent(json);
  if (cell_v_valid) {
    String chunk = "";
    for (int c = 0; c < CELL_COUNT; c++) {
      if (c > 0) chunk += ",";
      chunk += String(cell_mv[c]);
      if (chunk.length() > 300) { server.sendContent(chunk); chunk = ""; }
    }
    server.sendContent(chunk);
  }

  // Cell temperatures (°C); null for sensors that returned 0xFF
  server.sendContent("],\"cellTemps\":[");
  if (cell_t_valid) {
    String chunk = "";
    for (int c = 0; c < CELL_COUNT; c++) {
      if (c > 0) chunk += ",";
      chunk += (cell_temp_c[c] != INT8_MIN) ? String(cell_temp_c[c]) : "null";
      if (chunk.length() > 300) { server.sendContent(chunk); chunk = ""; }
    }
    server.sendContent(chunk);
  }

  server.sendContent("]}");
  server.sendContent("");
}

// ── Setup ─────────────────────────────────────────────────────────────────────
void setup() {
  Serial.begin(115200);

  memset(signals, 0, sizeof(signals));
  memset(cell_mv,     0xFF, sizeof(cell_mv));      // 0xFFFF = not yet received
  memset(cell_temp_c, INT8_MIN, sizeof(cell_temp_c));

  // rxQueue=64: large enough to absorb a full 32-frame cell-voltage dump
  // (sent 1 frame/5ms by the BMS) even if loop() is briefly stalled by an
  // HTTP response. Default queue depth (5) was dropping the tail of the
  // longer voltage dumps, so cell_v_valid (set on the last frame) never latched.
  if (!ESP32Can.begin(CAN_SPEED, CAN_TX_PIN, CAN_RX_PIN, 0xFFFF, 64)) {
    Serial.println("CAN init failed — check wiring on TX=43 / RX=44");
  } else {
    Serial.println("CAN3 ready  500 kbps  TX=43 RX=44");
  }

  WiFi.mode(WIFI_AP);
  WiFi.softAP(SSID, PASSWORD);
  delay(50);
  WiFi.softAPConfig(local_ip, gateway, subnet);
  Serial.print("AP IP: ");
  Serial.println(WiFi.softAPIP());

  dnsServer.start(DNS_PORT, "*", WiFi.softAPIP());
  server.on("/", HTTP_handle_root);
  server.on("/data", HTTP_handle_data);
  server.on("/config", HTTP_handle_config);
  server.on("/set_soc", HTTP_handle_set_soc);
  server.on("/set_cal", HTTP_handle_set_cal);
  server.begin();
  Serial.println("HTTP server started");
}

// ── Loop ──────────────────────────────────────────────────────────────────────
void loop() {
  dnsServer.processNextRequest();

  // Alternate voltage (type 1) and temperature (type 2) dump requests every 5 s
  // Each type therefore refreshes every 10 s
  static uint32_t lastDumpReq  = 0;
  static uint8_t  nextDumpType = 1;
  uint32_t now = millis();
  if (now - lastDumpReq >= CELL_DUMP_INTERVAL) {
    lastDumpReq = now;
    requestCellDump(nextDumpType);
    nextDumpType = (nextDumpType == 1) ? 2 : 1;
  }

  // Retry an incomplete dump (e.g. dropped CAN frames) after 1 s instead of
  // waiting for the next scheduled alternation.
  bool dump_pending = (active_dump == 1 && !cell_v_valid) ||
                      (active_dump == 2 && !cell_t_valid);
  if (dump_pending && (now - dump_start_ms >= 1000)) {
    requestCellDump(active_dump);
  }

  // Drain all pending CAN frames (timeout=0 → non-blocking)
  CanFrame frame;
  while (ESP32Can.readFrame(frame, 0)) {
    processCANMessage(frame.identifier, frame.extd,
                      frame.data, frame.data_length_code);
  }

  server.handleClient();
}
