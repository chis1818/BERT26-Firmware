#include <Arduino.h>
#include <SPI.h>
#include <ST7796_t3.h>

/*
  BERT26 APPS calibration sketch

  Shows, for both accelerator pedal channels (APPS 1/2):
    - the raw ADC counts, with the min/max seen since power-up
    - the pedal travel percentage each sensor reports, scaled with the same
      calibration constants and math as VCU_BERT26.ino

  Also shows:
    - DEV: the difference between the two travel percentages (the VCU flags
      an implausibility above 10%)
    - SEP: the separation between the two raw outputs as % of ADC full scale
      (FSAE T.4.2.4 requires >=10% at any pedal position above 10% travel)

  Everything is streamed to the serial monitor at 10 Hz as well.

  Use: press the pedal slowly through full travel a few times, then read the
  min/max for each channel off the screen or serial log. Those are the values
  for APPS_1_RAW_MIN/MAX and APPS_2_RAW_MIN/MAX below and in VCU_BERT26.ino.
  Update the constants here after calibrating so the percentages are right.
  (APPS 1 is the inverted channel: its raw count is highest at rest.)

  BSE 1/2 raws are on the bottom row for calibrating the brake thresholds.

  Send 'r' in the serial monitor to reset the min/max tracking.
*/

// ---------------- Display pins (same wiring as VCU_BERT26) ----------------
static const int TFT_MISO = 12;
static const int TFT_MOSI = 11;
static const int TFT_SCK  = 13;
static const int TFT_DC   = 8;
static const int TFT_CS   = 10;
static const int TFT_RST  = 9;
static const int TFT_LED  = 29;

ST7796_t3 tft(TFT_CS, TFT_DC, TFT_RST);

// ---------------- Pedal pins (same wiring as VCU_BERT26) ----------------
static const int APPS_1_PIN = 38;  // Inverted channel
static const int APPS_2_PIN = 41;  // Normal channel
static const int BSE_1_PIN  = 17;
static const int BSE_2_PIN  = 16;

// Calibration constants, keep in sync with VCU_BERT26.ino.
static const int APPS_1_RAW_MIN = 325;
static const int APPS_1_RAW_MAX = 2400;
static const int APPS_2_RAW_MIN = 2130;
static const int APPS_2_RAW_MAX = 4013;

static const int ADC_RESOLUTION_BITS = 12;
static const int ADC_MAX_COUNTS = (1 << ADC_RESOLUTION_BITS) - 1;

static const uint32_t UPDATE_PERIOD_MS = 100;  // 10 Hz screen + serial

int apps1Min = ADC_MAX_COUNTS;
int apps1Max = 0;
int apps2Min = ADC_MAX_COUNTS;
int apps2Max = 0;

void setup() {
  Serial.begin(115200);

  analogReadResolution(ADC_RESOLUTION_BITS);
  pinMode(APPS_1_PIN, INPUT);
  pinMode(APPS_2_PIN, INPUT);
  pinMode(BSE_1_PIN, INPUT);
  pinMode(BSE_2_PIN, INPUT);

  SPI.setMOSI(TFT_MOSI);
  SPI.setMISO(TFT_MISO);
  SPI.setSCK(TFT_SCK);

  pinMode(TFT_RST, OUTPUT);
  digitalWrite(TFT_RST, HIGH);
  delay(20);
  digitalWrite(TFT_RST, LOW);
  delay(20);
  digitalWrite(TFT_RST, HIGH);
  delay(150);

  pinMode(TFT_LED, OUTPUT);
  analogWrite(TFT_LED, 255);

  tft.init(320, 480);
  tft.setRotation(135);
  tft.fillScreen(ST7735_BLACK);

  drawStaticLabels();

  while (!Serial && millis() < 1500) {}
  Serial.println("BERT26 APPS calibration");
  Serial.println("Send 'r' to reset min/max");
}

void loop() {
  handleSerialInput();

  static uint32_t lastUpdateMs = 0;
  uint32_t now = millis();
  if (now - lastUpdateMs < UPDATE_PERIOD_MS) {
    return;
  }
  lastUpdateMs = now;

  int rawApps1 = analogRead(APPS_1_PIN);
  int rawApps2 = analogRead(APPS_2_PIN);
  int rawBse1 = analogRead(BSE_1_PIN);
  int rawBse2 = analogRead(BSE_2_PIN);

  if (rawApps1 < apps1Min) apps1Min = rawApps1;
  if (rawApps1 > apps1Max) apps1Max = rawApps1;
  if (rawApps2 < apps2Min) apps2Min = rawApps2;
  if (rawApps2 > apps2Max) apps2Max = rawApps2;

  // Same travel scaling as the VCU (unfiltered: no EMA here).
  float apps1Pct = scalePercentInverted(rawApps1, APPS_1_RAW_MIN, APPS_1_RAW_MAX);
  float apps2Pct = scalePercentNormal(rawApps2, APPS_2_RAW_MIN, APPS_2_RAW_MAX);
  float deviationPct = fabsf(apps1Pct - apps2Pct);
  float separationPct = fabsf((float)(rawApps2 - rawApps1)) * 100.0f / (float)ADC_MAX_COUNTS;

  Serial.print("[APPS] raw1=");
  Serial.print(rawApps1);
  Serial.print(" (min ");
  Serial.print(apps1Min);
  Serial.print(" max ");
  Serial.print(apps1Max);
  Serial.print(") pct1=");
  Serial.print(apps1Pct, 1);
  Serial.print("% raw2=");
  Serial.print(rawApps2);
  Serial.print(" (min ");
  Serial.print(apps2Min);
  Serial.print(" max ");
  Serial.print(apps2Max);
  Serial.print(") pct2=");
  Serial.print(apps2Pct, 1);
  Serial.print("% dev=");
  Serial.print(deviationPct, 1);
  Serial.print("% sep=");
  Serial.print(separationPct, 1);
  Serial.print("% | [BSE] raw1=");
  Serial.print(rawBse1);
  Serial.print(" raw2=");
  Serial.println(rawBse2);

  drawValues(rawApps1, rawApps2, apps1Pct, apps2Pct,
             deviationPct, separationPct, rawBse1, rawBse2);
}

void handleSerialInput() {
  while (Serial.available()) {
    char c = (char)Serial.read();
    if (c == 'r' || c == 'R') {
      apps1Min = ADC_MAX_COUNTS;
      apps1Max = 0;
      apps2Min = ADC_MAX_COUNTS;
      apps2Max = 0;
      Serial.println("[APPS] min/max reset");
    }
  }
}

float scalePercentNormal(int rawValue, int rawMin, int rawMax) {
  float scaled = (float)(rawValue - rawMin) * 100.0f / (float)(rawMax - rawMin);
  return constrain(scaled, 0.0f, 100.0f);
}

float scalePercentInverted(int rawValue, int rawMin, int rawMax) {
  float scaled = 100.0f - ((float)(rawValue - rawMin) * 100.0f / (float)(rawMax - rawMin));
  return constrain(scaled, 0.0f, 100.0f);
}

void drawStaticLabels() {
  tft.setTextColor(ST7735_WHITE);
  tft.setTextSize(2);
  tft.setCursor(14, 10);
  tft.print("APPS Calibration");

  tft.setTextColor(ST7735_YELLOW);
  tft.setCursor(14, 45);
  tft.print("APPS 1 (pin 38, inverted)");
  tft.setCursor(14, 130);
  tft.print("APPS 2 (pin 41)");

  tft.setTextColor(ST7735_WHITE);
  tft.setCursor(14, 250);
  tft.print("BSE raw:");

  tft.setTextSize(1);
  tft.setCursor(14, 300);
  tft.print("Serial 115200, 'r' resets min/max. DEV>10% = implausible, SEP<10% above 10% travel = T.4.2.4");
}

void drawValues(int rawApps1, int rawApps2, float apps1Pct, float apps2Pct,
                float deviationPct, float separationPct, int rawBse1, int rawBse2) {
  char text[40];
  uint16_t devColor = (deviationPct > 10.0f) ? ST7735_RED : ST7735_GREEN;

  // APPS 1: raw, travel %, min/max
  snprintf(text, sizeof(text), "%4d", rawApps1);
  drawCachedText(14, 70, 110, 30, 4, text, ST7735_GREEN, 0);
  snprintf(text, sizeof(text), "%5.1f%%", apps1Pct);
  drawCachedText(140, 70, 170, 30, 4, text, devColor, 1);
  snprintf(text, sizeof(text), "min %4d", apps1Min);
  drawCachedText(330, 63, 140, 16, 2, text, ST7735_WHITE, 2);
  snprintf(text, sizeof(text), "max %4d", apps1Max);
  drawCachedText(330, 85, 140, 16, 2, text, ST7735_WHITE, 3);

  // APPS 2: raw, travel %, min/max
  snprintf(text, sizeof(text), "%4d", rawApps2);
  drawCachedText(14, 155, 110, 30, 4, text, ST7735_GREEN, 4);
  snprintf(text, sizeof(text), "%5.1f%%", apps2Pct);
  drawCachedText(140, 155, 170, 30, 4, text, devColor, 5);
  snprintf(text, sizeof(text), "min %4d", apps2Min);
  drawCachedText(330, 148, 140, 16, 2, text, ST7735_WHITE, 6);
  snprintf(text, sizeof(text), "max %4d", apps2Max);
  drawCachedText(330, 170, 140, 16, 2, text, ST7735_WHITE, 7);

  // Channel deviation (implausibility) and raw separation (T.4.2.4 margin).
  snprintf(text, sizeof(text), "DEV %5.1f%%  SEP %5.1f%%", deviationPct, separationPct);
  drawCachedText(14, 215, 360, 18, 2, text, devColor, 8);

  snprintf(text, sizeof(text), "1: %4d   2: %4d", rawBse1, rawBse2);
  drawCachedText(130, 250, 280, 18, 2, text, ST7735_CYAN, 9);
}

// Redraws a text field only when its value changes, to avoid flicker.
// slot selects one of the cached previous-value buffers.
void drawCachedText(int x, int y, int w, int h, uint8_t textSize,
                    const char *text, uint16_t color, int slot) {
  static char lastText[10][40];

  if (slot < 0 || slot >= 10) {
    return;
  }

  if (strcmp(text, lastText[slot]) == 0) {
    return;
  }

  tft.fillRect(x, y, w, h, ST7735_BLACK);
  tft.setTextSize(textSize);
  tft.setTextColor(color);
  tft.setCursor(x, y);
  tft.print(text);

  strncpy(lastText[slot], text, sizeof(lastText[slot]) - 1);
  lastText[slot][sizeof(lastText[slot]) - 1] = '\0';
}
