/*
  ASTRO-GUARD  -  Astronaut Health Monitor  (NASA Space Apps 2026)
  Board : ESP32 DevKit V1
  Parts : SSD1306 OLED, 16x2 I2C LCD, MAX30102, DS18B20, MQ-2, Flame sensor,
          Active buzzer, 3 push buttons, MPU-6050 (GY-521)

  Libraries (Library Manager):
    Adafruit SSD1306, Adafruit GFX, LiquidCrystal_I2C (Frank de Brabander),
    OneWire, DallasTemperature, SparkFun MAX3010x Pulse and Proximity Sensor Library
    (MPU-6050 needs NO extra library - it is read directly over I2C)

  Buttons:  B1 = next page   B2 = previous page / choose value (check-in)   B3 = OK / silence alarm
            On a FALL alert, B3 = "I'M OK" (clears the alert)

  DASHBOARD LINK (115200 baud, both directions):
    ESP32 -> page, once per second:
      {"hr":72,"spo2":98,"temp":36.8,"gas":12,"flame":0,"flame_ao":0}
    page -> ESP32, whenever the astronaut logs something in "Daily check-in":
      {"bg":110}              blood sugar in mg/dL
      {"mood":"stressed"}     optimal | neutral | fatigued | stressed
      {"ok":1}               astronaut pressed "I'M OK" on the dashboard (clears a fall alert)
    ESP32 -> page, 5 times per second (MPU-6050 motion card):
      {"mpu":1,"pitch":3.2,"roll":-1.5,"g":1.01,"gmin":0.98,"gmax":1.06,"dyn":0.020,"gyro":4,"fall":0}
    The ESP32 uses these to raise short alerts on the suit OLED/LCD/buzzer,
    exactly like the dashboard does (same thresholds).

  BLUETOOTH (HC-05 on Serial2, 9600 baud): the same JSON lines are sent to the phone, and the
  phone can send the same commands back ({"bg":110}, {"mood":"stressed"}, {"ok":1}).

  NOT A MEDICAL DEVICE - prototype for a hackathon.
*/
#include <Wire.h>
#include <math.h>
#include <Adafruit_GFX.h>
#include <Adafruit_SSD1306.h>
#include <LiquidCrystal_I2C.h>
#include <OneWire.h>
#include <DallasTemperature.h>
#include <Preferences.h>
#include "MAX30105.h"
#include "heartRate.h"
#include "spo2_algorithm.h"

// ================= PINS =================
#define PIN_SDA     21   // shared I2C bus: OLED, LCD, MAX30102, MPU-6050
#define PIN_SCL     22
#define PIN_DS18    4
#define PIN_MQ2     34   // via voltage divider!
#define PIN_FLAME_D 27
#define PIN_FLAME_A 35
#define PIN_BUZZER  26
#define PIN_B1      32
#define PIN_B2      33
#define PIN_B3      25
// HC-05 Bluetooth: VCC->5V, GND->GND, HC05 TXD->GPIO16 (RX2), HC05 RXD<-GPIO17 (TX2)
#define PIN_BT_RX   16
#define PIN_BT_TX   17
#define BT_BAUD     9600         // HC-05 factory data-mode baud rate
#define BT_MOTION_MS 500         // motion line to the phone 2x/s (9600 baud can't carry 5x/s)
// MPU-6050: VCC->3V3, GND->GND, SDA->21, SCL->22, AD0->GND (address 0x68), INT not used

// ================= SETTINGS =================
#define BASELINE_SECS   8        // MQ2 calibration time at boot (clean air!)
#define REMIND_MS       (15UL*60UL*1000UL)  // mission routine reminder (set to 60000 for demo)
#define TEMP_OFFSET     0.0f     // calibrate against a real thermometer
#define FLAME_ACTIVE    HIGH      // most flame modules give LOW when fire is detected
#define HIST            53       // HR trend samples
#define HIST_PERIOD_MS  3000

// ---- motion / fall detection (tune these on the real device) ----
#define FALL_FREEFALL_G 0.45f    // total acceleration below this = free-fall
#define FALL_IMPACT_G   2.5f     // spike after free-fall = impact
#define FALL_HARD_G     3.5f     // very hard hit even without a seen free-fall
#define FALL_STILL_PCT  70       // % of the 2 s after impact that must be "still"
#define MOVE_DYN        0.15f    // smoothed |g-1| above this = moving / active

// ---- blood sugar (mg/dL), same limits as the dashboard ----
#define BG_CRIT_LOW     55       // below  -> danger
#define BG_WARN_LOW     70       // below  -> warning
#define BG_WARN_HIGH    140      // above  -> warning
#define BG_CRIT_HIGH    200      // above  -> danger
#define STRESS_HR       100      // "Stressed" mood + HR above this = stress alert

Adafruit_SSD1306 oled(128, 64, &Wire, -1);
LiquidCrystal_I2C *lcd = nullptr;
MAX30105 particleSensor;
OneWire oneWire(PIN_DS18);
DallasTemperature ds(&oneWire);
Preferences prefs;

bool oledOk = false, lcdOk = false, maxOk = false, mpuOk = false;
uint8_t mpuAddr = 0x68;

// ================= DATA =================
struct Vitals {
  int hr = 0, spo2 = 0;
  bool hrValid = false, spoValid = false, finger = false;
  float temp = NAN; bool tempOk = false, onBody = false;
  float gasEma = 0, gasRatio = 1; int gasRaw = 0;
  int flameInt = 0; bool fire = false;
  bool fall = false;                       // latched until B3 is pressed
} V;

struct Motion {
  float ax = 0, ay = 0, az = 1;            // smoothed acceleration (g)
  float g = 1;                             // instantaneous total acceleration (g)
  float gmag = 0, gEma = 0;                // gyro magnitude (deg/s), raw and smoothed
  float dyn = 0;                           // smoothed |g-1| = how much we are moving
  float pitch = 0, roll = 0;               // degrees
  bool moving = false;
} M;

float gasBase = 300;
float mGMin = 9, mGMax = 0;                // min / max total g since the last motion line was sent
uint32_t beatT = 0; bool lcdBeat = false;

// MAX30102 buffers
uint32_t irBuf[100], redBuf[100];
int filled = 0, newCount = 0;
uint32_t lastHrOk = 0, lastSpoOk = 0;

// history
int hist[HIST]; int histCount = 0; uint32_t lastHist = 0;

// ---- dashboard check-in data (received over serial) ----
enum { DM_NONE, DM_OPTIMAL, DM_NEUTRAL, DM_FATIGUED, DM_STRESSED };
int dMood = DM_NONE;                       // mood chosen on the dashboard
int bgVal = 0;                             // last blood sugar in mg/dL (0 = none logged)
uint32_t bgT = 0;                          // when it was logged
bool stress = false;                       // sensor fusion: stressed mood + high pulse
char rxBuf[64]; uint8_t rxLen = 0;        // USB serial line buffer
char btBuf[64]; uint8_t btLen = 0;        // Bluetooth (HC-05) line buffer

// status
int lHR = 0, lSp = 0, lT = 0, lG = 0, lBG = 0, alertLevel = 0, score = 100;
enum { SRC_NONE, SRC_HEALTH, SRC_GAS, SRC_FIRE, SRC_FALL, SRC_SUGAR };
int alertSrc = SRC_NONE;
const char *aMsg1 = "", *aMsg2 = "";
uint32_t silenceUntil = 0, chirpUntil = 0;

// fall detection state machine
enum { FS_IDLE, FS_FREEFALL, FS_IMPACT };
int fState = FS_IDLE; uint32_t fStateT = 0; int lowCnt = 0;
uint32_t stillOk = 0, stillTot = 0;
uint32_t activeSecs = 0;

// gamification
uint32_t xp = 0, scans = 0, logs = 0;
bool measuring = false, measured = false; uint32_t measureStart = 0;
uint32_t lastMeasure = 0; bool reminderActive = false;

// check-in
int ciStage = 0, ciMood = 1; float ciSleep = 7.0; bool logged = false;

// UI
enum { PG_HOME, PG_VITALS, PG_ENV, PG_MOTION, PG_TREND, PG_CHECKIN, PG_CREW, PAGES };
int page = 0; uint32_t pageT = 0;

// ================= BUTTONS =================
struct Btn { uint8_t pin; bool last; bool pressed; uint32_t t; };
Btn btns[3] = {{PIN_B1, HIGH, false, 0}, {PIN_B2, HIGH, false, 0}, {PIN_B3, HIGH, false, 0}};
void readButtons() {
  uint32_t now = millis();
  for (int i = 0; i < 3; i++) {
    bool s = digitalRead(btns[i].pin);
    if (s != btns[i].last && now - btns[i].t > 30) {
      btns[i].t = now; btns[i].last = s;
      if (s == LOW) btns[i].pressed = true;
    }
  }
}
bool pressed(int i) { bool p = btns[i].pressed; btns[i].pressed = false; return p; }
void chirp(uint32_t ms) { chirpUntil = millis() + ms; }

// ================= LEVELS =================
int lvlHR(int h)   { if (h >= 60 && h <= 100) return 0; if ((h >= 50 && h < 60) || (h > 100 && h <= 120)) return 1; return 2; }
int lvlSpo(int s)  { if (s >= 95) return 0; if (s >= 90) return 1; return 2; }
int lvlTemp(float t) { if (t >= 36.0 && t <= 37.5) return 0; if ((t >= 35.0 && t < 36.0) || (t > 37.5 && t <= 38.4)) return 1; return 2; }
int lvlGas(float r) { if (r < 1.6) return 0; if (r < 2.6) return 1; return 2; }
int lvlBG(int v)   { if (v <= 0) return 0; if (v < BG_CRIT_LOW || v > BG_CRIT_HIGH) return 2; if (v < BG_WARN_LOW || v > BG_WARN_HIGH) return 1; return 0; }

// ================= DASHBOARD INPUT (serial RX) =================
// Accepts one JSON-ish line per message, e.g. {"bg":110}  or  {"mood":"stressed"}
void parseCommand(const char *s) {
  const char *p = strstr(s, "\"bg\"");
  if (p) {
    p = strchr(p, ':');
    if (p) {
      int v = atoi(p + 1);
      if (v >= 20 && v <= 600) { bgVal = v; bgT = millis(); chirp(60); }
    }
  }
  if (strstr(s, "\"ok\"")) { V.fall = false; chirp(60); }   // "I'M OK" pressed on the dashboard
  p = strstr(s, "\"mood\"");
  if (p) {
    int m = DM_NONE;
    if (strstr(p, "optimal")) m = DM_OPTIMAL;
    else if (strstr(p, "neutral")) m = DM_NEUTRAL;
    else if (strstr(p, "fatigued")) m = DM_FATIGUED;
    else if (strstr(p, "stressed")) m = DM_STRESSED;
    if (m != DM_NONE) { dMood = m; chirp(60); }
  }
}

static void pumpLine(Stream &port, char *buf, uint8_t &len) {
  while (port.available()) {
    char c = (char)port.read();
    if (c == '\n' || c == '\r') {
      if (len > 0) { buf[len] = 0; parseCommand(buf); len = 0; }
    } else if (len < 63) {
      buf[len++] = c;
    } else {
      len = 0;                                    // line too long: drop it
    }
  }
}

void rxTask() {
  pumpLine(Serial,  rxBuf, rxLen);                // from the PC dashboard (USB)
  pumpLine(Serial2, btBuf, btLen);                // from the phone (HC-05), same commands
}

// Send one line to the phone. Never blocks: if the HC-05 TX buffer is full the line is dropped,
// so a slow Bluetooth link can't stall fall detection.
void btSend(const char *line, int n) {
  if (Serial2.availableForWrite() >= n) Serial2.write((const uint8_t *)line, n);
}

const char *moodName() {
  switch (dMood) {
    case DM_OPTIMAL:  return "OPTIMAL";
    case DM_NEUTRAL:  return "NEUTRAL";
    case DM_FATIGUED: return "FATIGUED";
    case DM_STRESSED: return "STRESSED";
  }
  return "--";
}

// ================= SENSORS =================
void pollMax() {
  if (!maxOk) return;
  particleSensor.check();
  while (particleSensor.available()) {
    uint32_t red = particleSensor.getFIFORed();
    uint32_t ir = particleSensor.getFIFOIR();
    particleSensor.nextSample();
    V.finger = ir > 50000;
    if (!V.finger) { filled = 0; newCount = 0; V.hrValid = false; V.spoValid = false; continue; }
    if (checkForBeat((long)ir)) { beatT = millis(); lcdBeat = true; }
    bool compute = false;
    if (filled < 100) {
      redBuf[filled] = red; irBuf[filled] = ir; filled++;
      if (filled == 100) compute = true;
    } else {
      memmove(irBuf, irBuf + 1, 99 * sizeof(uint32_t));
      memmove(redBuf, redBuf + 1, 99 * sizeof(uint32_t));
      irBuf[99] = ir; redBuf[99] = red;
      if (++newCount >= 25) { newCount = 0; compute = true; }
    }
    if (compute) {
      int32_t spo2v, hrv; int8_t vS, vH;
      maxim_heart_rate_and_oxygen_saturation(irBuf, 100, redBuf, &spo2v, &vS, &hrv, &vH);
      uint32_t now = millis();
      if (vH && hrv > 40 && hrv < 200) {
        V.hr = V.hrValid ? (V.hr + hrv) / 2 : hrv; V.hrValid = true; lastHrOk = now;
      }
      if (vS && spo2v >= 80 && spo2v <= 100) {
        V.spo2 = V.spoValid ? (V.spo2 + spo2v) / 2 : spo2v; V.spoValid = true; lastSpoOk = now;
      }
    }
  }
  uint32_t now = millis();
  if (V.hrValid && now - lastHrOk > 8000) V.hrValid = false;
  if (V.spoValid && now - lastSpoOk > 8000) V.spoValid = false;
}

void gasTask() {
  static uint32_t t = 0;
  if (millis() - t < 100) return; t = millis();
  V.gasRaw = analogRead(PIN_MQ2);
  V.gasEma = V.gasEma * 0.9f + V.gasRaw * 0.1f;
  // MQ2 heater warm-up: during the first 5 minutes the baseline follows the reading
  // (no gas alarms); afterwards it only drifts slowly downward in clean air.
  if (millis() < 300000UL) gasBase = max(50.0f, V.gasEma);
  else if (V.gasEma < gasBase) gasBase = gasBase * 0.999f + V.gasEma * 0.001f;
  V.gasRatio = V.gasEma / gasBase;
}

void flameTask() {
  static uint32_t t = 0; static int cnt = 0;
  if (millis() - t < 50) return; t = millis();
  if (digitalRead(PIN_FLAME_D) == FLAME_ACTIVE) { if (cnt < 3) cnt++; } else cnt = 0;
  V.fire = (cnt >= 3);
  int raw = analogRead(PIN_FLAME_A);
  V.flameInt = constrain(100 - raw * 100 / 4095, 0, 100);
}

void tempTask() {
  static uint32_t tLast = 0; static bool req = false;
  uint32_t now = millis();
  if (!req && now - tLast >= 1000) { ds.requestTemperatures(); req = true; tLast = now; }
  else if (req && now - tLast >= 800) {
    float t = ds.getTempCByIndex(0); req = false;
    if (t > -100 && t < 80) { V.temp = t + TEMP_OFFSET; V.tempOk = true; V.onBody = V.temp >= 30.0; }
    else V.tempOk = false;
  }
}

// ---------- MPU-6050 (raw I2C, no library) ----------
bool mpuWrite(uint8_t reg, uint8_t val) {
  Wire.beginTransmission(mpuAddr); Wire.write(reg); Wire.write(val);
  return Wire.endTransmission() == 0;
}

bool mpuBegin() {
  const uint8_t addrs[2] = {0x68, 0x69};
  for (int i = 0; i < 2; i++) {
    Wire.beginTransmission(addrs[i]);
    if (Wire.endTransmission() == 0) {
      mpuAddr = addrs[i];
      mpuWrite(0x6B, 0x00);   // wake up (clear sleep bit)
      delay(50);
      mpuWrite(0x1A, 0x03);   // digital low-pass filter ~44 Hz
      mpuWrite(0x1B, 0x08);   // gyro  +-500 deg/s  (65.5 LSB per deg/s)
      mpuWrite(0x1C, 0x10);   // accel +-8 g        (4096 LSB per g)
      return true;
    }
  }
  return false;
}

void motionTask() {
  if (!mpuOk) return;
  static uint32_t t = 0; uint32_t now = millis();
  // sample fast while a fall may be happening so the impact peak is not missed
  uint32_t gap = (fState == FS_IDLE) ? 10 : 2;
  if (now - t < gap) return; t = now;

  Wire.beginTransmission(mpuAddr); Wire.write(0x3B);
  if (Wire.endTransmission(false) != 0) return;
  if (Wire.requestFrom((int)mpuAddr, 14) != 14) return;
  int16_t raw[7];
  for (int i = 0; i < 7; i++) { int hi = Wire.read(); int lo = Wire.read(); raw[i] = (int16_t)((hi << 8) | lo); }

  float ax = raw[0] / 4096.0f, ay = raw[1] / 4096.0f, az = raw[2] / 4096.0f;
  float gx = raw[4] / 65.5f,   gy = raw[5] / 65.5f,   gz = raw[6] / 65.5f;
  float g  = sqrtf(ax * ax + ay * ay + az * az);
  float gm = sqrtf(gx * gx + gy * gy + gz * gz);

  M.g = g; M.gmag = gm;
  if (g < mGMin) mGMin = g;
  if (g > mGMax) mGMax = g;
  M.gEma = M.gEma * 0.9f + gm * 0.1f;
  M.ax = M.ax * 0.8f + ax * 0.2f;
  M.ay = M.ay * 0.8f + ay * 0.2f;
  M.az = M.az * 0.8f + az * 0.2f;
  M.pitch = atan2f(-M.ax, sqrtf(M.ay * M.ay + M.az * M.az)) * 57.2958f;
  M.roll  = atan2f(M.ay, M.az) * 57.2958f;
  M.dyn = M.dyn * 0.95f + fabsf(g - 1.0f) * 0.05f;
  M.moving = (M.dyn > MOVE_DYN) || (M.gEma > 50.0f);

  // ---- fall detection: free-fall -> impact -> lying still ----
  switch (fState) {
    case FS_IDLE:
      if (g < FALL_FREEFALL_G) { if (++lowCnt >= 2) { fState = FS_FREEFALL; fStateT = now; } }
      else lowCnt = 0;
      if (g > FALL_HARD_G) { fState = FS_IMPACT; fStateT = now; stillOk = stillTot = 0; }
      break;
    case FS_FREEFALL:
      if (g > FALL_IMPACT_G) { fState = FS_IMPACT; fStateT = now; stillOk = stillTot = 0; }
      else if (now - fStateT > 700) { fState = FS_IDLE; lowCnt = 0; }
      break;
    case FS_IMPACT:
      if (now - fStateT >= 500) {                 // ignore the bounce, then watch 2 s
        stillTot++;
        if (fabsf(g - 1.0f) < 0.25f && gm < 45.0f) stillOk++;
        if (now - fStateT >= 2500) {
          if (stillTot > 0 && (stillOk * 100UL / stillTot) >= FALL_STILL_PCT) { V.fall = true; chirp(300); }
          fState = FS_IDLE; lowCnt = 0;
        }
      }
      break;
  }
}

void activityTask() {
  static uint32_t t = 0; uint32_t now = millis();
  if (now - t < 1000) return; t = now;
  if (mpuOk && M.dyn >= MOVE_DYN) {
    activeSecs++;
    if (activeSecs % 60 == 0) { xp += 1; prefs.putUInt("xp", xp); }   // +1 XP per active minute
  }
}

const char *actLabel() {
  if (M.dyn < 0.04f) return "STILL";
  if (M.dyn < MOVE_DYN) return "LIGHT";
  if (M.dyn < 0.5f) return "ACTIVE";
  return "INTENSE";
}

// ================= SHORT SUIT MESSAGE =================
// Very short text (max ~21 chars) for the OLED warning bar while the astronaut is moving.
// Highest priority first. Returns "" when everything is fine.
const char *shortMsg() {
  if (V.fire)  return "FIRE - EVACUATE";
  if (V.fall)  return "FALL - PRESS B3";
  if (lG == 2)  return "TOXIC AIR - MASK";
  if (lSp == 2) return "LOW O2 - OXYGEN ON";
  if (lHR == 2) return (V.hr > 100) ? "HR DANGER - SIT" : "HR LOW - CALL DOC";
  if (lT == 2)  return (V.temp > 37.5) ? "TEMP HIGH - COOL" : "TEMP LOW - WARM";
  if (lBG == 2) return (bgVal < BG_WARN_LOW) ? "SUGAR LOW - EAT NOW" : "SUGAR HIGH - CALL";
  if (stress)   return "STRESS - BREAK 10M";
  if (lBG == 1) return (bgVal < BG_WARN_LOW) ? "SUGAR LOW - EAT" : "SUGAR HIGH - WATER";
  if (lG == 1)  return "AIR POOR - VENT";
  if (lSp == 1) return "O2 LOW - BREATHE";
  if (lHR == 1) return (V.hr > 100) ? "HR HIGH - REST" : "HR LOW - CHECK DOC";
  if (lT == 1)  return (V.temp > 37.5) ? "TEMP HIGH - HYDRATE" : "TEMP LOW - WARM UP";
  return "";
}

// ================= STATUS / SCORE =================
void computeStatus() {
  uint32_t now = millis();
  lHR = V.hrValid ? lvlHR(V.hr) : 0;
  lSp = V.spoValid ? lvlSpo(V.spo2) : 0;
  lT  = (V.tempOk && V.onBody) ? lvlTemp(V.temp) : 0;
  lG  = lvlGas(V.gasRatio);
  lBG = lvlBG(bgVal);

  // sensor fusion: "Stressed" mood from the dashboard + pulse above limit right now
  stress = (dMood == DM_STRESSED) && V.hrValid && (V.hr > STRESS_HR);

  int worst = max(max(lHR, lSp), max(max(lT, lG), lBG));
  if (stress && worst < 1) worst = 1;
  if (V.fire || V.fall) worst = 2;
  static uint32_t critT = 0;
  if (!V.fire && !V.fall) {
    if (worst == 2) { if (critT == 0) critT = now; if (now - critT < 2500) worst = 1; }
    else critT = 0;
  }
  alertLevel = worst;
  if (V.fire)      { alertSrc = SRC_FIRE;   aMsg1 = "FIRE!";      aMsg2 = "EVACUATE"; }
  else if (V.fall) { alertSrc = SRC_FALL;   aMsg1 = "FALL!";      aMsg2 = "ARE YOU OK"; }
  else if (lG==2)  { alertSrc = SRC_GAS;    aMsg1 = "TOXIC AIR";  aMsg2 = "WEAR MASK"; }
  else if (lSp==2) { alertSrc = SRC_HEALTH; aMsg1 = "LOW OXYGEN"; aMsg2 = "CHECK O2"; }
  else if (lHR==2) { alertSrc = SRC_HEALTH; aMsg1 = "HEART RATE"; aMsg2 = "ABNORMAL"; }
  else if (lT==2)  { alertSrc = SRC_HEALTH; aMsg1 = "BODY TEMP";  aMsg2 = "ABNORMAL"; }
  else if (lBG==2) {
    alertSrc = SRC_SUGAR;
    if (bgVal < BG_WARN_LOW) { aMsg1 = "LOW SUGAR";  aMsg2 = "EAT CARBS"; }
    else                     { aMsg1 = "HIGH SUGAR"; aMsg2 = "CALL CTRL"; }
  }
  else             { alertSrc = SRC_NONE;   aMsg1 = "";           aMsg2 = ""; }

  int s = 100;
  s -= (lHR == 2 ? 25 : lHR == 1 ? 12 : 0);
  s -= (lSp == 2 ? 25 : lSp == 1 ? 12 : 0);
  s -= (lT  == 2 ? 25 : lT  == 1 ? 12 : 0);
  s -= (lG  == 2 ? 25 : lG  == 1 ? 12 : 0);
  s -= (lBG == 2 ? 25 : lBG == 1 ? 12 : 0);
  if (stress) s -= 10;
  if (V.fire) s -= 50;
  if (V.fall) s -= 30;
  if (logged) { if (ciMood == 0) s -= 10; if (ciSleep < 5) s -= 15; else if (ciSleep < 6) s -= 8; }
  score = constrain(s, 0, 100);

  // short beep whenever a new warning message appears
  static const char *lastMsg = "";
  const char *m = shortMsg();
  if (m[0] && m != lastMsg) chirp(120);
  lastMsg = m;
}

const char *adviceFor() {
  static const char *tips[] = {"ALL NOMINAL", "DRINK WATER", "EXERCISE 2H/DAY", "SLEEP 8H/NIGHT"};
  uint32_t now = millis();
  if (V.fire) return "FIRE! EVACUATE";
  if (V.fall) return "FALL! PRESS B3";
  if (lG == 2) return "TOXIC AIR! MASK";
  if (lSp == 2) return "LOW SpO2! O2 ON";
  if (lHR == 2) return "HR ABNORMAL!";
  if (lT == 2) return "TEMP ABNORMAL!";
  if (lBG == 2) return (bgVal < BG_WARN_LOW) ? "LOW SUGAR! EAT" : "HI SUGAR! CALL";
  if (stress) return "STRESS: BREAK";
  if (lBG == 1) return (bgVal < BG_WARN_LOW) ? "SUGAR LOW: EAT" : "HI SUGAR: WATER";
  if (lG == 1) return "VENTILATE CABIN";
  if (lSp == 1) return "DEEP BREATHS";
  if (lHR == 1) return (V.hr > 100) ? "REST & BREATHE" : "CHECK W/ DOC";
  if (lT == 1) return (V.temp > 37.5) ? "HYDRATE & COOL" : "WARM UP";
  if (logged && ciMood == 0) return "CONTACT CREW";
  if (logged && ciSleep < 6) return "SLEEP MORE";
  if (reminderActive) return "TIME TO MEASURE";
  if (V.finger && M.moving) return "HOLD STILL";
  if (!V.finger && (now / 3000) % 2) return "PLACE FINGER";
  return tips[(now / 3500) % 4];
}

// ================= GAMIFICATION =================
void saveProgress() { prefs.putUInt("xp", xp); prefs.putUInt("scans", scans); prefs.putUInt("logs", logs); }

void gamification() {
  uint32_t now = millis();
  if (V.finger && V.hrValid && V.spoValid) {
    if (!measuring) { measuring = true; measureStart = now; measured = false; }
    else if (!measured && now - measureStart >= 15000) {
      measured = true; xp += 5; scans++; saveProgress(); chirp(250);
      reminderActive = false; lastMeasure = now;
    }
  } else measuring = false;
  if (!reminderActive && now - lastMeasure > REMIND_MS) { reminderActive = true; chirp(150); }
}

void histTask() {
  uint32_t now = millis();
  if (now - lastHist < HIST_PERIOD_MS) return; lastHist = now;
  if (!V.hrValid) return;
  if (histCount < HIST) hist[histCount++] = V.hr;
  else { memmove(hist, hist + 1, (HIST - 1) * sizeof(int)); hist[HIST - 1] = V.hr; }
}

// ================= BUZZER =================
void buzzerTask() {
  uint32_t now = millis(); bool on = false;
  if (chirpUntil > now) on = true;
  else if (alertLevel == 2 && now > silenceUntil) {
    uint32_t p;
    switch (alertSrc) {
      case SRC_FIRE: on = (now % 200) < 100; break;
      case SRC_GAS:  on = (now % 800) < 400; break;
      case SRC_FALL: p = now % 1200; on = (p < 150) || (p >= 300 && p < 450) || (p >= 600 && p < 750); break;
      case SRC_SUGAR: p = now % 2000; on = (p < 250) || (p >= 500 && p < 750); break;
      default: p = now % 1600; on = (p < 200) || (p >= 400 && p < 600); break;
    }
  }
  digitalWrite(PIN_BUZZER, on);
}

// ================= INPUT =================
void handleInput() {
  readButtons();
  bool b1 = pressed(0), b2 = pressed(1), b3 = pressed(2);
  uint32_t now = millis();
  if (b1 || b2 || b3) chirp(15);
  if (alertLevel == 2 && now > silenceUntil) {
    if (b3) {
      if (alertSrc == SRC_FALL) V.fall = false;          // "I'M OK"
      else silenceUntil = now + (alertSrc == SRC_FIRE ? 15000 : 60000);
    }
    return;
  }
  if (b1) { page = (page + 1) % PAGES; pageT = now; return; }
  if (page == PG_CHECKIN) {
    if (b2) {
      if (ciStage == 0) ciMood = (ciMood + 1) % 3;
      else if (ciStage == 1) { ciSleep += 0.5; if (ciSleep > 10) ciSleep = 3; }
      else if (ciStage == 2) ciStage = 0;
    }
    if (b3) {
      if (ciStage == 0) ciStage = 1;
      else if (ciStage == 1) { ciStage = 2; logged = true; xp += 20; logs++; saveProgress(); chirp(200); }
    }
  } else {
    if (b2) { page = (page + PAGES - 1) % PAGES; pageT = now; }
    if (b3 && reminderActive) { reminderActive = false; lastMeasure = now - REMIND_MS + 120000UL; }
  }
}

// ================= DRAW HELPERS =================
#define WH SSD1306_WHITE
#define BK SSD1306_BLACK

void drawHeart(int cx, int cy, int r, bool fill) {
  int rr = r / 2 + 1;
  if (fill) {
    oled.fillCircle(cx - r / 2, cy - r / 3, rr, WH);
    oled.fillCircle(cx + r / 2, cy - r / 3, rr, WH);
    oled.fillTriangle(cx - r - 1, cy - r / 6, cx + r + 1, cy - r / 6, cx, cy + r + 2, WH);
  } else {
    oled.drawCircle(cx - r / 2, cy - r / 3, rr, WH);
    oled.drawCircle(cx + r / 2, cy - r / 3, rr, WH);
    oled.drawTriangle(cx - r - 1, cy - r / 6, cx + r + 1, cy - r / 6, cx, cy + r + 2, WH);
  }
}

void drawFlame(int cx, int by, int h, bool fill) {
  int w = h / 2;
  int fl = fill ? (int)(sinf(millis() / 60.0f) * 2) : 0;
  if (fill) {
    oled.fillCircle(cx, by - w, w, WH);
    oled.fillTriangle(cx - w, by - w, cx + w, by - w, cx + fl, by - h - 2, WH);
    oled.fillCircle(cx, by - w + 1, w / 2, BK);
  } else {
    oled.drawCircle(cx, by - w, w, WH);
    oled.drawTriangle(cx - w, by - w, cx + w, by - w, cx, by - h, WH);
  }
}

void drawFace(int cx, int cy, int mood) {
  oled.drawCircle(cx, cy, 10, WH);
  oled.drawPixel(cx - 3, cy - 3, WH); oled.drawPixel(cx + 3, cy - 3, WH);
  oled.drawPixel(cx - 3, cy - 2, WH); oled.drawPixel(cx + 3, cy - 2, WH);
  if (mood == 2) {
    oled.drawLine(cx - 4, cy + 3, cx - 2, cy + 5, WH); oled.drawLine(cx - 2, cy + 5, cx + 2, cy + 5, WH);
    oled.drawLine(cx + 2, cy + 5, cx + 4, cy + 3, WH);
  } else if (mood == 1) {
    oled.drawLine(cx - 4, cy + 4, cx + 4, cy + 4, WH);
  } else {
    oled.drawLine(cx - 4, cy + 5, cx - 2, cy + 3, WH); oled.drawLine(cx - 2, cy + 3, cx + 2, cy + 3, WH);
    oled.drawLine(cx + 2, cy + 3, cx + 4, cy + 5, WH);
  }
}

void header(const char *title) {
  oled.setTextSize(1); oled.setTextColor(WH);
  oled.setCursor(0, 1); oled.print(title);
  if (alertLevel >= 1 && (millis() / 400) % 2) { oled.setCursor(68, 1); oled.print("!"); }
  for (int i = 0; i < PAGES; i++) {
    int x = 127 - (PAGES - i) * 7 + 2;
    if (i == page) oled.fillCircle(x, 4, 2, WH); else oled.drawCircle(x, 4, 2, WH);
  }
  oled.drawFastHLine(0, 10, 128, WH);
}

// Bottom warning bar: short message such as "HR HIGH - REST", drawn on every page
// so the astronaut sees it at a glance while moving. Blinks between solid and outline.
void drawWarnBar() {
  const char *m = shortMsg();
  if (!m[0]) return;
  int len = strlen(m);
  int x = (128 - len * 6) / 2; if (x < 1) x = 1;
  bool solid = ((millis() / 500) % 2) == 0;
  if (solid) {
    oled.fillRect(0, 54, 128, 10, WH);
    oled.setTextColor(BK);
  } else {
    oled.fillRect(0, 54, 128, 10, BK);
    oled.drawRect(0, 54, 128, 10, WH);
    oled.setTextColor(WH);
  }
  oled.setTextSize(1);
  oled.setCursor(x, 55); oled.print(m);
  oled.setTextColor(WH);
}

void rowGauge(int y, const char *lab, const char *val, float v, float lo, float hi,
              float nLo, float nHi, int lvl, bool valid) {
  oled.setTextSize(1);
  oled.setCursor(0, y); oled.print(lab);
  oled.setCursor(30, y); oled.print(val);
  const char *st = !valid ? "--" : (lvl == 0 ? "OK" : (lvl == 1 ? "WARN" : "CRIT"));
  bool show = !(valid && lvl == 2 && ((millis() / 300) % 2));
  if (show) { oled.setCursor(128 - strlen(st) * 6, y); oled.print(st); }
  oled.drawRect(0, y + 9, 128, 5, WH);
  int xl = (int)((nLo - lo) / (hi - lo) * 126) + 1;
  int xh = (int)((nHi - lo) / (hi - lo) * 126) + 1;
  oled.drawFastVLine(xl, y + 8, 7, WH); oled.drawFastVLine(xh, y + 8, 7, WH);
  if (valid) {
    int xm = constrain((int)((v - lo) / (hi - lo) * 126) + 1, 2, 125);
    oled.fillRect(xm - 1, y + 7, 3, 9, WH);
  }
}

// ================= PAGES =================
void drawHome() {
  header("ASTRO-GUARD");
  uint32_t now = millis();
  float pulse = V.finger ? max(0.0f, 1.0f - (now - beatT) / 280.0f) : 0;
  drawHeart(18, 28, 7 + (int)(3 * pulse), V.finger);
  oled.setTextSize(2); oled.setCursor(38, 14);
  if (V.hrValid) oled.print(V.hr); else oled.print("--");
  oled.setTextSize(1); oled.setCursor(38, 32); oled.print("BPM");
  oled.setCursor(80, 14);
  oled.print("O2 "); if (V.spoValid) { oled.print(V.spo2); oled.print("%"); } else oled.print("--");
  oled.setCursor(80, 25);
  if (V.tempOk && V.onBody) { oled.print("T "); oled.print(V.temp, 1); oled.print("C"); }
  else oled.print("T --");
  oled.setCursor(80, 36);
  oled.print("AIR "); oled.print(lG == 0 ? "OK" : lG == 1 ? "LOW" : "BAD");

  // message row
  oled.setCursor(0, 46);
  if (reminderActive && (now / 500) % 2) {
    oled.fillRect(0, 45, 128, 9, WH); oled.setTextColor(BK);
    oled.setCursor(10, 46); oled.print("* TIME TO MEASURE *"); oled.setTextColor(WH);
  } else if (!V.finger) {
    if ((now / 500) % 2) { oled.setCursor(22, 46); oled.print("> PLACE FINGER <"); }
  } else if (M.moving) {
    if ((now / 500) % 2) { oled.setCursor(28, 46); oled.print("> HOLD STILL <"); }
  } else if (measuring && !measured) {
    int sec = (now - measureStart) / 1000;
    oled.setCursor(0, 46); oled.print("MEASURING "); oled.print(sec); oled.print("/15s");
  } else if (measured) {
    oled.setCursor(0, 46); oled.print("SCAN COMPLETE +5XP");
  } else {
    oled.setCursor(0, 46); oled.print("READING...");
  }
  // score bar
  oled.setCursor(0, 55); oled.print("HP");
  oled.drawRect(14, 54, 88, 9, WH);
  int w = score * 84 / 100;
  oled.fillRect(16, 56, w, 5, WH);
  for (int x = 16; x < 16 + w; x++) if (((x + now / 60) % 8) == 0) oled.drawFastVLine(x, 56, 5, BK);
  oled.setCursor(106, 55); oled.print(score);
}

void drawVitals() {
  header("VITALS");
  char b[16];
  snprintf(b, sizeof(b), V.hrValid ? "%d bpm" : "-- bpm", V.hr);
  rowGauge(13, "HR", b, V.hr, 40, 160, 60, 100, lHR, V.hrValid);
  snprintf(b, sizeof(b), V.spoValid ? "%d %%" : "-- %%", V.spo2);
  rowGauge(30, "SpO2", b, V.spo2, 85, 100, 95, 100, lSp, V.spoValid);
  bool tv = V.tempOk && V.onBody;
  if (tv) snprintf(b, sizeof(b), "%.1f C", V.temp); else snprintf(b, sizeof(b), V.tempOk ? "amb %.0f" : "-- C", V.temp);
  rowGauge(47, "TEMP", b, V.temp, 34, 40, 36.0, 37.5, lT, tv);
}

void drawEnv() {
  header("ENVIRONMENT");
  oled.setTextSize(1); oled.setCursor(0, 14); oled.print("AIR QUALITY");
  oled.setTextSize(2); oled.setCursor(0, 26);
  oled.print(lG == 0 ? "SAFE" : lG == 1 ? "POOR" : "DANGER");
  oled.setTextSize(1);
  oled.drawRect(0, 44, 70, 6, WH);
  int fw = constrain((int)((V.gasRatio - 1.0f) / 3.0f * 68), 0, 68);
  oled.fillRect(1, 45, fw, 4, WH);
  oled.drawFastVLine(1 + (int)(0.6f / 3.0f * 68), 42, 10, WH);
  oled.drawFastVLine(1 + (int)(1.6f / 3.0f * 68), 42, 10, WH);
  // flame
  if (V.fire) {
    drawFlame(104, 38, 22, true);
    if ((millis() / 250) % 2) { oled.setCursor(86, 41); oled.print("FIRE!"); }
  } else {
    drawFlame(104, 36, 20, false);
    oled.setCursor(86, 41); oled.print("CLEAR");
  }
  oled.setCursor(0, 55);
  oled.print("MQ2 "); oled.print(V.gasRaw); oled.print(" x"); oled.print(V.gasRatio, 1);
  oled.print(" IR"); oled.print(V.flameInt);
}

void drawMotion() {
  header("MOTION");
  if (!mpuOk) {
    oled.setCursor(4, 28); oled.print("MPU6050 NOT FOUND");
    oled.setCursor(16, 42); oled.print("CHECK WIRING");
    return;
  }
  // bubble level
  const int cx = 26, cy = 37, rr = 22;
  oled.drawCircle(cx, cy, rr, WH); oled.drawCircle(cx, cy, 8, WH);
  oled.drawFastHLine(cx - rr, cy, 6, WH); oled.drawFastHLine(cx + rr - 5, cy, 6, WH);
  oled.drawFastVLine(cx, cy - rr, 6, WH); oled.drawFastVLine(cx, cy + rr - 5, 6, WH);
  int bx = cx + constrain((int)(M.ay * 40), -14, 14);
  int by = cy + constrain((int)(M.ax * 40), -14, 14);
  oled.fillCircle(bx, by, 4, WH);
  // numbers
  oled.setCursor(58, 14); oled.print("G "); oled.print(M.g, 2);
  oled.setCursor(58, 24); oled.print("PITCH "); oled.print((int)M.pitch);
  oled.setCursor(58, 34); oled.print("ROLL "); oled.print((int)M.roll);
  oled.setCursor(58, 44); oled.print(actLabel());
  oled.setCursor(58, 54); oled.print("ACT "); oled.print(activeSecs); oled.print("s");
}

void drawTrend() {
  header("HR TREND");
  oled.setCursor(0, 13); oled.print("140"); oled.setCursor(6, 46); oled.print("40");
  // normal band
  for (int x = 20; x < 128; x += 4) {
    oled.drawPixel(x, 53 - (100 - 40) * 38 / 100, WH);
    oled.drawPixel(x, 53 - (60 - 40) * 38 / 100, WH);
  }
  oled.drawFastVLine(19, 14, 40, WH);
  int mn = 999, mx = 0, sum = 0, px = -1, py = -1;
  for (int i = 0; i < histCount; i++) {
    int v = hist[i]; mn = min(mn, v); mx = max(mx, v); sum += v;
    int x = 21 + i * 2;
    int y = 53 - constrain((v - 40) * 38 / 100, 0, 38);
    if (px >= 0) oled.drawLine(px, py, x, y, WH);
    px = x; py = y;
  }
  if (px >= 0 && (millis() / 300) % 2) oled.fillCircle(px, py, 2, WH);
  oled.setCursor(0, 56);
  if (histCount) { oled.print("AVG "); oled.print(sum / histCount); oled.print(" MIN "); oled.print(mn); oled.print(" MAX "); oled.print(mx); }
  else oled.print("COLLECTING DATA...");
}

void drawCheckin() {
  header("CHECK-IN");
  uint32_t now = millis();
  if (ciStage == 0) {
    oled.setCursor(10, 14); oled.print("HOW DO YOU FEEL?");
    for (int i = 0; i < 3; i++) {
      int cx = 22 + i * 42, cy = 34;
      if (i == ciMood) { cy += (int)(sinf(now / 150.0f) * 2); oled.drawRoundRect(cx - 14, cy - 14, 28, 28, 4, WH); }
      drawFace(cx, cy, i);
    }
    oled.setCursor(0, 56); oled.print("B2:CHOOSE  B3:OK");
  } else if (ciStage == 1) {
    oled.setCursor(10, 14); oled.print("SLEEP LAST NIGHT");
    oled.setTextSize(3); oled.setCursor(18, 26);
    oled.print(ciSleep, 1); oled.setTextSize(1); oled.print(" h");
    oled.setCursor(0, 56); oled.print("B2:+0.5h   B3:OK");
  } else {
    oled.setCursor(30, 14); oled.print("LOGGED! +20XP");
    drawFace(22, 36, ciMood);
    oled.setCursor(42, 26); oled.print("MOOD: "); oled.print(ciMood == 0 ? "LOW" : ciMood == 1 ? "OK" : "GOOD");
    oled.setCursor(42, 38); oled.print("SLEEP: "); oled.print(ciSleep, 1); oled.print("h");
    // data logged from the dashboard (blood sugar + mood)
    oled.setCursor(0, 47);
    oled.print("BG "); if (bgVal > 0) oled.print(bgVal); else oled.print("--");
    oled.print(" "); oled.print(moodName());
    oled.setCursor(0, 56); oled.print("B2: NEW CHECK-IN");
  }
}

void drawCrew() {
  header("CREW STATUS");
  uint32_t now = millis();
  int level = xp / 50 + 1, xpIn = xp % 50;
  oled.setTextSize(2); oled.setCursor(0, 14); oled.print("LVL "); oled.print(level);
  oled.setTextSize(1); oled.setCursor(0, 32); oled.print("XP "); oled.print(xp); oled.print("/"); oled.print(level * 50);
  oled.drawRect(0, 42, 80, 7, WH); oled.fillRect(2, 44, xpIn * 76 / 50, 3, WH);
  oled.setCursor(0, 55); oled.print("SCANS "); oled.print(scans); oled.print("  LOGS "); oled.print(logs);
  // orbit animation
  oled.drawCircle(104, 28, 16, WH); oled.fillCircle(104, 28, 5, WH);
  float a = now / 700.0f;
  oled.fillCircle(104 + (int)(16 * cosf(a)), 28 + (int)(16 * sinf(a)), 2, WH);
  float b = now / 1300.0f + 2.0f;
  oled.drawPixel(104 + (int)(16 * cosf(b)), 28 + (int)(16 * sinf(b)), WH);
}

void drawAlert() {
  bool blink = (millis() / 250) % 2;
  uint16_t bg = blink ? WH : BK, fg = blink ? BK : WH;
  oled.fillRect(0, 0, 128, 64, bg);
  oled.drawRect(0, 0, 128, 64, fg);
  oled.drawTriangle(4, 14, 18, 14, 11, 2, fg);
  oled.setTextColor(fg); oled.setTextSize(1);
  oled.setCursor(9, 6); oled.print("!");
  oled.setCursor(26, 5); oled.print("ALERT");
  oled.setTextSize(2);
  oled.setCursor((128 - strlen(aMsg1) * 12) / 2, 20); oled.print(aMsg1);
  oled.setCursor((128 - strlen(aMsg2) * 12) / 2, 38); oled.print(aMsg2);
  oled.setTextSize(1);
  if (alertSrc == SRC_FALL) { oled.setCursor(22, 55); oled.print("B3: I'M OK"); }
  else { oled.setCursor(30, 55); oled.print("B3: SILENCE"); }
  oled.setTextColor(WH);
}

void render() {
  if (!oledOk) return;
  uint32_t now = millis();
  oled.clearDisplay(); oled.setTextColor(WH); oled.setTextSize(1);
  if (alertLevel == 2 && now > silenceUntil) drawAlert();
  else {
    switch (page) {
      case PG_HOME: drawHome(); break;
      case PG_VITALS: drawVitals(); break;
      case PG_ENV: drawEnv(); break;
      case PG_MOTION: drawMotion(); break;
      case PG_TREND: drawTrend(); break;
      case PG_CHECKIN: drawCheckin(); break;
      case PG_CREW: drawCrew(); break;
    }
    drawWarnBar();                       // short suit warning on top of any page
    uint32_t dt = now - pageT;
    if (dt < 300) { int x = dt * 128 / 300; oled.fillRect(x, 0, 128 - x, 64, BK); oled.drawFastVLine(x, 0, 64, WH); }
  }
  oled.display();
}

// ================= LCD =================
byte heartChar[8] = {0b00000, 0b01010, 0b11111, 0b11111, 0b01110, 0b00100, 0b00000, 0b00000};
uint32_t lcdHeartOff = 0; bool lcdHeartShown = false;

void lcdTask() {
  if (!lcdOk) return;
  static uint32_t t = 0; uint32_t now = millis();
  if (now - t >= 500) {
    t = now;
    char hs[6], ss[6], ts[8], l0[20], l1[20];
    if (V.hrValid) snprintf(hs, sizeof(hs), "%3d", V.hr); else strcpy(hs, "---");
    if (V.spoValid) snprintf(ss, sizeof(ss), "%3d", V.spo2); else strcpy(ss, "---");
    if (V.tempOk && V.onBody) snprintf(ts, sizeof(ts), "%4.1f", V.temp); else strcpy(ts, "----");
    snprintf(l0, sizeof(l0), "HR%s SP%s %s", hs, ss, ts);
    snprintf(l1, sizeof(l1), "%-15s", adviceFor());
    lcd->setCursor(0, 0); lcd->print(l0);
    lcd->setCursor(0, 1); lcd->print(l1);
  }
  if (lcdBeat) {
    lcdBeat = false; lcd->setCursor(15, 1); lcd->write(byte(0));
    lcdHeartShown = true; lcdHeartOff = now + 150;
  } else if (lcdHeartShown && now >= lcdHeartOff) {
    lcd->setCursor(15, 1); lcd->print(' '); lcdHeartShown = false;
  }
}

// ================= BOOT =================
void bootSequence() {
  struct Star { float x, y, s; } st[28];
  for (auto &s : st) { s.x = random(0, 128); s.y = random(0, 64); s.s = 0.5f + random(0, 20) / 10.0f; }
  uint32_t t0 = millis(), dur = BASELINE_SECS * 1000UL;
  long sum = 0; int n = 0;
  const char *title = "ASTRO-GUARD";
  if (lcdOk) { lcd->setCursor(0, 0); lcd->print("ASTRO-GUARD v1.1"); lcd->setCursor(0, 1); lcd->print("CALIBRATING AIR "); }
  while (millis() - t0 < dur) {
    uint32_t el = millis() - t0;
    sum += analogRead(PIN_MQ2); n++;
    if (oledOk) {
      oled.clearDisplay();
      for (auto &s : st) { s.x -= s.s; if (s.x < 0) { s.x = 127; s.y = random(0, 64); } oled.drawPixel((int)s.x, (int)s.y, WH); }
      int r = (el / 20) % 40;
      oled.drawCircle(64, 24, r, WH);
      oled.fillCircle(64, 24, 6, WH);
      int ang = el / 8;
      oled.fillCircle(64 + (int)(22 * cosf(ang * 0.05f)), 24 + (int)(10 * sinf(ang * 0.05f)), 2, WH);
      int chars = min((int)(el / 150), 11);
      char buf[14]; strncpy(buf, title, chars); buf[chars] = 0;
      oled.setTextColor(WH); oled.setTextSize(2); oled.setCursor(10, 40); oled.print(buf);
      oled.setTextSize(1); oled.setCursor(0, 56);
      oled.print("WARMING SENSORS");
      oled.drawRect(94, 56, 34, 7, WH); oled.fillRect(96, 58, el * 30 / dur, 3, WH);
      oled.display();
    }
  }
  gasBase = max(50.0f, (float)sum / n);
  V.gasEma = gasBase;
  if (lcdOk) lcd->clear();
}

// ================= DASHBOARD LINK =================
// One JSON line per second for the VEGA web dashboard (Web Serial, 115200 baud).
// 0 = "no valid reading" (no finger / not on body), which the dashboard shows as "No signal".
// (The dashboard sends {"bg":..} / {"mood":".."} back; those are handled in rxTask().)
void sendToDashboard() {
  int gasPct = constrain((int)((V.gasRatio - 1.0f) * 38.0f), 0, 100);  // 1.6x -> ~22 (warn), 2.6x -> ~61 (danger)
  float t = (V.tempOk && V.onBody) ? V.temp : 0.0f;
  char line[96];
  int n = snprintf(line, sizeof(line), "{\"hr\":%d,\"spo2\":%d,\"temp\":%.1f,\"gas\":%d,\"flame\":%d,\"flame_ao\":%d}\n",
                   V.hrValid ? V.hr : 0,
                   V.spoValid ? V.spo2 : 0,
                   t, gasPct, V.fire ? 1 : 0, V.flameInt);
  Serial.write((const uint8_t *)line, n);        // USB -> PC dashboard
  btSend(line, n);                                // Bluetooth -> phone
}

// Motion line for the dashboard's MPU card, 5x per second. gmin/gmax keep brief free-fall / impact peaks visible.
void sendMotion() {
  static uint32_t t = 0; uint32_t now = millis();
  if (now - t < 200) return; t = now;
  static uint32_t btT = 0; bool toBt = (now - btT >= BT_MOTION_MS);
  if (toBt) btT = now;
  char line[130]; int n;
  if (!mpuOk) {
    n = snprintf(line, sizeof(line), "{\"mpu\":0}\n");
  } else {
    float lo = (mGMin > 8) ? M.g : mGMin, hi = (mGMax <= 0) ? M.g : mGMax;
    n = snprintf(line, sizeof(line), "{\"mpu\":1,\"pitch\":%.1f,\"roll\":%.1f,\"g\":%.2f,\"gmin\":%.2f,\"gmax\":%.2f,\"dyn\":%.3f,\"gyro\":%d,\"fall\":%d}\n",
                 M.pitch, M.roll, M.g, lo, hi, M.dyn, (int)M.gEma, V.fall ? 1 : 0);
    mGMin = 9; mGMax = 0;
  }
  Serial.write((const uint8_t *)line, n);
  if (toBt) btSend(line, n);
}

// ================= SETUP / LOOP =================
void setup() {
  Serial.begin(115200);
  Serial2.begin(BT_BAUD, SERIAL_8N1, PIN_BT_RX, PIN_BT_TX);   // HC-05
  pinMode(PIN_B1, INPUT_PULLUP); pinMode(PIN_B2, INPUT_PULLUP); pinMode(PIN_B3, INPUT_PULLUP);
  pinMode(PIN_FLAME_D, INPUT); pinMode(PIN_BUZZER, OUTPUT); digitalWrite(PIN_BUZZER, LOW);
  analogReadResolution(12);
  Wire.begin(PIN_SDA, PIN_SCL); Wire.setClock(400000);

  oledOk = oled.begin(SSD1306_SWITCHCAPVCC, 0x3C);
  uint8_t addr = 0;
  Wire.beginTransmission(0x27); if (Wire.endTransmission() == 0) addr = 0x27;
  else { Wire.beginTransmission(0x3F); if (Wire.endTransmission() == 0) addr = 0x3F; }
  if (addr) { lcd = new LiquidCrystal_I2C(addr, 16, 2); lcd->init(); lcd->backlight(); lcd->createChar(0, heartChar); lcdOk = true; }
  maxOk = particleSensor.begin(Wire, I2C_SPEED_FAST);
  if (maxOk) particleSensor.setup(60, 4, 2, 100, 411, 4096);
  mpuOk = mpuBegin();
  ds.begin();
  ds.setWaitForConversion(false);

  prefs.begin("astro", false);
  xp = prefs.getUInt("xp", 0); scans = prefs.getUInt("scans", 0); logs = prefs.getUInt("logs", 0);

  Serial.printf("OLED:%d LCD:%d(0x%02X) MAX30102:%d MPU6050:%d(0x%02X)\n", oledOk, lcdOk, addr, maxOk, mpuOk, mpuAddr);
  bootSequence();
  lastMeasure = millis(); pageT = millis();
}

void loop() {
  uint32_t now = millis();
  rxTask();                              // blood sugar / mood from the dashboard
  pollMax();
  motionTask();
  flameTask(); gasTask(); tempTask();
  handleInput();
  computeStatus();
  gamification();
  activityTask();
  histTask();
  buzzerTask();
  // while a fall may be unfolding, skip the (slow) OLED refresh so the impact peak is sampled
  bool fastWindow = (fState == FS_FREEFALL) || (fState == FS_IMPACT && now - fStateT < 500);
  static uint32_t lastO = 0;
  if (!fastWindow && now - lastO >= 40) { lastO = now; render(); }
  lcdTask();
  static uint32_t lastS = 0;
  if (now - lastS >= 1000) {
    lastS = now;
    sendToDashboard();
  }
  sendMotion();
}
