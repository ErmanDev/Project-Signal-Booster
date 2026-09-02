/*
 * Lantapan Hub — ESP32 + SIMCom A7670C signal-seeking antenna
 *
 * MQTT rides the SIM (A7670C CMQTT AT over LTE). WiFi is not required.
 * Publishes real AT+CSQ telemetry to index.html. No mock / random RSSI.
 *
 * Libraries: ESP32Servo (plus ESP32 Arduino core Preferences).
 * PubSubClient / WiFi are only compiled when WIFI_FALLBACK is 1.
 *
 * ========== PIN MAP (every wire) ==========
 * ESP32 DEVKIT V1 silkscreen — GPIO numbers stay 16 / 17 / 27 / 13.
 * There is no D16 printed. UART2 is labeled RX2 / TX2, not D16 / D17.
 * Do not use RX0 / TX0 (USB serial).
 *
 * A7670C TX          → ESP32 RX2 (GPIO 16; next to D4 — no D16 printed)
 * A7670C RX          → ESP32 TX2 (GPIO 17; next to D5)
 * A7670C PWRKEY      → ESP32 D27 (GPIO 27)  (pulse LOW ~1.2s if AT is silent)
 * Servo signal       → ESP32 D13 (GPIO 13)  (yellow/orange)
 * A7670C VIN         → boost 5.0 V   (board has VIN only — no VBAT pad)
 * Servo VCC (red)    → boost 5.0 V
 * Servo GND (brown)  → common GND
 * Power switch       → battery +  (NOT a GPIO)
 * Boost EN (if any)  → switched battery + so the boost dies with the switch
 * Common GND         → Li-ion−, boost GND, A7670C GND, ESP32 GND, servo GND
 *
 * UART: 115200 8N1 on UART2 (RX2/TX2). Common GND is required or AT will never answer.
 *
 * ========== POWER PATH (this board: VIN only) ==========
 * Li-ion+ → POWER SWITCH → boost IN+
 * Boost OUT 5.0 V → A7670C VIN + ESP32 VIN + servo VCC
 * Li-ion− → common GND
 *
 * Do not wire the cell to the modem. VIN is the 5 V input; the breakout
 * already regulates to ~3.8 V internally. 470 µF+ on A7670C VIN/GND if room.
 *
 * TM / Globe SIMs need a data-enabled load (mobile data), not voice-only.
 */

#include <ESP32Servo.h>
#include <Preferences.h>

// Optional WiFi backhaul. Leave 0 — MQTT is meant to go out the SIM.
#ifndef WIFI_FALLBACK
#define WIFI_FALLBACK 0
#endif

#if WIFI_FALLBACK
#include <WiFi.h>
#include <PubSubClient.h>
const char* WIFI_SSID = "YOUR_WIFI_SSID";
const char* WIFI_PASS = "YOUR_WIFI_PASS";
#endif

// ---------- MQTT (must match index.html) ----------
const char* MQTT_BROKER_HOST = "broker.emqx.io";
const uint16_t MQTT_PORT = 1883;  // plain TCP — do not use 8883
const char* MQTT_STATUS_TOPIC = "signalbooster/hub1/status";
const char* MQTT_COMMAND_TOPIC = "signalbooster/hub1/command";
const char* MQTT_CLIENT_ID = "signalbooster-hub1";
const char* MQTT_URL = "tcp://broker.emqx.io:1883";

// ---------- Pins ----------
static const int MODEM_RX_PIN = 16;   // silkscreen RX2 (next to D4; no D16) <- A7670C TX
static const int MODEM_TX_PIN = 17;   // silkscreen TX2 (next to D5) -> A7670C RX
static const int PWRKEY_PIN = 27;     // silkscreen D27 — pulse LOW to boot modem on battery
static const int SERVO_PIN = 13;      // silkscreen D13 — PWM to hobby servo

static const long MODEM_BAUD = 115200;
static const uint16_t SERVO_MIN_US = 500;
static const uint16_t SERVO_MAX_US = 2400;

// Same quality map as index.html. 3 UI bars = Fair = rssi >= 12.
static const int RSSI_STRONG = 25;
static const int RSSI_GOOD = 18;
static const int RSSI_FAIR = 12;      // hunt below this; hold at this or better
static const int RSSI_UNKNOWN = 99;   // AT+CSQ "not known or not detectable"

static const int SERVO_MIN = 0;
static const int SERVO_MAX = 180;
static const int HUNT_STEP_DEG = 2;
static const unsigned long HUNT_STEP_MS = 400;   // slow sweep
static const unsigned long STATUS_HOLD_MS = 2000;
static const unsigned long STATUS_HUNT_MS = 5000;
static const unsigned long MODEM_POLL_HOLD_MS = 2000;
static const unsigned long MODEM_POLL_HUNT_MS = 700;
static const unsigned long MODEM_SLOW_POLL_MS = 10000;
static const unsigned long LTE_RETRY_MS = 8000;
static const unsigned long MQTT_RETRY_MS = 5000;
static const unsigned long AT_TIMEOUT_MS = 1500;
static const unsigned long CGATT_TIMEOUT_MS = 30000;
static const unsigned long NETOPEN_TIMEOUT_MS = 60000;
static const unsigned long MQTT_CONNECT_TIMEOUT_MS = 60000;
static const unsigned long PWRKEY_PULSE_MS = 1200;
static const int WEAK_STREAK_TO_HUNT = 2;
static const int APN_MAX = 4;

HardwareSerial Modem(2);
Servo antennaServo;
Preferences prefs;

#if WIFI_FALLBACK
WiFiClient wifiClient;
PubSubClient wifiMqtt(wifiClient);
unsigned long lastWifiAttemptMs = 0;
#endif

enum ServoMode { MODE_AUTO, MODE_MANUAL };

enum LtePhase {
  LTE_WAIT_SIM,
  LTE_READ_IMSI,
  LTE_WAIT_REG,
  LTE_SET_APN,
  LTE_ATTACH,
  LTE_NETOPEN,
  LTE_IPADDR,
  LTE_MQTT_START,
  LTE_MQTT_ACCQ,
  LTE_MQTT_CONNECT,
  LTE_MQTT_SUB,
  LTE_READY,
  LTE_BACKOFF
};

int servoAngle = 90;
int bestAngle = 90;
int bestSignal = 0;
int rssi = -1;                 // last valid 0–31, or -1 if never read
int lastCsqRaw = RSSI_UNKNOWN;
bool internet = false;         // true only when MQTT is actually connected
String networkName = "LTE";
String simStatus = "UNKNOWN";
String modemIp = "-";
String imsi = "";
int satellites = 0;            // 0 unless GNSS actually returns a count
ServoMode servoMode = MODE_AUTO;
int huntDir = 1;
int weakStreak = 0;
bool hunting = false;  // hold NVS heading until the first real CSQ says Weak

bool mqttConnected = false;
bool mqttStarted = false;
bool mqttAcquired = false;
LtePhase ltePhase = LTE_WAIT_SIM;
LtePhase resumeAfterBackoff = LTE_WAIT_SIM;
int apnIndex = 0;
int apnCount = 0;
const char* apnList[APN_MAX];
unsigned long lastStatusMs = 0;
unsigned long lastHuntStepMs = 0;
unsigned long lastModemPollMs = 0;
unsigned long lastSlowPollMs = 0;
unsigned long lastLteAttemptMs = 0;
unsigned long backoffUntilMs = 0;

String rxTopic;
String rxPayload;
int rxTopicLen = 0;
int rxPayloadLen = 0;
enum RxState { RX_IDLE, RX_TOPIC, RX_PAYLOAD };
RxState rxState = RX_IDLE;

String qualityFromRssi(int value) {
  if (value < 0 || value >= RSSI_UNKNOWN) return "No Signal";
  if (value >= RSSI_STRONG) return "Strong";
  if (value >= RSSI_GOOD) return "Good";
  if (value >= RSSI_FAIR) return "Fair";
  return "Weak";
}

int publishSignal() {
  if (rssi >= 0 && rssi <= 31) return rssi;
  return 0;
}

bool isFairOrBetter(int value) {
  return value >= RSSI_FAIR && value <= 31;
}

void applyServoAngle(int angle) {
  servoAngle = constrain(angle, SERVO_MIN, SERVO_MAX);
  antennaServo.write(servoAngle);
}

void rememberBest(int signalAtAngle, int angle) {
  if (signalAtAngle < 0 || signalAtAngle > 31) return;
  if (signalAtAngle <= bestSignal) return;
  bestSignal = signalAtAngle;
  bestAngle = angle;
  prefs.putInt("best_angle", bestAngle);
  prefs.putInt("best_signal", bestSignal);
  Serial.printf("New best: rssi=%d at %d deg (saved to NVS)\n", bestSignal, bestAngle);
}

void loadBestFromNvs() {
  prefs.begin("hub", false);
  bestAngle = constrain(prefs.getInt("best_angle", 90), SERVO_MIN, SERVO_MAX);
  bestSignal = constrain(prefs.getInt("best_signal", 0), 0, 31);
  servoAngle = bestAngle;
}

String escapeJson(const String& in) {
  String out;
  out.reserve(in.length() + 8);
  for (unsigned i = 0; i < in.length(); i++) {
    char c = in[i];
    if (c == '"' || c == '\\') {
      out += '\\';
      out += c;
    } else if (c >= 32 && c <= 126) {
      out += c;
    }
  }
  return out;
}

String extractQuoted(const String& src, int from = 0) {
  int a = src.indexOf('"', from);
  if (a < 0) return "";
  int b = src.indexOf('"', a + 1);
  if (b < 0) return "";
  return src.substring(a + 1, b);
}

bool jsonHasMode(const String& json, const char* mode) {
  String needle1 = String("\"mode\":\"") + mode + "\"";
  String needle2 = String("\"mode\": \"") + mode + "\"";
  return json.indexOf(needle1) >= 0 || json.indexOf(needle2) >= 0;
}

bool jsonGetInt(const String& json, const char* key, int& out) {
  String quoted = String("\"") + key + "\"";
  int k = json.indexOf(quoted);
  if (k < 0) return false;
  int colon = json.indexOf(':', k + quoted.length());
  if (colon < 0) return false;
  int i = colon + 1;
  while (i < (int)json.length() && (json[i] == ' ' || json[i] == '\t')) i++;
  bool neg = false;
  if (i < (int)json.length() && json[i] == '-') {
    neg = true;
    i++;
  }
  if (i >= (int)json.length() || json[i] < '0' || json[i] > '9') return false;
  int value = 0;
  while (i < (int)json.length() && json[i] >= '0' && json[i] <= '9') {
    value = value * 10 + (json[i] - '0');
    i++;
  }
  out = neg ? -value : value;
  return true;
}

void handleCommandJson(const String& cmd) {
  Serial.print("CMD ");
  Serial.println(cmd);

  if (jsonHasMode(cmd, "manual")) {
    servoMode = MODE_MANUAL;
    hunting = false;
    weakStreak = 0;
    int angle;
    if (jsonGetInt(cmd, "servo_angle", angle)) {
      applyServoAngle(angle);
    }
  } else if (jsonHasMode(cmd, "auto")) {
    servoMode = MODE_AUTO;
    weakStreak = 0;
    hunting = !isFairOrBetter(rssi);
  } else {
    int angle;
    if (jsonGetInt(cmd, "servo_angle", angle) && servoMode == MODE_MANUAL) {
      applyServoAngle(angle);
    }
  }
}

#if WIFI_FALLBACK
void handleWifiCommand(char* topic, byte* payload, unsigned int length) {
  String cmd;
  cmd.reserve(length + 1);
  for (unsigned int i = 0; i < length; i++) cmd += (char)payload[i];
  handleCommandJson(cmd);
}
#endif

void setMqttUp(bool up) {
  mqttConnected = up;
  internet = up;
  if (!up) {
    Serial.println("MQTT down — hunt/servo keep running");
  }
}

void addApn(const char* apn) {
  if (apnCount >= APN_MAX || !apn) return;
  for (int i = 0; i < apnCount; i++) {
    if (strcmp(apnList[i], apn) == 0) return;
  }
  apnList[apnCount++] = apn;
}

void chooseApnsFromImsi(const String& raw) {
  apnCount = 0;
  apnIndex = 0;
  String digits;
  for (unsigned i = 0; i < raw.length(); i++) {
    if (raw[i] >= '0' && raw[i] <= '9') digits += raw[i];
  }
  imsi = digits;
  String plmn = digits.substring(0, 5);
  Serial.print("IMSI ");
  Serial.print(imsi);
  Serial.print(" PLMN ");
  Serial.println(plmn);

  if (plmn == "51502" || plmn == "51501") {
    // Globe / TM (Touch Mobile)
    addApn("internet.globe.com.ph");
    addApn("internet");
  } else if (plmn == "51503" || plmn == "51505") {
    // Smart / TNT
    addApn("internet");
  } else if (plmn == "51566") {
    addApn("internet.dito.ph");
  } else {
    addApn("internet.globe.com.ph");
    addApn("internet");
    addApn("internet.dito.ph");
  }
}

void normalizeOperator(const String& name) {
  String u = name;
  u.toUpperCase();
  if (u.indexOf("TNT") >= 0) {
    networkName = "TNT";
  } else if (u.indexOf("SMART") >= 0) {
    networkName = "Smart";
  } else if (u.indexOf("DITO") >= 0) {
    networkName = "DITO";
  } else if (u.indexOf("TOUCH") >= 0 || u == "TM" || u.indexOf("TM ") == 0) {
    networkName = "TM";
  } else if (u.indexOf("GLOBE") >= 0 || u.indexOf("GOM") >= 0) {
    networkName = "Globe";
  } else if (name.length() > 0) {
    networkName = name;
  }
}

void applyImsiFallbackOperator() {
  if (networkName != "LTE") return;
  if (imsi.startsWith("51502") || imsi.startsWith("51501")) networkName = "Globe";
  else if (imsi.startsWith("51503") || imsi.startsWith("51505")) networkName = "Smart";
  else if (imsi.startsWith("51566")) networkName = "DITO";
}

void huntStep() {
  if (servoMode != MODE_AUTO || !hunting) return;
  if (millis() - lastHuntStepMs < HUNT_STEP_MS) return;
  lastHuntStepMs = millis();

  int next = servoAngle + (huntDir * HUNT_STEP_DEG);
  if (next >= SERVO_MAX) {
    next = SERVO_MAX;
    huntDir = -1;
  } else if (next <= SERVO_MIN) {
    next = SERVO_MIN;
    huntDir = 1;
  }
  applyServoAngle(next);
}

void parseCsq(const String& resp);
void parseCpin(const String& resp);
void parseCops(const String& resp);
bool parseRegistered(const String& resp, const char* tag);
void parseCgpaddr(const String& resp);
void parseIpaddr(const String& resp);
void parseGnssSatellites(const String& resp);
void parseImsi(const String& resp);

void finishMqttRx() {
  if (rxTopic.indexOf(MQTT_COMMAND_TOPIC) >= 0 && rxPayload.length() > 0) {
    handleCommandJson(rxPayload);
  }
  rxTopic = "";
  rxPayload = "";
  rxTopicLen = 0;
  rxPayloadLen = 0;
  rxState = RX_IDLE;
}

void onModemLine(const String& line) {
  if (line.startsWith("+CMQTTCONNLOST")) {
    setMqttUp(false);
    if (ltePhase == LTE_READY) ltePhase = LTE_MQTT_CONNECT;
    return;
  }
  if (line.startsWith("+CMQTTCONNECT:")) {
    int comma = line.indexOf(',');
    if (comma > 0) {
      int err = line.substring(comma + 1).toInt();
      if (err == 0) {
        setMqttUp(true);
        Serial.println("MQTT connected over LTE");
      } else {
        setMqttUp(false);
        Serial.printf("MQTT connect error %d\n", err);
      }
    }
    return;
  }
  if (line.startsWith("+NETCLOSE")) {
    if (ltePhase == LTE_READY || ltePhase == LTE_MQTT_CONNECT || ltePhase == LTE_MQTT_SUB) {
      mqttStarted = false;
      mqttAcquired = false;
      setMqttUp(false);
      ltePhase = LTE_NETOPEN;
    }
    return;
  }
  if (line.startsWith("+CMQTTRXSTART:")) {
    rxTopic = "";
    rxPayload = "";
    rxState = RX_IDLE;
    int c1 = line.indexOf(',');
    int c2 = line.indexOf(',', c1 + 1);
    if (c1 > 0 && c2 > c1) {
      rxTopicLen = line.substring(c1 + 1, c2).toInt();
      rxPayloadLen = line.substring(c2 + 1).toInt();
    }
    return;
  }
  if (line.startsWith("+CMQTTRXTOPIC:")) {
    rxState = RX_TOPIC;
    rxTopic = "";
    return;
  }
  if (line.startsWith("+CMQTTRXPAYLOAD:")) {
    rxState = RX_PAYLOAD;
    rxPayload = "";
    return;
  }
  if (line.startsWith("+CMQTTRXEND")) {
    finishMqttRx();
    return;
  }

  if (rxState == RX_TOPIC) {
    rxTopic += line;
    if ((int)rxTopic.length() >= rxTopicLen) rxState = RX_IDLE;
    return;
  }
  if (rxState == RX_PAYLOAD) {
    if (rxPayload.length()) rxPayload += '\n';
    rxPayload += line;
    if ((int)rxPayload.length() >= rxPayloadLen) rxState = RX_IDLE;
  }
}

void pumpBackground() {
  huntStep();
  yield();
}

String sendAT(const char* command, unsigned long timeoutMs = AT_TIMEOUT_MS) {
  while (Modem.available()) {
    String leftover;
    leftover.reserve(64);
    while (Modem.available()) leftover += (char)Modem.read();
    leftover.replace("\r", "\n");
    int start = 0;
    while (start < (int)leftover.length()) {
      int nl = leftover.indexOf('\n', start);
      if (nl < 0) nl = leftover.length();
      String line = leftover.substring(start, nl);
      line.trim();
      if (line.length()) onModemLine(line);
      start = nl + 1;
    }
  }

  if (command && command[0]) {
    Modem.print(command);
    Modem.print("\r\n");
  }

  String resp;
  resp.reserve(256);
  String line;
  unsigned long start = millis();
  bool done = false;
  while (millis() - start < timeoutMs) {
    pumpBackground();
    while (Modem.available()) {
      char c = (char)Modem.read();
      resp += c;
      if (c == '\n') {
        line.trim();
        if (line.length()) onModemLine(line);
        line = "";
      } else if (c != '\r') {
        line += c;
      }
      if (resp.indexOf("OK") >= 0 || resp.indexOf("ERROR") >= 0) {
        unsigned long drainUntil = millis() + 40;
        while (millis() < drainUntil) {
          pumpBackground();
          while (Modem.available()) {
            char d = (char)Modem.read();
            resp += d;
            if (d == '\n') {
              line.trim();
              if (line.length()) onModemLine(line);
              line = "";
            } else if (d != '\r') {
              line += d;
            }
          }
        }
        done = true;
        break;
      }
    }
    if (done) break;
  }
  return resp;
}

String sendATWait(const char* command, const char* needle, unsigned long timeoutMs) {
  String resp = sendAT(command, timeoutMs);
  if (resp.indexOf(needle) >= 0) return resp;

  String extra;
  extra.reserve(128);
  String line;
  unsigned long start = millis();
  while (millis() - start < timeoutMs) {
    pumpBackground();
    while (Modem.available()) {
      char c = (char)Modem.read();
      extra += c;
      resp += c;
      if (c == '\n') {
        line.trim();
        if (line.length()) onModemLine(line);
        line = "";
      } else if (c != '\r') {
        line += c;
      }
      if (resp.indexOf(needle) >= 0) return resp;
    }
  }
  return resp;
}

bool sendATPrompt(const char* command, const char* data, int dataLen, unsigned long timeoutMs = 4000) {
  while (Modem.available()) Modem.read();
  Modem.print(command);
  Modem.print("\r\n");

  unsigned long start = millis();
  bool sawPrompt = false;
  String line;
  String resp;
  while (millis() - start < timeoutMs) {
    pumpBackground();
    while (Modem.available()) {
      char c = (char)Modem.read();
      resp += c;
      if (c == '>') {
        sawPrompt = true;
        break;
      }
      if (c == '\n') {
        line.trim();
        if (line.length()) onModemLine(line);
        line = "";
      } else if (c != '\r') {
        line += c;
      }
    }
    if (sawPrompt) break;
  }
  if (!sawPrompt) {
    Serial.printf("No '>' for %s\n", command);
    return false;
  }

  for (int i = 0; i < dataLen; i++) Modem.write((uint8_t)data[i]);

  start = millis();
  bool ok = false;
  while (millis() - start < timeoutMs) {
    pumpBackground();
    while (Modem.available()) {
      char c = (char)Modem.read();
      resp += c;
      if (c == '\n') {
        line.trim();
        if (line.length()) onModemLine(line);
        line = "";
      } else if (c != '\r') {
        line += c;
      }
      if (resp.indexOf("OK") >= 0) {
        ok = true;
        break;
      }
      if (resp.indexOf("ERROR") >= 0) return false;
    }
    if (ok) break;
  }
  return ok;
}

bool modemAnswers() {
  String r = sendAT("AT", 800);
  return r.indexOf("OK") >= 0;
}

bool openModemAt(long baud) {
  Modem.begin(baud, SERIAL_8N1, MODEM_RX_PIN, MODEM_TX_PIN);
  delay(250);
  return modemAnswers();
}

bool waitForAt(unsigned long giveUpMs) {
  unsigned long start = millis();
  while (millis() - start < giveUpMs) {
    huntStep();
    if (modemAnswers()) return true;
    delay(400);
  }
  return false;
}

void pulsePwrkey() {
  Serial.println("PWRKEY: pulse LOW 1.2s (modem silent)");
  pinMode(PWRKEY_PIN, OUTPUT);
  digitalWrite(PWRKEY_PIN, HIGH);
  delay(50);
  digitalWrite(PWRKEY_PIN, LOW);
  delay(PWRKEY_PULSE_MS);
  digitalWrite(PWRKEY_PIN, HIGH);
}

bool ensureModemAwake() {
  pinMode(PWRKEY_PIN, OUTPUT);
  digitalWrite(PWRKEY_PIN, HIGH);

  Serial.println("Trying modem at 115200...");
  if (openModemAt(MODEM_BAUD) || waitForAt(5000)) {
    Serial.println("Modem already answering AT @ 115200");
    sendAT("ATE0", 800);
    return true;
  }

  pulsePwrkey();
  Serial.println("Waiting up to 12s for A7670C boot...");
  if (openModemAt(MODEM_BAUD) || waitForAt(12000)) {
    sendAT("ATE0", 800);
    return true;
  }

  Serial.println("Trying 9600 (some A7670C boards ship at this baud)...");
  if (openModemAt(9600L) || waitForAt(2000)) {
    Serial.println("Modem answering @ 9600. Leaving baud as-is.");
    sendAT("ATE0", 800);
    return true;
  }

  Serial.println("Modem still silent after PWRKEY. Check UART (TX/RX crossed), GND, and 5 V on A7670C VIN.");
  Modem.begin(MODEM_BAUD, SERIAL_8N1, MODEM_RX_PIN, MODEM_TX_PIN);
  return false;
}

void parseCsq(const String& resp) {
  int tag = resp.indexOf("+CSQ:");
  if (tag < 0) {
    lastCsqRaw = RSSI_UNKNOWN;
    return;
  }
  int n = 0;
  int i = tag + 5;
  while (i < (int)resp.length() && (resp[i] == ' ' || resp[i] == '\t')) i++;
  if (i >= (int)resp.length() || resp[i] < '0' || resp[i] > '9') {
    lastCsqRaw = RSSI_UNKNOWN;
    return;
  }
  while (i < (int)resp.length() && resp[i] >= '0' && resp[i] <= '9') {
    n = n * 10 + (resp[i] - '0');
    i++;
  }
  lastCsqRaw = n;
  if (n >= 0 && n <= 31) {
    rssi = n;
    rememberBest(rssi, servoAngle);
  }
}

void parseCpin(const String& resp) {
  if (resp.indexOf("READY") >= 0) {
    simStatus = "READY";
    return;
  }
  if (resp.indexOf("SIM PIN") >= 0) {
    simStatus = "SIM PIN";
    return;
  }
  if (resp.indexOf("SIM PUK") >= 0) {
    simStatus = "SIM PUK";
    return;
  }
  if (resp.indexOf("NOT INSERTED") >= 0 || resp.indexOf("CME ERROR: 10") >= 0) {
    simStatus = "NO SIM";
    return;
  }
  if (resp.indexOf("ERROR") >= 0) {
    simStatus = "ERROR";
    return;
  }
}

void parseCops(const String& resp) {
  int tag = resp.indexOf("+COPS:");
  if (tag < 0) return;
  String name = extractQuoted(resp, tag);
  name.trim();
  if (name.length() == 0) return;
  bool digits = true;
  for (unsigned i = 0; i < name.length(); i++) {
    if (name[i] < '0' || name[i] > '9') {
      digits = false;
      break;
    }
  }
  if (digits) {
    if (name.startsWith("51502") || name == "51502") networkName = "Globe";
    else if (name.startsWith("51503") || name == "51503") networkName = "Smart";
    else if (name.startsWith("51566") || name == "51566") networkName = "DITO";
    else networkName = "LTE";
    applyImsiFallbackOperator();
  } else {
    normalizeOperator(name);
  }
}

bool parseRegistered(const String& resp, const char* tag) {
  int t = resp.indexOf(tag);
  if (t < 0) return false;
  int comma = resp.indexOf(',', t);
  if (comma < 0) return false;
  int i = comma + 1;
  while (i < (int)resp.length() && (resp[i] == ' ' || resp[i] == '"')) i++;
  int stat = 0;
  if (i >= (int)resp.length() || resp[i] < '0' || resp[i] > '9') return false;
  while (i < (int)resp.length() && resp[i] >= '0' && resp[i] <= '9') {
    stat = stat * 10 + (resp[i] - '0');
    i++;
  }
  return stat == 1 || stat == 5;
}

void parseCgpaddr(const String& resp) {
  int tag = resp.indexOf("+CGPADDR:");
  if (tag < 0) return;
  String quoted = extractQuoted(resp, tag);
  if (quoted.length() >= 7 && quoted.indexOf('.') > 0) {
    modemIp = quoted;
    return;
  }
  int comma = resp.indexOf(',', tag);
  if (comma < 0) return;
  String ip = resp.substring(comma + 1);
  ip.replace("\r", "");
  ip.replace("\n", "");
  ip.replace("\"", "");
  ip.replace("OK", "");
  ip.trim();
  int cut = ip.indexOf(' ');
  if (cut > 0) ip = ip.substring(0, cut);
  if (ip.length() >= 7 && ip.indexOf('.') > 0) modemIp = ip;
}

void parseIpaddr(const String& resp) {
  int tag = resp.indexOf("+IPADDR:");
  if (tag < 0) return;
  String ip = resp.substring(tag + 8);
  ip.replace("\r", "");
  ip.replace("\n", "");
  ip.replace("\"", "");
  ip.replace("OK", "");
  ip.trim();
  int cut = ip.indexOf(' ');
  if (cut > 0) ip = ip.substring(0, cut);
  if (ip.length() >= 7 && ip.indexOf('.') > 0) modemIp = ip;
}

void parseImsi(const String& resp) {
  String digits;
  for (unsigned i = 0; i < resp.length(); i++) {
    if (resp[i] >= '0' && resp[i] <= '9') digits += resp[i];
    else if (digits.length() >= 14) break;
    else if (resp[i] == '\n' || resp[i] == '\r') {
      if (digits.length() >= 14) break;
      digits = "";
    }
  }
  if (digits.length() >= 14) chooseApnsFromImsi(digits);
}

void parseGnssSatellites(const String& resp) {
  int tag = resp.indexOf("+CGNSSINFO:");
  if (tag < 0) return;
  int i = tag + 11;
  int field = 0;
  int gps = -1, glo = -1, bds = -1;
  while (i < (int)resp.length() && field < 4) {
    while (i < (int)resp.length() && (resp[i] == ' ' || resp[i] == '\t')) i++;
    int n = 0;
    bool any = false;
    while (i < (int)resp.length() && resp[i] >= '0' && resp[i] <= '9') {
      n = n * 10 + (resp[i] - '0');
      i++;
      any = true;
    }
    if (field == 1 && any) gps = n;
    if (field == 2 && any) glo = n;
    if (field == 3 && any) bds = n;
    int comma = resp.indexOf(',', i);
    if (comma < 0) break;
    i = comma + 1;
    field++;
  }
  if (gps < 0 && glo < 0 && bds < 0) return;
  int total = 0;
  if (gps > 0) total += gps;
  if (glo > 0) total += glo;
  if (bds > 0) total += bds;
  satellites = total;
}

void pollCsq() {
  parseCsq(sendAT("AT+CSQ"));
}

void pollModemIdentity() {
  parseCpin(sendAT("AT+CPIN?"));
  parseCops(sendAT("AT+COPS?"));
  parseCgpaddr(sendAT("AT+CGPADDR=1"));
  parseGnssSatellites(sendAT("AT+CGNSSINFO", 800));
  if (networkName.length() == 0 || networkName == "WiFi") networkName = "LTE";
  applyImsiFallbackOperator();
}

void updateHuntState() {
  if (servoMode != MODE_AUTO) {
    hunting = false;
    weakStreak = 0;
    return;
  }

  if (isFairOrBetter(rssi)) {
    weakStreak = 0;
    if (hunting) {
      hunting = false;
      Serial.printf("Hold at %d deg (rssi=%d, %s)\n",
                    servoAngle, rssi, qualityFromRssi(rssi).c_str());
    }
    return;
  }

  weakStreak++;
  if (weakStreak >= WEAK_STREAK_TO_HUNT && !hunting) {
    hunting = true;
    Serial.printf("Signal dropped (%s, rssi=%d) — hunting\n",
                  qualityFromRssi(rssi).c_str(), rssi);
  }
  if (rssi < 0) hunting = true;
}

bool atOk(const String& resp) {
  return resp.indexOf("OK") >= 0 && resp.indexOf("ERROR") < 0;
}

bool atOkOrAlready(const String& resp) {
  if (resp.indexOf("OK") >= 0) return true;
  String u = resp;
  u.toUpperCase();
  return u.indexOf("ALREADY") >= 0 || u.indexOf("OPENED") >= 0;
}

void enterBackoff(unsigned long waitMs, LtePhase resume) {
  resumeAfterBackoff = resume;
  backoffUntilMs = millis() + waitMs;
  ltePhase = LTE_BACKOFF;
}

void nextApnOrBackoff(const char* why) {
  Serial.println(why);
  if (apnIndex + 1 < apnCount) {
    apnIndex++;
    Serial.printf("Trying next APN: %s\n", apnList[apnIndex]);
    ltePhase = LTE_SET_APN;
    return;
  }
  apnIndex = 0;
  enterBackoff(LTE_RETRY_MS, LTE_WAIT_REG);
}

void serviceLteMqtt() {
  if (ltePhase == LTE_BACKOFF) {
    if (millis() < backoffUntilMs) return;
    ltePhase = resumeAfterBackoff;
  }

  if (ltePhase == LTE_READY) {
    internet = mqttConnected;
    return;
  }

  if (lastLteAttemptMs != 0 && millis() - lastLteAttemptMs < 80) return;

  switch (ltePhase) {
    case LTE_WAIT_SIM: {
      String r = sendAT("AT+CPIN?", 2000);
      parseCpin(r);
      if (simStatus == "READY") {
        sendAT("AT+COPS=0", 3000);
        ltePhase = LTE_READ_IMSI;
      } else {
        Serial.print("Waiting for SIM: ");
        Serial.println(simStatus);
        enterBackoff(LTE_RETRY_MS, LTE_WAIT_SIM);
      }
      break;
    }
    case LTE_READ_IMSI: {
      String r = sendAT("AT+CIMI", 2000);
      parseImsi(r);
      if (apnCount == 0) chooseApnsFromImsi("");
      Serial.printf("APN[0]=%s\n", apnList[0]);
      ltePhase = LTE_WAIT_REG;
      break;
    }
    case LTE_WAIT_REG: {
      bool reg = parseRegistered(sendAT("AT+CEREG?", 2000), "+CEREG:");
      if (!reg) reg = parseRegistered(sendAT("AT+CGREG?", 2000), "+CGREG:");
      if (!reg) reg = parseRegistered(sendAT("AT+CREG?", 2000), "+CREG:");
      parseCops(sendAT("AT+COPS?"));
      if (reg) {
        Serial.println("Registered on LTE — setting APN");
        ltePhase = LTE_SET_APN;
      } else {
        Serial.println("Waiting for LTE registration (servo still hunts)");
        enterBackoff(3000, LTE_WAIT_REG);
      }
    }
    case LTE_SET_APN: {
      if (apnCount == 0) chooseApnsFromImsi(imsi);
      if (apnIndex >= apnCount) apnIndex = 0;
      const char* apn = apnList[apnIndex];
      char cmd[96];
      snprintf(cmd, sizeof(cmd), "AT+CGDCONT=1,\"IP\",\"%s\"", apn);
      Serial.print("CGDCONT ");
      Serial.println(apn);
      sendAT(cmd, 3000);
      ltePhase = LTE_ATTACH;
      break;
    }
    case LTE_ATTACH: {
      String r = sendAT("AT+CGATT=1", CGATT_TIMEOUT_MS);
      if (atOkOrAlready(r) || r.indexOf("+CGATT: 1") >= 0) {
        ltePhase = LTE_NETOPEN;
      } else {
        nextApnOrBackoff("CGATT failed");
      }
      break;
    }
    case LTE_NETOPEN: {
      String st = sendAT("AT+NETOPEN?", 2000);
      if (st.indexOf("+NETOPEN: 1") >= 0) {
        ltePhase = LTE_IPADDR;
        break;
      }
      String r = sendATWait("AT+NETOPEN", "+NETOPEN:", NETOPEN_TIMEOUT_MS);
      if (r.indexOf("+NETOPEN: 0") >= 0 || r.indexOf("+NETOPEN: 1") >= 0 ||
          r.indexOf("already") >= 0 || r.indexOf("ALREADY") >= 0) {
        ltePhase = LTE_IPADDR;
      } else {
        sendAT("AT+NETCLOSE", 5000);
        nextApnOrBackoff("NETOPEN failed");
      }
      break;
    }
    case LTE_IPADDR: {
      String r = sendAT("AT+IPADDR", 3000);
      parseIpaddr(r);
      if (modemIp == "-") parseCgpaddr(sendAT("AT+CGPADDR=1"));
      Serial.print("PDP IP ");
      Serial.println(modemIp);
      ltePhase = LTE_MQTT_START;
      break;
    }
    case LTE_MQTT_START: {
      String r = sendAT("AT+CMQTTSTART", 15000);
      if (atOkOrAlready(r) || r.indexOf("+CMQTTSTART: 0") >= 0) {
        mqttStarted = true;
        ltePhase = LTE_MQTT_ACCQ;
      } else if (r.indexOf("+CMQTTSTART:") >= 0) {
        // already started (non-zero but service is up)
        mqttStarted = true;
        ltePhase = LTE_MQTT_ACCQ;
      } else {
        Serial.println("CMQTTSTART failed — will retry. Hunt keeps running.");
        enterBackoff(MQTT_RETRY_MS, LTE_MQTT_START);
      }
      break;
    }
    case LTE_MQTT_ACCQ: {
      // No SSL — omit server_type or pass 0. Never use 8883.
      char cmd[80];
      snprintf(cmd, sizeof(cmd), "AT+CMQTTACCQ=0,\"%s\"", MQTT_CLIENT_ID);
      String r = sendAT(cmd, 5000);
      if (atOkOrAlready(r) || r.indexOf("ERROR") < 0) {
        mqttAcquired = true;
        ltePhase = LTE_MQTT_CONNECT;
      } else {
        sendAT("AT+CMQTTREL=0", 3000);
        enterBackoff(MQTT_RETRY_MS, LTE_MQTT_ACCQ);
      }
      break;
    }
    case LTE_MQTT_CONNECT: {
      char cmd[96];
      snprintf(cmd, sizeof(cmd), "AT+CMQTTCONNECT=0,\"%s\",60,1", MQTT_URL);
      Serial.println("CMQTTCONNECT tcp://broker.emqx.io:1883");
      String r = sendATWait(cmd, "+CMQTTCONNECT:", MQTT_CONNECT_TIMEOUT_MS);
      int tag = r.indexOf("+CMQTTCONNECT:");
      bool ok = false;
      if (tag >= 0) {
        int comma = r.indexOf(',', tag);
        ok = comma > 0 && r.substring(comma + 1).toInt() == 0;
      }
      if (ok || mqttConnected) {
        setMqttUp(true);
        ltePhase = LTE_MQTT_SUB;
      } else {
        setMqttUp(false);
        Serial.println("MQTT connect failed — hunt keeps running");
        sendAT("AT+CMQTTDISC=0,60", 5000);
        enterBackoff(MQTT_RETRY_MS, LTE_MQTT_CONNECT);
      }
      break;
    }
    case LTE_MQTT_SUB: {
      int tlen = strlen(MQTT_COMMAND_TOPIC);
      char cmd[48];
      snprintf(cmd, sizeof(cmd), "AT+CMQTTSUB=0,%d,1", tlen);
      if (sendATPrompt(cmd, MQTT_COMMAND_TOPIC, tlen, 8000)) {
        Serial.println("Subscribed to command topic");
        ltePhase = LTE_READY;
        lastStatusMs = 0;
      } else {
        // try two-step subscribe
        snprintf(cmd, sizeof(cmd), "AT+CMQTTSUBTOPIC=0,%d,1", tlen);
        sendATPrompt(cmd, MQTT_COMMAND_TOPIC, tlen, 5000);
        String r = sendATWait("AT+CMQTTSUB=0", "+CMQTTSUB:", 8000);
        if (r.indexOf("+CMQTTSUB: 0,0") >= 0 || r.indexOf("OK") >= 0) {
          ltePhase = LTE_READY;
          lastStatusMs = 0;
        } else {
          enterBackoff(MQTT_RETRY_MS, LTE_MQTT_CONNECT);
        }
      }
      break;
    }
    default:
      break;
  }
  lastLteAttemptMs = millis();
}

#if WIFI_FALLBACK
void connectWifiFallback() {
  if (mqttConnected) return;
  if (WiFi.status() == WL_CONNECTED) {
    if (wifiMqtt.connected()) {
      wifiMqtt.loop();
      internet = true;
      mqttConnected = true;
      return;
    }
    if (lastWifiAttemptMs != 0 && millis() - lastWifiAttemptMs < MQTT_RETRY_MS) return;
    lastWifiAttemptMs = millis();
    char clientId[32];
    snprintf(clientId, sizeof(clientId), "hub1-wifi");
    if (wifiMqtt.connect(clientId)) {
      wifiMqtt.subscribe(MQTT_COMMAND_TOPIC);
      setMqttUp(true);
      Serial.println("WIFI_FALLBACK MQTT up");
    }
    return;
  }
  if (lastWifiAttemptMs != 0 && millis() - lastWifiAttemptMs < 8000) return;
  lastWifiAttemptMs = millis();
  Serial.println("WIFI_FALLBACK connecting");
  WiFi.mode(WIFI_STA);
  WiFi.setSleep(false);
  WiFi.begin(WIFI_SSID, WIFI_PASS);
}
#endif

String buildStatusJson() {
  String quality = qualityFromRssi(rssi);
  String json;
  json.reserve(280);
  json += "{";
  json += "\"internet\":";
  json += internet ? "true" : "false";
  json += ",\"network\":\"";
  json += escapeJson(networkName == "WiFi" ? String("LTE") : networkName);
  json += "\",\"signal\":";
  json += String(publishSignal());
  json += ",\"signal_quality\":\"";
  json += escapeJson(quality);
  json += "\",\"servo_angle\":";
  json += String(servoAngle);
  json += ",\"best_angle\":";
  json += String(bestAngle);
  json += ",\"best_signal\":";
  json += String(bestSignal);
  json += ",\"satellites\":";
  json += String(satellites);
  json += ",\"sim\":\"";
  json += escapeJson(simStatus);
  json += "\",\"ip\":\"";
  json += escapeJson(modemIp);
  json += "\",\"servo_mode\":\"";
  json += (servoMode == MODE_MANUAL) ? "manual" : "auto";
  json += "\"}";
  return json;
}

void publishStatusLte() {
  if (!mqttConnected) return;
  String json = buildStatusJson();
  int tlen = strlen(MQTT_STATUS_TOPIC);
  int plen = json.length();
  char tcmd[48];
  char pcmd[48];
  snprintf(tcmd, sizeof(tcmd), "AT+CMQTTTOPIC=0,%d", tlen);
  snprintf(pcmd, sizeof(pcmd), "AT+CMQTTPAYLOAD=0,%d", plen);
  if (!sendATPrompt(tcmd, MQTT_STATUS_TOPIC, tlen, 4000)) {
    Serial.println("MQTT topic failed");
    return;
  }
  if (!sendATPrompt(pcmd, json.c_str(), plen, 4000)) {
    Serial.println("MQTT payload failed");
    return;
  }
  // qos=1, timeout=60, retained=1
  String r = sendATWait("AT+CMQTTPUB=0,1,60,1", "+CMQTTPUB:", 8000);
  if (r.indexOf("+CMQTTPUB: 0,0") >= 0 || r.indexOf("OK") >= 0) {
    Serial.println(json);
  } else {
    r = sendATWait("AT+CMQTTPUB=0,1,60", "+CMQTTPUB:", 8000);
    if (r.indexOf("OK") >= 0 || r.indexOf("+CMQTTPUB: 0,0") >= 0) {
      Serial.println(json);
    } else {
      Serial.println("MQTT publish failed");
      setMqttUp(false);
      ltePhase = LTE_MQTT_CONNECT;
    }
  }
}

#if WIFI_FALLBACK
void publishStatusWifi() {
  if (!wifiMqtt.connected()) return;
  String json = buildStatusJson();
  if (wifiMqtt.publish(MQTT_STATUS_TOPIC, json.c_str(), true)) {
    Serial.println(json);
  }
}
#endif

void publishStatus() {
#if WIFI_FALLBACK
  if (wifiMqtt.connected()) {
    publishStatusWifi();
    return;
  }
#endif
  publishStatusLte();
}

void setup() {
  Serial.begin(115200);
  delay(200);
  Serial.println();
  Serial.println("Lantapan Hub  ESP32 + A7670C  MQTT-over-SIM");
  Serial.println("GPIO16 RX2 <- modem TX | GPIO17 TX2 -> modem RX | GPIO27 PWRKEY | GPIO13 servo");
#if WIFI_FALLBACK
  Serial.println("WIFI_FALLBACK=1 (optional). Prefer LTE MQTT.");
#else
  Serial.println("WIFI_FALLBACK=0 — no WiFi required. TM SIM needs mobile data.");
#endif

  loadBestFromNvs();

  ESP32PWM::allocateTimer(0);
  ESP32PWM::allocateTimer(1);
  ESP32PWM::allocateTimer(2);
  ESP32PWM::allocateTimer(3);
  antennaServo.setPeriodHertz(50);
  antennaServo.attach(SERVO_PIN, SERVO_MIN_US, SERVO_MAX_US);
  applyServoAngle(bestAngle);
  Serial.printf("Restored heading from NVS: %d deg (best rssi %d)\n", bestAngle, bestSignal);

  ensureModemAwake();
  sendAT("AT+CMEE=2", 800);

#if WIFI_FALLBACK
  wifiMqtt.setServer(MQTT_BROKER_HOST, MQTT_PORT);
  wifiMqtt.setCallback(handleWifiCommand);
  wifiMqtt.setBufferSize(512);
  wifiMqtt.setKeepAlive(30);
#endif

  lastModemPollMs = 0;
  ltePhase = LTE_WAIT_SIM;
}

void loop() {
  serviceLteMqtt();
#if WIFI_FALLBACK
  connectWifiFallback();
#endif

  huntStep();

  unsigned long pollEvery = hunting ? MODEM_POLL_HUNT_MS : MODEM_POLL_HOLD_MS;
  if (lastModemPollMs == 0 || millis() - lastModemPollMs >= pollEvery) {
    lastModemPollMs = millis();
    pollCsq();
    updateHuntState();
  }

  if (lastSlowPollMs == 0 || millis() - lastSlowPollMs >= MODEM_SLOW_POLL_MS) {
    lastSlowPollMs = millis();
    pollModemIdentity();
  }

  unsigned long statusEvery = hunting ? STATUS_HUNT_MS : STATUS_HOLD_MS;
  if (lastStatusMs == 0 || millis() - lastStatusMs >= statusEvery) {
    lastStatusMs = millis();
    publishStatus();
  }
}
