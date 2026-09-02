/*
 * Lantapan Hub — ESP32 + SIMCom A7670C signal-seeking antenna
 *
 * Cellular MQTT (A7670C AT, plain TCP :1883). No WiFi required.
 * Real AT+CSQ only — no mock / random RSSI. Do not invent satellites.
 *
 * Libraries: ESP32Servo (ESP32 Arduino core: Preferences)
 * Optional WiFi backhaul is compile-time WIFI_FALLBACK 0 (off).
 *
 * ========== PIN MAP (every wire) ==========
 * A7670C TX          → ESP32 GPIO 16 (UART2 RX)
 * A7670C RX          → ESP32 GPIO 17 (UART2 TX)
 * A7670C PWRKEY      → ESP32 GPIO 27  (pulse LOW ~1.2s if AT is silent)
 * Servo signal       → ESP32 GPIO 13  (yellow/orange)
 * A7670C VIN         → boost 5.0 V   (board has VIN only — no VBAT pad)
 * Servo VCC (red)    → boost 5.0 V
 * Servo GND (brown)  → common GND
 * Power switch       → battery +  (NOT a GPIO)
 * Boost EN (if any)  → switched battery + so the boost dies with the switch
 * Common GND         → Li-ion−, boost GND, A7670C GND, ESP32 GND, servo GND
 *
 * UART: 115200 8N1. Common GND is required or AT will never answer.
 *
 * ========== POWER PATH (this board: VIN only) ==========
 * Li-ion+ → POWER SWITCH → boost IN+
 * Boost OUT 5.0 V → A7670C VIN + ESP32 VIN + servo VCC
 * Li-ion− → common GND
 *
 * Do not wire the cell to the modem. VIN is the 5 V input; the breakout
 * already regulates to ~3.8 V internally. 470 µF+ on A7670C VIN/GND if room.
 */

#include <ESP32Servo.h>
#include <Preferences.h>

#ifndef WIFI_FALLBACK
#define WIFI_FALLBACK 0
#endif

#if WIFI_FALLBACK
#include <WiFi.h>
#include <PubSubClient.h>
const char* WIFI_SSID = "YOUR_WIFI_SSID";
const char* WIFI_PASS = "YOUR_WIFI_PASS";
#endif

// ---------- MQTT (must match index.html; cellular TCP, not TLS) ----------
static const char* MQTT_HOST_URL = "tcp://broker.emqx.io:1883";
static const char* MQTT_CLIENT_ID = "signalbooster-hub1";
static const char* MQTT_STATUS_TOPIC = "signalbooster/hub1/status";
static const char* MQTT_COMMAND_TOPIC = "signalbooster/hub1/command";

// ---------- Pins ----------
static const int MODEM_RX_PIN = 16;   // ESP32 RX2  <- A7670C TX
static const int MODEM_TX_PIN = 17;   // ESP32 TX2  -> A7670C RX
static const int PWRKEY_PIN = 27;     // pulse LOW to boot modem on battery
static const int SERVO_PIN = 13;      // PWM to hobby servo

static const long MODEM_BAUD = 115200;
static const uint16_t SERVO_MIN_US = 500;
static const uint16_t SERVO_MAX_US = 2400;

// Same quality map as index.html. 3 UI bars = Fair = rssi >= 12.
static const int RSSI_STRONG = 25;
static const int RSSI_GOOD = 18;
static const int RSSI_FAIR = 12;
static const int RSSI_UNKNOWN = 99;

static const int SERVO_MIN = 0;
static const int SERVO_MAX = 180;
static const int HUNT_STEP_DEG = 2;
static const unsigned long HUNT_STEP_MS = 400;
static const unsigned long PUBLISH_HOLD_MS = 2000;
static const unsigned long PUBLISH_HUNT_MS = 5000;
static const unsigned long MODEM_POLL_HOLD_MS = 2000;
static const unsigned long MODEM_POLL_HUNT_MS = 700;
static const unsigned long MODEM_SLOW_POLL_MS = 8000;
static const unsigned long AT_TIMEOUT_MS = 1500;
static const unsigned long PWRKEY_PULSE_MS = 1200;
static const unsigned long REG_TIMEOUT_MS = 60000;
static const unsigned long RETRY_WAIT_MS = 12000;
static const int WEAK_STREAK_TO_HUNT = 2;
static const int APN_MAX = 3;

HardwareSerial Modem(2);
Servo antennaServo;
Preferences prefs;

#if WIFI_FALLBACK
WiFiClient wifiClient;
PubSubClient wifiMqtt(wifiClient);
#endif

enum ServoMode { MODE_AUTO, MODE_MANUAL };

enum NetState {
  NET_WAIT_SIM,
  NET_WAIT_REG,
  NET_SET_APN,
  NET_ATTACH,
  NET_OPEN,
  NET_MQTT_START,
  NET_MQTT_ACCQ,
  NET_MQTT_CONNECT,
  NET_MQTT_SUB,
  NET_MQTT_UP,
  NET_RETRY_WAIT
};

int servoAngle = 90;
int bestAngle = 90;
int bestSignal = 0;
int rssi = -1;
int lastCsqRaw = RSSI_UNKNOWN;
bool internet = false;          // true only when MQTT is actually connected
bool mqttUp = false;
String networkName = "LTE";
String simStatus = "UNKNOWN";
String modemIp = "-";
String imsi = "";
int satellites = 0;             // stays 0 unless GNSS is actually parsed
ServoMode servoMode = MODE_AUTO;
int huntDir = 1;
int weakStreak = 0;
bool hunting = false;

NetState netState = NET_WAIT_SIM;
unsigned long netStateSince = 0;
unsigned long lastStatusMs = 0;
unsigned long lastHuntStepMs = 0;
unsigned long lastModemPollMs = 0;
unsigned long lastSlowPollMs = 0;
unsigned long lastRegPollMs = 0;
bool copsStarted = false;
bool inAtCommand = false;

const char* apnList[APN_MAX];
int apnCount = 0;
int apnIndex = 0;

enum RxPhase { RX_IDLE, RX_TOPIC, RX_PAYLOAD };
RxPhase rxPhase = RX_IDLE;
String rxLine;
String rxTopic;
String rxPayload;
int rxPayloadLen = 0;

void huntStep();
void updateHuntState();
void handleCommandJson(const String& cmd);
void onMqttLost();

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

bool allDigits(const String& s) {
  if (s.length() == 0) return false;
  for (unsigned i = 0; i < s.length(); i++) {
    if (s[i] < '0' || s[i] > '9') return false;
  }
  return true;
}

void setNetworkFromCops(const String& copsName) {
  String u = copsName;
  u.toUpperCase();
  if (u.indexOf("DITO") >= 0) {
    networkName = "DITO";
  } else if (u.indexOf("SMART") >= 0) {
    networkName = "Smart";
  } else if (u.indexOf("TNT") >= 0) {
    networkName = "TNT";
  } else if (u.indexOf("TM") >= 0) {
    networkName = "TM";
  } else if (u.indexOf("GLOBE") >= 0) {
    networkName = "Globe";
  } else if (copsName.length() && !allDigits(copsName)) {
    networkName = copsName;
  } else if (imsi.startsWith("51566")) {
    networkName = "DITO";
  } else if (imsi.startsWith("51503")) {
    networkName = "Smart";
  } else if (imsi.startsWith("51502")) {
    networkName = "Globe";
  } else if (networkName.length() == 0 || networkName == "WiFi") {
    networkName = "LTE";
  }
}

void addApn(const char* apn) {
  if (apnCount >= APN_MAX) return;
  apnList[apnCount++] = apn;
}

void buildApnList() {
  apnCount = 0;
  apnIndex = 0;
  if (imsi.startsWith("51502")) {
    addApn("internet.globe.com.ph");
    addApn("internet");
  } else if (imsi.startsWith("51503")) {
    addApn("internet");
  } else if (imsi.startsWith("51566")) {
    addApn("internet.dito.ph");
  } else {
    addApn("internet");
    addApn("internet.globe.com.ph");
    addApn("internet.dito.ph");
  }
}

void enterState(NetState next) {
  netState = next;
  netStateSince = millis();
}

int parseAfterTag(const String& resp, const char* tag) {
  int t = resp.indexOf(tag);
  if (t < 0) return -1;
  int i = t + strlen(tag);
  while (i < (int)resp.length() && (resp[i] == ' ' || resp[i] == '\t' || resp[i] == ':')) i++;
  if (i < (int)resp.length() && resp[i] == ':') {
    i++;
    while (i < (int)resp.length() && (resp[i] == ' ' || resp[i] == '\t')) i++;
  }
  if (i >= (int)resp.length() || resp[i] < '0' || resp[i] > '9') return -1;
  int n = 0;
  while (i < (int)resp.length() && resp[i] >= '0' && resp[i] <= '9') {
    n = n * 10 + (resp[i] - '0');
    i++;
  }
  return n;
}

int parseSecondField(const String& resp, const char* tag) {
  int t = resp.indexOf(tag);
  if (t < 0) return -1;
  int comma = resp.indexOf(',', t);
  if (comma < 0) return -1;
  int i = comma + 1;
  while (i < (int)resp.length() && (resp[i] == ' ' || resp[i] == '\t')) i++;
  if (i >= (int)resp.length() || resp[i] < '0' || resp[i] > '9') return -1;
  int n = 0;
  while (i < (int)resp.length() && resp[i] >= '0' && resp[i] <= '9') {
    n = n * 10 + (resp[i] - '0');
    i++;
  }
  return n;
}

void onMqttLost() {
  if (mqttUp) Serial.println("MQTT lost — hunt keeps running, will retry cellular");
  mqttUp = false;
  internet = false;
  if (netState == NET_MQTT_UP) enterState(NET_RETRY_WAIT);
}

void handleRxLine(const String& line) {
  if (line.indexOf("+CMQTTCONNLOST:") >= 0 ||
      line.indexOf("+CMQTTNONET:") >= 0 ||
      line.indexOf("+CMQTTDISC:") >= 0) {
    onMqttLost();
    return;
  }

  if (line.startsWith("+CMQTTRXSTART:")) {
    rxPhase = RX_IDLE;
    rxTopic = "";
    rxPayload = "";
    rxPayloadLen = 0;
    return;
  }
  if (line.startsWith("+CMQTTRXTOPIC:")) {
    rxPhase = RX_TOPIC;
    rxTopic = "";
    return;
  }
  if (line.startsWith("+CMQTTRXPAYLOAD:")) {
    rxPayloadLen = parseSecondField(line, "+CMQTTRXPAYLOAD:");
    if (rxPayloadLen < 0) rxPayloadLen = 0;
    rxPayload = "";
    rxPhase = RX_PAYLOAD;
    return;
  }
  if (line.startsWith("+CMQTTRXEND:")) {
    if (rxPayload.length()) handleCommandJson(rxPayload);
    rxPhase = RX_IDLE;
    rxTopic = "";
    rxPayload = "";
    return;
  }

  if (rxPhase == RX_TOPIC) {
    rxTopic = line;
    rxPhase = RX_IDLE;
  }
}

void feedUrcByte(char c) {
  if (rxPhase == RX_PAYLOAD) {
    rxPayload += c;
    if (rxPayloadLen > 0 && (int)rxPayload.length() >= rxPayloadLen) {
      rxPhase = RX_IDLE;
    }
    return;
  }

  if (c == '\n') {
    rxLine.trim();
    if (rxLine.length()) handleRxLine(rxLine);
    rxLine = "";
  } else if (c != '\r') {
    if (rxLine.length() < 240) rxLine += c;
  }
}

void serviceFast() {
  huntStep();
#if WIFI_FALLBACK
  if (wifiMqtt.connected()) wifiMqtt.loop();
#endif
  yield();
}

String sendAT(const char* command, unsigned long timeoutMs = AT_TIMEOUT_MS,
              const char* extraWait = nullptr) {
  inAtCommand = true;
  while (Modem.available()) feedUrcByte((char)Modem.read());

  Modem.print(command);
  Modem.print("\r\n");

  String resp;
  resp.reserve(320);
  unsigned long start = millis();
  bool sawOkOrErr = false;

  while (millis() - start < timeoutMs) {
    serviceFast();
    while (Modem.available()) {
      char c = (char)Modem.read();
      resp += c;
      feedUrcByte(c);
      if (!sawOkOrErr && (resp.indexOf("OK") >= 0 || resp.indexOf("ERROR") >= 0)) {
        sawOkOrErr = true;
        if (extraWait == nullptr) {
          unsigned long drainUntil = millis() + 40;
          while (millis() < drainUntil) {
            serviceFast();
            while (Modem.available()) {
              char d = (char)Modem.read();
              resp += d;
              feedUrcByte(d);
            }
          }
          inAtCommand = false;
          return resp;
        }
      }
      if (extraWait && resp.indexOf(extraWait) >= 0) {
        unsigned long drainUntil = millis() + 80;
        while (millis() < drainUntil) {
          serviceFast();
          while (Modem.available()) {
            char d = (char)Modem.read();
            resp += d;
            feedUrcByte(d);
          }
        }
        inAtCommand = false;
        return resp;
      }
    }
    if (sawOkOrErr && extraWait == nullptr) break;
    if (sawOkOrErr && extraWait && resp.indexOf("ERROR") >= 0 &&
        millis() - start > 1500) {
      break;
    }
  }
  inAtCommand = false;
  return resp;
}

bool sendPrompt(const char* command, const char* data, unsigned long timeoutMs = 4000) {
  inAtCommand = true;
  while (Modem.available()) feedUrcByte((char)Modem.read());

  Modem.print(command);
  Modem.print("\r\n");

  String resp;
  unsigned long start = millis();
  bool sent = false;
  while (millis() - start < timeoutMs) {
    serviceFast();
    while (Modem.available()) {
      char c = (char)Modem.read();
      resp += c;
      feedUrcByte(c);
      if (!sent && resp.indexOf('>') >= 0) {
        Modem.write(reinterpret_cast<const uint8_t*>(data), strlen(data));
        sent = true;
      }
      if (sent && (resp.indexOf("OK") >= 0 || resp.indexOf("ERROR") >= 0)) {
        inAtCommand = false;
        return resp.indexOf("OK") >= 0;
      }
    }
  }
  inAtCommand = false;
  return false;
}

bool okResp(const String& r) {
  return r.indexOf("OK") >= 0 && r.indexOf("ERROR") < 0;
}

bool alreadyOpen(const String& r) {
  return r.indexOf("already") >= 0 || r.indexOf("ALREADY") >= 0;
}

bool modemAnswers() {
  return okResp(sendAT("AT", 800));
}

bool openModemAt(long baud) {
  Modem.begin(baud, SERIAL_8N1, MODEM_RX_PIN, MODEM_TX_PIN);
  delay(250);
  return modemAnswers();
}

bool waitForAt(unsigned long giveUpMs) {
  unsigned long start = millis();
  while (millis() - start < giveUpMs) {
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
  }
}

void parseCops(const String& resp) {
  int tag = resp.indexOf("+COPS:");
  if (tag < 0) return;
  String name = extractQuoted(resp, tag);
  name.trim();
  setNetworkFromCops(name);
}

void parseImsi(const String& resp) {
  String digits;
  for (unsigned i = 0; i < resp.length(); i++) {
    if (resp[i] >= '0' && resp[i] <= '9') digits += resp[i];
    else if (digits.length() >= 14) break;
    else if (digits.length() > 0 && digits.length() < 5) digits = "";
  }
  if (digits.length() >= 10) imsi = digits;
}

void parseIpaddr(const String& resp) {
  int tag = resp.indexOf("+IPADDR:");
  if (tag < 0) tag = resp.indexOf("+CGPADDR:");
  if (tag < 0) return;
  String quoted = extractQuoted(resp, tag);
  if (quoted.length() >= 7 && quoted.indexOf('.') > 0) {
    modemIp = quoted;
    return;
  }
  int colon = resp.indexOf(':', tag);
  if (colon < 0) return;
  String ip = resp.substring(colon + 1);
  int comma = ip.indexOf(',');
  if (comma >= 0) ip = ip.substring(comma + 1);
  ip.replace("\r", "");
  ip.replace("\n", "");
  ip.replace("\"", "");
  ip.replace("OK", "");
  ip.trim();
  int cut = ip.indexOf(' ');
  if (cut > 0) ip = ip.substring(0, cut);
  if (ip.length() >= 7 && ip.indexOf('.') > 0) modemIp = ip;
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

bool isRegistered() {
  if (parseRegistered(sendAT("AT+CREG?"), "+CREG:")) return true;
  if (parseRegistered(sendAT("AT+CGREG?"), "+CGREG:")) return true;
  if (parseRegistered(sendAT("AT+CEREG?"), "+CEREG:")) return true;
  return false;
}

void pollCsq() {
  parseCsq(sendAT("AT+CSQ"));
}

void pollModemIdentity() {
  parseCpin(sendAT("AT+CPIN?"));
  parseCops(sendAT("AT+COPS?"));
  if (networkName == "WiFi" || networkName.length() == 0) networkName = "LTE";
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

void handleCommandJson(const String& cmd) {
  Serial.print("CMD ");
  Serial.println(cmd);

  if (jsonHasMode(cmd, "manual")) {
    servoMode = MODE_MANUAL;
    hunting = false;
    weakStreak = 0;
    int angle;
    if (jsonGetInt(cmd, "servo_angle", angle)) applyServoAngle(angle);
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
void wifiMqttCallback(char* topic, byte* payload, unsigned int length) {
  String cmd;
  for (unsigned int i = 0; i < length; i++) cmd += (char)payload[i];
  handleCommandJson(cmd);
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
  json += escapeJson(networkName);
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

bool cellularPublish(const String& json) {
  char topicCmd[32];
  snprintf(topicCmd, sizeof(topicCmd), "AT+CMQTTTOPIC=0,%u", (unsigned)strlen(MQTT_STATUS_TOPIC));
  if (!sendPrompt(topicCmd, MQTT_STATUS_TOPIC)) return false;

  char payCmd[32];
  snprintf(payCmd, sizeof(payCmd), "AT+CMQTTPAYLOAD=0,%u", (unsigned)json.length());
  if (!sendPrompt(payCmd, json.c_str())) return false;

  String r = sendAT("AT+CMQTTPUB=0,0,20", 8000, "+CMQTTPUB:");
  int err = parseSecondField(r, "+CMQTTPUB:");
  return err == 0 || (err < 0 && okResp(r));
}

void publishStatus() {
  if (!mqttUp) return;
  internet = true;
  String json = buildStatusJson();

  bool ok = cellularPublish(json);
#if WIFI_FALLBACK
  if (!ok && wifiMqtt.connected()) {
    ok = wifiMqtt.publish(MQTT_STATUS_TOPIC, json.c_str(), true);
  }
#endif
  if (ok) Serial.println(json);
  else {
    Serial.println("MQTT publish failed");
    onMqttLost();
  }
}

void cleanupMqttSession() {
  sendAT("AT+CMQTTDISC=0,60", 4000, "+CMQTTDISC:");
  sendAT("AT+CMQTTREL=0", 2000);
  sendAT("AT+CMQTTSTOP", 4000, "+CMQTTSTOP:");
}

bool tryNextApnOrRetry() {
  apnIndex++;
  if (apnIndex < apnCount) {
    Serial.printf("APN failed, trying fallback %s\n", apnList[apnIndex]);
    enterState(NET_SET_APN);
    return true;
  }
  enterState(NET_RETRY_WAIT);
  return false;
}

void stepNetwork() {
  switch (netState) {
    case NET_WAIT_SIM: {
      if (lastRegPollMs != 0 && millis() - lastRegPollMs < 1000) break;
      lastRegPollMs = millis();
      parseCpin(sendAT("AT+CPIN?"));
      if (simStatus == "READY") {
        Serial.println("SIM READY");
        copsStarted = false;
        enterState(NET_WAIT_REG);
      } else if (millis() - netStateSince > 20000) {
        Serial.println("SIM not READY yet — retry");
        enterState(NET_RETRY_WAIT);
      }
      break;
    }

    case NET_WAIT_REG: {
      if (!copsStarted) {
        sendAT("AT+COPS=0", 8000);
        copsStarted = true;
        lastRegPollMs = 0;
      }
      if (lastRegPollMs != 0 && millis() - lastRegPollMs < 2000) break;
      lastRegPollMs = millis();
      parseCops(sendAT("AT+COPS?"));
      if (isRegistered()) {
        Serial.printf("Registered on %s\n", networkName.c_str());
        enterState(NET_SET_APN);
      } else if (millis() - netStateSince > REG_TIMEOUT_MS) {
        Serial.println("Register timeout 60s — keep hunting, retry");
        enterState(NET_RETRY_WAIT);
      }
      break;
    }

    case NET_SET_APN: {
      if (imsi.length() < 10) parseImsi(sendAT("AT+CIMI", 2000));
      if (apnCount == 0) buildApnList();
      if (apnIndex >= apnCount) apnIndex = 0;
      const char* apn = apnList[apnIndex];
      String cmd = String("AT+CGDCONT=1,\"IP\",\"") + apn + "\"";
      Serial.printf("APN %s (IMSI %s)\n", apn, imsi.length() ? imsi.c_str() : "?");
      sendAT(cmd.c_str(), 4000);
      enterState(NET_ATTACH);
      break;
    }

    case NET_ATTACH: {
      String r = sendAT("AT+CGATT=1", 20000);
      if (okResp(r) || r.indexOf("+CGATT: 1") >= 0) {
        enterState(NET_OPEN);
      } else {
        Serial.println("CGATT failed");
        tryNextApnOrRetry();
      }
      break;
    }

    case NET_OPEN: {
      String st = sendAT("AT+NETOPEN?", 2000);
      if (st.indexOf("+NETOPEN: 1") >= 0) {
        parseIpaddr(sendAT("AT+IPADDR", 3000));
        enterState(NET_MQTT_START);
        break;
      }
      String r = sendAT("AT+NETOPEN", 45000, "+NETOPEN:");
      bool opened = parseAfterTag(r, "+NETOPEN:") == 0 || alreadyOpen(r);
      if (!opened && r.indexOf("ERROR") >= 0 && alreadyOpen(r)) opened = true;
      if (!opened) {
        Serial.println("NETOPEN failed");
        tryNextApnOrRetry();
        break;
      }
      parseIpaddr(sendAT("AT+IPADDR", 3000));
      if (modemIp == "-") parseIpaddr(sendAT("AT+CGPADDR=1", 3000));
      Serial.printf("PDP IP %s\n", modemIp.c_str());
      enterState(NET_MQTT_START);
      break;
    }

    case NET_MQTT_START: {
      String r = sendAT("AT+CMQTTSTART", 20000, "+CMQTTSTART:");
      int err = parseAfterTag(r, "+CMQTTSTART:");
      if (err == 0 || alreadyOpen(r) || r.indexOf("started") >= 0 ||
          (err < 0 && okResp(r))) {
        enterState(NET_MQTT_ACCQ);
      } else if (err == 1 || r.indexOf("ERROR") >= 0) {
        // already running on some firmware
        enterState(NET_MQTT_ACCQ);
      } else {
        Serial.println("CMQTTSTART failed");
        enterState(NET_RETRY_WAIT);
      }
      break;
    }

    case NET_MQTT_ACCQ: {
      String r = sendAT("AT+CMQTTACCQ=0,\"signalbooster-hub1\"", 4000);
      if (!okResp(r)) {
        r = sendAT("AT+CMQTTACCQ=0,\"signalbooster-hub1\",0", 4000);
      }
      if (okResp(r) || alreadyOpen(r) || r.indexOf("ERROR") >= 0) {
        // ERROR often means the client index is already acquired — continue.
        enterState(NET_MQTT_CONNECT);
      } else {
        enterState(NET_RETRY_WAIT);
      }
      break;
    }

    case NET_MQTT_CONNECT: {
      String cmd = String("AT+CMQTTCONNECT=0,\"") + MQTT_HOST_URL + "\",60,1";
      Serial.println("MQTT CONNECT tcp://broker.emqx.io:1883 (plain, no TLS)");
      String r = sendAT(cmd.c_str(), 60000, "+CMQTTCONNECT:");
      int err = parseSecondField(r, "+CMQTTCONNECT:");
      if (err == 0) {
        enterState(NET_MQTT_SUB);
      } else {
        Serial.printf("CMQTTCONNECT failed err=%d\n", err);
        cleanupMqttSession();
        enterState(NET_RETRY_WAIT);
      }
      break;
    }

    case NET_MQTT_SUB: {
      char subCmd[32];
      snprintf(subCmd, sizeof(subCmd), "AT+CMQTTSUB=0,%u,1",
               (unsigned)strlen(MQTT_COMMAND_TOPIC));
      if (!sendPrompt(subCmd, MQTT_COMMAND_TOPIC, 8000)) {
        Serial.println("CMQTTSUB prompt failed");
        cleanupMqttSession();
        enterState(NET_RETRY_WAIT);
        break;
      }
      mqttUp = true;
      internet = true;
      Serial.println("Cellular MQTT up, subscribed to command topic");
      enterState(NET_MQTT_UP);
      lastStatusMs = 0;
      break;
    }

    case NET_MQTT_UP:
      internet = true;
      break;

    case NET_RETRY_WAIT:
      mqttUp = false;
      internet = false;
      if (millis() - netStateSince >= RETRY_WAIT_MS) {
        cleanupMqttSession();
        copsStarted = false;
        apnCount = 0;
        apnIndex = 0;
        enterState(NET_WAIT_SIM);
      }
      break;
  }
}

void setup() {
  Serial.begin(115200);
  delay(200);
  Serial.println();
  Serial.println("Lantapan Hub  ESP32 + A7670C  cellular MQTT");
  Serial.println("GPIO16 RX2 <- modem TX | GPIO17 TX2 -> modem RX | GPIO27 PWRKEY | GPIO13 servo");
  Serial.println("Backhaul: A7670C TCP mqtt broker.emqx.io:1883  WIFI_FALLBACK="
#if WIFI_FALLBACK
                 "1"
#else
                 "0"
#endif
  );

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
  sendAT("ATE0", 800);
  sendAT("AT+CMEE=2", 800);

#if WIFI_FALLBACK
  WiFi.mode(WIFI_STA);
  wifiMqtt.setServer("broker.emqx.io", 1883);
  wifiMqtt.setCallback(wifiMqttCallback);
  wifiMqtt.setBufferSize(512);
#endif

  enterState(NET_WAIT_SIM);
}

void loop() {
  while (Modem.available()) feedUrcByte((char)Modem.read());
  serviceFast();

  unsigned long pollEvery = hunting ? MODEM_POLL_HUNT_MS : MODEM_POLL_HOLD_MS;
  if (!inAtCommand && (lastModemPollMs == 0 || millis() - lastModemPollMs >= pollEvery)) {
    lastModemPollMs = millis();
    pollCsq();
    updateHuntState();
  }

  if (!inAtCommand && (lastSlowPollMs == 0 || millis() - lastSlowPollMs >= MODEM_SLOW_POLL_MS)) {
    lastSlowPollMs = millis();
    pollModemIdentity();
  }

  if (!inAtCommand) stepNetwork();

  unsigned long pubEvery = hunting ? PUBLISH_HUNT_MS : PUBLISH_HOLD_MS;
  if (mqttUp && (lastStatusMs == 0 || millis() - lastStatusMs >= pubEvery)) {
    lastStatusMs = millis();
    publishStatus();
  }

#if WIFI_FALLBACK
  if (!mqttUp && WiFi.status() != WL_CONNECTED) {
    static unsigned long lastWifi;
    if (lastWifi == 0 || millis() - lastWifi > 8000) {
      lastWifi = millis();
      WiFi.begin(WIFI_SSID, WIFI_PASS);
    }
  } else if (!mqttUp && WiFi.status() == WL_CONNECTED && !wifiMqtt.connected()) {
    static unsigned long lastWm;
    if (lastWm == 0 || millis() - lastWm > 8000) {
      lastWm = millis();
      if (wifiMqtt.connect(MQTT_CLIENT_ID)) {
        wifiMqtt.subscribe(MQTT_COMMAND_TOPIC);
        mqttUp = true;
        internet = true;
        Serial.println("WIFI_FALLBACK MQTT up");
      }
    }
  }
#endif
}
