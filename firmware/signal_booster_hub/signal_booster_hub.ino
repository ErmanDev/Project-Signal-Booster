/*
 * Lantapan Hub — ESP32 + SIMCom A7670C signal-seeking antenna
 *
 * Publishes real AT+CSQ telemetry to the existing index.html dashboard.
 * No mock / random RSSI.
 *
 * Libraries: ESP32Servo, PubSubClient (plus ESP32 Arduino core: WiFi, Preferences)
 *
 * ========== PIN MAP (every wire) ==========
 * A7670C TX          → ESP32 GPIO 16 (UART2 RX)
 * A7670C RX          → ESP32 GPIO 17 (UART2 TX)
 * A7670C PWRKEY      → ESP32 GPIO 27  (pulse LOW ~1.2s if AT is silent)
 * Servo signal       → ESP32 GPIO 13  (yellow/orange)
 * Servo VCC (red)    → boost 5.0 V
 * Servo GND (brown)  → common GND
 * Power switch       → battery +  (NOT a GPIO)
 * Boost EN (if any)  → switched battery + so the boost dies with the switch
 * Common GND         → battery −, boost GND, ESP32 GND, modem GND, servo GND
 *
 * UART: 115200 8N1. Common GND is required or AT will never answer.
 *
 * ========== POWER PATH ==========
 * Li-ion+ → POWER SWITCH → split:
 *   (A) A7670C VBAT 3.4–4.2 V if it is a bare module / VBAT pad
 *   (B) boost IN+
 * Boost OUT 5.0 V → ESP32 VIN and servo VCC only.
 * If the A7670C is a 5 V breakout with an onboard 3.8 V regulator,
 * feed that board from boost 5 V AFTER the switch, not from raw 4.2 V.
 * Never put 5 V into a bare VBAT pad.
 * Put 470 µF+ across the modem power pins.
 */

#include <WiFi.h>
#include <PubSubClient.h>
#include <ESP32Servo.h>
#include <Preferences.h>

// ---------- WiFi (MQTT backhaul) — fill these in before flashing ----------
const char* WIFI_SSID = "YOUR_WIFI_SSID";
const char* WIFI_PASS = "YOUR_WIFI_PASS";

// ---------- MQTT (must match index.html) ----------
const char* MQTT_BROKER = "broker.emqx.io";
const uint16_t MQTT_PORT = 1883;
const char* MQTT_STATUS_TOPIC = "signalbooster/hub1/status";
const char* MQTT_COMMAND_TOPIC = "signalbooster/hub1/command";

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
static const int RSSI_FAIR = 12;      // hunt below this; hold at this or better
static const int RSSI_UNKNOWN = 99;   // AT+CSQ "not known or not detectable"

static const int SERVO_MIN = 0;
static const int SERVO_MAX = 180;
static const int HUNT_STEP_DEG = 2;
static const unsigned long HUNT_STEP_MS = 400;   // slow sweep
static const unsigned long STATUS_INTERVAL_MS = 2000;
static const unsigned long MODEM_POLL_HOLD_MS = 2000;
static const unsigned long MODEM_POLL_HUNT_MS = 700;
static const unsigned long MODEM_SLOW_POLL_MS = 10000;
static const unsigned long WIFI_RETRY_MS = 8000;
static const unsigned long MQTT_RETRY_MS = 5000;
static const unsigned long AT_TIMEOUT_MS = 1500;
static const unsigned long PWRKEY_PULSE_MS = 1200;
static const int WEAK_STREAK_TO_HUNT = 2;

HardwareSerial Modem(2);
WiFiClient wifiClient;
PubSubClient mqtt(wifiClient);
Servo antennaServo;
Preferences prefs;

enum ServoMode { MODE_AUTO, MODE_MANUAL };

int servoAngle = 90;
int bestAngle = 90;
int bestSignal = 0;
int rssi = -1;                 // last valid 0–31, or -1 if never read
int lastCsqRaw = RSSI_UNKNOWN;
bool internet = false;
String networkName = "LTE";
String simStatus = "UNKNOWN";
String modemIp = "-";
int satellites = 0;            // 0 unless GNSS actually returns a count
ServoMode servoMode = MODE_AUTO;
int huntDir = 1;
int weakStreak = 0;
bool hunting = false;  // hold NVS heading until the first real CSQ says Weak

unsigned long lastStatusMs = 0;
unsigned long lastHuntStepMs = 0;
unsigned long lastModemPollMs = 0;
unsigned long lastSlowPollMs = 0;
unsigned long lastWifiAttemptMs = 0;
unsigned long lastMqttAttemptMs = 0;

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

void pumpMqtt() {
  if (mqtt.connected()) mqtt.loop();
  yield();
}

String sendAT(const char* command, unsigned long timeoutMs = AT_TIMEOUT_MS) {
  while (Modem.available()) Modem.read();
  Modem.print(command);
  Modem.print("\r\n");

  String resp;
  resp.reserve(256);
  unsigned long start = millis();
  bool done = false;
  while (millis() - start < timeoutMs) {
    pumpMqtt();
    while (Modem.available()) {
      char c = (char)Modem.read();
      resp += c;
      if (resp.indexOf("OK") >= 0 || resp.indexOf("ERROR") >= 0) {
        // drain a few leftover bytes
        unsigned long drainUntil = millis() + 40;
        while (millis() < drainUntil) {
          while (Modem.available()) resp += (char)Modem.read();
        }
        done = true;
        break;
      }
    }
    if (done) break;
  }
  return resp;
}

bool modemAnswers() {
  String r = sendAT("AT", 800);
  return r.indexOf("OK") >= 0;
}

void pulsePwrkey() {
  Serial.println("PWRKEY: pulse LOW 1.2s (modem silent)");
  pinMode(PWRKEY_PIN, OUTPUT);
  digitalWrite(PWRKEY_PIN, HIGH);
  delay(50);
  digitalWrite(PWRKEY_PIN, LOW);
  delay(PWRKEY_PULSE_MS);
  digitalWrite(PWRKEY_PIN, HIGH);
  delay(3000);
}

bool ensureModemAwake() {
  pinMode(PWRKEY_PIN, OUTPUT);
  digitalWrite(PWRKEY_PIN, HIGH);
  Modem.begin(MODEM_BAUD, SERIAL_8N1, MODEM_RX_PIN, MODEM_TX_PIN);
  delay(300);

  if (modemAnswers()) {
    Serial.println("Modem already answering AT");
    sendAT("ATE0", 800);
    return true;
  }

  pulsePwrkey();
  if (modemAnswers()) {
    sendAT("ATE0", 800);
    return true;
  }

  Serial.println("Modem still silent after PWRKEY. Check UART, GND, VBAT.");
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
  // numeric MCC/MNC is not a friendly operator label
  bool digits = true;
  for (unsigned i = 0; i < name.length(); i++) {
    if (name[i] < '0' || name[i] > '9') {
      digits = false;
      break;
    }
  }
  if (digits) {
    networkName = "LTE";
  } else {
    networkName = name;
  }
}

bool parseRegistered(const String& resp, const char* tag) {
  int t = resp.indexOf(tag);
  if (t < 0) return false;
  // +CREG: <n>,<stat>   or   +CEREG: <n>,<stat>,...
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
  // unquoted: +CGPADDR: 1,10.1.2.3
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

void parseGnssSatellites(const String& resp) {
  // +CGNSSINFO: <mode>,<GPS-SVs>,<GLONASS-SVs>,<BEIDOU-SVs>,...
  // Only accept a real parse. Never invent a count.
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

  bool reg = parseRegistered(sendAT("AT+CEREG?"), "+CEREG:");
  if (!reg) {
    reg = parseRegistered(sendAT("AT+CREG?"), "+CREG:");
  }
  // Prefer real registration. If the unsolicited format did not parse, a
  // READY SIM plus a valid CSQ still means the radio is on a cell.
  internet = reg || (simStatus == "READY" && rssi >= 0 && rssi <= 31);

  parseCgpaddr(sendAT("AT+CGPADDR=1"));

  // Only overwrite satellites when GNSS actually returns a count.
  parseGnssSatellites(sendAT("AT+CGNSSINFO", 800));

  if (networkName.length() == 0) networkName = "LTE";
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

void handleCommand(char* topic, byte* payload, unsigned int length) {
  String cmd;
  cmd.reserve(length + 1);
  for (unsigned int i = 0; i < length; i++) cmd += (char)payload[i];
  Serial.print("CMD ");
  Serial.print(topic);
  Serial.print(" ");
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

void connectWifi() {
  if (WiFi.status() == WL_CONNECTED) return;
  if (lastWifiAttemptMs != 0 && millis() - lastWifiAttemptMs < WIFI_RETRY_MS) return;
  lastWifiAttemptMs = millis();

  Serial.print("WiFi connecting to ");
  Serial.println(WIFI_SSID);
  WiFi.mode(WIFI_STA);
  WiFi.setSleep(false);
  WiFi.disconnect(false, false);
  WiFi.begin(WIFI_SSID, WIFI_PASS);
}

void connectMqtt() {
  if (WiFi.status() != WL_CONNECTED || mqtt.connected()) return;
  if (lastMqttAttemptMs != 0 && millis() - lastMqttAttemptMs < MQTT_RETRY_MS) return;
  lastMqttAttemptMs = millis();

  uint8_t mac[6];
  WiFi.macAddress(mac);
  char clientId[32];
  snprintf(clientId, sizeof(clientId), "hub1-%02x%02x%02x", mac[3], mac[4], mac[5]);

  Serial.print("MQTT connecting as ");
  Serial.println(clientId);
  if (mqtt.connect(clientId)) {
    mqtt.subscribe(MQTT_COMMAND_TOPIC);
    Serial.println("MQTT connected, subscribed to command topic");
  } else {
    Serial.print("MQTT failed, state=");
    Serial.println(mqtt.state());
  }
}

void publishStatus() {
  if (!mqtt.connected()) return;

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

  if (mqtt.publish(MQTT_STATUS_TOPIC, json.c_str(), true)) {
    Serial.println(json);
  } else {
    Serial.println("MQTT publish failed");
  }
}

void setup() {
  Serial.begin(115200);
  delay(200);
  Serial.println();
  Serial.println("Lantapan Hub  ESP32 + A7670C");
  Serial.println("GPIO16 RX2 <- modem TX | GPIO17 TX2 -> modem RX | GPIO27 PWRKEY | GPIO13 servo");

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

  mqtt.setServer(MQTT_BROKER, MQTT_PORT);
  mqtt.setCallback(handleCommand);
  mqtt.setBufferSize(512);
  mqtt.setKeepAlive(30);

  connectWifi();
  lastModemPollMs = 0;
}

void loop() {
  connectWifi();
  connectMqtt();
  pumpMqtt();

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

  if (lastStatusMs == 0 || millis() - lastStatusMs >= STATUS_INTERVAL_MS) {
    lastStatusMs = millis();
    publishStatus();
  }
}
