#include <SoftwareSerial.h>
#include <Servo.h>

#define MODEM_RX D5
#define MODEM_TX D6
#define SERVO_PIN D1

SoftwareSerial modem(MODEM_RX, MODEM_TX);
Servo signalServo;

int angles[] = {0, 30, 60, 90, 120, 150, 180};
int bestSignal = -1;
int bestAngle = 0;

// ----- Cellular data + MQTT settings (edit for your SIM/provider) -----
const char* APN = "internet.globe.com.ph";
const char* APN_USER = "";
const char* APN_PASS = "";
const char* APN_FALLBACK = "internet";
const char* MQTT_BROKER = "broker.emqx.io";
const int MQTT_PORT = 1883;
const char* MQTT_TOPIC = "signalbooster/hub1/status";

bool mqttReady = false;
long modemBaud = 0;

String sendAT(String command, unsigned long timeout) {
  while (modem.available()) modem.read();

  modem.println(command);

  String response = "";
  unsigned long start = millis();

  while (millis() - start < timeout) {
    while (modem.available()) {
      response += (char)modem.read();
    }
  }

  return response;
}

String escapeJson(String input) {
  input.replace("\\", "\\\\");
  input.replace("\"", "\\\"");
  return input;
}

String signalQualityLabel(int signal) {
  if (signal >= 25) return "Strong";
  if (signal >= 18) return "Good";
  if (signal >= 12) return "Fair";
  if (signal >= 0) return "Weak";
  return "No Signal";
}

bool responseHasOk(String response) {
  return response.indexOf("OK") != -1;
}

bool initModemAutoBaud() {
  const long baudCandidates[] = {115200, 9600, 57600, 38400, 19200};
  const int candidateCount = sizeof(baudCandidates) / sizeof(baudCandidates[0]);

  Serial.println("MODEM: AUTO BAUD DETECT");

  for (int i = 0; i < candidateCount; i++) {
    long baud = baudCandidates[i];
    modem.begin(baud);
    delay(500);

    String resp = sendAT("AT", 1500);
    resp.trim();

    Serial.print("MODEM BAUD TRY ");
    Serial.print(baud);
    Serial.print(": ");
    Serial.println(resp.length() ? "RESPONSE" : "NO RESPONSE");

    if (responseHasOk(resp)) {
      modemBaud = baud;
      Serial.print("MODEM BAUD LOCKED: ");
      Serial.println(modemBaud);
      sendAT("ATE0", 1500);  // Disable command echo for cleaner parsing.
      return true;
    }
  }

  return false;
}

void printAtResult(const char* step, String response) {
  Serial.print("[AT] ");
  Serial.print(step);
  Serial.print(" => ");
  response.replace("\r", " ");
  response.replace("\n", " ");
  response.trim();
  Serial.println(response.length() ? response : "(empty)");
}

bool setupPdpWithApn(const char* apnValue) {
  String resp;
  String cmd = String("AT+CGDCONT=1,\"IP\",\"") + apnValue + "\"";
  resp = sendAT(cmd, 5000);
  printAtResult("CGDCONT", resp);
  if (!responseHasOk(resp)) return false;

  if (strlen(APN_USER) > 0 || strlen(APN_PASS) > 0) {
    cmd = String("AT+CGAUTH=1,1,\"") + APN_PASS + "\",\"" + APN_USER + "\"";
    resp = sendAT(cmd, 5000);
    printAtResult("CGAUTH", resp);
    if (!responseHasOk(resp)) return false;
  }

  resp = sendAT("AT+CGACT=1,1", 12000);
  printAtResult("CGACT", resp);
  if (!responseHasOk(resp)) return false;

  return true;
}

bool initCellularData() {
  String resp;
  resp = sendAT("AT", 3000);
  printAtResult("AT", resp);
  if (!responseHasOk(resp)) return false;

  resp = sendAT("AT+CPIN?", 4000);
  printAtResult("CPIN?", resp);
  if (resp.indexOf("READY") == -1) return false;

  resp = sendAT("AT+CSQ", 4000);
  printAtResult("CSQ", resp);

  resp = sendAT("AT+CREG?", 4000);
  printAtResult("CREG?", resp);
  resp = sendAT("AT+CGREG?", 4000);
  printAtResult("CGREG?", resp);

  resp = sendAT("AT+CGATT=1", 12000);
  printAtResult("CGATT", resp);
  if (!responseHasOk(resp)) return false;

  Serial.print("APN TRY: ");
  Serial.println(APN);
  if (setupPdpWithApn(APN)) return true;

  if (String(APN_FALLBACK) != String(APN)) {
    Serial.print("APN TRY: ");
    Serial.println(APN_FALLBACK);
    if (setupPdpWithApn(APN_FALLBACK)) return true;
  }
  return false;
}

bool initMQTT() {
  sendAT("AT+CMQTTSTOP", 2000);
  String resp = sendAT("AT+CMQTTSTART", 8000);
  if (!responseHasOk(resp) && resp.indexOf("+CMQTTSTART: 0") == -1) return false;

  resp = sendAT("AT+CMQTTACCQ=0,\"esp12e-hub\",0", 4000);
  if (!responseHasOk(resp)) return false;

  String connectCmd = String("AT+CMQTTCONNECT=0,\"tcp://") + MQTT_BROKER + ":" + MQTT_PORT + "\",60,1";
  resp = sendAT(connectCmd, 15000);
  if (!responseHasOk(resp)) return false;
  if (resp.indexOf("+CMQTTCONNECT: 0,0") == -1 && resp.indexOf("+CMQTTCONNECT: 0, 0") == -1) return false;

  return true;
}

bool mqttPublish(String payload) {
  String topic = MQTT_TOPIC;
  String resp = sendAT(String("AT+CMQTTTOPIC=0,") + topic.length(), 3000);
  if (resp.indexOf(">") == -1) return false;
  modem.print(topic);
  delay(300);

  resp = sendAT(String("AT+CMQTTPAYLOAD=0,") + payload.length(), 3000);
  if (resp.indexOf(">") == -1) return false;
  modem.print(payload);
  delay(300);

  resp = sendAT("AT+CMQTTPUB=0,1,60", 6000);
  return responseHasOk(resp);
}

void publishStatus(int signal, String fix, String lat, String lng, String sats) {
  if (!mqttReady) return;

  String payload = "{";
  payload += "\"internet\":" + String(signal >= 0 ? "true" : "false") + ",";
  payload += "\"network\":\"SIM Cellular\",";
  payload += "\"signal\":" + String(signal >= 0 ? signal : 0) + ",";
  payload += "\"signal_quality\":\"" + signalQualityLabel(signal) + "\",";
  payload += "\"servo_angle\":" + String(bestAngle) + ",";
  payload += "\"best_angle\":" + String(bestAngle) + ",";
  payload += "\"best_signal\":" + String(bestSignal >= 0 ? bestSignal : 0) + ",";
  payload += "\"sim\":\"READY\",";
  payload += "\"ip\":\"-\",";
  payload += "\"gps_fix\":\"" + escapeJson(fix) + "\",";
  payload += "\"latitude\":\"" + escapeJson(lat) + "\",";
  payload += "\"longitude\":\"" + escapeJson(lng) + "\",";
  payload += "\"satellites\":\"" + escapeJson(sats) + "\"";
  payload += "}";

  if (!mqttPublish(payload)) {
    Serial.println("MQTT PUBLISH: FAILED");
    mqttReady = initMQTT();
    Serial.println(mqttReady ? "MQTT: RECONNECTED" : "MQTT: RECONNECT FAILED");
  } else {
    Serial.println("MQTT PUBLISH: OK");
    Serial.println(payload);
  }
}

int getSignal() {
  String response = sendAT("AT+CSQ", 2000);
  int index = response.indexOf("+CSQ:");

  if (index == -1) return -1;

  int comma = response.indexOf(",", index);

  if (comma == -1) return -1;

  String value = response.substring(index + 5, comma);
  value.trim();

  int signal = value.toInt();

  if (signal == 99) return -1;

  return signal;
}

void getGPS(String &fix, String &latitude, String &longitude, String &satellites) {
  fix = "";
  latitude = "";
  longitude = "";
  satellites = "";

  String response = sendAT("AT+CGNSINF", 3000);
  int index = response.indexOf("+CGNSINF:");

  if (index == -1) {
    Serial.println("GPS: NO DATA");
    return;
  }

  String data = response.substring(index + 9);
  data.trim();

  int commas[20];
  int count = 0;

  for (int i = 0; i < data.length(); i++) {
    if (data[i] == ',' && count < 20) {
      commas[count++] = i;
    }
  }

  if (count < 15) {
    Serial.println("GPS: INVALID");
    return;
  }

  fix = data.substring(commas[0] + 1, commas[1]);
  latitude = data.substring(commas[1] + 1, commas[2]);
  longitude = data.substring(commas[2] + 1, commas[3]);
  satellites = data.substring(commas[14] + 1, commas[15]);

  Serial.print("GPS FIX: ");
  Serial.println(fix);

  Serial.print("LATITUDE: ");
  Serial.println(latitude);

  Serial.print("LONGITUDE: ");
  Serial.println(longitude);

  Serial.print("SATELLITES: ");
  Serial.println(satellites);
}

void scanSignal() {
  bestSignal = -1;
  bestAngle = 0;

  Serial.println();
  Serial.println("===== SIGNAL SCAN =====");

  for (int i = 0; i < 7; i++) {
    int angle = angles[i];

    signalServo.write(angle);
    delay(4000);

    int signal = getSignal();

    Serial.println();
    Serial.print("ANGLE: ");
    Serial.print(angle);
    Serial.println("°");

    Serial.print("SIGNAL: ");
    Serial.print(signal);
    Serial.println("/31");

    String fix, lat, lng, sats;
    getGPS(fix, lat, lng, sats);

    if (signal > bestSignal && signal <= 31) {
      bestSignal = signal;
      bestAngle = angle;
    }
  }

  Serial.println();
  Serial.println("===== BEST RESULT =====");

  Serial.print("BEST SIGNAL: ");
  Serial.print(bestSignal);
  Serial.println("/31");

  Serial.print("BEST ANGLE: ");
  Serial.print(bestAngle);
  Serial.println("°");

  signalServo.write(bestAngle);
}

void setup() {
  Serial.begin(115200);

  signalServo.attach(SERVO_PIN);
  signalServo.write(0);

  delay(12000);

  if (!initModemAutoBaud()) {
    Serial.println("MODEM: NOT RESPONDING ON COMMON BAUD RATES");
    return;
  }

  Serial.println();
  Serial.println("A7670C SIGNAL AND GPS SYSTEM");
  Serial.println("NETWORK: SIM Cellular");

  Serial.println(sendAT("AT", 2000));
  Serial.println(sendAT("AT+CPIN?", 2000));
  Serial.println(sendAT("AT+CSQ", 2000));

  sendAT("AT+CGNSSPWR=1", 3000);

  Serial.println("CELLULAR DATA: INITIALIZING");
  if (initCellularData()) {
    Serial.println("CELLULAR DATA: READY");
    Serial.println("MQTT: INITIALIZING");
    mqttReady = initMQTT();
    Serial.println(mqttReady ? "MQTT: CONNECTED" : "MQTT: FAILED");
  } else {
    Serial.println("CELLULAR DATA: FAILED");
  }

  delay(3000);

  scanSignal();
}

void loop() {
  delay(15000);

  Serial.println();
  Serial.println("===== CURRENT STATUS =====");

  int signal = getSignal();

  Serial.print("SERVO ANGLE: ");
  Serial.print(bestAngle);
  Serial.println("°");

  Serial.print("CELLULAR SIGNAL: ");
  Serial.print(signal);
  Serial.println("/31");

  String fix, lat, lng, sats;
  getGPS(fix, lat, lng, sats);
  publishStatus(signal, fix, lat, lng, sats);
}