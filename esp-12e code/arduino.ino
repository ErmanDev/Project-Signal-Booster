#include <ESP8266WiFi.h>
#include <PubSubClient.h>
#include <Servo.h>

const char* WIFI_SSID = "YOUR_WIFI_SSID";
const char* WIFI_PASSWORD = "YOUR_WIFI_PASSWORD";

const char* MQTT_BROKER = "broker.emqx.io";
const int MQTT_PORT = 1883;
const char* MQTT_STATUS_TOPIC = "signalbooster/hub1/status";
const char* MQTT_COMMAND_TOPIC = "signalbooster/hub1/command";

WiFiClient wifiClient;
PubSubClient mqttClient(wifiClient);
Servo servo;

#define SERVO_PIN D4

int satellites = 8;
int signalStrength = 80;
int servoAngle = 90;
int bestSignal = 0;
int bestAngle = 90;
bool manualMode = false;

const unsigned long PUBLISH_INTERVAL_MS = 3000;
const unsigned long RECONNECT_INTERVAL_MS = 5000;
unsigned long lastPublishMs = 0;
unsigned long lastWifiAttemptMs = 0;
unsigned long lastMqttAttemptMs = 0;

String signalQuality() {
  if (signalStrength >= 80) return "Strong";
  if (signalStrength >= 60) return "Good";
  if (signalStrength >= 40) return "Fair";
  return "Weak";
}

void updateMockData() {
  satellites = random(6, 13);
  signalStrength = random(35, 101);

  if (signalStrength > bestSignal) {
    bestSignal = signalStrength;
    bestAngle = servoAngle;
  }
}

void applyServoAngle(int angle) {
  servoAngle = constrain(angle, 0, 180);
  servo.write(servoAngle);
  Serial.print("Servo angle: ");
  Serial.println(servoAngle);
}

void handleMqttCommand(char* topic, byte* payload, unsigned int length) {
  String command;
  for (unsigned int i = 0; i < length; i++) {
    command += (char)payload[i];
  }

  if (command.indexOf("\"mode\":\"manual\"") >= 0 ||
      command.indexOf("\"mode\": \"manual\"") >= 0) {
    manualMode = true;
  } else if (command.indexOf("\"mode\":\"auto\"") >= 0 ||
             command.indexOf("\"mode\": \"auto\"") >= 0) {
    manualMode = false;
  }

  int keyIndex = command.indexOf("servo_angle");
  if (keyIndex >= 0) {
    int colonIndex = command.indexOf(':', keyIndex);
    if (colonIndex >= 0) {
      applyServoAngle(command.substring(colonIndex + 1).toInt());
    }
  }
}

void connectWifi() {
  if (WiFi.status() == WL_CONNECTED) return;
  if (lastWifiAttemptMs != 0 &&
      millis() - lastWifiAttemptMs < RECONNECT_INTERVAL_MS) return;

  lastWifiAttemptMs = millis();
  Serial.print("Connecting to WiFi: ");
  Serial.println(WIFI_SSID);
  WiFi.disconnect();
  WiFi.begin(WIFI_SSID, WIFI_PASSWORD);
}

void connectMqtt() {
  if (WiFi.status() != WL_CONNECTED || mqttClient.connected()) return;
  if (lastMqttAttemptMs != 0 &&
      millis() - lastMqttAttemptMs < RECONNECT_INTERVAL_MS) return;

  lastMqttAttemptMs = millis();
  String clientId = "esp8266-signal-booster-" + String(ESP.getChipId(), HEX);

  if (mqttClient.connect(clientId.c_str())) {
    mqttClient.subscribe(MQTT_COMMAND_TOPIC);
    Serial.println("MQTT connected.");
  } else {
    Serial.print("MQTT connection failed, state: ");
    Serial.println(mqttClient.state());
  }
}

void publishTelemetry() {
  if (!mqttClient.connected()) return;

  updateMockData();

  String json = "{";
  json += "\"internet\":true";
  json += ",\"network\":\"" + WiFi.SSID() + "\"";
  json += ",\"signal\":" + String(signalStrength);
  json += ",\"signal_quality\":\"" + signalQuality() + "\"";
  json += ",\"servo_angle\":" + String(servoAngle);
  json += ",\"best_angle\":" + String(bestAngle);
  json += ",\"best_signal\":" + String(bestSignal);
  json += ",\"satellites\":" + String(satellites);
  json += ",\"sim\":\"N/A\"";
  json += ",\"ip\":\"" + WiFi.localIP().toString() + "\"";
  json += ",\"servo_mode\":\"";
  json += (manualMode ? "manual" : "auto");
  json += "\"}";

  if (mqttClient.publish(MQTT_STATUS_TOPIC, json.c_str(), true)) {
    Serial.println(json);
  } else {
    Serial.println("Telemetry publish failed.");
  }
}

void setup() {
  Serial.begin(115200);
  randomSeed(analogRead(A0));

  servo.attach(SERVO_PIN);
  applyServoAngle(servoAngle);

  WiFi.mode(WIFI_STA);
  WiFi.setAutoReconnect(true);
  WiFi.persistent(false);

  mqttClient.setServer(MQTT_BROKER, MQTT_PORT);
  mqttClient.setCallback(handleMqttCommand);
  mqttClient.setBufferSize(512);

  connectWifi();
}

void loop() {
  connectWifi();
  connectMqtt();
  mqttClient.loop();

  if (mqttClient.connected() &&
      (lastPublishMs == 0 ||
       millis() - lastPublishMs >= PUBLISH_INTERVAL_MS)) {
    lastPublishMs = millis();
    publishTelemetry();
  }

  delay(10);
}
