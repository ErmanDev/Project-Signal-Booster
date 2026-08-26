#include <ESP8266WiFi.h>
#include <ESP8266WebServer.h>
#include <Servo.h>

const char* ssid = "SYSTEM";
const char* password = "CHANGE_ME";

ESP8266WebServer server(80);
Servo servo;

#define SERVO_PIN D4

int satellites = 8;
int signalStrength = 80;
int servoAngle = 90;

unsigned long lastUpdate = 0;

void updateData() {
  if (millis() - lastUpdate < 3000) {
    return;
  }

  lastUpdate = millis();
  satellites = random(6, 13);
  signalStrength = random(65, 96);
  servoAngle = random(0, 181);
  servo.write(servoAngle);

  Serial.println();
  Serial.println("========== SYSTEM DATA ==========");
  Serial.print("Satellites: ");
  Serial.println(satellites);
  Serial.print("Signal Strength: ");
  Serial.print(signalStrength);
  Serial.println("%");
  Serial.println("Signal Status: GOOD SIGNAL");
  Serial.print("Servo Angle: ");
  Serial.print(servoAngle);
  Serial.println(" degrees");
  Serial.println("=================================");
}

void addApiHeaders() {
  server.sendHeader("Access-Control-Allow-Origin", "*");
  server.sendHeader("Access-Control-Allow-Methods", "GET, OPTIONS");
  server.sendHeader("Access-Control-Allow-Headers", "Content-Type");
  server.sendHeader("Access-Control-Allow-Private-Network", "true");
  server.sendHeader("Cache-Control", "no-store");
}

void handleData() {
  String json = "{";
  json += "\"satellites\":" + String(satellites);
  json += ",\"signal\":" + String(signalStrength);
  json += ",\"status\":\"GOOD SIGNAL\"";
  json += ",\"angle\":" + String(servoAngle);
  json += "}";

  addApiHeaders();
  server.send(200, "application/json", json);
}

void handleOptions() {
  addApiHeaders();
  server.send(204);
}

void handleNotFound() {
  addApiHeaders();
  server.send(404, "application/json", "{\"error\":\"Not found\"}");
}

void setup() {
  Serial.begin(115200);
  randomSeed(analogRead(A0));

  servo.attach(SERVO_PIN);
  servo.write(servoAngle);

  WiFi.mode(WIFI_AP);
  WiFi.softAP(ssid, password);

  Serial.println();
  Serial.println("=================================");
  Serial.println("       MONITORING SYSTEM");
  Serial.println("=================================");
  Serial.print("WiFi SSID: ");
  Serial.println(ssid);
  Serial.print("WiFi Password: ");
  Serial.println(password);
  Serial.print("API Address: http://");
  Serial.print(WiFi.softAPIP());
  Serial.println("/data");
  Serial.println("Servo initialized.");

  server.on("/data", HTTP_GET, handleData);
  server.on("/data", HTTP_OPTIONS, handleOptions);
  server.onNotFound(handleNotFound);
  server.begin();

  Serial.println("API server started.");
  Serial.println("=================================");
}

void loop() {
  server.handleClient();
  updateData();
}
