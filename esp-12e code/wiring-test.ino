#include <SoftwareSerial.h>

// ESP-12E <-> A7670C wiring (crossed UART):
// ESP D6/TX -> A7670C RX
// ESP D5/RX -> A7670C TX
// GND -> GND (common ground required)
#define MODEM_RX D5
#define MODEM_TX D6
#define SERVO_PIN D1

SoftwareSerial modem(MODEM_RX, MODEM_TX);

long activeBaud = 0;
unsigned long lastPingMs = 0;

String sendAT(const String &cmd, unsigned long timeoutMs) {
  while (modem.available()) modem.read();
  modem.println(cmd);

  String resp = "";
  unsigned long start = millis();
  while (millis() - start < timeoutMs) {
    while (modem.available()) {
      resp += (char)modem.read();
    }
  }
  return resp;
}

bool detectBaud() {
  const long candidates[] = {115200, 9600, 57600, 38400, 19200};
  const int count = sizeof(candidates) / sizeof(candidates[0]);

  Serial.println("=== A7670C AUTO-BAUD TEST ===");
  for (int i = 0; i < count; i++) {
    modem.begin(candidates[i]);
    delay(500);

    String resp = sendAT("AT", 1500);
    resp.trim();

    Serial.print("TRY ");
    Serial.print(candidates[i]);
    Serial.print(" -> ");
    if (resp.length() == 0) {
      Serial.println("NO RESPONSE");
      continue;
    }

    Serial.print("RESP: ");
    Serial.println(resp);

    if (resp.indexOf("OK") != -1) {
      activeBaud = candidates[i];
      sendAT("ATE0", 1500);  // cleaner output
      Serial.print("LOCKED BAUD: ");
      Serial.println(activeBaud);
      return true;
    }
  }
  return false;
}

void setup() {
  Serial.begin(115200);
  delay(1500);

  Serial.println();
  Serial.println("ESP-12E <-> A7670C WIRING TEST");
  Serial.println("Wait 10s for modem boot...");
  delay(10000);

  if (!detectBaud()) {
    Serial.println("FAIL: no AT response.");
    Serial.println("Check: power, GND, RX/TX crossed, or wrong pins.");
    return;
  }

  Serial.println("PASS: modem responds to AT.");
  Serial.println("You can now type AT commands in Serial Monitor.");
  Serial.println("Use 'Both NL & CR' and 115200 for Serial Monitor.");
}

void loop() {
  // PC -> modem passthrough
  if (Serial.available()) {
    String cmd = Serial.readStringUntil('\n');
    cmd.trim();
    if (cmd.length()) {
      String resp = sendAT(cmd, 2500);
      Serial.print(">> ");
      Serial.println(cmd);
      Serial.print("<< ");
      if (resp.length()) {
        Serial.println(resp);
      } else {
        Serial.println("(empty)");
      }
    }
  }

  // Automatic keepalive ping every 5s
  if (activeBaud > 0 && millis() - lastPingMs > 5000) {
    lastPingMs = millis();
    String resp = sendAT("AT", 1200);
    Serial.print("[PING] ");
    Serial.println(resp.indexOf("OK") != -1 ? "OK" : "NO RESPONSE");
  }
}
