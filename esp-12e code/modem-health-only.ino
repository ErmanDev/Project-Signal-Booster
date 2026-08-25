#include <SoftwareSerial.h>

// Modem-only wiring
// A7670C TX -> ESP D5 (RX)
// A7670C RX -> ESP D6 (TX)
// GND common
#define MODEM_RX D5
#define MODEM_TX D6

SoftwareSerial modem(MODEM_RX, MODEM_TX);

long activeBaud = 0;
unsigned long lastHeartbeat = 0;

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

bool hasOK(const String &resp) {
  return resp.indexOf("OK") != -1;
}

String cleanLine(String s) {
  s.replace("\r", " ");
  s.replace("\n", " ");
  s.trim();
  return s.length() ? s : "(empty)";
}

bool autoDetectBaud() {
  const long rates[] = {115200, 9600, 57600, 38400, 19200};
  const int count = sizeof(rates) / sizeof(rates[0]);

  Serial.println("=== MODEM BAUD DETECT ===");
  for (int i = 0; i < count; i++) {
    modem.begin(rates[i]);
    delay(400);
    String r = sendAT("AT", 1500);

    Serial.print("TRY ");
    Serial.print(rates[i]);
    Serial.print(" -> ");
    Serial.println(cleanLine(r));

    if (hasOK(r)) {
      activeBaud = rates[i];
      sendAT("ATE0", 1200);
      Serial.print("LOCKED BAUD: ");
      Serial.println(activeBaud);
      return true;
    }
  }
  return false;
}

void runCoreChecks() {
  String r;

  r = sendAT("AT", 1500);
  Serial.print("[AT] ");
  Serial.println(hasOK(r) ? "PASS" : "FAIL");
  Serial.println(cleanLine(r));

  r = sendAT("ATI", 2500);
  Serial.print("[ATI] ");
  Serial.println(hasOK(r) ? "PASS" : "FAIL");
  Serial.println(cleanLine(r));

  r = sendAT("AT+CPIN?", 2500);
  bool simReady = r.indexOf("READY") != -1;
  Serial.print("[CPIN] ");
  Serial.println(simReady ? "PASS (SIM READY)" : "FAIL");
  Serial.println(cleanLine(r));

  r = sendAT("AT+CSQ", 2500);
  Serial.print("[CSQ] ");
  Serial.println(hasOK(r) ? "PASS" : "FAIL");
  Serial.println(cleanLine(r));
}

void setup() {
  Serial.begin(115200);
  delay(1000);
  Serial.println();
  Serial.println("A7670C MODEM HEALTH TEST");
  Serial.println("Wait 12s for modem boot...");
  delay(12000);

  if (!autoDetectBaud()) {
    Serial.println("MODEM LINK FAIL: No valid AT response.");
    Serial.println("Check power, wiring, ground, PWRKEY state.");
    return;
  }

  runCoreChecks();
  Serial.println("Type 'AT' or other command in Serial Monitor, press Enter.");
}

void loop() {
  // Manual passthrough command from PC
  if (Serial.available()) {
    String cmd = Serial.readStringUntil('\n');
    cmd.trim();
    if (cmd.length()) {
      String r = sendAT(cmd, 3000);
      Serial.print(">> ");
      Serial.println(cmd);
      Serial.print("<< ");
      Serial.println(cleanLine(r));
    }
  }

  // Heartbeat every 5 seconds
  if (activeBaud > 0 && millis() - lastHeartbeat > 5000) {
    lastHeartbeat = millis();
    String r = sendAT("AT", 1200);
    Serial.print("[HEARTBEAT] ");
    Serial.println(hasOK(r) ? "MODEM OK" : "MODEM NO RESPONSE");
  }
}
