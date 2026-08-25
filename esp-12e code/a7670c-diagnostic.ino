#include <SoftwareSerial.h>

// ---------- Wiring (ESP-12E <-> A7670C) ----------
// ESP D6/TX -> A7670C RX
// ESP D5/RX -> A7670C TX
// ESP GND     -> A7670C GND
// External stable 5V supply for A7670C is strongly recommended.
#define MODEM_RX D5
#define MODEM_TX D6
#define SERVO_PIN D1

SoftwareSerial modem(MODEM_RX, MODEM_TX);

long modemBaud = 0;
bool firstRunDone = false;

const char *APN_PRIMARY = "internet.globe.com.ph";
const char *APN_FALLBACK = "internet";

String sendAT(const String &command, unsigned long timeoutMs) {
  while (modem.available()) modem.read();
  modem.println(command);

  String response = "";
  unsigned long start = millis();
  while (millis() - start < timeoutMs) {
    while (modem.available()) {
      response += (char)modem.read();
    }
  }
  return response;
}

String oneLine(String value) {
  value.replace("\r", " ");
  value.replace("\n", " ");
  value.trim();
  return value.length() ? value : "(empty)";
}

bool hasOk(const String &resp) {
  return resp.indexOf("OK") != -1;
}

void printStep(const char *name, bool pass, String details) {
  Serial.print(pass ? "[PASS] " : "[FAIL] ");
  Serial.print(name);
  Serial.print(" -> ");
  Serial.println(oneLine(details));
}

bool autoBaud() {
  const long candidates[] = {115200, 9600, 57600, 38400, 19200};
  const int count = sizeof(candidates) / sizeof(candidates[0]);

  Serial.println("=== AUTO-BAUD ===");
  for (int i = 0; i < count; i++) {
    modem.begin(candidates[i]);
    delay(500);

    String resp = sendAT("AT", 1500);
    bool ok = hasOk(resp);

    Serial.print("Try ");
    Serial.print(candidates[i]);
    Serial.print(": ");
    Serial.println(ok ? "OK" : "NO RESPONSE");

    if (ok) {
      modemBaud = candidates[i];
      sendAT("ATE0", 1200);  // disable echo
      Serial.print("Locked baud: ");
      Serial.println(modemBaud);
      return true;
    }
  }
  return false;
}

bool testPdpWithApn(const char *apn) {
  String cmd = String("AT+CGDCONT=1,\"IP\",\"") + apn + "\"";
  String resp = sendAT(cmd, 5000);
  bool ok = hasOk(resp);
  printStep("CGDCONT", ok, resp);
  if (!ok) return false;

  resp = sendAT("AT+CGACT=1,1", 12000);
  ok = hasOk(resp);
  printStep("CGACT", ok, resp);
  if (!ok) return false;

  resp = sendAT("AT+CGPADDR=1", 5000);
  ok = hasOk(resp);
  printStep("CGPADDR", ok, resp);
  return ok;
}

void runDiagnostics() {
  Serial.println();
  Serial.println("=== A7670C DIAGNOSTIC START ===");

  if (!autoBaud()) {
    Serial.println("[FAIL] MODEM LINK -> No AT response on all tested baud rates.");
    Serial.println("Check wiring, power, ground, and PWRKEY state.");
    return;
  }

  String resp = sendAT("ATI", 3000);
  printStep("ATI", hasOk(resp), resp);

  resp = sendAT("AT+CPIN?", 4000);
  bool simReady = (resp.indexOf("READY") != -1);
  printStep("CPIN?", simReady, resp);

  resp = sendAT("AT+CSQ", 4000);
  printStep("CSQ", hasOk(resp), resp);

  resp = sendAT("AT+CREG?", 4000);
  bool regCs = (resp.indexOf(",1") != -1 || resp.indexOf(",5") != -1);
  printStep("CREG?", regCs, resp);

  resp = sendAT("AT+CGREG?", 4000);
  bool regPs = (resp.indexOf(",1") != -1 || resp.indexOf(",5") != -1);
  printStep("CGREG?", regPs, resp);

  resp = sendAT("AT+CGATT=1", 12000);
  bool attached = hasOk(resp);
  printStep("CGATT=1", attached, resp);

  if (!simReady || !attached) {
    Serial.println("[STOP] SIM/Attach not ready. Fix network/SIM first.");
    return;
  }

  Serial.print("Testing APN: ");
  Serial.println(APN_PRIMARY);
  if (testPdpWithApn(APN_PRIMARY)) {
    Serial.println("[PASS] PDP context active with primary APN.");
  } else {
    Serial.print("Testing fallback APN: ");
    Serial.println(APN_FALLBACK);
    if (String(APN_PRIMARY) != String(APN_FALLBACK) && testPdpWithApn(APN_FALLBACK)) {
      Serial.println("[PASS] PDP context active with fallback APN.");
    } else {
      Serial.println("[FAIL] PDP/APN activation failed.");
    }
  }

  resp = sendAT("AT+CGNSPWR=1", 4000);
  printStep("CGNSPWR=1", hasOk(resp), resp);
  resp = sendAT("AT+CGNSINF", 4000);
  printStep("CGNSINF", hasOk(resp), resp);

  Serial.println("=== A7670C DIAGNOSTIC END ===");
}

void setup() {
  Serial.begin(115200);
  delay(1000);
  Serial.println();
  Serial.println("Booting diagnostic sketch...");
  Serial.println("Waiting 12s for modem startup...");
  delay(12000);
}

void loop() {
  if (!firstRunDone) {
    firstRunDone = true;
    runDiagnostics();
    Serial.println();
    Serial.println("Type R + Enter to run diagnostics again.");
    return;
  }

  if (Serial.available()) {
    String cmd = Serial.readStringUntil('\n');
    cmd.trim();
    if (cmd.equalsIgnoreCase("R")) {
      runDiagnostics();
      Serial.println("Type R + Enter to run diagnostics again.");
    }
  }
}
