# Lantapan Hub — Signal Booster

ESP32 firmware that reads **real** LTE signal (`AT+CSQ`) from a SIMCom **A7670C**, aims a hobby servo that holds the LTE antenna, and publishes the existing dashboard JSON to MQTT so `index.html` can show **LIVE** bars and angle.

There is no mock RSSI in the firmware.

## What you need

- ESP32 DevKit V1
- SIMCom A7670C (LTE Cat-1). There is no “A7076C” / “simcol” part — use A7670C.
- One Li-ion cell, a **physical power switch**, a 5 V boost converter
- Hobby servo with the LTE antenna on the horn
- 470 µF (or larger) capacitor across the modem power pins
- Common ground everywhere

## Power path (switch is not a GPIO)

The node is battery-powered. The switch sits in the **battery +** line so the whole system is dead when the switch is open.

```
Li-ion+ ──► POWER SWITCH ──┬──► (A) A7670C VBAT  3.4–4.2 V
                           │     (bare module / VBAT pad only)
                           │
                           └──► (B) boost IN+
                                      │
                                      ▼
                                 boost OUT 5.0 V
                                      │
                          ┌───────────┴───────────┐
                          ▼                       ▼
                     ESP32 VIN              servo VCC (red)
```

- **Bare A7670C / VBAT pad:** 3.4–4.2 V from the switched cell. Never put 5 V into a bare VBAT pad.
- **5 V breakout with an onboard 3.8 V regulator:** feed that board from **boost 5 V after the switch**, not from raw 4.2 V, if the silk/docs say 5 V only.
- **Boost EN** (if the converter has an enable pin): tie it to **switched +** so the boost dies with the switch.
- **Common GND:** battery −, boost GND, ESP32 GND, modem GND, servo GND.
- **470 µF+** on the modem power pins (LTE bursts pull hard).

## Pin map (every wire)

| From | To | Notes |
| --- | --- | --- |
| A7670C TX | ESP32 **GPIO 16** (RX2) | Modem → ESP32 |
| A7670C RX | ESP32 **GPIO 17** (TX2) | ESP32 → modem, **115200 8N1** |
| A7670C PWRKEY | ESP32 **GPIO 27** | Firmware pulses **LOW ~1.2 s** if `AT` is silent; skipped if the modem already answers |
| Servo signal (yellow/orange) | ESP32 **GPIO 13** | PWM |
| Servo VCC (red) | Boost **5.0 V** | Do not power the servo from an ESP32 5 V/3.3 V pin |
| Servo GND (brown) | Common GND | |
| Power switch | **Battery +** only | Not a GPIO |
| Boost EN (if present) | Switched battery + | Boost off when switch is open |
| All grounds | Together | UART and servo will fail without this |

Same map is in the header of `firmware/signal_booster_hub/signal_booster_hub.ino`.

## How the servo behaves

Dashboard bars and the hunt logic use the **same** map as `index.html`:

| `AT+CSQ` RSSI | Quality | Bars | Servo |
| --- | --- | --- | --- |
| ≥ 25 | Strong | 5 | Hold |
| ≥ 18 | Good | 4 | Hold |
| ≥ 12 | Fair | **3** | Hold |
| 0–11 | Weak | 2 | Slow hunt |
| 99 / no read | No Signal | 0 | Slow hunt |

- **Auto:** if the signal is Weak or No Signal, the servo sweeps slowly (2° every 400 ms, 0°↔180°). It remembers the best angle and RSSI. At Fair or better (3+ bars) it **stops and holds**. If the signal later drops below 3 bars, it hunts again.
- **Manual:** dashboard `{ "mode":"manual", "servo_angle": N }` moves the horn and holds. `{ "mode":"auto" }` returns to hunt/hold.
- Last **best_angle** (and best RSSI) is stored in ESP32 **NVS / Preferences**. After you flip the power switch back on, the servo starts near the last good heading.

`payload.network` is the cellular operator from `AT+COPS?`, or `"LTE"` if the name is missing. It is never `"WiFi"`. Satellites stay `0` unless the modem actually answers `AT+CGNSSINFO`.

## MQTT (do not change these — the web page already uses them)

| | |
| --- | --- |
| Broker | `broker.emqx.io` |
| ESP32 | `mqtt://broker.emqx.io:1883` (WiFi backhaul) |
| Browser | `wss://broker.emqx.io:8084/mqtt` |
| Status | `signalbooster/hub1/status` |
| Command | `signalbooster/hub1/command` |

Status JSON (about every 2 seconds):

```json
{
  "internet": true,
  "network": "Globe",
  "signal": 18,
  "signal_quality": "Good",
  "servo_angle": 90,
  "best_angle": 92,
  "best_signal": 20,
  "satellites": 0,
  "sim": "READY",
  "ip": "10.12.34.56",
  "servo_mode": "auto"
}
```

`signal` is the raw `AT+CSQ` RSSI **0–31** (not a fake percent). `ip` is the modem PDP address when the A7670C reports one.

## Libraries

In Arduino IDE: **Sketch → Include Library → Manage Libraries**, then install:

1. **ESP32Servo** (Kevin Harrington / John K. Bennett)
2. **PubSubClient** (Nick O’Leary)

Board support: **esp32** by Espressif (`https://espressif.github.io/arduino-esp32/package_esp32_index.json`). Board: **ESP32 Dev Module** (DevKit V1).

## How to flash

1. Open `firmware/signal_booster_hub/signal_booster_hub.ino` in Arduino IDE (folder name must match the `.ino` name).
2. At the top of the sketch, set your Wi-Fi (this is only the MQTT backhaul, not the “network” field on the dashboard):

   ```cpp
   const char* WIFI_SSID = "YOUR_WIFI_SSID";
   const char* WIFI_PASS = "YOUR_WIFI_PASS";
   ```

3. Tools: Board **ESP32 Dev Module**, upload speed 115200, the COM port of the DevKit.
4. Upload. Open **Serial Monitor at 115200**. You should see `AT` succeed, then CSQ lines and a JSON payload every ~2 s.
5. If the modem is silent on battery, GPIO 27 pulses PWRKEY. If it stays silent: check TX/RX (they must be crossed), common GND, and modem power (3.4–4.2 V on VBAT, or 5 V on a regulated breakout).

## Open the dashboard and confirm LIVE

1. Open `index.html` in a browser (double-click, or any static host). The page auto-connects to the same broker/topics.
2. Wait until the top pill says **CONNECTED · LIVE** (not DISCONNECTED / NO DATA / STALE).
3. Confirm **real** values, not the old offline placeholders:
   - Signal number moves with `AT+CSQ` (0–31) and the bars match Weak / Fair / Good / Strong
   - Antenna angle matches the servo
   - SIM shows `READY` (or the modem’s real state)
   - Network is the carrier name or `LTE` — not `WiFi`
4. Use **Manual** on the page and drag the slider: the servo should follow. Switch back to **Auto** to hunt/hold again.

If the pill stays **NO DATA**, the ESP32 is not reaching `broker.emqx.io:1883` — check `WIFI_SSID` / `WIFI_PASS` and the Serial Monitor MQTT lines.
