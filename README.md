# Lantapan Hub — Signal Booster

ESP32 firmware that reads **real** LTE signal (`AT+CSQ`) from a SIMCom **A7670C**, aims a hobby servo that holds the LTE antenna, and publishes the existing dashboard JSON to MQTT **over the SIM** so `index.html` can show **LIVE** bars and angle.

There is no mock RSSI in the firmware. The dashboard does not invent live readings — it waits for the cellular hub.

**WiFi is not required.** MQTT uses the A7670C’s built-in MQTT AT client on `tcp://broker.emqx.io:1883` (not TLS / not 8883).

A **TM** (Globe prepaid) SIM needs **mobile data** enabled (load / promo). Voice-only credit will not bring the hub online.

## What you need

- ESP32 DevKit V1
- SIMCom A7670C (LTE Cat-1). There is no “A7076C” / “simcol” part — use A7670C.
- A data-enabled SIM: **TM** (now), also Globe, Smart, TNT, or DITO
- One Li-ion cell, a **physical power switch**, a 5 V boost converter
- Hobby servo with the LTE antenna on the horn
- 470 µF (or larger) on A7670C **VIN/GND** if there is room (LTE bursts pull hard)
- Common ground everywhere

## Power path (switch is not a GPIO)

This A7670C board has **VIN only** — no VBAT pad. Do not wire the cell to the modem. VIN is the **5 V** input; the breakout already regulates to ~3.8 V internally.

The switch sits in the **battery +** line so the whole system is dead when the switch is open.

```
Li-ion+ ──► POWER SWITCH ──► boost IN+
                                 │
                                 ▼
                            boost OUT 5.0 V
                                 │
              ┌──────────────────┼──────────────────┐
              ▼                  ▼                  ▼
         A7670C VIN          ESP32 VIN         servo VCC (red)

Li-ion− ──► common GND (boost, A7670C, ESP32, servo)
```

- **Boost EN** (if the converter has an enable pin): tie it to **switched +** so the boost dies with the switch.
- **470 µF+** across A7670C VIN and GND if there is room.

## Pin map (every wire)

Labels below match the **ESP32 DEVKIT V1 silkscreen**. GPIO numbers stay **16 / 17 / 27 / 13**. There is **no D16** printed — UART2 is **RX2** (next to D4) and **TX2** (next to D5). Do **not** use RX0 / TX0 (USB serial).

| From | To | Notes |
| --- | --- | --- |
| A7670C TX | ESP32 **RX2** (GPIO 16, next to D4 — no D16 printed) | Modem → ESP32 |
| A7670C RX | ESP32 **TX2** (GPIO 17, next to D5) | ESP32 → modem, **115200 8N1** |
| A7670C PWRKEY | ESP32 **D27** (GPIO 27) | Firmware pulses **LOW ~1.2 s** if `AT` is silent; skipped if the modem already answers |
| A7670C VIN | Boost **5.0 V** | Board has VIN only. Do not wire the cell to the modem |
| Servo signal (yellow/orange) | ESP32 **D13** (GPIO 13) | PWM |
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
- Hunt and servo **keep running** if NETOPEN or MQTT is down.

`payload.network` is the cellular operator from `AT+COPS?` (Globe / TM / Smart / TNT / DITO, or `LTE`). It is never `"WiFi"`. `internet` is **true only when MQTT is connected**. Satellites stay `0` unless the modem actually answers `AT+CGNSSINFO`.

## SIM / APN (from IMSI)

`AT+CIMI` MCC+MNC picks the APN. Unknown SIMs try the list in order.

| IMSI prefix | Carrier | APN |
| --- | --- | --- |
| 51502 (also 51501) | Globe / **TM** | `internet.globe.com.ph`, then `internet` |
| 51503 (also 51505) | Smart / TNT | `internet` |
| 51566 | DITO | `internet.dito.ph` |
| anything else | try in order | `internet.globe.com.ph`, `internet`, `internet.dito.ph` |

Bring-up: `AT+CGDCONT` → `AT+CGATT=1` → `AT+NETOPEN` → `AT+IPADDR` → `AT+CMQTTSTART` → `AT+CMQTTACCQ` (no SSL) → `AT+CMQTTCONNECT` `tcp://broker.emqx.io:1883`.

## MQTT (do not change these — the web page already uses them)

| | |
| --- | --- |
| Broker | `broker.emqx.io` |
| A7670C (LTE) | `tcp://broker.emqx.io:1883` — no SSL, not 8883 |
| Browser | `wss://broker.emqx.io:8084/mqtt` |
| Status | `signalbooster/hub1/status` |
| Command | `signalbooster/hub1/command` |

`WIFI_FALLBACK` defaults to **0**. Leave it off unless you explicitly want a Wi-Fi backup.

Status JSON (about every 2 s when locked, 5 s while hunting):

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

Board support: **esp32** by Espressif (`https://espressif.github.io/arduino-esp32/package_esp32_index.json`). Board: **ESP32 Dev Module** (DevKit V1).

PubSubClient is **not** required unless you set `WIFI_FALLBACK` to 1.

## How to flash

1. Open `firmware/signal_booster_hub/signal_booster_hub.ino` in Arduino IDE (folder name must match the `.ino` name).
2. Put a **data-enabled** SIM in the A7670C. For **TM**, turn on mobile data (load / promo). No Wi-Fi SSID is needed.
3. Tools: Board **ESP32 Dev Module**, upload speed 115200, the COM port of the DevKit.
4. Upload. Open **Serial Monitor at 115200**. You should see `AT` succeed, IMSI/APN, `NETOPEN`, then `CMQTTCONNECT` and a JSON payload every few seconds.
5. If the modem is silent on battery, **D27** (GPIO 27) pulses PWRKEY and the sketch waits up to ~12 s for boot. It tries **115200** first, then **9600**. If it stays silent: check TX/RX (they must be crossed: A7670C TX → **RX2**, A7670C RX → **TX2**), common GND, and **5 V on A7670C VIN** from the boost.

## Open the dashboard and confirm LIVE

Production: [https://project-signal-booster.vercel.app](https://project-signal-booster.vercel.app)

The dashboard auto-connects to the MQTT broker on load and stays connected with reconnect; there is no Connect Cloud / Disconnect Cloud button.

1. Open the page (or `index.html` locally). It auto-connects to the broker and **waits for the LTE hub**. Until a real MQTT payload arrives, status is **OFFLINE / WAITING FOR LTE** and gauges show **—** (not a fake 0-bar live reading). Cached values are never treated as CONNECTED.
2. Power the hub. Wait until the top pill says **CONNECTED · LIVE** (not DISCONNECTED / WAITING FOR LTE / STALE).
3. Confirm **real** values:
   - Signal number moves with `AT+CSQ` (0–31) and the bars match Weak / Fair / Good / Strong
   - Antenna angle matches the servo
   - SIM shows `READY` (or the modem’s real state)
   - Network is TM / Globe / Smart / TNT / DITO / `LTE` — not `WiFi`
4. Telemetry older than **45 s** goes OFFLINE (last-seen stays dim).
5. Use **Manual** on the page and drag the slider: the servo should follow. Switch back to **Auto** to hunt/hold again.

If the pill stays **WAITING FOR LTE**, the A7670C is not publishing to `broker.emqx.io:1883` — check Serial Monitor (`NETOPEN` / `CMQTTCONNECT`), SIM data, and that the TM/Globe/Smart/DITO APN attached.
