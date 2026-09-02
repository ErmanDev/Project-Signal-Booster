# Lantapan Hub — Signal Booster

ESP32 firmware that reads **real** LTE signal (`AT+CSQ`) from a SIMCom **A7670C**, aims a hobby servo that holds the LTE antenna, and publishes the existing dashboard JSON over **cellular MQTT** so `index.html` can show **LIVE** bars and angle.

There is no mock RSSI. **WiFi is not required.** The SIM needs mobile data.

## What you need

- ESP32 DevKit V1
- SIMCom A7670C (LTE Cat-1). There is no “A7076C” / “simcol” part — use A7670C.
- A SIM with **mobile data**: DITO, Smart, TNT, TM, or Globe. **TM** (Globe prepaid) uses the Globe APN.
- One Li-ion cell, a **physical power switch**, a 5 V boost converter
- Hobby servo with the LTE antenna on the horn
- 470 µF (or larger) on A7670C **VIN/GND** if there is room (LTE bursts pull hard)
- Common ground everywhere

## Power path (switch is not a GPIO)

This A7670C board has **VIN only** — no VBAT pad. Do not wire the cell to the modem. VIN is the **5 V** input; the breakout already regulates to ~3.8 V internally.

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

| From | To | Notes |
| --- | --- | --- |
| A7670C TX | ESP32 **GPIO 16** (RX2) | Modem → ESP32 |
| A7670C RX | ESP32 **GPIO 17** (TX2) | ESP32 → modem, **115200 8N1** |
| A7670C PWRKEY | ESP32 **GPIO 27** | Firmware pulses **LOW ~1.2 s** if `AT` is silent; skipped if the modem already answers |
| A7670C VIN | Boost **5.0 V** | Board has VIN only. Do not wire the cell to the modem |
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
- Hunt **keeps running** even if `NETOPEN` or MQTT fails (weak signal). Getting online is not required to aim the antenna.
- **Manual:** dashboard `{ "mode":"manual", "servo_angle": N }` moves the horn and holds. `{ "mode":"auto" }` returns to hunt/hold.
- Last **best_angle** is stored in ESP32 **NVS / Preferences**. After you flip the power switch back on, the servo starts near the last good heading.

`payload.network` is the cellular operator from `AT+COPS?` (TM / Globe / Smart / TNT / DITO). It is never `"WiFi"`. Satellites stay `0` unless GNSS is actually read.

## Cellular MQTT (no WiFi)

The A7670C talks to the same public broker the webpage uses, over **plain MQTT TCP** (not TLS, not port 8883):

1. Wait `AT+CPIN?` **READY**
2. `AT+COPS=0`, wait `+CREG` / `+CGREG` **1 or 5** (about 60 s, servo still hunts)
3. Pick APN from `AT+CIMI` (Philippines MCC **515**), no user/pass:
   - **51502 Globe and TM** → `internet.globe.com.ph`, fallback `internet`
   - **51503 Smart and TNT** → `internet`
   - **51566 DITO** → `internet.dito.ph`
   - unknown → `internet`, then `internet.globe.com.ph`, then `internet.dito.ph`
4. `AT+CGDCONT=1,"IP","<apn>"`
5. `AT+CGATT=1`, `AT+NETOPEN`, `AT+IPADDR` (that IP is `payload.ip`)
6. `AT+CMQTTSTART`
7. `AT+CMQTTACCQ=0,"signalbooster-hub1"` (plain MQTT, no SSL flag)
8. `AT+CMQTTCONNECT=0,"tcp://broker.emqx.io:1883",60,1`
9. Subscribe `signalbooster/hub1/command`
10. Publish JSON to `signalbooster/hub1/status`

| | |
| --- | --- |
| Broker | `broker.emqx.io` |
| Device | `tcp://broker.emqx.io:1883` (A7670C built-in MQTT AT) |
| Browser | `wss://broker.emqx.io:8084/mqtt` |
| Status | `signalbooster/hub1/status` |
| Command | `signalbooster/hub1/command` |

`internet` is **true only when MQTT is actually connected**. Publish is about every **2 s** when the antenna is locked (3+ bars), and about every **5 s** while hunting so TX bursts do not wreck CSQ.

WiFi is not used. A compile-time `WIFI_FALLBACK` exists in the sketch and defaults to **0** (off).

Status JSON:

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

`signal` is the raw `AT+CSQ` RSSI **0–31**.

## Libraries

In Arduino IDE: **Sketch → Include Library → Manage Libraries**, then install:

1. **ESP32Servo** (Kevin Harrington / John K. Bennett)

Board support: **esp32** by Espressif (`https://espressif.github.io/arduino-esp32/package_esp32_index.json`). Board: **ESP32 Dev Module** (DevKit V1).

`PubSubClient` is only needed if you compile with `WIFI_FALLBACK 1`. Leave it off.

## How to flash

1. Open `firmware/signal_booster_hub/signal_booster_hub.ino` in Arduino IDE (folder name must match the `.ino` name).
2. Put a **data-enabled** SIM in the A7670C. TM works with the Globe APN (`internet.globe.com.ph`). No WiFi name or password is required.
3. Tools: Board **ESP32 Dev Module**, upload speed 115200, the COM port of the DevKit.
4. Upload. Open **Serial Monitor at 115200**. You should see `SIM READY`, registration, APN, `NETOPEN`, then `Cellular MQTT up` and a JSON payload.
5. If the modem is silent on battery, GPIO 27 pulses PWRKEY and the sketch waits up to ~12 s for boot. It tries **115200** first, then **9600**. If it stays silent: check TX/RX (they must be crossed), common GND, and **5 V on A7670C VIN** from the boost.

## Open the dashboard and confirm LIVE

1. Open `index.html` in a browser (double-click, or any static host). The page auto-connects to the same broker/topics over WSS 8084.
2. Wait until the top pill says **CONNECTED · LIVE** (not DISCONNECTED / NO DATA / STALE).
3. Confirm **real** values:
   - Signal number moves with `AT+CSQ` (0–31) and the bars match Weak / Fair / Good / Strong
   - Antenna angle matches the servo
   - SIM shows `READY`
   - Network is **TM / Globe / Smart / TNT / DITO** (or the COPS name) — not `WiFi`
4. Use **Manual** on the page and drag the slider: the servo should follow. Switch back to **Auto** to hunt/hold again.

If the pill stays **NO DATA**, the modem is not on `broker.emqx.io:1883` yet. Watch Serial for SIM / register / APN / `NETOPEN` / `CMQTTCONNECT`. The servo should still hunt while that comes up. The SIM must have mobile data.
