# ESP32 Data Center Rack Monitoring & Control Module (Wokwi + MQTT)

**Author:** Shaun Richard Verghese (Student ID: 2780093K)

## 1) What this project is (plain words)

I’m simulating a small “rack module” inside a data centre rack using an ESP32 in
Wokwi. In a real data centre, racks must be monitored for environment conditions,
physical access, and abnormal operating states. When something goes wrong, a
technician needs a clear alarm signal and a clear audit trail (events + telemetry).

My rule for this project: every component must have a believable data-centre
purpose, not just “a demo sensor”.

## 2) Hardware used (and what each part represents)

### LED1 (Green) — Cooling fan status indicator (remote-controlled)

- LED1 simulates a rack’s cooling/fan status indicator.
- A backend system can turn it ON/OFF via MQTT.

### LED2 (Alarm) — Alarm beacon (state-driven)

- LED2 is the “alarm light” for the rack module.
- Key design rule I learned while debugging:
  - I must **not** let multiple parts of the code fight over the same LED output.
  - Each alarm source sets its own boolean state.
  - LED2 is computed from those states in **one place**.
- LED2 turns ON if **any** alarm source is active:
  - `doorAlarmActive OR envAlarmActive OR powerAlarmActive`
- LED2 turns OFF only when **all** alarm states are false.

### Button 1 (Door sensor) — Rack door open/close

- Pressed = door **OPEN**
- Released = door **CLOSED**
- Door OPEN triggers a **latched** door alarm (stays active until acknowledged)
- Door CLOSED does **not** clear the alarm (realistic: tech must acknowledge)

### Button 2 (Technician ACK) — Human acknowledgement

- Press publishes an ACK event including:
  - my identity (`shaun/2780093K`)
  - which alarm sources were active at the moment of ACK
- Design choice:
  - Button 2 clears **only** the door alarm (latched condition).
  - Environment + power alarms are real-time conditions and should clear only
    when the condition returns to normal.

### Potentiometer (Analog) — Simulated rack power load (%)

- ADC read is converted to 0–100%
- Published in telemetry as `load_pct`
- If `load_pct >= threshold`, the power alarm becomes active

### DHT22 — Temperature + Humidity (telemetry + env alarm)

- Read periodically (every 60 seconds) to avoid hammering the sensor
- If temp/humidity exceed thresholds, env alarm becomes active

## 3) Pin mapping (Wokwi wiring must match this)

| Component | GPIO | Notes |
| --- | --- | --- |
| DHT22 DATA | GPIO4 | Use pull-up in Wokwi wiring |
| Potentiometer (middle) | GPIO34 | ADC |
| Button 1 (door) | GPIO14 | `INPUT_PULLDOWN` in code |
| Button 2 (ACK) | GPIO27 | `INPUT`, assumes external pull-down in wiring |
| LED1 | GPIO12 | — |
| LED2 | GPIO13 | — |

## 4) Alarm thresholds (demo-friendly)

- `TEMP_ALARM_C` = 30.0 °C
- `HUM_ALARM_PCT` = 70.0 %
- `POWER_ALARM_PCT` = 80 %

## 5) MQTT broker details

- Host: `broker.hivemq.com`
- Port: `1883`

## 6) MQTT topic structure

Base topic (device namespace):

```
dc/2780093K/shaun/esp32
```

I include:

- `dc` (data centre prefix)
- student id (`2780093K`)
- username (`shaun`)
- device type (`esp32`)

This prevents topic collisions when multiple students/devices are used.

### 6.1 Published topics (ESP32 → Broker)

#### A) Status / Presence (LWT enabled)

Topic:

```
dc/2780093K/shaun/esp32/status
```

Payloads:

- `online` (published by ESP32 on successful MQTT connect; retained)
- `offline` (published automatically by broker via LWT if ESP32 dies; retained)

#### B) Events (instant, discrete changes)

Topic:

```
dc/2780093K/shaun/esp32/event
```

Payload format: JSON

Examples:

```json
{"event":"system","value":"online","ts":12345}
{"event":"door","value":"OPEN","ts":12345}
{"event":"alarm","value":"DOOR_ON","ts":12345}
{"event":"alarm","value":"ENV_ON","ts":12345}
{"event":"alarm","value":"POWER_ON","ts":12345}
{"event":"ack","value":"shaun/2780093K ACK DOOR,ENV","ts":12345}
```

#### C) Telemetry (periodic snapshots)

Topic:

```
dc/2780093K/shaun/esp32/telemetry
```

Publish interval:

- Every `SENSOR_INTERVAL_MS` (60 seconds)

### 6.2 Subscribed topics (Broker → ESP32)

#### A) LED1 cooling fan control

Topic:

```
dc/2780093K/shaun/esp32/cmd/led1
```

Payloads:

- `ON`
- `OFF`

#### B) LED2 alarm control (kept for completeness)

Topic:

```
dc/2780093K/shaun/esp32/cmd/led2
```

Payloads:

- `ON`
- `OFF`

### 6.3 Wildcard subscription (monitor everything)

Subscribe to:

```
dc/2780093K/shaun/esp32/#
```

## 7) Core design rule (why LED2 is stable)

LED2 is controlled in **one place only**:

- Door logic sets `doorAlarmActive` (latched)
- DHT logic sets `envAlarmActive` (automatic)
- Pot logic sets `powerAlarmActive` (automatic)

Then:

- LED2 = ON if **any** of those states are true

## 8) How ACK works (and what it means)

When I press Button 2:

1. I publish an ACK event like:
   - `"shaun/2780093K ACK DOOR,ENV"`
2. I clear **only** `doorAlarmActive` (latched condition)
3. `env` + `power` alarms remain active until conditions return to normal

So:

- MQTT **can** show multiple anomalies (ENV + POWER + DOOR) because transitions are
  published as separate alarm events.
- I **can** acknowledge multiple active alarms in one press because the ACK payload
  lists everything active at that moment.
- I **only clear** the DOOR alarm with ACK (by design).

## 9) Running & testing (Wokwi + HiveMQ)

1. Start Wokwi simulation and open Serial Monitor.
2. In HiveMQ Web Client:
   - Host: `broker.hivemq.com`
   - Port: `1883`
   - Subscribe: `dc/2780093K/shaun/esp32/#`

### Test LED1

- Publish to `dc/2780093K/shaun/esp32/cmd/led1` with `ON` or `OFF`.

### Test door alarm

- Button 1 press → `DOOR_ON`, LED2 on (latched)
- Button 2 press → `DOOR_OFF`, LED2 off (if no other alarms)

### Test env/power alarms

- Change temp/humidity/pot and wait for the next 60s sensor interval for alarm
  transitions (`ENV_ON/OFF`, `POWER_ON/OFF`).

## 11) Room device + multi-build workflow

This repo now includes two builds:

- Rack device (Shaun): `env:esp32dev`
- Room device (Naren): `env:room`

### Build both firmwares (PlatformIO)

```bash
pio run -e esp32dev
pio run -e room
```

### Switch Wokwi config (rack vs room)

Wokwi uses `diagram.json` + `wokwi.toml`. Use the helper to switch:

```bat
scripts\switch-wokwi.bat rack
scripts\switch-wokwi.bat room
```

If you want both simulations running at the same time, open two VS Code
windows on two copies of the repo.

## 12) HiveMQ subscriptions and test publishes

### Subscribe (everything for each device)

```text
dc/2780093K/shaun/esp32/#
dc/5047992u/Naren/esp32/#
```

### Subscribe (minimal)

```text
dc/2780093K/shaun/esp32/status
dc/2780093K/shaun/esp32/event
dc/2780093K/shaun/esp32/telemetry
dc/5047992u/Naren/esp32/telemetry
dc/5047992u/Naren/esp32/alert/+
```

### Example: Shaun MQTT appears in Naren output

This happens because the room device subscribes to Shaun's rack topics
(`dc/2780093K/shaun/esp32/status` and `dc/2780093K/shaun/esp32/event`)
and logs all received messages.

```text
[MQTT TX] dc/5047992u/Naren/esp32/event : {"event":"rack_status","value":0,"ts":150}
[1970-01-01 08:02:30] [MQTT RX] dc/2780093K/shaun/esp32/event : {"event":"system","value":"online","ts":2534588}
[1970-01-01 08:02:31] [SENSOR] T:21.7C H:49.0% Smoke:0%
```

### Publish test commands

Rack LED1 control:

```text
Topic: dc/2780093K/shaun/esp32/cmd/led1
Payload: ON
```

Rack LED2 control:

```text
Topic: dc/2780093K/shaun/esp32/cmd/led2
Payload: OFF
```

Room commands (JSON):

```json
{"led":"green","state":1}
{"led":"yellow","state":0}
{"led":"red","state":1}
{"clear_fire":1}
{"request_telemetry":1}
{"request_status":1}
```

Room command topic:

```text
dc/5047992u/Naren/esp32/command
```

## 10) Quick reference

**Broker**

- `broker.hivemq.com:1883`

**Wildcard subscribe**

- `dc/2780093K/shaun/esp32/#`

**Publish (cooling LED1)**

- Topic: `dc/2780093K/shaun/esp32/cmd/led1`
- Payload: `ON` / `OFF`

**Published topics**

- `dc/2780093K/shaun/esp32/status`
- `dc/2780093K/shaun/esp32/event`
- `dc/2780093K/shaun/esp32/telemetry`
