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

