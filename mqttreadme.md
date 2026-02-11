# MQTT Readme (Plain English Guide)

This document explains how MQTT is used across the Rack, Room, and Building devices in this project, in simple terms. It also lists every topic that is published or subscribed, and includes a practical “how to test” section.

---

**MQTT In One Minute**
- MQTT is a lightweight messaging system.
- Devices **publish** messages to topics.
- Other devices or clients **subscribe** to topics to receive those messages.
- A **broker** (HiveMQ in this project) is the middleman that routes messages.

---

**Roles In This Project**
- **Broker**: `broker.hivemq.com` (public HiveMQ broker).
- **Clients**:
  - Rack device (Shaun’s ESP32)
  - Room device (Naren’s ESP32)
  - Building device (Sprinkler alarm ESP32)
- You (the human tester using HiveMQ Web Client or any MQTT client)

---

**Topic Naming Pattern**
- Topics use this pattern:
  - `dc/<student_id>/<name>/<device>/<category>`
- Example:
  - `dc/2780093K/shaun/esp32/telemetry`

---

**Published Topics (Device → Broker)**

**Rack device (Shaun) publishes**
- `dc/2780093K/shaun/esp32/status`
  - Purpose: online/offline presence (retained + LWT).
  - Payload: `online` or `offline`
- `dc/2780093K/shaun/esp32/event`
  - Purpose: discrete events (door, alarm transitions, ACK, system online).
  - Payload: JSON, e.g. `{"event":"door","value":"OPEN","ts":123456}`
- `dc/2780093K/shaun/esp32/event_text`
  - Purpose: plain‑text mirror of event messages for quick reading.
  - Payload: text, e.g. `event=door value=OPEN ts=123456`
- `dc/2780093K/shaun/esp32/telemetry`
  - Purpose: periodic sensor snapshot.
  - Payload: JSON, e.g. `{"temp_c":31.2,"humidity_pct":72.5,"load_pct":84,"led1":1,"led2":1,"ts":130000}`
- `dc/2780093K/shaun/esp32/telemetry_text`
  - Purpose: plain‑text mirror of telemetry for quick reading.
  - Payload: text, e.g. `temp_c=31.2 humidity_pct=72.5 load_pct=84 led1=1 led2=1 ts=130000`

**Room device (Naren) publishes**
- `dc/5047992u/Naren/esp32/telemetry`
  - Purpose: periodic room sensor snapshot.
  - Payload: JSON, e.g. `{"temp":24.3,"hum":51.2,"pot":1234,"smoke_pct":30,"tech_latched":0,"fire_active":0,"wifi_rssi":-61,"ts":1700000000}`
- `dc/5047992u/Naren/esp32/telemetry_text`
  - Purpose: plain‑text mirror of telemetry for quick reading.
  - Payload: text, e.g. `temp=24.3 hum=51.2 pot=1234 smoke_pct=30 tech_latched=0 fire_active=0 wifi_rssi=-61 ts=1700000000`
- `dc/5047992u/Naren/esp32/event`
  - Purpose: discrete events (buttons, rack status, alert changes).
  - Payload: JSON, e.g. `{"event":"button_ack_pressed","value":1,"ts":1700000000}`
- `dc/5047992u/Naren/esp32/event_text`
  - Purpose: plain‑text mirror of event messages.
  - Payload: text, e.g. `event=button_ack_pressed value=1 ts=1700000000`
- `dc/5047992u/Naren/esp32/alert/Technician`
  - Purpose: technician alert warnings and clears.
  - Payload: JSON, e.g. `{"level":"warning","temp":28.1,"hum":65.2,"reason":"humidity_high","ts":1700000000}`
- `dc/5047992u/Naren/esp32/alert/Technician_text`
  - Purpose: plain‑text mirror of technician alerts.
  - Payload: text, e.g. `level=warning temp=28.1 hum=65.2 reason=humidity_high ts=1700000000`
- `dc/5047992u/Naren/esp32/alert/Fire`
  - Purpose: fire alarm alerts (retained).
  - Payload: JSON, e.g. `{"level":"critical","temp":58.4,"reason":"fire_detected","ts":1700000000}`
- `dc/5047992u/Naren/esp32/alert/Fire_text`
  - Purpose: plain‑text mirror of fire alerts (retained).
  - Payload: text, e.g. `level=critical temp=58.4 reason=fire_detected ts=1700000000`
- `dc/5047992u/Naren/esp32/LWT`
  - Purpose: online/offline presence (retained).
  - Payload: JSON, e.g. `{"status":"online"}`
- `dc/2780093K/shaun/esp32/cmd/led1`
  - Purpose: signal the rack cooling LED on fire alarm.
  - Payload: `ON` or `OFF`

**Building device (Sprinkler alarm) publishes**
- `dc/2780093K/shaun/building/status`
  - Purpose: online/offline presence (retained + LWT).
  - Payload: `online` or `offline`
- `dc/2780093K/shaun/building/event`
  - Purpose: discrete events (test, reset, alarm).
  - Payload: JSON, e.g. `{"event":"test","value":"ON","ts":123456}`
- `dc/2780093K/shaun/building/event_text`
  - Purpose: plain‑text mirror of event messages.
  - Payload: text, e.g. `event=test value=ON ts=123456`
- `dc/2780093K/shaun/building/telemetry`
  - Purpose: periodic sensor snapshot.
  - Payload: JSON, e.g. `{"temp_c":31.2,"humidity_pct":45.5,"tank_pct":72,"armed":1,"test_mode":0,"alarm":0,"ts":130000}`
- `dc/2780093K/shaun/building/telemetry_text`
  - Purpose: plain‑text mirror of telemetry for quick reading.
  - Payload: text, e.g. `temp_c=31.2 humidity_pct=45.5 tank_pct=72 armed=1 test_mode=0 alarm=0 ts=130000`

---

**Subscribed Topics (Broker → Device)**

**Rack device (Shaun) subscribes**
- `dc/2780093K/shaun/esp32/cmd/led1`
  - Purpose: cooling indicator ON/OFF commands.
  - Payload: `ON` or `OFF`
- `dc/2780093K/shaun/esp32/cmd/led2`
  - Purpose: alarm LED override (kept for completeness).
  - Payload: `ON` or `OFF`

**Room device (Naren) subscribes**
- `dc/5047992u/Naren/esp32/command`
  - Purpose: commands sent to the room device.
  - Payload (JSON examples):
    - `{"led":"green","state":1}`
    - `{"clear_fire":1}`
    - `{"request_telemetry":1}`
  - Payload (plain‑text, case‑insensitive):
    - `green_on`, `green_off`, `yellow_on`, `yellow_off`, `red_on`, `red_off`
    - `clear_fire`, `request_telemetry`, `request_status`
- `dc/2780093K/shaun/esp32/status`
  - Purpose: rack online/offline status (room logs it as events).
  - Payload: `online` or `offline`
- `dc/2780093K/shaun/esp32/event`
  - Purpose: rack alarm events (door/env/power).
  - Payload: JSON from rack device

**Building device (Sprinkler alarm) subscribes**
- `dc/2780093K/shaun/building/cmd`
  - Purpose: command inputs.
  - Payload: `TEST` or `RESET`

---

**Wildcards: What They Mean**
- `#` means “everything under this path.”
- `+` means “exactly one level here.”
- Wildcards are normally used in your **MQTT client** to monitor.
- Devices in this project subscribe to exact topics only (no wildcards in device code).

Examples for your client:
- `dc/2780093K/shaun/esp32/#` → see all rack messages
- `dc/5047992u/Naren/esp32/#` → see all room messages
- `dc/2780093K/shaun/building/#` → see all building messages

---

**How To Test (HiveMQ Web Client)**

1. Open the web client:
   - `https://www.hivemq.com/demos/websocket-client/`
2. Connect using:
   - Host: `broker.hivemq.com`
   - Port: `8884` with SSL enabled (or `8000` without SSL)
3. Subscribe to:
   - `dc/2780093K/shaun/esp32/#`
   - `dc/5047992u/Naren/esp32/#`
   - `dc/2780093K/shaun/building/#`
4. Publish test commands:
   - Rack LED1: topic `dc/2780093K/shaun/esp32/cmd/led1`, payload `ON`
   - Room green LED: topic `dc/5047992u/Naren/esp32/command`, payload `green_on`
   - Building test: topic `dc/2780093K/shaun/building/cmd`, payload `TEST`

---

**Quick Summary**
- The broker routes messages based on topics.
- Devices publish status, events, and telemetry.
- Devices subscribe to command topics.
- Wildcards are for your client, not required in device code.
