# System Readme (Rack + Room + Building MQTT)

This file is my plain‑English guide to build, run, and test both ESP32 devices,
plus the exact MQTT subscribe/publish commands I use in HiveMQ.

---

## 1) Build the firmware (PlatformIO)

### Build the rack device (Shaun)
Command:
```powershell  
pio run -e esp32dev
```
Explanation:
- This builds the rack‑level firmware (the Shaun device).
- The output is placed under `.pio/build/esp32dev/`.

### Build the room device (Naren)
Command:
```powershell
pio run -e room
```
Explanation:
- This builds the room‑level firmware (the Naren device).
- The output is placed under `.pio/build/room/`.

### Build the building device (Sprinkler alarm)
Command:
```powershell
pio run -e building
```
Explanation:
- This builds the building-level firmware (sprinkler alarm).
- The output is placed under `.pio/build/building/`.

If `pio` is not recognized, I use the PlatformIO sidebar in VS Code:
PlatformIO → Project Tasks → esp32dev/room → Build.

---

## 2) Switch Wokwi between rack, room, and building

Wokwi always uses `diagram.json` and `wokwi.toml`.  
I created a helper so I can swap those files in one command.

### Switch to rack
Command:
```bat
scripts\switch-wokwi.bat rack
```
Explanation:
- Copies `diagram.rack.json` → `diagram.json`
- Copies `wokwi.rack.toml` → `wokwi.toml`
- This makes Wokwi run the rack device.

### Switch to room
Command:
```bat
scripts\switch-wokwi.bat room
```
Explanation:
- Copies `diagram.room.json` → `diagram.json`
- Copies `wokwi.room.toml` → `wokwi.toml`
- This makes Wokwi run the room device.

### Switch to building
Command:
```bat
scripts\switch-wokwi.bat building
```
Explanation:
- Copies `diagram.building.json` → `diagram.json`
- Copies `wokwi.building.toml` → `wokwi.toml`
- This makes Wokwi run the building device.

After switching, I stop the sim and start it again so Wokwi reloads.

---

## 3) Can I run multiple sims at the same time?

Not in a single workspace. Wokwi only uses one `diagram.json` and
one `wokwi.toml` per project.

If I want multiple running at once, I do this:
- Open multiple VS Code windows on multiple copies of this repo.
- Each window runs a different Wokwi config (rack vs room vs building).

---

## 4) HiveMQ Web Client connection settings

The web client uses WebSockets, not TCP.

### Primary (TLS)
Host: `broker.hivemq.com`  
Port: `8884`  
SSL: **enabled**  
Username/Password: blank

### Fallback (non‑TLS)
Host: `broker.hivemq.com`  
Port: `8000`  
SSL: **disabled**

Web client URL:
```
https://www.hivemq.com/demos/websocket-client/
```

---

## 5) Subscribe commands (what I listen to)

### Subscribe to everything for all devices
```text
dc/2780093K/shaun/esp32/#
dc/5047992u/Naren/esp32/#
dc/2780093K/shaun/building/#
```
Explanation:
- The `#` wildcard means “everything under this namespace.”
- This lets me see all telemetry, events, and alerts.

### Subscribe to minimal core topics
```text
dc/2780093K/shaun/esp32/status
dc/2780093K/shaun/esp32/event
dc/2780093K/shaun/esp32/telemetry
dc/5047992u/Naren/esp32/telemetry
dc/5047992u/Naren/esp32/alert/+
dc/2780093K/shaun/building/status
dc/2780093K/shaun/building/event
dc/2780093K/shaun/building/telemetry
```
Explanation:
- `status` shows online/offline.
- `event` shows discrete events (door, alarms, ACK, etc.).
- `telemetry` shows periodic sensor snapshots.
- `alert/+` shows technician + fire alerts from the room device.

---

## 6) Publish commands (what I send)

### Rack device (Shaun)

**LED1 cooling indicator (ON/OFF)**
Topic:
```text
dc/2780093K/shaun/esp32/cmd/led1
```
Payload:
```text
ON
```
Explanation:
- `ON` turns the rack cooling indicator on.
- `OFF` turns it off.

**LED2 alarm override (ON/OFF)**
Topic:
```text
dc/2780093K/shaun/esp32/cmd/led2
```
Payload:
```text
OFF
```
Explanation:
- This manually forces the alarm LED, but the rack logic may override it
  because LED2 is state‑driven in the rack code.

### Room device (Naren)

**Room command topic (JSON payloads)**
Topic:
```text
dc/5047992u/Naren/esp32/command
```

Payload examples:
```json
{"led":"green","state":1}
{"led":"yellow","state":0}
{"led":"red","state":1}
{"clear_fire":1}
{"request_telemetry":1}
{"request_status":1}
```
Explanation:
- LED commands directly control the room LEDs.
- `clear_fire` attempts to clear the fire alarm (only if temp is safe).
- `request_telemetry` publishes telemetry immediately.
- `request_status` publishes a one‑off status snapshot.

---

### Building device (Sprinkler alarm)

**Published topics**
```text
dc/2780093K/shaun/building/status
dc/2780093K/shaun/building/event
dc/2780093K/shaun/building/telemetry
```

**Command topic**
```text
dc/2780093K/shaun/building/cmd
```

**Command payloads**
```text
TEST
RESET
```

**Status / presence (LWT, retained)**
- `online` published on connect
- `offline` published by broker if device drops

**Example payloads**
```json
{"event":"test","value":"ON","ts":123456}
{"event":"reset","value":"PRESSED","ts":123789}
{"event":"alarm","value":"ON","ts":124000}
```
```json
{"temp_c":31.2,"humidity_pct":45.5,"tank_pct":72,"armed":1,"test_mode":0,"alarm":0,"ts":130000}
```

---

## 7) Example: Shaun MQTT appears in Naren output

This happens because the room device subscribes to Shaun's rack topics
(`dc/2780093K/shaun/esp32/status` and `dc/2780093K/shaun/esp32/event`)
and logs all received messages.

```text
[MQTT TX] dc/5047992u/Naren/esp32/event : {"event":"rack_status","value":0,"ts":150}
[1970-01-01 08:02:30] [MQTT RX] dc/2780093K/shaun/esp32/event : {"event":"system","value":"online","ts":2534588}
[1970-01-01 08:02:31] [SENSOR] T:21.7C H:49.0% Smoke:0%
```

---

## 8) Quick test I use

1) Run rack sim and subscribe to:
   `dc/2780093K/shaun/esp32/#`
2) Run room sim and subscribe to:
   `dc/5047992u/Naren/esp32/#`
3) In room sim, raise DHT temperature above 57°C.
4) I expect:
   - Fire alert retained in room namespace
   - Room telemetry shows `fire_active: 1`
   - A command `ON` is published to the rack LED1 topic

If the rack device is not running at the same time, I will still see the
`ON` command in HiveMQ but the rack LED won’t change.

### Building quick test
1) Run building sim and subscribe to:
   `dc/2780093K/shaun/building/#`
2) Press TEST button (BTN1).
3) I expect:
   - Event `{"event":"test","value":"ON",...}` published
   - Alarm event `{"event":"alarm","value":"ON",...}` published
   - Telemetry shows `test_mode: 1` and `alarm: 1` on next interval
4) Press RESET button (BTN2).
5) I expect:
   - Event `{"event":"reset","value":"PRESSED",...}` published
   - Alarm event `{"event":"alarm","value":"OFF",...}` published
   - Telemetry shows `test_mode: 0` and `alarm: 0` on next interval
