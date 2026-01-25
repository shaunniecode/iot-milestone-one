# System Readme (Rack + Room MQTT)

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

If `pio` is not recognized, I use the PlatformIO sidebar in VS Code:
PlatformIO → Project Tasks → esp32dev/room → Build.

---

## 2) Switch Wokwi between rack and room

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

After switching, I stop the sim and start it again so Wokwi reloads.

---

## 3) Can I run both sims at the same time?

Not in a single workspace. Wokwi only uses one `diagram.json` and
one `wokwi.toml` per project.

If I want both running at once, I do this:
- Open two VS Code windows on two copies of this repo.
- Each window runs a different Wokwi config (rack vs room).

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

### Subscribe to everything for both devices
```text
dc/2780093K/shaun/esp32/#
dc/5047992u/Naren/esp32/#
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

## 7) Quick test I use

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
