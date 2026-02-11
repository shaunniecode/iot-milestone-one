# IntegratedEdgeMQTT (Milestone 2)

**Author:** Shaun Richard Verghese (Student ID: 2780093K)

## 1) Project overview (start to finish)

This project combines three separate ESP32 milestone builds into one integrated system:

- **Rack device (Shaun)**: monitors rack environment, door access, and power load.
- **Room device (Naren)**: monitors room conditions and raises technician/fire alerts.
- **Building device (Sprinkler alarm)**: monitors tank/temp and handles test/reset.

Milestone 1 delivered three independent devices. Milestone 2 integrates them through a shared MQTT broker so they communicate and behave as one system. MQTT is the "glue" that connects all devices by topics instead of direct wiring.

## 2) System architecture (plain English)

- **Broker**: `broker.hivemq.com` routes messages by topic.
- **Clients**: the three ESP32 devices plus the human tester (HiveMQ Web Client).
- Devices **publish** status/events/telemetry to the broker.
- Devices **subscribe** to command or status topics they care about.
- The broker delivers messages to all subscribers.

## 3) Device summaries

### Rack device (Shaun)
- Tracks temperature, humidity, and power load.
- Door sensor is latched until acknowledged.
- Publishes status, events, telemetry (JSON), plus plain-text mirror topics.
- Subscribes to LED command topics.

### Room device (Naren)
- Tracks temperature, humidity, and smoke simulation.
- Produces technician alerts and fire alarms (retained).
- Publishes JSON plus plain-text mirrors.
- Subscribes to command topic and to rack status/event.
- Sends rack LED1 command on fire alarm.

### Building device (Sprinkler alarm)
- Monitors tank level and temperature.
- Test and reset buttons drive alarm state.
- Publishes JSON plus plain-text mirrors.
- Subscribes to `cmd` for TEST/RESET.

## 4) Build and run

### Build firmware (PlatformIO)
```bash
pio run -e esp32dev
pio run -e room
pio run -e building
```

### Switch Wokwi config
```bat
scripts\switch-wokwi.bat rack
scripts\switch-wokwi.bat room
scripts\switch-wokwi.bat building
```

## 5) MQTT broker settings

- Host: `broker.hivemq.com`
- Port: `1883` (MQTT TCP)

Web client (HiveMQ):
```
https://www.hivemq.com/demos/websocket-client/
```

## 6) Topics (complete list)

### Rack publishes (JSON + plain-text mirrors)
- `dc/2780093K/shaun/esp32/status` (online/offline)
- `dc/2780093K/shaun/esp32/event`
- `dc/2780093K/shaun/esp32/event_text`
- `dc/2780093K/shaun/esp32/telemetry`
- `dc/2780093K/shaun/esp32/telemetry_text`

### Rack subscribes
- `dc/2780093K/shaun/esp32/cmd/led1`
- `dc/2780093K/shaun/esp32/cmd/led2`

### Room publishes (JSON + plain-text mirrors)
- `dc/5047992u/Naren/esp32/telemetry`
- `dc/5047992u/Naren/esp32/telemetry_text`
- `dc/5047992u/Naren/esp32/event`
- `dc/5047992u/Naren/esp32/event_text`
- `dc/5047992u/Naren/esp32/alert/Technician`
- `dc/5047992u/Naren/esp32/alert/Technician_text`
- `dc/5047992u/Naren/esp32/alert/Fire`
- `dc/5047992u/Naren/esp32/alert/Fire_text`
- `dc/5047992u/Naren/esp32/LWT`
- `dc/2780093K/shaun/esp32/cmd/led1` (room -> rack command on fire)

### Room subscribes
- `dc/5047992u/Naren/esp32/command`
- `dc/2780093K/shaun/esp32/status`
- `dc/2780093K/shaun/esp32/event`

### Building publishes (JSON + plain-text mirrors)
- `dc/2780093K/shaun/building/status`
- `dc/2780093K/shaun/building/event`
- `dc/2780093K/shaun/building/event_text`
- `dc/2780093K/shaun/building/telemetry`
- `dc/2780093K/shaun/building/telemetry_text`

### Building subscribes
- `dc/2780093K/shaun/building/cmd`

## 7) Quick test checklist

1. Subscribe in HiveMQ:
   - `dc/2780093K/shaun/esp32/#`
   - `dc/5047992u/Naren/esp32/#`
   - `dc/2780093K/shaun/building/#`
2. Publish test commands:
   - Rack LED1: topic `dc/2780093K/shaun/esp32/cmd/led1`, payload `ON`
   - Room LED: topic `dc/5047992u/Naren/esp32/command`, payload `green_on`
   - Building test: topic `dc/2780093K/shaun/building/cmd`, payload `TEST`

## 8) Hardware wiring (GPIO mapping)

### Rack device (Shaun)
| Component | GPIO | Notes |
| --- | --- | --- |
| DHT22 DATA | GPIO4 | DHTPIN |
| Potentiometer (middle) | GPIO34 | POT_PIN, ADC |
| Button 1 (door) | GPIO14 | BTN1_PIN, `INPUT_PULLDOWN` |
| Button 2 (ACK) | GPIO27 | BTN2_PIN, external pull-down |
| LED1 (cooling) | GPIO12 | LED1_PIN |
| LED2 (alarm) | GPIO13 | LED2_PIN |

### Room device (Naren)
| Component | GPIO | Notes |
| --- | --- | --- |
| DHT22 DATA | GPIO15 | DHTPIN |
| Potentiometer (smoke) | GPIO34 | POT_PIN, ADC |
| LED Green | GPIO12 | LED_GREEN |
| LED Yellow | GPIO14 | LED_YELLOW |
| LED Red | GPIO13 | LED_RED |
| Button ACK | GPIO27 | BTN_ACK, `INPUT_PULLDOWN` |
| Button Fire Reset | GPIO26 | BTN_FIRE, `INPUT_PULLDOWN` |

### Building device (Sprinkler alarm)
| Component | GPIO | Notes |
| --- | --- | --- |
| LED Green (armed) | GPIO12 | LED_GREEN |
| LED Red (alarm) | GPIO13 | LED_RED |
| Button TEST | GPIO14 | BTN_TEST, pressed = HIGH |
| Button RESET | GPIO27 | BTN_RESET, pressed = HIGH |
| Potentiometer (tank) | GPIO34 | POT_PIN, ADC |
| DHT22 DATA | GPIO4 | DHT_PIN |

## 9) Design rationale (key logic)

### Rack alarm logic
- LED2 is state-driven to avoid conflicting writes.
- Door alarm is latched until ACK.
- Env + power alarms auto-clear when conditions normalize.

### Room alert logic
- Technician alert uses consecutive reads to avoid false positives.
- Fire alert is retained and only cleared when safe.
- Room can command rack LED1 during fire for cross-device signaling.

### Building alarm logic
- Test button forces alarm on.
- Reset clears alarm/test.
- Auto alarm if temperature crosses threshold.

## 10) Troubleshooting

### MQTT
- If no messages appear, confirm you subscribed to the correct topic.
- Use wildcards like `dc/2780093K/shaun/esp32/#` to see everything.
- If LWT shows `offline`, reconnect the simulator or broker.

### Wokwi
- After switching configs with `scripts\switch-wokwi.bat`, restart the sim.
- Ensure `diagram.json` and `wokwi.toml` match the intended device.

### Sensors
- DHT can fail on first read; wait for the next interval.
- Pot values are analog and may need adjustment in Wokwi.

## 11) Test cases / expected outputs

### Rack
- Door open -> `event` shows `DOOR_ON`, LED2 ON (latched).
- ACK button -> `event` shows `DOOR_OFF`, LED2 OFF if no other alarms.
- Temp/humidity high -> `event` shows `ENV_ON`, LED2 ON.

### Room
- Out-of-range temp/humidity/smoke -> technician alert published.
- Fire threshold exceeded -> fire alert retained, rack LED1 command `ON`.
- `green_on` command -> green LED turns ON and event logs LED change.

### Building
- TEST command/button -> event `test ON`, alarm ON.
- RESET command/button -> event `reset PRESSED`, alarm OFF.
- Temp threshold exceeded -> event `alarm ON`.

## 12) Documentation map

- `systemreadme.md` — step-by-step usage + MQTT commands
- `mqttreadme.md` — detailed MQTT explanation and relationships
- `diagram.*.json` + `wokwi.*.toml` — Wokwi configs
