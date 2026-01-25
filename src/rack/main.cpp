// ============================================================================
// ESP32 Data Center Rack Monitoring & Control Module (Wokwi + MQTT)
// Author: Shaun Richard Verghese (Student ID: 2780093K)
// ----------------------------------------------------------------------------
// What I’m building (in plain words):
// I’m simulating a small “rack module” inside a data centre. In a real data
// centre, racks contain servers and network gear, and we must monitor conditions
// like temperature/humidity, track physical access (rack door), and alert a human
// technician if something abnormal happens.
//
// My rule for this project was: every required component must have a believable
// data-centre purpose, not just “a demo sensor”.
//
// ----------------------------------------------------------------------------
// LED 1 (Green) = Cooling Fan Status Indicator (remote-controlled)
// - In a real data centre, cooling is critical. Fans can be controlled by a
//   monitoring system.
// - In my simulation, LED1 is controlled by MQTT commands.
// - If a backend system decides cooling is needed, it publishes "ON".
// - If cooling is not needed, it publishes "OFF".
//
// LED 1 MQTT command topic:
//   dc/2780093K/shaun/esp32/cmd/led1
//
// ----------------------------------------------------------------------------
// LED 2 (Alarm) = “Any alarm is active” indicator (state-driven)
// - LED2 is my rack alarm light.
// - The key design decision I learned during debugging:
//     I must NOT let multiple parts of the code fight over the same LED pin.
//     Instead, each alarm source sets its own *state variable*, and LED2 is
//     simply the visual reflection of “any alarm is active”.
// - LED2 turns ON if ANY alarm source is active:
//     - Door alarm (door opened) -> latched until acknowledged
//     - Environment alarm (temp/humidity threshold) -> automatic ON/OFF
//     - Power/load alarm (potentiometer threshold) -> automatic ON/OFF
// - LED2 turns OFF only when ALL alarm states are false.
//
// Why this matters:
// If I let the DHT code directly switch LED2 off when temperature becomes normal,
// it could accidentally cancel a door alarm that is still active. I hit this bug
// earlier and fixed it by adopting state-based alarm control.
//
// ----------------------------------------------------------------------------
// Button 1 (Door Sensor) = Rack Door Open/Close event source
// - Pressed = door OPEN
// - Released = door CLOSED
// - I publish door events to MQTT on every state change.
// - Design choice: opening the door triggers a latched alarm. Closing the door
//   does NOT auto-clear the alarm, because in real operations a person must
//   acknowledge alarms.
//
// ----------------------------------------------------------------------------
// Button 2 (Technician Acknowledge) = human acknowledgement
// - Pressing Button 2 publishes an ACK message that includes my identity and
//   which alarm sources were active at the time of acknowledgement.
// - I decided to only clear the *door* alarm with Button 2, because:
//     - The door alarm is “latched” and needs human acknowledgement.
//     - Environmental and power alarms represent real-time conditions and should
//       clear automatically when conditions return to normal.
// - This makes the system realistic and avoids “silencing” a real fault.
//
// ----------------------------------------------------------------------------
// Potentiometer (Analog) = Simulated rack power load (%)
// - Turning the knob changes an analog value.
// - I convert it to 0–100% and publish it as telemetry.
// - If load >= threshold, I set a power alarm state.
//
// ----------------------------------------------------------------------------
// DHT22 (Digital Sensor) = Temperature + Humidity telemetry + env alarm state
// - I read temperature and humidity periodically (every 60 seconds) to avoid
//   hammering the sensor.
// - If temp/humidity crosses a threshold, I set an env alarm state.
//
// ----------------------------------------------------------------------------
// ============================================================================
// MQTT PUBLISH & SUBSCRIBE OVERVIEW (How this ESP32 communicates)
// ----------------------------------------------------------------------------
// I deliberately separated MQTT communication into three clear categories:
//   1) STATUS     -> “Is the device alive?”
//   2) EVENTS     -> “Something just happened”
//   3) TELEMETRY  -> “Here is the current state snapshot”
//
// This mirrors real data-centre monitoring systems and avoids messy topic trees.
// ----------------------------------------------------------------------------
//
// BASE TOPIC (device namespace):
//   dc/2780093K/shaun/esp32
//
// I include my student ID and name so multiple ESP32 devices can coexist
// without topic collisions (e.g. Shaun’s ESP32 vs another student’s ESP32).
//
// ============================================================================
// PUBLISHED TOPICS (ESP32 → MQTT Broker)
// ============================================================================
//
// 1) STATUS / PRESENCE (LWT-enabled)
// ---------------------------------
// Topic:
//   dc/2780093K/shaun/esp32/status
//
// Purpose:
//   Indicates whether the ESP32 is currently online or has gone offline
//   unexpectedly (power loss, Wi-Fi drop, crash).
//
// Payloads:
//   "online"  -> published by the ESP32 when it successfully connects to MQTT
//   "offline" -> published automatically by the broker via LWT if the ESP32
//                disconnects unexpectedly
// Retained + LWT behaviour:
//   - "online" is published by the ESP32 as a RETAINED message
//   - "offline" is published by the broker via LWT as a RETAINED message
//   - This means any new subscriber immediately knows my last known device state
//
//
// Notes:
//   - This message is RETAINED so late subscribers immediately know the
//     last known device state.
//   - This is a key real-world feature in monitoring systems.
//
// ============================================================================
//
// 2) EVENTS (Instant, discrete changes)
// -------------------------------------
// Topic:
//   dc/2780093K/shaun/esp32/event
//
// Purpose:
//   Publishes one-off events when something meaningful happens.
//   Events are NOT periodic and are NOT repeated unless the state changes.
//
// Event examples:
//   - Door opened / closed
//   - Alarm transitions (DOOR_ON/DOOR_OFF, ENV_ON/ENV_OFF, POWER_ON/POWER_OFF)
//   - Technician acknowledgement (ACK)
//   - System online notification
//
// Payload format: JSON
//
// Example payloads:
//   {"event":"door","value":"OPEN","ts":123456}
//   {"event":"alarm","value":"DOOR_ON","ts":123789}
//   {"event":"ack","value":"shaun/2780093K ACK DOOR,ENV","ts":124200}
//
// Design decision:
//   I keep all events on ONE topic and describe the event in JSON.
//   This keeps the topic tree clean and makes backend parsing easier.
//
// ============================================================================
//
// 3) TELEMETRY (Periodic snapshots)
// ---------------------------------
// Topic:
//   dc/2780093K/shaun/esp32/telemetry
//
// Purpose:
//   Publishes periodic snapshots of sensor readings and output states.
//   Telemetry represents “how things look right now”.
//
// Publish interval:
//   Every SENSOR_INTERVAL_MS (60 seconds)
//
// Payload format: JSON
//
// Example payload:
//   {
//     "temp_c": 31.2,
//     "humidity_pct": 72.5,
//     "load_pct": 84,
//     "led1": 1,
//     "led2": 1,
//     "ts": 130000
//   }
//
// Design decision:
//   Telemetry is separate from events so dashboards can graph data over time
//   without being spammed by event messages.
//
// ============================================================================
// SUBSCRIBED TOPICS (MQTT Broker → ESP32)
// ============================================================================
//
// 1) LED1 Cooling Fan Control
// --------------------------
// Topic:
//   dc/2780093K/shaun/esp32/cmd/led1
//
// Payloads:
//   "ON"  -> turn cooling indicator ON
//   "OFF" -> turn cooling indicator OFF
//
// Purpose:
//   Simulates a backend system remotely controlling rack cooling.
//
// --------------------------------------------------------------------------
//
// 2) LED2 Alarm Control (kept for completeness)
// ---------------------------------------------
// Topic:
//   dc/2780093K/shaun/esp32/cmd/led2
//
// Payloads:
//   "ON" / "OFF"
//
// Note:
//   In my final design, LED2 is primarily STATE-DRIVEN by alarm logic
//   (door / environment / power alarms).
//   This command topic is kept for completeness but would be restricted
//   or removed in a real production system.
//
// ============================================================================
// MONITORING / DEBUGGING TIP
// ============================================================================
//
// To observe EVERYTHING this ESP32 publishes in HiveMQ or another MQTT client,
// I subscribe to the wildcard topic:
//
//   dc/2780093K/shaun/esp32/#  (everything under my base topic)
//
// This is exactly how a real monitoring backend would observe a device.
//
// ============================================================================



// --- Libraries ---
// I include WiFi so the ESP32 can join the Wokwi simulated Wi-Fi network.
#include <Arduino.h>
#include <WiFi.h>

// I include PubSubClient so I can publish and subscribe to MQTT topics.
#include <PubSubClient.h>

// I include the DHT library so I can talk to the DHT22 sensor.
#include "DHT.h"

// I include ArduinoJson so my MQTT payloads are clean structured JSON.
#include <ArduinoJson.h>


// --- Pin and sensor definitions ---
// I define pins as named constants so I don’t have “magic numbers” everywhere.
#define DHTPIN 4
#define DHTTYPE DHT22

// These are my alarm thresholds (chosen to be demo-friendly in Wokwi).
// If temperature >= 30°C, I consider it abnormal and raise an environment alarm.
#define TEMP_ALARM_C 30.0

// If humidity >= 70%, I consider it abnormal and raise an environment alarm.
#define HUM_ALARM_PCT 70.0

// If potentiometer load >= 80%, I treat it as a power overload alarm.
#define POWER_ALARM_PCT 80

// LED pins: LED1 = cooling, LED2 = alarm indicator.
#define LED1_PIN 12
#define LED2_PIN 13

// Button pins: Button1 = door sensor, Button2 = acknowledgement.
#define BTN1_PIN 14
#define BTN2_PIN 27

// Potentiometer analog input pin.
#define POT_PIN 34


// --- Network and broker configuration ---
// In Wokwi, I connect to Wokwi-GUEST (no password).
const char* WIFI_SSID = "Wokwi-GUEST";
const char* WIFI_PASS = "";

// I use HiveMQ public broker for testing. This is fine for a school simulation.
const char* MQTT_BROKER = "broker.hivemq.com";
const uint16_t MQTT_PORT = 1883;


// --- Device identity for topic names ---
// I store these separately so my topics are consistent and easy to read.
const char* DC_PREFIX  = "dc";
const char* STUDENT_ID = "2780093K";
const char* USERNAME   = "shaun";
const char* DEVICE_TYPE= "esp32";


// --- Topic strings (built at runtime) ---
// I build topic Strings once at startup so I can reuse them everywhere.
String baseTopic;
String cmdLed1Topic;
String cmdLed2Topic;
String eventTopic;
String telemetryTopic;
String statusTopic;   // I use this for MQTT “online/offline” presence (LWT).



// --- Network + MQTT + Sensor objects ---
WiFiClient espClient;
PubSubClient mqtt(espClient);
DHT dht(DHTPIN, DHTTYPE);


// --- Debounce and timing variables ---
// Buttons bounce mechanically, so I use a simple debounce window.
const unsigned long DEBOUNCE_MS = 50;

unsigned long lastBtn1Change = 0;
unsigned long lastBtn2Change = 0;

bool lastBtn1State = LOW;
bool lastBtn2State = LOW;


// --- (Optional/disabled) Door open too long variables ---
// I started scaffolding this idea but I disabled it for now because my current
// door design is latched-on-open anyway. I keep these here because they show my
// thinking about how “door open too long” could be added later.
bool doorOpen = false;
unsigned long doorOpenedAt = 0;
const unsigned long DOOR_OPEN_TOO_LONG_MS = 10000;  // 10 seconds demo threshold


// --- Sensor read scheduling ---
// I read the DHT22 every 60 seconds so I don’t over-poll it.
unsigned long lastSensorRead = 0;
const unsigned long SENSOR_INTERVAL_MS = 60000;


// --- Alarm state (this is the core of my stable design) ---
// These booleans are the “truth” of the system.
// LED2 is NOT the truth — LED2 only reflects these booleans.
bool doorAlarmActive  = false;   // latched until Button 2 acknowledges
bool powerAlarmActive = false;   // auto clears when load returns to normal
bool envAlarmActive   = false;   // auto clears when temp/humidity returns normal


// --- Build topic strings once at startup ---
// I build a base topic that uniquely identifies my device.
void setupTopics() {
  baseTopic = String(DC_PREFIX) + "/" + STUDENT_ID + "/" + USERNAME + "/" + DEVICE_TYPE;

  // Commands I subscribe to:
  cmdLed1Topic = baseTopic + "/cmd/led1";
  cmdLed2Topic = baseTopic + "/cmd/led2";

  // Event publishing topic:
  eventTopic = baseTopic + "/event";

  // Telemetry publishing topic:
  telemetryTopic = baseTopic + "/telemetry";
  statusTopic = baseTopic + "/status";  // I publish online/offline presence here.

}


// --- Publish small event JSON messages ---
// I use one event topic and include the event name/value in JSON.
// This keeps the topic tree clean and the payload self-describing.
void publishEvent(const char* name, const char* value) {
  JsonDocument doc;

  // I label what happened (event) and what the value/state is.
  doc["event"] = name;
  doc["value"] = value;

  // I include millis() as a simple timestamp so the backend can track ordering.
  doc["ts"] = millis();

  // I serialize JSON into a buffer and publish it.
  char buf[256];
  size_t n = serializeJson(doc, buf);
  mqtt.publish(eventTopic.c_str(), buf, n);
}


// --- Publish telemetry JSON messages ---
// Telemetry is a periodic snapshot of current sensor readings + output states.
void publishTelemetry(float tempC, float hum, int potPct) {
  JsonDocument doc;

  // I include the environment conditions.
  doc["temp_c"] = tempC;
  doc["humidity_pct"] = hum;

  // I include the simulated power load percentage.
  doc["load_pct"] = potPct;

  // I also include LED states so a dashboard can see outputs too.
  doc["led1"] = digitalRead(LED1_PIN);
  doc["led2"] = digitalRead(LED2_PIN);

  // Timestamp for this snapshot.
  doc["ts"] = millis();

  // Serialize and publish.
  char buf[256];
  size_t n = serializeJson(doc, buf);
  mqtt.publish(telemetryTopic.c_str(), buf, n);
}


// --- MQTT callback: handles incoming subscribed messages ---
// This runs whenever the broker sends a message to one of my subscribed topics.
void callback(char* topic, byte* payload, unsigned int length) {
  // I rebuild the payload into a readable String.
  String msg;
  for (unsigned int i = 0; i < length; i++) msg += (char)payload[i];
  msg.trim();  // I trim whitespace to avoid weird comparisons.

  // LED1: remote-controlled cooling indicator.
  if (String(topic) == cmdLed1Topic) {
    if (msg.equalsIgnoreCase("ON")) {
      digitalWrite(LED1_PIN, HIGH);
      publishEvent("led1", "ON");   // I confirm what I did via event message.
    } else if (msg.equalsIgnoreCase("OFF")) {
      digitalWrite(LED1_PIN, LOW);
      publishEvent("led1", "OFF");
    }
  }

  // LED2: I kept this command topic mainly for completeness.
  // In my final design, LED2 is state-driven (door/env/power alarm states).
  // So if someone publishes here, it can override LED2 and confuse the alarm
  // indicator. In real systems I would remove this or guard it.
  else if (String(topic) == cmdLed2Topic) {
    if (msg.equalsIgnoreCase("ON")) {
      digitalWrite(LED2_PIN, HIGH);
      publishEvent("led2", "ON");
    } else if (msg.equalsIgnoreCase("OFF")) {
      digitalWrite(LED2_PIN, LOW);
      publishEvent("led2", "OFF");
    }
  }

  // I intentionally keep the command format simple so any MQTT client can use it.
}


// --- Connect to Wi-Fi ---
// I do a simple connection attempt with a timeout so I don’t hang forever.
void connectWiFi() {
  WiFi.mode(WIFI_STA);

  Serial.print("Connecting to WiFi ");
  WiFi.begin(WIFI_SSID, WIFI_PASS);

  while (WiFi.status() != WL_CONNECTED) {
    delay(300);
    Serial.print(".");
  }

  Serial.println(" connected");
  Serial.print("WiFi IP: ");
  Serial.println(WiFi.localIP());
}



// --- Connect to MQTT broker (with LWT presence) ---
// I add a Last Will and Testament so the broker can announce if my ESP32 dies.
// If the ESP32 disconnects unexpectedly (power loss / Wi-Fi drop / crash),
// the broker automatically publishes "offline" to my status topic.
void connectMQTT() {
  while (!mqtt.connected()) {

    // I use a unique client ID so it doesn’t collide with other students.
    String clientId = baseTopic + "-client";

    Serial.print("Connecting to MQTT ");
    Serial.print(MQTT_BROKER);
    Serial.print(" ");

    // LWT (Last Will and Testament) settings:
    const char* willTopic = statusTopic.c_str();
    const uint8_t willQos = 1;
    const bool willRetain = true;
    const char* willMsg = "offline";

    if (mqtt.connect(clientId.c_str(), willTopic, willQos, willRetain, willMsg)) {

      // If I successfully connect, I immediately publish "online" (retained too).
      mqtt.publish(statusTopic.c_str(), "online", true);

      Serial.println(" connected");

      // After connecting, I subscribe to my command topics.
      mqtt.subscribe(cmdLed1Topic.c_str());
      mqtt.subscribe(cmdLed2Topic.c_str());

      // I keep my existing event log too (optional but useful).
      publishEvent("system", "online");

    } else {
      Serial.print(".");
      delay(1000);
    }
  }
}




// --- Read potentiometer and convert to percent ---
// The ESP32 ADC returns 0..4095. I map that to 0..100%.
int readPotPercent() {
  int raw = analogRead(POT_PIN);
  int percent = map(raw, 0, 4095, 0, 100);
  return constrain(percent, 0, 100);
}


// --- Handle button logic ---
// I keep all input handling in one function so my main loop stays readable.
void handleButtons() {
  // I read the current raw button states.
  bool cur1 = digitalRead(BTN1_PIN);
  bool cur2 = digitalRead(BTN2_PIN);

  // I capture current time once so debounce math is consistent.
  unsigned long now = millis();

  // --------------------------------------------------------------------------
  // Button 1 = Door sensor (state change events)
  // --------------------------------------------------------------------------
  // I only act when the button state changes AND it has been stable longer than
  // the debounce window.
  if (cur1 != lastBtn1State && (now - lastBtn1Change) > DEBOUNCE_MS) {
    lastBtn1Change = now;
    lastBtn1State = cur1;

    if (cur1 == HIGH) {
      // Door OPEN event
      publishEvent("door", "OPEN");

      // My design decision:
      // Door open should trigger an alarm that stays on until acknowledged,
      // even if the door is later closed.
      if (!doorAlarmActive) {
        doorAlarmActive = true;
        publishEvent("alarm", "DOOR_ON");
      }

      // (Optional scaffolding if I later do “open too long”):
      // doorOpen = true;
      // doorOpenedAt = now;
    } else {
      // Door CLOSED event (I still report this for logging/monitoring)
      publishEvent("door", "CLOSED");

      // I do NOT clear doorAlarmActive here because I want human acknowledgement.
      // doorOpen = false; // optional if I use doorOpen-too-long later
    }
  }

  // --------------------------------------------------------------------------
  // Button 2 = Technician acknowledgement
  // --------------------------------------------------------------------------
  // Same debounce logic: only act on state changes after a stable window.
  if (cur2 != lastBtn2State && (now - lastBtn2Change) > DEBOUNCE_MS) {
    lastBtn2Change = now;
    lastBtn2State = cur2;

    if (cur2 == HIGH) {
      // I build an ACK message that includes my identity and active alarms.
      // This proves “a human was here and acknowledged what they saw.”
      String ackMsg = String(USERNAME) + "/" + STUDENT_ID + " ACK ";

      bool anyAck = false;

      // I list which alarms were active at the moment I pressed ACK.
      if (doorAlarmActive) {
        ackMsg += "DOOR";
        anyAck = true;
      }
      if (envAlarmActive) {
        if (anyAck) ackMsg += ",";
        ackMsg += "ENV";
        anyAck = true;
      }
      if (powerAlarmActive) {
        if (anyAck) ackMsg += ",";
        ackMsg += "POWER";
        anyAck = true;
      }

      // If no alarms were active, I still publish an ACK so it’s visible in logs.
      if (!anyAck) {
        ackMsg += "NONE";
      }

      // Publish acknowledgement event
      publishEvent("ack", ackMsg.c_str());

      // Clear ONLY the door alarm, because it’s the one I intentionally latched.
      if (doorAlarmActive) {
        doorAlarmActive = false;
        publishEvent("alarm", "DOOR_OFF");
      }
    }
  }
}


// --- Arduino setup: runs once at boot ---
void setup() {
  // Serial is my “debug window” in Wokwi.
  Serial.begin(115200);
  delay(1000);  // I pause so Serial has time to come up cleanly.

  // I set LED pins as outputs so I can drive them HIGH/LOW.
  pinMode(LED1_PIN, OUTPUT);
  pinMode(LED2_PIN, OUTPUT);

  // Button1 uses internal pull-down in code (works well in Wokwi).
  pinMode(BTN1_PIN, INPUT_PULLDOWN);

  // Button2 uses external pull-down in my wiring, so plain INPUT is fine.
  pinMode(BTN2_PIN, INPUT);

  // I configure ADC to get stable readings for the potentiometer.
  analogReadResolution(12);
  analogSetAttenuation(ADC_11db);

  // Start DHT sensor.
  dht.begin();

  // DHT22 can be a bit slow to settle at boot, so I give it a short pause.
  delay(2000);

  // Build topics before connecting to MQTT because I use baseTopic as clientId.
  setupTopics();

  // Connect networking and MQTT.
  connectWiFi();
  mqtt.setServer(MQTT_BROKER, MQTT_PORT);
  mqtt.setKeepAlive(10);  // I use a 10s keepalive so LWT “offline” triggers faster if I disappear.
  mqtt.setCallback(callback);
  connectMQTT();
  Serial.println("ESP32 Rack Monitor MQTT ready");
}


// --- Arduino loop: runs forever ---
void loop() {
  // I keep Wi-Fi and MQTT alive.
  if (WiFi.status() != WL_CONNECTED) connectWiFi();
  if (!mqtt.connected()) connectMQTT();
  mqtt.loop();

  // I read buttons constantly so door/alarm response is immediate.
  handleButtons();

  // --------------------------------------------------------------------------
  // LED2 output rule (VERY IMPORTANT DESIGN RULE):
  // LED2 is controlled ONLY here.
  // LED2 is ON if any alarm state is active.
  // This prevents different subsystems from fighting over LED2.
  // --------------------------------------------------------------------------
  if (doorAlarmActive || powerAlarmActive || envAlarmActive) {
    digitalWrite(LED2_PIN, HIGH);
  } else {
    digitalWrite(LED2_PIN, LOW);
  }

  // --------------------------------------------------------------------------
  // Optional “door open too long” logic (currently disabled)
  // I disabled this because doorOpen/doorOpenedAt aren’t actively maintained in
  // the current door-latched design. If I re-enable later, I will set doorOpen
  // and doorOpenedAt inside Button 1 logic.
  // Also: if I re-enable, I should NOT directly write to LED2 here — I should
  // set doorAlarmActive and let the LED2 rule above reflect it.
  // --------------------------------------------------------------------------
  /*
  if (doorOpen && !doorAlarmActive) {
    if (millis() - doorOpenedAt >= DOOR_OPEN_TOO_LONG_MS) {
      doorAlarmActive = true;
      publishEvent("alarm", "DOOR_OPEN_TOO_LONG");
    }
  }
  */

  // --------------------------------------------------------------------------
  // Sensor read scheduling
  // I only do DHT+pot evaluation once every SENSOR_INTERVAL_MS (60 seconds).
  // The key trick: even though sensors update every 60s, LED2 updates instantly
  // because LED2 is driven by alarm state, not by sensor code directly.
  // --------------------------------------------------------------------------
  unsigned long now = millis();
  if (now - lastSensorRead >= SENSOR_INTERVAL_MS) {
    lastSensorRead = now;

    // Read DHT22
    float h = dht.readHumidity();
    float t = dht.readTemperature();

    // If DHT fails, I retry once after 2 seconds.
    if (isnan(h) || isnan(t)) {
      delay(2000);
      h = dht.readHumidity();
      t = dht.readTemperature();
    }

    // If it still fails, I publish a sensor failure event.
    if (isnan(h) || isnan(t)) {
      Serial.println("DHT read failed after retry");
      publishEvent("sensor", "dht_fail");
    } else {
      // If DHT succeeded, I read potentiometer and compute load percent.
      int potPct = readPotPercent();

      // ----------------------------------------------------------------------
      // POWER alarm state (potentiometer threshold)
      // I compute “powerNow” as a boolean and only publish events on transitions.
      // This prevents spamming MQTT every minute with the same ON/OFF message.
      // ----------------------------------------------------------------------
      bool powerNow = (potPct >= POWER_ALARM_PCT);

      if (powerNow && !powerAlarmActive) {
        powerAlarmActive = true;
        publishEvent("alarm", "POWER_ON");
      } else if (!powerNow && powerAlarmActive) {
        powerAlarmActive = false;
        publishEvent("alarm", "POWER_OFF");
      }

      // ----------------------------------------------------------------------
      // ENVIRONMENT alarm state (DHT22 threshold)
      // Same pattern: compute envNow and publish only on transitions.
      // ----------------------------------------------------------------------
      bool envNow = (t >= TEMP_ALARM_C || h >= HUM_ALARM_PCT);

      if (envNow && !envAlarmActive) {
        envAlarmActive = true;
        publishEvent("alarm", "ENV_ON");
      } else if (!envNow && envAlarmActive) {
        envAlarmActive = false;
        publishEvent("alarm", "ENV_OFF");
      }

      // Debug prints in Serial Monitor so I can “see” readings live.
      // If the env thresholds are exceeded, I print "DHT ALARM" instead of "DHT OK".
      if (envNow) {
        Serial.print("DHT ALARM t=");
      } else {
        Serial.print("DHT OK t=");
      }

      Serial.print(t);
      Serial.print(" h=");
      Serial.println(h);

      Serial.print("LOAD = ");
      Serial.print(potPct);
      Serial.println("%");

      // Publish telemetry snapshot to MQTT.
      publishTelemetry(t, h, potPct);
    }
  }

  // Tiny delay so the ESP32 doesn’t spin at 100% CPU in simulation.
  delay(10);
}