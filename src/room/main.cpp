
// ============================================================================
// ESP32 Datacenter Node - Complete Sketch
// ============================================================================
// Features:
//   - DHT22 temperature & humidity monitoring
//   - Potentiometer smoke simulation
//   - 3 Status LEDs (Green/Yellow/Red)
//   - 2 Buttons (Technician ACK + Fire Reset with 10s hold)
//   - NTP time synchronization
//   - MQTT with LWT (Last Will Testament) and retained fire alerts
//   - Telemetry publishing
//   - Alert system with consecutive read confirmation
//   - Proper button debouncing
//   - Rack device integration via MQTT
// ============================================================================

// Libraries
#include <WiFi.h>
#include <PubSubClient.h>
#include "DHT.h"
#include "time.h"

// ============================================================================
// USER CONFIGURATION
// ============================================================================
const char* ssid     = "Wokwi-GUEST";
const char* password = "";

const char* mqtt_server = "broker.hivemq.com";
const uint16_t mqtt_port = 1883;
const char* mqtt_user = "";
const char* mqtt_pass = "";

// --- Device identity for topic names ---
const char* DC_PREFIX  = "dc";
const char* STUDENT_ID = "5047992u";
const char* USERNAME   = "Naren";
const char* DEVICE_TYPE= "esp32";

// MQTT Topics (room-level)
const char* topicTelemetry   = "dc/5047992u/Naren/esp32/telemetry";
const char* topicEvent       = "dc/5047992u/Naren/esp32/event";
const char* topicCommand     = "dc/5047992u/Naren/esp32/command";
const char* topicTechAlert   = "dc/5047992u/Naren/esp32/alert/Technician";
const char* topicFireAlert   = "dc/5047992u/Naren/esp32/alert/Fire";
const char* topicLWT         = "dc/5047992u/Naren/esp32/LWT";

// Rack-level topics (different owner - subscribe/publish selectively)
const char* rackTopicEvent   = "dc/2780093K/shaun/esp32/event";
const char* rackTopicStatus  = "dc/2780093K/shaun/esp32/status";
const char* rackCmdLed1      = "dc/2780093K/shaun/esp32/cmd/led1";

// ============================================================================
// PIN DEFINITIONS
// ============================================================================
#define DHTPIN      15
#define DHTTYPE     DHT22
#define POT_PIN     34
#define LED_GREEN   12
#define LED_YELLOW  14
#define LED_RED     13
#define BTN_ACK     27    // Technician reset (short press)
#define BTN_FIRE    26    // Fire reset (hold 10s)

// ============================================================================
// THRESHOLD CONFIGURATION
// ============================================================================
// Technician alert thresholds
const float TECH_TEMP_LOW   = 18.0;   // C
const float TECH_TEMP_HIGH  = 27.0;   // C
const float TECH_HUM_LOW    = 40.0;   // %
const float TECH_HUM_HIGH   = 60.0;   // %

// Fire threshold
const float FIRE_THRESH     = 57.0;   // C
const float FIRE_HYSTERESIS = 2.0;    // C (clear at FIRE_THRESH - FIRE_HYSTERESIS)

// Smoke detection threshold (potentiometer percentage)
const int SMOKE_THRESH_PERCENT = 50;

// ============================================================================
// TIMING CONFIGURATION
// ============================================================================
const unsigned long TECH_REPEAT_MS      = 5UL * 60UL * 1000UL;  // 5 minutes between repeat alerts
const unsigned long DHT_READ_INTERVAL   = 20000UL;              // 20 seconds between sensor reads
const unsigned long TELEMETRY_INTERVAL  = 60000UL;              // 60 seconds between telemetry
const unsigned long DEBOUNCE_MS         = 50UL;                 // 50ms button debounce
const unsigned long FIRE_HOLD_MS        = 10000UL;              // 10 seconds hold to reset fire
const unsigned long WIFI_TIMEOUT_MS     = 15000UL;              // 15 seconds WiFi connection timeout

// Consecutive reads required to confirm alert condition
const int CONSECUTIVE_REQUIRED = 2;

// ============================================================================
// GLOBAL OBJECTS
// ============================================================================
DHT dht(DHTPIN, DHTTYPE);
WiFiClient espClient;
PubSubClient client(espClient);

// ============================================================================
// STATE VARIABLES
// ============================================================================
// Alert states
bool techAlertLatched = false;
bool fireAlarmActive = false;
unsigned long lastTechAlertTime = 0;

// Timing trackers
unsigned long lastDhtRead = 0;
unsigned long lastTelemetry = 0;

// Consecutive read counters for hysteresis
int consecutiveOutCount = 0;
int consecutiveInCount = 0;

// Last known sensor values (for button handling between reads)
float lastKnownTemp = 22.0;
float lastKnownHum = 50.0;
int lastKnownPot = 0;

// Button debounce state - ACK button
unsigned long lastBtnAckChange = 0;
int lastBtnAckState = LOW;
int currentBtnAckState = LOW;

// Button debounce state - Fire button
unsigned long lastBtnFireChange = 0;
int lastBtnFireState = LOW;
int currentBtnFireState = LOW;
unsigned long firePressStart = 0;
bool fireHoldInProgress = false;

// MQTT LWT payloads
const char* LWT_OFFLINE = "{\"status\":\"offline\"}";
const char* LWT_ONLINE  = "{\"status\":\"online\"}";

// ============================================================================
// UTILITY FUNCTIONS
// ============================================================================

// Forward declarations for functions used before their definitions
void publishTelemetry(float temp, float hum, int potVal);
void clearFireAlarmRetained(float temp);

/**
 * Publish JSON payload to MQTT topic
 */
void publishJson(const char* topic, const char* payload, bool retain = false) {
  if (client.connected()) {
    client.publish(topic, payload, retain);
    Serial.print("[MQTT TX] ");
    Serial.print(topic);
    Serial.print(" : ");
    Serial.println(payload);
  } else {
    Serial.println("[MQTT] Not connected, message dropped");
  }
}

/**
 * Publish event with name, value, and timestamp
 */
void publishEvent(const char* name, int value) {
  char payload[128];
  time_t t = time(NULL);
  snprintf(payload, sizeof(payload),
           "{\"event\":\"%s\",\"value\":%d,\"ts\":%ld}",
           name, value, t);
  publishJson(topicEvent, payload, false);
}

/**
 * Get current timestamp string for logging
 */
void printTimestamp() {
  time_t t = time(NULL);
  struct tm* tm_info = localtime(&t);
  char buffer[26];
  strftime(buffer, 26, "%Y-%m-%d %H:%M:%S", tm_info);
  Serial.print("[");
  Serial.print(buffer);
  Serial.print("] ");
}

// ============================================================================
// LED CONTROL FUNCTIONS
// ============================================================================

/**
 * Set LED states based on current alarm conditions
 */
void updateLEDStates() {
  if (fireAlarmActive) {
    // Fire alarm: Red blinks, Yellow on, Green off
    // (Blinking handled separately in loop)
    digitalWrite(LED_YELLOW, HIGH);
    digitalWrite(LED_GREEN, LOW);
  } else if (techAlertLatched) {
    // Tech alert: Yellow on, Red on, Green off
    digitalWrite(LED_YELLOW, HIGH);
    digitalWrite(LED_RED, HIGH);
    digitalWrite(LED_GREEN, LOW);
  } else {
    // Normal: Green on, others off
    digitalWrite(LED_GREEN, HIGH);
    digitalWrite(LED_YELLOW, LOW);
    digitalWrite(LED_RED, LOW);
  }
}

/**
 * Handle fire alarm LED blinking
 */
void handleFireLEDBlinking() {
  if (fireAlarmActive) {
    unsigned long phase = millis() % 500;
    digitalWrite(LED_RED, phase < 250 ? HIGH : LOW);
  }
}

// ============================================================================
// MQTT CALLBACK
// ============================================================================

/**
 * MQTT message callback handler
 */
void mqttCallback(char* topic, byte* payload, unsigned int length) {
  String msg;
  for (unsigned int i = 0; i < length; i++) {
    msg += (char)payload[i];
  }
  msg.trim();

  printTimestamp();
  Serial.print("[MQTT RX] ");
  Serial.print(topic);
  Serial.print(" : ");
  Serial.println(msg);

  // Rack-level status + alarms
  if (String(topic) == rackTopicStatus) {
    int online = (msg.indexOf("online") >= 0) ? 1 : 0;
    publishEvent("rack_status", online);
    return;
  }

  if (String(topic) == rackTopicEvent) {
    if (msg.indexOf("DOOR_ON") >= 0) {
      publishEvent("rack_door_alarm", 1);
    } else if (msg.indexOf("DOOR_OFF") >= 0) {
      publishEvent("rack_door_alarm", 0);
    } else if (msg.indexOf("ENV_ON") >= 0) {
      publishEvent("rack_env_alarm", 1);
    } else if (msg.indexOf("ENV_OFF") >= 0) {
      publishEvent("rack_env_alarm", 0);
    } else if (msg.indexOf("POWER_ON") >= 0) {
      publishEvent("rack_power_alarm", 1);
    } else if (msg.indexOf("POWER_OFF") >= 0) {
      publishEvent("rack_power_alarm", 0);
    }
    return;
  }

  // LED control commands
  // Format: {"led":"green","state":1} or {"led":"green","state":0}
  if (msg.indexOf("\"led\"") >= 0) {
    bool state = (msg.indexOf("\"state\":1") >= 0);

    if (msg.indexOf("\"green\"") >= 0) {
      digitalWrite(LED_GREEN, state ? HIGH : LOW);
      publishEvent("led_green_set", state ? 1 : 0);
    }
    if (msg.indexOf("\"yellow\"") >= 0) {
      digitalWrite(LED_YELLOW, state ? HIGH : LOW);
      publishEvent("led_yellow_set", state ? 1 : 0);
    }
    if (msg.indexOf("\"red\"") >= 0) {
      digitalWrite(LED_RED, state ? HIGH : LOW);
      publishEvent("led_red_set", state ? 1 : 0);
    }
  }

  // Remote fire clear command
  // Format: {"clear_fire":1}
  if (msg.indexOf("\"clear_fire\"") >= 0 && msg.indexOf(":1") >= 0) {
    if (!fireAlarmActive) {
      publishEvent("remote_fire_clear_no_alarm", 0);
    } else {
      // Only allow remote clear if temp below safe threshold
      float curTemp = dht.readTemperature();
      if (!isnan(curTemp) && curTemp <= (FIRE_THRESH - FIRE_HYSTERESIS)) {
        clearFireAlarmRetained(curTemp);
        publishEvent("remote_fire_clear_success", (int)curTemp);
      } else {
        publishEvent("remote_fire_clear_failed_temp", isnan(curTemp) ? -999 : (int)curTemp);
      }
    }
  }

  // Request immediate telemetry
  // Format: {"request_telemetry":1}
  if (msg.indexOf("\"request_telemetry\"") >= 0 && msg.indexOf(":1") >= 0) {
    publishTelemetry(lastKnownTemp, lastKnownHum, lastKnownPot);
    publishEvent("telemetry_requested", 1);
  }

  // Request status
  // Format: {"request_status":1}
  if (msg.indexOf("\"request_status\"") >= 0 && msg.indexOf(":1") >= 0) {
    char payload[256];
    time_t t = time(NULL);
    snprintf(payload, sizeof(payload),
             "{\"tech_latched\":%d,\"fire_active\":%d,\"wifi_rssi\":%d,\"uptime\":%lu,\"ts\":%ld}",
             techAlertLatched ? 1 : 0,
             fireAlarmActive ? 1 : 0,
             WiFi.RSSI(),
             millis() / 1000,
             t);
    publishJson(topicEvent, payload, false);
  }
}

// ============================================================================
// WIFI FUNCTIONS
// ============================================================================

/**
 * Setup WiFi connection
 */
void setupWiFi() {
  Serial.println();
  Serial.print("[WiFi] Connecting to ");
  Serial.println(ssid);

  WiFi.mode(WIFI_STA);
  WiFi.begin(ssid, password);

  unsigned long start = millis();
  while (WiFi.status() != WL_CONNECTED && (millis() - start) < WIFI_TIMEOUT_MS) {
    delay(200);
    Serial.print(".");
  }

  Serial.println();
  if (WiFi.status() == WL_CONNECTED) {
    Serial.print("[WiFi] Connected! IP: ");
    Serial.println(WiFi.localIP());
    Serial.print("[WiFi] RSSI: ");
    Serial.print(WiFi.RSSI());
    Serial.println(" dBm");
  } else {
    Serial.println("[WiFi] Connection failed!");
  }
}

// ============================================================================
// MQTT FUNCTIONS
// ============================================================================

/**
 * Connect/reconnect to MQTT broker
 */
void reconnectMQTT() {
  int attempts = 0;
  const int maxAttempts = 3;

  while (!client.connected() && attempts < maxAttempts) {
    attempts++;
    Serial.print("[MQTT] Attempting connection (");
    Serial.print(attempts);
    Serial.print("/");
    Serial.print(maxAttempts);
    Serial.println(")...");

    // Generate unique client ID
    String clientId = "esp32-datacenter-";
    clientId += String(random(0xffff), HEX);

    // Connect with LWT (Last Will Testament)
    if (client.connect(clientId.c_str(), mqtt_user, mqtt_pass,
                       topicLWT, 0, true, LWT_OFFLINE)) {
      Serial.println("[MQTT] Connected!");

      // Publish online status (retained)
      client.publish(topicLWT, LWT_ONLINE, true);

      // Subscribe to command topic
      client.subscribe(topicCommand);
      Serial.print("[MQTT] Subscribed to: ");
      Serial.println(topicCommand);

      // Subscribe to rack status/event topics
      client.subscribe(rackTopicStatus);
      client.subscribe(rackTopicEvent);
      Serial.print("[MQTT] Subscribed to: ");
      Serial.println(rackTopicStatus);
      Serial.print("[MQTT] Subscribed to: ");
      Serial.println(rackTopicEvent);

      // Publish connection event
      publishEvent("mqtt_connected", 1);

    } else {
      Serial.print("[MQTT] Failed, rc=");
      Serial.print(client.state());
      Serial.println(" - retrying in 2 seconds");
      delay(2000);
    }
  }
}

// ============================================================================
// TELEMETRY FUNCTIONS
// ============================================================================

/**
 * Publish telemetry data
 */
void publishTelemetry(float temp, float hum, int potVal) {
  char payload[300];
  time_t t = time(NULL);

  int smokePercent = map(potVal, 0, 4095, 0, 100);

  snprintf(payload, sizeof(payload),
           "{\"temp\":%.2f,\"hum\":%.2f,\"pot\":%d,\"smoke_pct\":%d,"
           "\"tech_latched\":%d,\"fire_active\":%d,\"wifi_rssi\":%d,\"ts\":%ld}",
           temp, hum, potVal, smokePercent,
           techAlertLatched ? 1 : 0,
           fireAlarmActive ? 1 : 0,
           WiFi.RSSI(),
           t);

  publishJson(topicTelemetry, payload, false);
}

// ============================================================================
// ALERT FUNCTIONS
// ============================================================================

/**
 * Publish technician alert
 */
void publishTechAlert(float temp, float hum, const char* reason) {
  unsigned long now = millis();

  // Rate limit repeat alerts
  if (techAlertLatched && (now - lastTechAlertTime) < TECH_REPEAT_MS) {
    return;
  }

  lastTechAlertTime = now;
  techAlertLatched = true;

  char payload[256];
  time_t t = time(NULL);
  snprintf(payload, sizeof(payload),
           "{\"level\":\"warning\",\"temp\":%.2f,\"hum\":%.2f,\"reason\":\"%s\",\"ts\":%ld}",
           temp, hum, reason, t);

  publishJson(topicTechAlert, payload, false);

  printTimestamp();
  Serial.print("[ALERT] Technician alert triggered: ");
  Serial.println(reason);

  updateLEDStates();
}

/**
 * Clear technician alert
 */
void publishTechCleared(float temp, float hum) {
  char payload[256];
  time_t t = time(NULL);
  snprintf(payload, sizeof(payload),
           "{\"level\":\"cleared\",\"temp\":%.2f,\"hum\":%.2f,\"reason\":\"tech_acknowledged\",\"ts\":%ld}",
           temp, hum, t);

  publishJson(topicTechAlert, payload, false);

  techAlertLatched = false;
  consecutiveOutCount = 0;
  consecutiveInCount = 0;

  printTimestamp();
  Serial.println("[ALERT] Technician alert cleared");

  updateLEDStates();
}

/**
 * Publish fire alarm (RETAINED)
 */
void publishFireAlarm(float temp) {
  fireAlarmActive = true;

  char payload[200];
  time_t t = time(NULL);
  snprintf(payload, sizeof(payload),
           "{\"level\":\"critical\",\"temp\":%.2f,\"reason\":\"fire_detected\",\"ts\":%ld}",
           temp, t);

  publishJson(topicFireAlert, payload, true);  // RETAINED

  // Signal rack cooling indicator
  publishJson(rackCmdLed1, "ON", false);

  printTimestamp();
  Serial.println("[ALERT] *** FIRE ALARM ACTIVATED ***");

  // Fire alarm LED state handled in loop (blinking)
}

/**
 * Clear fire alarm and overwrite retained message
 */
void clearFireAlarmRetained(float temp) {
  fireAlarmActive = false;

  char payload[200];
  time_t t = time(NULL);
  snprintf(payload, sizeof(payload),
           "{\"level\":\"cleared\",\"temp\":%.2f,\"reason\":\"fire_cleared\",\"ts\":%ld}",
           temp, t);

  publishJson(topicFireAlert, payload, true);  // Overwrite retained

  // Clear rack cooling indicator
  publishJson(rackCmdLed1, "OFF", false);

  printTimestamp();
  Serial.println("[ALERT] Fire alarm cleared");

  updateLEDStates();
}

// ============================================================================
// BUTTON HANDLING
// ============================================================================

/**
 * Handle button inputs with debouncing
 */
void handleButtons(unsigned long now) {
  // -------------------------------------------------------------------------
  // Technician ACK Button (short press)
  // -------------------------------------------------------------------------
  int readingAck = digitalRead(BTN_ACK);

  if (readingAck != lastBtnAckState) {
    lastBtnAckChange = now;
  }

  if ((now - lastBtnAckChange) > DEBOUNCE_MS) {
    if (readingAck != currentBtnAckState) {
      currentBtnAckState = readingAck;

      // Button pressed (rising edge)
      if (currentBtnAckState == HIGH) {
        printTimestamp();
        Serial.println("[BUTTON] ACK button pressed");
        publishEvent("button_ack_pressed", 1);

        // Attempt to clear tech alert
        if (fireAlarmActive) {
          publishEvent("ack_failed_fire_active", 1);
          printTimestamp();
          Serial.println("[BUTTON] ACK failed: fire alarm active");
        } else if (!techAlertLatched) {
          publishEvent("ack_failed_no_alert", 1);
          printTimestamp();
          Serial.println("[BUTTON] ACK failed: no tech alert active");
        } else {
          // Check if conditions are back in range
          bool tempOk = (lastKnownTemp >= TECH_TEMP_LOW && lastKnownTemp <= TECH_TEMP_HIGH);
          bool humOk = (lastKnownHum >= TECH_HUM_LOW && lastKnownHum <= TECH_HUM_HIGH);
          int smokePercent = map(lastKnownPot, 0, 4095, 0, 100);
          bool smokeOk = (smokePercent < SMOKE_THRESH_PERCENT);

          if (tempOk && humOk && smokeOk) {
            publishTechCleared(lastKnownTemp, lastKnownHum);
          } else {
            publishEvent("ack_failed_conditions", 1);
            printTimestamp();
            Serial.print("[BUTTON] ACK failed: conditions not normal (T:");
            Serial.print(tempOk ? "OK" : "BAD");
            Serial.print(" H:");
            Serial.print(humOk ? "OK" : "BAD");
            Serial.print(" S:");
            Serial.print(smokeOk ? "OK" : "BAD");
            Serial.println(")");
          }
        }
      }
    }
  }
  lastBtnAckState = readingAck;

  // -------------------------------------------------------------------------
  // Fire Reset Button (10 second hold)
  // -------------------------------------------------------------------------
  int readingFire = digitalRead(BTN_FIRE);

  if (readingFire != lastBtnFireState) {
    lastBtnFireChange = now;
  }

  if ((now - lastBtnFireChange) > DEBOUNCE_MS) {
    if (readingFire != currentBtnFireState) {
      currentBtnFireState = readingFire;

      // Button pressed (rising edge)
      if (currentBtnFireState == HIGH) {
        firePressStart = now;
        fireHoldInProgress = true;
        printTimestamp();
        Serial.println("[BUTTON] Fire reset button pressed - hold for 10 seconds");
        publishEvent("fire_button_pressed", 1);
      }
      // Button released (falling edge)
      else if (currentBtnFireState == LOW) {
        if (fireHoldInProgress) {
          unsigned long holdDuration = now - firePressStart;
          printTimestamp();
          Serial.print("[BUTTON] Fire reset button released after ");
          Serial.print(holdDuration / 1000.0, 1);
          Serial.println(" seconds");
          publishEvent("fire_button_released", (int)(holdDuration / 1000));
        }
        fireHoldInProgress = false;
        firePressStart = 0;
      }
    }
  }
  lastBtnFireState = readingFire;

  // Check if fire button held for required duration
  if (fireHoldInProgress && (now - firePressStart >= FIRE_HOLD_MS)) {
    fireHoldInProgress = false;

    printTimestamp();
    Serial.println("[BUTTON] Fire reset hold complete - attempting reset");

    if (!fireAlarmActive) {
      publishEvent("fire_reset_no_alarm", 1);
      Serial.println("[BUTTON] Fire reset failed: no fire alarm active");
    } else {
      // Check if temperature is safe
      float safeTemp = FIRE_THRESH - FIRE_HYSTERESIS;
      if (lastKnownTemp <= safeTemp) {
        clearFireAlarmRetained(lastKnownTemp);
        publishEvent("fire_reset_success", 1);
      } else {
        publishEvent("fire_reset_failed_temp", (int)lastKnownTemp);
        printTimestamp();
        Serial.print("[BUTTON] Fire reset failed: temp ");
        Serial.print(lastKnownTemp);
        Serial.print("C > safe threshold ");
        Serial.print(safeTemp);
        Serial.println("C");
      }
    }
  }

  // Visual feedback: show hold progress on Serial
  if (fireHoldInProgress && fireAlarmActive) {
    static unsigned long lastProgressPrint = 0;
    if (now - lastProgressPrint >= 1000) {
      lastProgressPrint = now;
      unsigned long elapsed = (now - firePressStart) / 1000;
      Serial.print("[BUTTON] Fire reset hold: ");
      Serial.print(elapsed);
      Serial.print("/");
      Serial.print(FIRE_HOLD_MS / 1000);
      Serial.println(" seconds");
    }
  }
}

// ============================================================================
// ALERT EVALUATION WITH HYSTERESIS
// ============================================================================

/**
 * Check sensor readings and evaluate alert conditions
 */
void checkAlertsWithHysteresis(float temp, float hum, int potVal) {
  int smokePercent = map(potVal, 0, 4095, 0, 100);

  // -------------------------------------------------------------------------
  // Fire check (highest priority)
  // -------------------------------------------------------------------------
  if (!fireAlarmActive && temp > FIRE_THRESH) {
    publishFireAlarm(temp);
    return;  // Fire takes priority
  }

  // If fire is active, suppress tech alert evaluation
  if (fireAlarmActive) {
    return;
  }

  // -------------------------------------------------------------------------
  // Technician alert evaluation
  // -------------------------------------------------------------------------
  bool tempLow  = (temp < TECH_TEMP_LOW);
  bool tempHigh = (temp > TECH_TEMP_HIGH);
  bool humLow   = (hum < TECH_HUM_LOW);
  bool humHigh  = (hum > TECH_HUM_HIGH);
  bool smokeDetected = (smokePercent >= SMOKE_THRESH_PERCENT);

  bool outOfRange = tempLow || tempHigh || humLow || humHigh || smokeDetected;

  if (outOfRange) {
    consecutiveOutCount++;
    consecutiveInCount = 0;
  } else {
    consecutiveInCount++;
    consecutiveOutCount = 0;
  }

  // Trigger alert after consecutive out-of-range readings
  if (consecutiveOutCount >= CONSECUTIVE_REQUIRED && !techAlertLatched) {
    // Determine the reason for the alert
    const char* reason;
    if (smokeDetected) {
      reason = "smoke_detected";
    } else if (tempLow) {
      reason = "temp_low";
    } else if (tempHigh) {
      reason = "temp_high";
    } else if (humLow) {
      reason = "humidity_low";
    } else if (humHigh) {
      reason = "humidity_high";
    } else {
      reason = "unknown";
    }

    publishTechAlert(temp, hum, reason);
  }

  // Note: Tech alert is NOT auto-cleared
  // Technician must press ACK button when conditions return to normal
}

// ============================================================================
// SETUP
// ============================================================================

void setup() {
  Serial.begin(115200);
  delay(100);

  Serial.println();
  Serial.println("============================================");
  Serial.println("  ESP32 Data Center Room Monitoring System");
  Serial.println("  Version 1.0");
  Serial.println("============================================");
  Serial.println();

  // Initialize pins
  pinMode(LED_GREEN, OUTPUT);
  pinMode(LED_YELLOW, OUTPUT);
  pinMode(LED_RED, OUTPUT);
  pinMode(BTN_ACK, INPUT_PULLDOWN);    // Use internal pull-down
  pinMode(BTN_FIRE, INPUT_PULLDOWN);   // Use internal pull-down

  // Startup LED test sequence
  Serial.println("[INIT] LED test sequence...");
  digitalWrite(LED_RED, HIGH);
  delay(200);
  digitalWrite(LED_RED, LOW);
  digitalWrite(LED_YELLOW, HIGH);
  delay(200);
  digitalWrite(LED_YELLOW, LOW);
  digitalWrite(LED_GREEN, HIGH);
  delay(200);

  // Initialize DHT sensor
  Serial.println("[INIT] Starting DHT22 sensor...");
  dht.begin();

  // Connect to WiFi
  setupWiFi();

  // Configure MQTT
  client.setServer(mqtt_server, mqtt_port);
  client.setCallback(mqttCallback);
  client.setBufferSize(512);  // Increase buffer for larger messages

  // Configure NTP time
  Serial.println("[INIT] Configuring NTP...");
  configTime(8 * 3600, 0, "pool.ntp.org", "time.google.com");

  // Wait for time sync
  Serial.print("[INIT] Waiting for NTP sync");
  time_t now = time(nullptr);
  int ntpAttempts = 0;
  while (now < 8 * 3600 * 2 && ntpAttempts < 20) {
    delay(500);
    Serial.print(".");
    now = time(nullptr);
    ntpAttempts++;
  }
  Serial.println();

  if (now > 8 * 3600 * 2) {
    struct tm timeinfo;
    localtime_r(&now, &timeinfo);
    Serial.print("[INIT] NTP synced: ");
    Serial.println(asctime(&timeinfo));
  } else {
    Serial.println("[INIT] NTP sync failed, continuing anyway");
  }

  // Set initial LED state
  updateLEDStates();

  // Initialize timing
  lastDhtRead = millis() - DHT_READ_INTERVAL;  // Force immediate first read
  lastTelemetry = millis() - TELEMETRY_INTERVAL;

  Serial.println();
  Serial.println("[INIT] Setup complete!");
  Serial.println("============================================");
  Serial.print("[CONFIG] Tech temp range: ");
  Serial.print(TECH_TEMP_LOW);
  Serial.print("C - ");
  Serial.print(TECH_TEMP_HIGH);
  Serial.println("C");
  Serial.print("[CONFIG] Tech humidity range: ");
  Serial.print(TECH_HUM_LOW);
  Serial.print("% - ");
  Serial.print(TECH_HUM_HIGH);
  Serial.println("%");
  Serial.print("[CONFIG] Fire threshold: ");
  Serial.print(FIRE_THRESH);
  Serial.println("C");
  Serial.print("[CONFIG] Smoke threshold: ");
  Serial.print(SMOKE_THRESH_PERCENT);
  Serial.println("%");
  Serial.println("============================================");
  Serial.println();
}

// ============================================================================
// MAIN LOOP
// ============================================================================

void loop() {
  unsigned long now = millis();

  // -------------------------------------------------------------------------
  // WiFi connection check
  // -------------------------------------------------------------------------
  if (WiFi.status() != WL_CONNECTED) {
    Serial.println("[WiFi] Connection lost, reconnecting...");
    setupWiFi();
  }

  // -------------------------------------------------------------------------
  // MQTT connection check
  // -------------------------------------------------------------------------
  if (!client.connected()) {
    reconnectMQTT();
  }
  client.loop();

  // -------------------------------------------------------------------------
  // Button handling (every loop iteration for responsive hold detection)
  // -------------------------------------------------------------------------
  handleButtons(now);

  // -------------------------------------------------------------------------
  // Sensor reading and alert evaluation (periodic)
  // -------------------------------------------------------------------------
  if (now - lastDhtRead >= DHT_READ_INTERVAL) {
    lastDhtRead = now;

    float temp = dht.readTemperature();
    float hum = dht.readHumidity();
    int pot = analogRead(POT_PIN);

    if (!isnan(temp) && !isnan(hum)) {
      // Store for button handling between reads
      lastKnownTemp = temp;
      lastKnownHum = hum;
      lastKnownPot = pot;

      // Debug output
      int smokePercent = map(pot, 0, 4095, 0, 100);
      printTimestamp();
      Serial.print("[SENSOR] T:");
      Serial.print(temp, 1);
      Serial.print("C H:");
      Serial.print(hum, 1);
      Serial.print("% Smoke:");
      Serial.print(smokePercent);
      Serial.println("%");

      // Evaluate alert conditions
      checkAlertsWithHysteresis(temp, hum, pot);

      // Publish telemetry at configured interval
      if (now - lastTelemetry >= TELEMETRY_INTERVAL) {
        publishTelemetry(temp, hum, pot);
        lastTelemetry = now;
      }

    } else {
      printTimestamp();
      Serial.println("[SENSOR] DHT22 read failed!");
      publishEvent("dht_read_error", 1);
    }
  }

  // -------------------------------------------------------------------------
  // Fire alarm LED blinking
  // -------------------------------------------------------------------------
  handleFireLEDBlinking();

  // Small delay to prevent CPU hogging
  delay(10);
}
