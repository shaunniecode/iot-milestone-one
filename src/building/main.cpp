// ============================================================================
// Building Sprinkler Alarm (Simple)
// ============================================================================
// - DHT22 temp/humidity
// - Potentiometer simulates tank level (%)
// - Green LED = system armed
// - Red LED = alarm/test
// - Button 1 = test mode (alarm ON)
// - Button 2 = reset (alarm OFF)
// ============================================================================

#include <Arduino.h>
#include <WiFi.h>
#include <PubSubClient.h>
#include <DHT.h>

// ===== Pins (from diagram.building.json) =====
#define LED_GREEN 12
#define LED_RED   13

#define BTN_TEST  14   // wired to 3V3 -> pressed = HIGH
#define BTN_RESET 27   // wired to 3V3 + pulldown -> pressed = HIGH

#define POT_PIN   34
#define DHT_PIN   4
#define DHT_TYPE  DHT22

DHT dht(DHT_PIN, DHT_TYPE);

// ===== Network / MQTT =====
const char* WIFI_SSID = "Wokwi-GUEST";
const char* WIFI_PASS = "";

const char* MQTT_BROKER = "broker.hivemq.com";
const uint16_t MQTT_PORT = 1883;

const char* DC_PREFIX  = "dc";
const char* STUDENT_ID = "2780093K";
const char* USERNAME   = "shaun";
const char* DEVICE_TYPE= "building";

String baseTopic;
String statusTopic;
String eventTopic;
String telemetryTopic;
String cmdTopic;

WiFiClient espClient;
PubSubClient mqtt(espClient);

// ===== Simple System States =====
bool armed = true;        // always armed in this simple version
bool alarmActive = false; // alarm output state
bool testMode = false;    // test mode state

const float TEMP_ALARM_C = 45.0; // alarm threshold
const unsigned long TELEMETRY_INTERVAL_MS = 60000UL;

// Button edge detect
bool lastTestBtn = LOW;
bool lastResetBtn = LOW;

unsigned long lastPrintMs = 0;
unsigned long lastTelemetryMs = 0;
bool lastAlarmActive = false;

// ===== MQTT helpers =====
void setupTopics() {
  baseTopic = String(DC_PREFIX) + "/" + STUDENT_ID + "/" + USERNAME + "/" + DEVICE_TYPE;
  statusTopic = baseTopic + "/status";
  eventTopic = baseTopic + "/event";
  telemetryTopic = baseTopic + "/telemetry";
  cmdTopic = baseTopic + "/cmd";
}

void publishEvent(const char* name, const char* value) {
  char payload[192];
  snprintf(payload, sizeof(payload),
           "{\"event\":\"%s\",\"value\":\"%s\",\"ts\":%lu}",
           name, value, millis());
  mqtt.publish(eventTopic.c_str(), payload);
}

void publishTelemetry(float tempC, float hum, int potPct) {
  char payload[220];
  snprintf(payload, sizeof(payload),
           "{\"temp_c\":%.2f,\"humidity_pct\":%.2f,\"tank_pct\":%d,"
           "\"armed\":%d,\"test_mode\":%d,\"alarm\":%d,\"ts\":%lu}",
           tempC, hum, potPct,
           armed ? 1 : 0, testMode ? 1 : 0, alarmActive ? 1 : 0, millis());
  mqtt.publish(telemetryTopic.c_str(), payload);
}

void connectWiFi() {
  WiFi.mode(WIFI_STA);
  WiFi.begin(WIFI_SSID, WIFI_PASS);
  while (WiFi.status() != WL_CONNECTED) {
    delay(300);
  }
}

void connectMQTT() {
  while (!mqtt.connected()) {
    String clientId = baseTopic + "-client";

    const char* willTopic = statusTopic.c_str();
    const uint8_t willQos = 1;
    const bool willRetain = true;
    const char* willMsg = "offline";

    if (mqtt.connect(clientId.c_str(), willTopic, willQos, willRetain, willMsg)) {
      mqtt.publish(statusTopic.c_str(), "online", true);
      mqtt.subscribe(cmdTopic.c_str());
      publishEvent("system", "online");
    } else {
      delay(1000);
    }
  }
}

void callback(char* topic, byte* payload, unsigned int length) {
  String msg;
  for (unsigned int i = 0; i < length; i++) msg += (char)payload[i];
  msg.trim();

  if (String(topic) != cmdTopic) return;

  if (msg.equalsIgnoreCase("TEST")) {
    testMode = true;
    alarmActive = true;
    publishEvent("test", "ON");
  } else if (msg.equalsIgnoreCase("RESET")) {
    testMode = false;
    alarmActive = false;
    publishEvent("reset", "PRESSED");
  }
}

void setup() {
  Serial.begin(115200);

  pinMode(LED_GREEN, OUTPUT);
  pinMode(LED_RED, OUTPUT);

  // match wiring: pressed = HIGH
  pinMode(BTN_TEST, INPUT);
  pinMode(BTN_RESET, INPUT);

  analogReadResolution(12);
  dht.begin();

  setupTopics();
  connectWiFi();
  mqtt.setServer(MQTT_BROKER, MQTT_PORT);
  mqtt.setKeepAlive(10);
  mqtt.setCallback(callback);
  connectMQTT();

  Serial.println("Building Sprinkler Alarm (simple) started.");
}

void loop() {
  if (WiFi.status() != WL_CONNECTED) connectWiFi();
  if (!mqtt.connected()) connectMQTT();
  mqtt.loop();

  // Green LED = Armed status
  digitalWrite(LED_GREEN, armed ? HIGH : LOW);

  // ---- Read buttons ----
  bool testBtn = digitalRead(BTN_TEST);
  bool resetBtn = digitalRead(BTN_RESET);

  // BTN_TEST: enter test mode (alarm ON)
  if (testBtn == HIGH && lastTestBtn == LOW) {
    testMode = true;
    alarmActive = true;
    Serial.println("BTN1 pressed -> TEST MODE ON (Alarm ON)");
    publishEvent("test", "ON");
  }
  lastTestBtn = testBtn;

  // BTN_RESET: clear test mode & alarm
  if (resetBtn == HIGH && lastResetBtn == LOW) {
    testMode = false;
    alarmActive = false;
    Serial.println("BTN2 pressed -> RESET (Alarm OFF, Test OFF)");
    publishEvent("reset", "PRESSED");
  }
  lastResetBtn = resetBtn;

  // ---- Read sensors ----
  float tempC = dht.readTemperature();
  float hum = dht.readHumidity();

  int potRaw = analogRead(POT_PIN);        // 0..4095
  int potPct = map(potRaw, 0, 4095, 0, 100);

  // ---- Alarm logic (only if not test mode) ----
  if (!testMode && armed && !isnan(tempC)) {
    if (tempC >= TEMP_ALARM_C) {
      alarmActive = true;
    } else {
      alarmActive = false;
    }
  }

  if (alarmActive != lastAlarmActive) {
    publishEvent("alarm", alarmActive ? "ON" : "OFF");
    lastAlarmActive = alarmActive;
  }

  if (isnan(tempC) || isnan(hum)) {
    publishEvent("sensor", "dht_fail");
  }

  // Red LED = Alarm or Test
  digitalWrite(LED_RED, alarmActive ? HIGH : LOW);

  // ---- Print at the same interval as telemetry ----
  if (millis() - lastPrintMs >= TELEMETRY_INTERVAL_MS) {
    lastPrintMs = millis();

    Serial.println("---- Building Sprinkler Status ----");
    Serial.print("Tank Level (Pot): ");
    Serial.print(potPct);
    Serial.println("%");
    Serial.print("Temperature: ");
    Serial.print(tempC);
    Serial.println(" C");
    Serial.print("Humidity: ");
    Serial.print(hum);
    Serial.println(" %");
    Serial.print("Test Mode: ");
    Serial.println(testMode ? "ON" : "OFF");
    Serial.print("Alarm: ");
    Serial.println(alarmActive ? "ON" : "OFF");
    Serial.println("----------------------------------");
  }

  if (millis() - lastTelemetryMs >= TELEMETRY_INTERVAL_MS) {
    lastTelemetryMs = millis();
    if (!isnan(tempC) && !isnan(hum)) {
      publishTelemetry(tempC, hum, potPct);
    }
  }

  delay(10);
}
