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

// ===== Simple System States =====
bool armed = true;        // always armed in this simple version
bool alarmActive = false; // alarm output state
bool testMode = false;    // test mode state

const float TEMP_ALARM_C = 45.0; // alarm threshold

// Button edge detect
bool lastTestBtn = LOW;
bool lastResetBtn = LOW;

unsigned long lastPrintMs = 0;

void setup() {
  Serial.begin(115200);

  pinMode(LED_GREEN, OUTPUT);
  pinMode(LED_RED, OUTPUT);

  // match wiring: pressed = HIGH
  pinMode(BTN_TEST, INPUT);
  pinMode(BTN_RESET, INPUT);

  analogReadResolution(12);
  dht.begin();

  Serial.println("Building Sprinkler Alarm (simple) started.");
}

void loop() {
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
  }
  lastTestBtn = testBtn;

  // BTN_RESET: clear test mode & alarm
  if (resetBtn == HIGH && lastResetBtn == LOW) {
    testMode = false;
    alarmActive = false;
    Serial.println("BTN2 pressed -> RESET (Alarm OFF, Test OFF)");
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

  // Red LED = Alarm or Test
  digitalWrite(LED_RED, alarmActive ? HIGH : LOW);

  // ---- Print every 2 seconds ----
  if (millis() - lastPrintMs >= 2000) {
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

  delay(10);
}
