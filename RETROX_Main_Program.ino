/*
  RETROX - ESP32 Sensor Worker / Telemetry Node
  Board: ESP32 Dev Module (ESP32-WROOM-32)

  Arduino IDE libraries:
    - Adafruit MLX90614 Library
    - LiquidCrystal_I2C (compatible library)

  The ESP32 serves GET /api/data as JSON with permissive CORS so the local
  RETROX_Dashboard_ESP32.html can poll it from the same Wi-Fi network.

  IMPORTANT:
  - Sensor pin assignments below are example wiring; verify your modules.
  - ACS712 current conversion must be calibrated for your exact module and ADC.
  - A raw analog vibration sensor is NOT a calibrated mm/s measurement. The
    vibrationMmS value is null until you calibrate a suitable sensor.
  - Do not connect mains voltage or industrial machine circuits directly to GPIO.
    Use properly isolated, protected sensor interfaces and an electrician.
*/

#include <WiFi.h>
#include <WebServer.h>
#include <Wire.h>
#include <LiquidCrystal_I2C.h>
#include <Adafruit_MLX90614.h>
#include <math.h>

// -------- Wi-Fi configuration --------
// Fill these in to join your router. If connection fails, ESP32 creates its AP.
const char* WIFI_SSID = "Guard";
const char* WIFI_PASSWORD = "guard123";
const char* AP_SSID = "RETROX-ESP32";
const char* AP_PASSWORD = "retroX123"; // At least 8 characters

// -------- GPIO mapping --------
constexpr uint8_t PIN_CURRENT = 34;   // ACS712 analog OUT (must be <= 3.3 V)
constexpr uint8_t PIN_VIBRATION = 35; // Analog vibration module OUT
constexpr uint8_t PIN_RPM = 27;       // Hall / magnetic proximity pulse, active LOW
constexpr uint8_t PIN_BEAM = 25;      // Laser/LDR digital comparator output
constexpr uint8_t PIN_COOLANT = 26;   // Float switch to GND when LOW level
constexpr uint8_t PIN_SDA = 21;
constexpr uint8_t PIN_SCL = 22;

// -------- Calibration --------
// ACS712 nominal sensitivity values: 5A=0.185, 20A=0.100, 30A=0.066 V/A.
// Change this to match your exact module and calibrate with a known load.
constexpr float CURRENT_ZERO_V = 1.65f;   // Calibrate actual zero-current output
constexpr float CURRENT_SENS_V_PER_A = 0.100f; // Example: ACS712 20A version
constexpr uint16_t CURRENT_SAMPLE_COUNT = 400;

// Number of Hall/magnetic pulses per spindle revolution.
constexpr float PULSES_PER_REVOLUTION = 1.0f;

// Set true only after calibrating your vibration sensor against a proper meter.
constexpr bool VIBRATION_CALIBRATED = false;
// Example scale only; do NOT use as a real measurement until calibrated.
constexpr float VIBRATION_ADC_TO_MMS = 0.001f;

// -------- Devices --------
WebServer server(80);
LiquidCrystal_I2C lcd(0x27, 16, 2);
Adafruit_MLX90614 mlx = Adafruit_MLX90614();

// -------- Telemetry --------
volatile uint32_t rpmPulseCount = 0;
uint32_t lastRpmUpdateMs = 0;
uint32_t lastSensorReadMs = 0;
uint32_t lastLcdUpdateMs = 0;
float rpmValue = 0.0f;
float temperatureC = NAN;
float currentA = NAN;
float vibrationMmS = NAN;
uint16_t vibrationRaw = 0;
bool temperatureSensorOk = false;
bool machineRunning = false;
bool beamInterrupted = false;
bool coolantLow = false;
uint32_t partsCount = 0;

void IRAM_ATTR onRpmPulse() {
  rpmPulseCount++;
}

float readCurrentAmps() {
  // Sample the sensor and calculate AC RMS current relative to the zero point.
  // For DC current, this method needs a different calibration/averaging approach.
  double sumSquares = 0.0;
  for (uint16_t i = 0; i < CURRENT_SAMPLE_COUNT; i++) {
    const float volts = (analogRead(PIN_CURRENT) / 4095.0f) * 3.3f;
    const float centered = volts - CURRENT_ZERO_V;
    sumSquares += (double)centered * centered;
    delayMicroseconds(250);
  }
  const float rmsVoltage = sqrt(sumSquares / CURRENT_SAMPLE_COUNT);
  const float amps = rmsVoltage / CURRENT_SENS_V_PER_A;
  if (!isfinite(amps) || amps > 1000.0f) return NAN;
  return amps;
}

void readSensors() {
  // MLX90614 provides non-contact object temperature.
  const float t = mlx.readObjectTempC();
  temperatureSensorOk = isfinite(t) && t > -70.0f && t < 380.0f;
  temperatureC = temperatureSensorOk ? t : NAN;

  currentA = readCurrentAmps();
  vibrationRaw = analogRead(PIN_VIBRATION);
  vibrationMmS = VIBRATION_CALIBRATED
      ? (float)vibrationRaw * VIBRATION_ADC_TO_MMS
      : NAN;

  // Assumes active-LOW digital modules/switches. Reverse logic if your module differs.
  beamInterrupted = (digitalRead(PIN_BEAM) == LOW);
  coolantLow = (digitalRead(PIN_COOLANT) == LOW);
}

void updateRpm() {
  const uint32_t now = millis();
  if (now - lastRpmUpdateMs < 1000) return;

  noInterrupts();
  const uint32_t pulses = rpmPulseCount;
  rpmPulseCount = 0;
  interrupts();

  const float elapsedSeconds = (now - lastRpmUpdateMs) / 1000.0f;
  lastRpmUpdateMs = now;
  if (PULSES_PER_REVOLUTION > 0.0f && elapsedSeconds > 0.0f) {
    rpmValue = (pulses * 60.0f) / (PULSES_PER_REVOLUTION * elapsedSeconds);
  }
  machineRunning = rpmValue > 5.0f;
  if (machineRunning) partsCount += 0; // Cycle counting requires a separate cycle sensor.
}

String jsonNumber(float value, uint8_t decimals = 2) {
  if (!isfinite(value)) return "null";
  return String(value, decimals);
}

void addCorsHeaders() {
  server.sendHeader("Access-Control-Allow-Origin", "*");
  server.sendHeader("Access-Control-Allow-Methods", "GET, OPTIONS");
  server.sendHeader("Access-Control-Allow-Headers", "Content-Type");
  server.sendHeader("Cache-Control", "no-store, max-age=0");
}

void handleData() {
  addCorsHeaders();
  String json;
  json.reserve(400);
  json = "{";
  json += "\"device\":\"RETROX_ESP32\",";
  json += "\"temperatureC\":" + jsonNumber(temperatureC, 1) + ",";
  json += "\"temperatureSensorOk\":" + String(temperatureSensorOk ? "true" : "false") + ",";
  json += "\"currentA\":" + jsonNumber(currentA, 2) + ",";
  json += "\"vibrationMmS\":" + jsonNumber(vibrationMmS, 2) + ",";
  json += "\"vibrationRaw\":" + String(vibrationRaw) + ",";
  json += "\"rpm\":" + String((uint32_t)roundf(rpmValue)) + ",";
  json += "\"running\":" + String(machineRunning ? "true" : "false") + ",";
  json += "\"coolantLow\":" + String(coolantLow ? "true" : "false") + ",";
  json += "\"beamInterrupted\":" + String(beamInterrupted ? "true" : "false") + ",";
  json += "\"parts\":" + String(partsCount) + ",";
  json += "\"cycleSeconds\":null,";
  json += "\"oee\":null,";
  json += "\"uptimeSeconds\":" + String(millis() / 1000UL);
  json += "}";
  server.send(200, "application/json", json);
}

void handleRoot() {
  addCorsHeaders();
  String page = "<!doctype html><html><head><meta name='viewport' content='width=device-width,initial-scale=1'>";
  page += "<title>RETROX ESP32</title><style>body{font:16px Arial;background:#101820;color:#eee;padding:20px}h1{color:#ff8735}code{color:#6cf}</style></head><body>";
  page += "<h1>RETROX ESP32 Worker</h1><p>Telemetry endpoint is running.</p>";
  page += "<p>JSON: <a href='/api/data'><code>/api/data</code></a></p>";
  page += "<p>Connect your RETROX webpage to this device's IP address.</p></body></html>";
  server.send(200, "text/html", page);
}

void handleOptions() {
  addCorsHeaders();
  server.send(204, "text/plain", "");
}

void updateLCD() {
  if (millis() - lastLcdUpdateMs < 1000) return;
  lastLcdUpdateMs = millis();
  lcd.clear();
  lcd.setCursor(0, 0);
  lcd.print("RPM:");
  lcd.print((uint32_t)roundf(rpmValue));
  lcd.print(machineRunning ? " RUN" : " STOP");
  lcd.setCursor(0, 1);
  lcd.print("Temp:");
  if (isfinite(temperatureC)) lcd.print(temperatureC, 1);
  else lcd.print("--.-");
  lcd.print((char)223);
  lcd.print("C");
}

void setupWiFi() {
  WiFi.mode(WIFI_STA);
  if (String(WIFI_SSID) != "YOUR_WIFI_NAME" && strlen(WIFI_SSID) > 0) {
    WiFi.begin(WIFI_SSID, WIFI_PASSWORD);
    Serial.print("Connecting to Wi-Fi");
    const uint32_t started = millis();
    while (WiFi.status() != WL_CONNECTED && millis() - started < 15000) {
      delay(300);
      Serial.print('.');
    }
  }

  if (WiFi.status() == WL_CONNECTED) {
    Serial.println("\nWi-Fi connected.");
    Serial.print("ESP32 IP: http://");
    Serial.println(WiFi.localIP());
    lcd.clear();
    lcd.print("WiFi connected");
    lcd.setCursor(0, 1);
    lcd.print(WiFi.localIP());
  } else {
    WiFi.disconnect(true);
    WiFi.mode(WIFI_AP);
    WiFi.softAP(AP_SSID, AP_PASSWORD);
    Serial.println("\nRouter unavailable; fallback AP started.");
    Serial.print("Wi-Fi SSID: "); Serial.println(AP_SSID);
    Serial.print("Wi-Fi password: "); Serial.println(AP_PASSWORD);
    Serial.println("Dashboard URL: http://192.168.4.1");
    lcd.clear();
    lcd.print("AP: RETROX-ESP32");
    lcd.setCursor(0, 1);
    lcd.print("192.168.4.1");
  }
}

void setup() {
  Serial.begin(115200);
  delay(200);

  pinMode(PIN_RPM, INPUT_PULLUP);
  pinMode(PIN_BEAM, INPUT_PULLUP);
  pinMode(PIN_COOLANT, INPUT_PULLUP);
  analogReadResolution(12);
  analogSetPinAttenuation(PIN_CURRENT, ADC_11db);
  analogSetPinAttenuation(PIN_VIBRATION, ADC_11db);

  Wire.begin(PIN_SDA, PIN_SCL);
  lcd.init();
  lcd.backlight();
  lcd.setCursor(0, 0);
  lcd.print("RETROX Worker");
  lcd.setCursor(0, 1);
  lcd.print("Starting...");

  if (!mlx.begin()) {
    Serial.println("WARNING: MLX90614 not detected. Check SDA/SCL and power.");
  }

  attachInterrupt(digitalPinToInterrupt(PIN_RPM), onRpmPulse, FALLING);
  setupWiFi();

  server.on("/", HTTP_GET, handleRoot);
  server.on("/api/data", HTTP_GET, handleData);
  server.on("/api/data", HTTP_OPTIONS, handleOptions);
  server.onNotFound([]() {
    addCorsHeaders();
    server.send(404, "application/json", "{\"error\":\"not found\"}");
  });
  server.begin();
  Serial.println("HTTP server started.");

  lastRpmUpdateMs = millis();
  lastSensorReadMs = 0;
  lastLcdUpdateMs = 0;
}

void loop() {
  server.handleClient();
  updateRpm();
  if (millis() - lastSensorReadMs >= 500) {
    lastSensorReadMs = millis();
    readSensors();
  }
  updateLCD();
  delay(2);
}
