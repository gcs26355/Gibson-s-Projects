/*
 * ESP32 Multi-Sensor IoT Data Logger
 * --------------------------------------------------------------
 * Sensors:
 *   - LPS33HW        : I2C pressure sensor (raw register driver)
 *   - DS18B20        : OneWire digital temperature sensor
 *   - 10k thermistor : hardware-linearized, read as analog voltage
 *   - Capacitive pad : RC charge-time touch sensor, starts/stops logging
 *
 * Behavior:
 *   - Touching the capacitive pad starts a new ~40 minute logging
 *     session. An averaged sample is taken every ~40 seconds and
 *     stored in RAM (no radio activity during this phase).
 *   - Touching the pad again stops the session early and sends
 *     whatever has been collected so far.
 *   - If a session runs the full 40 minutes (60 samples), the batch
 *     is sent automatically and logging stops until the next touch.
 *   - WiFi/MQTT are only powered up to send a batch, then torn back
 *     down (Connect -> Send -> Disconnect), to keep the radio off
 *     for as much of the cycle as possible.
 *
 * --------------------------------------------------------------
 * FIXES APPLIED (see inline "FIX:" comments for details):
 *   1. The auto-send-at-60-samples behavior described above was never
 *      implemented - takeSample() just silently stopped recording once
 *      the buffer was full, and logging never turned itself off or sent
 *      the batch. This was the main "ending" bug.
 *   2. The touch threshold was only 3 microseconds above the calibrated
 *      baseline, which is far tighter than the run-to-run jitter you get
 *      from an RC charge-time reading. That caused spurious/repeated
 *      start & stop toggles from ordinary noise - the main "start-up"
 *      bug. Threshold is now a percentage-based margin, and a simple
 *      up/down debounce counter was added so a momentary noise blip
 *      can't cause a false start or a false stop.
 * --------------------------------------------------------------
 */

#include <Wire.h>
#include <WiFi.h>
#include <PubSubClient.h>
#include <ArduinoJson.h>
#include <OneWire.h>
#include <DallasTemperature.h>

// ================= User configuration =================
const char* WIFI_SSID     = "Michigan";
const char* WIFI_PASSWORD = "roblouise329";

const char* MQTT_BROKER    = "192.168.86.22";
const uint16_t MQTT_PORT   = 1883;
const char* MQTT_CLIENT_ID = "esp32-sensor-node";
const char* MQTT_TOPIC     = "sensors/node1/data";

// ================= Timing configuration =================
const unsigned long SAMPLE_INTERVAL_MS = 20000UL;              // one sample every 20 s

// Change from a 40-minute total duration to a 10-minute batch size
const unsigned long BATCH_DURATION_MS = 10UL * 60UL * 1000UL;  
const size_t        MAX_SAMPLES       = BATCH_DURATION_MS / SAMPLE_INTERVAL_MS; // 30 samples

const unsigned long TOUCH_POLL_INTERVAL_MS = 50;  // how often to check the touch pad

const unsigned long THERMISTOR_AVG_WINDOW_MS     = 500; // keep thermistor powered ~0.5 s
const unsigned long THERMISTOR_SAMPLE_SPACING_MS = 20;  // ~25 sub-samples per reading

const int N_DS18B20_SAMPLES = 3;
const int N_LPS33HW_SAMPLES = 3;
const unsigned long LPS33HW_SAMPLE_SPACING_MS = 45; // > 1/25Hz ODR period, for independent samples

const size_t MQTT_BUFFER_SIZE = 12288; // big enough for a 60-sample JSON batch

// ================= Pin definitions =================
#define ONEWIRE_BUS_PIN    4
#define THERMISTOR_PIN     34   // ADC1 channel
#define THERMISTOR_PWR_PIN 33   // GPIO to power the thermistor circuit

#define TOUCH_SEND_PIN  27      // drives the RC charge
#define TOUCH_SENSE_PIN 13      // reads through resistor, connected to foil

// ================= Thermistor calibration =================
const float THERMISTOR_SLOPE_C_PER_MV = 0.00318766755178f;
const float THERMISTOR_OFFSET_C       = 19.8523791157f;

// ================= LPS33HW raw I2C driver =================
#define LPS33HW_ADDR   0x5D   // default addr (0x5C if SA0 pulled low)
#define WHO_AM_I_REG   0x0F
#define CTRL_REG1      0x10
#define PRESS_OUT_XL   (0x28 | 0x80)  // OR 0x80 enables auto-increment
#define TEMP_OUT_L     (0x2B | 0x80)

bool initLPS33HW() {
  Wire.beginTransmission(LPS33HW_ADDR);
  Wire.write(WHO_AM_I_REG);
  Wire.endTransmission(false);
  Wire.requestFrom(LPS33HW_ADDR, 1);
  uint8_t whoAmI = Wire.read();

  if (whoAmI != 0xB1) {
    Serial.print("LPS33HW not found, WHO_AM_I = 0x");
    Serial.println(whoAmI, HEX);
    return false;
  }

  // ODR = 25 Hz, block data update enabled
  Wire.beginTransmission(LPS33HW_ADDR);
  Wire.write(CTRL_REG1);
  Wire.write(0x30);
  Wire.endTransmission();
  delay(10);
  return true;
}

// Returns true on success, fills pressure_hPa and temp_C by reference
bool readPressureTemp(float &pressure_hPa, float &temp_C) {
  // --- Pressure (3 bytes, auto-increment) ---
  Wire.beginTransmission(LPS33HW_ADDR);
  Wire.write(PRESS_OUT_XL);
  if (Wire.endTransmission(false) != 0) return false;
  Wire.requestFrom(LPS33HW_ADDR, 3);
  if (Wire.available() < 3) return false;

  uint8_t xl = Wire.read();
  uint8_t l  = Wire.read();
  uint8_t h  = Wire.read();

  int32_t pressureRaw = ((int32_t)h << 16) | ((int32_t)l << 8) | xl;
  if (pressureRaw & 0x800000) pressureRaw -= (1 << 24);  // sign extend 24-bit
  pressure_hPa = pressureRaw / 4096.0f;

  // --- Temperature (2 bytes, auto-increment) ---
  Wire.beginTransmission(LPS33HW_ADDR);
  Wire.write(TEMP_OUT_L);
  if (Wire.endTransmission(false) != 0) return false;
  Wire.requestFrom(LPS33HW_ADDR, 2);
  if (Wire.available() < 2) return false;

  uint8_t tl = Wire.read();
  uint8_t th = Wire.read();

  int16_t tempRaw = ((int16_t)th << 8) | tl;
  temp_C = tempRaw / 100.0f;

  return true;
}

// Average both pressure and temperature from the LPS33HW
void readAveragedLPS33HW(float &avgPressure, float &avgTemp) {
  float sumP = 0, sumT = 0;
  int validCount = 0;
  for (int i = 0; i < N_LPS33HW_SAMPLES; i++) {
    float p, t;
    if (readPressureTemp(p, t)) {
      sumP += p;
      sumT += t;
      validCount++;
    }
    if (i < N_LPS33HW_SAMPLES - 1) delay(LPS33HW_SAMPLE_SPACING_MS);
  }

  if (validCount > 0) {
    avgPressure = sumP / validCount;
    avgTemp = sumT / validCount;
  } else {
    avgPressure = NAN;
    avgTemp = NAN;
  }
}

// ================= DS18B20 =================
OneWire oneWire(ONEWIRE_BUS_PIN);
DallasTemperature ds18b20(&oneWire);

float readAveragedDS18B20() {
  float sum = 0;
  int validCount = 0;
  for (int i = 0; i < N_DS18B20_SAMPLES; i++) {
    ds18b20.requestTemperatures();   // blocking, ~750 ms at default 12-bit resolution
    float t = ds18b20.getTempCByIndex(0);
    if (t != DEVICE_DISCONNECTED_C) {
      sum += t;
      validCount++;
    }
  }
  return validCount > 0 ? sum / validCount : NAN;
}

// ================= Thermistor =================
float milliVoltsToTempC(float milliVolts) {
  return THERMISTOR_SLOPE_C_PER_MV * milliVolts + THERMISTOR_OFFSET_C;
}

// Power the thermistor divider for ~THERMISTOR_AVG_WINDOW_MS and average
// several ADC readings taken during that window, then power it back down.
float readAveragedThermistor(uint32_t &avgMilliVolts) {
  digitalWrite(THERMISTOR_PWR_PIN, HIGH);
  delay(10); // let the divider settle before the first reading

  uint32_t sumMv = 0;
  uint32_t count = 0;
  unsigned long start = millis();
  while (millis() - start < THERMISTOR_AVG_WINDOW_MS) {
    sumMv += analogReadMilliVolts(THERMISTOR_PIN);
    count++;
    delay(THERMISTOR_SAMPLE_SPACING_MS);
  }

  digitalWrite(THERMISTOR_PWR_PIN, LOW);

  avgMilliVolts = (count > 0) ? (sumMv / count) : 0;
  return milliVoltsToTempC((float)avgMilliVolts);
}

// ================= Capacitive touch (logging start/stop switch) =================
int  baseline = 0;
int  threshold = 0;
int  elapsed_1 = 0;
int  elapsed_2 = 0;
int  elapsed_3 = 0;
int  average_elapsed = 0;
bool pressed_last = false;
bool loggingEnabled = false;

// FIX: Debounce state for the touch switch. A single noisy reading above
// threshold can no longer register as a touch by itself - it has to stay
// above threshold for TOUCH_DEBOUNCE_POLLS consecutive polls in a row
// (tracked via an up/down counter), and has to fully drop back below
// threshold before another touch can be registered. This is what prevents
// one physical touch from being misread as touch-release-touch and
// toggling the logging state two or three times in a row.
const int TOUCH_DEBOUNCE_POLLS = 1; // ~200 ms of sustained signal at 50 ms/poll
int touchDebounceCount = 0;

int charge_discharge() {
  // Discharge
  pinMode(TOUCH_SENSE_PIN, OUTPUT);
  digitalWrite(TOUCH_SENSE_PIN, LOW);
  delayMicroseconds(10);
  pinMode(TOUCH_SENSE_PIN, INPUT);

  // Charge and time
  digitalWrite(TOUCH_SEND_PIN, HIGH);
  unsigned long start = micros();
  while (digitalRead(TOUCH_SENSE_PIN) == LOW) {
    if (micros() - start > 5000) break; // timeout so we never hang
  }
  unsigned long elapsed = micros() - start;
  digitalWrite(TOUCH_SEND_PIN, LOW);

  return elapsed;
}

void toggleLogging(); // defined near the sampling logic below

void pollTouchSensor() {
  elapsed_1 = charge_discharge();
  average_elapsed = (elapsed_1 + elapsed_2 + elapsed_3) / 3;
  // Serial.println(average_elapsed); // uncomment to tune threshold/margin below

  // FIX: up/down debounce counter instead of a single-sample decision.
  // Above threshold -> counter climbs (capped); below -> counter falls.
  // Only a fully-saturated counter counts as "touched", and it must fully
  // drain back to 0 before a new touch can be registered. This absorbs
  // the kind of +/-few-microsecond jitter that a single-sample threshold
  // (like the old "baseline + 3") would misread as press/release cycles.
  if (average_elapsed > threshold) {
    if (touchDebounceCount < TOUCH_DEBOUNCE_POLLS) touchDebounceCount++;
  } else {
    if (touchDebounceCount > 0) touchDebounceCount--;
  }

  bool touched = (touchDebounceCount >= TOUCH_DEBOUNCE_POLLS);

  if (touched && !pressed_last) {
    toggleLogging();
    pressed_last = true;
  } else if (touchDebounceCount == 0) {
    pressed_last = false;
  }

  elapsed_3 = elapsed_2;
  elapsed_2 = elapsed_1;
}

// ================= Sample buffer =================
struct SensorSample {
  uint32_t t_ms;
  float    pressure_hPa;
  float    lps33hw_tempC;
  float    ds18b20_tempC;
  float    thermistor_tempC;
  uint32_t thermistor_mV;
};

SensorSample sampleBuffer[MAX_SAMPLES];
size_t sampleCount = 0;
unsigned long loggingStartMillis = 0;
unsigned long lastSampleMillis = 0;

// ================= Connectivity =================
WiFiClient wifiClient;
PubSubClient mqttClient(wifiClient);

bool connectWiFi() {
  const unsigned long WIFI_CONNECT_TIMEOUT_MS = 15000;
  const int MAX_WIFI_ATTEMPTS = 3;

  for (int attempt = 1; attempt <= MAX_WIFI_ATTEMPTS; attempt++) {
    WiFi.mode(WIFI_STA);
    delay(300);
    WiFi.begin(WIFI_SSID, WIFI_PASSWORD);
    Serial.printf("Connecting to WiFi (attempt %d/%d)", attempt, MAX_WIFI_ATTEMPTS);

    unsigned long start = millis();
    while (WiFi.status() != WL_CONNECTED && millis() - start < WIFI_CONNECT_TIMEOUT_MS) {
      delay(500);
      Serial.print(".");
    }

    if (WiFi.status() == WL_CONNECTED) {
      Serial.println("\nWiFi connected.");
      return true;
    }

    // NEW: print the actual status code so we know *why* it failed
    Serial.printf("\nWiFi connect attempt timed out. status=%d (", WiFi.status());
    switch (WiFi.status()) {
      case WL_NO_SSID_AVAIL:  Serial.print("NO_SSID_AVAIL - AP not found/out of range"); break;
      case WL_CONNECT_FAILED: Serial.print("CONNECT_FAILED - wrong password or auth rejected"); break;
      case WL_CONNECTION_LOST:Serial.print("CONNECTION_LOST"); break;
      case WL_DISCONNECTED:   Serial.print("DISCONNECTED - still negotiating/no reply"); break;
      case WL_IDLE_STATUS:    Serial.print("IDLE_STATUS - stuck before scan even completed"); break;
      default:                Serial.print("other"); break;
    }
    Serial.println(")");

    WiFi.disconnect(true);
    delay(500);
  }

  Serial.println("WiFi failed after all attempts.");
  return false;
}

bool connectMQTT() {
  const int MAX_MQTT_ATTEMPTS = 5;
  mqttClient.setBufferSize(MQTT_BUFFER_SIZE);
  mqttClient.setServer(MQTT_BROKER, MQTT_PORT);

  for (int attempt = 1; attempt <= MAX_MQTT_ATTEMPTS; attempt++) {
    Serial.printf("Connecting to MQTT broker (attempt %d/%d)...", attempt, MAX_MQTT_ATTEMPTS);
    if (mqttClient.connect(MQTT_CLIENT_ID)) {
      Serial.println("connected");
      return true;
    }
    Serial.printf("failed, rc=%d, retrying in 2s\n", mqttClient.state());
    delay(2000);
  }
  Serial.println("MQTT failed after all attempts.");
  return false;
}

void disconnectNetwork() {
  mqttClient.disconnect();
  delay(50); // allow the MQTT disconnect packet to send
  WiFi.disconnect(true);
  WiFi.mode(WIFI_OFF);
  Serial.println("Network disconnected. Radio OFF.");
}

void sendBatch() {
  if (sampleCount == 0) {
    Serial.println("Nothing to send.");
    return;
  }

  if (!connectWiFi()) {
    Serial.println("Skipping this send - will try again next cycle.");
    disconnectNetwork(); // make sure radio is off, don't clear sampleCount
    return;
  }

  if (!connectMQTT()) {
  Serial.println("Skipping this send - will try again next cycle.");
  disconnectNetwork();
  return;   // sampleCount NOT cleared - data preserved for retry
  }

  DynamicJsonDocument doc(MQTT_BUFFER_SIZE);
  doc["node_id"]          = MQTT_CLIENT_ID;
  doc["session_start_ms"] = loggingStartMillis;
  doc["sample_count"]     = sampleCount;

  JsonArray samples = doc.createNestedArray("samples");
  for (size_t i = 0; i < sampleCount; i++) {
    JsonObject o = samples.createNestedObject();
    o["t_ms"]  = sampleBuffer[i].t_ms;
    o["p_hPa"] = sampleBuffer[i].pressure_hPa;
    o["lps_C"] = sampleBuffer[i].lps33hw_tempC;
    o["ds_C"]  = sampleBuffer[i].ds18b20_tempC;
    o["th_C"]  = sampleBuffer[i].thermistor_tempC;
    o["th_mV"] = sampleBuffer[i].thermistor_mV;
  }

  static char payload[MQTT_BUFFER_SIZE];
  size_t len = serializeJson(doc, payload, sizeof(payload));

  if (mqttClient.publish(MQTT_TOPIC, payload, len)) {
    Serial.printf("Published batch of %u samples (%u bytes)\n", (unsigned)sampleCount, (unsigned)len);
  } else {
    Serial.println("MQTT publish failed (check broker connection & payload size).");
  }

  mqttClient.loop();
  delay(100); // give the stack a moment to flush the outbound packet

  disconnectNetwork();

  sampleCount = 0; // clear the buffer once it's been sent
}

// ================= Sampling / logging session logic =================
void takeSample() {
  if (sampleCount >= MAX_SAMPLES) return; // safety net; shouldn't trigger, see fix below

  SensorSample s;
  s.t_ms = millis() - loggingStartMillis;

  // Get both values from the LPS33HW
  float p, t;
  readAveragedLPS33HW(p, t);
  s.pressure_hPa = p;
  s.lps33hw_tempC = t;

  s.ds18b20_tempC = readAveragedDS18B20();

  uint32_t mv;
  s.thermistor_tempC = readAveragedThermistor(mv);
  s.thermistor_mV = mv;

  sampleBuffer[sampleCount++] = s;

  Serial.printf("Sample %u/%u  P=%.4f hPa  LPS_T=%.4f C  DS18B20=%.4f C  Therm=%.4f C  Therm=%u mV Clock=%u ms\n",
                (unsigned)sampleCount, (unsigned)MAX_SAMPLES,
                s.pressure_hPa, s.lps33hw_tempC, s.ds18b20_tempC, s.thermistor_tempC, s.thermistor_mV, s.t_ms);

  // FIX: this is the missing piece that implemented the documented
  // "session runs the full 40 minutes -> auto-send and stop" behavior.
  // Previously nothing checked for the buffer filling up, so once
  // sampleCount hit MAX_SAMPLES the function above (the early "return;")
  // just silently no-op'd forever - logging never stopped and the batch
  // was never sent on its own; it required a manual touch to notice.
  if (sampleCount >= MAX_SAMPLES) {
    Serial.println("10-minute mark reached - sending batch and continuing.");
    sendBatch(); // This sends the data and resets sampleCount back to 0
    
    // Notice that `loggingEnabled = false;` has been removed. 
    // This allows the ESP32 to seamlessly start filling the next 10-minute batch!
  }
}

void toggleLogging() {
  if (!loggingEnabled) {
    sampleCount = 0;
    loggingStartMillis = millis();
    lastSampleMillis = loggingStartMillis - SAMPLE_INTERVAL_MS; // sample right away
    loggingEnabled = true;
    Serial.println("Logging STARTED.");
  } else {
    Serial.println("Logging STOPPED (touch) - sending collected data.");
    loggingEnabled = false;
    sendBatch();
  }
}

// ================= Setup / Loop =================
unsigned long lastTouchPollMillis = 0;

void setup() {
  Serial.begin(115200);

  // 1. Wait 3 seconds so the Serial Monitor has time to catch the first messages
  delay(3000);
  Serial.println("\n\n--- ESP32 SYSTEM STARTUP ---");

  Serial.println("[1] Setting up pins...");
  pinMode(THERMISTOR_PWR_PIN, OUTPUT);
  digitalWrite(THERMISTOR_PWR_PIN, LOW);
  pinMode(TOUCH_SEND_PIN, OUTPUT);
  digitalWrite(TOUCH_SEND_PIN, LOW);

  Serial.println("[2] Initializing I2C bus...");
  Wire.begin();
  // Set a timeout so the ESP32 doesn't freeze forever if a sensor is stuck
  Wire.setTimeOut(150);

  Serial.println("[3] Connecting to LPS33HW...");
  while (!initLPS33HW()) {
    Serial.println("Retrying LPS33HW init... (Is it plugged in?)");
    delay(1000);
  }

  Serial.println("[4] Initializing DS18B20...");
  ds18b20.begin();

  Serial.println("[5] Configuring ADC...");
  analogReadResolution(12);
  analogSetPinAttenuation(THERMISTOR_PIN, ADC_11db);

  Serial.println("[6] Calibrating touch pad. Keep hands away...");
  delay(3000); // Give the power rails and your hands time to clear

  long baseline_sum = 0;
  for (int i = 0; i < 10; i++) {
    baseline_sum += charge_discharge();
    delay(10);
  }

  baseline = baseline_sum / 10;

  // FIX: the old threshold ("baseline + 3" microseconds) was only a few
  // microseconds of margin, which is well within the normal run-to-run
  // jitter of an RC charge-time reading. That made the pad register
  // phantom touches from ordinary electrical noise, which is what was
  // causing logging to start/stop on its own. A touch adds real body
  // capacitance, so it should shift the reading by a lot more than
  // measurement noise does - a percentage-based margin is more robust
  // than a fixed handful of microseconds, and scales automatically with
  // your specific pad/resistor/RC circuit.
  //
  // Tune this on your hardware: uncomment the Serial.println(average_elapsed)
  // line in pollTouchSensor(), watch the values with the pad untouched vs.
  // touched, and set TOUCH_MARGIN_PERCENT comfortably between the two
  // (roughly 1/3 to 1/2 of the way from baseline to a touched reading).
  const int TOUCH_MARGIN_PERCENT = 20;   // touch trigger = baseline + 20%
  const int TOUCH_MIN_MARGIN     = 1;   // floor, in case baseline is very small
  int margin = (baseline * TOUCH_MARGIN_PERCENT) / 100;
  if (margin < TOUCH_MIN_MARGIN) margin = TOUCH_MIN_MARGIN;
  threshold = baseline + margin;

  elapsed_2 = elapsed_3 = baseline;

  Serial.print("Touch baseline: ");
  Serial.print(baseline);
  Serial.print(" | Threshold: ");
  Serial.println(threshold);

  WiFi.mode(WIFI_OFF);
  Serial.println("--- SETUP COMPLETE. Radio OFF. Touch pad to start. ---");
  // delay(1000);
  // loggingEnabled = true;
}

void loop() {
  unsigned long now = millis();

  // Poll the touch pad frequently so start/stop stays responsive
  if (now - lastTouchPollMillis >= TOUCH_POLL_INTERVAL_MS) {
    lastTouchPollMillis = now;
    pollTouchSensor();
  }

  // While a session is running, take one averaged sample every SAMPLE_INTERVAL_MS
  if (loggingEnabled && (now - lastSampleMillis >= SAMPLE_INTERVAL_MS)) {
    lastSampleMillis = now;
    takeSample();
  }
  delay(50);
}
